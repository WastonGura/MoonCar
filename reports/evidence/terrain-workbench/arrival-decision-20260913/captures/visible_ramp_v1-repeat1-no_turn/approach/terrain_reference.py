"""Offline SDF visual-surface diagnostics. Never an input to the rover planner."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np

from simulation_experiment import transform
from survey_evaluation import write_csv, write_json


def read_shapes(path):
    world = ET.parse(path).getroot().find('world')
    if world is None or world.find('include') is not None or world.find('frame') is not None:
        raise ValueError('Expected explicit world geometry without includes or named frames')
    shapes = []
    for model in world.findall('model'):
        if model.findtext('static') != 'true' or model.find('model') is not None:
            raise ValueError('Only flat, static model hierarchy is supported')
        for link in model.findall('link'):
            for visual in link.findall('visual'):
                geometry = visual.find('geometry')
                if geometry is None or len(geometry) != 1:
                    raise ValueError('Expected exactly one visual geometry')
                shape = geometry[0]; kind = shape.tag
                if kind in ('box', 'ellipsoid', 'plane'):
                    field = {'box': 'size', 'ellipsoid': 'radii', 'plane': 'size'}[kind]
                    size = np.array(list(map(float, shape.findtext(field).split())))
                    if size.shape != ((2,) if kind == 'plane' else (3,)):
                        raise ValueError('Invalid geometry dimensions')
                    if kind == 'box': size /= 2
                    if kind == 'plane' and shape.findtext('normal', '0 0 1').split() != ['0', '0', '1']:
                        raise ValueError('Only +Z local plane normal is supported')
                elif kind == 'cylinder':
                    size = np.array([float(shape.findtext('radius')), float(shape.findtext('length')) / 2])
                else:
                    raise ValueError(f'Unsupported visual geometry: {kind}')
                matrix = transform(model) @ transform(link) @ transform(visual)
                if not np.isfinite(matrix).all() or not np.isfinite(size).all() or np.any(size <= 0):
                    raise ValueError('Non-finite pose or non-positive dimensions')
                # ponytail: cylinders in this scene are upright; reject tilted ones until needed.
                if kind == 'cylinder' and not np.allclose(matrix[:3, 2], [0, 0, 1]):
                    raise ValueError('Tilted cylinders are not supported')
                shapes.append((f"{model.get('name')}/{link.get('name')}/{visual.get('name')}", kind, size, matrix))
    if not shapes: raise ValueError('No visual surfaces')
    return shapes


def surface(shapes, xy):
    """Highest visual intersection of each infinite vertical line, in world metres."""
    xy = np.asarray(xy, dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or not np.isfinite(xy).all():
        raise ValueError('Expected finite Nx2 coordinates')
    z = np.full(len(xy), -np.inf); owner = np.full(len(xy), -1)
    world_zero = np.column_stack((xy, np.zeros(len(xy))))
    for index, (_, kind, size, matrix) in enumerate(shapes):
        local = (world_zero - matrix[:3, 3]) @ matrix[:3, :3]
        direction = matrix[2, :3]
        hit = np.full(len(xy), -np.inf)
        if kind == 'ellipsoid':
            p, d = local / size, direction / size
            a = d @ d; b = 2 * (p @ d); c = np.sum(p*p, axis=1) - 1
            disc = b*b - 4*a*c
            ok = disc >= 0
            hit[ok] = (-b[ok] + np.sqrt(disc[ok])) / (2*a)
        elif kind == 'box':
            low = np.full(len(xy), -np.inf); high = np.full(len(xy), np.inf)
            ok = np.ones(len(xy), dtype=bool)
            for axis in range(3):
                if abs(direction[axis]) < 1e-12:
                    ok &= np.abs(local[:, axis]) <= size[axis]
                else:
                    a = (-size[axis] - local[:, axis]) / direction[axis]
                    b = (size[axis] - local[:, axis]) / direction[axis]
                    low = np.maximum(low, np.minimum(a, b)); high = np.minimum(high, np.maximum(a, b))
            ok &= high >= low
            hit[ok] = high[ok]
        elif kind == 'plane':
            if abs(direction[2]) < 1e-12: raise ValueError('Vertical planes cannot define elevation')
            height = -local[:, 2] / direction[2]
            point = local + height[:, None] * direction
            ok = np.all(np.abs(point[:, :2]) <= size / 2, axis=1)
            hit[ok] = height[ok]
        else:
            ok = np.sum(local[:, :2]**2, axis=1) <= size[0]**2
            hit[ok] = size[1] - local[ok, 2]
        update = hit > z
        z[update], owner[update] = hit[update], index
    return z, owner


def reference_grid(shapes, origin, radius, resolution):
    """Uniform 5x5 cell samples; fit same z-at-centre / slope representation as elevation."""
    if (not math.isfinite(resolution) or not .05 <= resolution <= 2 or not math.isfinite(radius)
            or not 0 < radius <= 50 or len(origin) != 2 or not np.isfinite(origin).all()):
        raise ValueError('Invalid evaluation scope')
    keys = np.array([(x, y)
                     for x in range(math.floor((origin[0]-radius)/resolution), math.ceil((origin[0]+radius)/resolution))
                     for y in range(math.floor((origin[1]-radius)/resolution), math.ceil((origin[1]+radius)/resolution))
                     if abs((x+.5)*resolution) <= 25 and abs((y+.5)*resolution) <= 25
                     and math.hypot((x+.5)*resolution-origin[0], (y+.5)*resolution-origin[1]) <= radius])
    if not len(keys): raise ValueError('Empty evaluation ROI')
    offsets = np.array([(x, y) for x in np.linspace(-.4, .4, 5) for y in np.linspace(-.4, .4, 5)]) * resolution
    centres = (keys+.5)*resolution
    heights, owners = surface(shapes, (centres[:, None, :] + offsets).reshape(-1, 2))
    heights, owners = heights.reshape(-1, 25), owners.reshape(-1, 25)
    design = np.column_stack((offsets, np.ones(25)))
    result = []
    for key, center, samples, ids in zip(keys, centres, heights, owners):
        cell = dict(ix=int(key[0]), iy=int(key[1]), x=float(center[0]), y=float(center[1]))
        if np.any(ids < 0):
            cell.update(category='no_reference', z=None, slope=None, roughness=None)
        else:
            plane = np.linalg.lstsq(design, samples, rcond=None)[0]
            roughness = float(np.sqrt(np.mean((design @ plane - samples)**2)))
            slope = math.degrees(math.atan(np.linalg.norm(plane[:2])))
            edge = len(set(ids)) > 1 or roughness > .025 or slope >= 60
            flat = all(shapes[i][1] == 'plane' for i in ids)
            cell.update(category='edge_or_complex' if edge else ('ground' if flat else 'terrain'),
                        z=float(plane[2]), slope=slope, roughness=roughness,
                        visual_surfaces=';'.join(sorted({shapes[i][0] for i in ids})))
        result.append(cell)
    return result


def statistics(rows):
    matched = [c for c in rows if c['height_error_m'] is not None]
    heights = np.array([c['height_error_m'] for c in matched])
    slopes = np.array([c['slope_error_deg'] for c in matched])
    return dict(reference_cells=len(rows), observed_cells=sum(c['observed'] for c in rows),
                valid_observed_cells=sum(c['valid_observed'] for c in rows), matched_cells=len(matched),
                missing_cells=sum(not c['observed'] for c in rows),
                invalid_observed_cells=sum(c['observed'] and not c['valid_observed'] for c in rows),
                height_rmse_m=float(np.sqrt(np.mean(heights**2))) if len(heights) else None,
                height_bias_m=float(np.mean(heights)) if len(heights) else None,
                height_p95_abs_m=float(np.percentile(np.abs(heights),95)) if len(heights) else None,
                slope_mae_deg=float(np.mean(np.abs(slopes))) if len(slopes) else None)


def analyse(folder, output):
    import matplotlib  # Fail before writing output if the plotting environment is unavailable.
    folder, output = Path(folder).resolve(), Path(output).resolve()
    if output == folder or folder in output.parents:
        raise ValueError('Output must be separate from frozen evidence')
    if output.exists(): raise ValueError('Output already exists; preserve previous analysis')
    inputs = {name: hashlib.sha256((folder/name).read_bytes()).hexdigest()
              for name in ('summary.json','quality-end.json','elevation.csv','survey_yard.sdf','observations.npz')}
    frozen = json.loads((folder/'hashes.json').read_text(encoding='utf-8'))
    if any(frozen.get(name) != digest for name, digest in inputs.items()):
        raise ValueError('Input does not match frozen evidence hashes')
    summary = json.loads((folder/'summary.json').read_text(encoding='utf-8'))
    quality = json.loads((folder/'quality-end.json').read_text(encoding='utf-8'))
    resolution = quality['resolution']
    reference = reference_grid(read_shapes(folder/'survey_yard.sdf'), summary['origin'], summary['radius_m'], resolution)
    if not math.isclose(len(reference)*resolution**2, summary['roi_area_m2']):
        raise ValueError('Reference ROI differs from original evaluation')
    with (folder/'elevation.csv').open(encoding='utf-8-sig', newline='') as handle:
        observed = {(int(c['ix']),int(c['iy'])): c for c in csv.DictReader(handle)}
    rows = []
    for ref in reference:
        c = observed.get((ref['ix'],ref['iy']))
        valid = bool(c and c['valid'] == 'True')
        row = dict(**ref, observed=bool(c), valid_observed=valid,
                   quality=float(c['quality']) if c else None,
                   observed_z=float(c['z']) if valid else None,
                   observed_slope=float(c['slope']) if valid else None,
                   height_error_m=None, slope_error_deg=None)
        if valid and ref['z'] is not None:
            row.update(height_error_m=row['observed_z']-ref['z'], slope_error_deg=row['observed_slope']-ref['slope'])
        rows.append(row)
    groups = [{"category": name, **statistics([c for c in rows if name == 'all' or c['category'] == name])}
              for name in ('all','ground','terrain','edge_or_complex','no_reference')]
    bins = []
    for category in ('all','ground','terrain','edge_or_complex'):
        for low, high in ((0,.2),(.2,.4),(.4,.6),(.6,.8),(.8,1.00001)):
            subset = [c for c in rows if c['quality'] is not None and low <= c['quality'] < high
                      and (category == 'all' or c['category'] == category)]
            bins.append(dict(category=category, q_min=low, q_max=min(1,high), **statistics(subset)))
    output.mkdir(parents=True)
    write_json(output/'reference.json', dict(version='visual-reference-v1', frame='map', resolution=resolution,
               source='Frozen survey_yard.sdf visual geometry; analytic vertical intersections',
               alignment_description='Nominal identity world-to-map from configured initial pose; no fitted correction; trajectory alignment unverified',
               cells=[{k:c[k] for k in ('ix','iy','z','slope')} for c in reference if c['z'] is not None]))
    write_csv(output/'cells.csv', rows); write_csv(output/'terrain-groups.csv', groups); write_csv(output/'quality-bins.csv', bins)
    limitations = [
        'Nominal identity map/world alignment only; errors include localization and representation effects, not calibrated camera accuracy.',
        'Archived missions lack synchronized ground-truth trajectories; a later parked alignment check cannot certify them.',
        'Reference is highest VISUAL surface, not collision geometry; no material, wheel slip or traversability truth.',
        'Highest reference surface may differ from the visible side sampled by the camera. Uniform 5x5 fit differs from uneven observed voxel sampling; edge/complex cells are separately reported, never silently removed.',
        'Five samples per cell axis can miss features narrower than 5 cm; primitive analytic shapes differ slightly from rendered meshes.',
        'Missing projected ROI cells include occluded/unreachable surfaces; missing is not automatically an algorithm failure.',
        'Q-bin association is descriptive; correlated cells and different terrain/poses do not establish causality or algorithm superiority.'
    ]
    plot(rows, bins, output, summary['run_id'], resolution)
    write_json(output/'summary.json', dict(version='visual-reference-v1', status='diagnostic_complete',run_id=summary['run_id'],
               alignment_status='nominal_unverified', accepted_accuracy=False, resolution=resolution,
               roi_area_m2=len(rows)*resolution**2, raw_evaluated_frames=summary['raw_evaluated_frames'],
               source_folder=str(folder), input_sha256=inputs, groups=groups, limitations=limitations))
    (output/'README.md').write_text('''# 地形参考诊断

本目录使用保存的仿真场景视觉几何，与实验结束时的已采集地表进行比较。世界坐标与地图坐标暂按配置的恒等对应计算；没有用采集结果反向拟合参考，也没有证明整段轨迹已准确对齐。

- `surface-comparison.png` / `.pdf`：参考、采集、高度和坡度误差；白色表示该图没有可显示值，颜色更深表示数值更大。
- `coverage-status.png` / `.pdf`：固定区域内未观测、观测但无有效地表拟合、有效拟合的分布。
- `quality-error.png` / `.pdf`：按地形类别分开的 Q 与误差、样本数量。Q 不是准确率，格子也不是独立重复实验。
- `cells.csv`：每格参考、观测、误差和缺失状态；`terrain-groups.csv`、`quality-bins.csv` 为分组统计。误差仅对有效匹配格计算，空值不计作零。
- `summary.json`：来源哈希、评价范围、统计与限制；`hashes.json` 用于完整性核查。

参考用每格 5×5 个均匀位置的最高视觉表面拟合。不同表面交界、拟合残差超过 2.5 厘米或坡度至少 60° 的格子单列为 edge_or_complex；这些是分析口径，不是通行阈值。最高表面不一定是相机看到的侧面，参考与稀疏观测的表示差异也会形成误差。

目前数据属于仿真诊断，不是传感器精度验收，也不能证明探测策略优于其他算法。不要仅用被平地占据的总体误差概括复杂地形效果。
''',encoding='utf-8')
    for name in ('terrain_reference.py', 'simulation_experiment.py', 'survey_evaluation.py'):
        shutil.copyfile(Path(__file__).with_name(name), output/name)
    write_json(output/'hashes.json', {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.iterdir()) if p.is_file()})
    return groups


def plot(rows, bins, output, run_id, resolution):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PatchCollection
    from matplotlib.patches import Rectangle, Patch
    from matplotlib.colors import LinearSegmentedColormap
    gray=LinearSegmentedColormap.from_list('visible_gray',['#d8d8d8','#101010'])
    fig, axes = plt.subplots(2, 3, figsize=(12,8), layout='constrained')
    keys=('z','observed_z','height_error_m','slope','observed_slope','slope_error_deg')
    labels=('Reference fitted height (m)','Observed valid height (m)','Absolute height error (m)',
            'Reference fitted slope (deg)','Observed valid slope (deg)','Absolute slope error (deg)')
    height_limits=(min(0,min(c['observed_z'] or 0 for c in rows)),max(.01,max(max(c['z'] or 0,c['observed_z'] or 0) for c in rows)))
    for ax, key, label in zip(axes.flat,keys,labels):
        valid = [c for c in rows if c[key] is not None]
        values = np.array([abs(c[key]) if 'error' in key else c[key] for c in valid])
        collection = PatchCollection([Rectangle((c['ix']*resolution,c['iy']*resolution),resolution,resolution) for c in valid], cmap=gray)
        collection.set_array(values)
        collection.set_clim(*(height_limits if key in ('z','observed_z') else ((0,90) if key in ('slope','observed_slope') else (0,max(.01,float(values.max()) if len(values) else 0)))))
        ax.add_collection(collection); ax.autoscale_view(); ax.set_aspect('equal')
        ax.set(xlabel='Nominal map/world X (m)', ylabel='Y (m)', title=label)
        ax.set_xlim(min(c['x'] for c in rows)-resolution,max(c['x'] for c in rows)+resolution)
        ax.set_ylim(min(c['y'] for c in rows)-resolution,max(c['y'] for c in rows)+resolution)
        fig.colorbar(collection,ax=ax,shrink=.7)
    fig.suptitle(f'{run_id}: nominal alignment diagnostic (not accuracy acceptance)',fontsize=11)
    fig.savefig(output/'surface-comparison.png',dpi=180); fig.savefig(output/'surface-comparison.pdf'); plt.close(fig)
    fig,ax=plt.subplots(figsize=(5,5),layout='constrained')
    colors=['#efefef','#858585','#202020']
    for status,label in enumerate(('Unobserved','Observed, no valid fit','Valid observed fit')):
        selected=[c for c in rows if (2 if c['valid_observed'] else (1 if c['observed'] else 0))==status]
        ax.add_collection(PatchCollection([Rectangle((c['ix']*resolution,c['iy']*resolution),resolution,resolution) for c in selected],facecolor=colors[status]))
    ax.autoscale_view(); ax.set_aspect('equal'); ax.set(xlabel='Nominal map/world X (m)',ylabel='Y (m)',title='Fixed ROI: missing and invalid cells retained')
    ax.legend(handles=[Patch(facecolor=color,label=label) for color,label in zip(colors,('Unobserved','Observed, no valid fit','Valid observed fit'))],loc='upper left',fontsize=8)
    fig.savefig(output/'coverage-status.png',dpi=180); fig.savefig(output/'coverage-status.pdf'); plt.close(fig)
    fig, axes = plt.subplots(1,2,figsize=(9,3.8),layout='constrained')
    for category, style in (('ground','o-'),('terrain','s--'),('edge_or_complex','^:')):
        group=[b for b in bins if b['category']==category and b['matched_cells']]
        x=[(b['q_min']+b['q_max'])/2 for b in group]
        axes[0].plot(x,[b['height_rmse_m'] for b in group],style,color='black',label=category)
        axes[1].plot(x,[b['matched_cells'] for b in group],style,color='black',label=category)
    for ax in axes: ax.set(xlabel='Sampling Q bin midpoint',xlim=(0,1)); ax.legend(fontsize=8)
    axes[0].set_ylabel('Height RMSE (m)'); axes[1].set_ylabel('Matched cells (not independent trials)')
    fig.suptitle('Q is sampling sufficiency, not percentage accuracy',fontsize=11)
    fig.savefig(output/'quality-error.png',dpi=180); fig.savefig(output/'quality-error.pdf'); plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path); parser.add_argument('output',type=Path)
    args = parser.parse_args()
    print(json.dumps(analyse(args.folder,args.output),indent=2))
