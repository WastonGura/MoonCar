#!/usr/bin/env python3
"""Validate -> replay baseline reconstruction -> export -> report. No ROS or hardware."""
import argparse
import csv
from datetime import datetime, timezone
import html
import json
import math
from pathlib import Path
import shutil
import sys

import numpy as np

from survey_contract import (ROOT, Frame, TASK_FORMAT, INPUT_FORMAT, digest, grid_indices,
                             keys, load_input, named, read_json, validate_task, TerrainMap)
from survey_configuration import audit_configuration, project_configuration
from terrain_geometry import transform_points
from mapping_acceptance import checked_cells, score

STATES = {
    'unobserved': ('未观测', '#eeeeee'),
    'invalid_fit': ('无法有效拟合', '#b77945'),
    'fitted_accuracy_unknown': ('已拟合，精度未知', '#568ab0'),
    'qualified': ('相对所给参考达标', '#4c956c'),
    'error_exceeds_threshold': ('相对参考误差超限', '#c34b52'),
    'outside_representation': ('参考声明为表示范围外', '#9388a2'),
    'not_evaluable': ('参考／证据不足，不可评价', '#dbba65'),
}


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def task_cells(task):
    x0, y0, x1, y1 = grid_indices(task); r = task['grid']['cell_m']
    return {(ix, iy): dict(ix=ix, iy=iy, x=(ix+.5)*r, y=(iy+.5)*r)
            for iy in range(y0, y1) for ix in range(x0, x1)}


def load_reference(task, task_path, cells):
    if 'reference' not in task: return None, [], False
    path = (Path(task_path).resolve().parent / task['reference']['path']).resolve()
    ref = read_json(path)
    keys(ref, ('format', 'frame', 'unit', 'vertical_datum', 'resolution', 'source', 'alignment', 'cells'))
    if ref['format'] != 'mooncar-survey-reference-v1': raise ValueError('Unsupported reference adapter')
    for key in ('frame', 'unit', 'vertical_datum'):
        if ref[key] != task['coordinates'][key]: raise ValueError('Reference coordinate mismatch: ' + key)
    if ref['resolution'] != task['grid']['cell_m']: raise ValueError('Reference grid size mismatch')
    keys(ref['source'], ('kind', 'description')); named(ref['source']['description'], 'reference source')
    if ref['source']['kind'] not in ('synthetic', 'gazebo_world', 'independent_measurement'):
        raise ValueError('Reference must declare an independent source category')
    alignment = ref['alignment']; keys(alignment, ('status', 'description'), ('evidence_file', 'sha256'))
    named(alignment['description'], 'reference alignment description')
    files = [path]
    if alignment['status'] not in ('unverified', 'synthetic_exact', 'provided_verified'):
        raise ValueError('Unknown reference alignment status')
    qualified = alignment['status'] == 'synthetic_exact'
    if qualified and (task['source']['kind'] != 'synthetic' or ref['source']['kind'] != 'synthetic'):
        raise ValueError('Synthetic exact references apply only to synthetic observations')
    if alignment['status'] == 'provided_verified':
        if not alignment.get('evidence_file') or not alignment.get('sha256'):
            raise ValueError('Verified alignment declaration requires a hashed evidence file')
        evidence = (path.parent / alignment['evidence_file']).resolve()
        if not evidence.is_file() or digest(evidence) != alignment['sha256']:
            raise ValueError('Reference alignment evidence hash mismatch')
        files.append(evidence); qualified = True
    refs = checked_cells(ref['cells'], True)
    if set(refs) != set(cells):
        raise ValueError('Reference must cover every declared cell; missing cells cannot shrink the denominator')
    r = task['grid']['cell_m']
    for c in refs.values():
        if any(not math.isclose(c[key], (c[index]+.5)*r, abs_tol=1e-9, rel_tol=0)
               for key, index in (('x', 'ix'), ('y', 'iy'))):
            raise ValueError('Reference cell centres differ from task grid')
    qualified = qualified and task['source']['kind'] != 'unknown'
    return ref, files, qualified


def reconstruct(task, frames, cells):
    terrain = TerrainMap(task['grid']['cell_m'], task['grid']['voxel_m'],
                         task['coordinates']['frame'], task['source']['description'])
    observed = {}; log = []
    for f in frames:
        log.append(fuse_frame(task, f, cells, terrain, observed))
    return terrain, observed, log


def fuse_frame(task, f, cells, terrain, observed):
    """One frame of the same offline reconstruction, also used by the live receiver."""
    r = task['grid']['cell_m']
    start, end = task['time']['start_s'], task['time']['end_s']
    row = dict(stamp_s=f.stamp, input_points=len(f.points), status='outside_time_window', task_points=0)
    if start <= f.stamp <= end:
        # Match baseline storage precision before assigning boundary cells.
        sensor_points = f.points.astype(np.float32)
        world = transform_points(sensor_points, f.translation, f.quaternion)
        indices = np.floor(world[:, :2] / r).astype(np.int64)
        mask = np.array([tuple(k) in cells for k in indices], dtype=bool)
        for key in indices[mask]:
            key = tuple(key); observed[key] = observed.get(key, 0) + 1
        selected = sensor_points[mask]; row['task_points'] = len(selected)
        row['status'] = 'used' if len(selected) >= 3 else 'too_few_task_points'
        if len(selected) >= 3:
            robot = f.robot
            terrain.add_frame(selected, f.stamp, f.translation, f.quaternion,
                              robot[:3] if robot is not None else None,
                              robot[3:] if robot is not None else None)
    row['projection_note'] = f.note
    return row


def quality_rows(task, terrain, observed, cells, reference, qualified):
    fits = {(c['ix'], c['iy']): c for c in terrain.quality(with_surface=True)['cells']}
    summary = None; scored = {}
    if reference:
        profile = {key: task['targets'][key] for key in ('height_tolerance_m', 'slope_tolerance_deg', 'minimum_precision')}
        ref = dict(reference, reference_audit=dict(passed=qualified))
        result, summary = score(ref, terrain.elevation(), evidence=dict(
            coordinate_contract_verified=qualified, sensor_lineage_verified=qualified,
            cutoff_verified=True, safety_pass=None,
            note='Input/coordinate declarations checked; physical calibration and safety are not independently certified.'), profile=profile)
        scored = {(c['ix'], c['iy']): c for c in result}
    rows = []
    for key, c in cells.items():
        fit = fits.get(key); has_points = key in observed
        valid = bool(fit and fit['valid'])
        state = 'fitted_accuracy_unknown' if valid else 'invalid_fit' if has_points else 'unobserved'
        q = scored.get(key)
        if q and q['state'] == 'outside_representation': state = q['state']
        elif q and valid: state = q['state']
        # Preserve sparse observed cells, even when baseline fusion requires >=3/frame.
        if q and has_points and not valid and q['state'] != 'outside_representation': state = 'invalid_fit'
        row = dict(c, state=state, state_zh=STATES[state][0], observed=has_points, valid=valid,
                   fit_state=fit['fit_state'] if fit else 'too_few_points_for_fusion' if has_points else 'unobserved',
                   observed_points=observed.get(key, 0), estimated_z=fit['z'] if fit else None,
                   estimated_slope=fit['slope'] if fit else None,
                   residual_roughness=fit['roughness'] if fit else None,
                   sampling_sufficiency=fit.get('quality') if fit else None,
                   qualified=q['qualified'] if q else None,
                   height_error_m=q['height_error_m'] if q else None,
                   slope_error_deg=q['slope_error_deg'] if q else None)
        rows.append(row)
    return rows, summary


def export_grid(task, terrain, rows, output):
    """Use baseline exporter, then pad its valid bounding box to the declared task grid."""
    meta = terrain.export_dem(output/'dem.asc')
    old = np.loadtxt(output/'dem.asc', skiprows=6, ndmin=2)
    x0, y0, x1, y1 = grid_indices(task); r = task['grid']['cell_m']
    grid = np.full((y1-y0, x1-x0), -9999., dtype=float)
    if meta['valid_cells']:
        ox = round(meta['xllcorner']/r); oy = round(meta['yllcorner']/r)
        col, top = ox-x0, y1-(oy+meta['nrows'])
        grid[top:top+meta['nrows'], col:col+meta['ncols']] = old
    head = f'ncols {x1-x0}\nnrows {y1-y0}\nxllcorner {x0*r:.12g}\nyllcorner {y0*r:.12g}\ncellsize {r:.12g}\nNODATA_value -9999\n'
    with (output/'dem.asc').open('w', encoding='ascii') as handle:
        handle.write(head)
        np.savetxt(handle, grid, fmt='%.4f')
    meta.update(ncols=x1-x0, nrows=y1-y0, xllcorner=x0*r, yllcorner=y0*r,
                total_cells=(x1-x0)*(y1-y0), task_region=task['region'],
                coordinates=task['coordinates'], unit='m', cell_membership='half-open rectangle, cell centres',
                source_kind=task['source']['kind'], pose_version=task['source']['pose_version'],
                calibration_version=task['source']['calibration_version'],
                limitation='Local single-surface estimate; NODATA is missing/invalid, not flat ground; grid spacing is not accuracy.')
    write_json(output/'dem.json', meta)
    with (output/'cells.csv').open('w', newline='', encoding='utf-8-sig') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)


def render_report(task, rows, summary, configuration, output):
    esc = lambda value: html.escape(str(value), quote=True)
    x0, y0, x1, y1 = grid_indices(task)
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {x1-x0} {y1-y0}" role="img" aria-label="逐格勘测质量，横轴X向右，Y向上">']
    for row in rows:
        label = f"{row['x']:.3f}, {row['y']:.3f} m: {row['state_zh']}"
        svg.append(f'<rect x="{row["ix"]-x0}" y="{y1-row["iy"]-1}" width="1" height="1" fill="{STATES[row["state"]][1]}"><title>{esc(label)}</title></rect>')
    svg.append('</svg>'); svg = ''.join(svg)
    (output/'quality-map.svg').write_text(svg, encoding='utf-8')
    legend = ''.join(f'<span><i style="background:{color}"></i>{label}</span>' for label, color in STATES.values())
    coverage = summary['coverage']; accuracy = summary['accuracy']
    def fmt(value): return '未知／不可评价' if value is None else f'{value:.5g}' if isinstance(value, (int, float)) else esc(value)
    percent = lambda v: '未知／不可评价' if v is None else f'{100*v:.2f}%'
    metrics = ''.join(f'<tr><th>{label}</th><td>{fmt(value)}</td></tr>' for label, value in [
        ('区域内观测覆盖', percent(coverage['observed_fraction'])), ('区域内有效拟合覆盖', percent(coverage['fitted_fraction'])),
        ('相对参考的几何达标率P', percent(accuracy.get('P'))), ('相对参考的达标地表分S（0–100）', accuracy.get('S')),
        ('高度RMSE（m）', accuracy.get('height_rmse_m')), ('坡度MAE（°）', accuracy.get('slope_mae_deg'))])
    config_rows = ''.join(f'<tr><th>{esc(c["label"])}</th><td>{esc(c["expected"])}</td><td>{esc(c["observed"])}</td><td>{esc(c["state"])}</td></tr>' for c in configuration.get('checks', []))
    page = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>MoonCar 地形勘测报告 — {esc(task['id'])}</title>
<style>body{{font:16px/1.65 system-ui,sans-serif;max-width:1050px;margin:32px auto;padding:0 20px;color:#20343d}}h1{{font-size:26px}}table{{border-collapse:collapse;width:100%;margin:16px 0}}td,th{{border:1px solid #ccd5d8;padding:8px;text-align:left;overflow-wrap:anywhere}}.note{{background:#edf3f6;padding:16px}}svg{{display:block;width:100%;max-height:600px;background:#eee}}.legend{{display:flex;gap:12px;flex-wrap:wrap;font-size:14px}}i{{display:inline-block;width:14px;height:14px;margin-right:5px}}a{{color:#175b8c}}</style>
<h1>地形勘测报告</h1><p>任务：{esc(task['id'])}；来源：{esc(task['source']['kind'])}（输入声明）；坐标：{esc(task['coordinates']['frame'])}，米。</p>
<p class="note">离线处理已完成。精度状态：{esc(accuracy['status_zh'])}。拟合成功、采样充分、几何达标与实车安全是不同结论。本报告不提供实车或安全认证。</p>
<h2>任务范围与时间</h2><p>矩形边界：{esc(task['region']['bounds_m'])} m；按格心纳入。格长：{task['grid']['cell_m']} m；体素：{task['grid']['voxel_m']} m。时间：{task['time']['start_s']}～{task['time']['end_s']} s，时钟：{esc(task['time']['clock'])}。</p>
<p>高程基准：{esc(task['coordinates']['vertical_datum'])}。目标状态：{esc(task['targets']['status'])}；依据：{esc(task['targets']['basis'])}。高度容差：{task['targets']['height_tolerance_m']} m；坡度容差：{task['targets']['slope_tolerance_deg']}°；几何达标率目标：{task['targets']['minimum_precision']}。覆盖率仅作统计，不作为通过条件；未测区域精度未知。</p>
<h2>覆盖与误差</h2><table>{metrics}</table><p>观测与拟合覆盖以任务全部格子为分母；参考评价单列其可表达区域，不能用小分母冒充全场完成。全部逐格结果保留在CSV；残差／充分度不作为测量误差界限。</p>
<h2>质量分布</h2><div class="legend">{legend}</div>{svg}<p>X向右，Y向上；图示为任务格网，不补洞、不填平。范围：{esc(task['region']['bounds_m'])} m。</p>
<h2>配置核对</h2><p>状态：{esc(configuration['status'])}；证据范围：{esc(configuration.get('scope','未提供'))}。unknown为未提供；difference为差异待核对；consistent仅表示在声明比较容差内一致。源码声明不等于实际运行配置。工具未修改参数。</p>
<table><tr><th>项目</th><th>预测／期望</th><th>观测／声明</th><th>核对状态</th></tr>{config_rows}</table>
<h2>成果与来源</h2><p><a href="observations.npz">观测档案</a> · <a href="cloud.ply">点云</a> · <a href="dem.asc">高程DEM</a> · <a href="cells.csv">逐格状态CSV</a> · <a href="summary.json">指标与资格</a> · <a href="configuration-audit.json">配置核对</a> · <a href="input-index.json">输入索引</a> · <a href="task-replay.json">复算任务</a></p>
<p>参考对应和标定版本由输入材料声明；哈希与结构核验不代替独立实测。原始材料保留在inputs，算法和来源清单见run-metadata.json与checksums.json。失败与未知项不会被改成合格。</p></html>'''
    (output/'report.html').write_text(page, encoding='utf-8')
    (output/'README.md').write_text(f'''# 地形勘测结果

任务：{task['id']}。来源：{task['source']['kind']}（输入声明）。

处理完成；{accuracy['status_zh']}。本结果不构成实物或安全认证。

打开 report.html 查看任务边界、质量图、指标和配置差异。cells.csv 保留全部格子；dem.asc 的空白为 NODATA。使用同一基线重新处理：

`python scripts/offline_survey.py run "{output / 'task-replay.json'}" --output 新目录`

任务和来源见 task.json、input-index.json；复算输入位于 inputs。无参考时准确度保持未知。
''', encoding='utf-8')


def run_task(task_path, output):
    task_path = Path(task_path).resolve(); output = Path(output).resolve()
    task = validate_task(read_json(task_path)); frames, contract, files, source = load_input(task, task_path)
    cells = task_cells(task); reference, reference_files, qualified = load_reference(task, task_path, cells)
    files += reference_files
    config = None
    configuration = dict(status='not_checked', scope='not_provided', checks=[], differences=0, unknown=5)
    if 'configuration' in task:
        path = (task_path.parent/task['configuration']['path']).resolve(); config = read_json(path)
        for row in config.get('source_files', []):
            row['path'] = str((path.parent/row['path']).resolve())
        configuration = audit_configuration(config); files.append(path)
        files += [Path(row['path']).resolve() for row in config.get('source_files', []) if Path(row['path']).is_file()]
    files = list(dict.fromkeys(files))
    if output.exists(): raise ValueError('Output already exists; choose a new directory')
    if any(file == output or file.is_relative_to(output) for file in files) or (source.is_dir() and output.is_relative_to(source)):
        raise ValueError('Output must not contain or overwrite inputs, nor be inside a folder input')
    before = {str(file): digest(file) for file in files}
    terrain, observed, frame_log = reconstruct(task, frames, cells)
    rows, accuracy = quality_rows(task, terrain, observed, cells, reference, qualified)
    n = len(rows); a = task['grid']['cell_m']**2
    coverage = dict(task_cells=n, task_grid_area_m2=n*a, rectangle_area_m2=(task['region']['bounds_m'][2]-task['region']['bounds_m'][0])*(task['region']['bounds_m'][3]-task['region']['bounds_m'][1]),
                    observed_cells=sum(c['observed'] for c in rows), fitted_cells=sum(c['valid'] for c in rows))
    coverage.update(observed_fraction=coverage['observed_cells']/n, fitted_fraction=coverage['fitted_cells']/n,
                    state_counts={state: sum(c['state'] == state for c in rows) for state in STATES})
    if accuracy is None:
        accuracy = dict(status='no_independent_reference', P=None, S=None, height_rmse_m=None,
                        slope_mae_deg=None, evaluable=False, geometry_pass=None)
    accuracy['status_zh'] = ('相对所给参考可评价；仍需注意参考来源与实物适用范围' if accuracy['evaluable']
                             else '无合格独立参考或对应证据，准确度未知')
    summary = dict(format='mooncar-survey-result-v1', execution_status='completed', task_id=task['id'],
                   source=task['source'], coverage=coverage, accuracy=accuracy,
                   frames=dict(input=len(frames), fused=len(terrain.frames),
                               outside_time_window=sum(f['status'] == 'outside_time_window' for f in frame_log),
                               too_few_task_points=sum(f['status'] == 'too_few_task_points' for f in frame_log)),
                   configuration_status=configuration['status'],
                   safety_certified=False, physical_accuracy_certified=False,
                   provenance_meaning='Source, calibration and reference qualification are declarations; input hashes and contracts are checked, not independent physical certification.')
    # Input files are rechecked after use before publishing results.
    if any(digest(Path(path)) != sha for path, sha in before.items()): raise ValueError('Input changed during processing')
    output.mkdir(parents=True)
    try:
        inputs = output/'inputs'; inputs.mkdir(); index = []
        copied = {}
        for i, file in enumerate(files):
            target = inputs/f'{i:03d}-{file.name}'; shutil.copyfile(file, target)
            if digest(target) != before[str(file)]: raise ValueError('Input changed while copying')
            copied[str(file)] = target.relative_to(output).as_posix()
            index.append(dict(original=str(file), copy=target.relative_to(output).as_posix(), sha256=before[str(file)]))
        write_json(output/'input-index.json', index); write_json(output/'task.json', task)
        write_json(output/'input-contract.json', contract); write_json(output/'frame-log.json', frame_log)
        terrain.save(output/'observations.npz'); terrain.export_ply(output/'cloud.ply'); terrain.export_mesh(output/'surface.obj')
        export_grid(task, terrain, rows, output)
        write_json(output/'configuration-audit.json', configuration)
        if reference: write_json(output/'reference.json', reference)
        if config: write_json(output/'configuration.json', config)
        # Re-run from the copied original adapter data, not the fused/filtered output.
        replay = json.loads(json.dumps(task)); replay['input']['path'] = copied.get(str(source), 'inputs/folder')
        if task['input']['format'] == 'folder-v1':
            folder = inputs/'folder'; folder.mkdir()
            for file in files:
                if file.is_relative_to(source):
                    target = folder/file.relative_to(source); target.parent.mkdir(parents=True, exist_ok=True); shutil.copyfile(inputs/Path(copied[str(file)]).name, target)
            replay['input']['path'] = 'inputs/folder'
        else:
            replay['input']['contract'] = copied[str((task_path.parent/task['input']['contract']).resolve())]
        if reference:
            replay['reference']['path'] = 'reference.json'
            if reference['alignment'].get('evidence_file'):
                original = (reference_files[0].parent/reference['alignment']['evidence_file']).resolve()
                reference['alignment']['evidence_file'] = copied[str(original)]
                write_json(output/'reference.json', reference)
        if config:
            for row in config.get('source_files', []): row['path'] = copied.get(str(Path(row['path']).resolve()), row['path'])
            # Relative source paths in packaged config are resolved by run_task before audit.
            write_json(output/'configuration.json', config); replay['configuration']['path'] = 'configuration.json'
        write_json(output/'task-replay.json', replay)
        write_json(output/'summary.json', summary)
        sources = [Path(__file__), ROOT/'scripts/survey_contract.py', ROOT/'scripts/survey_configuration.py',
                   ROOT/'scripts/mapping_acceptance.py', ROOT/'ros2_ws/src/mooncar_terrain/scripts/terrain_map.py',
                   ROOT/'ros2_ws/src/mooncar_terrain/scripts/terrain_geometry.py']
        write_json(output/'run-metadata.json', dict(python=sys.version, numpy=np.__version__, completed_utc=datetime.now(timezone.utc).isoformat(),
            baseline_algorithms_unchanged=True, code_sha256={p.relative_to(ROOT).as_posix(): digest(p) for p in sources},
            input_hashes_unchanged=True, task_frame=task['coordinates'], source_kind=task['source']['kind']))
        render_report(task, rows, summary, configuration, output)
        write_json(output/'checksums.json', {p.relative_to(output).as_posix(): digest(p) for p in output.rglob('*') if p.is_file()})
    except Exception as error:
        write_json(output/'failure.json', dict(execution_status='failed', error=str(error)))
        raise
    return summary


def create_example(directory):
    """Known synthetic plane with a missing patch, biased patch and mixed-height cell."""
    directory = Path(directory).resolve()
    if directory.exists(): raise ValueError('Example directory already exists')
    directory.mkdir(parents=True); data = directory/'data'; data.mkdir()
    task = dict(format=TASK_FORMAT, id='synthetic-terrain-survey',
        coordinates=dict(frame='survey_local', unit='m', axes='ENU', vertical_datum='synthetic plane origin'),
        region=dict(type='rectangle', bounds_m=[0, 0, 2, 2]),
        time=dict(clock='simulation', start_s=1, end_s=3), grid=dict(cell_m=.25, voxel_m=.05),
        targets=dict(height_tolerance_m=.05, slope_tolerance_deg=5,
                     minimum_precision=.9, status='provisional', basis='Existing provisional simulation targets; not real-device certification'),
        source=dict(kind='synthetic', description='Known analytic plane; deliberate holes/bias/mixed surfaces; NOT measured data',
                    calibration_version='synthetic-identity-v1', pose_version='synthetic-fixed-v1'),
        input=dict(format='folder-v1', path='data'), reference=dict(path='reference.json'),
        configuration=dict(path='configuration.json'))
    contract = dict(format=INPUT_FORMAT, **task['coordinates'], clock='simulation', source_kind='synthetic',
                    calibration_version=task['source']['calibration_version'], pose_version=task['source']['pose_version'])
    x, y = np.meshgrid(np.arange(.0125, 2, .025), np.arange(.0125, 2, .025))
    points = np.column_stack((x.ravel(), y.ravel(), (.1*x+.05*y).ravel()))
    points = points[~((points[:, 0] >= 1.5) & (points[:, 1] >= 1.5))]
    bias = (points[:, 0] < .5) & (points[:, 1] < .5); points[bias, 2] += .12
    mixed = (points[:, 0] >= .75) & (points[:, 0] < 1.) & (points[:, 1] >= .75) & (points[:, 1] < 1.)
    duplicate = points[mixed].copy(); duplicate[:, 2] += .4
    points = np.concatenate((points, duplicate))
    sensor = points - [0, 0, 1.]; np.save(data/'points.npy', sensor.astype(np.float32), allow_pickle=False)
    rows = [dict(stamp_s=stamp, points_file='points.npy', pose=dict(translation_m=[0, 0, 1], quaternion_xyzw=[0, 0, 0, 1])) for stamp in (1., 2., 3., 4.)]
    write_json(data/'manifest.json', dict(contract=contract, frames=rows))
    reference = dict(format='mooncar-survey-reference-v1', frame='survey_local', unit='m',
        vertical_datum=task['coordinates']['vertical_datum'], resolution=.25,
        source=dict(kind='synthetic', description='Analytic plane z=0.1x+0.05y, defined before reconstruction'),
        alignment=dict(status='synthetic_exact', description='Same explicitly constructed synthetic coordinate frame'),
        cells=[dict(c, category='terrain', z=.1*c['x']+.05*c['y'], slope=math.degrees(math.atan(math.hypot(.1,.05)))) for c in task_cells(task).values()])
    write_json(directory/'reference.json', reference); write_json(directory/'configuration.json', project_configuration())
    write_json(directory/'task.json', task)
    return dict(example_task=str(directory/'task.json'), provenance='synthetic, not physical or Gazebo data')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('run'); p.add_argument('task', type=Path); p.add_argument('--output', type=Path, required=True)
    p = sub.add_parser('validate'); p.add_argument('task', type=Path)
    p = sub.add_parser('project-config'); p.add_argument('--output', type=Path, required=True)
    p = sub.add_parser('create-example'); p.add_argument('directory', type=Path)
    args = parser.parse_args()
    try:
        if args.command == 'create-example':
            result = create_example(args.directory)
        elif args.command == 'project-config':
            if args.output.exists(): raise ValueError('Configuration output already exists')
            args.output.parent.mkdir(parents=True, exist_ok=True); config = project_configuration()
            write_json(args.output, config); result = audit_configuration(config)
        elif args.command == 'validate':
            task = validate_task(read_json(args.task)); frames, _, _, _ = load_input(task, args.task)
            load_reference(task, args.task, task_cells(task))
            result = dict(valid=True, task_id=task['id'], frames=len(frames), note='Contract validation is not a measurement certificate')
        else:
            result = run_task(args.task, args.output)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)); return 0
    except (ValueError, OSError, KeyError, TypeError, OverflowError) as error:
        print(json.dumps(dict(status='rejected', error=str(error)), ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
