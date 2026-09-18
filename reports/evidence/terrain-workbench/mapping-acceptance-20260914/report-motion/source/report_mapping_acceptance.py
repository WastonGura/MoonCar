"""Freeze an offline reference, then evaluate a synchronized real depth capture."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import zipfile
import numpy as np
from scipy.spatial.transform import Rotation

from mapping_acceptance import PROFILE, VERSION, build_reference, score
from terrain_reference import read_shapes
from trajectory_evaluation import ROOT, camera_extrinsic, interpolate
from survey_evaluation import write_json,write_csv
from terrain_geometry import project_depth
from terrain_map import TerrainMap


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path):return json.loads(path.read_text(encoding='utf-8'))


def prepare(world,output,origin,radius,budget):
    if output.exists():raise ValueError('Preserve previous plans: choose a new directory')
    if not math.isfinite(budget) or not 5<=budget<=300:raise ValueError('Budget must be 5..300 simulation seconds')
    reference=build_reference(read_shapes(world),origin,radius)
    reference.update(profile=PROFILE,budget_seconds=budget,created_utc=datetime.now(timezone.utc).isoformat(),
        world_sha256=digest(world),frame='map',world_from_map=np.eye(4).tolist(),
        purpose='Single-surface evaluation; fixed circle intersected with [-25,25] square',
        time_origin='first synchronized recorded depth frame; not experiment launch')
    output.mkdir(parents=True);shutil.copyfile(world,output/'survey_yard.sdf')
    write_json(output/'reference.json',reference)
    write_json(output/'hashes.json',{p.name:digest(p) for p in output.iterdir() if p.is_file()})
    return reference


def matrices(value,n,name):
    a=np.asarray(value)
    if a.shape!=(n,4,4) or not np.isfinite(a).all() or not np.allclose(a[:,3,:],[0,0,0,1],atol=1e-8):
        raise ValueError('Invalid transforms: '+name)
    rot=a[:,:3,:3]
    if not np.allclose(rot@rot.transpose(0,2,1),np.eye(3),atol=1e-6) or not np.allclose(np.linalg.det(rot),1,atol=1e-6):
        raise ValueError('Non-rigid transforms: '+name)
    return a


def load_npz(path):
    # Bound decompression before loading; raw camera images can be larger than map archives.
    with zipfile.ZipFile(path) as z:
        if len(z.infolist())>20 or sum(i.file_size for i in z.infolist())>500_000_000:
            raise ValueError('Capture exceeds 500 MB evaluation limit')
    with np.load(path,allow_pickle=False) as z:return {k:z[k] for k in z.files}


def rebuild(folder,reference):
    """Primary map uses estimated camera TF only. Truth is exclusively an audit input."""
    manifest=read_json(folder/'manifest.json');data=load_npz(folder/'samples.npz')
    stamps=data['stamp'];n=len(stamps);offsets=data['offsets'];points=data['points']
    if stamps.shape!=(n,) or not n or not np.isfinite(stamps).all() or np.any(stamps<0) or np.any(np.diff(stamps)<=0):
        raise ValueError('Capture timestamps must strictly increase')
    if (offsets.shape!=(n+1,) or offsets.dtype.kind not in 'iu' or offsets[0]!=0 or offsets[-1]!=len(points)
        or np.any(np.diff(offsets)<3) or points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all()):
        raise ValueError('Invalid captured point arrays')
    cameras=matrices(data['estimated_camera'],n,'estimated_camera')
    problems=[];audit=dict(sensor_lineage_verified=False,coordinate_contract_verified=False,cutoff_verified=False,
                          safety_pass=None,runtime_fault=None,problems=problems)
    audit['cutoff_verified']=(manifest.get('status')=='captured' and manifest.get('frames')==n
        and math.isclose(manifest.get('first_stamp',-1),stamps[0]) and math.isclose(manifest.get('last_stamp',-1),stamps[-1])
        and manifest.get('seconds')==reference['budget_seconds'])
    end=float(stamps[0]+reference['budget_seconds']);selected=np.flatnonzero(stamps<=end)
    audit.update(recorded_frames=n,used_frames=len(selected),ignored_after_cutoff=n-len(selected),
        first_stamp=float(stamps[0]),cutoff_stamp=end,last_used_stamp=float(stamps[selected[-1]]),
        dropped_selected_frames=manifest.get('dropped_selected_frames'),
        record_scope='Reconstructed synchronized recorded sensor frames; not a claim to contain every frontend keyframe')
    control_path=folder.parent/(folder.name+'-control.json')
    if control_path.exists():
        control=read_json(control_path)
        missing_action=control.get('condition')=='approach' and 'action_started_sim_stamp' not in control
        audit['runtime_fault']=bool(control.get('status')!='completed' or missing_action or control.get('motion_terminal_phase')=='failed')
        audit['execution_note']='Requested approach never started' if missing_action else 'Control record inspected; not a collision certificate'
    raw_files=('raw-depth.npz','depth-format.json','camera-info.json')
    if all((folder/f).exists() for f in raw_files) and 'projection_pixel_offset' in manifest:
        raw=load_npz(folder/'raw-depth.npz');fmt=read_json(folder/'depth-format.json');info=read_json(folder/'camera-info.json')
        if not np.array_equal(raw['stamps'],stamps) or raw['data'].shape[0]!=n:raise ValueError('Raw depth timestamps differ')
        intrinsic=[info['k'][i] for i in (0,4,2,5)];matched=0
        for i in range(n):
            cloud=project_depth(raw['data'][i].tobytes(),fmt['width'],fmt['height'],fmt['step'],fmt['encoding'],
                fmt['bigendian'],intrinsic,pixel_offset=manifest['projection_pixel_offset']).astype(np.float32)
            if not np.array_equal(cloud,points[offsets[i]:offsets[i+1]]):
                raise ValueError('Raw depth does not reproduce recorded cloud at frame '+str(i))
            matched+=1
        audit['sensor_lineage_verified']=True;audit['bitwise_reprojected_frames']=matched
    contract_path=folder/'coordinate-contract.json'
    if contract_path.exists():
        contract=read_json(contract_path);task=contract['task'];start=task['start']
        expected=np.eye(4);expected[:3,:3]=Rotation.from_euler('z',start[2]).as_matrix();expected[:2,3]=start[:2]
        actual=matrices(data['survey_from_odom'],n,'survey_from_odom')
        matrices(data['map_from_survey'],n,'map_from_survey')
        launch=folder/'runtime-navigation.launch.py'
        # A changed deployment must be reviewed again, never silently certified from frame names.
        approved=ROOT/'ros2_ws/src/mooncar_navigation/launch/navigation.launch.py'
        extrinsic,optical=camera_extrinsic(folder/'runtime-robot.sdf')
        robot=matrices(data['estimated_robot'],n,'estimated_robot')
        coordinate_checks=dict(
            declared_identity=np.array_equal(contract.get('world_from_map'),np.eye(4)) and np.array_equal(reference['world_from_map'],np.eye(4)),
            frame_names=contract['frames']==dict(map_frame='map',odom_frame='survey_odom',base_frame='base_footprint'),
            partition=contract['partition']==manifest['partition'],
            audited_launch=digest(launch)==contract['navigation_launch_sha256']==digest(approved),
            spawn_anchor=bool(np.allclose(actual,expected,atol=1e-6,rtol=0)),
            camera_tf=bool(np.allclose(robot@extrinsic,cameras,atol=1e-6,rtol=0)),
            optical_frame=optical==manifest['optical_frame'],
            world=digest(folder/'survey_yard.sdf')==reference['world_sha256'])
        audit['coordinate_checks']=coordinate_checks
        audit['coordinate_contract_verified']=all(coordinate_checks.values())
        # Report pose error without registration, clipping or selecting favourable frames.
        truth_rows=[json.loads(line) for line in (folder/'truth.jsonl').read_text().splitlines() if line.strip()]
        truth=[(r['stamp'],r['pose']) for r in truth_rows]
        gt=np.stack([interpolate(truth,float(t),manifest['max_truth_gap_seconds'])[0] for t in stamps])
        if not np.allclose(gt,data['truth_robot'],atol=1e-8,rtol=0):raise ValueError('Stored truth differs from synchronized source')
        audit['pose_xy_rmse_m']=float(np.sqrt(np.mean(np.sum((robot[:,:2,3]-gt[:,:2,3])**2,axis=1))))
        audit['pose_z_rmse_m']=float(np.sqrt(np.mean((robot[:,2,3]-gt[:,2,3])**2)))
        audit['coordinate_limitation']='Declared initial surveyed axes, no fitted correction; localization drift remains in measured map.'
    elif digest(folder/'survey_yard.sdf')!=reference['world_sha256']:
        problems.append('Recorded world differs from frozen reference world')
    model=TerrainMap(resolution=reference['resolution'],source='synchronized real depth capture; estimated ROS camera poses')
    baseline=None
    for i in selected:
        camera=cameras[i]
        model.add_frame(points[offsets[i]:offsets[i+1]],float(stamps[i]),camera[:3,3],Rotation.from_matrix(camera[:3,:3]).as_quat())
        if baseline is None:baseline=model.elevation()
    return model,baseline,audit


def plot(rows,output,summary):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap,BoundaryNorm
    keys=[(c['ix'],c['iy']) for c in rows];xs,ys=zip(*keys);r=summary['resolution']
    extent=(min(xs)*r,(max(xs)+1)*r,min(ys)*r,(max(ys)+1)*r)
    def grid(field):
        a=np.full((max(ys)-min(ys)+1,max(xs)-min(xs)+1),np.nan)
        for c in rows:
            value=c.get(field)
            if value is not None:a[c['iy']-min(ys),c['ix']-min(xs)]=value
        return np.ma.masked_invalid(a)
    states=['unobserved','invalid_fit','error_exceeds_threshold','qualified','outside_representation','not_evaluable']
    display=[dict(c,state_id=states.index(c['state'])) for c in rows]
    rows=display
    fig,axes=plt.subplots(2,3,figsize=(12,7.8),layout='constrained')
    heights=[c[k] for c in rows for k in ('reference_z','estimated_z') if c[k] is not None and math.isfinite(c[k])]
    low=min(heights) if heights else 0;high=max(heights) if heights else 1
    if high-low<.001:high=low+.001
    fields=[('reference_z','Reference height (m)'),('estimated_z','Measured height (m)'),('state_id','Cell evaluation'),
            ('height_error_m','Signed height error (m)'),('slope_error_deg','Signed slope error (deg)'),('reference_slope','Reference slope (deg)')]
    for ax,(field,title) in zip(axes.flat,fields):
        values=grid(field)
        gray=ListedColormap(plt.get_cmap('gray')(np.linspace(.10,.85,256)));gray.set_bad('white')
        opts=dict(cmap=gray,origin='lower',extent=extent,interpolation='nearest')
        if field in ('reference_z','estimated_z'):opts.update(vmin=low,vmax=high)
        if field=='state_id':opts.update(cmap=ListedColormap(['white','#cccccc','#555555','black','#eeeeee','#999999']),norm=BoundaryNorm(np.arange(-.5,6),6))
        im=ax.imshow(values,**opts);bar=fig.colorbar(im,ax=ax,shrink=.8)
        if field=='state_id':
            bar.set_ticks(range(6));bar.set_ticklabels(['Unobserved','Invalid fit','Over error limit','Qualified','Excluded','Not evaluable'])
        ax.set(title=title,xlabel='X (m)',ylabel='Y (m)',aspect='equal')
    fig.suptitle('MoonCar single-surface capture evaluation',fontsize=14)
    value='N/A' if summary['S'] is None else f"{summary['S']:.2f}"
    fig.supxlabel(f"Status: {summary['status']} | S: {value} | White error cells: no valid evaluated estimate. No error clipping.",fontsize=8)
    fig.savefig(output/'maps.png',dpi=170)
    # Matplotlib 3.6's indexed PNG predictor omits bit depth in PDF DecodeParms.
    # Uncompressed RGB avoids diagonal corruption of categorical raster cells.
    with matplotlib.rc_context({'pdf.compression':0}):fig.savefig(output/'maps.pdf')
    plt.close(fig)


def evaluate(folder,plan,output):
    folder,plan,output=folder.resolve(),plan.resolve(),output.resolve()
    if output.exists() or folder in output.parents or plan in output.parents:raise ValueError('Choose a new output outside frozen inputs')
    reference=read_json(plan/'reference.json');hashes=read_json(plan/'hashes.json')
    if any(hashes.get(name)!=digest(plan/name) for name in ('reference.json','survey_yard.sdf')):
        raise ValueError('Frozen reference hash mismatch')
    if digest(plan/'survey_yard.sdf')!=reference['world_sha256']:raise ValueError('Reference world hash mismatch')
    model,baseline,audit=rebuild(folder,reference)
    rows,result=score(reference,model.elevation(),evidence=audit,profile=reference['profile'])
    _,initial=score(reference,baseline,evidence=audit,profile=reference['profile'])
    result.update(resolution=reference['resolution'],budget_seconds=reference['budget_seconds'],
        baseline_S=initial['S'],delta_S=result['S']-initial['S'] if result['S'] is not None else None,
        source=str(folder),reference=str(plan),reference_audit=reference['reference_audit'],
        analysis_kind='Evaluation pipeline demonstration; not a strategy efficacy comparison',
        input_sha256={p.name:digest(p) for p in folder.iterdir() if p.is_file()},
        reference_sha256=hashes)
    control_path=folder.parent/(folder.name+'-control.json')
    if control_path.exists():result['control_sha256']=digest(control_path)
    output.mkdir(parents=True);write_json(output/'summary.json',result);write_csv(output/'cells.csv',rows)
    write_json(output/'reference.json',reference);model.save(output/'observations.npz');plot(rows,output,result)
    def number(x):return '不可评价 / 无定义' if x is None else f'{x:.4f}'
    table='\n'.join(f'| {label} | {number(result[key])} |' for key,label in [
        ('S','达标地表覆盖分 S（0–100）'),('C_obs','原始观测覆盖率（0–1）'),('C','有效重建覆盖率 C（0–1）'),
        ('P','几何达标率 P（0–1）'),('height_rmse_m','高度 RMSE（m）'),('slope_mae_deg','坡度 MAE（°）'),
        ('evaluation_area_m2','评价面积（m²）'),('excluded_area_m2','表示范围外面积（m²）'),('baseline_S','首帧分数 S₀'),('delta_S','本次分数增量')])
    (output/'README.md').write_text(f'''# MoonCar 地形采集评价报告

状态：`{result['status']}`。这是评价流程的真实记录示例，不是三策略优劣结论或实物验收。
未通过项：{'; '.join(result['failed_gates']) or '无已确认的失败项；仍须检查证据和安全状态'}。

| 指标 | 结果 |
| --- | ---: |
{table}

S = 100 × 几何达标采集面积 / 固定评价面积。高度和坡度必须同时满足冻结阈值；未观测和错误估计不从分母删除。
高度容差 {reference['profile']['height_tolerance_m']} m，坡度容差 {reference['profile']['slope_tolerance_deg']}°，均为暂定仿真目标。
安全记录未在本评价器中认证，因此几何达标也不自动代表整体验收通过。

![参考、采集、逐格状态和误差](maps.png)

黑白图中深色“Qualified”为合格，浅灰或空白不意味着合格。高度/坡度误差图保留全部有效估计的误差；没有可信对应时不画达标结论。

评价区域、参考收敛检查、平地/地形分项、时间截止、原始深度重投影核验、坐标核验及输入哈希见 `summary.json`。
逐格数据见 `cells.csv`，重建后的传感器档案见 `observations.npz`。记录每约 0.5 秒选帧；不宣称覆盖前端所有关键帧。
参考来自冻结 SDF 的解析视觉几何与中央 80% 网格采样；曲面网格化、细于采样间距的特征仍有近似误差。
本结果保留估计相机位姿误差，未使用真值位姿替换，也未对最终地图进行配准修正。

证据问题：{'; '.join(result['problems']) or '未发现阻止几何计分的证据缺口；安全状态仍未认证'}。
''',encoding='utf-8')
    source=output/'source';source.mkdir()
    for path in [Path(__file__),ROOT/'scripts/mapping_acceptance.py',ROOT/'scripts/terrain_reference.py',
                 ROOT/'scripts/trajectory_evaluation.py',ROOT/'scripts/survey_evaluation.py',ROOT/'scripts/simulation_experiment.py',
                 ROOT/'ros2_ws/src/mooncar_terrain/scripts/terrain_map.py',ROOT/'ros2_ws/src/mooncar_terrain/scripts/terrain_geometry.py']:
        shutil.copyfile(path,source/path.name)
    write_json(output/'hashes.json',{str(p.relative_to(output)):digest(p) for p in output.rglob('*') if p.is_file() and p.name!='hashes.json'})
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('prepare');p.add_argument('world',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--origin',type=float,nargs=2,required=True);p.add_argument('--radius',type=float,required=True);p.add_argument('--budget',type=float,required=True)
    p=sub.add_parser('evaluate');p.add_argument('capture',type=Path);p.add_argument('plan',type=Path);p.add_argument('output',type=Path)
    a=parser.parse_args()
    result=prepare(a.world,a.output,a.origin,a.radius,a.budget) if a.command=='prepare' else evaluate(a.capture,a.plan,a.output)
    print(json.dumps({k:result[k] for k in ('status','S','C','P','reference_audit','problems') if k in result},ensure_ascii=False,indent=2))
