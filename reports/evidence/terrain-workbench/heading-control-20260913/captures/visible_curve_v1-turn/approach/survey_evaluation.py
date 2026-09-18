"""Versioned survey metrics, automatic evidence capture and offline reports (no ROS)."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
import uuid
import zipfile

ROOT = Path(__file__).resolve().parents[1]
VERSION = 'survey-metrics-v1'


def write_json(path, value):
    temporary = Path(str(path)+'.'+uuid.uuid4().hex+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    temporary.replace(path)


def scoped_cells(quality, origin, radius, keys=None):
    r = quality['resolution']
    return {(c['ix'], c['iy']): c for c in quality['cells']
            if abs((c['ix']+.5)*r) <= 25 and abs((c['iy']+.5)*r) <= 25
            and math.hypot((c['ix']+.5)*r-origin[0], (c['iy']+.5)*r-origin[1]) <= radius
            and (keys is None or (c['ix'], c['iy']) in keys)}


def metrics(quality, origin, radius):
    """Cell-centre disk intersected with the fixed 50 m world; never discovered-area denominator."""
    r = quality['resolution']; cells = scoped_cells(quality, origin, radius)
    count = sum(abs((x+.5)*r) <= 25 and abs((y+.5)*r) <= 25
                and math.hypot((x+.5)*r-origin[0], (y+.5)*r-origin[1]) <= radius
                for x in range(math.floor((origin[0]-radius)/r), math.ceil((origin[0]+radius)/r))
                for y in range(math.floor((origin[1]-radius)/r), math.ceil((origin[1]+radius)/r)))
    area = count*r*r
    return dict(roi_area_m2=area, observed_area_m2=len(cells)*r*r,
                observed_fraction=len(cells)/count if count else None,
                quality_area_m2=sum(c['quality'] for c in cells.values())*r*r,
                sufficient_area_m2=sum(c['quality'] >= .8 for c in cells.values())*r*r,
                mean_observed_quality=sum(c['quality'] for c in cells.values())/len(cells) if cells else None,
                valid_surface_area_m2=sum(c.get('fit_state')=='valid' for c in cells.values())*r*r if all('fit_state' in c for c in cells.values()) else None,
                frames=quality.get('frames', 0), quality_stamp=quality.get('stamp'))


def gain(before, after, origin, radius, keys=None):
    if before['resolution'] != after['resolution']:
        raise ValueError('Quality resolution changed during action')
    old = scoped_cells(before, origin, radius, keys); new = scoped_cells(after, origin, radius, keys)
    area = before['resolution']**2
    eligible={k for k,c in old.items() if c.get('fit_state') in ('too_few_voxels','insufficient_xy_spread')}
    recovered=sum(new.get(k,{}).get('fit_state')=='valid' for k in eligible)
    has_surface=all('fit_state' in c for c in (*old.values(),*new.values()))
    return dict(new_observed_m2=len(new.keys()-old.keys())*area,
                resampling_eligible_cells=len(eligible) if has_surface else None,
                resampling_recovered_cells=recovered if has_surface else None,
                recovered_surface_m2=recovered*area if has_surface else None,
                new_quality_m2=sum(c['quality'] for k,c in new.items() if k not in old)*area,
                resampling_quality_m2=sum(new.get(k, {'quality':0})['quality']-c['quality'] for k,c in old.items())*area,
                quality_gain_m2=(sum(c['quality'] for c in new.values())-sum(c['quality'] for c in old.values()))*area)


class EvidenceRecorder:
    """Independent sampling; plotting runs in a subprocess after motion has stopped."""
    def __init__(self, survey):
        self.survey = survey
        self.folder = survey.path.parent/'research'/survey.path.stem
        self.folder.mkdir(parents=True, exist_ok=False)
        self.done = threading.Event(); self.errors = []; self.last_scene = -math.inf
        self.path_m = 0.; self.last_pose = None; self.last_stamp = None
        manifest = dict(version=VERSION, run_id=survey.path.stem, created_utc=datetime.now(timezone.utc).isoformat(),
                        task=survey.task, mission=json.loads(json.dumps(survey.state())), strategy=survey.data.get('strategy','quality_view_heuristic_v1'),
                        quality_formula='C * (0.7 + 0.3 * min(V/3, 1))', sufficient_threshold=.8,
                        scope='cell centres inside start-relative disk intersected with [-25,25]^2',
                        runtime=sys.version, status='recording', reference_status='not_provided',
                        ros_distro=os.environ.get('ROS_DISTRO'),
                        limitations=['sampling quality is not accuracy or traversability',
                                     'pose path length includes localization drift; sampled, not wheel odometry',
                                     'ROI coverage is projected observed cells, including obstacles; not reachable-ground completion',
                                     'scene images are asynchronous GUI captures, not timestamp-synchronized sensor frames'])
        write_json(self.folder/'manifest.json', manifest)
        self.manifest = manifest
        # Save exact dirty sources/configuration, not just a commit that omits local changes.
        with zipfile.ZipFile(self.folder/'source.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
            for base in ('scripts', 'containers', 'ros2_ws/src'):
                for path in sorted((ROOT/base).rglob('*')):
                    if path.is_file() and path.suffix in ('.py','.yaml','.xacro','.sdf','.xml','.sh'):
                        archive.write(path, path.relative_to(ROOT).as_posix())
            for path in sorted((ROOT/'web').glob('*')):
                if path.suffix in ('.js','.html','.css','.json'):archive.write(path,path.relative_to(ROOT).as_posix())
        for name in ('task.json','survey_yard.sdf','gui.config'):
            source = survey.path.parent/name
            if source.exists(): shutil.copyfile(source, self.folder/name)
        self.sample('start')
        self.worker = threading.Thread(target=self.run, daemon=True); self.worker.start()

    def sample(self, label=None):
        survey = self.survey
        with survey.motion.lock:
            quality = survey.motion.quality
            state = survey.motion.state()
            stamp = survey.now()
        if quality is None: raise ValueError('No sampling quality available')
        elapsed = max(0, stamp-survey.data['started'])
        pose = state.get('pose')
        if pose and self.last_pose and self.last_stamp is not None and stamp > self.last_stamp:
            self.path_m += math.hypot(pose['x']-self.last_pose['x'], pose['y']-self.last_pose['y'])
        self.last_pose, self.last_stamp = pose, stamp
        row = dict(sim_stamp=stamp, elapsed_seconds=elapsed, phase=survey.data['phase'], pose=pose,
                   ready=state.get('ready'), motion_error=state.get('error'), path_m=self.path_m,
                   diagnostics=state.get('diagnostics'),stop_diagnostics=state.get('stop_diagnostics'),
                   **metrics(quality, survey.origin, survey.data['radius']))
        with (self.folder/'telemetry.jsonl').open('a', encoding='utf-8') as handle:
            handle.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
        if label or elapsed-self.last_scene >= 30:
            name = label or f'{elapsed:08.1f}s'
            write_json(self.folder/f'quality-{name}.json', quality)
            write_json(self.folder/f'navigation-{name}.json', state)
            scene = survey.path.parent/'scene.jpg'
            if scene.exists() and time.time()-scene.stat().st_mtime < 5:
                shutil.copyfile(scene, self.folder/f'scene-{name}.jpg')
                row['scene_age_wall_seconds'] = time.time()-scene.stat().st_mtime
            else:
                row['scene_missing'] = True
            write_json(self.folder/f'checkpoint-{name}.json', row)
            self.last_scene = elapsed
        return row

    def run(self):
        last = self.survey.now()
        while not self.done.wait(.25):
            now = self.survey.now()
            if now-last < 1: continue
            try: self.sample()
            except Exception as error:
                self.errors.append(str(error));self.done.set()
                self.survey.data['evidence_error']=str(error)
                self.survey.stop('实验记录失败，已停止自主探测','fault')
            last = now

    def finish(self):
        self.done.set(); self.worker.join(timeout=5)
        try:
            final_row=self.sample('end')
            target_stamp=final_row['quality_stamp'] or 0
            write_json(self.folder/'mission.json', self.survey.state())
            # Flush the existing recorder service instead of waiting five slow simulation seconds.
            saved=subprocess.run(['ros2','service','call','/mooncar/map/save','std_srvs/srv/Trigger','{}'],
                                 capture_output=True,text=True,timeout=20)
            (self.folder/'archive-save.log').write_text(saved.stdout+saved.stderr,encoding='utf-8')
            if saved.returncode: self.errors.append('Recorder save service failed; checking periodic archive')
            # Recorder saves asynchronously; wait for the final published observation to reach disk.
            sys.path.insert(0, str(ROOT/'ros2_ws/src/mooncar_terrain/scripts'))
            from terrain_map import open_archive
            deadline = time.monotonic()+10
            while True:
                sources = sorted(self.survey.path.parent.glob('*.npz'))
                if not sources: raise ValueError('Raw observation archive missing')
                source = sources[-1]
                with open_archive(source)[0] as data:
                    saved_stamp = float(data['stamps'][-1]) if len(data['stamps']) else 0
                if saved_stamp >= target_stamp or time.monotonic() >= deadline: break
                time.sleep(.5)
            # Atomic producer replacement means opening this file captures one complete version.
            with source.open('rb') as src, (self.folder/'observations.npz').open('wb') as dst:
                shutil.copyfileobj(src, dst)
            if saved_stamp < target_stamp: self.errors.append('Final raw archive lags published quality')
            self.manifest.update(raw_source=source.name, raw_saved_stamp=saved_stamp,
                                 final_quality_stamp=target_stamp)
        except Exception as error: self.errors.append(str(error))
        self.manifest.update(status='captured' if not self.errors else 'capture_incomplete', capture_errors=self.errors)
        write_json(self.folder/'manifest.json', self.manifest)
        log = (self.folder/'report.log').open('w')
        try:
            subprocess.Popen([sys.executable, str(Path(__file__).resolve()), 'report', str(self.folder)],
                             stdout=log, stderr=subprocess.STDOUT)
        finally: log.close()


def read_rows(path):
    rows = []
    with path.open(encoding='utf-8') as handle:
        for line in handle:
            if line.strip(): rows.append(json.loads(line))
    if not rows: raise ValueError('No telemetry samples')
    return rows


def write_csv(path, rows):
    if not rows: path.write_text('', encoding='utf-8-sig'); return
    fields = list(dict.fromkeys(k for row in rows for k in row))
    temporary=Path(str(path)+'.'+uuid.uuid4().hex+'.tmp')
    with temporary.open('w', newline='', encoding='utf-8-sig') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    temporary.replace(path)


def reference_errors(cells, reference, resolution, frame):
    """Reference is evaluator-only; its alignment/provenance must be supplied explicitly."""
    if (reference.get('frame') != frame or reference.get('resolution') != resolution
            or not reference.get('alignment_description') or not reference.get('source')):
        raise ValueError('Reference requires matching frame/resolution, source and alignment_description')
    lookup={}
    for c in reference['cells']:
        if any(type(c[k]) is not int for k in ('ix','iy')) or not all(type(c[k]) in (float,int) and math.isfinite(c[k]) for k in ('z','slope')):
            raise ValueError('Reference contains invalid cell data')
        key=(c['ix'],c['iy'])
        if key in lookup: raise ValueError('Duplicate reference cell')
        lookup[key]=c
    pairs=[(c,lookup[(c['ix'],c['iy'])]) for c in cells if c.get('valid') and (c['ix'],c['iy']) in lookup]
    return dict(reference_status='provided_alignment_not_independently_verified',reference_cells=len(lookup),
                reference_matched_cells=len(pairs),
                height_rmse_m=math.sqrt(sum((a['z']-b['z'])**2 for a,b in pairs)/len(pairs)) if pairs else None,
                slope_mae_deg=sum(abs(a['slope']-b['slope']) for a,b in pairs)/len(pairs) if pairs else None)


def report(folder, reference=None):
    folder = Path(folder); manifest = json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
    shutil.copyfile(Path(__file__).resolve(),folder/'report_source.py')
    manifest['report_source_sha256']=hashlib.sha256((folder/'report_source.py').read_bytes()).hexdigest()
    manifest['report_status']='processing';write_json(folder/'manifest.json',manifest)
    mission_path = folder/'mission.json'
    mission = json.loads(mission_path.read_text(encoding='utf-8')) if mission_path.exists() else manifest['mission']
    rows = read_rows(folder/'telemetry.jsonl'); first, last = rows[0], rows[-1]
    actions = mission.get('history', [])
    summary = dict(version=VERSION, run_id=manifest['run_id'], strategy=manifest['strategy'],
                   status=mission.get('phase') if mission_path.exists() else 'interrupted',
                   stop_diagnostics=mission.get('stop_diagnostics'),last_motion_stop=last.get('stop_diagnostics'),
                   reason=mission.get('reason') if mission_path.exists() else 'No terminal mission record; service may have exited',
                   capture_status=manifest['status'], budget_seconds=mission['budget_seconds'],
                   source_sha256=hashlib.sha256((folder/'source.zip').read_bytes()).hexdigest() if (folder/'source.zip').exists() else None,
                   scene_sha256=hashlib.sha256((folder/'survey_yard.sdf').read_bytes()).hexdigest() if (folder/'survey_yard.sdf').exists() else None,
                   elapsed_seconds=mission.get('elapsed_seconds',last['elapsed_seconds']),
                   radius_m=mission['radius'], origin=mission['origin'], roi_area_m2=last['roi_area_m2'],
                   initial_observed_area_m2=first['observed_area_m2'], final_observed_area_m2=last['observed_area_m2'],
                   new_observed_area_m2=last['observed_area_m2']-first['observed_area_m2'],
                   initial_quality_area_m2=first['quality_area_m2'], final_quality_area_m2=last['quality_area_m2'],
                   quality_gain_m2=last['quality_area_m2']-first['quality_area_m2'],
                   final_observed_fraction=last['observed_fraction'], sufficient_area_m2=last['sufficient_area_m2'],
                   path_m=last['path_m'], new_frames=last['frames']-first['frames'],
                   actions=len(actions), failed_actions=sum(not a.get('success',False) for a in actions),
                   height_rmse_m=None, slope_mae_deg=None, reference_status='not_provided',
                   limitations=manifest['limitations'])
    elapsed = summary['elapsed_seconds']
    summary['quality_gain_m2_per_sim_second'] = summary['quality_gain_m2']/elapsed if elapsed else None
    summary['initial_valid_surface_area_m2']=first.get('valid_surface_area_m2')
    if first.get('valid_surface_area_m2') is not None and last.get('valid_surface_area_m2') is not None:
        summary['valid_surface_gain_m2']=last['valid_surface_area_m2']-first['valid_surface_area_m2']
    initial=json.loads((folder/'quality-start.json').read_text(encoding='utf-8'))
    final=json.loads((folder/'quality-end.json').read_text(encoding='utf-8'))
    recovery=gain(initial,final,mission['origin'],mission['radius'])
    summary.update({k:recovery[k] for k in ('resampling_eligible_cells','resampling_recovered_cells','recovered_surface_m2')})
    summary['resampling_success_fraction']=recovery['resampling_recovered_cells']/recovery['resampling_eligible_cells'] if recovery['resampling_eligible_cells'] else None
    summary['observation_lag_at_end_seconds'] = last['sim_stamp']-last['quality_stamp'] if last.get('quality_stamp') is not None else None
    scene_hashes={hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.glob('scene-*.jpg')}
    summary['scene_images']=len(list(folder.glob('scene-*.jpg')))
    summary['scene_distinct_images']=len(scene_hashes)
    summary['scene_warning']='All saved scene images identical; use only as scene context, not evidence of vehicle motion' if len(scene_hashes)==1 else ('No scene images' if not scene_hashes else None)
    write_csv(folder/'telemetry.csv',[{**{k:v for k,v in row.items() if k!='pose'},
                                     **{f'pose_{k}':v for k,v in (row.get('pose') or {}).items()}} for row in rows])
    write_csv(folder/'actions.csv',[{k:(json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list)) else v)
                                    for k,v in a.items() if k!='visible_cells'} for a in actions])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PatchCollection
    from matplotlib.colors import LinearSegmentedColormap
    from matplotlib.patches import Rectangle, Circle
    import numpy as np
    observed_gray=LinearSegmentedColormap.from_list('observed_gray',['#c4c4c4','#151515'])
    origin, radius = mission['origin'], mission['radius']
    fig, axes = plt.subplots(1,2,figsize=(10,4.8),layout='constrained')
    for ax,label in zip(axes,('start','end')):
        path = folder/f'quality-{label}.json'
        if not path.exists(): ax.set_title(f'{label}: missing'); continue
        q = json.loads(path.read_text(encoding='utf-8')); r=q['resolution']
        cells = scoped_cells(q,origin,radius).values()
        patches = [Rectangle((c['ix']*r,c['iy']*r),r,r) for c in cells]
        collection = PatchCollection(patches,cmap='Greys',edgecolor='none')
        collection.set_array(np.array([c['quality'] for c in cells])); collection.set_clim(0,1); ax.add_collection(collection)
        ax.add_patch(Circle(origin,radius,fill=False,color='black',linestyle='--'))
        track=[row['pose'] for row in (rows if label=='end' else rows[:1]) if row.get('pose')]
        ax.plot([p['x'] for p in track],[p['y'] for p in track],color='black',linewidth=.8)
        ax.plot(*origin,'kx'); ax.set(xlim=(origin[0]-radius-.5,origin[0]+radius+.5),
                                    ylim=(origin[1]-radius-.5,origin[1]+radius+.5),aspect='equal',
                                    xlabel='map X (m)',ylabel='map Y (m)',title=f'{label}: sampling sufficiency')
    fig.colorbar(collection,ax=axes,label='Sampling score Q (not accuracy)',shrink=.75)
    fig.savefig(folder/'sampling-comparison.png',dpi=180); fig.savefig(folder/'sampling-comparison.pdf'); plt.close(fig)
    fig, axes=plt.subplots(1,2,figsize=(10,3.6),layout='constrained')
    t=[row['elapsed_seconds'] for row in rows]
    for key,label,style in [('observed_area_m2','Observed projected area','-'),('quality_area_m2','Quality-weighted area','--'),('sufficient_area_m2','Q >= 0.8 area',':')]:
        axes[0].plot(t,[row[key] for row in rows],style,color='black',label=label)
    axes[0].set(xlabel='Simulation seconds',ylabel='Area (m2)'); axes[0].legend(fontsize=8)
    axes[1].plot(t,[row['path_m'] for row in rows],color='black'); axes[1].set(xlabel='Simulation seconds',ylabel='Sampled localization path (m)',ylim=(0,max(1,last['path_m']*1.05)))
    for ax in axes: ax.set_xlim(left=0)
    fig.savefig(folder/'progress.png',dpi=180); fig.savefig(folder/'progress.pdf'); plt.close(fig)
    raw=folder/'observations.npz'; cells=[]
    if raw.exists():
        sys.path.insert(0,str(ROOT/'ros2_ws/src/mooncar_terrain/scripts'))
        from terrain_map import TerrainMap,open_archive
        with open_archive(raw)[0] as data:
            count=int(np.searchsorted(data['stamps'], last['quality_stamp'],side='right'))
        terrain=TerrainMap.load(raw,until=count)
        # The raw archive may carry warm-start history; freeze at the evaluated observation stamp.
        scoped=scoped_cells(terrain.quality(),origin,radius)
        cells=[c for c in terrain.elevation() if (c['ix'],c['iy']) in scoped]
        write_csv(folder/'elevation.csv',cells)
        summary['valid_surface_area_m2']=sum(c['valid'] for c in cells)*terrain.resolution**2
        summary['raw_evaluated_frames']=count
        fig,axes=plt.subplots(1,2,figsize=(10,4.8),layout='constrained')
        cloud=terrain.cloud();keep=(np.hypot(cloud[:,0]-origin[0],cloud[:,1]-origin[1])<=radius)&(np.abs(cloud[:,:2])<=25).all(axis=1)
        cloud=cloud[keep]
        if len(cloud):
            plot=axes[0].scatter(cloud[:,0],cloud[:,1],c=cloud[:,2],s=.4,cmap=observed_gray,vmin=-.1,vmax=1.5)
            fig.colorbar(plot,ax=axes[0],label='Observed height (m)',shrink=.7)
        valid=[c for c in cells if c['valid']]
        patch=PatchCollection([Rectangle((c['ix']*terrain.resolution,c['iy']*terrain.resolution),terrain.resolution,terrain.resolution) for c in valid],cmap=observed_gray)
        patch.set_array(np.array([c['slope'] for c in valid]));patch.set_clim(0,60);axes[1].add_collection(patch)
        fig.colorbar(patch,ax=axes[1],label='Fitted slope (degrees)',shrink=.7)
        for ax,title in zip(axes,('Observed point cloud (top view)','Valid single-surface terrain')):
            ax.set(xlim=(origin[0]-radius-.5,origin[0]+radius+.5),ylim=(origin[1]-radius-.5,origin[1]+radius+.5),aspect='equal',title=title,xlabel='map X (m)',ylabel='map Y (m)')
        fig.savefig(folder/'collected-map.png',dpi=180);fig.savefig(folder/'collected-map.pdf');plt.close(fig)
        if reference:
            ref=json.loads(Path(reference).read_text(encoding='utf-8'))
            summary.update(reference_errors(cells,ref,terrain.resolution,terrain.frame))
            write_json(folder/'reference.json',ref)
    elif reference: raise ValueError('Raw archive required for reference comparison')
    write_json(folder/'summary.json',summary)
    (folder/'README.md').write_text(f'''# 自主探测实验 {summary['run_id']}

- 结束状态：{summary['status']}；{summary['reason']}
- 证据状态：{summary['capture_status']}；版本：{VERSION}
- 仿真时间：{elapsed:.1f} / {summary['budget_seconds']} 秒；定位采样路径：{summary['path_m']:.2f} 米
- 固定评价面积：{summary['roi_area_m2']:.2f} 平方米；新增投影观测面积：{summary['new_observed_area_m2']:.2f} 平方米
- 质量加权面积增量：{summary['quality_gain_m2']:.2f} 平方米；新增深度关键帧：{summary['new_frames']}
- 动作：{summary['actions']}；未成功动作：{summary['failed_actions']}。取消或预算中断计入未成功，原因见 actions.csv。
- 高度 RMSE（米）：{summary['height_rmse_m']}；坡度 MAE（度）：{summary['slope_mae_deg']}。None 表示未测；参考状态：{summary['reference_status']}。

![采样对比](sampling-comparison.png)
![进度曲线](progress.png)
![采集地图](collected-map.png)

scene-*.jpg 为异步 Gazebo 现场截图；checkpoint-*.json 记录抓取时间及画面缺失。
现场图片检查：{summary['scene_images']} 张，{summary['scene_distinct_images']} 个不同画面；{summary['scene_warning'] or '画面有变化，但不证明帧级同步'}。
observations.npz 是原始深度与位姿档案，可能包含本任务之前的观测。任务时间窗口和开始基线见 manifest.json/telemetry.csv；不能把整个档案当成此次新增成果。
mission.json/actions.csv 保留决策及失败；source.zip 与 survey_yard.sdf 保存实际代码和场景。
CSV 可导入 Excel，PNG 可插入论文，PDF 图可用于排版。hashes.json 用于核验文件完整性。

Q 只是采样充分度。观测投影面积可能包含障碍表面，不能解释为可通行地面覆盖率；空白是未观测。
路径来自定位采样，含定位漂移；未做轮速里程真值验证。单次实验不构成算法优越性证据。
''',encoding='utf-8')
    manifest['report_status']='ready';write_json(folder/'manifest.json',manifest)
    write_json(folder/'hashes.json',{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(folder.iterdir())
                                  if p.is_file() and p.name not in ('hashes.json','report.log')})
    return summary


def summarize(root, output):
    results=[]
    for path in sorted(Path(root).rglob('manifest.json')):
        if path.parent.name.startswith('autonomy-'):
            try:
                result=json.loads((path.parent/'summary.json').read_text(encoding='utf-8'))
            except (OSError,ValueError):
                manifest=json.loads(path.read_text(encoding='utf-8'))
                result=dict(run_id=manifest['run_id'],status='unreported',capture_status=manifest['status'])
            result['folder']=str(path.parent.resolve());results.append(result)
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    write_json(output/'runs.json',results)
    write_csv(output/'runs.csv',[{k:v for k,v in result.items() if not isinstance(v,(dict,list))} for result in results])
    (output/'README.md').write_text('# 实验总表\n\n见 runs.csv。保留失败、中断和未生成报告的实验，不自动筛除。不同评价范围、开始基线、时间预算或代码版本的实验不可直接合并求平均。\n',encoding='utf-8')
    print(f'{len(results)} runs -> {output}')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('report');p.add_argument('folder',type=Path);p.add_argument('--reference',type=Path)
    p=sub.add_parser('summarize');p.add_argument('root',type=Path);p.add_argument('output',type=Path)
    args=parser.parse_args()
    if args.command=='report':
        try: report(args.folder,args.reference)
        except Exception as error:
            manifest=json.loads((args.folder/'manifest.json').read_text(encoding='utf-8'))
            manifest.update(report_status='failed',report_error=str(error));write_json(args.folder/'manifest.json',manifest)
            raise
        summarize(args.folder.parent.parent.parent, args.folder.parent.parent.parent/'research-summary')
        if (args.folder/'survey_yard.sdf').exists():
            # Reference diagnostics are separate from the frozen capture and never feed planning.
            destination=args.folder.parent/'reference-analysis'/(args.folder.name+'-'+uuid.uuid4().hex[:8])
            try:
                from terrain_reference import analyse
                analyse(args.folder,destination)
                print(f'Nominal-alignment diagnostics (not accuracy acceptance): {destination}')
            except Exception as error:
                destination.parent.mkdir(parents=True,exist_ok=True)
                write_json(destination.with_suffix('.failure.json'),dict(status='failed',error=str(error),source=str(args.folder)))
                print(f'Reference diagnostics failed; original report preserved: {error}',file=sys.stderr)
    else:summarize(args.root,args.output)
