"""Sensor-only, pre-action features and post-action fit labels; never fit a policy here."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/mooncar_terrain/scripts'))
from terrain_map import TerrainMap
from survey_evaluation import write_json,write_csv


def features(cell,resolution,camera,candidate,yaw):
    center=(np.array([cell['ix'],cell['iy']])+.5)*resolution
    a=np.asarray(camera)[:2]-center;b=np.asarray(candidate)[:2]-center
    r0,r1=float(np.linalg.norm(a)),float(np.linalg.norm(b))
    bearing=int((math.atan2(b[1],b[0])+math.pi)/(math.pi/4))%8
    new_bearing=not bool(cell['bearing_mask']&(1<<bearing))
    delta=math.atan2(math.sin(math.atan2(-b[1],-b[0])-yaw),math.cos(math.atan2(-b[1],-b[0])-yaw))
    visible=.6<=r1<=5.5 and abs(delta)<=math.pi/6
    eligible=cell['fit_state'] in ('too_few_voxels','insufficient_xy_spread')
    ratio=(r0/r1)**2 if r1>0 else None
    count_proxy=cell['points']*ratio if ratio is not None else None
    # ponytail: horizontal inverse-square density proxy ignores incidence, occlusion
    # and voxel saturation. Keep offline until held-out geometry supports calibration.
    closer=eligible and cell['fit_state']=='too_few_voxels' and r1<r0 and count_proxy is not None and count_proxy>=6
    return dict(initial_range_m=r0,candidate_range_m=r1,density_ratio_proxy=ratio,
                point_count_proxy=count_proxy,bearing_change_deg=math.degrees(abs(math.atan2(float(a[0]*b[1]-a[1]*b[0]),float(a@b)))),
                new_bearing=new_bearing,candidate_in_fov=visible,eligible=eligible,
                direction_prediction=bool(eligible and visible and new_bearing),
                distance_direction_prediction=bool(eligible and visible and (new_bearing or closer)))


def extract(capture,control,goal,split):
    if control.get('status')!='completed':raise ValueError('Capture failed; retain as failure, not training rows')
    with np.load(capture/'samples.npz',allow_pickle=False) as data:
        points,offsets,poses,stamps=(data[k] for k in ('points','offsets','estimated_camera','stamp'))
    cutoff=min(float(stamps[0]+5),control.get('action_started_sim_stamp',math.inf))
    cut=int(np.searchsorted(stamps,cutoff,side='left'))
    if not 0<cut<len(stamps):raise ValueError('Need pre-action baseline and a subsequent observation window')
    terrain=TerrainMap();baseline=None;cloud=None
    for i in range(len(stamps)):
        terrain.add_frame(points[offsets[i]:offsets[i+1]],float(stamps[i]),poses[i,:3,3],Rotation.from_matrix(poses[i,:3,:3]).as_quat())
        if i==cut-1:
            baseline=terrain.quality(with_surface=True);cloud=terrain.cloud()
    final={(c['ix'],c['iy']):c for c in terrain.quality(with_surface=True)['cells']}
    camera=poses[cut-1,:3,3]
    if control['condition']=='stationary':
        candidate=camera;yaw=math.atan2(poses[cut-1,1,2],poses[cut-1,0,2])
    else:
        start=control['start_pose'];yaw=math.atan2(goal[1]-start['y'],goal[0]-start['x'])
        candidate=np.array([goal[0]+.435*math.cos(yaw),goal[1]+.435*math.sin(yaw),camera[2]])
    indices=np.floor(cloud[:,:2]/terrain.resolution).astype(int);rows=[]
    for cell in baseline['cells']:
        key=(cell['ix'],cell['iy']);sample=cloud[np.all(indices==key,axis=1),:2].astype(float)
        spread=float(np.linalg.svd(sample-sample.mean(axis=0),compute_uv=False)[-1]/math.sqrt(len(sample))) if len(sample)>=2 else 0.
        after=final.get(key)
        rows.append(dict(experiment=control['experiment'],split=split,condition=control['condition'],
            ix=key[0],iy=key[1],initial_state=cell['fit_state'],initial_points=cell['points'],
            initial_quality=cell['quality'],initial_xy_spread_m=spread,
            initial_spatial_coverage=cell['coverage'],initial_view_directions=cell['view_directions'],
            **features(cell,terrain.resolution,camera,candidate,yaw),
            final_state=after['fit_state'] if after else 'missing',final_points=after['points'] if after else None,
            recovered=after['fit_state']=='valid' if after else None))
    return rows,dict(experiment=control['experiment'],split=split,frames=len(stamps),baseline_frames=cut,
                     baseline_last_stamp=float(stamps[cut-1]),action_stamp=control.get('action_started_sim_stamp'),
                     motion_phase=control.get('motion_before_stop',{}).get('phase'),planned_goal=goal,
                     initial_deficient=sum(r['eligible'] for r in rows))


def report(specification,output):
    spec=json.loads(specification.read_text());output.mkdir(parents=True,exist_ok=False)
    rows=[];trials=[];hashes={}
    for trial in spec['trials']:
        folder=Path(trial['folder']);condition=trial['condition'];capture=folder/condition
        control=json.loads((folder/f'{condition}-control.json').read_text())
        plan=json.loads((folder/'plan.json').read_text())
        for path in (capture/'samples.npz',folder/f'{condition}-control.json',folder/'plan.json'):
            hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
        try:
            values,meta=extract(capture,control,trial.get('goal',plan.get('goal')),trial['split'])
            rows.extend(values);trials.append(meta)
        except ValueError as error:trials.append(dict(**trial,error=str(error)))
    groups=[]
    for trial in trials:
        if 'experiment' not in trial:continue
        for scope in ('all_initial_deficient','candidate_fov'):
            selected=[r for r in rows if r['experiment']==trial['experiment'] and r['eligible'] and (scope=='all_initial_deficient' or r['candidate_in_fov'])]
            for rule in ('direction_prediction','distance_direction_prediction'):
                known=[r for r in selected if r['recovered'] is not None]
                tp=sum(r[rule] and r['recovered'] for r in known);fp=sum(r[rule] and not r['recovered'] for r in known)
                fn=sum(not r[rule] and r['recovered'] for r in known);tn=sum(not r[rule] and not r['recovered'] for r in known)
                groups.append(dict(experiment=trial['experiment'],split=trial['split'],scope=scope,rule=rule,
                    eligible=len(selected),missing=len(selected)-len(known),tp=tp,fp=fp,fn=fn,tn=tn,
                    precision=tp/(tp+fp) if tp+fp else None,recall=tp/(tp+fn) if tp+fn else None))
    write_csv(output/'samples.csv',rows);write_csv(output/'rule-check.csv',groups)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    selected=[g for g in groups if g['scope']=='all_initial_deficient']
    fig,ax=plt.subplots(figsize=(10,5),layout='constrained')
    for i,g in enumerate(selected):
        bottom=0
        for key,hatch,label in (('tp','','Detected recovery'),('fp','//','False opportunity'),('fn','..','Missed recovery')):
            ax.bar(i,g[key],bottom=bottom,color='white',edgecolor='black',hatch=hatch,label=label if i==0 else None)
            bottom+=g[key]
    labels=[('Direction' if g['rule']=='direction_prediction' else '+ Range')+'\n'+g['experiment'][-6:]
            +('\nProspective' if g['split'].startswith('prospective') else '\nExploratory') for g in selected]
    ax.set_xticks(range(len(selected)),labels,fontsize=8)
    ax.set_ylabel('Initial deficient cells');ax.set_title('Geometric opportunity rule: fit recovery, not map accuracy')
    ax.legend(fontsize=8);fig.supxlabel('Correlated cells, three trajectories in one scene; not independent statistical replicates.',fontsize=8)
    fig.savefig(output/'rule-check.png',dpi=180);fig.savefig(output/'rule-check.pdf');plt.close(fig)
    write_json(output/'summary.json',dict(trials=trials,input_sha256=hashes,features_use_truth=False,
        fitted_parameters=False,accepted_policy=False,limits=spec['limits']))
    shutil.copyfile(specification,output/'plan.json')
    for file in (Path(__file__),ROOT/'ros2_ws/src/mooncar_terrain/scripts/terrain_map.py',ROOT/'ros2_ws/src/mooncar_terrain/scripts/terrain_geometry.py'):
        shutil.copyfile(file,output/file.name)
    print(json.dumps(trials,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('plan',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();report(a.plan,a.output)
