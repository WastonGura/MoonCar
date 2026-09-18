"""Sensor-only arrival recheck and event-aligned replay; no online policy change."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import numpy as np
from scipy.spatial.transform import Rotation
from resampling_dataset import TerrainMap,features,ROOT
from survey_evaluation import write_json,write_csv


def recheck(quality,camera,robot,yaw,remaining,keys=None):
    """Frozen opportunity rule on the arrival snapshot, not a recovery probability."""
    if not all(math.isfinite(v) for v in [*camera,*robot,yaw,remaining]) or abs(yaw)>math.pi:
        raise ValueError('Finite camera XYZ, robot XY/yaw, target yaw and remaining time required')
    angle=abs(math.atan2(math.sin(yaw-robot[2]),math.cos(yaw-robot[2])))
    seconds=angle/.5+3
    candidate=[robot[0]+.435*math.cos(yaw),robot[1]+.435*math.sin(yaw),camera[2]]
    eligible=[];opportunities=[]
    for cell in quality['cells']:
        key=(cell['ix'],cell['iy'])
        if keys is not None and key not in keys:continue
        if cell['fit_state'] not in ('too_few_voxels','insufficient_xy_spread'):continue
        eligible.append(key)
        if features(cell,quality['resolution'],camera,candidate,yaw)['distance_direction_prediction']:
            opportunities.append(key)
    reason=('already_aligned' if angle<=.12 else 'insufficient_budget' if remaining<seconds
            else 'no_predicted_remaining_recovery' if not opportunities else 'try_observation')
    # ponytail: existing horizontal FOV/range/bearing heuristic, no 3D occlusion or
    # calibrated recovery probability. Offline gate until false skips are measured.
    return dict(version='arrival-recheck-v1',turn=reason=='try_observation',reason=reason,
                remaining_deficient=len(eligible),opportunity_cells=sorted(opportunities),
                estimated_seconds=seconds,angle_rad=angle,remaining_seconds=remaining,
                meaning='Resampling-only suggestion; skipping this heading does not end exploration')


def replay(capture,control,yaw):
    if control.get('status')!='completed' or control.get('move_terminal',{}).get('phase')!='succeeded':
        raise ValueError('Requires completed capture and successful navigation')
    with np.load(capture/'samples.npz',allow_pickle=False) as data:
        points,offsets,poses,robots,stamps=(data[k] for k in ('points','offsets','estimated_camera','estimated_robot','stamp'))
    action=control['action_started_sim_stamp'];arrival=control['move_terminal']['sim_stamp']
    decision=control.get('observation_decision_sim_stamp',arrival)
    turn=control.get('observe_turn_terminal',{}).get('sim_stamp')
    seconds=json.loads((capture/'manifest.json').read_text())['seconds']
    if not stamps[0]<action<=arrival<=decision<stamps[-1] or turn is not None and not decision<=turn<=stamps[-1]:
        raise ValueError('Stage events must lie in the captured, monotonic window')
    cuts={'baseline':int(np.searchsorted(stamps,min(stamps[0]+5,action),side='left')),
          'preaction':int(np.searchsorted(stamps,action,side='right')),
          'arrival':int(np.searchsorted(stamps,arrival,side='right')),
          'decision':int(np.searchsorted(stamps,decision,side='right')),
          'post_turn':int(np.searchsorted(stamps,turn,side='right')) if turn is not None else None,
          'final':len(stamps)}
    if cuts['baseline']<1:raise ValueError('Empty baseline')
    terrain=TerrainMap();snapshots={};decisions={}
    for i,stamp in enumerate(stamps):
        terrain.add_frame(points[offsets[i]:offsets[i+1]],float(stamp),poses[i,:3,3],Rotation.from_matrix(poses[i,:3,:3]).as_quat())
        for name,count in cuts.items():
            if count!=i+1:continue
            quality=terrain.quality(with_surface=True)
            snapshots[name]={(c['ix'],c['iy']):c for c in quality['cells']}
            if name=='decision':
                robot=[*robots[i,:2,3],math.atan2(robots[i,1,0],robots[i,0,0])]
                focus={k for k,c in snapshots['baseline'].items() if c['fit_state'] in ('too_few_voxels','insufficient_xy_spread')}
                # Only observations at/before the decision and the declared budget.
                decisions['focus']=recheck(quality,poses[i,:3,3],robot,yaw,max(0,stamps[0]+seconds-decision),focus)
                decisions['all_observed']=recheck(quality,poses[i,:3,3],robot,yaw,max(0,stamps[0]+seconds-decision))
    return snapshots,dict(frames=len(stamps),stage_frames=cuts,
        stage_last_stamps={name:float(stamps[count-1]) if count else None for name,count in cuts.items()},
        action_stamp=action,arrival_event_stamp=arrival,decision_event_stamp=decision,turn_event_stamp=turn,decisions=decisions)


def transitions(before,after,keys):
    a={k for k in keys if before.get(k,{}).get('fit_state')=='valid'}
    b={k for k in keys if after.get(k,{}).get('fit_state')=='valid'}
    return dict(cells=len(keys),valid_before=len(a),valid_after=len(b),
                gained=len(b-a),lost=len(a-b),net=len(b)-len(a))


def report(bundle,output):
    output.mkdir(parents=True,exist_ok=False)
    plan=json.loads((bundle/'captures/plan.json').read_text())
    with (bundle/'analysis/samples.csv').open(encoding='utf-8-sig',newline='') as f:labels=list(csv.DictReader(f))
    rows=[];groups=[];trials=[];hashes={}
    for trial in plan['trials']:
        folder=bundle/'captures'/trial['folder'];control=json.loads((folder/'approach-control.json').read_text())
        if control.get('status')!='completed' or control.get('move_terminal',{}).get('phase')!='succeeded':
            trials.append(dict(folder=trial['folder'],status='excluded_execution_failure',control=control));continue
        states,meta=replay(folder/'approach',control,trial['observation_yaw'])
        selected=[r for r in labels if r['experiment']==control['experiment'] and r['eligible']=='True']
        for r in selected:
            key=(int(r['ix']),int(r['iy']))
            assert (states['final'][key]['fit_state']=='valid')==(r['recovered']=='True'),'Frozen final label changed'
            rows.append(dict(folder=trial['folder'],scene=trial['scene'],ix=key[0],iy=key[1],baseline_surface=r['baseline_surface'],
                **{name:c.get(key,{}).get('fit_state','missing') for name,c in states.items()}))
        target='sloped_top' if 'ramp' in trial['scene'] else 'curved_top'
        for scope in ('initial_deficient','pure_target'):
            keys={(int(r['ix']),int(r['iy'])) for r in selected if scope=='initial_deficient' or r['baseline_surface']==target}
            stages=list(states)
            for before,after in zip(stages,stages[1:]):
                groups.append(dict(folder=trial['folder'],scope=scope,stage=after,**transitions(states[before],states[after],keys)))
            groups.append(dict(folder=trial['folder'],scope=scope,stage='after_arrival_total',**transitions(states['arrival'],states['final'],keys)))
            groups.append(dict(folder=trial['folder'],scope=scope,stage='after_decision_total',**transitions(states['decision'],states['final'],keys)))
        trials.append(dict(folder=trial['folder'],scene=trial['scene'],experiment=control['experiment'],status='evaluated',**meta))
        for p in (folder/'approach/samples.npz',folder/'approach-control.json'):
            hashes[str(p.relative_to(bundle))]=hashlib.sha256(p.read_bytes()).hexdigest()
    write_csv(output/'cells.csv',rows);write_csv(output/'stages.csv',groups)
    write_json(output/'summary.json',dict(trials=trials,groups=groups,input_bundle=str(bundle),input_sha256=hashes,
        limits=['Cumulative replay at observed Nav2 completion events; stage gains are temporal associations, not isolated causal effects.',
        'Last synchronized observation at or before each event; exact sensor pose history is unchanged.',
        'Truth surface labels stratify evaluation only; gate uses estimated poses and sensor-only arrival snapshots.',
        'Fixed 35s protocol, no probability fitting, no online deployment. Gate addresses resampling, not unknown-area coverage.']))
    for p in (Path(__file__),ROOT/'scripts/resampling_dataset.py',ROOT/'ros2_ws/src/mooncar_terrain/scripts/terrain_map.py'):
        shutil.copyfile(p,output/p.name)
    print(json.dumps(dict(trials=trials,groups=groups),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bundle',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();report(a.bundle,a.output)
