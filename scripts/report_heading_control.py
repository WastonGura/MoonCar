"""Paired, same-budget heading controls; descriptive outcomes, not a policy claim."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np
from resampling_dataset import TerrainMap
from survey_evaluation import write_json,write_csv


def compare(no_turn,turn,target=None,exact=False):
    def select(rows):
        return {(r['ix'],r['iy']):r for r in rows if r['eligible']=='True'
                and (target is None or r['baseline_surface']==target)}
    a,b=select(no_turn),select(turn);keys=a.keys()&b.keys()
    fields=('initial_state','initial_points','initial_spatial_coverage','initial_view_directions')
    matching={k for k in keys if all(a[k][f]==b[k][f] for f in fields)}
    if exact:keys=matching
    known={k for k in keys if a[k]['recovered'] in ('True','False') and b[k]['recovered'] in ('True','False')}
    na=sum(a[k]['recovered']=='True' for k in known);nb=sum(b[k]['recovered']=='True' for k in known)
    return dict(common_cells=len(keys),matched_initial_summary_cells=len(matching),missing_pairs=len(keys-known),
                no_turn_recovered=na,turn_recovered=nb,net_recovered=nb-na,
                turn_only=sum(a[k]['recovered']=='False' and b[k]['recovered']=='True' for k in known),
                no_turn_only=sum(a[k]['recovered']=='True' and b[k]['recovered']=='False' for k in known),
                no_turn_rate=na/len(known) if known else None,turn_rate=nb/len(known) if known else None)


def report(root,analysis):
    plan=json.loads((root/'plan.json').read_text())
    with (analysis/'samples.csv').open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
    trials=[];pairs=[]
    for trial in plan['trials']:
        folder=root/trial['folder'];control=json.loads((folder/'approach-control.json').read_text())
        saved=json.loads((folder/'plan.json').read_text())
        assert saved['seconds']==trial['seconds']==35 and saved['baseline_seconds']==5
        assert saved['observe_turn']==(trial['arm']=='turn')
        result=dict(scene=trial['scene'],folder=trial['folder'],arm=trial['arm'],experiment=control['experiment'],
                    capture_status=control['status'],navigation=control.get('move_terminal',{}).get('phase'),
                    turn=control.get('observe_turn_terminal',{}).get('phase'),error=control.get('error'))
        if control['status']=='completed':
            with np.load(folder/'approach/samples.npz',allow_pickle=False) as data:
                first,last=map(float,(data['stamp'][0],data['stamp'][-1]))
                poses=data['truth_robot'];yaw=math.atan2(poses[-1,1,0],poses[-1,0,0])
                result.update(frames=len(data['stamp']),captured_span_sim_seconds=last-first,
                    action_window_sim_seconds=last-control['action_started_sim_stamp'],final_truth_yaw_deg=math.degrees(yaw),
                    travel_m=float(np.linalg.norm(np.diff(poses[:,:2,3],axis=0),axis=1).sum()))
            move=control.get('move_terminal',{}).get('sim_stamp');turned=control.get('observe_turn_terminal',{}).get('sim_stamp')
            result.update(turn_sim_seconds=turned-move if turned is not None and move is not None else None,
                          hold_sim_seconds=last-(turned if turned is not None else move) if move is not None else None)
            terrain=TerrainMap.load(analysis/(trial['folder']+'-approach')/'estimated-map.npz')
            valid=sum(c['valid'] for c in terrain.elevation())
            selected=[r for r in rows if r['folder']==trial['folder'] and r['eligible']=='True']
            result.update(final_valid_cells=valid,final_valid_area_m2=valid*terrain.resolution**2,
                          initial_deficient=len(selected),recovered=sum(r['recovered']=='True' for r in selected))
        trials.append(result)
    for scene in sorted({t['scene'] for t in plan['trials']}):
        arms={t['arm']:t for t in trials if t['scene']==scene}
        if set(arms)!={'turn','no_turn'} or any(t['capture_status']!='completed' for t in arms.values()):continue
        a=[r for r in rows if r['folder']==arms['no_turn']['folder']];b=[r for r in rows if r['folder']==arms['turn']['folder']]
        for scope,target,exact in [('common_initial_deficient',None,False),
             ('pure_target_common_initial_deficient','sloped_top' if 'ramp' in scene else 'curved_top',False),
             ('matched_initial_summary',None,True)]:
            pairs.append(dict(scene=scene,scope=scope,**compare(a,b,target,exact)))
    write_csv(analysis/'heading-trials.csv',trials);write_csv(analysis/'heading-pairs.csv',pairs)
    write_json(analysis/'heading-control.json',dict(trials=trials,pairs=pairs,accepted_policy=False,
        limits=plan['limits']+['Matching initial summaries does not make raw observations identical; paired cells are spatially correlated.',
        'Valid plane-fit area is neither height accuracy nor safe traversable area. Final yaw and travel use evaluator-only true poses.']))
    print(json.dumps(dict(trials=trials,pairs=pairs),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('analysis',type=Path)
    a=p.parse_args();report(a.root,a.analysis)
