import sys,json,csv
from pathlib import Path
import numpy as np
sys.path.insert(0,'/source/scripts')
from arrival_resampling import replay
from resampling_dataset import features
from survey_evaluation import write_csv,write_json
root=Path('/evidence/arrival-validation-evaluation-20260913')
plan=json.loads((root/'captures/plan.json').read_text())
with (root/'stages/cells.csv').open(encoding='utf-8-sig') as f:labels={(r['folder'],int(r['ix']),int(r['iy'])):r['baseline_surface'] for r in csv.DictReader(f)}
rows=[]
for t in plan['trials']:
    folder=root/'captures'/t['folder'];control=json.loads((folder/'approach-control.json').read_text())
    if control.get('status')!='completed' or control.get('move_terminal',{}).get('phase')!='succeeded':continue
    states,meta=replay(folder/'approach',control,0.)
    with np.load(folder/'approach/samples.npz',allow_pickle=False) as data:
        i=meta['stage_frames']['decision']-1;camera=data['estimated_camera'][i,:3,3];robot=data['estimated_robot'][i,:2,3]
    for key,cell in states['decision'].items():
        if (t['folder'],*key) not in labels or cell['fit_state'] not in ('too_few_voxels','insufficient_xy_spread'):continue
        f=features(cell,.25,camera,[robot[0]+.435,robot[1],camera[2]],0.)
        after=states['final'].get(key,{})
        rows.append(dict(folder=t['folder'],arm=t['arm'],ix=key[0],iy=key[1],surface=labels[(t['folder'],*key)],
                         decision_state=cell['fit_state'],decision_points=cell['points'],final_state=after.get('fit_state','missing'),
                         final_points=after.get('points'),recovered=after.get('fit_state')=='valid',**f))
write_csv(root/'stages/decision-features.csv',rows)
misses=[r for r in rows if r['recovered'] and not r['distance_direction_prediction']]
groups=[]
for arm in ['turn','no_turn']:
    selected=[r for r in misses if r['arm']==arm]
    groups.append(dict(arm=arm,misses=len(selected),outside_candidate_fov=sum(not r['candidate_in_fov'] for r in selected),
                      insufficient_spread=sum(r['decision_state']=='insufficient_xy_spread' for r in selected),
                      too_few_voxels=sum(r['decision_state']=='too_few_voxels' for r in selected),
                      unchanged_voxel_count=sum(r['decision_points']==r['final_points'] for r in selected)))
write_json(root/'stages/miss-diagnostics.json',dict(groups=groups,limits=['Post-hoc explanation of frozen prediction errors, not fitted parameters or proof of why the sensor changed.','No-turn candidate heading is counterfactual; observed recovery does not establish a turn benefit.']))
print(json.dumps(groups,indent=2))
