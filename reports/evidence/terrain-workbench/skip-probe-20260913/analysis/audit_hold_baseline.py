"""Diagnostic held-observation baseline calibration, not a turn-effect experiment."""
import sys,json,math
from pathlib import Path
import numpy as np
sys.path.insert(0,'/source/scripts')
from swept_resampling import snapshot,predict
from survey_evaluation import write_csv,write_json
captures=Path(sys.argv[1]);output=Path(sys.argv[2]);plan=json.loads((captures/'plan.json').read_text());rows=[];trials=[]
for trial in plan['trials']:
    folder=captures/trial['folder'];capture=folder/'approach';path=folder/'approach-control.json'
    if not path.exists():continue
    control=json.loads(path.read_text());selection=control.get('skip_probe')
    if not selection or 'observe_turn_terminal' in control:continue
    meta=json.loads((capture/'checkpoint.json').read_text());event=control['observation_decision_sim_stamp']
    with np.load(capture/'samples.npz',allow_pickle=False) as data:
        stamps=data['stamp'];end=int(np.searchsorted(stamps,event+10,side='right'))
        if stamps[-1]<event+10:continue
        start=int(np.searchsorted(stamps,event,side='right'))-1
        drift={}
        for kind in ('estimated_robot','truth_robot'):
            a,b=data[kind][start],data[kind][end-1]
            yaw=math.atan2(b[1,0],b[0,0])-math.atan2(a[1,0],a[0,0])
            drift[kind]=dict(xy_m=float(np.linalg.norm(b[:2,3]-a[:2,3])),yaw_rad=math.atan2(math.sin(yaw),math.cos(yaw)))
    terrain,camera,robot=snapshot(capture,meta['frames']);base=math.atan2(robot[1,0],robot[0,0]);delta=plan['deltas'][0]
    yaw=math.atan2(math.sin(base+delta),math.cos(base+delta))
    values,gate=predict(terrain.quality(True),terrain.cloud(),camera,robot,yaw,meta['camera_info'],meta['pixel_offset'],30,gate_version='v3')
    final,_,_=snapshot(capture,end);actual={(c['ix'],c['iy']):c['fit_state'] for c in final.quality(True)['cells']}
    for value in values:
        state=actual.get((value['ix'],value['iy']),'missing')
        rows.append(dict(folder=trial['folder'],**value,held10_state=state,held10_recovered=state=='valid'))
    selected=[r for r in rows if r['folder']==trial['folder']]
    trials.append(dict(folder=trial['folder'],no_turn_command=True,first_candidate_delta=delta,window_seconds=10,
        deficient=len(values),hold_predicted=sum(r['hold_prediction'] for r in selected),
        hold_recovered=sum(r['held10_recovered'] for r in selected),
        missed_hold_recovery=sum(not r['hold_prediction'] and r['held10_recovered'] for r in selected),
        hypothetical_marginal=sum(r['marginal_prediction'] for r in selected),
        hypothetical_marginal_already_recovered_holding=sum(r['marginal_prediction'] and r['held10_recovered'] for r in selected),drift=drift))
write_csv(output/'hold-baseline-cells.csv',rows)
write_json(output/'hold-baseline-audit.json',dict(diagnostic_only=True,trials=trials,
    limits=['Only runs without a turn command. First candidate +.02 rad is hypothetical and not executed.',
    'Holding does not freeze noise, localization or physical settling; measured drift reported. Recovery while holding cannot identify causal marginal turn benefit.',
    'Uses sensor-only prefix for predictors. Gazebo truth is read only for post-hoc drift diagnostics. No threshold changed or online admission granted.']))
print(json.dumps(trials,indent=2))
