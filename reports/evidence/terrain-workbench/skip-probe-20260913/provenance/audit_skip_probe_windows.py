"""Supplemental attribution audit; never changes the frozen primary admission."""
import sys,json
from pathlib import Path
import numpy as np
sys.path.insert(0,'/source/scripts')
from swept_resampling import snapshot
from survey_evaluation import write_json

captures=Path(sys.argv[1]);output=Path(sys.argv[2]);plan=json.loads((captures/'plan.json').read_text())
trials=[];states={}
for trial in plan['trials']:
    folder=captures/trial['folder'];path=folder/'approach-control.json'
    if not path.exists():continue
    control=json.loads(path.read_text())
    if not control.get('skip_probe',{}).get('executable'):continue
    capture=folder/'approach';event=control['observation_decision_sim_stamp']
    with np.load(capture/'samples.npz',allow_pickle=False) as data:
        stamps=data['stamp'];cuts=[int(np.searchsorted(stamps,t,side='right')) for t in (control['observation_feature_stamp'],event,event+10)]
        if not cuts[0] or stamps[-1]<event+10:
            trials.append(dict(folder=trial['folder'],status='insufficient_10s_window'));continue
        travel=float(np.linalg.norm(data['estimated_robot'][cuts[2]-1,:2,3]-data['estimated_robot'][cuts[1]-1,:2,3]))
    maps=[]
    for cut in cuts:
        terrain,_,_=snapshot(capture,cut)
        maps.append({(c['ix'],c['iy']):c['fit_state'] for c in terrain.quality(True)['cells']})
    eligible=lambda m:{k for k,v in m.items() if v in ('too_few_voxels','insufficient_xy_spread')}
    original=eligible(maps[0]);dispatch=eligible(maps[1]) & original
    states[trial['folder']]=(dispatch,{k for k in dispatch if maps[2].get(k)=='valid'})
    trials.append(dict(folder=trial['folder'],status='evaluated',feature_deficient=len(original),
        recovered_before_command=sum(maps[1].get(k)=='valid' for k in original),
        still_deficient_at_command=len(dispatch),recovered_next10s=len(states[trial['folder']][1]),
        estimated_xy_change_m=travel,window_seconds=10))
pairs=[]
for pair in sorted({t['pair'] for t in plan['trials']}):
    turn=states.get(pair+'-turn');hold=states.get(pair+'-no_turn')
    if turn is None or hold is None:continue
    common=turn[0]&hold[0];extra=(turn[1]-hold[1])&common;other=(hold[1]-turn[1])&common
    pairs.append(dict(pair=pair,common_dispatch_deficient=len(common),turn_only=len(extra),hold_only=len(other),net=len(extra)-len(other)))
write_json(output/'dispatch-window-audit.json',dict(trials=trials,pairs=pairs,
    meaning='Supplemental fixed 10s window from command; excludes recovery while computing selection. Does not alter primary predeclared admission. Estimated pose drift and independent reset trajectories remain confounds.'))
print(json.dumps(pairs,indent=2))
