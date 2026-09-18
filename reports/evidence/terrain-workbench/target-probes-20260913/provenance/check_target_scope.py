import sys,json,math
from pathlib import Path
import numpy as np
sys.path.insert(0,'/source/scripts')
from swept_resampling import snapshot,predict
from skip_probe import DELTAS
from survey_evaluation import write_json
base=Path('/source/reports/evidence/terrain-workbench/skip-probe-20260913/captures');trials=[]
for trial in json.loads((base/'plan.json').read_text())['trials']:
    folder=base/trial['folder'];capture=folder/'approach';control=json.loads((folder/'approach-control.json').read_text());meta=json.loads((capture/'checkpoint.json').read_text())
    with np.load(capture/'samples.npz',allow_pickle=False) as data:
        cut=int(np.searchsorted(data['stamp'],min(data['stamp'][0]+5,control['action_started_sim_stamp']),side='left'))
    initial,_,_=snapshot(capture,cut);keys={(c['ix'],c['iy']) for c in initial.quality(True)['cells'] if c['fit_state'] in ('too_few_voxels','insufficient_xy_spread')}
    terrain,camera,robot=snapshot(capture,meta['frames']);quality=terrain.quality(True)
    quality['cells']=[c for c in quality['cells'] if (c['ix'],c['iy']) in keys]
    yaw0=math.atan2(robot[1,0],robot[0,0]);candidates=[]
    for delta in DELTAS:
        yaw=math.atan2(math.sin(yaw0+delta),math.cos(yaw0+delta))
        rows,gate=predict(quality,terrain.cloud(),camera,robot,yaw,meta['camera_info'],meta['pixel_offset'],30,gate_version='v3',model_version='v4')
        candidates.append(dict(delta=delta,remaining_targets=len(rows),marginal=sum(r['marginal_prediction'] for r in rows),reason=gate['reason']))
    trials.append(dict(folder=trial['folder'],initial_targets=len(keys),candidates=candidates))
write_json(Path('/evidence/target-scope-development-20260913.json'),dict(development_only=True,
    scope='All cells deficient in the first five-second pre-move prefix, frozen by observation time; new transit deficits are separate targets',trials=trials))
for t in trials:print(t['folder'],t['initial_targets'],[(c['remaining_targets'],c['marginal']) for c in t['candidates']])
