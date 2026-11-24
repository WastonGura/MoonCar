"""Reproduce every prospective selection, including attempts without a candidate."""
import sys,json,hashlib
from pathlib import Path
import numpy as np
sys.path.insert(0,'/source/scripts')
from swept_resampling import snapshot
from skip_probe import select
from survey_evaluation import write_json
captures=Path(sys.argv[1]);output=Path(sys.argv[2]);plan=json.loads((captures/'plan.json').read_text());checks=[]
for trial in plan['trials']:
    folder=captures/trial['folder'];capture=folder/'approach';path=capture/'selection.json'
    if not path.exists():checks.append(dict(folder=trial['folder'],status='no_selection'));continue
    selection=json.loads(path.read_text());meta=json.loads((capture/'checkpoint.json').read_text());control=json.loads((folder/'approach-control.json').read_text())
    assert hashlib.sha256((capture/'checkpoint.npz').read_bytes()).hexdigest()==meta['sha256']==selection['checkpoint_sha256']
    with np.load(capture/'checkpoint.npz',allow_pickle=False) as prefix,np.load(capture/'samples.npz',allow_pickle=False) as full:
        assert set(prefix.files)=={'points','offsets','stamp','estimated_camera','estimated_robot'}
        for key in prefix.files:assert np.array_equal(prefix[key],full[key][:len(prefix[key])]),key
        assert len(prefix['stamp'])==meta['frames'] and prefix['stamp'][-1]==selection['feature_stamp']
    assert control['move_terminal']['sim_stamp']<=selection['feature_stamp']<=selection['completed_sim_stamp']<=control['observation_decision_sim_stamp']
    terrain,camera,robot=snapshot(capture,meta['frames'])
    chosen,candidates=select(terrain.quality(True),terrain.cloud(),camera,robot,meta['camera_info'],meta['pixel_offset'],
        max(0,meta['first_stamp']+trial['seconds']-meta['last_stamp']))
    assert chosen==selection['selected'] and candidates==selection['candidates'],trial['folder']
    checks.append(dict(folder=trial['folder'],status='verified',frames=meta['frames'],sensor_only=True,
        exact_final_prefix=True,selection_reproduced=True,selected=chosen is not None))
write_json(output/'prefix-audit.json',dict(checks=checks,meaning='All saved selections reproduced from exact sensor-only prefixes, including missing candidates. No later observations or truth inputs.'))
print('PREFIX AUDIT',len(checks),'attempts',sum(c['status']=='verified' for c in checks),'verified')
