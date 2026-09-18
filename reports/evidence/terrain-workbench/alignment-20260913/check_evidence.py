"""Recompute trajectory statistics and verify immutable capture / automatic-report artifacts."""
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np

root=Path(__file__).resolve().parent
for hashes in root.rglob('hashes.json'):
    for name,digest in json.loads(hashes.read_text(encoding='utf-8-sig')).items():
        path=hashes.parent/name
        assert path.resolve().is_relative_to(hashes.parent.resolve())
        assert hashlib.sha256(path.read_bytes()).hexdigest()==digest,(str(path),'hash')
folder=root/'trajectory-01'
manifest=json.loads((folder/'manifest.json').read_text())
summary=json.loads((folder/'summary.json').read_text())
assert manifest['status']=='captured' and summary['frames']==154
assert summary['accepted_global_accuracy'] is False and summary['correction_fitted'] is False
assert json.loads((folder/'drops.json').read_text())==[]
truth=[json.loads(line) for line in (folder/'truth.jsonl').read_text().splitlines()]
stamps=np.array([r['stamp'] for r in truth]);assert np.all(np.diff(stamps)>0)
with np.load(folder/'samples.npz',allow_pickle=False) as data:
    assert len(data['stamp'])==manifest['frames'] and np.all(np.diff(data['stamp'])>=.5-1e-8)
    assert data['offsets'][0]==0 and data['offsets'][-1]==len(data['points'])
    assert np.all(data['gap']<=.05) and np.isfinite(data['points']).all()
    np.testing.assert_allclose(data['truth_camera'],data['truth_robot']@np.array(manifest['camera_extrinsic']),atol=1e-12)
    xy=np.linalg.norm(data['estimated_robot'][:,:2,3]-data['truth_robot'][:,:2,3],axis=1)
    assert math.isclose(float(np.sqrt(np.mean(xy**2))),summary['xy_rmse_m'],abs_tol=1e-12)
    assert math.isclose(float(xy.max()),summary['xy_max_m'])
    positions=data['truth_robot'][:,:2,3]
    assert math.isclose(float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum()),summary['sampled_truth_path_m'])
    for i,stamp in enumerate(data['stamp']):
        index=int(np.searchsorted(stamps,stamp))
        if index<len(stamps) and abs(stamps[index]-stamp)<1e-8:
            expected=truth[index]['pose'][:3]
        else:
            assert 0<index<len(stamps) and stamps[index]-stamps[index-1]<=.05
            ratio=(stamp-stamps[index-1])/(stamps[index]-stamps[index-1])
            expected=(1-ratio)*np.array(truth[index-1]['pose'][:3])+ratio*np.array(truth[index]['pose'][:3])
        np.testing.assert_allclose(expected,data['truth_robot'][i,:3,3],atol=1e-10)
with (folder/'frames.csv').open(encoding='utf-8-sig',newline='') as handle:rows=list(csv.DictReader(handle))
for group in ('ground','interior'):
    count=sum(int(r[group+'_points']) for r in rows)
    assert count==summary[group+'_points']
    for mode in ('true_pose','map_pose'):
        rmse=math.sqrt(sum(int(r[group+'_points'])*float(r[mode+'_'+group+'_rmse_m'] or 0)**2 for r in rows)/count)
        assert math.isclose(rmse,summary[mode+'_'+group+'_rmse_m'],abs_tol=1e-12)
assert summary['ground_points']-summary['interior_points']==23713
mission=json.loads((root/'autonomy-c6626caf60/mission.json').read_text())
assert not mission['active'] and mission['phase']=='no_candidates'
assert len(mission['history'])==7 and all(h['success'] for h in mission['history'])
assert any(h['kind']=='advance' for h in mission['history'])
assert (root/'stopped-velocity.txt').read_text().count(': 0.0')==6
references=list((root/'reference-analysis').glob('*/summary.json'))
assert len(references)==1 and json.loads(references[0].read_text())['accepted_accuracy'] is False
print('PASS: synchronized raw truth, interpolation brackets, pose/mount matrices, weighted height metrics, automatic reports, hashes, and stopped velocity')
