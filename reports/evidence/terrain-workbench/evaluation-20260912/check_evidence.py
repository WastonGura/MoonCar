"""Verify frozen capture artifacts; no simulator or network required."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

root=Path(__file__).resolve().parent
sys.path.insert(0,str(root.parents[3]/'scripts'))
from survey_evaluation import metrics

results=[]
for folder in sorted(root.glob('autonomy-*')):
    manifest=json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
    summary=json.loads((folder/'summary.json').read_text(encoding='utf-8'))
    mission=json.loads((folder/'mission.json').read_text(encoding='utf-8'))
    assert manifest['status']=='captured' and manifest['report_status']=='ready'
    assert not manifest['capture_errors']
    for name,digest in json.loads((folder/'hashes.json').read_text(encoding='utf-8')).items():
        assert hashlib.sha256((folder/name).read_bytes()).hexdigest()==digest,name
    rows=[json.loads(line) for line in (folder/'telemetry.jsonl').read_text(encoding='utf-8').splitlines()]
    for label,row in [('start',rows[0]),('end',rows[-1])]:
        q=json.loads((folder/f'quality-{label}.json').read_text(encoding='utf-8'))
        score=metrics(q,mission['origin'],mission['radius'])
        assert abs(score['quality_area_m2']-row['quality_area_m2'])<1e-9
        assert score['roi_area_m2']==summary['roi_area_m2']
    assert abs(summary['quality_gain_m2']-(rows[-1]['quality_area_m2']-rows[0]['quality_area_m2']))<1e-9
    assert summary['height_rmse_m'] is None and summary['slope_mae_deg'] is None
    with np.load(folder/'observations.npz',allow_pickle=False) as data:
        assert data['stamps'][-1]>=rows[-1]['quality_stamp']
        assert int(np.searchsorted(data['stamps'],rows[-1]['quality_stamp'],side='right'))==summary['raw_evaluated_frames']
    for action in mission['history']:
        if 'prediction_residual_m2' in action:
            assert abs(action['prediction_residual_m2']-(action['predicted_view_gain']['quality_gain_m2']-action['expected_quality_gain_m2']))<1e-9
    for name in ('scene-start.jpg','scene-end.jpg','sampling-comparison.png','progress.png','collected-map.png'):
        assert (folder/name).stat().st_size>1000
    results.append(summary)
assert len(results)==3
assert sum(r['status']=='fault' for r in results)==2
budget=next(r for r in results if r['status']=='no_candidates')
assert budget['elapsed_seconds']<=budget['budget_seconds'] and budget['new_frames']>0
continuation=next(r for r in results if r['actions']==0)
assert continuation['initial_quality_area_m2']>0 and continuation['quality_gain_m2']==0
assert (root/'stopped-velocity.txt').read_text(encoding='utf-8-sig').count(': 0.0')==6
print('PASS: 3 complete evidence packs; fixed-scope gains, raw timestamps, hashes, failure retention and zero stopped velocity')
