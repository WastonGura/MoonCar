"""Recount rules from rows and verify original inputs; no ROS/dependencies required."""
import csv
import hashlib
import json
from pathlib import Path

root=Path(__file__).resolve().parent;repo=root.parents[3]
for manifest in root.rglob('hashes.json'):
    for name,digest in json.loads(manifest.read_text()).items():
        path=(manifest.parent/name).resolve()
        assert path.is_relative_to(manifest.parent.resolve())
        assert hashlib.sha256(path.read_bytes()).hexdigest()==digest,name
summary=json.loads((root/'analysis/summary.json').read_text())
assert not summary['features_use_truth'] and not summary['fitted_parameters'] and not summary['accepted_policy']
for name,digest in summary['input_sha256'].items():
    if name.startswith('/source/'):
        path=repo/name.removeprefix('/source/')
    else:
        assert name.startswith('/evidence/slope-short-approach-20260913/')
        path=root/'short-approach'/name.removeprefix('/evidence/slope-short-approach-20260913/')
    assert hashlib.sha256(path.read_bytes()).hexdigest()==digest,name
with (root/'analysis/samples.csv').open(newline='',encoding='utf-8-sig') as handle:rows=list(csv.DictReader(handle))
with (root/'analysis/rule-check.csv').open(newline='',encoding='utf-8-sig') as handle:groups=list(csv.DictReader(handle))
for group in groups:
    selected=[r for r in rows if r['experiment']==group['experiment'] and r['eligible']=='True'
              and (group['scope']=='all_initial_deficient' or r['candidate_in_fov']=='True')]
    assert len(selected)==int(group['eligible'])
    for key,predicted,recovered in (('tp','True','True'),('fp','True','False'),('fn','False','True'),('tn','False','False')):
        assert sum(r[group['rule']]==predicted and r['recovered']==recovered for r in selected)==int(group[key])
    assert sum(r['recovered']=='' for r in selected)==int(group['missing'])
assert len(summary['trials'])==3
control=json.loads((root/'short-approach/approach-control.json').read_text())
assert control['capture_exit']==0 and control['motion_before_stop']['phase']=='succeeded'
assert control['motion_before_stop']['diagnostics']['quality_present']
print(f'PASS: hashes, three captures, {len(rows)} sensor-only cell rows and {len(groups)} confusion-matrix rows')
