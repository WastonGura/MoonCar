"""Verify frozen inputs, overlapping subset counts, fixed rules and surface limits."""
import csv
import hashlib
import json
from pathlib import Path

root=Path(__file__).resolve().parent;repo=root.parents[3]
read=lambda name:json.loads((root/name).read_text(encoding='utf-8'))
def rows(name):
    with (root/name).open(encoding='utf-8-sig',newline='') as handle:return list(csv.DictReader(handle))
for file in root.rglob('hashes.json'):
    for name,digest in json.loads(file.read_text()).items():
        path=(file.parent/name).resolve();assert path.is_relative_to(file.parent.resolve())
        assert hashlib.sha256(path.read_bytes()).hexdigest()==digest,name
for folder in ('attribution','ridge-model'):
    for name,digest in read(folder+'/summary.json')['input_sha256'].items():
        if name.startswith('/source/'):path=repo/name.removeprefix('/source/')
        else:
            assert name.startswith('/evidence/ridge-heldout-20260913/')
            path=root/'ridge-captures'/name.removeprefix('/evidence/ridge-heldout-20260913/')
        assert hashlib.sha256(path.read_bytes()).hexdigest()==digest,name
data=rows('attribution/cells.csv')
with (root.parent/'resampling-model-20260913/analysis/samples.csv').open(encoding='utf-8-sig',newline='') as handle:
    previous={(r['experiment'],r['ix'],r['iy']):r for r in csv.DictReader(handle)}
for row in data:
    old=previous[(row['experiment'],row['ix'],row['iy'])]
    assert row['full_recovered']==old['recovered']
    assert (row['false_negative']=='True')==(old['recovered']=='True' and old['distance_direction_prediction']=='False')
for group in rows('attribution/subsets.csv'):
    selected=[r for r in data if r['experiment']==group['experiment'] and (group['cohort']=='all_deficient' or r['false_negative']=='True')]
    assert len(selected)==int(group['cells'])
    assert sum(r[group['subset']+'_recovered']=='True' for r in selected)==int(group['recovered'])
short=[r for r in data if r['experiment']=='experiment-9d39b7482383' and r['false_negative']=='True']
assert len(short)==30
assert sum(r['transit_recovered']=='True' and r['settled_recovered']=='True' for r in short)==14
model=rows('ridge-model/samples.csv')
for group in rows('ridge-model/rule-check.csv'):
    selected=[r for r in model if r['experiment']==group['experiment'] and r['eligible']=='True'
              and (group['scope']=='all_initial_deficient' or r['candidate_in_fov']=='True')]
    assert len(selected)==int(group['eligible'])
    for key,predicted,label in (('tp','True','True'),('fp','True','False'),('fn','False','True'),('tn','False','False')):
        assert sum(r[group['rule']]==predicted and r['recovered']==label for r in selected)==int(group[key])
audit=read('ridge-visible-audit/summary.json')
assert not audit['accepted_accuracy']
assert next(g for g in audit['groups'] if g['category']=='curved_top')['observed']==0
for name,digest in audit['input_sha256'].items():
    assert hashlib.sha256((root/'ridge-captures/approach'/name).read_bytes()).hexdigest()==digest
for condition in ('stationary','approach'):
    control=read('ridge-captures/'+condition+'-control.json')
    assert control['status']=='completed' and control['capture_exit']==0
    if condition=='approach':assert control['motion_terminal_phase']=='succeeded'
assert read('attribution/summary.json')['trials'][0]['motion_start_pose_stamp'] is None
print('PASS: hashes, original labels, subset/overlap counts, two fresh captures, rule confusion matrices and zero pure curved-top evidence')
