"""Verify copied evidence and keep incomplete runs out of numeric denominators."""
import csv
import hashlib
import json
from pathlib import Path

root=Path(__file__).resolve().parent
checked=0
for manifest in root.rglob('hashes.json'):
    for name,digest in json.loads(manifest.read_text(encoding='utf-8')).items():
        path=(manifest.parent/name).resolve()
        assert path.is_relative_to(manifest.parent.resolve()),name
        assert hashlib.sha256(path.read_bytes()).hexdigest()==digest,str(path)
        checked+=1
with (root/'comparison/comparison.csv').open(encoding='utf-8-sig',newline='') as handle:
    rows=list(csv.DictReader(handle))
measured=[r for r in rows if r['new_observed_area_m2']]
assert len(rows)==19 and len(measured)==12
assert sum(r['status']=='fault' for r in measured)==1
assert len({r['experiment'] for r in measured})==12
for start in ('[3.5, 3.0, 0.0]','[0.0, -3.0, 0.0]'):
    for strategy in ('nearest_view_v1','coverage_view_v1','surface_view_v1'):
        assert sum(r['start']==start and r['strategy']==strategy for r in measured)==2
for row in rows:
    if row['status']=='trial_failed':assert row['new_observed_area_m2']==''
core=json.loads((root/'comparison/core-source-check.json').read_text())
assert core['all_recorded_core_sources_equal'] and len(core['per_run'])==12
summary=json.loads((root/'slope-analysis/summary.json').read_text())
assert all(r['intervention_completed'] for r in summary['trials'])
assert summary['trials'][1]['true_path_m']>.5
for condition in ('stationary','approach'):
    analysis=json.loads((root/'slope-analysis'/condition/'summary.json').read_text())
    assert not analysis['accepted_accuracy']
    for name,digest in analysis['input_sha256'].items():
        assert hashlib.sha256((root/'slope-captures'/condition/name).read_bytes()).hexdigest()==digest
for row in summary['baseline_recovery']:
    assert 0<=row['recovered']<=row['eligible']
    if row['eligible']:assert row['recovery_fraction']==row['recovered']/row['eligible']
print(f'PASS: {checked} hashes; 19 retained attempts, 12 distinct measured runs, one runtime fault')
