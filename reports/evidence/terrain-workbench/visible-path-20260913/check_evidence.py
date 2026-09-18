"""Verify this frozen evidence bundle with Python's standard library."""
from pathlib import Path
import csv,hashlib,json
root=Path(__file__).resolve().parent
frozen=json.loads((root/'hashes.json').read_text(encoding='utf-8'))
for name,digest in frozen.items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
plan=json.loads((root/'captures/plan.json').read_text())
for name,digest in plan['source_sha256'].items():
    assert hashlib.sha256((root/'captures/source'/Path(name).name).read_bytes()).hexdigest()==digest,name
summary=json.loads((root/'analysis/summary.json').read_text())
assert not summary['accepted_policy'] and not summary['fitted_parameters']
assert len(summary['trials'])==4
identifiers=set();worlds=set()
for trial in summary['trials']:
    base=root/'captures'/trial['scene'];condition=trial['condition']
    control=json.loads((base/f'{condition}-control.json').read_text())
    manifest=json.loads((base/condition/'manifest.json').read_text())
    assert control['status']=='completed' and manifest['status']=='captured'
    assert trial['baseline_frames']>0 and trial['baseline_last_stamp']<trial['action_stamp']
    identifiers.add(control['experiment'])
    worlds.add(hashlib.sha256((base/condition/'survey_yard.sdf').read_bytes()).hexdigest())
    audit=json.loads((root/'analysis'/f"{trial['scene']}-{condition}"/'visible-audit/summary.json').read_text())
    target='sloped_top' if 'ramp' in trial['scene'] else 'curved_top'
    assert next(g for g in audit['groups'] if g['category']==target)['observed']>0
assert len(identifiers)==4 and len(worlds)==2
with (root/'analysis/samples.csv').open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
for group in summary['groups']:
    target='sloped_top' if 'ramp' in group['scene'] else 'curved_top'
    selected=[r for r in rows if r['scene']==group['scene'] and r['condition']==group['condition'] and r['eligible']=='True' and (group['scope']=='all_initial_deficient' or r['baseline_surface']==target)]
    assert len(selected)==group['cells']
    known=[r for r in selected if r['recovered'] in ('True','False')]
    rule=group['rule']
    for key,predicted,recovered in [('tp',True,True),('fp',True,False),('fn',False,True),('tn',False,False)]:
        assert sum((r[rule]=='True')==predicted and (r['recovered']=='True')==recovered for r in known)==group[key]
    assert sum(group[k] for k in ('tp','fp','fn','tn'))+group['missing']==group['cells']
print('PASS: bundle hashes, predeclared sources, four fresh captures, two distinct worlds, real target visibility, baseline timing and all confusion matrices')
