"""Portable audit of the six-attempt closeout, including failed attempts."""
import hashlib
import json
import math
from pathlib import Path
import zipfile

root = Path(__file__).resolve().parent
load = lambda p: json.loads(p.read_text(encoding='utf-8'))
sha = lambda b: hashlib.sha256(b).hexdigest()
for path, digest in load(root/'hashes.json').items():
    assert sha((root/path).read_bytes()) == digest, path
protocol = load(root/'protocol.json')
installed = load(root/'installed-source-check.json')
assert installed['matched'] > 0
assert all('/tests/' in name for name in installed['missing']+installed['mismatched'])
with zipfile.ZipFile(root/'source-before.zip') as archive:
    for name, digest in protocol['source_sha256'].items():
        assert sha(archive.read(name)) == digest, name
trials = load(root/'comparison/trials.json')
plan = load(root/'comparison/plan.json')['trials']
assert len(trials) == len(plan) == protocol['attempts'] == 6
assert all(t[k] == p[k] for t,p in zip(trials,plan) for k in ('start','strategy','repeat','budget_seconds','radius'))
assert len({t['experiment'] for t in trials if 'experiment' in t}) == sum('experiment' in t for t in trials)
for start in protocol['starts']:
    assert sorted(t['strategy'] for t in trials if t['start']==start) == sorted(protocol['strategies'])
checked = 0
for trial in trials:
    assert trial['status'] != 'starting'
    if 'folder' not in trial:
        assert trial['status']=='trial_failed'
        continue
    folder = root/'comparison/runs'/f"{trial['index']:02d}-{Path(trial['folder']).name}"
    with zipfile.ZipFile(folder/'source.zip') as archive:
        for name in archive.namelist():
            assert sha(archive.read(name)) == protocol['source_sha256'][name], name
    if not (folder/'summary.json').exists():
        assert 'report_error' in trial
        continue
    summary = load(folder/'summary.json')
    assert summary == trial['summary']
    origin, radius = summary['origin'], summary['radius_m']
    def scoped(quality):
        r=quality['resolution']
        return {(c['ix'],c['iy']):c for c in quality['cells']
                if abs((c['ix']+.5)*r)<=25 and abs((c['iy']+.5)*r)<=25
                and math.hypot((c['ix']+.5)*r-origin[0],(c['iy']+.5)*r-origin[1])<=radius}
    before, after = load(folder/'quality-start.json'), load(folder/'quality-end.json')
    assert before['resolution']==after['resolution']
    a,b=scoped(before),scoped(after); area=before['resolution']**2
    valid=lambda cells:sum(c['fit_state']=='valid' for c in cells.values())
    assert summary['valid_surface_gain_m2'] == (valid(b)-valid(a))*area
    assert summary['new_observed_area_m2'] == (len(b)-len(a))*area
    checked += 1
state=load(root/'final-state.json')
assert state['phase']=='stopped' and not state['motion']['active'] and not state['autonomy']['active']
result=load(root/'conclusion.json')
assert result['orchestration_exit_code']==load(root/'orchestration-error.json')['exit_code']==1
assert result['offline_report_recovery']
complete=all(t.get('summary',{}).get('capture_status')=='captured' and t['status'] in
             ('budget_reached','no_candidates','distance_reached') for t in trials)
success=[]
for start in protocol['starts']:
    values={t['strategy']:t.get('summary',{}).get('valid_surface_gain_m2') for t in trials if t['start']==start}
    success.append(all(v is not None for v in values.values()) and
                   all(values['surface_view_v1']>values[s] for s in ('nearest_view_v1','coverage_view_v1')))
assert result['consistent_descriptive_advantage']==(complete and all(success))
assert not result['significance_established'] and not result['v5_validated'] and not result['automatic_followup']
print(f'PASS: six retained attempts; {checked} numerical reports; frozen source matches; area arithmetic; stopped')
