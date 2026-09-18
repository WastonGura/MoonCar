"""Portable stdlib audit of frozen target-probe evidence and the recorded verdict."""
from pathlib import Path
import json,hashlib,csv
root=Path(__file__).resolve().parent
load=lambda p:json.loads((root/p).read_text(encoding='utf-8-sig'))
for path,digest in load('hashes.json').items():assert hashlib.sha256((root/path).read_bytes()).hexdigest()==digest,path
plan=load('captures/plan.json');result=load('analysis/summary.json');ids=set()
for path,digest in plan['source_sha256'].items():assert hashlib.sha256((root/'captures/source'/Path(path).name).read_bytes()).hexdigest()==digest
for trial in plan['trials']:
    folder='captures/'+trial['folder'];control=load(folder+'/approach-control.json');ids.add(control['experiment'])
    selected=load(folder+'/approach/selection.json');targets=load(folder+'/approach/target-cells.json')
    assert targets==control['frozen_targets'] and selected==control['skip_probe']
    assert targets['feature_stamp']<control['action_started_sim_stamp']
    assert selected['scope']=='initial_deficient_targets' and selected['support_model']=='v4'
    if selected['selected']:
        assert selected['selected']['deficient_cells']>0 and selected['selected']['marginal_cells']==0
    if trial['arm']=='no_turn':assert 'observe_turn_terminal' not in control
    for stem in ('preaction','checkpoint'):
        meta=load(folder+'/approach/'+stem+'.json')
        assert hashlib.sha256((root/folder/'approach'/(stem+'.npz')).read_bytes()).hexdigest()==meta['sha256']
assert len(ids)==4 and result['target_screen_passed']==(not result['rejections']) and not result['online_ready']
with (root/'analysis/cells.csv').open(encoding='utf-8-sig',newline='') as handle:rows=list(csv.DictReader(handle))
for p in result['pairs']:
    a={ (r['ix'],r['iy']):r for r in rows if r['folder']==p['pair']+'-turn'}
    b={ (r['ix'],r['iy']):r for r in rows if r['folder']==p['pair']+'-no_turn'}
    common=a.keys()&b.keys()
    if p['scope']=='shared_targets':common={k for k in common if a[k]['target']==b[k]['target']=='True'}
    extra=sum(a[k]['recovered']=='True' and b[k]['recovered']=='False' for k in common)
    hold=sum(b[k]['recovered']=='True' and a[k]['recovered']=='False' for k in common)
    assert (p['common'],p['turn_only'],p['hold_only'],p['net'])==(len(common),extra,hold,extra-hold)
diagnosis=load('diagnosis/summary.json')
assert diagnosis['hold_predictions']==14 and diagnosis['measured_replay_fit']==0
assert all(c['new_voxels_supported_by_depth']==0 for c in diagnosis['cells'])
print('PASS frozen sources/data, pre-move target timestamps, narrow scope, paired cohorts and preserved verdict')
