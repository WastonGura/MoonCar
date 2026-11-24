"""Portable stdlib evidence checker; a rejected model can still have valid evidence."""
from pathlib import Path
import base64,csv,hashlib,json,math
root=Path(__file__).resolve().parent
read=lambda p:json.loads((root/p).read_text(encoding='utf-8-sig'))
for name,digest in read('hashes.json').items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
plan=read('captures/plan.json');summary=read('analysis/summary.json');assessment=read('analysis/assessment.json')
assert len(plan['trials'])==8 and not summary['online_policy_changed'] and not assessment['online_policy_changed']
for name,digest in plan['source_sha256'].items():assert hashlib.sha256((root/'captures/source'/Path(name).name).read_bytes()).hexdigest()==digest,name
for name,digest in summary['input_sha256'].items():assert hashlib.sha256((root/'captures'/name).read_bytes()).hexdigest()==digest,name
ids=set()
for trial in plan['trials']:
    folder=root/'captures'/trial['folder'];path=folder/'approach-control.json'
    if not path.exists():
        assert (root/'captures'/(trial['folder']+'-failure.json')).exists()
        continue
    control=json.loads(path.read_text());ids.add(control['experiment'])
    saved=json.loads((folder/'plan.json').read_text())
    assert saved['start']==trial['start'] and saved['goal']==trial['goal'] and saved['seconds']==60 and saved['arrival_wait']==5 and saved['skip_probe']
    if trial['arm']=='no_turn':assert 'observe_turn_terminal' not in control
    selection=control.get('skip_probe')
    if selection:
        assert selection==json.loads((folder/'approach/selection.json').read_text())
        assert hashlib.sha256((folder/'approach/checkpoint.npz').read_bytes()).hexdigest()==selection['checkpoint_sha256']
        candidates=selection['candidates']
        assert [c['delta'] for c in candidates]==plan['deltas'][:len(candidates)]
        selected=selection['selected']
        if selected:
            assert selected==candidates[-1] and selected['deficient_cells']>0 and selected['marginal_cells']==0
            assert selected['gate']['reason']=='no_modelled_marginal_support'
            assert all(not(c['deficient_cells'] and c['gate']['reason']=='no_modelled_marginal_support') for c in candidates[:-1])
        if selection['executable']:
            assert selected and 0<=selection['feature_age_seconds']<=5 and selection['remaining_seconds']>=8
        assert selection['feature_stamp']<=selection['completed_sim_stamp']<=control['observation_decision_sim_stamp']
    if control.get('move_terminal',{}).get('phase')!='succeeded':continue
    query=json.loads((folder/control['nav2_plan_file']).read_text())
    assert query['query_completed_sim_stamp']<=control['action_started_sim_stamp']
    grid=query['map_snapshot'];cost=base64.b64decode(grid['data']);ox,oy=grid['origin'];res=grid['resolution']
    for a,b in zip(query['path'],query['path'][1:]):
        n=max(1,math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/(res/2)))
        for i in range(n+1):
            ix=math.floor((a[0]+(b[0]-a[0])*i/n-ox)/res);iy=math.floor((a[1]+(b[1]-a[1])*i/n-oy)/res)
            assert 0<=ix<grid['width'] and 0<=iy<grid['height'] and cost[iy*grid['width']+ix]<99
with (root/'analysis/cells.csv').open(encoding='utf-8-sig',newline='') as handle:rows=list(csv.DictReader(handle))
for p in summary['pairs']:
    if p['status']!='evaluated':continue
    arms={a:{(r['ix'],r['iy']):r for r in rows if r['pair']==p['pair'] and r['arm']==a and (p['scope']=='all_decision_deficient' or r['initial_deficient']=='True')} for a in ('turn','no_turn')}
    common=arms['turn'].keys()&arms['no_turn'].keys();known=[k for k in common if all(arms[a][k]['final_state']!='missing' for a in arms)]
    extra=sum(arms['turn'][k]['recovered']=='True' and arms['no_turn'][k]['recovered']=='False' for k in known)
    other=sum(arms['no_turn'][k]['recovered']=='True' and arms['turn'][k]['recovered']=='False' for k in known)
    assert p['common_remaining']==len(common) and p['turn_only']==extra and p['hold_only']==other and p['net_recovered']==extra-other
assert assessment['stop_validation_passed']==(not assessment['rejections'])
print('PASS frozen inputs/sources, prospective ordered selection, event boundary, known-space route, retained failures and paired counts')
