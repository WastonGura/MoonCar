"""Validate frozen swept-support evidence with Python standard library only."""
from pathlib import Path
import base64,csv,hashlib,json,math
root=Path(__file__).resolve().parent
read=lambda p:json.loads((root/p).read_text(encoding='utf-8-sig'))
for name,digest in read('hashes.json').items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
plan=read('captures/plan.json');summary=read('analysis/summary.json');assessment=read('analysis/assessment.json')
assert len(plan['trials'])==8 and not summary['online_policy_changed'] and not assessment['online_policy_changed']
for name,digest in plan['source_sha256'].items():assert hashlib.sha256((root/'captures/source'/Path(name).name).read_bytes()).hexdigest()==digest,name
assert hashlib.sha256((root/'analysis/swept_resampling.py').read_bytes()).hexdigest()==plan['source_sha256']['scripts/swept_resampling.py']
assert hashlib.sha256((root/'captures/assessment_source.py').read_bytes()).hexdigest()==read('captures/assessment-protocol.json')['source_sha256']
for name,digest in summary['input_sha256'].items():assert hashlib.sha256((root/'captures'/name).read_bytes()).hexdigest()==digest,name
ids=set()
for trial in plan['trials']:
    folder='captures/'+trial['folder'];control=read(folder+'/approach-control.json');saved=read(folder+'/plan.json');ids.add(control['experiment'])
    assert saved['start']==trial['start'] and saved['goal']==trial['goal'] and saved['seconds']==35 and saved['arrival_wait']==5
    assert saved['observation_yaw']==trial['observation_yaw'] and saved['observe_turn']==(trial['arm']=='turn')
    assert saved['observation_delta']==trial['observation_delta']
    expected=control['observation_decision_pose']['yaw']+trial['observation_delta']
    assert abs(math.atan2(math.sin(expected-control['observation_target_yaw']),math.cos(expected-control['observation_target_yaw'])))<1e-9
    if control['status']!='completed':continue
    query=read(folder+'/'+control['nav2_plan_file'])
    assert query['experiment']==control['experiment'] and query['query_started_sim_stamp']<=query['query_completed_sim_stamp']<=control['action_started_sim_stamp']
    if control.get('move_terminal',{}).get('phase')!='succeeded':continue
    assert control['observation_decision_sim_stamp']>=control['move_terminal']['sim_stamp']+5-1e-9
    if trial['arm']=='turn' and control.get('observation_reached'):
        assert control['observe_turn_terminal']['phase']=='succeeded' and abs(control['observation_yaw_error_rad'])<=.1
    if trial['arm']=='no_turn':assert 'observe_turn_terminal' not in control
    grid=query['map_snapshot'];cost=base64.b64decode(grid['data']);ox,oy=grid['origin'];res=grid['resolution']
    for a,b in zip(query['path'],query['path'][1:]):
        n=max(1,math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/(res/2)))
        for i in range(n+1):
            ix=math.floor((a[0]+(b[0]-a[0])*i/n-ox)/res);iy=math.floor((a[1]+(b[1]-a[1])*i/n-oy)/res)
            assert 0<=ix<grid['width'] and 0<=iy<grid['height'] and cost[iy*grid['width']+ix]<99
assert len(ids)==8
for t in summary['trials']:
    if t['status']=='evaluated':assert t['last_feature_stamp']<=t['decision_stamp'] and t['stage_frames']['decision']<t['stage_frames']['final']
with (root/'analysis/cells.csv').open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
for r in rows:
    assert r['decision_state'] in ('too_few_voxels','insufficient_xy_spread')
    assert (r['recovered']=='True')==(r['final_state']=='valid')
    assert (r['marginal_prediction']=='True')==(r['swept_prediction']=='True' and r['hold_prediction']=='False')
    assert int(r['swept_points'])>=int(r['hold_points'])
for g in summary['metrics']:
    selected=[r for r in rows if r['arm']==g['arm'] and r['final_state']!='missing' and (g['scope']=='all_decision_deficient' or r['initial_deficient']=='True')]
    for key,predicted,recovered in [('tp',True,True),('fp',True,False),('fn',False,True),('tn',False,False)]:
        assert g[key]==sum((r[g['rule']]=='True')==predicted and (r['recovered']=='True')==recovered for r in selected),(g,key)
for p in summary['pairs']:
    if p['status']!='evaluated':continue
    arms={arm:{(r['ix'],r['iy']):r for r in rows if r['pair']==p['pair'] and r['arm']==arm and (p['scope']=='all_decision_deficient' or r['initial_deficient']=='True')} for arm in ('turn','no_turn')}
    common=arms['turn'].keys() & arms['no_turn'].keys()
    known=[k for k in common if all(arms[a][k]['final_state']!='missing' for a in arms)]
    extra=sum(arms['turn'][k]['recovered']=='True' and arms['no_turn'][k]['recovered']=='False' for k in known)
    hold=sum(arms['no_turn'][k]['recovered']=='True' and arms['turn'][k]['recovered']=='False' for k in known)
    assert p['common_remaining']==len(common) and p['missing']==len(common)-len(known)
    assert p['turn_only']==extra and p['hold_only']==hold and p['net_recovered']==extra-hold
assert assessment['shadow_check_passed']==(not assessment['rejections'])
assert all(t['gate']['version']=='swept-support-v3' for t in summary['trials'] if t['status']=='evaluated')
print('PASS: immutable sources and inputs, eight independent captures, known-space pre-action paths, event cutoff, confusion counts, distinct cohorts and matched pairs')
