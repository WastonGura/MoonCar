"""Verify frozen arrival evidence without ROS or NumPy; replay tests are separate."""
from pathlib import Path
import base64,csv,hashlib,json,math
root=Path(__file__).resolve().parent
read=lambda p:json.loads((root/p).read_text(encoding='utf-8-sig'))
for name,digest in read('hashes.json').items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
plan=read('captures/plan.json');summary=read('stages/summary.json');assessment=read('stages/gate-assessment.json')
assert len(plan['trials'])==8 and not assessment['online_policy_changed']
for name,digest in plan['source_sha256'].items():assert hashlib.sha256((root/'captures/source'/Path(name).name).read_bytes()).hexdigest()==digest,name
for name,digest in read('captures/assessment-protocol.json')['files'].items():assert hashlib.sha256((root/'captures/assessment-source'/name).read_bytes()).hexdigest()==digest,name
for name,digest in summary['input_sha256'].items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
correction=read('captures/assessment-correction.json')
render=read('stages/plot-correction.json')
assert hashlib.sha256((root/'stages/assess_arrival_gate.py').read_bytes()).hexdigest()==render['assessment_source_sha256']
assert hashlib.sha256((root/'stages/gate-assessment.json').read_bytes()).hexdigest()==render['assessment_sha256']
assert hashlib.sha256((root/'analysis/path_resampling_plot_version.py').read_bytes()).hexdigest()==render['path_plot_source_sha256']
assert hashlib.sha256((root/'captures'/correction['file']).read_bytes()).hexdigest()==correction['sha256']
ids=set()
for trial in plan['trials']:
    folder='captures/'+trial['folder'];control=read(folder+'/approach-control.json');saved=read(folder+'/plan.json');ids.add(control['experiment'])
    assert saved['start']==trial['start'] and saved['goal']==trial['goal'] and saved['seconds']==35 and saved['arrival_wait']==1
    assert saved['observe_turn']==(trial['arm']=='turn')
    if control['status']!='completed':continue
    q=read(folder+'/'+control['nav2_plan_file'])
    assert q['experiment']==control['experiment'] and q['query_started_sim_stamp']<=q['query_completed_sim_stamp']<=control['action_started_sim_stamp']
    if control.get('move_terminal',{}).get('phase')!='succeeded':continue
    assert control['observation_decision_sim_stamp']>=control['move_terminal']['sim_stamp']+1-1e-9
    if trial['arm']=='no_turn':assert 'observe_turn_terminal' not in control
    elif control.get('observation_reached'):
        assert control['observe_turn_terminal']['sim_stamp']>=control['observation_decision_sim_stamp'] and abs(control['observation_yaw_error_rad'])<=.1
    grid=q['map_snapshot'];cost=base64.b64decode(grid['data']);ox,oy=grid['origin'];res=grid['resolution']
    for a,b in zip(q['path'],q['path'][1:]):
        n=max(1,math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/(res/2)))
        for i in range(n+1):
            ix=math.floor((a[0]+(b[0]-a[0])*i/n-ox)/res);iy=math.floor((a[1]+(b[1]-a[1])*i/n-oy)/res)
            assert 0<=ix<grid['width'] and 0<=iy<grid['height'] and cost[iy*grid['width']+ix]<99
assert len(ids)==8
with (root/'stages/cells.csv').open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
for t in summary['trials']:
    if t['status']!='evaluated':continue
    assert t['stage_last_stamps']['decision']<=t['decision_event_stamp']
    stages=[n for n,count in t['stage_frames'].items() if count is not None]
    for g in [g for g in summary['groups'] if g['folder']==t['folder']]:
        before,after=('arrival','final') if g['stage']=='after_arrival_total' else ('decision','final') if g['stage']=='after_decision_total' else (stages[stages.index(g['stage'])-1],g['stage'])
        target='sloped_top' if 'ramp' in t['scene'] else 'curved_top'
        selected=[r for r in rows if r['folder']==t['folder'] and (g['scope']=='initial_deficient' or r['baseline_surface']==target)]
        gained=sum(r[before]!='valid' and r[after]=='valid' for r in selected);lost=sum(r[before]=='valid' and r[after]!='valid' for r in selected)
        assert len(selected)==g['cells'] and gained==g['gained'] and lost==g['lost'] and gained-lost==g['net']
for pair in assessment['pairs']:
    members={p['arm']:p for p in plan['trials'] if p['pair']==pair['pair']}
    target='sloped_top' if 'ramp' in pair['pair'] else 'curved_top'
    arms={arm:{(r['ix'],r['iy']):r for r in rows if r['folder']==p['folder'] and r['decision'] in ('too_few_voxels','insufficient_xy_spread') and (pair['scope']=='all_common_remaining' or r['baseline_surface']==target)} for arm,p in members.items()}
    a,b=arms['no_turn'],arms['turn'];keys=a.keys()&b.keys();known={k for k in keys if a[k]['final']!='missing' and b[k]['final']!='missing'}
    assert len(keys)==pair['common_remaining'] and len(keys-known)==pair['missing_pairs']
    assert sum(a[k]['final']=='valid' for k in known)==pair['no_turn_recovered'] and sum(b[k]['final']=='valid' for k in known)==pair['turn_recovered']
for t in assessment['trials']:
    control=read('captures/'+t['folder']+'/approach-control.json')
    assert t['observation_executed_ok']==(t['arm']=='no_turn' or control.get('observation_reached') is True)
    selected=[r for r in rows if r['folder']==t['folder'] and r['decision'] in ('too_few_voxels','insufficient_xy_spread')]
    assert t['remaining_deficient']==len(selected) and t['post_decision_recovered']==sum(r['final']=='valid' for r in selected)
assert assessment['shadow_check_passed']==(not assessment['rejections'])
assert read('aborted-batch/aborted-batch.json')['motion_started'] is False
aborted=read('aborted-batch/visible_ramp_v1-repeat1-no_turn/approach-control.json')
assert 'action_started_sim_stamp' not in aborted and aborted['motion_before_stop']['phase']=='idle'
print('PASS: hashes, eight unique fresh attempts, pre-action known-space routes, common settling interval, pre-decision snapshots, stage gains/losses, paired cohorts and retained pre-motion abort')
