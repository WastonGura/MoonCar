"""Read-only checks for the frozen heading control evidence, using Python stdlib."""
import base64,csv,hashlib,json,math
from pathlib import Path
root=Path(__file__).resolve().parent
for name,digest in json.loads((root/'hashes.json').read_text()).items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
plan=json.loads((root/'captures/plan.json').read_text())
for name,digest in plan['source_sha256'].items():
    assert hashlib.sha256((root/'captures/source'/Path(name).name).read_bytes()).hexdigest()==digest,name
summary=json.loads((root/'analysis/heading-control.json').read_text())
assert len(summary['trials'])==4 and not summary['accepted_policy']
ids=set()
for trial in plan['trials']:
    folder=root/'captures'/trial['folder'];control=json.loads((folder/'approach-control.json').read_text());saved=json.loads((folder/'plan.json').read_text())
    ids.add(control['experiment'])
    assert saved['start']==trial['start'] and saved['goal']==trial['goal']
    assert saved['seconds']==trial['seconds']==35 and saved['baseline_seconds']==5
    assert saved['observe_turn']==(trial['arm']=='turn')
    if control['status']!='completed':continue
    query=json.loads((folder/control['nav2_plan_file']).read_text())
    assert query['experiment']==control['experiment']
    assert query['query_started_sim_stamp']<=query['query_completed_sim_stamp']<=control['action_started_sim_stamp']
    grid=query['map_snapshot'];costs=base64.b64decode(grid['data']);res=grid['resolution'];ox,oy=grid['origin']
    for a,b in zip(query['path'],query['path'][1:]):
        count=max(1,math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/(res/2)))
        for i in range(count+1):
            ix=math.floor((a[0]+i/count*(b[0]-a[0])-ox)/res);iy=math.floor((a[1]+i/count*(b[1]-a[1])-oy)/res)
            assert 0<=ix<grid['width'] and 0<=iy<grid['height'] and costs[iy*grid['width']+ix]<99
    if trial['arm']=='no_turn':assert 'observe_turn_terminal' not in control and 'observation_reached' not in control
    elif control.get('observation_reached'):
        assert control['move_terminal']['phase']==control['observe_turn_terminal']['phase']=='succeeded'
        assert abs(control['observation_yaw_error_rad'])<=.1
assert len(ids)==4
with (root/'analysis/samples.csv').open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
for pair in summary['pairs']:
    chosen={}
    for arm in ['turn','no_turn']:
        folder=next(t['folder'] for t in summary['trials'] if t['scene']==pair['scene'] and t['arm']==arm)
        target='sloped_top' if 'ramp' in pair['scene'] else 'curved_top'
        chosen[arm]={(r['ix'],r['iy']):r for r in rows if r['folder']==folder and r['eligible']=='True' and (pair['scope']!='pure_target_common_initial_deficient' or r['baseline_surface']==target)}
    a,b=chosen['no_turn'],chosen['turn'];keys=a.keys()&b.keys()
    if pair['scope']=='matched_initial_summary':keys={k for k in keys if all(a[k][f]==b[k][f] for f in ['initial_state','initial_points','initial_spatial_coverage','initial_view_directions'])}
    assert len(keys)==pair['common_cells']
    known={k for k in keys if a[k]['recovered'] in ['True','False'] and b[k]['recovered'] in ['True','False']}
    assert len(keys-known)==pair['missing_pairs']
    assert sum(a[k]['recovered']=='True' for k in known)==pair['no_turn_recovered']
    assert sum(b[k]['recovered']=='True' for k in known)==pair['turn_recovered']
    assert pair['turn_recovered']-pair['no_turn_recovered']==pair['net_recovered']
print('PASS: source and artifact hashes, four fresh arms, equal declared budget, pre-motion known-space paths, turn/control distinction and paired recovery counts')
