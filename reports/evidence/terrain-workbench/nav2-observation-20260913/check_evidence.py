"""Read-only verification of the saved Nav2 observation experiment bundle."""
import base64,csv,hashlib,json,math
from pathlib import Path
root=Path(__file__).resolve().parent
for name,digest in json.loads((root/'hashes.json').read_text()).items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
plan=json.loads((root/'captures/plan.json').read_text())
for name,digest in plan['source_sha256'].items():
    assert hashlib.sha256((root/'captures/source'/Path(name).name).read_bytes()).hexdigest()==digest,name
summary=json.loads((root/'analysis/summary.json').read_text())
assert len(summary['trials'])==4 and not summary['accepted_policy'] and not summary['fitted_parameters']
identifiers=set();good=0
for trial in summary['trials']:
    folder=root/'captures'/trial['folder'];control=json.loads((folder/'approach-control.json').read_text())
    identifiers.add(control['experiment'])
    if trial.get('status')=='failed':assert control['status']=='failed';continue
    query=json.loads((folder/control['nav2_plan_file']).read_text())
    assert query['experiment']==control['experiment']
    assert query['query_started_sim_stamp']<=query['query_completed_sim_stamp']<=control['action_started_sim_stamp']
    assert trial['baseline_last_stamp']<control['action_started_sim_stamp']
    assert len(query['path'])>=2 and query['observation_yaw']==0.
    grid=query['map_snapshot'];costs=base64.b64decode(grid['data']);res=grid['resolution'];ox,oy=grid['origin']
    for a,b in zip(query['path'],query['path'][1:]):
        count=max(1,math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/(res/2)))
        for i in range(count+1):
            x=a[0]+i/count*(b[0]-a[0]);y=a[1]+i/count*(b[1]-a[1]);ix=math.floor((x-ox)/res);iy=math.floor((y-oy)/res)
            assert 0<=ix<grid['width'] and 0<=iy<grid['height']
            assert costs[iy*grid['width']+ix]<99
    if control['observation_reached']:
        good+=1
        assert control['move_terminal']['phase']=='succeeded' and control['observe_turn_terminal']['phase']=='succeeded'
        assert abs(control['observation_yaw_error_rad'])<=.1
    else:assert not trial['observation_reached']
assert len(identifiers)==4
with (root/'analysis/samples.csv').open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
for g in summary['groups']:
    target='sloped_top' if 'ramp' in g['scene'] else 'curved_top'
    selected=[r for r in rows if r['experiment']==g['experiment'] and r['eligible']=='True' and (g['scope']=='all_initial_deficient' or r['baseline_surface']==target)]
    assert len(selected)==g['cells']
    for key,p,y in [('tp',True,True),('fp',True,False),('fn',False,True),('tn',False,False)]:
        assert sum((r[g['rule']]=='True')==p and (r['recovered']=='True')==y for r in selected if r['recovered'] in ('True','False'))==g[key]
assert (root/'aborted-batch/aborted-batch.json').exists()
print(f'PASS: hashes, frozen rules, four fresh attempts, pre-motion known-space Nav2 paths, {good}/4 observation headings reached, confusion matrices and retained aborted batch')
