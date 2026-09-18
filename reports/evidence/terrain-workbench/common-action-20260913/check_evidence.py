"""Portable audit of frozen data, action timing and common-cohort statistics."""
import json,csv,hashlib
from pathlib import Path
root=Path(__file__).resolve().parent
load=lambda p:json.loads((root/p).read_text(encoding='utf-8-sig'))
for path,digest in load('hashes.json').items():assert hashlib.sha256((root/path).read_bytes()).hexdigest()==digest,path
plan=load('captures/plan.json');summary=load('analysis/summary.json')
for name,digest in plan['source_sha256'].items():assert hashlib.sha256((root/'captures/source'/Path(name).name).read_bytes()).hexdigest()==digest,name
ids=set()
for row in summary['trials']:
    if row['status']!='evaluated':continue
    folder='captures/'+row['folder'];control=load(folder+'/approach-control.json');action=load(folder+'/approach/comparison.json')
    ids.add(control['experiment']);assert action==control['comparison']
    assert row['feature_stamp']<=row['decision_stamp'] and row['window_last_sample']<=row['decision_stamp']+20+1e-6
    assert abs(row['utility_m2']-row['new_observed_m2']-row['recovered_surface_m2'])<1e-10
    assert abs(row['utility_m2_per_sim_second']*20-row['utility_m2'])<1e-10
    assert abs(row['net_utility_m2']-row['utility_m2']+row['lost_valid_surface_m2'])<1e-10
    meta=load(folder+'/approach/manifest.json')
    assert meta['parameter_reads'][-1]['status']=='success' and meta['parameter_reads'][-1]['value']==meta['projection_pixel_offset']
    assert row['target_recovered_cells']+row['outside_target_recovered_cells']==row['resampling_recovered_cells']
    for stem in ('preaction','checkpoint'):
        meta=load(folder+'/approach/'+stem+'.json')
        assert hashlib.sha256((root/folder/'approach'/(stem+'.npz')).read_bytes()).hexdigest()==meta['sha256']
assert len(ids)==sum(t['status']=='evaluated' for t in summary['trials']) and not summary['online_ready']
assert summary['complete']==(len(summary['trials'])==len(plan['trials']) and all(t['status']=='evaluated' and t['execution_success'] and t['window_boundary_lag']<=1 for t in summary['trials']))
with (root/'analysis/cells.csv').open(encoding='utf-8-sig',newline='') as handle:rows=list(csv.DictReader(handle))
for pair in summary['paired']:
    members={arm:[r for r in rows if r['folder']==f"ramp-repeat{pair['repeat']}-{arm}"] for arm in ('hold','small_turn','wide_turn','advance')}
    def keys(arm,field):return {(r['ix'],r['iy']) for r in members[arm] if r[field]=='True'}
    old=set.union(*(keys(a,'before') for a in members));common=set.intersection(*(keys(a,'eligible') for a in members))
    arm=pair['arm'];new=keys(arm,'after')-old;recovered=keys(arm,'recovered')&common
    hn=keys('hold','after')-old;hr=keys('hold','recovered')&common
    assert (pair['shared_deficient'],pair['shared_new_cells'],pair['shared_recovered_cells'])==(len(common),len(new),len(recovered))
    assert pair['net_utility_vs_hold_m2']==(len(new)-len(hn)+len(recovered)-len(hr))*.0625
    valid=set.intersection(*(keys(a,'before_valid') for a in members))
    lost=len(keys(arm,'lost')&valid);held_lost=len(keys('hold','lost')&valid)
    assert pair['shared_valid']==len(valid) and pair['shared_lost']==lost
    assert pair['loss_adjusted_difference_m2']==pair['net_utility_vs_hold_m2']-(lost-held_lost)*.0625
for f in summary['forecasts']:
    assert f['model']=='common-known-surface-v5' and f['predicted_new_area_m2'] is None and f['total_action_rank'] is None
    assert f['known_net_m2']==(f['recovered_cells']-f['lost_cells'])*.0625
print('PASS frozen hashes, sensor prefix hashes, equal-time metrics, retained failures and shared cohorts')
