import sys,json,math,hashlib,shutil
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from swept_resampling import snapshot,predict,report
from survey_evaluation import write_json,write_csv
from skip_probe import DELTAS
base=Path('/source/reports/evidence/terrain-workbench');out=Path('/evidence/measured-hold-development-20260913');out.mkdir(exist_ok=False)
rows=[];old_matches=0;zero=0;hold_old=0;hold_new=0
bundle=base/'skip-probe-20260913';plan=json.loads((bundle/'captures/plan.json').read_text())
for trial in plan['trials']:
    capture=bundle/'captures'/trial['folder']/'approach';meta=json.loads((capture/'checkpoint.json').read_text())
    selection=json.loads((capture/'selection.json').read_text());terrain,camera,robot=snapshot(capture,meta['frames'])
    quality=terrain.quality(True);cloud=terrain.cloud();yaw0=math.atan2(robot[1,0],robot[0,0])
    for delta,stored in zip(DELTAS,selection['candidates']):
        yaw=math.atan2(math.sin(yaw0+delta),math.cos(yaw0+delta));counts={}
        for version in ('v2','v4'):
            values,gate=predict(quality,cloud,camera,robot,yaw,meta['camera_info'],meta['pixel_offset'],
                meta['first_stamp']+60-meta['last_stamp'],gate_version='v3',model_version=version)
            counts[version]=dict(marginal=sum(r['marginal_prediction'] for r in values),hold=sum(r['hold_prediction'] for r in values),gate=gate['reason'])
        assert counts['v2']['marginal']==stored['marginal_cells'];old_matches+=1
        rows.append(dict(folder=trial['folder'],delta=delta,old_marginal=counts['v2']['marginal'],new_marginal=counts['v4']['marginal'],new_reason=counts['v4']['gate']))
        if delta==DELTAS[0]:hold_old+=counts['v2']['hold'];hold_new+=counts['v4']['hold']
    values,gate=predict(quality,cloud,camera,robot,yaw0,meta['camera_info'],meta['pixel_offset'],30,gate_version='v3',model_version='v4')
    assert all(not r['marginal_prediction'] for r in values);zero+=1
write_csv(out/'candidate-scan.csv',rows)
write_json(out/'summary.json',dict(development_only=True,legacy_candidates_reproduced=old_matches,
    zero_view_checks=zero,old_hold_predictions=hold_old,new_hold_predictions=hold_new,
    negative_candidates=[r for r in rows if r['new_reason']=='no_modelled_marginal_support'],
    online_ready=False,limits='Existing captures are development data. Zero-view checks are not nonzero negative experiments. Future views remain unoccluded horizontal-plane hypotheses.'))
for name in ('small-angle-20260913','swept-support-20260913'):
    report(base/name/'captures',out/name,'v3','v4')
for name in ('swept_resampling.py','audit_hold_support.py'):shutil.copyfile('/source/scripts/'+name,out/name)
print((out/'summary.json').read_text(),flush=True)
