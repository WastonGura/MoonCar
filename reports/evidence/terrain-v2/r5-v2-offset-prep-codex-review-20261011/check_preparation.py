"""Offline preparation review, including synthetic missing-data probes."""
import csv
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
PREP = ROOT.parent / 'r5-v2-offset-diagnosis-prep-20261011'
BASELINE = ROOT.parent / 'r5-turn-gate-validation-20261011/campaign/14-pure_turn-ramp-1000us-left'


def synthetic(folder, contact_mode):
    folder.mkdir(exist_ok=True)
    fields = ['iteration','t','x','y','z','qw','qx','qy','qz','com_world_x','com_world_y','total_mass']
    for wheel in module.WHEELS:
        for axis in 'xyz':
            fields += [wheel+'_world_'+axis,wheel+'_axis_'+axis]
    with (folder/'physics-steps.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for iteration in range(5201,5204):
            row=dict.fromkeys(fields,0.)
            row.update(iteration=iteration,t=iteration*.001,qw=1.,total_mass=9.186)
            for wheel in module.WHEELS:
                row[wheel+'_world_z']=.076;row[wheel+'_axis_y']=1.
            writer.writerow(row)
    fields=['iteration','wheel','state','nx','ny','nz','x','y','z','depth','f1x','f1y','f1z']
    with (folder/'synthetic-contact-steps.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for iteration in range(5201,5204):
            for wheel in range(4):
                if wheel==1 and contact_mode=='missing':
                    continue
                row=dict.fromkeys(fields,0.)
                row.update(iteration=iteration,wheel=wheel,state=1,nz=1.,f1z=3.7)
                if wheel==1 and contact_mode in ['explicit_empty','unknown']:
                    row.update(state=0 if contact_mode=='explicit_empty' else -1,
                               nx=float('nan'),ny=float('nan'),nz=float('nan'),depth=float('nan'),
                               f1x=float('nan'),f1y=float('nan'),f1z=float('nan'))
                writer.writerow(row)
    return module.evaluate_run(folder)


if __name__=='__main__':
    spec=importlib.util.spec_from_file_location('gemini_diagnosis_evaluator',PREP/'evaluate_diagnosis_run.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    baseline_result=module.evaluate_run(BASELINE)
    probes={}
    for mode in ['complete','missing','explicit_empty','unknown']:
        probes[mode]=synthetic(ROOT/('synthetic-'+mode),mode)
    collinear=module.check_segment_intersection(np.array([-.2,.05]),np.array([-.1,.15]),
                                               np.array([-.25,0.]),np.array([0.,.25]))
    plan=json.loads((PREP/'campaign_plan.json').read_text())
    originals=json.loads((BASELINE/'result.json').read_text())
    manifest=json.loads((PREP/'manifest.json').read_text())
    results={
        'scope':'Offline only. Synthetic fixtures are not physics evidence or acceptance rounds.',
        'gemini_manifest_verified':all(hashlib.sha256((PREP/k).read_bytes()).hexdigest()==v['sha256'] for k,v in manifest.items()),
        'baseline_evaluator_output':baseline_result,
        'output_keys':list(baseline_result),
        'missing_contact_probes':probes,
        'collinear_overlapping_segments':{'expected_overlap':True,'reported_overlap':bool(collinear[0])},
        'original_schedule':{'parked_before':originals['schedule']['parked']['before'],
                             'motion_before':originals['schedule']['motion']['before'],
                             'motion_after':originals['schedule']['motion']['after'],
                             'stop_after':originals['schedule']['stop']['after']},
        'declared_new_schedule':plan['runs'][0]['schedule'],
        'prep_files':[p.name for p in PREP.iterdir() if p.is_file()],
        'evaluator_sha256':hashlib.sha256((PREP/'evaluate_diagnosis_run.py').read_bytes()).hexdigest(),
    }
    (ROOT/'review-results.json').write_text(json.dumps(results,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(results,indent=2,allow_nan=False))
