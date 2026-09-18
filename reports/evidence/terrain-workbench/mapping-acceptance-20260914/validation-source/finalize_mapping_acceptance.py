"""One-off, reproducible delivery audit; does not operate the simulator."""
import csv
import json
import math
from pathlib import Path
import sys
sys.path.insert(0,'/source/scripts')
from report_mapping_acceptance import evaluate,read_json,digest
from mapping_acceptance import score
from survey_evaluation import write_json,write_csv
from terrain_map import TerrainMap

root=Path('/source/reports/evidence/terrain-workbench/mapping-acceptance-20260914')
output=Path('/evidence/mapping-acceptance-delivery-20260914')
output.mkdir(exist_ok=False)
for source,destination in [('capture','report-unstarted'),('capture-motion','report-motion')]:
    result=evaluate(root/source/'approach',root/'reference',output/destination)
    print(destination,{k:result[k] for k in ('status','S','failed_gates')},flush=True)
    with (output/destination/'cells.csv').open() as f:cells=list(csv.DictReader(f))
    eligible=[c for c in cells if c['eligible']=='True']
    valid=[c for c in eligible if c['valid']=='True'];good=[c for c in eligible if c['qualified']=='True']
    assert math.isclose(result['S'],100*len(good)/len(eligible))
    assert math.isclose(result['height_rmse_m'],math.sqrt(sum(float(c['height_error_m'])**2 for c in valid)/len(valid)))
    assert len(good)==result['qualified_cells'] and len(valid)==result['estimated_cells']

reference=read_json(root/'reference/reference.json');result=read_json(output/'report-motion/summary.json')
model=TerrainMap.load(output/'report-motion/observations.npz');original=model.elevation()
variants={'real_capture':original,'synthetic_height_plus_20cm':[dict(c,z=c['z']+.2 if c['z'] is not None else None) for c in original],
          'synthetic_remove_alternating_cells':original[::2],'synthetic_empty':[]}
checks=[]
for name,cells in variants.items():
    _,value=score(reference,cells,evidence=result['evidence'])
    checks.append(dict(case=name,**{k:value[k] for k in ('C','P','S','height_rmse_m','status')}))
assert checks[1]['S']==0 and checks[1]['P']==0
assert checks[2]['S']<checks[0]['S'] and checks[3]['S']==0 and checks[3]['P'] is None
_,missing=score(reference,original,evidence=dict(result['evidence'],coordinate_contract_verified=False))
assert missing['S'] is None and missing['status']=='not_evaluable'
checks.append(dict(case='synthetic_unverified_coordinates',**{k:missing[k] for k in ('C','P','S','height_rmse_m','status')}))
write_csv(output/'validation-cases.csv',checks)
write_json(output/'validation.json',dict(status='passed',csv_scores_independently_recounted=True,
    all_valid_height_errors_in_rmse=True,perturbation_checks=checks,
    limitation='Artificial corruption validates evaluator response; it is not another rover strategy or independent physical experiment.'))
print(json.dumps(checks,ensure_ascii=False),flush=True)
