"""Compare only completed rounds under the same frozen execution signature."""
import json,sys
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from r5_d0_analysis import compare_pair
from run_r5_origin_campaign import pair_inputs
p=Path('/evidence');folder=p/'campaign';ledger=json.loads((folder/'ledger.json').read_text())
def read(index):
    e=ledger[index]
    if not e.get('qualified'):raise ValueError('Incomplete/failed round cannot be paired')
    return json.loads((folder/e['name']/'result.json').read_text())
step=[dict(first=ledger[i]['name'],second=ledger[i+4]['name'],comparison=compare_pair(read(i),read(i+4))) for i in range(2,6)]
repeats=[dict(first=ledger[i]['name'],second=ledger[i+2]['name'],comparison=pair_inputs(read(i),read(i+2))) for i in (2,3,6,7)]
straight=pair_inputs(read(0),read(1))
report=dict(step_comparisons=step,repeat_inputs=repeats,straight_input=straight,
    all_completed_pairs_passed=all(x['comparison']['passed'] for x in step+repeats) and straight['passed'],
    matrix_passed=False,reason='Round11 startup failed; no static/scenes/restore completion; comparisons are not overall acceptance')
(p/'completed-pairs.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))

e=ledger[-1];r=json.loads((folder/e['name']/'result.json').read_text());receipt=folder/e['name']/'production-bridge/initialization.json'
failure=dict(name=e['name'],ledger_error=e.get('error'),primary_runtime_error=r.get('error'),
    sim_ended=r.get('sim_ended'),schedule_phases=r.get('schedule_phases'),
    initialization=json.loads(receipt.read_text()) if receipt.is_file() else None,
    readiness_receipt_present=(receipt.parent/'ready.json').is_file(),
    bridge_log=(folder/e['name']/'bridge.log').read_text(),
    causal_location_confirmed=False,interpretation='Runtime startup validation exited before physical stepping; exact ENODATA operation untraced',
    previous_successes_retained=True,not_a_physical_instability_conclusion=True)
(p/'failure-analysis.json').write_text(json.dumps(failure,indent=2)+'\n')
