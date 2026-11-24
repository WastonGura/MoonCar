"""Independent post-run verification. Never changes round results or counts."""
import json, sys
from pathlib import Path
sys.path.insert(0, '/source/scripts')
from run_r5_origin_campaign import pair_inputs, sha
from r5_d0_analysis import compare_pair
from run_r5_turn_slip import signature

p = Path('/evidence')
c = p / 'campaign'
s = json.loads((c / 'summary.json').read_text())
assert s['status'] != 'running', 'Do not analyze a live campaign as complete'
ledger = json.loads((c / 'ledger.json').read_text())
results = [json.loads((c / e['name'] / 'result.json').read_text())
           if (c / e['name'] / 'result.json').is_file() else None for e in ledger]
qualified = {i for i, e in enumerate(ledger) if e.get('qualified')}
steps = []
for i in range(2, 6):
    if {i, i + 4} <= qualified:
        steps.append(dict(indices=[i, i + 4], result=compare_pair(results[i], results[i + 4])))
pairs = []
for i, j in ((2, 4), (3, 5), (6, 8), (7, 9), (0, 1), (0, 12), (0, 23)):
    if {i, j} <= qualified:
        pairs.append(dict(indices=[i, j], result=pair_inputs(results[i], results[j])))
out = dict(status=s['status'], completed=len(ledger), qualified=len(qualified),
           matrix_passed=s['status'] == 'candidate_verified',
           step_comparisons=steps, input_pairs=pairs,
           all_completed_pairs_passed=all(x['result']['passed'] for x in steps + pairs),
           historical_results_used=False, production_changed=False,
           whole_vehicle_accepted=False)
(p / 'completed-pairs.json').write_text(json.dumps(out, indent=2) + '\n')

frozen = json.loads((c / 'freeze.json').read_text())
actual = signature()
actual['test_startup'] = {str(x): sha(x) for x in (p / 'test-startup').iterdir() if x.is_file()}
actual['test_gate_shells'] = {n: sha('/source/scripts/' + n) for n in
    ('prepare_r5_turn_gate_validation.sh', 'execute_r5_turn_gate_validation.sh')}
actual['test_gate_tests'] = {n: sha('/source/ros2_ws/src/mooncar_gazebo/tests/' + n) for n in
    ('test_r5_startup_guard.py', 'test_turn_gate_validation.py')}
canonical = lambda v: json.dumps(v, sort_keys=True)
differences = [k for k in set(actual) | set(frozen)
               if canonical(actual.get(k)) != canonical(frozen.get(k))]
proof = dict(passed=not differences, changed_fields=sorted(differences),
             normalized_tuple_list=True, source_and_installed_files=True)
(p / 'freeze-final.json').write_text(json.dumps(proof, indent=2) + '\n')
print(json.dumps(dict(pairs=out, freeze=proof)))
assert proof['passed'] and out['all_completed_pairs_passed']
