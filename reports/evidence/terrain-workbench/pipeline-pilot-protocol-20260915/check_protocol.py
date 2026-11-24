"""Verify the finite headless pipeline pilot protocol before launch."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[4]
folder = Path(__file__).parent
protocol = json.loads((folder / 'protocol.json').read_text(encoding='utf-8'))

paths = {
    'freeze_json_sha256': root / 'reports/evidence/terrain-workbench/research-freeze-20260914/freeze.json',
    'source_zip_sha256': root / 'reports/evidence/terrain-workbench/research-freeze-20260914/source.zip',
    'runner_sha256': root / 'scripts/compare_survey_strategies.py',
    'evaluator_sha256': root / 'scripts/mapping_acceptance.py',
}
for key, path in paths.items():
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    assert actual == protocol['research_freeze'][key], f'changed: {path.relative_to(root)}'
assert hashlib.sha256((folder / 'run_pilot.sh').read_bytes()).hexdigest() == protocol['run_script_sha256']
assert protocol['strategies'] == ['nearest_view_v1', 'coverage_view_v1', 'surface_view_v1']
assert protocol['declared_trials'] == len(protocol['starts']) * len(protocol['strategies'])
assert protocol['repeats_per_strategy_condition'] == 1
assert protocol['budget_seconds'] == 30
assert protocol['radius_m'] == 8
assert protocol['capture_gui'] is False
assert protocol['interpretation'].startswith('Pipeline feasibility only')
print('PASS: six-trial headless pipeline pilot is frozen and not a strategy comparison')
