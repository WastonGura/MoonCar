"""Check the frozen finite experiment protocol before and after execution."""
import hashlib
import json
from pathlib import Path

here=Path(__file__).resolve().parent
root=Path(__file__).resolve().parents[4]
protocol=json.loads((here/'protocol.json').read_text(encoding='utf-8'))
freeze=protocol['research_freeze']
paths={
    'freeze_json_sha256':root/'reports/evidence/terrain-workbench/research-freeze-20260914/freeze.json',
    'source_zip_sha256':root/'reports/evidence/terrain-workbench/research-freeze-20260914/source.zip',
    'runner_sha256':root/'scripts/compare_survey_strategies.py',
    'evaluator_sha256':root/'scripts/mapping_acceptance.py'
}
for key,path in paths.items():assert hashlib.sha256(path.read_bytes()).hexdigest()==freeze[key],key
assert protocol['declared_trials']==len(protocol['strategies'])*sum(len(s['starts']) for s in protocol['scenes'])*protocol['repeats_per_strategy_condition']==54
assert protocol['budget_seconds']==90 and protocol['radius_m']==8
assert len({s['name'] for s in protocol['scenes']})==3
assert all(len(s['starts'])==2 for s in protocol['scenes'])
assert hashlib.sha256((here/'run_formal.sh').read_bytes()).hexdigest()==protocol['run_script_sha256']
print('PASS: frozen source and 54-trial protocol match')
