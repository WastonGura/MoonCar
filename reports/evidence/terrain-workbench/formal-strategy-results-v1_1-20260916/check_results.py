"""Verify the completed v1.1 reevaluation and lightweight formal summary."""
import hashlib
import json
import os
from pathlib import Path

report = Path(__file__).resolve().parent
root = Path(__file__).resolve().parents[4]
reevaluation = Path(os.environ.get('MOONCAR_REEVALUATION',
    root / 'tmp/ros_acceptance/formal-strategy-20260915-reevaluated-v1_1-20260916'))

assert (reevaluation / 'COMPLETE').is_file() and not (reevaluation / 'FAILED').exists()
for name, expected in json.loads((reevaluation / 'hashes.json').read_text()).items():
    assert hashlib.sha256((reevaluation / name).read_bytes()).hexdigest() == expected, name
for name, expected in json.loads((report / 'hashes.json').read_text()).items():
    assert hashlib.sha256((report / name).read_bytes()).hexdigest() == expected, name
trials = json.loads((reevaluation / 'trials.json').read_text(encoding='utf-8'))
assert len(trials) == 54 and all(t['acceptance']['status'] == 'failed' for t in trials)
assert all(t['acceptance']['S'] is not None for t in trials)
summary = json.loads((report / 'summary.json').read_text(encoding='utf-8'))
assert summary['evaluation_version'] == 'mapping-acceptance-v1.1'
assert summary['trial_count'] == 54 and summary['condition_count'] == 6
print('PASS: 54 evaluable v1.1 trials and formal strategy summary match frozen hashes')
