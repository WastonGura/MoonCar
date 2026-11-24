#!/usr/bin/env python3
"""Twelve fresh A/B/A attempts; no performance tuning or retries."""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

from r5_load_diagnostic import causal_decision, save, sha
from run_r5_origin_campaign import pair_inputs, signature as origin_signature

CASES = tuple((stage, profile, action, angular) for stage, profile in (('baseline', 'original'), ('mirror', 'mirror_x'))
              for action, angular in (('zero', 0.), ('left-1', .2), ('right-1', -.2), ('left-2', .2), ('right-2', -.2))) + (
                  ('restore', 'original', 'left', .2), ('restore', 'original', 'right', -.2))


def signature():
    sig = origin_signature()
    sig['mass_models'] = {p.name:sha(p) for p in Path('/evidence/mass-models').iterdir() if p.is_file()}
    sig['load_preflight'] = sha('/evidence/load-preflight.json')
    sig['preparation_source'] = sha('/source/scripts/prepare_r5_load.sh')
    sig['load_tests'] = sha('/source/ros2_ws/src/mooncar_gazebo/tests/test_load_diagnostic.py')
    return sig


def run(output):
    folder = Path(output); folder.mkdir()
    if not json.loads(Path('/evidence/load-preflight.json').read_text())['passed']:
        raise ValueError('Preflight did not pass')
    frozen = signature()
    if frozen['resources'] != json.loads(Path('/evidence/baseline/resources.json').read_text()):
        raise ValueError('Resource hashes changed')
    save(folder/'freeze.json', frozen)
    snapshot = folder/'source'
    for name in frozen['source']:
        dest = snapshot/name; dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(Path('/source')/name, dest)
    summary = dict(attempts=0, qualified=0, budget=12, status='running', cause_confirmed=False,
                   repair_verified=False, production_changed=False, execution_changed=False)
    ledger = []; results = []

    def persist():
        summary.update(attempts=len(ledger), qualified=sum(e.get('passed', False) for e in ledger))
        save(folder/'ledger.json', ledger); save(folder/'summary.json', summary)

    persist()
    try:
        for stage, profile, action, angular in CASES:
            if signature() != frozen:
                raise ValueError('Frozen execution changed before attempt')
            name = f'{len(ledger)+1:02d}-{stage}-{action}'
            entry = dict(name=name, stage=stage, profile=profile, angular=angular, started=time.time(), status='started')
            ledger.append(entry); persist()
            argv = [sys.executable, str(snapshot/'scripts/check_r5_load_round.py'), '--output', str(folder), '--name', name,
                    '--mass-profile', profile, '--angular', str(angular)]
            if stage == 'restore':
                argv.append('--restore')
            timed_out = False
            with (folder/(name+'.log')).open('x') as log:
                child = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    child.wait(580)
                except subprocess.TimeoutExpired:
                    timed_out = True; os.killpg(child.pid, signal.SIGINT)
                    try:
                        child.wait(8)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL); child.wait()
            path = folder/name/'result.json'
            result = json.loads(path.read_text()) if path.is_file() else dict(passed=False, error='Result missing')
            entry.update(passed=bool(result.get('passed') and child.returncode == 0 and not timed_out and time.time()-entry['started'] <= 600),
                         status='timeout' if timed_out else 'completed', exit_code=child.returncode,
                         wall_seconds=time.time()-entry['started'], audit=result.get('load_qualification'),
                         reason=result.get('load_error') or result.get('origin_error') or result.get('production_error') or result.get('error'))
            results.append(result); persist(); print(json.dumps(entry), flush=True)
            if signature() != frozen:
                summary['execution_changed'] = True
                raise ValueError('Frozen execution changed after attempt')
            if not entry['passed']:
                raise ValueError('Critical data/input/physical gate failed; no rerun: '+name)
            # Immediately qualify new same-direction input against all earlier ones.
            pairs = [pair_inputs(previous, result) for previous, old in zip(results[:-1], ledger[:-1]) if angular != 0 and old['angular'] == angular]
            entry['input_pairs'] = pairs; persist()
            if not all(p['passed'] for p in pairs):
                raise ValueError('Cross-stage/repeat input budgets differ; stop')
        summary.update(causal_decision(ledger))
        summary['status'] = 'completed'
        summary['decision'] = causal_decision(ledger)
    except Exception as error:
        summary.update(status='stopped', reason=repr(error), cause_confirmed=False)
    persist()
    save(folder/'final-freeze-check.json', dict(passed=signature() == frozen))
    return 0 if summary['status'] == 'completed' else 1


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(); parser.add_argument('--output', required=True)
    raise SystemExit(run(parser.parse_args().output))
