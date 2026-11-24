#!/usr/bin/env python3
"""Bounded feasibility probe with frozen inputs and independent fresh worlds."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from run_r5_origin_campaign import signature, sha, pair_inputs


def run(output):
    folder = Path(output); folder.mkdir()
    prediction = json.loads(Path('/evidence/prediction.json').read_text())
    if prediction['production_change'] or prediction['profile'] != 'load_dither':
        raise RuntimeError('Missing frozen drive-only prediction')
    if not json.loads(Path('/evidence/origin-preflight.json').read_text())['passed']:
        raise RuntimeError('Model/installed library preflight failed')
    frozen = signature(); frozen['prediction'] = sha('/evidence/prediction.json')
    if frozen['resources'] != json.loads(Path('/evidence/baseline/resources.json').read_text()):
        raise RuntimeError('Resources changed')
    (folder/'freeze.json').write_text(json.dumps(frozen, indent=2)+'\n')
    ledger = []; results = []
    summary = dict(status='running', attempts=0, attempt_limit=4,
                   repair_verified=False, production_changed=False, physics_changed=False)

    def save():
        summary['attempts'] = len(ledger)
        (folder/'ledger.json').write_text(json.dumps(ledger, indent=2)+'\n')
        (folder/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')

    try:
        for profile, angular in (('baseline', .2), ('baseline', -.2), ('load_dither', .2), ('load_dither', -.2)):
            current = signature(); current['prediction'] = sha('/evidence/prediction.json')
            if current != frozen: raise RuntimeError('Frozen source/configuration changed')
            name = f'{len(ledger)+1:02d}-{profile}-'+('left' if angular > 0 else 'right')
            entry = dict(name=name, profile=profile, angular=angular, started_wall=time.time(), status='started')
            ledger.append(entry); save()
            env = dict(os.environ, GZ_PARTITION='mooncar_turn_probe_'+name+'_'+str(os.getpid()))
            argv = [sys.executable, '/source/scripts/check_r5_turn_control_round.py', '--output', str(folder),
                    '--name', name, '--profile', profile, '--angular', str(angular)]
            with (folder/(name+'.log')).open('x') as log:
                child = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
                timeout = False
                try: child.wait(timeout=570)
                except subprocess.TimeoutExpired:
                    timeout = True; os.killpg(child.pid, signal.SIGINT)
                    try: child.wait(timeout=10)
                    except subprocess.TimeoutExpired: os.killpg(child.pid, signal.SIGKILL); child.wait()
            path = folder/name/'result.json'
            result = json.loads(path.read_text()) if path.is_file() else dict(passed=False, error='Missing round result')
            entry.update(qualified=bool(result.get('passed') and child.returncode == 0 and not timeout),
                         status='timeout' if timeout else 'completed', exit_code=child.returncode,
                         wall_seconds=time.time()-entry['started_wall'], outcome=result.get('turn_control_outcome'),
                         error=result.get('error') or result.get('origin_error') or result.get('production_error'))
            if entry['wall_seconds'] > 600: entry['qualified'] = False
            results.append(result); save(); print(json.dumps(entry), flush=True)
            if not entry['qualified']: raise RuntimeError('Input/capture/physical qualification failed; no replacement runs: '+name)
        comparisons = []
        for old, new in zip(results[:2], results[2:]):
            baseline = old['turn_control_outcome']; candidate = new['turn_control_outcome']
            inputs = pair_inputs(old, new)
            yaw_loss = abs(baseline['unwrapped_turn_rad'])-abs(candidate['unwrapped_turn_rad'])
            passed = bool(inputs['passed'] and candidate['center_excursion_passed'] and yaw_loss <= .05)
            comparisons.append(dict(passed=passed, inputs=inputs, actual_yaw_loss_rad=yaw_loss,
                                    baseline=baseline, candidate=candidate))
        summary.update(comparisons=comparisons,
                       status='feasible_needs_full_validation' if all(c['passed'] for c in comparisons) else 'candidate_ineffective',
                       hypothesis_supported=all(c['passed'] for c in comparisons),
                       reason='Fresh drive-only feasibility data; no production release or whole-vehicle acceptance')
    except Exception as error:
        summary.update(status='stopped', reason=repr(error))
    finally:
        current = signature(); current['prediction'] = sha('/evidence/prediction.json')
        summary['freeze_unchanged'] = current == frozen
        if not summary['freeze_unchanged']:
            summary.update(status='stopped', reason='Frozen signature changed')
        save()
    return 0 if summary['status'] == 'feasible_needs_full_validation' else 1


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--output', required=True)
    raise SystemExit(run(p.parse_args().output))
