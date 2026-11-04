#!/usr/bin/env python3
"""One explicit R5/window/native-V2 integration run; never retries or changes tuning."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.request import Request, build_opener, ProxyHandler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--sessions', required=True, type=Path)
    parser.add_argument('--url', default='http://127.0.0.1:8766')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    opener = build_opener(ProxyHandler({}))

    def write(name, value):
        (args.output/name).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')

    def request(path='', body=None):
        data = None if body is None else json.dumps(body).encode()
        req = Request(args.url+'/api/simulation'+path, data=data,
                      headers={'Content-Type': 'application/json', 'Origin': args.url})
        with opener.open(req, timeout=8) as stream:
            return json.load(stream)

    state = request()
    if state.get('task') or state.get('phase') != 'idle':
        raise RuntimeError('Preserve existing experiment; this run needs an idle backend')
    software = state.get('software', {})
    if not software.get('verified') or not software.get('product_clean'):
        raise RuntimeError('A verified committed runtime is required')
    baseline = software['baseline']
    assert baseline['simulation_profile'] == 'r5_support_v1'
    assert baseline['localization_mode'] == 'rgbd_window' and not baseline['active_revisit']
    task = dict(version=1, scene='survey_v2_native', simulation_profile='r5_support_v1',
                gui=False, region=[-1., -2., 6., 4.], start=[.5, 0., 0.])
    route = [('move', [1.2, 0.]), ('turn', math.pi/4), ('turn', 0.), ('move', [.5, 0.])]
    write('protocol.json', dict(software=software, task=task, route=route,
          wall_limit_s=600, one_world_only=True, scope='integration smoke, not full coverage/endurance or slope acceptance',
          tuning_changes=False, truth_used_by_control=False))
    observer = handle = None
    experiment = None
    result = dict(outcome='failed', software=software, actions=[])
    begun = time.monotonic()
    log = (args.output/'states.jsonl').open('x')

    def sample():
        value = request()
        if experiment and (value.get('task') or {}).get('id') != experiment:
            raise RuntimeError('Owned experiment identity changed')
        saved = dict(value)
        saved['motion'] = {k: v for k, v in value.get('motion', {}).items()
                           if k not in ('navigation_grid', 'path', 'track')}
        log.write(json.dumps(saved, ensure_ascii=False)+'\n'); log.flush()
        if time.monotonic()-begun > 600:
            raise TimeoutError('One-run wall budget reached')
        if value.get('phase') == 'failed':
            raise RuntimeError(value.get('error'))
        return value

    try:
        applied = request('/apply', task)
        write('apply.json', applied)
        deadline = time.monotonic()+30
        while experiment is None:
            value = sample()
            current = value.get('task') or {}
            if current and all(current.get(k) == v for k, v in task.items()):
                experiment = current['id']
            if time.monotonic() > deadline:
                raise TimeoutError('Experiment allocation timed out')
            time.sleep(.25)
        result['experiment'] = experiment
        env = dict(os.environ, ROS_DOMAIN_ID='89', GZ_PARTITION='mooncar-'+experiment)
        handle = (args.output/'observer.log').open('x')
        observer = subprocess.Popen([sys.executable, '-B', str(Path(__file__).with_name('observe_simulation.py')),
            '--folder', str(args.output/'observer'), '--world', 'mooncar_survey_v2', '--start-x', '.5'],
            env=env, stdout=handle, stderr=subprocess.STDOUT)
        deadline = time.monotonic()+140
        while True:
            value = sample()
            motion = value.get('motion', {})
            health = motion.get('diagnostics', {}).get('laser_localization', {})
            if motion.get('ready') and health.get('healthy'):
                break
            if time.monotonic() > deadline:
                raise TimeoutError('Latest R5/window runtime did not become motion-ready')
            time.sleep(.25)
        write('ready.json', value)
        loaded_path = args.sessions/experiment/'support-runtime/loaded.json'
        loaded = json.loads(loaded_path.read_text())
        write('support-loaded.json', loaded)
        for kind, target in route:
            payload = dict(experiment=experiment, **({'goal': target} if kind == 'move' else {'yaw': target}))
            action = dict(kind=kind, target=target)
            result['actions'].append(action)
            response = request('/'+kind, payload)
            token = response['token']
            while True:
                request('/heartbeat', dict(token=token))
                value = sample()
                if not value['motion'].get('active'):
                    break
                time.sleep(.2)
            action.update(phase=value['motion']['phase'], error=value['motion'].get('error'),
                          pose=value['motion'].get('pose'))
            if action['phase'] != 'succeeded':
                raise RuntimeError('Action stopped: '+json.dumps(action, ensure_ascii=False))
        result['outcome'] = 'passed'
    except Exception as error:
        result['error'] = repr(error)
    finally:
        if experiment:
            try:
                value = request()
                if (value.get('task') or {}).get('id') == experiment:
                    request('/cancel', {})
                    request('/stop', {})
            except Exception as error:
                result['cleanup_error'] = repr(error)
        if observer:
            if observer.poll() is None:
                observer.send_signal(signal.SIGINT)
            try:
                result['observer_exit'] = observer.wait(timeout=25)
            except subprocess.TimeoutExpired:
                observer.kill(); result['observer_exit'] = observer.wait(timeout=5)
                result['observer_forced'] = True
        if handle: handle.close()
        log.close()
        result['elapsed_wall_s'] = time.monotonic()-begun
        write('result.json', result)
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result['outcome'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
