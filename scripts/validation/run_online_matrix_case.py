#!/usr/bin/env python3
"""One measured R5 online case, without changing product readiness or safety gates."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import traceback
from urllib.error import HTTPError
from urllib.request import Request, build_opener, ProxyHandler
from trial_outcome import lifecycle_snapshot, mission_outcome


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=('direct', 'autonomy', 'ramp'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sessions', type=Path, required=True)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    api = 'http://127.0.0.1:8766'
    here = Path(__file__).resolve().parent
    def write(name, value):
        (args.output/name).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    def request(path='', body=None):
        req = Request(api+'/api/simulation'+path,
            data=None if body is None else json.dumps(body).encode(),
            headers={'Origin': api, 'Content-Type': 'application/json'})
        try:
            with build_opener(ProxyHandler({})).open(req, timeout=5) as stream:
                return json.load(stream)
        except HTTPError as error:
            raise RuntimeError('HTTP '+str(error.code)+': '+error.read().decode()) from error
    state = request()
    if state.get('task') or state.get('phase') != 'idle':
        raise RuntimeError('Preserve existing experiment; an idle owned backend is required')
    software = state.get('software', {})
    assert software.get('verified') and software.get('product_clean')
    assert software['baseline']['simulation_profile'] == 'r5_support_v1'
    ramp = args.case == 'ramp'
    task = dict(version=1, scene='survey_v2_native', simulation_profile='r5_support_v1', gui=os.environ.get('MOONCAR_VALIDATION_GUI')=='1',
        region=[-8., -8., 8., 8.] if ramp else [-15., -15., 15., 15.],
        start=[0., 0., 0.] if ramp else [.5, 0., 0.])
    wall_budget = 480 if args.case == 'direct' else 1200
    sim_budget = 300 if ramp else 180
    route = [('move', [1.2, 0.]), ('turn', math.pi/4), ('turn', 0.), ('move', [.5, 0.]),
             ('pan_tilt', [.4, -.2]), ('pan_tilt', [0., 0.])]
    write('protocol.json', dict(case=args.case, heading_trial=os.environ.get('MOONCAR_RAMP_HEADING_CASE'), task=task, software=software, wall_budget_s=wall_budget,
        mission_sim_budget_s=sim_budget, planned_direct_actions=route if args.case == 'direct' else None,
        readiness='Record product ready independently; test dispatch waits for all four actual Nav2 lifecycle states ACTIVE',
        product_changes=False, automatic_retries=False, truth_used_for_control=False,
        scope='Measured single run; budgets are collection limits, not performance acceptance thresholds'))
    result = dict(case=args.case, outcome='failed', actions=[], software=software)
    begun = time.monotonic(); experiment = None; processes = {}; handles = []; primary = None
    state_log = (args.output/'states.jsonl').open('x')
    grid_log = (args.output/'navigation-grids.jsonl').open('x') if os.environ.get('MOONCAR_VALIDATION_GUI')=='1' else None
    last_grid_version = None
    resources = (args.output/'resources.jsonl').open('x')
    readiness = (args.output/'readiness.jsonl').open('x')
    last_resource = last_progress = 0.; premature = []; first_ready = first_active = None
    snapshot_errors = []

    def lifecycle():
        return lifecycle_snapshot(args.output/'lifecycle/latest.json', snapshot_errors)

    def sample():
        nonlocal last_resource, last_progress, first_ready, first_active, last_grid_version
        value = request(); now = time.monotonic()
        if experiment and (value.get('task') or {}).get('id') != experiment:
            raise RuntimeError('Owned experiment identity changed')
        motion = value.get('motion', {}); live = lifecycle()
        saved = dict(value)
        saved['motion'] = {k: v for k, v in motion.items() if k not in ('navigation_grid', 'path', 'track')}
        state_log.write(json.dumps(dict(wall=now, state=saved), ensure_ascii=False)+'\n'); state_log.flush()
        grid = motion.get('navigation_grid')
        if grid_log is not None and grid and grid.get('version') != last_grid_version:
            grid_log.write(json.dumps(dict(wall=now, grid=grid))+'\n')
            grid_log.flush()
            last_grid_version = grid.get('version')
        active = bool(live.get('all_active')) and now-live.get('wall', 0) < 3
        if motion.get('ready') and first_ready is None: first_ready = now
        if active and first_active is None: first_active = now
        if motion.get('ready') and not active: premature.append(now)
        readiness.write(json.dumps(dict(wall=now, product_ready=motion.get('ready'), lifecycle=live,
                                       lifecycle_fresh_and_active=active))+'\n'); readiness.flush()
        if now-last_resource > 5:
            row = dict(wall=now, sim=motion.get('diagnostics', {}).get('sim_stamp'))
            if primary:
                try:
                    status = Path('/proc')/str(primary['localization']['process']['pid'])/'status'
                    fields = dict(x.split(':', 1) for x in status.read_text().splitlines() if ':' in x)
                    row['localization_rss_kib'] = int(fields['VmRSS'].split()[0])
                except (FileNotFoundError, KeyError, ProcessLookupError): row['primary_process_missing'] = True
            resources.write(json.dumps(row)+'\n'); resources.flush(); last_resource = now
        if now-last_progress > 25:
            print(json.dumps(dict(event='progress', case=args.case, wall_s=round(now-begun, 1),
                sim=motion.get('diagnostics', {}).get('sim_stamp'), motion=motion.get('phase'),
                autonomy=value.get('autonomy', {}).get('phase'))), flush=True)
            last_progress = now
        if now-begun > wall_budget: raise TimeoutError('Declared collection wall budget reached')
        if value.get('phase') == 'failed': raise RuntimeError(value.get('error'))
        return value, active

    def spawn(name, command, env):
        handle = (args.output/(name+'.log')).open('x'); handles.append(handle)
        processes[name] = subprocess.Popen(command, env=env, stdout=handle, stderr=subprocess.STDOUT)

    def await_ready(seconds):
        deadline = time.monotonic()+seconds
        while True:
            value, active = sample(); motion = value.get('motion', {})
            health = motion.get('diagnostics', {}).get('laser_localization', {})
            if motion.get('ready') and health.get('healthy') and active: return value
            if time.monotonic() > deadline: raise TimeoutError('Navigation lifecycle/localization readiness not met')
            time.sleep(.25)

    try:
        write('apply.json', request('/apply', task))
        deadline = time.monotonic()+30
        while experiment is None:
            value, _ = sample(); current = value.get('task') or {}
            if current and all(current.get(k) == v for k, v in task.items()): experiment = current['id']
            if time.monotonic() > deadline: raise TimeoutError('Experiment allocation timed out')
            time.sleep(.2)
        result['experiment'] = experiment
        env = dict(os.environ, ROS_DOMAIN_ID='89', GZ_PARTITION='mooncar-'+experiment)
        spawn('lifecycle', [sys.executable, '-B', str(here/'observe_lifecycle.py'),
                           '--output', str(args.output/'lifecycle')], env)
        spawn('observer', [sys.executable, '-B', str(here/'observe_simulation.py'), '--folder',
            str(args.output/'observer'), '--world', 'mooncar_ramp_v2' if ramp else 'mooncar_survey_v2',
            '--start-x', '0' if ramp else '.5'], env)
        value = await_ready(150); write('ready.json', value)
        actual = json.loads((args.sessions/experiment/'software.json').read_text())
        assert actual['simulation_profile'] == 'r5_support_v1' and actual['localization_mode'] == 'rgbd_window'
        loaded = json.loads((args.sessions/experiment/'support-runtime/loaded.json').read_text())
        assert loaded['profile'] == 'r5_support_v1' and loaded['main_step_s'] == .00025
        write('support-loaded.json', loaded)
        import primary_runtime
        primary = primary_runtime.apply(experiment, args.output/'runtime-affinity.json')
        if args.case == 'direct':
            for kind, target in route:
                action = dict(kind=kind, target=target); result['actions'].append(action)
                try:
                    await_ready(20)
                    payload = dict(experiment=experiment)
                    if kind == 'move': payload['goal'] = target
                    elif kind == 'turn': payload['yaw'] = target
                    else: payload.update(pan=target[0], tilt=target[1])
                    write('action-%02d-start.json' % len(result['actions']), sample()[0])
                    token = request('/'+kind, payload)['token']
                    stopped = threading.Event(); heartbeat_errors = []
                    def heartbeat():
                        while not stopped.wait(.2):
                            try: request('/heartbeat', dict(token=token))
                            except Exception as error: heartbeat_errors.append(repr(error))
                    worker = threading.Thread(target=heartbeat, daemon=True); worker.start()
                    try:
                        while True:
                            value, _ = sample()
                            if not value['motion'].get('active'): break
                            time.sleep(.25)
                    finally:
                        stopped.set(); worker.join(timeout=6)
                    action.update(phase=value['motion']['phase'], error=value['motion'].get('error'),
                        pose=value['motion'].get('pose'), heartbeat_errors=heartbeat_errors,
                        stop_diagnostics=value['motion'].get('stop_diagnostics'))
                    write('action-%02d-end.json' % len(result['actions']), value)
                except TimeoutError: raise
                except Exception as error:
                    action.update(phase='rejected_before_or_during_dispatch', error=repr(error))
                    request('/cancel', {})
            result['outcome'] = 'passed' if all(a.get('phase') == 'succeeded' for a in result['actions']) else 'failed'
        else:
            body = dict(experiment=experiment, budget_seconds=sim_budget, radius=8 if ramp else 14,
                        strategy='surface_view_v2')
            write('explore-request.json', body); write('explore-response.json', request('/explore', body))
            while True:
                value, _ = sample(); autonomy = value.get('autonomy', {})
                if not autonomy.get('active'): break
                time.sleep(.4)
            result['autonomy'] = autonomy
            result['outcome'] = mission_outcome(autonomy.get('phase'))
        write('terminal.json', sample()[0])
    except TimeoutError as error:
        result.update(outcome='incomplete_collection_budget' if 'Declared collection wall budget' in str(error) else 'failed',
                      error=repr(error), traceback=traceback.format_exc())
    except Exception as error:
        result.update(error=repr(error), traceback=traceback.format_exc())
    finally:
        if experiment:
            try:
                current = request()
                if (current.get('task') or {}).get('id') == experiment:
                    result['terminal_before_cleanup'] = current
                    request('/explore_stop', {}); request('/cancel', {})
                    tail = float(os.environ.get('MOONCAR_VALIDATION_VIDEO_TAIL_SECONDS', '0'))
                    if tail > 0:
                        sim = current['motion']['diagnostics'].get('sim_stamp')
                        deadline = time.monotonic()+30
                        while sim is not None and time.monotonic()<deadline:
                            value, _ = sample()
                            stamp = value['motion']['diagnostics'].get('sim_stamp')
                            if stamp is not None and stamp >= sim+tail:break
                            time.sleep(.25)
            except Exception as error: result['cleanup_request_error'] = repr(error)
        for name, process in reversed(list(processes.items())):
            if process.poll() is None: process.send_signal(signal.SIGINT)
            try: code = process.wait(timeout=25)
            except subprocess.TimeoutExpired:
                process.kill(); code = process.wait(timeout=5); result[name+'_forced'] = True
            result[name+'_exit'] = code
        if experiment:
            try: request('/stop', {})
            except Exception as error: result['world_stop_error'] = repr(error)
        for handle in handles: handle.close()
        for stream in (state_log, resources, readiness): stream.close()
        if grid_log is not None:grid_log.close()
        result.update(elapsed_wall_s=time.monotonic()-begun, first_product_ready_wall=first_ready,
            first_lifecycle_active_wall=first_active, premature_ready_samples=len(premature),
            snapshot_read_errors=snapshot_errors)
        write('result.json', result)
    print(json.dumps({k: result.get(k) for k in ('case', 'experiment', 'outcome', 'error', 'elapsed_wall_s')},
                     ensure_ascii=False), flush=True)
    return int(result['outcome'] not in ('passed', 'budget_completed'))


if __name__ == '__main__': raise SystemExit(main())
