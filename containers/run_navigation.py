"""Reproducible gateway mission; optional real Gazebo obstacle insertion."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import threading
import time
import uuid
from urllib.request import Request, urlopen

parser = argparse.ArgumentParser()
parser.add_argument('--goal', type=float, nargs=2, required=True)
parser.add_argument('--mode', choices=['fixed', 'full'], default='full')
parser.add_argument('--obstacle', type=float, nargs=2)
parser.add_argument('--obstacle-on-path', action='store_true')
parser.add_argument('--obstacle-size', type=float, default=.4)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
if not .1 <= args.obstacle_size <= 1. or (args.obstacle and args.obstacle_on_path):
    parser.error('Use one obstacle position mode and a size between 0.1 and 1 metre')
obstacle_xy = args.obstacle
base = 'http://127.0.0.1:8766'

def api(path, body=None):
    request = Request(base + path, data=None if body is None else json.dumps(body).encode(),
                      headers={'Content-Type':'application/json', 'Origin':base})
    with urlopen(request, timeout=10) as response:
        return json.load(response)

initial = api('/api/navigation')
assert initial['servers_ready'] and not initial['active'] and not initial['fault']
info = api('/api/info?session=' + initial['session'])
token = api('/api/navigation/start', dict(session=initial['session'], frame=info['frames'],
                                         goal=args.goal, yaw=0., mode=args.mode))['token']
stop = threading.Event()
heartbeat_errors = []
def heartbeat():
    while not stop.is_set():
        try:
            if not api('/api/navigation/heartbeat', dict(token=token))['active']:
                return
        except Exception as error:
            heartbeat_errors.append(str(error))
            return
        stop.wait(.5)
thread = threading.Thread(target=heartbeat, daemon=True)
thread.start()
begin = time.monotonic()
states, phase, spawned, creation = [], None, None, None
failure = None
try:
    while time.monotonic() - begin < 120.:
        state = api('/api/navigation')
        states.append(dict(elapsed=time.monotonic()-begin, phase=state['phase'],
                           pose=state['pose'], replans=state['replans']))
        if phase != state['phase']:
            phase = state['phase']
            print(f'{time.monotonic()-begin:.1f}s {phase}: {state["error"]}', flush=True)
        if (args.obstacle or args.obstacle_on_path) and state['phase'] == 'executing' and spawned is None:
            if args.obstacle_on_path:
                segments = [(a,b,math.hypot(a['x']-b['x'],a['y']-b['y'])) for a,b in zip(state['path'],state['path'][1:])]
                halfway = sum(d for _,_,d in segments)/2
                assert halfway >= .8, 'Test path is too short for a safe obstacle insertion'
                for a,b,d in segments:
                    if d and halfway <= d:
                        obstacle_xy = [a['x']+(b['x']-a['x'])*halfway/d,a['y']+(b['y']-a['y'])*halfway/d]
                        break
                    halfway -= d
            spawned = 'terrain_test_' + uuid.uuid4().hex[:8]
            sdf = (f'<sdf version="1.9"><model name="{spawned}"><static>true</static><link name="box">'
                   f'<collision name="c"><geometry><box><size>{args.obstacle_size} {args.obstacle_size} 0.6</size></box></geometry></collision>'
                   f'<visual name="v"><geometry><box><size>{args.obstacle_size} {args.obstacle_size} 0.6</size></box></geometry></visual>'
                   '</link></model></sdf>')
            creation = subprocess.Popen(['ros2','run','ros_gz_sim','create','-world','lunar_yard',
                                          '-string',sdf,'-x',str(obstacle_xy[0]),'-y',str(obstacle_xy[1]),'-z','0.3'],
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        if not state['active']:
            break
        time.sleep(.25)
    else:
        raise TimeoutError('Mission exceeded 120 seconds wall time')
    if creation:
        output = creation.communicate(timeout=10)[0]
        if creation.returncode:
            raise RuntimeError('Gazebo obstacle creation failed: ' + output)
except Exception as error:
    failure = str(error)
finally:
    stop.set()
    thread.join(timeout=12.)
    state = api('/api/navigation')
    if state['active']:
        api('/api/navigation/cancel', dict(token=token))
        deadline = time.monotonic() + 12.
        while state['active'] and time.monotonic() < deadline:
            time.sleep(.1)
            state = api('/api/navigation')
    if spawned:
        removal = subprocess.run(['gz','service','-s','/world/lunar_yard/remove','--reqtype','gz.msgs.Entity',
                                  '--reptype','gz.msgs.Boolean','--timeout','5000','--req',f'name: "{spawned}" type: MODEL'],
                                 capture_output=True, text=True, timeout=10)
        if removal.returncode or 'true' not in removal.stdout:
            failure = 'Failed to remove owned test obstacle'
    report = dict(initial=initial, final=state, goal=args.goal, mode=args.mode, obstacle=obstacle_xy,
                  obstacle_size=args.obstacle_size if obstacle_xy else None,
                  elapsed_seconds=time.monotonic()-begin, samples=states,
                  heartbeat_errors=heartbeat_errors, failure=failure)
    report['passed'] = (failure is None and not heartbeat_errors and state['phase']=='succeeded'
                        and not state['active'] and not state['fault']
                        and (not obstacle_xy or state['replans'] > 0))
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(f'Report: {args.output}; passed={report["passed"]}', flush=True)
if not report['passed']:
    raise SystemExit(1)
