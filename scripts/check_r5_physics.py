#!/usr/bin/env python3
"""Bounded, isolated Gazebo R5 physics diagnostics; never modifies source assets.

Run inside a Jazzy environment after building mooncar_description. Each case
uses a fresh paused world and archives the exact SDF plus raw transport streams.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

ENGINES = {'bullet': 'gz-physics-bullet-featherstone-plugin',
           'dart': 'gz-physics-dartsim-plugin'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def command(*args, timeout=20):
    return subprocess.run(args, capture_output=True, text=True, check=True,
                          timeout=timeout).stdout.strip()


def last_message(text):
    # gz topic -n 1 can deliver another queued message before exiting.
    return json.loads(text.splitlines()[-1])


def timestamp(row):
    stamp = row.get('header', {}).get('stamp', {})
    return int(stamp.get('sec', 0)) + int(stamp.get('nsec', 0)) / 1e9


def load(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def angles(pose):
    q = pose.get('orientation', {})
    x, y, z, w = [float(q.get(k, 1 if k == 'w' else 0)) for k in ('x', 'y', 'z', 'w')]
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1, min(1, 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def body_samples(rows):
    out = []
    for row in rows:
        poses = row.get('pose', [])
        if isinstance(poses, dict):
            poses = [poses]
        pose = next((p for p in poses if p.get('name') in ('mooncar', 'mooncar::base_footprint')), None)
        if pose is None:
            continue
        p = pose.get('position', {})
        out.append(dict(t=timestamp(pose), xyz=[float(p.get(k, 0)) for k in ('x', 'y', 'z')],
                        rpy=angles(pose)))
    return out


def stability(samples):
    if not samples:
        return dict(passed=False, reason='missing body truth')
    first = samples[0]
    drift = max(math.dist(s['xyz'][:2], first['xyz'][:2]) for s in samples)
    yaw = max(abs(math.atan2(math.sin(s['rpy'][2]-first['rpy'][2]),
                             math.cos(s['rpy'][2]-first['rpy'][2]))) for s in samples)
    tilt = max(max(abs(s['rpy'][0]), abs(s['rpy'][1])) for s in samples)
    gap = max((b['t']-a['t'] for a, b in zip(samples, samples[1:])), default=math.inf)
    return dict(passed=drift <= .05 and yaw <= .05 and tilt <= .1 and gap <= .100001,
                drift_m=drift, yaw_change_rad=yaw, max_tilt_rad=tilt,
                max_truth_gap_s=gap, samples=len(samples), first=first, last=samples[-1])


def prepare(args, folder):
    if args.world:
        root = ET.parse(args.world).getroot()
        world = root.find('world')
        for uri in root.findall('.//mesh/uri'):
            if uri.text.startswith('model://mooncar_gazebo/'):
                uri.text = str(Path(args.package) / uri.text.removeprefix('model://mooncar_gazebo/'))
    else:
        root = ET.fromstring('''<sdf version="1.10"><world name="r5_check">
          <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"><engine><filename/></engine></plugin>
          <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
          <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
          <physics name="check" type="ode"><max_step_size/><real_time_factor>1</real_time_factor></physics>
          <gravity>0 0 -1.62</gravity><model name="plate"><static>true</static><link name="surface">
          <collision name="plate"><pose>0 0 -.05 0 0 0</pose><geometry><box><size>40 40 .1</size></box></geometry>
          <surface><friction><ode><mu>.8</mu><mu2>.72</mu2></ode><bullet><friction>.8</friction><friction2>.72</friction2></bullet></friction></surface>
          </collision></link></model></world></sdf>''')
        world = root.find('world')
    world.set('name', 'r5_check')
    world.find("plugin[@name='gz::sim::systems::Physics']/engine/filename").text = args.plugin or ENGINES[args.backend]
    world.find('physics/max_step_size').text = str(args.step)
    ET.SubElement(world, 'plugin', filename='gz-sim-contact-system', name='gz::sim::systems::Contact')
    robot = deepcopy(ET.parse(args.model).find('model'))
    robot.set('name', 'mooncar')
    if robot.find('pose') is None:
        ET.SubElement(robot, 'pose').text = '0 0 .03 0 0 0'
    else:
        robot.find('pose').text = '0 0 .03 0 0 0'
    for plugin in list(robot.findall('plugin')):
        if (args.disable_gimbal and plugin.get('name', '').endswith('JointPositionController')
                or args.disable_drive and plugin.get('name', '').endswith('DiffDrive')):
            robot.remove(plugin)
    # PosePublisher reports actual link and model world poses, not wheel odometry.
    plugin = ET.SubElement(robot, 'plugin', filename='gz-sim-pose-publisher-system',
                           name='gz::sim::systems::PosePublisher')
    for key, value in dict(publish_link_pose='true', publish_model_pose='true',
                           publish_sensor_pose='true', use_pose_vector_msg='true',
                           update_frequency='100', static_publisher='false').items():
        ET.SubElement(plugin, key).text = value
    for link in robot.findall('link'):
        collision = link.find('collision')
        if 'wheel' in link.get('name', '') and collision is not None:
            sensor = ET.SubElement(link, 'sensor', name=link.get('name')+'_contact', type='contact')
            ET.SubElement(sensor, 'update_rate').text = '100'
            ET.SubElement(sensor, 'always_on').text = 'true'
            ET.SubElement(sensor, 'topic').text = '/contacts/'+link.get('name')
            ET.SubElement(ET.SubElement(sensor, 'contact'), 'collision').text = collision.get('name')
    world.append(robot)
    path = folder / 'world.sdf'
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
    return path


def run(args):
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    ledger = output / 'diagnostic-ledger.json'
    cases = json.loads(ledger.read_text()) if ledger.exists() else []
    if len(cases) >= 20:
        raise RuntimeError('20 diagnostic rounds exhausted')
    folder = output / args.name
    folder.mkdir()  # Never overwrite a failed or completed case.
    cases.append(dict(name=args.name, started=time.time(), status='started'))
    ledger.write_text(json.dumps(cases, indent=2))
    os.environ['GZ_PARTITION'] = 'mooncar_r5_20261006_'+args.name
    os.environ['ROS_DOMAIN_ID'] = '95'
    world = prepare(args, folder)
    (folder / 'config.json').write_text(json.dumps(dict(vars(args), world_sha256=sha(world),
                                                      model_sha256=sha(args.model)), indent=2))
    streams = []
    def listen(label, topic):
        raw = (folder / (label+'.jsonl')).open('w')
        err = (folder / (label+'.stderr')).open('w')
        process = subprocess.Popen(['gz', 'topic', '-e', '--json-output', '-t', topic],
                                   stdout=raw, stderr=err)
        streams.append((process, raw, err))
    log = (folder / 'server.log').open('w')
    server = subprocess.Popen(['gz', 'sim', '-s', '-v', '4', str(world)],
                              stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    deadline = time.monotonic()+600
    service = '/world/r5_check/control'
    def control(request):
        answer = command('gz', 'service', '-s', service, '--reqtype', 'gz.msgs.WorldControl',
                         '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', request)
        if 'data: true' not in answer:
            raise RuntimeError(answer)
    try:
        while service not in command('gz', 'service', '-l'):
            if server.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('world unavailable')
            time.sleep(.2)
        listen('truth', '/model/mooncar/pose')
        listen('joints', 'joint_states')
        listen('input', '/cmd_vel')
        for name in ('front_left', 'front_right', 'rear_left', 'rear_right'):
            link = name+'_wheel_link'
            listen(name+'_contact', f'/world/r5_check/model/mooncar/link/{link}/sensor/{link}_contact/contact')
        time.sleep(1.5)
        command('gz', 'topic', '-t', '/cmd_vel', '-m', 'gz.msgs.Twist', '-p', 'linear: {x: 0}, angular: {z: 0}')
        time.sleep(.3)
        count = round(args.seconds / args.step)
        control(f'pause: true, multi_step: {count}')
        while True:
            stats = last_message(command('gz', 'topic', '-e', '--json-output', '-n', '1',
                                          '-t', '/world/r5_check/stats'))
            if int(stats.get('iterations', 0)) >= count and stats.get('paused'):
                break
            if time.monotonic() > deadline:
                raise TimeoutError('600 second case deadline')
            time.sleep(.1)
        time.sleep(.5)
    finally:
        for process, raw, err in streams:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
            raw.close(); err.close()
        os.killpg(server.pid, signal.SIGINT)
        try:
            server.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(server.pid, signal.SIGKILL); server.wait()
        log.close()
    samples = body_samples(load(folder/'truth.jsonl'))
    result = stability(samples)
    result.update(case=args.name, raw_counts={p.stem: sum(1 for _ in p.open()) for p in folder.glob('*.jsonl')})
    result['recorded_seconds'] = samples[-1]['t']-samples[0]['t'] if samples else 0
    result['data_complete'] = (result['recorded_seconds'] >= args.seconds-.11 and
                               result['raw_counts'].get('joints', 0) > 0 and
                               result['raw_counts'].get('input', 0) > 0)
    result['passed'] = result['passed'] and result['data_complete']
    (folder/'result.json').write_text(json.dumps(result, indent=2))
    cases[-1].update(status='complete', passed=result['passed'])
    ledger.write_text(json.dumps(cases, indent=2))
    print(json.dumps(result), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--name', required=True)
    parser.add_argument('--model', required=True, help='expanded model SDF')
    parser.add_argument('--backend', choices=ENGINES, default='bullet')
    parser.add_argument('--plugin', help='explicit isolated physics library path')
    parser.add_argument('--step', type=float, choices=(.002, .001, .0005), default=.002)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--world')
    parser.add_argument('--package', default='/source/ros2_ws/src/mooncar_gazebo')
    parser.add_argument('--disable-gimbal', action='store_true')
    parser.add_argument('--disable-drive', action='store_true')
    args = parser.parse_args()
    try:
        run(args)
    except Exception as error:
        folder = Path(args.output)/args.name
        if folder.exists():
            (folder/'capture-error.json').write_text(json.dumps(dict(error=repr(error)), indent=2))
        raise


if __name__ == '__main__':
    main()
