#!/usr/bin/env python3
"""Four actual contact-response contrasts; diagnostic coefficients are not soil data."""
import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

from check_r5_physics import command, last_message, load, sha, timestamp


def run(output, plugin):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    ledger = output.parent/'diagnostics/diagnostic-ledger.json'
    results = []
    plate = output/'plate.obj'
    plate.write_text('v -20 -20 0\nv 20 -20 0\nv 20 20 0\nv -20 20 0\nvn 0 0 1\nf 1//1 2//1 3//1\nf 1//1 3//1 4//1\n')
    template = ET.fromstring('''<sdf version="1.10"><world name="contact_check">
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"><engine><filename/></engine></plugin>
      <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
      <physics name="entry" type="ode"><max_step_size>.001</max_step_size><real_time_factor>1</real_time_factor></physics>
      <gravity>0 0 -1.62</gravity><model name="plate"><static>true</static><link name="surface">
      <collision name="plate"><pose>0 0 -.05 0 0 0</pose><geometry><box><size>40 40 .1</size></box></geometry>
      <surface><friction><ode><mu/><mu2/></ode></friction></surface></collision></link></model>
      <model name="block"><pose>0 0 .04 0 0 0</pose><link name="body"><inertial><mass>1</mass>
      <inertia><ixx>.0010416666666667</ixx><iyy>.0010416666666667</iyy><izz>.0016666666666667</izz></inertia></inertial>
      <collision name="block"><geometry><box><size>.1 .1 .05</size></box></geometry>
      <surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision></link>
      <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <dimensions>3</dimensions><odom_publish_frequency>100</odom_publish_frequency><odom_topic>/truth</odom_topic>
      <odom_frame>world</odom_frame><gaussian_noise>0</gaussian_noise></plugin></model></world></sdf>''')
    for kind in ('primitive', 'mesh'):
        for mu in (0, 1):
            cases = json.loads(ledger.read_text())
            if len(cases) >= 20:
                raise RuntimeError('20 diagnostic rounds exhausted')
            name = f'{kind}-mu{mu}'
            folder = output/name; folder.mkdir()
            cases.append(dict(name='entry-'+name, status='started', started=time.time()))
            ledger.write_text(json.dumps(cases, indent=2))
            root = deepcopy(template)
            root.find('./world/plugin/engine/filename').text = str(Path(plugin).resolve())
            collision = root.find("./world/model[@name='plate']/link/collision")
            for key in ('mu', 'mu2'):
                collision.find('surface/friction/ode/'+key).text = str(mu)
            if kind == 'mesh':
                collision.find('pose').text = '0 0 0 0 0 0'
                geometry = collision.find('geometry'); geometry.remove(geometry.find('box'))
                ET.SubElement(ET.SubElement(geometry, 'mesh'), 'uri').text = str(plate)
            world = folder/'world.sdf'; ET.ElementTree(root).write(world)
            os.environ['GZ_PARTITION'] = 'mooncar_r5_entry_'+name
            streams = []
            for label, topic in [('truth', '/truth'), ('input', '/world/contact_check/wrench/persistent')]:
                raw = (folder/(label+'.jsonl')).open('w'); err = (folder/(label+'.stderr')).open('w')
                proc = subprocess.Popen(['gz','topic','-e','--json-output','-t',topic], stdout=raw, stderr=err)
                streams.append((proc, raw, err))
            log = (folder/'server.log').open('w')
            server = subprocess.Popen(['gz','sim','-s','-v','4',str(world)], stdout=log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+600
            service = '/world/contact_check/control'
            def step(count, target):
                answer = command('gz','service','-s',service,'--reqtype','gz.msgs.WorldControl',
                                 '--reptype','gz.msgs.Boolean','--timeout','5000','--req',f'pause: true, multi_step: {count}')
                assert 'data: true' in answer, answer
                while True:
                    stats = last_message(command('gz','topic','-e','--json-output','-n','1','-t','/world/contact_check/stats'))
                    if stats.get('paused') and int(stats.get('iterations',0)) >= target:
                        return
                    if time.monotonic() > deadline:
                        raise TimeoutError('contact check deadline')
                    time.sleep(.1)
            try:
                while service not in command('gz','service','-l'):
                    if time.monotonic() > deadline or server.poll() is not None:
                        raise RuntimeError('contact world unavailable')
                    time.sleep(.2)
                time.sleep(1.5); step(2000,2000)
                command('gz','topic','-t','/world/contact_check/wrench/persistent','-m','gz.msgs.EntityWrench',
                        '-p','entity: {name: "block::body", type: LINK}, wrench: {force: {x: .486}}')
                time.sleep(.5); step(8000,10000); time.sleep(.5)
            finally:
                for proc, raw, err in streams:
                    proc.terminate(); proc.wait(timeout=5); raw.close(); err.close()
                os.killpg(server.pid,signal.SIGINT)
                try: server.wait(timeout=5)
                except subprocess.TimeoutExpired: os.killpg(server.pid,signal.SIGKILL); server.wait()
                log.close()
            rows = load(folder/'truth.jsonl')
            start = min(rows, key=lambda r:abs(timestamp(r)-2))
            dx = rows[-1]['pose']['position'].get('x',0)-start['pose']['position'].get('x',0)
            gap = max(timestamp(b)-timestamp(a) for a,b in zip(rows,rows[1:]))
            result = dict(kind=kind, mu=mu, dx_m=dx, samples=len(rows), max_gap_s=gap,
                          plugin_sha256=sha(plugin), world_sha256=sha(world),
                          passed=(dx>5 if mu==0 else abs(dx)<.02) and gap<=.100001 and timestamp(rows[-1])>=9.99)
            (folder/'result.json').write_text(json.dumps(result,indent=2))
            cases[-1].update(status='complete',passed=result['passed']); ledger.write_text(json.dumps(cases,indent=2))
            results.append(result); print(json.dumps(result),flush=True)
    (output/'results.json').write_text(json.dumps(dict(passed=all(r['passed'] for r in results),results=results),indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True); parser.add_argument('--plugin',required=True)
    args=parser.parse_args(); run(args.output,args.plugin)
