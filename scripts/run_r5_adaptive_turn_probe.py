#!/usr/bin/env python3
"""Frozen fresh observability qualification. Never silently enables actuation."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def signature():
    paths=list(Path('/source/scripts').glob('*.py'))
    paths+=list(Path('/source/containers/r5_turn_runtime').glob('*'))
    for root in (Path('/source/ros2_ws/src/mooncar_gazebo'),Path('/source/ros2_ws/src/mooncar_description')):
        paths += [p for p in root.rglob('*') if p.suffix in ('.py','.json','.yaml','.xacro','.sdf','.obj','.png','.npz')]
    paths += [Path('/oldclosure/prepared-d0/formal.sdf'),Path('/oldclosure/prepared-d0/round_fixture.py'),
        Path('/opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so'),
        Path('/opt/mooncar/turn/r5-v2/libode.so.8.0.2'),
        Path('/evidence/observer/libmooncar-origin-observer.so'),
        Path('/evidence/contact-observer/libmooncar-contact-observer.so')]
    return {str(p):sha(p) for p in sorted(set(paths)) if p.is_file()}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    parser.add_argument('--previous-attempts',type=int,default=0);a=parser.parse_args()
    if not 0<=a.previous_attempts<=2:raise ValueError('Insufficient remaining four-attempt budget')
    out=Path(a.output);out.mkdir(exist_ok=False)
    frozen=signature();(out/'freeze.json').write_text(json.dumps(frozen,indent=2)+'\n')
    result=dict(status='running',maximum_attempts=4,previous_attempts=a.previous_attempts,attempts=[],production_changed=False,
        active_adaptive_runs=0,old_results_not_used_for_counts=True,
        scope='Fresh sensor-shadow qualification first; unsuitable feedback must stop. No parameter search.')
    def save():(out/'campaign.json').write_text(json.dumps(result,indent=2)+'\n')
    save()
    try:
        for name,scene,angular in (('01-curve-left','curve',.2),('02-curve-right','curve',-.2)):
            if signature()!=frozen:raise RuntimeError('Frozen signature changed; no continuation')
            name=out.name+'-'+name
            entry=dict(name=name,scene=scene,angular=angular,started_wall=time.time());result['attempts'].append(entry);save()
            env=dict(os.environ,GZ_PARTITION=name+'-'+str(os.getpid()))
            with (out/(name+'.log')).open('x') as log:
                child=subprocess.Popen([sys.executable,'/source/scripts/check_r5_adaptive_turn_round.py',
                    '--output',str(out),'--name',name,'--scene',scene,'--angular',str(angular)],
                    stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
                try:child.wait(timeout=570)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGINT)
                    try:child.wait(timeout=15)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                    entry['timeout']=True
            path=out/(name+'-feedback-summary.json')
            summary=json.loads(path.read_text()) if path.is_file() else dict(feedback_ready=False,reason='Missing shadow summary')
            entry.update(exit_code=child.returncode,wall_seconds=time.time()-entry['started_wall'],feedback=summary);save()
            if not summary['feedback_ready'] or child.returncode!=0 or entry.get('timeout'):
                raise RuntimeError('Independent feedback not qualified; adaptive actuation prohibited')
        result.update(status='feedback_qualified_needs_active_validation',reason='Shadow is not an actuation or whole-vehicle pass')
    except Exception as error:result.update(status='stopped',reason=repr(error))
    finally:
        result['freeze_unchanged']=signature()==frozen
        if not result['freeze_unchanged']:result.update(status='stopped',reason='Signature changed')
        save()
    return 0 if result['status']=='feedback_qualified_needs_active_validation' else 1


if __name__=='__main__':raise SystemExit(main())
