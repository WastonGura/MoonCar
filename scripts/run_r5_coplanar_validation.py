#!/usr/bin/env python3
"""Six bounded fresh-world tests; freeze first, stop on data/input/runtime failure."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROUNDS=[('01-stock-left','stock',0.,.2),('02-fixed-left','candidate',0.,.2),
        ('03-restore-left','stock',0.,.2),('04-fixed-right','candidate',0.,-.2),
        ('05-fixed-straight','candidate',.1,0.),('06-fixed-left-repeat','candidate',0.,.2)]

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def qualified_attempt(attempt):
    loader=attempt.get('loader') or {}
    # Keep the historical diagnostic for traceability. The wrapper's typed
    # motion qualification owns the straight-vs-turn distinction.
    motion=loader.get('motion_qualification') or {}
    return bool(attempt.get('exit_code')==0 and attempt.get('qualification') and
                loader.get('passed') is True and motion.get('passed') is True)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True);p.add_argument('--runtime',required=True)
    p.add_argument('--freeze-only',action='store_true');p.add_argument('--execute',action='store_true')
    a=p.parse_args()
    if a.freeze_only==a.execute:raise ValueError('Choose exactly one mode')
    out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    freeze_path=out/'freeze.json';library=Path(a.runtime)/'libode.so.8.0.2'
    if a.freeze_only:
        if freeze_path.exists():raise FileExistsError(freeze_path)
        files=set(Path('/source/scripts').glob('*.py'))
        files.update(Path('/source/containers/r5_coplanar_contact').glob('*'))
        files.add(Path('/source/containers/build_ode_coplanar.py'))
        for root in ('/evidence','/source/ros2_ws/src/mooncar_gazebo','/source/ros2_ws/src/mooncar_control','/work/install'):
            for path in Path(root).rglob('*'):
                if path.is_file() and path.suffix in ('.py','.yaml','.xacro','.urdf','.sdf','.so','.json','.obj','.png','.npz') and '__pycache__' not in str(path):files.add(path)
        files.update([library,library.parent.parent/'manifest.json'])
        freeze=dict(files={str(path):sha(path) for path in sorted(files)},
            rounds=ROUNDS,maximum_new_attempts=6,wall_seconds_per_round=600,
            prediction='Extra internal-edge constraint disappears; motion bias must be measured independently. Original geometry/commands unchanged. Restore must reproduce original behavior.',
            stop_on='Missing data, invalid input, runtime/loader failure, or execution signature change. Geometry magnitude alone is reported, not hidden.',
            vehicle_acceptance=False)
        freeze_path.write_text(json.dumps(freeze,indent=2)+'\n')
        print(f'Frozen {len(files)} input files; no Gazebo started');return 0
    freeze=json.loads(freeze_path.read_text())
    status_path=out/'campaign.json'
    if status_path.exists():raise FileExistsError('Refusing to resume or overwrite prior campaign')
    collision=json.loads((out.parent/'collision-check-verified/result.json').read_text())
    if not collision['passed']:raise RuntimeError('Collision regression did not pass')
    status=dict(status='running',attempts=[],maximum_attempts=6,freeze_sha256=sha(freeze_path),formal_defaults_changed=False)
    def save():status_path.write_text(json.dumps(status,indent=2)+'\n')
    save()
    for name,profile,linear,angular in ROUNDS:
        changes=[path for path,digest in freeze['files'].items() if not Path(path).is_file() or sha(path)!=digest]
        if changes:
            status.update(status='stopped',reason='Frozen input changed',changed=changes);save();return 1
        attempt=dict(name=name,profile=profile,linear=linear,angular=angular,started_wall=time.time())
        status['attempts'].append(attempt);save() # startup failure counts too
        args=[sys.executable,'-B','/source/scripts/check_r5_coplanar_round.py',
            '--output',str(out),'--name',name,'--world','/evidence/worlds/world_restore_x00.sdf',
            '--ode-profile',profile,'--candidate-runtime',a.runtime,
            '--linear',str(linear),'--angular',str(angular)]
        started=time.monotonic()
        try:
            with (out/(name+'.log')).open('w') as log:
                result=subprocess.run(args,stdout=log,stderr=subprocess.STDOUT,timeout=600,check=False)
            attempt['exit_code']=result.returncode
        except Exception as error:
            attempt['error']=repr(error);attempt['exit_code']=None
        attempt['wall_seconds']=time.monotonic()-started
        loader=out/(name+'-loader.json');diag=out/name/'diagnostic-result.json'
        attempt['loader']=json.loads(loader.read_text()) if loader.is_file() else None
        attempt['qualification']=json.loads(diag.read_text()) if diag.is_file() else None
        save();print(json.dumps(dict(name=name,exit_code=attempt['exit_code'],wall_seconds=attempt['wall_seconds'],qualification=attempt['qualification'])),flush=True)
        if not qualified_attempt(attempt):
            status.update(status='stopped',reason='Round failed input/data/runtime qualification',failed_round=name);save();return 1
    status['status']='completed';save();return 0

if __name__=='__main__':raise SystemExit(main())
