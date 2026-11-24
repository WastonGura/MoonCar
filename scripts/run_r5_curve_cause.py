"""Frozen nine-round causal diagnosis. Stop on missing or invalid evidence."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def signature(prepared):
    paths = list(Path('/source/scripts').glob('*.py'))
    for root in ('/source/ros2_ws/src/mooncar_gazebo', '/source/ros2_ws/src/mooncar_description', prepared):
        paths += [p for p in Path(root).rglob('*') if p.is_file() and p.suffix in
                  ('.py', '.sdf', '.urdf', '.json', '.yaml', '.xacro', '.obj', '.npz', '.png')]
    paths += [Path('/oldclosure/prepared-d0/formal.sdf'), Path('/oldclosure/prepared-d0/round_fixture.py'),
              Path('/opt/mooncar/turn/r5-v2/libode.so.8.0.2'),
              Path('/opt/mooncar/turn/r5-v2/libmooncar-axle-allocation.so'),
              Path('/opt/mooncar/turn/r5-v2/libmooncar-turn-slip-control.so'),
              Path('/opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so'),
              Path('/evidence/observer/libmooncar-origin-observer.so'),
              Path('/evidence/contact-observer/libmooncar-contact-observer.so')]
    return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(paths))}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared', required=True)
    parser.add_argument('--output', required=True)
    args=parser.parse_args()
    out=Path(args.output);out.mkdir(parents=True, exist_ok=False)
    manifest=json.loads((Path(args.prepared)/'manifest.json').read_text())
    frozen=signature(args.prepared)
    (out/'freeze.json').write_text(json.dumps(frozen, indent=2)+'\n')
    ledger=dict(status='running', maximum_attempts=manifest['maximum_attempts'], attempts=[],
                production_changed=False, old_results_not_counted=True)
    def save():
        (out/'campaign.json').write_text(json.dumps(ledger, indent=2)+'\n')
    save()
    for spec in manifest['rounds']:
        if signature(args.prepared)!=frozen:
            ledger.update(status='stopped',reason='Frozen input changed');save();return 1
        row=dict(name=spec['name'],started_wall=time.time());ledger['attempts'].append(row);save()
        with (out/(spec['name']+'.log')).open('x') as log:
            child=subprocess.Popen([sys.executable,'-B','/source/scripts/check_r5_curve_cause_round.py',
                '--prepared',args.prepared,'--output',args.output,'--name',spec['name']],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:child.wait(timeout=600)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=15)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                row['timeout']=True
        row.update(exit_code=child.returncode,wall_seconds=time.time()-row['started_wall'])
        p=out/spec['name']/'cause-qualification.json'
        row['qualification']=json.loads(p.read_text()) if p.exists() else {'diagnostic_qualified':False}
        save();print(json.dumps(row),flush=True)
        if child.returncode!=0 or row.get('timeout') or not row['qualification']['diagnostic_qualified']:
            ledger.update(status='stopped',reason='Diagnostic input/acquisition/world/runtime qualification failed');save();return 1
    ledger.update(status='completed',freeze_unchanged=signature(args.prepared)==frozen)
    save();return 0


if __name__=='__main__':
    raise SystemExit(main())
