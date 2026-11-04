"""Fail-closed simulator supervisor; repaired ODE is confined to this child."""
import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time
from turn_guard import sha, verify

def loaded_runtime(pid, root, manifest):
    descendants={pid}
    for _ in range(5):
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():continue
            try:lines=(entry/'status').read_text().splitlines()
            except (FileNotFoundError,ProcessLookupError):continue
            parent=next((int(x.split()[1]) for x in lines if x.startswith('PPid:')),None)
            if parent in descendants:descendants.add(int(entry.name))
    for candidate in descendants:
        try:rows=Path(f'/proc/{candidate}/maps').read_text().splitlines()
        except (FileNotFoundError,ProcessLookupError):continue
        paths={x.split()[-1] for x in rows if '/' in x}
        ode={x for x in paths if Path(x).name.startswith('libode.so.')}
        expected=str((Path(root)/'libode.so.8.0.2').resolve())
        if ode:
            if ode!={expected}:raise RuntimeError('Unexpected ODE loaded: '+repr(ode))
            required=[expected,manifest['dart_path']]+[str((Path(root)/n).resolve()) for n in manifest['libraries'] if n!='libode.so.8.0.2']
            if all(n in paths for n in required):
                hashes={n:sha(n) for n in required}
                for name,digest in hashes.items():
                    expected_digest=manifest['dart_sha256'] if name==manifest['dart_path'] else manifest['libraries'][Path(name).name]
                    if digest!=expected_digest:raise RuntimeError('Actual loaded steering library checksum mismatch: '+name)
                return dict(pid=candidate,libraries=hashes)
    return None

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--world',required=True)
    p.add_argument('--gz-args',required=True);p.add_argument('--receipt')
    p.add_argument('--profile',choices=('r5_v2','r5_support_v1'),default='r5_v2')
    a=p.parse_args()
    if a.profile=='r5_support_v1':
        from support_profile import verify as verify_support
        m=verify_support(a.root)
    else:m=verify(a.root)
    env=dict(os.environ);env['LD_LIBRARY_PATH']=str(Path(a.root).resolve())+':'+env.get('LD_LIBRARY_PATH','')
    child=subprocess.Popen(['gz','sim']+shlex.split(a.gz_args)+[a.world],env=env,start_new_session=True)
    def stop(sig,_):
        if child.poll() is None:os.killpg(child.pid,sig)
    old={s:signal.signal(s,stop) for s in (signal.SIGINT,signal.SIGTERM)}
    try:
        end=time.monotonic()+20
        while child.poll() is None:
            proof=loaded_runtime(child.pid,a.root,m)
            if proof:
                if a.profile=='r5_support_v1':
                    import xml.etree.ElementTree as ET
                    actual_step=float(ET.parse(a.world).findtext('./world/physics/max_step_size','nan'))
                    if actual_step!=.00025:raise RuntimeError('Support candidate actual world step mismatch')
                    proof.update(profile=a.profile,main_step_s=actual_step,world_sha256=sha(a.world))
                    if a.receipt:Path(a.receipt).with_suffix('.world.sdf').write_bytes(Path(a.world).read_bytes())
                if a.receipt:Path(a.receipt).write_text(json.dumps(proof,indent=2)+'\n')
                print('MoonCar steering runtime ready: '+json.dumps(proof),flush=True)
                return child.wait()
            if time.monotonic()>end:raise TimeoutError('Steering runtime actual-load receipt timeout')
            time.sleep(.05)
        raise RuntimeError('Simulator exited before steering runtime ready')
    finally:
        # A launcher can exit while its server remains: always signal the group.
        try:os.killpg(child.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        try:child.wait(5)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid,signal.SIGKILL);child.wait()
        for s,h in old.items():signal.signal(s,h)

if __name__=='__main__':raise SystemExit(main())
