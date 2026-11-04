"""Package the checked ODE artifact and compile scoped production drive systems."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--ode',required=True)
    p.add_argument('--ode-build-manifest',help='Archived ODE build or packaged runtime manifest')
    a=p.parse_args();out=Path(a.output);out.mkdir(exist_ok=False)
    source=Path('/source/containers/r5_turn_runtime')
    if sha(a.ode)!='313dbad175a34e0fd0387792bc922766b5e7727c3883ede5b84322cd055140c7':
        raise RuntimeError('Not the independently verified ODE repair')
    build=out/'build'
    for argv in (['cmake','-S',str(source),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release'],['cmake','--build',str(build),'-j2']):
        with (out/('configure.log' if '-S' in argv else 'compile.log')).open('w') as log:
            subprocess.run(argv,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=2700)
    runtime=out/'runtime';runtime.mkdir()
    shutil.copy2(a.ode,runtime/'libode.so.8.0.2');(runtime/'libode.so.8').symlink_to('libode.so.8.0.2')
    for name in ('libmooncar-adaptive-axle-allocation.so','libmooncar-turn-slip-control.so'):shutil.copy2(build/name,runtime/name)
    stock='/usr/lib/x86_64-linux-gnu/libode.so.8.0.2'
    dart='/opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so'
    recorded=json.loads(Path(a.ode_build_manifest).read_text()) if a.ode_build_manifest else json.loads(Path(a.ode).parent.parent.joinpath('manifest.json').read_text())
    ode_build=recorded.get('ode_build_manifest',recorded)
    if ode_build.get('library_sha256') != sha(a.ode):raise RuntimeError('ODE provenance/library mismatch')
    manifest=dict(ode_version='0.16.2',gz_sim_major=8,gz_physics_version='7.8.0',gimbal_changed=True,gimbal_profile=[20,.1,0,1,5],
        allocation_profile='independent_laser_feedback_v1',stock_ode_path=stock,stock_ode_sha256=sha(stock),dart_path=dart,dart_sha256=sha(dart),
        libraries={p.name:sha(p) for p in runtime.glob('*.so*') if not p.is_symlink()},
        sources={p.name:sha(p) for p in source.iterdir() if p.is_file()},
        ode_build_manifest=ode_build,
        scope='Unreleased R5/v2 candidate; sensor-qualified adaptive axle allocation, pure-turn slip and D=0 gimbal')
    (runtime/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest['libraries']))

if __name__=='__main__':main()
