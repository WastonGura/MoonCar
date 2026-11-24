#!/usr/bin/env python3
"""Isolated collision-library candidate through the existing real ROS fixture.

Only the gz sim process receives the candidate library path. Test vehicle,
terrain, step scheduling, safety relay, image delivery and drive are unchanged.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

def motion_qualification(diagnostic, result, linear, angular):
    """Keep straight travel separate from the historical in-place-turn screen."""
    if linear==0:
        return dict(passed=bool(diagnostic.get('eligible')),kind='turn',
                    historical_diagnostic_retained=True)
    checks=dict(diagnostic.get('checks',{}))
    required=('data_complete','ground_contact_gate','stop_zero','observer_flush_confirmed',
        'receiver_verified','raw_fixture_passed','capture_drained','image_content',
        'origin_qualification','gate','allocation','contact_source','no_critical_errors','finite_geometry')
    probe=result.get('turn_slip_probe',{})
    endpoint=probe.get('endpoint_xy_m',[])
    straight=bool(angular==0 and linear>0 and len(endpoint)==2 and
                  1<=endpoint[0]<=2.5 and probe.get('max_lateral_m',float('inf'))<=.01)
    underlying=bool(result.get('passed') and result.get('raw_fixture_passed') and
                    not result.get('turn_slip_error') and not result.get('origin_error') and not result.get('error'))
    return dict(passed=bool(all(checks.get(k) is True for k in required) and straight and underlying),
                kind='straight',input_and_capture=all(checks.get(k) is True for k in required),
                straight_geometry=straight,underlying_fixture=underlying,
                historical_diagnostic_retained=True,
                reason='Forward distance is intentional; preserve original forward/lateral limits')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True);parser.add_argument('--name',required=True)
    parser.add_argument('--world',required=True)
    parser.add_argument('--ode-profile',choices=('stock','candidate'),required=True)
    parser.add_argument('--candidate-runtime',required=True)
    parser.add_argument('--angular',type=float,default=.2)
    parser.add_argument('--linear',type=float,default=0)
    parser.add_argument('--round-fixture',default='/evidence/check_r5_v2_offset_round.py')
    parser.add_argument('--manifest',help='Scoped production runtime manifest instead of build manifest')
    a=parser.parse_args()
    out=Path(a.output);folder=out/a.name
    if folder.exists():raise FileExistsError(folder)
    out.mkdir(parents=True,exist_ok=True)
    manifest=json.loads(Path(a.manifest).read_text()) if a.manifest else json.loads((Path(a.candidate_runtime).parent/'manifest.json').read_text())
    if a.manifest:
        manifest['library_sha256']=manifest['libraries']['libode.so.8.0.2']
    library=Path(a.candidate_runtime)/'libode.so.8.0.2'
    if hashlib.sha256(library.read_bytes()).hexdigest()!=manifest['library_sha256']:
        raise RuntimeError('Candidate library hash mismatch')
    original=subprocess.Popen
    stopped=threading.Event();workers=[];loaded=[];launches=[]
    def observe(proc):
        deadline=time.monotonic()+15
        while time.monotonic()<deadline and not stopped.is_set():
            # gz is a launcher; inspect the server and its descendants.
            descendants={proc.pid}
            for _ in range(5):
                for entry in Path('/proc').iterdir():
                    if not entry.name.isdigit():continue
                    try:status=(entry/'status').read_text()
                    except (FileNotFoundError,ProcessLookupError):continue
                    parent=next((int(line.split()[1]) for line in status.splitlines() if line.startswith('PPid:')),None)
                    if parent in descendants:descendants.add(int(entry.name))
            for pid in descendants:
                try:
                    rows=[line for line in Path(f'/proc/{pid}/maps').read_text().splitlines() if '/libode.so.' in line]
                    cmd=Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
                except (FileNotFoundError,ProcessLookupError):continue
                if rows:
                    paths=sorted({line.split()[-1] for line in rows})
                    loaded.append(dict(pid=pid,command=cmd,paths=paths,
                                       hashes={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in paths}))
                    return
            stopped.wait(.05)
    def start(argv,*args,**kwargs):
        is_server=len(argv)>1 and argv[:2]==['gz','sim']
        if is_server:
            env=dict(kwargs.get('env',os.environ))
            if a.ode_profile=='candidate':
                env['LD_LIBRARY_PATH']=str(library.parent)+':'+env.get('LD_LIBRARY_PATH','')
            kwargs['env']=env
        else:
            env=kwargs.get('env',os.environ)
            if str(library.parent) in env.get('LD_LIBRARY_PATH',''):
                raise RuntimeError('ODE candidate path leaked outside Gazebo')
        proc=original(argv,*args,**kwargs)
        launches.append(dict(pid=proc.pid,command=argv,candidate_path_in_environment=str(library.parent) in kwargs.get('env',os.environ).get('LD_LIBRARY_PATH','')))
        if is_server:
            worker=threading.Thread(target=observe,args=(proc,),daemon=True);workers.append(worker);worker.start()
        return proc
    import check_r5_turn_slip_round
    old_round=check_r5_turn_slip_round.main
    def motion_round():
        # The reused historical diagnostic entry fixes linear to zero; allow
        # this declared zero/straight side-effect check, before its parser runs.
        sys.argv[sys.argv.index('--linear')+1]=str(a.linear)
        return old_round()
    check_r5_turn_slip_round.main=motion_round
    fixture=Path(a.round_fixture)
    spec=importlib.util.spec_from_file_location('coplanar_fixture',fixture)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    old_argv=sys.argv
    subprocess.Popen=start
    error=None;code=1
    try:
        sys.argv=['round','--output',str(out),'--name',a.name,'--world',a.world,'--angular',str(a.angular)]
        code=module.main()
    except Exception as exception:error=repr(exception)
    finally:
        sys.argv=old_argv;subprocess.Popen=original;check_r5_turn_slip_round.main=old_round
        stopped.set()
        for worker in workers:worker.join(timeout=2)
    expected=manifest['library_sha256'] if a.ode_profile=='candidate' else 'c511d66333708785c1a22f6cd572c4c84495a535566c9c80eb37d8e0f67bc469'
    library_verified=bool(loaded and all(list(item['hashes'].values())==[expected] for item in loaded))
    diagnostic=folder/'diagnostic-result.json';raw=folder/'result.json'
    qualification=motion_qualification(json.loads(diagnostic.read_text()),json.loads(raw.read_text()),a.linear,a.angular) if diagnostic.is_file() and raw.is_file() else dict(passed=False,reason='Missing result')
    if folder.is_dir():
        (folder/'motion-qualification.json').write_text(json.dumps(qualification,indent=2)+'\n')
    # The historical diagnostic reports code 1 for successful straight travel
    # solely because it applies the turn screen. Its original record is kept.
    accepted_exit=code==0 or (a.linear>0 and qualification['passed'])
    result=dict(name=a.name,profile=a.ode_profile,loaded=loaded,launches=launches,
                library_verified=library_verified,error=error,fixture_exit=code,
                passed=accepted_exit and qualification['passed'] and not error and library_verified,
                motion_qualification=qualification,formal_defaults_changed=False)
    (out/(a.name+'-loader.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('name','profile','library_verified','error','fixture_exit','passed')}),flush=True)
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
