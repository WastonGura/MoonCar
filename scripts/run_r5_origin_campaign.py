#!/usr/bin/env python3
"""Bounded fresh origin baseline. Candidate execution needs a prior prediction."""
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

BASELINE=(('zero',0.,0.,0.),('straight',.1,0.,0.),
          ('left-1',0.,.2,0.),('right-1',0.,-.2,0.),
          ('left-2',0.,.2,0.),('right-2',0.,-.2,0.),
          ('heading90-left',0.,.2,math.pi/2),('heading90-right',0.,-.2,math.pi/2))

def frozen_files(root):
    files=list((root/'scripts').glob('*.py'))+list((root/'containers/r5_origin_observer').glob('*'))
    files+=list((root/'ros2_ws/src/mooncar_description/urdf').rglob('*.xacro'))
    files+=list((root/'ros2_ws/src/mooncar_gazebo/launch').glob('*.py'))+list((root/'ros2_ws/src/mooncar_gazebo/config').glob('*'))
    files+=list((root/'ros2_ws/src/mooncar_control/scripts').glob('*.py'))
    return {str(p.relative_to(root)):sha(p) for p in files if p.is_file()}

def signature():
    from image_dds_validation import installed_files
    resources=json.loads(Path('/evidence/baseline/resources.json').read_text())
    return dict(source=frozen_files(Path('/source')),installed=installed_files(),
        resources={n:sha(next(p for p in (Path('/source')/n,Path('/source/ros2_ws/src/mooncar_gazebo')/n) if p.is_file())) for n in resources},
        libraries={str(p):sha(p) for p in (Path('/evidence/observer/libmooncar-origin-observer.so'),
            Path('/evidence/observer/mooncar-image-observer'),Path('/opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so'))},
        runtime={str(p):sha(p) for p in Path(os.environ['MOONCAR_IMAGE_DDS_RUNTIME']).rglob('*') if p.is_file()},
        model={n:sha(Path('/evidence')/n) for n in ('r5-d0.sdf','r5-d0.urdf','r5-before.sdf','r5-before.urdf')},
        image=sha('/evidence/image.json'),preflight=sha('/evidence/origin-preflight.json'))

def pair_inputs(a,b):
    wa,wb=[r['formal']['command_window'] for r in (a,b)]
    if 'wheel_budget' not in wa or 'wheel_budget' not in wb:return dict(passed=False,reason='Missing motion budgets')
    da=abs(wa['wheel_budget']['linear_integral_m']-wb['wheel_budget']['linear_integral_m'])
    dw=abs(wa['wheel_budget']['angular_integral_rad']-wb['wheel_budget']['angular_integral_rad'])
    dt=abs(wa['duration_s']-wb['duration_s'])
    return dict(passed=bool(a['passed'] and b['passed'] and da<=.001+1e-9 and dw<=.002+1e-9 and dt<=.001+1e-9),
                distance_difference_m=da,turn_difference_rad=dw,duration_difference_s=dt)

def eligible_candidate(decision):
    return bool(decision.get('specific_error') and decision.get('single_change') and decision.get('prediction') and
                decision.get('unchanged_parameters_verified') and decision.get('frozen_before_execution'))

def run(output):
    folder=Path(output);folder.mkdir();ledger=[]
    if not json.loads(Path('/evidence/origin-preflight.json').read_text())['passed']:raise RuntimeError('Preflight failed')
    frozen=signature()
    if frozen['resources']!=json.loads(Path('/evidence/baseline/resources.json').read_text()):raise RuntimeError('Resources changed')
    (folder/'freeze.json').write_text(json.dumps(frozen,indent=2)+'\n')
    snapshot=folder/'source';snapshot.mkdir()
    for name in frozen['source']:
        dest=snapshot/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(Path('/source')/name,dest)
    summary=dict(attempts=0,budget=22,baseline_required=8,baseline_passed=0,
        cause_confirmed=False,repair_verified=False,turn_performance_accepted=False,production_changed=False,status='running')
    def save():
        summary['attempts']=len(ledger);summary['baseline_passed']=sum(r.get('passed',False) for r in ledger)
        (folder/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n');(folder/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    results=[]
    try:
        for case,linear,angular,yaw in BASELINE:
            if signature()!=frozen:raise RuntimeError('Frozen execution changed')
            name=f'{len(ledger)+1:02d}-{case}';entry=dict(name=name,case=case,started=time.time(),status='started');ledger.append(entry);save()
            argv=[sys.executable,str(snapshot/'scripts/check_r5_origin_round.py'),'--output',str(folder),'--name',name,
                '--model','/evidence/r5-d0.sdf','--step','.001','--linear',str(linear),'--angular',str(angular),'--spawn-yaw',str(yaw)]
            with (folder/(name+'.log')).open('x') as log:
                child=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                timeout=False
                try:child.wait(570)
                except subprocess.TimeoutExpired:
                    timeout=True;os.killpg(child.pid,signal.SIGINT)
                    try:child.wait(10)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
            path=folder/name/'result.json';result=json.loads(path.read_text()) if path.is_file() else dict(passed=False,error='No result')
            entry.update(passed=bool(result.get('passed') and child.returncode==0 and not timeout and time.time()-entry['started']<=600),
                status='timeout' if timeout else 'completed',exit_code=child.returncode,wall_seconds=time.time()-entry['started'],
                reason=result.get('production_error') or result.get('origin_error') or result.get('error') or ('Qualification failed; inspect raw result' if not result.get('passed') else None))
            results.append(result);save();print(json.dumps(entry),flush=True)
            if signature()!=frozen:raise RuntimeError('Frozen execution changed after attempt')
            if not entry['passed']:raise RuntimeError('Critical data/input/physical gate failed; no rerun: '+name)
        summary['repeat_input_pairs']=[pair_inputs(results[2],results[4]),pair_inputs(results[3],results[5])]
        if not all(p['passed'] for p in summary['repeat_input_pairs']):raise RuntimeError('Repeat input qualification failed')
        summary.update(status='baseline_complete_awaiting_evidence_decision',reason='Eight qualified baselines; candidate requires a specific proven error and frozen prediction')
    except Exception as error:summary.update(status='stopped',reason=repr(error))
    save();return 0 if summary['baseline_passed']==8 else 1

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();raise SystemExit(run(a.output))
