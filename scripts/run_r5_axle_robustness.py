#!/usr/bin/env python3
"""Bounded new robustness evidence; no historical rounds fill this matrix."""
import json,os,signal,subprocess,sys,time
from pathlib import Path
from r5_axle_robustness import MATRIX
from run_r5_axle_allocation import freeze,receipt
from run_r5_origin_campaign import sha,pair_inputs
from r5_d0_analysis import compare_pair

def signature():
    f=freeze();f['matrix']=list(MATRIX)
    f['robust_prediction']=sha('/evidence/robustness-prediction.json')
    f['robust_execution_shell']=sha('/source/scripts/execute_r5_axle_robustness.sh')
    return f

def run(output):
    folder=Path(output);folder.mkdir();frozen=signature()
    reference=json.loads(Path('/source/reports/evidence/terrain-v2/r5-axle-allocation-20261010/campaign/freeze.json').read_text())
    for key in ('resources','libraries','allocator_models','fixed_contact_models','allocator_source','slip_observer_source','allocator_library','model'):
        if frozen[key]!=reference[key]:raise RuntimeError('Fixed physical/control candidate differs: '+key)
    (folder/'freeze.json').write_text(json.dumps(frozen,indent=2)+'\n')
    ledger=[];results=[];summary=dict(status='running',attempts=0,limit=len(MATRIX),production_changed=False,goal_achieved=False)
    def save():
        summary['attempts']=len(ledger);(folder/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n')
        (folder/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    try:
        for scene,step,case,linear,angular in MATRIX:
            if signature()!=frozen:raise RuntimeError('Frozen execution changed')
            name=f'{len(ledger)+1:02d}-{scene}-{int(step*1e6)}us-{case}'
            e=dict(name=name,scene=scene,step=step,case=case,linear=linear,angular=angular,started_wall=time.time())
            ledger.append(e);save()
            argv=[sys.executable,'/source/scripts/check_r5_axle_robust_round.py','--output',str(folder),'--name',name,
                '--scene',scene,'--step',str(step),'--linear',str(linear),'--angular',str(angular)]
            proof=None;timeout=False
            with (folder/(name+'.log')).open('x') as log:
                child=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                while child.poll() is None:
                    if proof is None and (folder/name).is_dir():proof=receipt(folder/name)
                    if time.time()-e['started_wall']>570:
                        timeout=True;os.killpg(child.pid,signal.SIGINT)
                        try:child.wait(10)
                        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                        break
                    time.sleep(.5)
            path=folder/name/'result.json';r=json.loads(path.read_text()) if path.is_file() else dict(passed=False,error='No result')
            e.update(qualified=bool(r.get('passed') and child.returncode==0 and not timeout and proof and proof['slip_loaded'] and time.time()-e['started_wall']<=600),
                exit_code=child.returncode,timed_out=timeout,wall_seconds=time.time()-e['started_wall'],
                error=r.get('robustness_error') or r.get('axle_probe_error') or r.get('origin_error') or r.get('production_error') or r.get('error'))
            results.append(r);save();print(json.dumps(e),flush=True)
            if not e['qualified']:raise RuntimeError('Critical qualification failed; no rerun: '+name)
            if angular and r['axle_probe']['trajectory']['maximum_displacement_m']>.01:
                raise RuntimeError('Fixed candidate failed previous 1 cm turn screen; no search: '+name)
            if len(results)==13:
                comparisons=[compare_pair(results[i],results[i+7]) for i in range(6)]
                repeats=[pair_inputs(results[i],results[i+1]) for i in (0,2,4,7,9,11)]
                summary.update(step_comparisons=comparisons,repeats=repeats)
                if not all(x['passed'] for x in comparisons+repeats):raise RuntimeError('Step/repeat comparison failed; no reference step scan')
        summary.update(status='robustness_complete',test_candidate_robustness_verified=True,
            whole_vehicle_accepted=False,hardware_calibrated=False)
    except Exception as error:summary.update(status='stopped',reason=repr(error))
    finally:
        summary['freeze_unchanged']=signature()==frozen
        if not summary['freeze_unchanged']:summary.update(status='stopped',reason='Frozen execution changed')
        save()
    return 0 if summary['status']=='robustness_complete' else 1

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);raise SystemExit(run(p.parse_args().output))
