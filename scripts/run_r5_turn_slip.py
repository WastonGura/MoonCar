#!/usr/bin/env python3
"""Single fixed candidate, bounded new matrix and unchanged physical inputs."""
import json,os,signal,subprocess,sys,time
from pathlib import Path
from run_r5_contact_source import signature as source_signature,receipt as source_receipt
from run_r5_origin_campaign import sha,pair_inputs
from r5_d0_analysis import compare_pair

MATRIX=(('always','ramp',.001,'straight',.1,0.),('pure_turn','ramp',.001,'straight-1',.1,0.))+tuple(
    ('pure_turn','flat',step,direction+f'-{repeat}',0.,angular)
    for step in (.001,.0005) for repeat in (1,2) for direction,angular in (('left',.2),('right',-.2)))+(
    ('pure_turn','flat',.0005,'zero',0.,0.),('pure_turn','ramp',.001,'zero',0.,0.),
    ('pure_turn','ramp',.001,'straight-2',.1,0.),('pure_turn','ramp',.001,'left',0.,.2),('pure_turn','ramp',.001,'right',0.,-.2))+tuple(
    ('pure_turn',scene,.001,case,linear,angular) for scene in ('curve','survey')
    for case,linear,angular in (('zero',0.,0.),('straight',.1,0.),('left',0.,.2),('right',0.,-.2)))+(
    ('restore','ramp',.001,'straight',.1,0.),)

def signature():
    f=source_signature();f['turn_library']=sha('/evidence/turn-slip/libmooncar-turn-slip-control.so')
    f['turn_source']={p.name:sha(p) for p in Path('/source/containers/r5_turn_slip_control').iterdir() if p.is_file()}
    f['turn_models']={p.name:sha(p) for p in Path('/evidence/turn-models').iterdir() if p.is_file()}
    f['turn_prediction']=sha('/evidence/turn-prediction.json');f['matrix']=list(MATRIX)
    f['turn_shells']={n:sha('/source/scripts/'+n) for n in ('prepare_r5_turn_slip.sh','execute_r5_turn_slip.sh')}
    f['turn_tests']=sha('/source/ros2_ws/src/mooncar_gazebo/tests/test_turn_slip_control.py')
    return f

def receipt(folder):
    r=source_receipt(folder,'slip')
    if r is None:return None
    maps=Path('/proc')/str(r['pid'])/'maps'
    try:
        paths={line.split()[-1] for line in maps.read_text().splitlines() if 'libmooncar-turn-slip-control.so' in line}
        r['turn_libraries']={p:sha(p) for p in paths};r['passed']=r['passed'] and len(paths)==1
        (Path(folder)/'actual-gazebo-libraries.json').write_text(json.dumps(r,indent=2)+'\n');return r
    except (FileNotFoundError,ProcessLookupError):return None

def run(output):
    p=Path(output);p.mkdir();frozen=signature()
    reference=json.loads(Path('/source/reports/evidence/terrain-v2/r5-contact-source-20261011/campaign/freeze.json').read_text())
    for key in ('resources','libraries','allocator_models','fixed_contact_models','allocator_source','slip_observer_source','allocator_library','model','contact_source','contact_library','contact_models'):
        if frozen[key]!=reference[key]:raise ValueError('Established model/source changed '+key)
    (p/'freeze.json').write_text(json.dumps(frozen,indent=2)+'\n')
    ledger=[];results=[];s=dict(status='running',attempts=0,limit=len(MATRIX),production_changed=False,goal_achieved=False)
    def save():
        s['attempts']=len(ledger);(p/'summary.json').write_text(json.dumps(s,indent=2)+'\n');(p/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n')
    try:
        for profile,scene,step,case,linear,angular in MATRIX:
            if signature()!=frozen:raise RuntimeError('Frozen execution changed')
            name=f'{len(ledger)+1:02d}-{profile}-{scene}-{int(step*1e6)}us-{case}'
            e=dict(name=name,profile=profile,scene=scene,step=step,case=case,linear=linear,angular=angular,started_wall=time.time());ledger.append(e);save()
            args=[sys.executable,'/source/scripts/check_r5_turn_slip_round.py','--output',str(p),'--name',name,'--profile',profile,'--scene',scene,'--step',str(step),'--linear',str(linear),'--angular',str(angular)]
            proof=None;timeout=False
            with (p/(name+'.log')).open('x') as log:
                child=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                while child.poll() is None:
                    if proof is None and (p/name).is_dir():proof=receipt(p/name)
                    if time.time()-e['started_wall']>570:
                        timeout=True;os.killpg(child.pid,signal.SIGINT)
                        try:child.wait(10)
                        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                        break
                    time.sleep(.5)
            path=p/name/'result.json';r=json.loads(path.read_text()) if path.is_file() else dict(passed=False,error='No result')
            e.update(qualified=bool(r.get('passed') and child.returncode==0 and not timeout and proof and proof['passed'] and time.time()-e['started_wall']<=600),
                exit_code=child.returncode,wall_seconds=time.time()-e['started_wall'],timed_out=timeout,
                error=r.get('turn_slip_error') or r.get('origin_error') or r.get('production_error') or r.get('error'))
            results.append(r);save();print(json.dumps(e),flush=True)
            if not e['qualified']:raise RuntimeError('Critical qualification failed; no rerun: '+name)
            if len(results)==11:
                s['step_comparisons']=[compare_pair(results[i],results[i+4]) for i in range(2,6)]
                s['repeat_pairs']=[pair_inputs(results[i],results[i+2]) for i in (2,3,6,7)]
                if not all(x['passed'] for x in s['step_comparisons']+s['repeat_pairs']):raise RuntimeError('Step/repeat comparison failed; no parameter search')
        pairs=[pair_inputs(results[0],results[i]) for i in (1,12,23)]
        lateral=[abs(results[i]['turn_slip_probe']['endpoint_xy_m'][1]) for i in (0,1,12,23)]
        s.update(status='candidate_verified',input_pairs=pairs,side_effect_comparison_m=lateral,
            gated_side_effect_removed=bool(lateral[0]>=.10 and lateral[1]<=.01 and lateral[2]<=.01 and lateral[3]>=.10 and abs(lateral[0]-lateral[3])<=.005 and all(x['passed'] for x in pairs)),
            whole_vehicle_accepted=False,hardware_calibrated=False)
        if not s['gated_side_effect_removed']:raise RuntimeError('Single-change side-effect/restore prediction not supported')
    except Exception as error:s.update(status='stopped',reason=repr(error))
    finally:
        s['freeze_unchanged']=signature()==frozen
        if not s['freeze_unchanged']:s.update(status='stopped',reason='Frozen execution changed')
        save()
    return 0 if s['status']=='candidate_verified' else 1

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);raise SystemExit(run(p.parse_args().output))
