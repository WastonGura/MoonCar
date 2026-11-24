#!/usr/bin/env python3
"""New bounded matrix; fixed physical/contact model, one derived driver change."""
import json,os,signal,subprocess,sys,time
from pathlib import Path
from run_r5_origin_campaign import signature,sha,pair_inputs
from run_r5_slip_probe import MATRIX


def freeze():
    f=signature()
    f['allocator_models']={p.name:sha(p) for p in Path('/evidence/allocator-models').iterdir() if p.is_file()}
    f['fixed_contact_models']={p.name:sha(p) for p in Path('/evidence/slip-models').iterdir() if p.is_file()}
    f['allocator_source']={p.name:sha(p) for p in Path('/source/containers/r5_axle_allocation').iterdir() if p.is_file()}
    f['slip_observer_source']={p.name:sha(p) for p in Path('/source/containers/r5_slip_observer').iterdir() if p.is_file()}
    f['allocator_library']=sha('/evidence/allocator/libmooncar-axle-allocation.so')
    f['scope']=sha('/evidence/scope.json');f['prediction']=sha('/evidence/axle-prediction.json')
    f['execution_shell']=sha('/source/scripts/execute_r5_axle_allocation.sh')
    return f


def receipt(folder):
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            maps=(p/'maps').read_text()
            if 'libmooncar-axle-allocation.so' not in maps:continue
            paths=set(line.split()[-1] for line in maps.splitlines() if '/' in line and
                      any(k in line for k in ('axle-allocation','wheel-slip','dartsim','origin-observer','libdart.so','libgz-sim8.so')))
            r=dict(pid=int(p.name),wall=time.time(),libraries={n:sha(n) for n in paths if Path(n).is_file()},
                   axle_loaded=True,slip_loaded=any('wheel-slip' in n for n in paths))
            (folder/'actual-gazebo-libraries.json').write_text(json.dumps(r,indent=2)+'\n');return r
        except (FileNotFoundError,ProcessLookupError,PermissionError):continue
    return None


def compare(old,new):
    a=old['axle_probe']['trajectory'];b=new['axle_probe']['trajectory']
    contact=new['axle_probe']['contact'];loss=abs(a['unwrapped_turn_rad'])-abs(b['unwrapped_turn_rad'])
    return dict(passed=bool(b['maximum_displacement_m']<=.01 and loss<=.05 and
        abs(contact['icr_body_x_median_m'])<=.002 and new['axle_probe']['longitudinal_slip_guard_passed']),
        maximum_displacement_m=b['maximum_displacement_m'],yaw_loss_rad=loss,
        icr_body_x_m=contact['icr_body_x_median_m'],whole_vehicle_accepted=False)


def run(output):
    folder=Path(output);folder.mkdir();frozen=freeze()
    if frozen['resources']!=json.loads(Path('/evidence/baseline/resources.json').read_text()):raise RuntimeError('Resource change')
    if frozen['libraries']['/opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so']!='cfe9b7c6bf72c204645df81f1be7bc646ffbd342acea52c0684a1ad24848274c':raise RuntimeError('DART library changed')
    (folder/'freeze.json').write_text(json.dumps(frozen,indent=2)+'\n')
    ledger=[];results=[];summary=dict(status='running',attempts=0,limit=12,production_changed=False,physical_parameters_changed=False,goal_achieved=False)
    def save():
        summary['attempts']=len(ledger);(folder/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n')
        (folder/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    try:
        for profile,case,linear,angular in MATRIX:
            if freeze()!=frozen:raise RuntimeError('Frozen execution changed')
            name=f'{len(ledger)+1:02d}-{profile}-{case}'
            e=dict(name=name,profile=profile,case=case,linear=linear,angular=angular,started_wall=time.time());ledger.append(e);save()
            argv=[sys.executable,'/source/scripts/check_r5_axle_round.py','--output',str(folder),'--name',name,
                  '--profile',profile,'--linear',str(linear),'--angular',str(angular)]
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
                     error=r.get('axle_probe_error') or r.get('origin_error') or r.get('production_error') or r.get('error'))
            results.append(r);save();print(json.dumps(e),flush=True)
            if not e['qualified']:raise RuntimeError('Qualification failed; no reruns: '+name)
            if len(results)==10:
                pairs=[pair_inputs(results[2],results[i]) for i in (6,8)]+[pair_inputs(results[3],results[i]) for i in (7,9)]
                comparisons=[compare(results[2],results[i]) for i in (6,8)]+[compare(results[3],results[i]) for i in (7,9)]
                summary.update(input_pairs=pairs,comparisons=comparisons)
                if not all(x['passed'] for x in pairs+comparisons):raise RuntimeError('Fixed candidate failed prediction/input gate; no search')
        pairs=[pair_inputs(results[2],results[10]),pair_inputs(results[3],results[11])]
        recurrence=all(abs(results[a]['axle_probe']['trajectory']['maximum_displacement_m']-results[b]['axle_probe']['trajectory']['maximum_displacement_m'])<=.001 for a,b in ((2,10),(3,11)))
        summary.update(status='diagnostic_complete',restore_input_pairs=pairs,
                       cause_confirmed=bool(recurrence and all(x['passed'] for x in pairs)),
                       one_cm_turn_screen_passed=True,whole_vehicle_accepted=False)
    except Exception as error:summary.update(status='stopped',reason=repr(error))
    finally:
        summary['freeze_unchanged']=freeze()==frozen
        if not summary['freeze_unchanged']:summary.update(status='stopped',reason='Frozen signature changed')
        save()
    return 0 if summary['status']=='diagnostic_complete' else 1


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);raise SystemExit(run(p.parse_args().output))
