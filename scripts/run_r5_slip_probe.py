#!/usr/bin/env python3
"""Bounded fresh baseline/candidate/restoration matrix; no production release."""
import json,os,signal,subprocess,sys,time
from pathlib import Path
from run_r5_origin_campaign import signature,sha,pair_inputs
from r5_slip_probe import compare

MATRIX=(('baseline','zero',0.,0.),('baseline','straight',.1,0.),('baseline','left',0.,.2),('baseline','right',0.,-.2),
        ('candidate','zero',0.,0.),('candidate','straight',.1,0.),('candidate','left-1',0.,.2),('candidate','right-1',0.,-.2),
        ('candidate','left-2',0.,.2),('candidate','right-2',0.,-.2),('restore','left',0.,.2),('restore','right',0.,-.2))


def freeze():
    value=signature()
    value['slip_models']={p.name:sha(p) for p in Path('/evidence/slip-models').iterdir() if p.is_file()}
    value['observer_source']={p.name:sha(p) for p in Path('/source/containers/r5_slip_observer').iterdir() if p.is_file()}
    value['prediction']=sha('/evidence/prediction.json');value['authorization']=sha('/evidence/authorization.json')
    value['execution_shell']=sha('/source/scripts/execute_r5_slip_probe.sh')
    return value


def live_maps(folder):
    """Read actual loaded libraries before Gazebo exits; no state changes."""
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():continue
        try:
            maps=(process/'maps').read_text()
            if 'libmooncar-origin-observer.so' not in maps:continue
            paths=set(line.split()[-1] for line in maps.splitlines() if '/' in line and any(k in line for k in ('wheel-slip','dartsim','libgz-sim8','libdart.so','origin-observer')))
            receipt=dict(pid=int(process.name),wall=time.time(),libraries={p:sha(p) for p in paths if Path(p).is_file()},
                         wheel_slip_loaded=any('wheel-slip' in p for p in paths))
            (folder/'actual-gazebo-libraries.json').write_text(json.dumps(receipt,indent=2)+'\n')
            return receipt
        except (FileNotFoundError,ProcessLookupError,PermissionError):continue
    return None


def run(output):
    folder=Path(output);folder.mkdir();frozen=freeze()
    if frozen['libraries']['/opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so']!='cfe9b7c6bf72c204645df81f1be7bc646ffbd342acea52c0684a1ad24848274c':
        raise RuntimeError('Qualified DART library changed')
    if frozen['resources']!=json.loads(Path('/evidence/baseline/resources.json').read_text()):raise RuntimeError('Resource change')
    (folder/'freeze.json').write_text(json.dumps(frozen,indent=2)+'\n')
    ledger=[];results=[];summary=dict(status='running',attempts=0,limit=12,production_changed=False,goal_achieved=False)
    def save():
        summary['attempts']=len(ledger)
        (folder/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n');(folder/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    try:
        for profile,case,linear,angular in MATRIX:
            if freeze()!=frozen:raise RuntimeError('Frozen code/config changed')
            name=f'{len(ledger)+1:02d}-{profile}-{case}'
            entry=dict(name=name,profile=profile,case=case,linear=linear,angular=angular,started_wall=time.time())
            ledger.append(entry);save()
            argv=[sys.executable,'/source/scripts/check_r5_slip_round.py','--output',str(folder),'--name',name,
                  '--profile',profile,'--linear',str(linear),'--angular',str(angular)]
            proof=None;timeout=False
            with (folder/(name+'.log')).open('x') as log:
                child=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                while child.poll() is None:
                    target=folder/name
                    if proof is None and target.is_dir():proof=live_maps(target)
                    if time.time()-entry['started_wall']>570:
                        timeout=True;os.killpg(child.pid,signal.SIGINT)
                        try:child.wait(10)
                        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                        break
                    time.sleep(.5)
            path=folder/name/'result.json';result=json.loads(path.read_text()) if path.is_file() else dict(passed=False,error='No result')
            loaded=bool(proof and proof['wheel_slip_loaded']==(profile=='candidate'))
            entry.update(qualified=bool(result.get('passed') and child.returncode==0 and not timeout and loaded),
                exit_code=child.returncode,timed_out=timeout,wall_seconds=time.time()-entry['started_wall'],
                library_proof_passed=loaded,error=result.get('slip_probe_error') or result.get('origin_error') or result.get('production_error') or result.get('error'))
            results.append(result);save();print(json.dumps(entry),flush=True)
            if not entry['qualified']:raise RuntimeError('Qualification failed; no reruns: '+name)
            if len(results)==10:
                pairs=[pair_inputs(results[2],results[i]) for i in (6,8)]+[pair_inputs(results[3],results[i]) for i in (7,9)]
                if not all(p['passed'] for p in pairs):raise RuntimeError('Candidate input pairs invalid')
                comparisons=[compare(results[2],results[i]) for i in (6,8)]+[compare(results[3],results[i]) for i in (7,9)]
                summary.update(input_pairs=pairs,comparisons=comparisons)
                if not all(p['mechanism_probe_passed'] for p in comparisons):raise RuntimeError('Candidate mechanism prediction failed; stop without parameter search')
        restores=[pair_inputs(results[2],results[10]),pair_inputs(results[3],results[11])]
        recurrence=all(abs(results[i]['slip_probe']['trajectory']['maximum_displacement_m']-results[j]['slip_probe']['trajectory']['maximum_displacement_m'])<=.005 for i,j in ((2,10),(3,11)))
        summary.update(status='diagnostic_complete',restore_input_pairs=restores,
                       mechanism_confirmed=bool(recurrence and all(p['passed'] for p in restores)),
                       previous_one_cm_screen_passed=all(p['previous_one_cm_screen_passed'] for p in summary['comparisons']),
                       whole_vehicle_accepted=False)
    except Exception as error:summary.update(status='stopped',reason=repr(error))
    finally:
        summary['freeze_unchanged']=freeze()==frozen
        if not summary['freeze_unchanged']:summary.update(status='stopped',reason='Freeze changed')
        save()
    return 0 if summary['status']=='diagnostic_complete' else 1


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',required=True)
    raise SystemExit(run(p.parse_args().output))
