#!/usr/bin/env python3
"""One frozen six-round source/geometry contrast; no rerun or parameter search."""
import json,os,signal,subprocess,sys,time
from pathlib import Path
from run_r5_axle_allocation import freeze
from run_r5_origin_campaign import sha,pair_inputs
from r5_contact_source import MATRIX

def signature():
    f=freeze()
    f['contact_source']={p.name:sha(p) for p in Path('/source/containers/r5_contact_observer').iterdir() if p.is_file()}
    f['contact_library']=sha('/evidence/contact-observer/libmooncar-contact-observer.so')
    f['contact_models']={p.name:sha(p) for p in Path('/evidence/contact-models').iterdir() if p.is_file()}
    f['contact_prediction']=sha('/evidence/contact-prediction.json');f['matrix']=list(MATRIX)
    f['contact_shells']={n:sha('/source/scripts/'+n) for n in ('prepare_r5_contact_source.sh','execute_r5_contact_source.sh')}
    f['contact_tests']=sha('/source/ros2_ws/src/mooncar_gazebo/tests/test_contact_source.py')
    return f

def receipt(folder,profile):
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():continue
        try:
            maps=(process/'maps').read_text()
            if 'libmooncar-contact-observer.so' not in maps:continue
            names=set(line.split()[-1] for line in maps.splitlines() if '/' in line and
                any(n in line for n in ('contact-observer','axle-allocation','wheel-slip','dartsim','origin-observer','libdart.so','libgz-sim8.so')))
            r=dict(pid=int(process.name),libraries={n:sha(n) for n in names if Path(n).is_file()},
                contact_observer_loaded=True,slip_loaded=any('wheel-slip' in n for n in names),expected_slip=profile=='slip')
            r['passed']=r['slip_loaded']==r['expected_slip'] and any('axle-allocation' in n for n in names) and any('origin-observer' in n for n in names)
            (folder/'actual-gazebo-libraries.json').write_text(json.dumps(r,indent=2)+'\n');return r
        except (FileNotFoundError,ProcessLookupError,PermissionError):continue
    return None

def run(output):
    p=Path(output);p.mkdir();frozen=signature()
    reference=json.loads(Path('/source/reports/evidence/terrain-v2/r5-axle-robustness-20261010/campaign/freeze.json').read_text())
    for key in ('resources','libraries','allocator_models','fixed_contact_models','allocator_source','slip_observer_source','allocator_library','model'):
        if frozen[key]!=reference[key]:raise ValueError('Existing physical/control model changed '+key)
    (p/'freeze.json').write_text(json.dumps(frozen,indent=2)+'\n')
    ledger=[];results=[];s=dict(status='running',attempts=0,limit=len(MATRIX),production_changed=False,goal_achieved=False)
    def save():
        s['attempts']=len(ledger);(p/'summary.json').write_text(json.dumps(s,indent=2)+'\n');(p/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n')
    try:
        for profile,case,linear in MATRIX:
            if signature()!=frozen:raise RuntimeError('Frozen execution changed')
            name=f'{len(ledger)+1:02d}-{profile}-{case}';e=dict(name=name,profile=profile,case=case,linear=linear,started_wall=time.time());ledger.append(e);save()
            args=[sys.executable,'/source/scripts/check_r5_contact_source_round.py','--output',str(p),'--name',name,'--profile',profile,'--linear',str(linear)]
            proof=None;timeout=False
            with (p/(name+'.log')).open('x') as log:
                child=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                while child.poll() is None:
                    if proof is None and (p/name).is_dir():proof=receipt(p/name,profile)
                    if time.time()-e['started_wall']>570:
                        timeout=True;os.killpg(child.pid,signal.SIGINT)
                        try:child.wait(10)
                        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                        break
                    time.sleep(.5)
            path=p/name/'result.json';r=json.loads(path.read_text()) if path.is_file() else dict(passed=False,error='No result')
            e.update(qualified=bool(r.get('passed') and child.returncode==0 and not timeout and proof and proof['passed'] and time.time()-e['started_wall']<=600),
                exit_code=child.returncode,wall_seconds=time.time()-e['started_wall'],timed_out=timeout,
                error=r.get('contact_source_error') or r.get('origin_error') or r.get('production_error') or r.get('error'))
            results.append(r);save();print(json.dumps(e),flush=True)
            if not e['qualified']:raise RuntimeError('Critical qualification failed; no rerun: '+name)
        pairs=[pair_inputs(results[1],results[i]) for i in (2,3,4,5)]
        original=[abs(results[i]['contact_source_probe']['endpoint_xy_m'][1]) for i in (1,2,5)]
        slip=[abs(results[i]['contact_source_probe']['endpoint_xy_m'][1]) for i in (3,4)]
        # Frozen diagnostic effect screen, not vehicle acceptance: old <=1cm,
        # slip >=10cm, restored old and valid identical control budgets.
        causal=all(x<=.01 for x in original) and all(x>=.10 for x in slip) and max(original)-min(original)<=.005 and max(slip)-min(slip)<=.005
        s.update(status='diagnostic_complete',input_pairs=pairs,original_lateral_m=original,slip_lateral_m=slip,
            causal_prediction_supported=bool(causal and all(x['passed'] for x in pairs)),
            whole_vehicle_accepted=False,production_changed=False)
        if not all(x['passed'] for x in pairs):raise RuntimeError('Input contrast invalid')
    except Exception as error:s.update(status='stopped',reason=repr(error))
    finally:
        s['freeze_unchanged']=signature()==frozen
        if not s['freeze_unchanged']:s.update(status='stopped',reason='Frozen execution changed')
        save()
    return 0 if s['status']=='diagnostic_complete' else 1

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);raise SystemExit(run(p.parse_args().output))
