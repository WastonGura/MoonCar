"""Eight approved fresh-world contact-representation diagnostic attempts."""
import json,os,signal,subprocess,sys,time
from pathlib import Path
from run_r5_turn_slip import signature as old_signature,receipt
from run_r5_origin_campaign import sha
from r5_contact_geometry import MATRIX,inputs,mesh_receipt
from r5_library_readiness import Readiness

def signature():
    f=old_signature()
    f['diagnostic_matrix']=MATRIX
    f['geometry_fixtures']={p.name:sha(p) for p in Path('/probe/fixtures').iterdir() if p.is_file()}
    f['metadata_library']=sha('/probe/metadata/libmooncar-contact-metadata.so')
    f['metadata_source']={p.name:sha(p) for p in Path('/source/containers/r5_contact_metadata').iterdir() if p.is_file()}
    f['geometry_preparation']=sha('/source/scripts/prepare_r5_contact_geometry.sh')
    f['mesh_preflight_source']={p.name:sha(p) for p in Path('/source/containers/r5_contact_geometry').iterdir() if p.is_file()}
    f['mesh_preflight']={p.name:sha(p) for p in Path('/probe/mesh-preflight').iterdir() if p.is_file()}
    proof=json.loads(Path('/probe/mesh-preflight/result.json').read_text())
    if not mesh_receipt(proof):raise ValueError('Native mesh loading gate failed')
    f['authorization']=sha('/probe/authorization.json')
    f['test_startup']={p.name:sha(p) for p in Path('/evidence/test-startup').iterdir() if p.is_file()}
    return f

def run(output):
    auth=json.loads(Path('/probe/authorization.json').read_text())
    if auth.get('budget')!=8 or auth.get('answer')!='允许这8次测试诊断':raise ValueError('Missing approved scope')
    p=Path(output);p.mkdir(exist_ok=False);frozen=signature()
    previous=json.loads(Path('/evidence/campaign/freeze.json').read_text())
    for key in ('resources','libraries','model','runtime','allocator_models','allocator_library','turn_models','turn_library','contact_library'):
        if frozen[key]!=previous[key]:raise ValueError('Established physical runtime changed: '+key)
    (p/'freeze.json').write_text(json.dumps(frozen,indent=2)+'\n')
    ledger=[];results=[];s=dict(status='running',attempts=0,limit=8,production_changed=False,goal_achieved=False)
    def save():
        s['attempts']=len(ledger)
        (p/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n');(p/'summary.json').write_text(json.dumps(s,indent=2)+'\n')
    try:
        for profile,case,angular in MATRIX:
            if signature()!=frozen:raise RuntimeError('Frozen execution changed')
            name=f'{len(ledger)+1:02d}-{profile}-{case}'
            e=dict(name=name,profile=profile,case=case,angular=angular,started_wall=time.time())
            ledger.append(e);save();proof=None;timeout=False
            args=[sys.executable,'/source/scripts/check_r5_contact_geometry.py','--output',str(p),'--name',name,'--ground-profile',profile,'--angular',str(angular)]
            readiness=Readiness(p/(name+'-library-history.jsonl'));child=None
            try:
              with (p/(name+'.log')).open('x') as log:
                child=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                while child.poll() is None:
                    if proof is None and (p/name).is_dir():
                        candidate=receipt(p/name)
                        if candidate:
                            maps=Path('/proc')/str(candidate['pid'])/'maps'
                            paths={line.split()[-1] for line in maps.read_text().splitlines() if 'libmooncar-contact-metadata.so' in line}
                            candidate['metadata_libraries']={path:sha(path) for path in paths}
                            candidate['metadata_loaded']=candidate['metadata_libraries']=={'/probe/metadata/libmooncar-contact-metadata.so':frozen['metadata_library']}
                            candidate['passed']=candidate['passed'] and candidate['metadata_loaded']
                            proof=readiness.observe(candidate)
                            if proof:(p/name/'actual-diagnostic-libraries.json').write_text(json.dumps(proof,indent=2)+'\n')
                    if time.time()-e['started_wall']>570:
                        timeout=True;os.killpg(child.pid,signal.SIGINT)
                        try:child.wait(10)
                        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                        break
                    time.sleep(.5)
            finally:
                readiness.close()
                if child is not None and child.poll() is None:
                    os.killpg(child.pid,signal.SIGINT)
                    try:child.wait(10)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
            rawpath=p/name/'result.json';diagpath=p/name/'diagnostic-result.json'
            r=json.loads(rawpath.read_text()) if rawpath.is_file() else {}
            d=json.loads(diagpath.read_text()) if diagpath.is_file() else {}
            e.update(qualified=bool(d.get('eligible') and child.returncode==0 and not timeout and proof and proof['passed'] and time.time()-e['started_wall']<=600),
                original_vehicle_screen_passed=bool(d.get('geometric_screen_passed')),maximum_displacement_m=d.get('maximum_displacement_m'),
                exit_code=child.returncode,timed_out=timeout,wall_seconds=time.time()-e['started_wall'],
                error=r.get('production_error') or r.get('origin_error') or r.get('turn_slip_error') or r.get('error'))
            results.append(r);save();print(json.dumps(e),flush=True)
            if not e['qualified']:raise RuntimeError('Diagnostic evidence/input failure; no rerun: '+name)
            if len(results)>2:
                pair=inputs(results[(len(results)-1)%2],results[-1]);e['corresponding_input_pair']=pair;save()
                if not pair['passed']:raise RuntimeError('Control budget mismatch')
        s.update(status='diagnostic_complete',geometric_failures=sum(not e['original_vehicle_screen_passed'] for e in ledger),
                 mechanism_confirmed=False,repair_verified=False,whole_vehicle_accepted=False)
    except Exception as error:s.update(status='stopped',reason=repr(error))
    finally:
        s['freeze_unchanged']=signature()==frozen
        if not s['freeze_unchanged']:s.update(status='stopped',reason='Frozen execution changed')
        save()
    return 0 if s['status']=='diagnostic_complete' else 1

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);raise SystemExit(run(p.parse_args().output))
