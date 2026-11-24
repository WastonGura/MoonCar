"""One frozen 17-round suspension campaign; stop on acquisition/input failure."""
import argparse,hashlib,json,os,signal,subprocess,sys,time,shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from run_r5_curve_cause import signature
from r5_flat_scene import validate_spec
from r5_suspension_protocol import validate_campaign,validate_environment
from r5_trial_processes import owned,cleanup

def frozen(prepared):
    result=signature(prepared)
    for p in Path('/diagnosis/dds-preflight').glob('*/result.json'):
        result[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    for root in ('/source/containers/r5_suspension_observer',):
        for p in Path(root).glob('*'):
            if p.is_file():result[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    external=list(Path('/oldcause/runtime/prepared/plane-assets').glob('*.obj'))
    external += [Path('/oldclosure/prepared-d0')/scene/'world.sdf' for scene in ('curve-0.001','ramp-0.001')]
    for p in external:result[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    p=Path('/work/suspension-observer/libmooncar-suspension-observer.so');result[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    for root in ('/source/ros2_ws/src','/source/containers','/work/install'):
        for p in Path(root).rglob('*'):
            if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ('.pyc','.log'):
                result[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--prepared',required=True);p.add_argument('--output',required=True)
    p.add_argument('--static-gate',required=True);a=p.parse_args()
    manifest=json.loads((Path(a.prepared)/'manifest.json').read_text())
    validate_campaign(manifest)
    environment=validate_environment(manifest['execution_environment'])
    gatepath=Path(a.static_gate);gate=json.loads(gatepath.read_text())
    analysis=gatepath.parent/'analysis.json'
    if not gate['passed'] or hashlib.sha256(analysis.read_bytes()).hexdigest()!=gate['analysis_sha256']:
        raise ValueError('Static bias diagnosis gate not satisfied')
    if gate.get('physics_profile')!=manifest.get('physics_profile'):raise ValueError('Diagnostic physics profile differs from campaign')
    for spec in manifest['rounds']+[manifest['static_bench']]:validate_spec(spec)
    for spec in manifest['rounds']:
        if spec['rigid']:continue
        publisher=ET.parse(spec['world']).find("world/model[@name='mooncar']/plugin[@name='gz::sim::systems::JointStatePublisher']")
        if len(publisher.findall('joint_name'))!=10:raise ValueError('Suspension TF joint states not declared')
    out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    (out/'environment.json').write_text(json.dumps(environment,indent=2)+'\n')
    def signature():
        value=frozen(a.prepared)
        for file in (gatepath,analysis,gatepath.parent/'freeze.json',out/'environment.json'):
            value[str(file)]=hashlib.sha256(file.read_bytes()).hexdigest()
        return value
    freeze=signature()
    (out/'freeze.json').write_text(json.dumps(freeze,indent=2)+'\n')
    snapshot=out/'snapshot';snapshot.mkdir()
    for source,digest in freeze.items():
        if not (snapshot/digest).exists():shutil.copyfile(source,snapshot/digest)
    ledger=dict(status='running',maximum_attempts=17,attempts=[],static_gate=gate,
                production_changed=False,old_results_not_counted=True,monitoring_profile=manifest['monitoring_profile'])
    def save():(out/'campaign.json').write_text(json.dumps(ledger,indent=2)+'\n')
    save()
    for spec in manifest['rounds']:
        validate_environment(manifest['execution_environment'])
        if signature()!=freeze:ledger.update(status='stopped',reason='Frozen inputs changed');save();return 1
        row=dict(name=spec['name'],started_wall=time.time(),token=out.name+'-'+spec['name']);ledger['attempts'].append(row);save()
        identities={};failure=None;child=None
        with (out/(spec['name']+'.log')).open('x') as log:
            child=subprocess.Popen([sys.executable,'-B','/source/scripts/check_r5_suspension_round.py',
                '--prepared',a.prepared,'--output',a.output,'--name',spec['name']],stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
                env=dict(os.environ,MOONCAR_TRIAL_ID=row['token']))
            row['launcher_pid']=child.pid;save()
            try:
                deadline=time.monotonic()+600
                while child.poll() is None:
                    for identity in owned(row['token']):identities[(identity['pid'],identity['start_ticks'])]=identity
                    if time.monotonic()>deadline:raise TimeoutError('Round deadline exceeded')
                    time.sleep(.1)
            except BaseException as error:failure=repr(error)
            finally:
                row['process_identities']=list(identities.values())
                row['cleanup']=cleanup(row['token'],ET.parse(spec['world']).find('world').get('name'),
                    failure is not None or child.poll()!=0,list(identities.values()),child.poll)
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    failure=failure or 'Trial launcher remains after cleanup; host must stop dedicated container'
                    row['cleanup']['passed']=False
        row.update(exit_code=child.returncode,wall_seconds=time.time()-row['started_wall'])
        q=out/spec['name']/'suspension-qualification.json';row['qualification']=json.loads(q.read_text()) if q.is_file() else dict(diagnostic_qualified=False)
        save();print(json.dumps(row),flush=True)
        if failure or child.returncode or not row['qualification']['diagnostic_qualified'] or not row['cleanup']['passed']:
            ledger.update(status='stopped',reason=failure or ('Cleanup failed: host must stop dedicated container' if not row['cleanup']['passed'] else 'Input/acquisition/runtime qualification failed'),freeze_unchanged=signature()==freeze);save();return 1
    ledger.update(status='completed',freeze_unchanged=signature()==freeze);save();return 0
if __name__=='__main__':raise SystemExit(main())
