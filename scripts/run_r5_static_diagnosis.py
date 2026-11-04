"""Freeze and execute one predeclared mechanics batch; never retry a trial."""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import time
from pathlib import Path
from run_r5_suspension import frozen
from r5_trial_processes import cleanup,owned
from r5_monitoring import validate_monitoring

PROBE='/work/suspension-observer/libmooncar-native-spring-probe.so'


def signature(prepared,diagnostic):
    values=frozen(prepared)
    paths=list(Path(diagnostic).rglob('*.sdf'))+[Path(diagnostic)/'manifest.json',Path(PROBE),
          Path('/work/suspension-observer/libmooncar-idle-slip-policy.so'),
          Path('/opt/ros/jazzy/opt/gz_dartsim_vendor/lib/libdart.so.6.13.2')]
    for path in paths:values[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    return values


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--prepared',required=True);parser.add_argument('--diagnostic',required=True)
    args=parser.parse_args();out=Path(args.diagnostic)
    manifest=json.loads((out/'manifest.json').read_text())
    from r5_suspension_protocol import validate_environment
    campaign=json.loads((Path(args.prepared)/'manifest.json').read_text())
    validate_environment(campaign['execution_environment'])
    if validate_monitoring(manifest)!=validate_monitoring(campaign):raise ValueError('Diagnostic monitoring profile differs from campaign')
    if manifest['maximum_attempts']!=12 or len(manifest['trials'])!=12:raise ValueError('Exactly twelve trials required')
    if (out/'ledger.json').exists():raise ValueError('Refuse batch retry')
    gatepath=Path('/diagnosis/nonphysics-gate.json');gate=json.loads(gatepath.read_text())
    if not gate['passed'] or gate['freeze']!=frozen(args.prepared):raise ValueError('Nonphysics regressions missing or frozen source changed')
    freeze=signature(args.prepared,args.diagnostic)
    (out/'freeze.json').write_text(json.dumps(freeze,indent=2)+'\n')
    snapshot=out/'snapshot';snapshot.mkdir()
    for source,digest in freeze.items():
        target=snapshot/digest
        if not target.exists():shutil.copyfile(source,target)
    ledger=dict(status='running',maximum_attempts=12,formal_matrix_attempts=0,attempts=[],production_changed=False,monitoring_profile=manifest['monitoring_profile'])
    def save():(out/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n')
    save()
    for trial in manifest['trials']:
        if signature(args.prepared,args.diagnostic)!=freeze:
            ledger.update(status='stopped',reason='Frozen signature changed');save();return 1
        path=Path(trial['world']);folder=path.parent
        row=dict(name=trial['name'],token=out.name+'-'+trial['name'],started_wall=time.time())
        ledger['attempts'].append(row);save()
        env=dict(os.environ,MOONCAR_TRIAL_ID=row['token'],LD_PRELOAD=PROBE,
                 MOONCAR_NATIVE_SPRING_CSV=str(folder/'native.csv'))
        env['LD_LIBRARY_PATH']='/opt/mooncar/turn/r5-v2:'+env.get('LD_LIBRARY_PATH','')
        process=None;failure=None;seen={};loaded=set()
        try:
            with (folder/'run.log').open('x') as log:
                process=subprocess.Popen(['gz','sim','-s','-r','--iterations',str(trial['iterations']),str(path)],
                    env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                row['launcher_pid']=process.pid;save();deadline=time.monotonic()+180
                while process.poll() is None:
                    for identity in owned(row['token']):
                        seen[(identity['pid'],identity['start_ticks'])]=identity
                        try:
                            for line in Path(f"/proc/{identity['pid']}/maps").read_text().splitlines():
                                name=line.split()[-1]
                                if 'libmooncar-physics7-dartsim' in name or 'libmooncar-native-spring' in name or 'libdart.so' in name:loaded.add(name)
                        except OSError:pass
                    if time.monotonic()>deadline:raise TimeoutError('Trial deadline exceeded')
                    time.sleep(.02)
                row['exit_code']=process.returncode
                if process.returncode:failure='Native simulation failed'
        except BaseException as error:
            failure=repr(error)
        finally:
            row['process_identities']=list(seen.values());row['loaded_libraries']=sorted(loaded)
            row['cleanup']=cleanup(row['token'],trial['world_name'],failure is not None,
                                   list(seen.values()),process.poll if process is not None else None)
            if process is not None:
                try:process.wait(timeout=2)
                except subprocess.TimeoutExpired:pass
            row['wall_seconds']=time.time()-row['started_wall']
            row['cleanup_passed']=row['cleanup']['passed']
            row['loaded_library_hashes']={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in loaded}
            required={'/opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so',PROBE}
            if not required.issubset(loaded):failure=failure or 'Native runtime/probe not observed'
            if not row['cleanup_passed']:failure='Cleanup failed: host must stop dedicated container'
            if failure is None:
                try:
                    from analyze_r5_static_diagnosis import metrics
                    row['metrics']=metrics(trial)
                    if trial['kind']=='joint' and not all(j['mechanics_passed'] for j in row['metrics']['joints'].values()):
                        failure='Single-joint mechanics gate failed'
                except Exception as error:failure='Acquisition/analysis qualification failed: '+repr(error)
            row['failure']=failure;save();print(json.dumps(row),flush=True)
        if failure:
            ledger.update(status='stopped',reason=failure,freeze_unchanged=signature(args.prepared,args.diagnostic)==freeze);save();return 1
    ledger.update(status='completed',freeze_unchanged=signature(args.prepared,args.diagnostic)==freeze);save();return 0
if __name__=='__main__':raise SystemExit(main())
