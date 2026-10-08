#!/usr/bin/env python3
"""Fresh, ten-attempt production launch and capture campaign; stops at first failure."""
import hashlib, json, os, shutil, signal, subprocess, sys, time
from pathlib import Path
import xml.etree.ElementTree as ET

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def signature():
    root=Path('/source');resources=json.loads(Path('/evidence/baseline/resources.json').read_text())
    files=list((root/'scripts').glob('*.py'))+list((root/'ros2_ws/src/mooncar_gazebo/launch').glob('*.py'))+list((root/'containers/image_dds').glob('*.py'))
    return dict(sources={str(p.relative_to(root)):sha(p) for p in files},
        resources={n:sha(next(p for p in (root/n,root/'ros2_ws/src/mooncar_gazebo'/n) if p.is_file())) for n in resources},
        runtime_manifest=sha(Path(os.environ['MOONCAR_IMAGE_DDS_RUNTIME'])/'manifest.json'),
        model=sha('/evidence/r5-before.sdf'),urdf=sha('/evidence/r5-before.urdf'),
        observer={p.name:sha(p) for p in Path('/evidence/observer').iterdir() if p.is_file()},
        dart=sha('/opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so'))

def run(output):
    root=Path(output);root.mkdir();ledger=[];frozen=signature()
    if frozen['resources']!=json.loads(Path('/evidence/baseline/resources.json').read_text()):raise RuntimeError('Resources changed')
    preflight=json.loads(Path('/evidence/derived-preflight/result.json').read_text())
    if not preflight.get('passed') or frozen['runtime_manifest']!=preflight['manifest_sha256']:raise RuntimeError('Derived image preflight missing')
    (root/'freeze.json').write_text(json.dumps(frozen,indent=2)+'\n')
    snapshot=root/'source';snapshot.mkdir()
    for name in frozen['sources']:
        path=snapshot/name;path.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(Path('/source')/name,path)
    model=root/'r5-d0.sdf';tree=ET.parse('/evidence/r5-before.sdf')
    controllers=tree.findall("./model/plugin[@name='gz::sim::systems::JointPositionController']")
    if len(controllers)!=2 or any(float(c.findtext('d_gain'))!=.5 for c in controllers):raise RuntimeError('Baseline controller changed')
    for c in controllers:c.find('d_gain').text='0.0'
    tree.write(model);shutil.copy2('/evidence/r5-before.urdf',model.with_suffix('.urdf'))
    summary=dict(passed=False,attempts=0,calibrations_passed=0,default_published=False,vehicle_acceptance=False)
    def save():
        summary['attempts']=len(ledger)
        summary['calibrations_passed']=sum(e['stage']=='calibration' and e.get('passed',False) for e in ledger)
        (root/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n');(root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    def execute(stage,step=.001,scene='ramp',legacy=False):
        if len(ledger)>=10:raise RuntimeError('Ten attempts exhausted')
        if signature()!=frozen:raise RuntimeError('Execution changed after freeze')
        name=f'{len(ledger)+1:02d}-{stage}-{scene}-{int(step*1e6)}us'
        entry=dict(name=name,stage=stage,step=step,scene=scene,status='started',started=time.time());ledger.append(entry);save()
        if stage=='calibration':
            argv=[sys.executable,str(snapshot/'scripts/check_image_dds_round.py'),'--output',str(root),'--name',name,'--model',str(model),'--step',str(step)]
        else:
            argv=[sys.executable,str(snapshot/'scripts/check_image_dds_launch.py'),'--output',str(root),'--name',name,'--scene',scene]+(['--legacy'] if legacy else [])
        # Imports use frozen source snapshots, while formal launch uses the installed overlay checked before freeze.
        with (root/(name+'.log')).open('x') as log:
            child=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            timeout=False
            try:child.wait(590)
            except subprocess.TimeoutExpired:
                timeout=True;os.killpg(child.pid,signal.SIGINT)
                try:child.wait(10)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        path=root/name/'result.json';result=json.loads(path.read_text()) if path.exists() else {'passed':False,'error':'No result'}
        entry.update(passed=bool(result.get('passed') and child.returncode==0 and not timeout),finished=time.time(),
            exit_code=child.returncode,wall_seconds=time.time()-entry['started'],status='timeout' if timeout else 'completed',
            reason=result.get('error') or result.get('analysis_error'))
        save();print(json.dumps(entry),flush=True)
        if signature()!=frozen:raise RuntimeError('Execution changed after round')
        if not entry['passed']:raise RuntimeError('Frozen attempt failed: '+name)
        return result
    try:
        execute('formal-start');execute('formal-start')
        rounds=[execute('calibration',step=s,scene='flat') for s in (.001,.0005,.001,.0005)]
        from run_r5_capture_campaign import qualify_input_pairs
        summary['input_pairs']=qualify_input_pairs([dict(signature=frozen,result=r) for r in rounds])
        for scene in ('ramp','curve','survey'):execute('scene',scene=scene)
        execute('compatibility',legacy=True)
        summary.update(passed=True,status='passed',reason='All ten fresh attempts and calibration input pairs passed')
    except Exception as error:summary.update(status='stopped',reason=repr(error))
    save();return 0 if summary['passed'] else 1

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();raise SystemExit(run(a.output))
