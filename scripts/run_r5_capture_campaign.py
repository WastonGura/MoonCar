#!/usr/bin/env python3
"""Four matched diagnostics then four frozen calibrations; no vehicle acceptance."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from check_r5_physics import sha
from run_r5_d0_acceptance import SOURCES,CampaignStop
from r5_capture_io import capture_hash,image_options,image_argv
from r5_d0_analysis import compare_pair

def qualify_input_pairs(results):
    if len(results)!=4:raise CampaignStop('Four fresh calibrations required')
    pairs=[]
    for repeat in (0,1):
        a,b=results[repeat*2:repeat*2+2]
        same=all(a['signature'][k]==b['signature'][k] for k in a['signature'])
        verdict=compare_pair(a['result'],b['result']) if same else dict(valid=False,passed=False,reason='Signature mismatch')
        pairs.append(dict(repeat=repeat+1,**verdict))
    if not all(p['valid'] for p in pairs):raise CampaignStop('Input budget/timing pairing invalid')
    return pairs

def run(args):
    root=Path(args.output);root.mkdir(parents=True)
    source=root/'source';source.mkdir()
    for name in SOURCES:shutil.copy2(Path(__file__).with_name(name),source/name)
    hashes={p.name:sha(p) for p in source.glob('*.py')}
    plan=dict(source_sha256=hashes,baseline_model_sha256=sha(args.model),plugin_sha256=sha(args.plugin),observer_sha256=sha(args.observer_plugin),
        diagnostics=4,calibrations=4,max_new_attempts=8,wall_limit_seconds=600,publication=False,
        motion_schedule='paused_steps',profile=dict(P=20,I=.1,D=0,integral_limit_nm=1,torque_limit_nm=5))
    (root/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    tree=ET.parse(args.model)
    controllers=tree.findall("./model/plugin[@name='gz::sim::systems::JointPositionController']")
    expected={'p_gain':20.,'i_gain':.1,'d_gain':.5,'i_max':1.,'i_min':-1.,'cmd_max':5.,'cmd_min':-5.}
    if len(controllers)!=2 or any(float(c.findtext(k))!=v for c in controllers for k,v in expected.items()):raise CampaignStop('Baseline controller changed')
    for c in controllers:c.find('d_gain').text='0.0'
    model=root/'r5-d0.sdf';tree.write(model);shutil.copy2(Path(args.model).with_suffix('.urdf'),model.with_suffix('.urdf'))
    ledger=[];results=[];calibrations=[];pairs=[];status='running';reason=None;fixed=False
    if getattr(args,'prior_diagnostics',None):
        prior=Path(args.prior_diagnostics)
        if (prior/'calibration-freeze.json').exists():raise CampaignStop('Cannot resume after formal freeze')
        prior_plan=json.loads((prior/'plan.json').read_text())
        if any(prior_plan[k]!=plan[k] for k in ('baseline_model_sha256','plugin_sha256','observer_sha256')):raise CampaignStop('Prior diagnostic model/library changed')
        ledger=json.loads((prior/'ledger.json').read_text());previous=json.loads((prior/'summary.json').read_text())
        if not 1<=len(ledger)<4 or any(e['stage']!='diagnostic' or e['status']=='started' for e in ledger):raise CampaignStop('Prior diagnostic budget/state invalid')
        results=previous['results']
        for record in results:record['folder']=str(prior/'diagnostics'/record['name'])
        plan['prior_diagnostics']=str(prior);plan['inherited_attempts']=len(ledger)
        (root/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    def save():
        (root/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n')
        (root/'summary.json').write_text(json.dumps(dict(status=status,passed=status=='passed',stop_reason=reason,
            attempts=len(ledger),capture_fixed=fixed,calibration_passed=len(calibrations)==4 and bool(pairs),
            formal_vehicle_acceptance_run=False,publication=False,input_pairs=pairs,results=results),indent=2)+'\n')
    def execute(argv,name,stage,step,profile):
        if len(ledger)>=8:raise CampaignStop('Eight attempts exhausted')
        if any(sha(source/name)!=value for name,value in hashes.items()):raise CampaignStop('Frozen execution source changed')
        entry=dict(name=name,stage=stage,step=step,capture_profile=profile,started=time.time(),status='started');ledger.append(entry);save()
        with (root/(name+'.log')).open('x') as stream:
            argv=argv+image_argv(args)
            child=subprocess.Popen(argv,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            try:child.wait(timeout=590)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                entry.update(status='wall_timeout',passed=False,finished=time.time());save()
                raise CampaignStop('Campaign child exceeded wall budget; stop dedicated container')
        entry.update(finished=time.time(),exit_code=child.returncode)
        return entry
    try:
        for number,profile in enumerate(('legacy','legacy','async','async'),1):
            if number<=len([e for e in ledger if e['stage']=='diagnostic']):continue
            name=f'diagnostic-{number:02d}-{profile}'
            argv=[sys.executable,str(source/'check_r5_runtime.py'),'--output',str(root/'diagnostics'),'--name',name,'--model',str(model),
                '--plugin',args.plugin,'--observer-plugin',args.observer_plugin,'--package',args.package,'--formal-d0','--scene','flat',
                '--kind','drive','--step','.001','--linear','0','--angular','.2','--motion-schedule','paused_steps','--capture-profile',profile]
            entry=execute(argv,name,'diagnostic',.001,profile)
            path=root/'diagnostics'/name/'result.json';result=json.loads(path.read_text()) if path.exists() else {}
            entry.update(status='passed' if result.get('passed') else 'failed',passed=bool(result.get('passed')),error=result.get('error'))
            results.append(dict(stage='diagnostic',name=name,capture_profile=profile,result=result,folder=str(path.parent)));save()
            if result.get('error') or 'sim_started' not in result:raise CampaignStop('Diagnostic execution/collection error: '+name)
            if profile=='async' and not result.get('passed'):raise CampaignStop('Asynchronous diagnostic failed: '+name)
        fixed=True
        freeze=dict(source_sha256=hashes,model_sha256=sha(model),plugin_sha256=sha(args.plugin),observer_sha256=sha(args.observer_plugin),
            capture_config_sha256=capture_hash('async',**image_options(args)),diagnostic_results=[r['name'] for r in results])
        (root/'calibration-freeze.json').write_text(json.dumps(freeze,indent=2)+'\n')
        for number,step in enumerate((.001,.0005,.001,.0005),1):
            name=f'calibration-{number:02d}-{int(step*1e6)}us'
            argv=[sys.executable,str(source/'run_r5_schedule_calibration.py'),'--output',str(root/'calibration'),'--model',args.model,
                '--plugin',args.plugin,'--observer-plugin',args.observer_plugin,'--package',args.package,'--step',str(step),'--capture-profile','async']
            entry=execute(argv,name,'calibration',step,'async')
            cal=json.loads((root/'calibration'/'calibration-ledger.json').read_text())[-1]
            result=json.loads((root/'calibration'/cal['name']/'result.json').read_text())
            entry.update(status=cal['status'],passed=cal['passed'],error=cal.get('reason'))
            same=all(cal['signature'][k]==freeze[k] for k in ('source_sha256','model_sha256','plugin_sha256','observer_sha256','capture_config_sha256'))
            record=dict(stage='calibration',name=cal['name'],step=step,signature=cal['signature'],result=result,folder=str(root/'calibration'/cal['name']))
            results.append(record);save()
            if not same or not cal['passed']:raise CampaignStop('Frozen calibration failed or signature changed: '+name)
            calibrations.append(record)
        pairs=qualify_input_pairs(calibrations);status='passed'
    except CampaignStop as error:status='stopped';reason=str(error)
    finally:save()
    print(json.dumps(dict(status=status,capture_fixed=fixed,calibrations=len(calibrations),attempts=len(ledger),stop_reason=reason)),flush=True)
    return 0 if status=='passed' else 1

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('output','model','plugin','observer-plugin'):p.add_argument('--'+name,required=True)
    p.add_argument('--package',default='/source/ros2_ws/src/mooncar_gazebo')
    p.add_argument('--prior-diagnostics',help='Carry all pre-freeze attempts into the remaining eight-attempt budget; does not overwrite them')
    p.add_argument('--image-trace',choices=('off','headers'),default='off')
    p.add_argument('--image-sampling',choices=('legacy','nanoseconds'),default='legacy')
    p.add_argument('--image-observer')
    from r5_image_delivery import delivery_arguments
    delivery_arguments(p)
    sys.exit(run(p.parse_args()))
