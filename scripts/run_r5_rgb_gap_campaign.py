#!/usr/bin/env python3
"""Bounded image diagnosis and four fresh frozen calibrations; no publication."""
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
from r5_capture_io import capture_hash,image_options,image_argv
from run_r5_d0_acceptance import SOURCES,CampaignStop
from run_r5_capture_campaign import qualify_input_pairs

def check_attempt_budget(entries,stage):
    if stage not in ('diagnostic','calibration'):raise CampaignStop('Unknown campaign stage')
    if len(entries)>=12 or sum(e['stage']==stage for e in entries)>=(4 if stage=='calibration' else 8):
        raise CampaignStop('Attempt budget exhausted')

def run(args):
    root=Path(args.output);root.mkdir(parents=True,exist_ok=True)
    if (root/'campaign-stop.json').exists():raise CampaignStop('Campaign stopped; use a new authorized evidence directory')
    ledger_path=root/'ledger.json'
    entries=json.loads(ledger_path.read_text()) if ledger_path.exists() else []
    freeze_path=root/'calibration-freeze.json'
    frozen=json.loads(freeze_path.read_text()) if freeze_path.exists() else None
    if frozen and any(e['stage']=='calibration' and not e.get('passed') for e in entries):
        raise CampaignStop('Frozen calibration failed; campaign permanently stopped')
    if frozen and not args.calibrate:raise CampaignStop('Diagnostics forbidden after freeze')
    hashes={n:sha(Path(__file__).with_name(n)) for n in SOURCES}
    image_source=Path('/source/containers/r5_image_observer')
    signature=dict(source_sha256=hashes,baseline_model_sha256=sha(args.model),plugin_sha256=sha(args.plugin),
        observer_sha256=sha(args.observer_plugin),native_image_observer_sha256=sha(args.image_observer),
        native_image_source_sha256={p.name:sha(p) for p in image_source.iterdir() if p.is_file()},
        capture_config_sha256=capture_hash('async',**image_options(args)))
    model=root/'r5-d0.sdf'
    if not model.exists():
        tree=ET.parse(args.model);controllers=tree.findall("./model/plugin[@name='gz::sim::systems::JointPositionController']")
        expected={'p_gain':20.,'i_gain':.1,'d_gain':.5,'i_max':1.,'i_min':-1.,'cmd_max':5.,'cmd_min':-5.}
        if len(controllers)!=2 or any(float(c.findtext(k))!=v for c in controllers for k,v in expected.items()):raise CampaignStop('Baseline changed')
        for c in controllers:c.find('d_gain').text='0.0'
        tree.write(model);shutil.copy2(Path(args.model).with_suffix('.urdf'),model.with_suffix('.urdf'))
    resources=json.loads(Path('/source/reports/evidence/terrain-v2/r5-capture-repair-20261007/source-and-resource-hashes.json').read_text())['resources']
    current={n:sha(next(p for p in (Path('/source')/n,Path(args.package)/n) if p.is_file())) for n in resources}
    if current!=resources or len(current)!=70:raise CampaignStop('Frozen scene resources changed')
    signature.update(model_sha256=sha(model),model_urdf_sha256=sha(model.with_suffix('.urdf')),resources_sha256=current)
    if getattr(args,'cpp_image_observer',None):
        tool_source=Path('/source/containers/r5_image_delivery')
        signature['delivery_tool_source_sha256']={p.name:sha(p) for p in tool_source.iterdir() if p.is_file()}
        signature['stock_bridge_sha256']=sha('/opt/ros/jazzy/lib/ros_gz_bridge/bridge_node')
        signature['stock_bridge_library_sha256']=sha('/opt/ros/jazzy/lib/libros_gz_bridge.so')
        if getattr(args,'traced_bridge',None):
            signature['traced_bridge_library_sha256']=sha(Path(args.traced_bridge).parents[1]/'libros_gz_bridge.so')
    if frozen and signature!=frozen:raise CampaignStop('Frozen signature changed')
    if not (root/'plan.json').exists():
        (root/'plan.json').write_text(json.dumps(dict(max_diagnostics=8,max_calibrations=4,max_attempts=12,wall_limit_seconds=600,
            capture_profile='async',image_trace='headers',publication=False,model_sha256=sha(model),initial_signature=signature),indent=2)+'\n')
    def save():ledger_path.write_text(json.dumps(entries,indent=2)+'\n')
    def execute(stage,step,label):
        check_attempt_budget(entries,stage)
        number=len(entries)+1;name=f'{number:02d}-{label}-{int(step*1e6)}us'
        source=root/(name+'-source');source.mkdir()
        for n in SOURCES:shutil.copy2(Path(__file__).with_name(n),source/n)
        entry=dict(name=name,stage=stage,step=step,sampling=args.image_sampling,signature=signature,started=time.time(),status='started')
        entries.append(entry);save()
        argv=[sys.executable,str(source/'check_r5_runtime.py'),'--output',str(root),'--name',name,'--model',str(model),
            '--plugin',args.plugin,'--observer-plugin',args.observer_plugin,'--package',args.package,'--formal-d0',
            '--scene','flat','--kind','drive','--step',str(step),'--linear','0','--angular','.2','--motion-schedule','paused_steps',
            '--capture-profile','async']+image_argv(args)
        with (root/(name+'.log')).open('x') as stream:
            from r5_dds_shm import process_env
            child=subprocess.Popen(argv,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True,
                env=process_env(args,os.environ,root/name,'main'))
            timed_out=False
            try:child.wait(590)
            except subprocess.TimeoutExpired:
                timed_out=True;os.killpg(child.pid,signal.SIGINT)
                try:child.wait(10)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        path=root/name/'result.json';result=json.loads(path.read_text()) if path.exists() else {}
        entry.update(finished=time.time(),exit_code=child.returncode,passed=bool(result.get('passed') and child.returncode==0 and not timed_out),
            status='wall_timeout' if timed_out else ('passed' if result.get('passed') and child.returncode==0 else 'failed'),
            reasons=result.get('error') or result.get('formal',{}).get('sensors') or 'Missing result')
        save();print(json.dumps(dict(name=name,passed=entry['passed'],status=entry['status'])),flush=True)
        return entry,result
    if args.calibrate:
        eligible=[e for e in entries if e['stage']=='diagnostic' and e.get('passed') and e['signature']==signature]
        if len(eligible)<2:raise CampaignStop('Need two complete same-configuration diagnostics before freeze')
        if not frozen:freeze_path.write_text(json.dumps(signature,indent=2)+'\n')
        rounds=[]
        for number,step in enumerate((.001,.0005,.001,.0005),1):
            entry,result=execute('calibration',step,f'calibration-{number}')
            rounds.append(dict(signature=signature,result=result))
            if not entry['passed']:
                (root/'summary.json').write_text(json.dumps(dict(passed=False,status='stopped',reason='Frozen calibration failed',attempts=len(entries),
                    calibration_rounds=len(rounds),publication=False),indent=2)+'\n')
                return 1
        pairs=qualify_input_pairs(rounds)
        from r5_d0_analysis import compare_pair
        repeats=[dict(step=rounds[i]['result']['schedule']['step'],**compare_pair(rounds[i]['result'],rounds[i+2]['result'])) for i in (0,1)]
        (root/'summary.json').write_text(json.dumps(dict(passed=True,status='passed',input_pairs=pairs,same_step_repeatability=repeats,attempts=len(entries),calibration_rounds=4,
            publication=False,vehicle_acceptance=False),indent=2)+'\n')
        return 0
    entry,_=execute('diagnostic',.001,args.label)
    return 0 if entry['passed'] else 1

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    from r5_image_delivery import delivery_arguments
    delivery_arguments(p)
    for n in ('output','model','plugin','observer-plugin','image-observer'):p.add_argument('--'+n,required=True)
    p.add_argument('--package',default='/source/ros2_ws/src/mooncar_gazebo')
    p.add_argument('--image-trace',choices=('headers',),default='headers')
    p.add_argument('--image-sampling',choices=('legacy','nanoseconds'),default='legacy')
    p.add_argument('--label',default='baseline');p.add_argument('--calibrate',action='store_true')
    sys.exit(run(p.parse_args()))
