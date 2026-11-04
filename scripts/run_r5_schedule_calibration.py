#!/usr/bin/env python3
"""At most four fresh-world attempts to qualify paused-step scheduling."""
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
from run_r5_d0_acceptance import SOURCES
from r5_capture_io import capture_hash,image_options,image_argv


def run(args):
    root=Path(args.output);root.mkdir(parents=True,exist_ok=True)
    ledger=root/'calibration-ledger.json';entries=json.loads(ledger.read_text()) if ledger.exists() else []
    if len(entries)>=4:raise RuntimeError('Four calibration attempts exhausted')
    name=f'attempt-{len(entries)+1:02d}-left-{int(args.step*1e6)}us'
    source=root/(name+'-source');source.mkdir()
    for filename in SOURCES:shutil.copy2(Path(__file__).with_name(filename),source/filename)
    model=root/'r5-d0.sdf'
    if not model.exists():
        tree=ET.parse(args.model);controllers=tree.findall("./model/plugin[@name='gz::sim::systems::JointPositionController']")
        baseline={'p_gain':20.,'i_gain':.1,'d_gain':.5,'i_max':1.,'i_min':-1.,'cmd_max':5.,'cmd_min':-5.}
        if len(controllers)!=2 or any(float(c.findtext(k))!=v for c in controllers for k,v in baseline.items()):
            raise RuntimeError('Original R5 controller differs from fixed baseline')
        for c in controllers:c.find('d_gain').text='0.0'
        tree.write(model);shutil.copy2(Path(args.model).with_suffix('.urdf'),model.with_suffix('.urdf'))
    signature=dict(model_sha256=sha(model),plugin_sha256=sha(args.plugin),observer_sha256=sha(args.observer_plugin),
        capture_config_sha256=capture_hash(getattr(args,'capture_profile','legacy'),**image_options(args)),
        source_sha256={p.name:sha(p) for p in source.glob('*.py')})
    entry=dict(name=name,step=args.step,signature=signature,started=time.time(),status='started');entries.append(entry)
    ledger.write_text(json.dumps(entries,indent=2)+'\n')
    argv=[sys.executable,str(source/'check_r5_runtime.py'),'--output',str(root),'--name',name,'--model',str(model),
        '--plugin',args.plugin,'--observer-plugin',args.observer_plugin,'--package',args.package,'--formal-d0',
        '--scene','flat','--kind','drive','--step',str(args.step),'--linear','0','--angular','.2','--motion-schedule','paused_steps',
        '--capture-profile',getattr(args,'capture_profile','legacy')]
    with (root/(name+'.log')).open('x') as stream:
        argv=argv+image_argv(args)
        child=subprocess.Popen(argv,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:child.wait(timeout=590)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
    path=root/name/'result.json';result=json.loads(path.read_text()) if path.exists() else {}
    entry.update(finished=time.time(),exit_code=child.returncode,passed=bool(result.get('passed') and child.returncode==0),
        status='passed' if result.get('passed') and child.returncode==0 else 'failed',
        reason=result.get('error') or result.get('formal',{}).get('reason'))
    ledger.write_text(json.dumps(entries,indent=2)+'\n');print(json.dumps(entry),flush=True)
    return 0 if entry['passed'] else 1

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('output','model','plugin','observer-plugin'):p.add_argument('--'+name,required=True)
    p.add_argument('--package',default='/source/ros2_ws/src/mooncar_gazebo')
    p.add_argument('--step',type=float,choices=(.001,.0005),required=True)
    p.add_argument('--capture-profile',choices=('legacy','async'),default='legacy')
    p.add_argument('--image-trace',choices=('off','headers'),default='off')
    p.add_argument('--image-sampling',choices=('legacy','nanoseconds'),default='legacy')
    p.add_argument('--image-observer')
    from r5_image_delivery import delivery_arguments
    delivery_arguments(p)
    sys.exit(run(p.parse_args()))
