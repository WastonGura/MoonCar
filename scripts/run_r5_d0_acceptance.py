#!/usr/bin/env python3
"""Fresh-world bounded D=0 validation; never changes production defaults."""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from check_r5_physics import sha
from r5_d0_analysis import compare_pair,choose_step
from r5_capture_io import capture_hash,image_options,image_argv

SOURCES=('check_r5_runtime.py','check_r5_physics.py','r5_runtime_witness.py',
         'experiment_motion.py','r5_control_analysis.py','r5_d0_analysis.py','run_r5_d0_acceptance.py',
         'r5_step_schedule.py','run_r5_schedule_calibration.py','r5_capture_io.py','r5_capture_probe.py','r5_capture_analysis.py',
         'run_r5_capture_campaign.py','r5_image_trace.py','run_r5_rgb_gap_campaign.py',
         'r5_image_delivery.py','r5_delivery_transport.py','run_r5_image_delivery_campaign.py','r5_rtps_headers.py',
         'r5_dds_shm.py','run_r5_dds_shm_campaign.py','preflight_r5_dds_observer.py')
class CampaignStop(RuntimeError):pass

def check_budget(entries):
    if len(entries)>=34:raise CampaignStop('34 formal rounds exhausted')

def campaign_exit_code(status):
    """Completing all rounds is not sufficient to pass formal acceptance."""
    return 0 if status=='passed' else 1

def validate_calibration(folder,plan):
    if not folder:raise CampaignStop('Paused steps require fresh scheduling calibration')
    entries=json.loads((Path(folder)/'calibration-ledger.json').read_text())
    if not 2<=len(entries)<=4:raise CampaignStop('Calibration attempt budget or coverage invalid')
    signature=('model_sha256','plugin_sha256','observer_sha256','source_sha256')
    if 'capture_config_sha256' in plan:signature+=('capture_config_sha256',)
    for step in (.001,.0005):
        passed=[e for e in entries if e.get('passed') and e['step']==step and all(e['signature'].get(k)==plan[k] for k in signature)]
        if not passed:raise CampaignStop('Missing same-signature successful calibration for '+str(step))
    return len(entries)

def repeatability(results):
    out=[]
    for step in (.001,.0005,.00025):
        for motion in ('straight','left','right'):
            pair=[r for r in results if r['case'].get('motion')==motion and r['case']['step']==step]
            if len(pair)==2:out.append(dict(step=step,motion=motion,**compare_pair(pair[0]['result'],pair[1]['result'])))
    return out

def run(args):
    root=Path(args.output);root.mkdir(parents=True)  # refuse any old evidence
    if shutil.disk_usage(root).free<12*1024**3:raise CampaignStop('Need 12 GiB free evidence space')
    source=root/'source';source.mkdir()
    for name in SOURCES:shutil.copy2(Path(__file__).with_name(name),source/name)
    model=root/'r5-d0.sdf';baseline=Path(args.model);tree=ET.parse(baseline)
    controllers=tree.findall("./model/plugin[@name='gz::sim::systems::JointPositionController']")
    original={'p_gain':20.,'i_gain':.1,'d_gain':.5,'i_max':1.,'i_min':-1.,'cmd_max':5.,'cmd_min':-5.}
    if len(controllers)!=2 or any(float(c.findtext(k))!=v for c in controllers for k,v in original.items()):
        raise CampaignStop('Baseline differs from frozen original R5 controller')
    for c in controllers:c.find('d_gain').text='0.0'
    tree.write(model);shutil.copy2(baseline.with_suffix('.urdf'),model.with_suffix('.urdf'))
    plan=dict(profile=dict(P=20,I=.1,D=0,integral_limit_nm=1,torque_limit_nm=5),
              baseline_model_sha256=sha(baseline),model_sha256=sha(model),plugin_sha256=sha(args.plugin),
              observer_sha256=sha(args.observer_plugin),source_sha256={p.name:sha(p) for p in source.glob('*.py')},
              wall_limit_seconds=600,max_formal_rounds=34,max_capture_repair_rounds=2,
              expected_rounds_1ms=22,expected_rounds_05ms=32,publication=False,
              sensor_archive=dict(rgb_hz=1,depth_hz=1,laser_hz=2,imu='all received',joint='30 Hz',truth='100 Hz',physics='every step'),
              command_schedule_tolerance_s=.05,paired_duration_tolerance_s=.01,
              motion_schedule=getattr(args,'motion_schedule','legacy'),max_total_attempts=38,
              capture_profile=getattr(args,'capture_profile','legacy'),capture_config_sha256=capture_hash(getattr(args,'capture_profile','legacy'),**image_options(args)))
    if plan['motion_schedule']=='paused_steps':
        plan['calibration_attempts']=validate_calibration(args.calibration_dir,plan)
        plan['command_schedule_tolerance_s']='one physical step'
        plan['paired_duration_tolerance_s']='one coarser physical step'
    (root/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    entries=[];results=[];comparisons={};choice=None;status='running';cloud=False;scenes=False
    def save():
        summary=dict(status=status,passed=status=='passed',publication=False,
                     cloud_passed=cloud,turn_passed=bool(choice and choice['passed']),scene_runs_passed=scenes,
                     step_choice=choice,comparisons=comparisons,repeatability=repeatability(results),
                     formal_rounds=len(entries),results=results)
        (root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        (root/'formal-ledger.json').write_text(json.dumps(entries,indent=2)+'\n')
    def case(scene,kind,step,repeat,motion=None,linear=.1,angular=0.):
        check_budget(entries)
        name=f'{len(entries)+1:02d}-{scene}-{motion or kind}-{int(step*1e6)}us-r{repeat}'
        spec=dict(name=name,scene=scene,kind=kind,step=step,repeat=repeat,motion=motion,linear=linear,angular=angular)
        entries.append(dict(case=spec,started=time.time(),status='started'));save()
        argv=[sys.executable,str(source/'check_r5_runtime.py'),'--output',str(root),'--name',name,
              '--model',str(model),'--plugin',args.plugin,'--observer-plugin',args.observer_plugin,
              '--package',args.package,'--formal-d0','--scene',scene,'--kind',kind,'--step',str(step),
              '--linear',str(linear),'--angular',str(angular),'--motion-schedule',plan['motion_schedule'],
              '--capture-profile',plan['capture_profile']]
        with (root/(name+'.log')).open('w') as stream:
            argv=argv+image_argv(args)
            child=subprocess.Popen(argv,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            try:child.wait(timeout=590)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                entries[-1].update(finished=time.time(),status='wall_timeout');save()
                raise CampaignStop('Round wall timeout; exclusive container must be stopped')
        path=root/name/'result.json'
        entries[-1].update(finished=time.time(),exit_code=child.returncode)
        if not path.is_file():
            entries[-1]['status']='capture_failed';save();raise CampaignStop('Missing result: '+name)
        result=json.loads(path.read_text());result['config']=json.loads((root/name/'config.json').read_text())
        entries[-1].update(status='passed' if result['passed'] else 'failed',passed=result['passed'])
        results.append(dict(case=spec,result=result));save()
        print(json.dumps(dict(name=name,passed=result['passed'],wall_seconds=result['wall_seconds'],
                              command=result.get('formal',{}).get('command_window'))),flush=True)
        if not(result.get('data_complete') and result.get('observer_flush_confirmed') and
               result.get('step_observation',{}).get('complete') and result.get('formal',{}).get('data_complete')):
            raise CampaignStop('Critical capture failure: '+name)
        if child.returncode!=0 or result.get('error'):
            raise CampaignStop('Runtime/physical failure: '+name)
        if not result.get('formal',{}).get('command_window',{}).get('passed'):
            raise CampaignStop('Received command scheduling anomaly: '+name)
        return result
    def stationary(step):
        for kind in ('static','gimbal-sweep'):
            for repeat in (1,2):
                if not case('flat',kind,step,repeat)['passed']:
                    raise CampaignStop('Static/gimbal gate failed')
    def motions(step):
        for motion,linear,angular in (('straight',.1,0.),('left',0.,.2),('right',0.,-.2)):
            for repeat in (1,2):case('flat','drive',step,repeat,motion,linear,angular)
    def compare(a,b):
        pairs=[]
        for motion in ('straight','left','right'):
            for repeat in (1,2):
                selected=[]
                for step in (a,b):
                    matching=[r for r in results if r['case'].get('motion')==motion and r['case']['step']==step and r['case']['repeat']==repeat]
                    selected.append(matching[0]['result'])
                signature=('model_sha256','plugin_sha256','observer_sha256','source_sha256','capture_config_sha256')
                same=all(selected[0]['config'][k]==selected[1]['config'][k] for k in signature)
                verdict=compare_pair(*selected) if same else dict(passed=False,valid=False,reason='Frozen source/model/plugin differs')
                pairs.append(dict(motion=motion,repeat=repeat,**verdict))
        return pairs
    try:
        stationary(.001);cloud=True;save()
        motions(.001);motions(.0005)
        comparisons['1ms_05ms']=compare(.001,.0005);choice=choose_step(comparisons['1ms_05ms']);save()
        if plan['motion_schedule']=='paused_steps' and not all(p.get('valid') for p in comparisons['1ms_05ms']):
            raise CampaignStop('Invalid primary input pairing; do not run smaller timestep to bypass scheduling failure')
        if choice['status']=='needs_025ms_reference':
            motions(.00025)
            comparisons['05ms_025ms']=compare(.0005,.00025)
            if plan['motion_schedule']=='paused_steps' and not all(p.get('valid') for p in comparisons['05ms_025ms']):
                raise CampaignStop('Invalid reference input pairing')
            choice=choose_step(comparisons['1ms_05ms'],comparisons['05ms_025ms']);save()
        if choice['step']==.0005:stationary(.0005)
        scene_results=[]
        for scene in ('ramp','curve','survey'):
            for repeat in (1,2):scene_results.append(case(scene,'drive',choice['step'],repeat))
        scenes=all(r['passed'] for r in scene_results)
        status='passed' if cloud and choice['passed'] and scenes else 'completed_not_passed'
    except CampaignStop as failure:
        status='stopped';(root/'stop-reason.txt').write_text(str(failure)+'\n')
    finally:save()
    print(json.dumps(dict(status=status,cloud_passed=cloud,scene_passed=scenes,step_choice=choice,rounds=len(entries))),flush=True)
    return campaign_exit_code(status)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True);p.add_argument('--model',required=True);p.add_argument('--plugin',required=True)
    p.add_argument('--observer-plugin',required=True);p.add_argument('--package',default='/source/ros2_ws/src/mooncar_gazebo')
    p.add_argument('--motion-schedule',choices=('legacy','paused_steps'),default='legacy')
    p.add_argument('--calibration-dir')
    p.add_argument('--capture-profile',choices=('legacy','async'),default='legacy')
    p.add_argument('--image-trace',choices=('off','headers'),default='off')
    p.add_argument('--image-sampling',choices=('legacy','nanoseconds'),default='legacy')
    p.add_argument('--image-observer')
    from r5_image_delivery import delivery_arguments
    delivery_arguments(p)
    sys.exit(run(p.parse_args()))
