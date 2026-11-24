#!/usr/bin/env python3
"""Fresh isolated sensor shadow before enabling experimental feedback allocation.

Uses preserved physical fixtures and real sensors. A shadow failure is a reason
to stop, not permission to lower feedback quality thresholds.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml
import numpy as np
import hashlib
import xml.etree.ElementTree as ET
from r5_adaptive_turn import rotation


def feedback_accuracy(folder, decisions):
    """Truth is used ONLY in this post-run evaluator, never by the controller."""
    result=json.loads((folder/'result.json').read_text())
    boundary=result['schedule']['motion'];start=boundary['before']['sim'];end=boundary['after']['sim']
    data=np.genfromtxt(folder/'physics-steps.csv',delimiter=',',names=True)
    before=[r for r in decisions if r['stamp_ns']<=round(start*1e9)]
    if not before:return dict(passed=False,reason='Missing independent anchor scan')
    anchor=before[-1];time0=anchor['stamp_ns']*1e-9
    origin=data[np.argmin(abs(data['t']-time0))]
    from r5_d0_analysis import pose
    yaw0=pose(origin)['rpy'][2];xy=np.zeros(2);yaw=0.;errors=[]
    relevant=[r for r in decisions if anchor['stamp_ns']<r['stamp_ns']<=round(end*1e9)]
    for row in relevant:
        if not row.get('registration',{}).get('valid') or 'measured_body_delta_m' not in row:
            return dict(passed=False,reason='Unqualified registration breaks independent trajectory',
                        stamp_ns=row['stamp_ns'],qualified_prefix=len(errors))
        xy+=rotation(yaw) @ np.array(row['measured_body_delta_m'])
        yaw+=row['registration']['yaw']
        truth=data[np.argmin(abs(data['t']-row['stamp_ns']*1e-9))]
        if abs(truth['t']-row['stamp_ns']*1e-9)>.0010001:
            return dict(passed=False,reason='No qualified sensor/truth timestamp pairing')
        measured=rotation(-yaw0)@np.array([truth['x']-origin['x'],truth['y']-origin['y']])
        errors.append(float(np.linalg.norm(xy-measured)))
    maximum=max(errors,default=float('inf'))
    return dict(passed=bool(len(errors)>=180 and maximum<=.01),maximum_position_error_m=maximum,
        compared_samples=len(errors),position_error_limit_m=.01,truth_used_for_evaluation_only=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--name',required=True)
    p.add_argument('--scene',choices=('ramp','curve'),required=True)
    p.add_argument('--angular',type=float,default=.2)
    a=p.parse_args();out=Path(a.output);folder=out/a.name
    if folder.exists():raise FileExistsError('No resume or overwrite')
    out.mkdir(parents=True,exist_ok=True)
    # Frozen former worlds use merged-install URIs. Resolve through the actual
    # package index, with byte equality, rather than guessing the new layout.
    from ament_index_python.packages import get_package_share_directory
    share=Path(get_package_share_directory('mooncar_gazebo'))
    source_world=Path(f'/oldclosure/prepared-d0/{a.scene}-0.001/world.sdf')
    world_folder=out/(a.scene+'-0.001');world_folder.mkdir(exist_ok=True)
    world_tree=ET.parse(source_world);mapped=[]
    prefix='/work/install/share/mooncar_gazebo/'
    for uri in world_tree.findall('.//uri'):
        if (uri.text or '').startswith(prefix):
            relative=uri.text[len(prefix):];target=share/relative
            original_asset=Path('/source/ros2_ws/src/mooncar_gazebo')/relative
            if not target.is_file() or not original_asset.is_file():raise RuntimeError('Installed resource missing: '+str(target))
            digest=hashlib.sha256(target.read_bytes()).hexdigest()
            if digest!=hashlib.sha256(original_asset.read_bytes()).hexdigest():raise RuntimeError('Installed resource differs from source')
            mapped.append(dict(original=uri.text,resolved=str(target),sha256=digest));uri.text=str(target)
    world=world_folder/'world.sdf';world_tree.write(world)
    (out/(a.name+'-resource-map.json')).write_text(json.dumps(mapped,indent=2)+'\n')
    # Preserve the proven D=0 physical model and only redirect fixture paths.
    fixture=out/(a.name+'-fixture.py')
    text=Path('/oldclosure/prepared-d0/round_fixture.py').read_text()
    text=text.replace('/closure/prepared-d0/formal.sdf','/oldclosure/prepared-d0/formal.sdf')
    fixture.write_text(text)
    original=subprocess.Popen;children=[]
    import r5_step_schedule
    original_health=r5_step_schedule.PausedSteps.healthy
    def healthy(schedule):
        if any(child.poll() is not None for child,_ in children):
            raise RuntimeError('Independent feedback observer exited')
        return original_health(schedule)
    r5_step_schedule.PausedSteps.healthy=healthy
    def start(argv,*pos,**kw):
        is_bridge=any('config_file:=' in str(v) for v in argv) and ('bridge_node' in argv or any('image_dds_guard.py' in str(v) for v in argv))
        if is_bridge:
            config=next(str(v).split('config_file:=',1)[1] for v in argv if 'config_file:=' in str(v))
            entries=yaml.safe_load(Path(config).read_text())
            entries.append(dict(ros_topic_name='/mooncar/turn_allocation',gz_topic_name='/mooncar/turn_allocation',
                ros_type_name='std_msgs/msg/String',gz_type_name='gz.msgs.StringMsg',direction='ROS_TO_GZ'))
            Path(config).write_text(yaml.safe_dump(entries))
        process=original(argv,*pos,**kw)
        if is_bridge and not children:
            log=(folder/'adaptive-controller.log').open('x')
            child=original([sys.executable,'/source/scripts/r5_adaptive_turn_node.py','--output',str(folder)],
                stdout=log,stderr=subprocess.STDOUT,env=kw.get('env',os.environ),start_new_session=True)
            children.append((child,log))
        return process
    # Bind the wrapper module before the nested fixture installs its own Popen hooks.
    spec=importlib.util.spec_from_file_location('adaptive_shadow_fixture','/source/scripts/check_r5_coplanar_round.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    old_argv=sys.argv;subprocess.Popen=start;error=None;code=1
    try:
        sys.argv=['shadow','--output',str(out),'--name',a.name,'--world',str(world),
            '--ode-profile','candidate','--candidate-runtime','/opt/mooncar/turn/r5-v2',
            '--manifest','/opt/mooncar/turn/r5-v2/manifest.json','--round-fixture',str(fixture),'--angular',str(a.angular)]
        code=module.main()
    except Exception as exc:error=repr(exc)
    finally:
        sys.argv=old_argv;subprocess.Popen=original
        r5_step_schedule.PausedSteps.healthy=original_health
        for child,log in children:
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=7)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
            log.close()
    rows=[json.loads(line) for line in (folder/'adaptive-feedback.jsonl').open()] if (folder/'adaptive-feedback.jsonl').exists() else []
    decisions=[r for r in rows if r['kind']=='adaptive']
    motion=[r for r in decisions if abs(r['request'][1])>0 and abs(r['request'][0])<1e-9]
    valid=[r for r in motion if r['decision']['valid']]
    status=json.loads((folder/'adaptive-node-status.json').read_text()) if (folder/'adaptive-node-status.json').exists() else {}
    diagnostic=json.loads((folder/'diagnostic-result.json').read_text()) if (folder/'diagnostic-result.json').exists() else {}
    required=('data_complete','ground_contact_gate','stop_zero','observer_flush_confirmed',
        'receiver_verified','raw_fixture_passed','capture_drained','image_content',
        'origin_qualification','gate','allocation','contact_source','no_critical_errors','finite_geometry')
    qualified=all(diagnostic.get('checks',{}).get(k) is True for k in required)
    reasons={}
    for r in motion:
        if not r['decision']['valid']:reasons[r['decision']['reason']]=reasons.get(r['decision']['reason'],0)+1
    try:accuracy=feedback_accuracy(folder,decisions)
    except Exception as exc:accuracy=dict(passed=False,reason=repr(exc))
    result=dict(kind='sensor_shadow',fresh_run=True,actuation_changed=False,frames=len(decisions),
        motion_frames=len(motion),valid_motion_frames=len(valid),invalid_reasons=reasons,
        original_fixture_exit=code,error=error,status=status,
        input_and_capture_qualified=qualified,
        feedback_accuracy=accuracy,
        controller_exits=[child.returncode for child,_ in children],
        feedback_ready=bool(qualified and accuracy['passed'] and len(motion)>=180 and len(valid)==len(motion) and
            status.get('drained',{}).get('drained') and not status.get('errors') and
            children and all(child.returncode==0 for child,_ in children)))
    # The old steering screen may fail, but the original failure remains. A
    # shadow does NOT relabel that geometry failure as a candidate pass.
    (out/(a.name+'-feedback-summary.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    return 0 if result['feedback_ready'] and not error else 1


if __name__=='__main__':raise SystemExit(main())
