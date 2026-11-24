#!/usr/bin/env python3
"""Fresh-world R5 static, gimbal, drive and timestep acceptance with real ROS/Gazebo.

Navigation readiness is outside the gimbal-only fixture: its gate uses actual
sensor readiness and wheel odometry; the production feedback state machine and
real actuator/bridge/joint feedback remain in use. Truth only reaches witnesses.
"""
import argparse
from argparse import Namespace
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
from copy import deepcopy

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from check_r5_physics import prepare, command, sha, stability
from r5_runtime_witness import Witness
from experiment_motion import ExperimentMotion
from r5_capture_io import capture_hash,image_options
from r5_image_delivery import delivery_arguments,analyze_delivery
from r5_dds_shm import verify_runtime,process_env,analyze_dds
from r5_robot_description import description_path,description_parameters,validate_description
from r5_trial_processes import finish_transport_stream
from r5_ground_reference import prepare_reference


def run(args):
    wall_started=time.monotonic()
    delivery=bool(getattr(args,'cpp_image_observer',None))
    if getattr(args,'image_bridge','stock')=='traced' and (not delivery or not getattr(args,'traced_bridge',None)):
        raise RuntimeError('Traced bridge requires C++ observer and explicitly built bridge')
    verify_runtime(args)
    if getattr(args,'image_delivery_profile','legacy') not in ('legacy','shm_tx8m'):
        raise RuntimeError('Delivery repair requires evidence-qualified implementation; no silent fallback')
    if getattr(args,'image_trace','off')=='headers':
        if not getattr(args,'formal_d0',False) or not getattr(args,'image_observer',None) or not Path(args.image_observer).is_file():
            raise RuntimeError('Header tracing requires formal observation and an existing native image observer')
    folder = Path(args.output)/args.name; folder.mkdir(parents=True)
    dds_processes={'main':os.getpid()}
    (folder/'dds-processes.json').write_text(json.dumps(dds_processes)+'\n')
    os.environ['ROS_DOMAIN_ID']='95'
    os.environ['GZ_PARTITION']='mooncar_r5_runtime_'+args.name+'_'+uuid.uuid4().hex[:12]
    package=Path(args.package)
    env=dict(os.environ,GZ_SIM_RESOURCE_PATH=str(package.parent)+':'+os.environ.get('GZ_SIM_RESOURCE_PATH',''))
    source_world=package/f'worlds/mooncar_{args.scene}_v2.sdf' if args.scene!='flat' else None
    world=prepare(Namespace(world=source_world, package=package, model=args.model, plugin=args.plugin,
                            backend='dart',step=args.step,disable_gimbal=False,disable_drive=False),folder)
    tree=ET.parse(world); w=tree.find('world')
    formal=bool(getattr(args,'formal_d0',False))
    if formal and args.scene=='flat':
        plate=w.find("model[@name='plate']/link")
        collision=plate.find('collision')
        visual=ET.SubElement(plate,'visual',name='plate_visual')
        for key in ('pose','geometry'):visual.append(deepcopy(collision.find(key)))
        material=ET.SubElement(visual,'material')
        ET.SubElement(material,'ambient').text='.35 .33 .30 1'
        ET.SubElement(material,'diffuse').text='.55 .52 .47 1'
    observer_path=None
    if getattr(args,'observer_plugin',None):
        observer_path=Path('/work')/(args.name+'-physics-steps.csv')
        p=ET.SubElement(w.find("model[@name='mooncar']"),'plugin',filename=str(Path(args.observer_plugin).resolve()),name='mooncar::StepObserver')
        ET.SubElement(p,'output').text=str(observer_path)
    if w.find("plugin[@name='gz::sim::systems::Sensors']") is None:
        p=ET.SubElement(w,'plugin',filename='gz-sim-sensors-system',name='gz::sim::systems::Sensors')
        ET.SubElement(p,'render_engine').text='ogre2'
        ET.SubElement(w,'plugin',filename='gz-sim-imu-system',name='gz::sim::systems::Imu')
        light=ET.SubElement(w,'light',type='directional',name='sun')
        for k,v in {'pose':'0 0 10 0 0 0','diffuse':'.95 .92 .86 1','direction':'-.45 -.3 -.84','cast_shadows':'true'}.items():
            ET.SubElement(light,k).text=v
    tree.write(world)
    urdf_path=description_path(args)
    description_proof=validate_description(world,urdf_path)
    robot_parameters=description_parameters(args)
    description_proof['publish_frequency_hz']=robot_parameters.get('publish_frequency')
    qualified_ground,ground_proof=prepare_reference(args,world)
    bridge_config=folder/'bridge.yaml'
    import yaml
    entries=yaml.safe_load((package/'config/ros_gz_bridge_r5.yaml').read_text())
    # Publish the same raw clocks/joints as simulation.launch, with its throttles.
    bridge_config.write_text(yaml.safe_dump(entries))
    config=dict(vars(args),ground_reference_qualification=ground_proof,robot_description_qualification=description_proof,world_sha256=sha(world),model_sha256=sha(args.model),plugin_sha256=sha(args.plugin),
                observer_sha256=sha(args.observer_plugin) if getattr(args,'observer_plugin',None) else None,
                source_sha256={p.name:sha(p) for p in [Path(__file__),Path(__file__).with_name('r5_runtime_witness.py'),
                                                       Path(__file__).with_name('experiment_motion.py'),
                                                       Path(__file__).with_name('r5_control_analysis.py'),
                                                       Path(__file__).with_name('r5_d0_analysis.py'),
                                                       Path(__file__).with_name('r5_step_schedule.py'),
                                                       Path(__file__).with_name('r5_capture_io.py'),
                                                       Path(__file__).with_name('r5_capture_probe.py'),
                                                       Path(__file__).with_name('r5_capture_analysis.py'),
                                                       Path(__file__).with_name('r5_image_trace.py')]},
                capture_config_sha256=capture_hash(getattr(args,'capture_profile','legacy'),**image_options(args)),
                fixture='No Nav2/planner. Production relay, sensors and feedback state machine; truth is observer-only.')
    (folder/'config.json').write_text(json.dumps(config,indent=2))
    processes=[]
    def start(argv,label):
        log=(folder/(label+'.log')).open('w')
        proc=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,env=process_env(args,env,folder,label),start_new_session=True)
        if label in ('bridge','capture-probe','cpp-images'):
            dds_processes[label]=proc.pid
            (folder/'dds-processes.json').write_text(json.dumps(dds_processes)+'\n')
        processes.append((proc,log));return proc
    server=start(['gz','sim','-s','-v','4','--headless-rendering','--render-engine','ogre2',str(world)],'simulation')
    if formal:start([os.sys.executable,str(Path(__file__).with_name('r5_capture_probe.py')),'--output',str(folder),
        '--image-trace',getattr(args,'image_trace','off')],'capture-probe')
    if getattr(args,'image_trace','off')=='headers':
        if not getattr(args,'image_observer',None):raise RuntimeError('Header tracing requires native image observer')
        start([args.image_observer,str(folder)],'native-images')
    if delivery:
        start([args.cpp_image_observer,str(folder)],'cpp-images')
        start([os.sys.executable,str(Path(__file__).with_name('r5_delivery_transport.py')),'--output',str(folder)],'delivery-transport')
    if getattr(args,'image_bridge','stock')=='traced':
        env['MOONCAR_IMAGE_BRIDGE_TRACE']=str(folder/'bridge-images.jsonl')
        previous_library_path=env.get('LD_LIBRARY_PATH','')
        env['LD_LIBRARY_PATH']=str(Path(args.traced_bridge).parents[1])+':'+previous_library_path
        start([args.traced_bridge,'--ros-args','-p','config_file:='+str(bridge_config)],'bridge')
        del env['MOONCAR_IMAGE_BRIDGE_TRACE']
        env['LD_LIBRARY_PATH']=previous_library_path
    else:start(['ros2','run','ros_gz_bridge','bridge_node','--ros-args','-p','config_file:='+str(bridge_config)],'bridge')
    start(['ros2','run','topic_tools','throttle','messages','/mooncar/clock_raw','100.0','/clock_out',
           '--ros-args','-p','use_sim_time:=true','-p','use_wall_clock:=false',
           '-r','/clock:=/mooncar/clock_raw','-r','/clock_out:=/clock'],'clock-throttle')
    start(['ros2','run','topic_tools','throttle','messages','/mooncar/joint_states_raw','30.0','/joint_states',
           '--ros-args','-p','use_sim_time:=true','-p','use_wall_clock:=false'],'joint-throttle')
    # Laser filter requires static robot frames; use the exact expanded URDF.
    description=folder/'robot.yaml'
    description.write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':robot_parameters}}))
    start(['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(description)],'robot-state')
    start(['ros2','run','laser_filters','scan_to_scan_filter_chain','--ros-args','--params-file',
           str(package/'config/laser_filter_r5.yaml'),'-p','use_sim_time:=true',
           '-r','scan:=/mooncar/scan_raw','-r','scan_filtered:=/mooncar/scan'],'laser-filter')
    start(['ros2','run','mooncar_control','cmd_vel_safety_relay.py','--ros-args','-p','use_sim_time:=true'],'safety-relay')
    start(['ros2','run','mooncar_control','wheel_odometry.py','--ros-args','-p','use_sim_time:=true',
           '-p','track_width:=0.3311','-p','wheel_radius:=0.076'],'wheel-odometry')
    raw_streams=[]
    for label,topic in [('gz-input','/cmd_vel')]+[(n+'-contact',f'/world/r5_check/model/mooncar/link/{n}_wheel_link/sensor/{n}_wheel_link_contact/contact')
                                                for n in ('front_left','front_right','rear_left','rear_right')]:
        raw=(folder/(label+'.jsonl')).open('w');err=(folder/(label+'.stderr')).open('w')
        p=subprocess.Popen(['gz','topic','-e','--json-output','-t',topic],stdout=raw,stderr=err,env=env,start_new_session=True)
        raw_streams.append((p,raw,err))
    deadline=wall_started+600
    rclpy.init();node=Witness(args.scene,folder,env,sample_resources=formal,
        capture_profile=getattr(args,'capture_profile','legacy'),deadline=deadline,image_trace=getattr(args,'image_trace','off'),
        image_sampling=getattr(args,'image_sampling','legacy'),image_observer_sha256=sha(args.image_observer) if getattr(args,'image_observer',None) else None)
    node.create_subscription(JointState,'/joint_states',lambda m:node.record(
        'joints',stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
        stamp_ns=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec,
        names=list(m.name),positions=list(m.position),velocities=list(m.velocity)),qos_profile_sensor_data)
    if getattr(args,'robot_description_urdf',None):
        from tf2_msgs.msg import TFMessage
        node.create_subscription(TFMessage,'/tf',lambda m:node.record('robot_tf',transforms=[
            dict(stamp_ns=t.header.stamp.sec*1_000_000_000+t.header.stamp.nanosec,
                parent=t.header.frame_id,child=t.child_frame_id,
                translation=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],
                rotation_xyzw=[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w])
            for t in m.transforms]),100)
    odom=[None]
    node.create_subscription(Odometry,'/odom',lambda m:odom.__setitem__(0,m),10)
    executor=SingleThreadedExecutor();executor.add_node(node)
    motion=None
    if args.kind in ('gimbal','gimbal-sweep'):
        motion=ExperimentMotion();motion.set_parameters([Parameter('use_sim_time',value=True)])
        if args.kind=='gimbal-sweep':
            motion.set_parameters([Parameter('gimbal_position_tolerance_rad',value=math.radians(1)),
                Parameter('gimbal_velocity_tolerance_rad_s',value=.05),
                Parameter('gimbal_settle_duration_s',value=.3),Parameter('gimbal_action_timeout_s',value=6.)])
        # Only substitute the unrelated Nav2/map gate. No Gazebo truth input.
        motion.state=lambda:dict(ready=all(node.counts.get(k,0)>1 for k in ('rgb','depth','scan','imu')) and odom[0] is not None,diagnostics={})
        def pose():
            if odom[0] is None:return None
            p=odom[0].pose.pose.position
            return dict(x=p.x,y=p.y,z=p.z,yaw=0.)
        motion.pose=pose;executor.add_node(motion)
    result=dict(passed=False)
    started=None; scan_started=None; timeout_started=None; scan_state=None
    error=None
    scheduled=None
    sweep=[];sweep_index=0;sweep_command=None;sweep_result=None
    try:
        service='/world/r5_check/control'
        while service not in command('gz','service','-l'):
            if server.poll() is not None or time.monotonic()>deadline:raise RuntimeError('World unavailable')
            executor.spin_once(timeout_sec=.02)
        # Let ROS endpoints discover each other while physics is paused.
        end=time.monotonic()+3
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.02);node.publisher.publish(Twist())
        if getattr(args,'motion_schedule','legacy')=='paused_steps' and args.kind=='drive':
            if observer_path is None:raise RuntimeError('Paused steps require the read-only observer')
            from r5_step_schedule import PausedSteps
            controller=PausedSteps(node,executor,processes,env,observer_path,args.step,deadline,command,
                lambda:bool(node.truth and odom[0] is not None and all(node.counts.get(k,0)>1 for k in ('rgb','depth','scan','imu'))))
            try:
                scheduled=controller.run(args.linear,args.angular);result['schedule']=scheduled
                started=scheduled['sim_started']
            finally:
                result['schedule_phases']=controller.phases;controller.close()
        else:
            answer=command('gz','service','-s',service,'--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean',
                           '--timeout','5000','--req','pause: false')
            assert 'data: true' in answer,answer
        while scheduled is None and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.01)
            if any(p.poll() is not None for p,_ in processes):raise RuntimeError('Runtime process exited')
            now=node.now()
            if started is None and node.truth and odom[0] is not None and all(node.counts.get(k,0)>1 for k in ('rgb','depth','scan','imu')):
                started=now;node.record('phase',phase='parked',started=now)
            elapsed=now-started if started is not None else 0
            cmd=Twist()
            if args.kind=='drive' and 5<=elapsed<25:
                cmd.linear.x=args.linear;cmd.angular.z=args.angular
            node.publisher.publish(cmd)
            if formal:
                node.record('source_command',values=[cmd.linear.x,0.,0.,0.,0.,cmd.angular.z])
                if node.truth and max(abs(node.truth[-1]['roll']),abs(node.truth[-1]['pitch']))>.8:
                    raise RuntimeError('Physical instability: roll/pitch exceeded 0.8 rad')
            if motion:
                motion.last_heartbeat=time.monotonic()
                if args.kind=='gimbal-sweep':
                    targets=[(math.pi/2,-math.pi/12),(-math.pi/2,-math.pi/12),(0.,0.)]
                    if elapsed>=10 and sweep_index<len(targets) and sweep_command is None:
                        motion.pan_tilt(*targets[sweep_index]);sweep_command=now
                        node.record('sweep_command',target=list(targets[sweep_index]),started=now)
                    if sweep_command is not None and motion.active and now-sweep_command>6.2:
                        motion.cancel('Acceptance target deadline exceeded')
                    if sweep_command is not None and not motion.active and sweep_result is None:
                        sweep_result=dict(target=list(targets[sweep_index]),started=sweep_command,finished=now,
                                          phase=motion.phase,error=motion.error,position=motion.gimbal_position)
                        node.record('sweep_result',**sweep_result)
                    if sweep_result is not None and now-sweep_result['finished']>=3:
                        sweep.append(sweep_result);sweep_index+=1;sweep_command=None;sweep_result=None
                    if sweep_index==len(targets) and timeout_started is None:
                        for sub in list(motion.subscriptions):
                            if sub.topic_name=='/joint_states':motion.destroy_subscription(sub)
                        motion.pan_tilt(.1,0.);timeout_started=now
                        node.record('phase',phase='feedback_loss_injection',started=now)
                    if timeout_started is not None and not motion.active:
                        result['sweep']=sweep
                        result['timeout']=dict(phase=motion.phase,error=motion.error,injected_feedback_loss=True)
                        break
                    continue
                if elapsed>=5 and scan_started is None:
                    motion.pan_tilt(math.pi/2,-math.pi/12)
                    scan_started=now;node.record('phase',phase='gimbal',started=now)
                if scan_started is not None and not motion.active and scan_state is None:
                    scan_state=dict(phase=motion.phase,error=motion.error,position=motion.gimbal_position,
                                    settled=motion.gimbal_settled_samples,finished=now)
                    node.record('gimbal_result',**scan_state)
                if scan_state and scan_state['phase']=='succeeded' and timeout_started is None and now-scan_state['finished']>=2:
                    # Controlled feedback-loss injection only for timeout handling;
                    # physical scan success above used unmodified actual feedback.
                    for sub in list(motion.subscriptions):
                        if sub.topic_name=='/joint_states':motion.destroy_subscription(sub)
                    motion.pan_tilt(0.,-math.pi/12)
                    timeout_started=now;node.record('phase',phase='feedback_loss_injection',started=now)
                if timeout_started is not None and not motion.active and now-timeout_started>=motion.sim_budget+.1:
                    result['gimbal']=scan_state
                    result['timeout']=dict(phase=motion.phase,error=motion.error,injected_feedback_loss=True)
                    break
            elif started is not None and elapsed>=(10 if args.kind=='static' else 28):break
        else:
            if scheduled is None:raise TimeoutError('600 second case deadline')
        if started is None:raise RuntimeError('Missing sensor readiness')
        samples=[dict(t=r['stamp'],xyz=[r['x'],r['y'],r['z']],rpy=[r['roll'],r['pitch'],
                 math.atan2(2*(r['quaternion'][3]*r['quaternion'][2]+r['quaternion'][0]*r['quaternion'][1]),
                            1-2*(r['quaternion'][1]**2+r['quaternion'][2]**2))]) for r in node.truth]
        active=[s for s in samples if s['t']>=started]
        def nearest(t):return min(samples,key=lambda s:abs(s['t']-t))
        safe=[r for r in node.rows if r['kind']=='safe_command']
        native_commands=[r for r in node.rows if r['kind']=='gz_command']
        joints=[r for r in node.rows if r['kind']=='joints']
        pose_gap=max(b['t']-a['t'] for a,b in zip(samples,samples[1:]))
        joint_gap=max(b['stamp']-a['stamp'] for a,b in zip(joints,joints[1:])) if len(joints)>1 else math.inf
        complete=all(node.counts.get(k,0)>1 for k in ('rgb','depth','scan','imu')) and pose_gap<=.100001 and joint_gap<=.100001 and bool(safe) and bool(native_commands)
        result.update(sensor_counts=node.counts,pose_gap_s=pose_gap,joint_gap_s=joint_gap,data_complete=complete,
                      errors=node.errors,sim_started=started,sim_ended=scheduled['sim_ended'] if scheduled else node.now())
        if qualified_ground is not None:
            ground=qualified_ground
            result['ground_reference_qualification']=ground_proof
        elif args.scene=='flat':
            ground=lambda x,y:0.
        else:
            spec=importlib.util.spec_from_file_location('r5_ground_detail',package/'scripts/terrain_scene_detail.py')
            detail=importlib.util.module_from_spec(spec);spec.loader.exec_module(detail)
            grid=np.load(package/f'models/terrain_v2/{args.scene}/height_grid.npz')
            ground=lambda x,y:detail.sample_ground(grid['x'],grid['y'],grid['z'],x,y)
        clearance=[s['xyz'][2]-ground(*s['xyz'][:2]) for s in active if s['t']>=started+1]
        result['footprint_ground_clearance_range_m']=[min(clearance),max(clearance)]
        result['ground_contact_gate']=min(clearance)>=-.01 and max(clearance)<=.03
        complete=complete and result['ground_contact_gate']
        if args.kind=='static':
            result['static']=stability(active)
            result['passed']=complete and result['static']['passed'] and all(max(map(abs,r['values']))==0 for r in safe+native_commands)
        elif args.kind=='drive':
            a,b=nearest(started+5),nearest(started+25)
            if scheduled:
                from r5_d0_analysis import pose
                a=pose(scheduled['parked']['last_step']);b=pose(scheduled['motion']['last_step'])
            last=active[-1]; stop=nearest(last['t']-1)
            dx=b['xyz'][0]-a['xyz'][0]; displacement=math.dist(b['xyz'][:2],a['xyz'][:2])
            stop_distance=math.dist(last['xyz'][:2],stop['xyz'][:2])
            zeros=[r for r in safe if r['sim']>=started+25.2]
            native_zeros=[r for r in native_commands if r['sim']>=started+25.2]
            tilt=max(max(abs(s['rpy'][0]),abs(s['rpy'][1])) for s in active)
            result.update(dx_m=dx,displacement_m=displacement,endpoint=b,stop_drift_last_second_m=stop_distance,
                          stop_zero=bool(zeros) and bool(native_zeros) and all(max(map(abs,r['values']))==0 for r in zeros+native_zeros),max_tilt_rad=tilt,
                          parked=stability([s for s in active if s['t']<started+5]))
            # The commanded turn convergence cases have their own endpoint gates.
            straight=(1<=dx<=2.5) if args.angular==0 else True
            result['passed']=complete and straight and tilt<.35 and result['stop_zero'] and stop_distance<=.01 and result['parked']['passed']
        elif args.kind=='gimbal':
            scan=[s for s in samples if scan_started<=s['t']<=scan_state['finished']+2]
            result['body_during_scan']=stability(scan)
            result['passed']=complete and scan_state['phase']=='succeeded' and result['body_during_scan']['passed'] and result['timeout']['phase']=='failed' and all(max(map(abs,r['values']))==0 for r in safe+native_commands)
        else:
            result['body_during_scan']=stability(active)
            result['passed']=complete and result['body_during_scan']['passed'] and result['timeout']['phase']=='failed' and all(
                r['phase']=='succeeded' and r['finished']-r['started']<=6 for r in sweep) and len(sweep)==3 and all(
                max(map(abs,r['values']))==0 for r in safe+native_commands)
    except Exception as failure:
        error=repr(failure);result.update(passed=False,error=error,sim_ended=node.now(),sensor_counts=node.counts)
    finally:
        node.publisher.publish(Twist())
        try:
            command('gz','service','-s','/world/r5_check/control','--reqtype','gz.msgs.WorldControl',
                    '--reptype','gz.msgs.Boolean','--timeout','3000','--req','pause: true',timeout=5)
        except Exception:
            pass  # Preserve original failure while always shutting down processes.
        if observer_path is not None:
            # WorldControl acknowledges enqueueing; wait for the actual paused
            # step and buffered observer flush before terminating Gazebo.
            marker=Path(str(observer_path)+'.paused')
            until=time.monotonic()+5
            while not marker.is_file() and time.monotonic()<until:time.sleep(.02)
            if scheduled:
                from r5_step_schedule import pause_marker
                boundary=scheduled['stop']['after']
                result['observer_flush_confirmed']=pause_marker(marker,boundary['iteration'],boundary['sim'])
            else:result['observer_flush_confirmed']=marker.is_file()
            if result['observer_flush_confirmed']:shutil.copy2(marker,folder/'physics-pause-confirmation.txt')
            else:result['passed']=False
        for p,_ in processes:
            try:os.killpg(p.pid,signal.SIGINT)
            except ProcessLookupError:pass
        for p,log in processes:
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGKILL);p.wait()
            log.close()
        for p,raw,err in raw_streams:
            finish_transport_stream(p,raw,err)
        result['capture']=node.close();result['errors']=list(node.errors)
        if not result['capture']['drained']:result['passed']=False
        if motion:motion.destroy_node()
        executor.shutdown();rclpy.shutdown()
        if getattr(args,'dds_runtime','stock')=='observed':
            import ctypes
            ctypes.CDLL(str(Path(args.dds_prefix)/'lib/libmooncar-dds-trace.so')).mooncar_dds_flush()
        result['wall_seconds']=time.monotonic()-wall_started
        if observer_path is not None and observer_path.is_file():
            shutil.copy2(observer_path,folder/'physics-steps.csv')
            from r5_control_analysis import analyze_steps
            try:
                result['step_observation']=analyze_steps(folder/'physics-steps.csv',result)
            except Exception as failure:
                # Missing or malformed critical observations fail the round;
                # keep a reviewable verdict as well as the untouched raw CSV.
                result['step_observation']=dict(passed=False,complete=False,reason=repr(failure))
            result['passed']=result['passed'] and result['step_observation']['passed']
        elif observer_path is not None:
            result.update(passed=False,observer_error='Missing per-step CSV')
        result['contact_samples']={p.stem:sum(1 for _ in p.open()) for p in folder.glob('*-contact.jsonl')}
        if formal:
            try:
                from r5_capture_analysis import layered_capture
                result['capture_analysis']=layered_capture(folder,result)
                result['passed']=result['passed'] and result['capture_analysis']['passed']
                if getattr(args,'image_trace','off')=='headers':
                    from r5_image_trace import analyze_images
                    result['image_trace_analysis']=analyze_images(folder,result)
                    result['passed']=result['passed'] and result['image_trace_analysis']['passed']
                if delivery:
                    result['delivery_analysis']=analyze_delivery(folder,result,getattr(args,'image_bridge','stock')=='traced')
                    result['passed']=result['passed'] and result['delivery_analysis']['passed']
                result['dds']=analyze_dds(folder,result,args)
                result['passed']=result['passed'] and result['dds']['passed']
                from r5_d0_analysis import formal_observations
                result['formal']=formal_observations(folder,result,args.kind,args.linear,args.angular)
                result['formal']['contacts_complete']=len(result['contact_samples'])==4 and all(result['contact_samples'].values())
                result['formal']['passed']=result['formal']['passed'] and result['formal']['contacts_complete']
                result['passed']=result['passed'] and result['formal']['passed']
                result['evidence_bytes']=sum(p.stat().st_size for p in folder.rglob('*') if p.is_file())
            except Exception as failure:
                result.update(passed=False,formal=dict(passed=False,data_complete=False,reason=repr(failure)))
        (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(dict(name=args.name,**result)),flush=True)
    if error:raise RuntimeError(error)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    delivery_arguments(parser)
    parser.add_argument('--output',required=True);parser.add_argument('--name',required=True)
    parser.add_argument('--model',required=True);parser.add_argument('--plugin',required=True)
    parser.add_argument('--package',default='/source/ros2_ws/src/mooncar_gazebo')
    parser.add_argument('--scene',choices=('flat','ramp','curve','survey'),default='flat')
    parser.add_argument('--kind',choices=('static','gimbal','gimbal-sweep','drive'),default='drive')
    parser.add_argument('--step',type=float,choices=(.002,.001,.0005,.00025),default=.001)
    parser.add_argument('--observer-plugin')
    parser.add_argument('--formal-d0',action='store_true',help='Test-only strict completeness, native command and resource observation')
    parser.add_argument('--motion-schedule',choices=('legacy','paused_steps'),default='legacy')
    parser.add_argument('--capture-profile',choices=('legacy','async'),default='legacy')
    parser.add_argument('--image-trace',choices=('off','headers'),default='off')
    parser.add_argument('--image-sampling',choices=('legacy','nanoseconds'),default='legacy',help='Test-only comparison; relative one-second sampling unchanged')
    parser.add_argument('--image-observer')
    parser.add_argument('--linear',type=float,default=.1);parser.add_argument('--angular',type=float,default=0.)
    run(parser.parse_args())
