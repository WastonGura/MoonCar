#!/usr/bin/env python3
"""Actual installed simulation.launch, unchanged controls, zero-command imagery validation."""
import argparse, hashlib, importlib.util, json, math, os, shutil, signal, subprocess, sys, time, uuid
from pathlib import Path
import xml.etree.ElementTree as ET

# xacro matches the literal namespace prefix for substitution arguments.
# ElementTree's default ns0 prefix changes executable model semantics.
ET.register_namespace('xacro','http://www.ros.org/wiki/xacro')

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p):return [json.loads(s) for s in Path(p).read_text().splitlines() if s]
def run(a):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Twist
    from r5_runtime_witness import Witness
    from r5_capture_analysis import coverage
    from check_r5_physics import command,last_message,timestamp
    from r5_step_schedule import pause_marker
    folder=Path(a.output)/a.name;folder.mkdir();started_wall=time.monotonic();deadline=started_wall+580
    env=dict(os.environ,GZ_PARTITION='mooncar_dds_launch_'+uuid.uuid4().hex,MOONCAR_IMAGE_DDS_PROFILE='auto',
        MOONCAR_IMAGE_DDS_RECEIPT_ROOT=str(folder/'bridge-receipts'))
    os.environ['GZ_PARTITION']=env['GZ_PARTITION']
    package=Path('/work/install/share/mooncar_gazebo');description=Path('/work/install/share/mooncar_description')
    model_name='mooncar.urdf.xacro' if a.legacy else 'mooncar_r5.urdf.xacro'
    installed=description/'urdf'/model_name;source=Path('/source/ros2_ws/src/mooncar_description/urdf')/model_name
    # These two plugins observe the spawned model; control/physical XML is unchanged.
    tree=ET.parse(source);gz=ET.SubElement(tree.getroot(),'gazebo')
    pose=ET.SubElement(gz,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher')
    for k,v in dict(publish_link_pose='true',publish_model_pose='true',publish_sensor_pose='true',use_pose_vector_msg='true',update_frequency='100',static_publisher='false').items():ET.SubElement(pose,k).text=v
    observer=folder/'physics-steps.csv'
    if not a.legacy:
        step=ET.SubElement(gz,'plugin',filename='/evidence/observer/libmooncar-step-observer.so',name='mooncar::StepObserver')
        ET.SubElement(step,'output').text=str(observer)
    tree.write(installed)
    world=package/'worlds'/('lunar_yard.sdf' if a.legacy else 'mooncar_'+a.scene+'_v2.sdf')
    name=ET.parse(world).find('world').get('name')
    (folder/'config.json').write_text(json.dumps(dict(scene=a.scene,legacy=a.legacy,world_sha256=sha(world),model_source_sha256=sha(source),
        model_instrumented_sha256=sha(installed),gimbal_default_changed=False,environment=env,observer_only=True),indent=2))
    log=(folder/'launch.log').open('x');process=subprocess.Popen(['ros2','launch','mooncar_gazebo','simulation.launch.py',
        'model_file:='+model_name,'world:='+str(world),'world_name:='+name,'spawn_z:=0.03',
        'gz_args:=-r -s -v 3 --headless-rendering --render-engine ogre2','software_rendering:=true'],env=env,
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Witness(a.scene,folder,env,sample_resources=True,capture_profile='async',deadline=deadline,image_trace='headers')
    node.create_subscription(JointState,'/joint_states',lambda m:node.record('joints',stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
        names=list(m.name),positions=list(m.position),velocities=list(m.velocity)),qos_profile_sensor_data)
    executor=SingleThreadedExecutor();executor.add_node(node);begin=None;result={'passed':False};live=None
    try:
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.01);node.publisher.publish(Twist())
            if process.poll() is not None:raise RuntimeError('Formal launch exited')
            if node.errors:raise RuntimeError('Archive worker failed: '+repr(node.errors))
            if node.truth and max(abs(node.truth[-1]['roll']),abs(node.truth[-1]['pitch']))>.8:
                raise RuntimeError('Unexpected physical instability during acquisition check')
            if begin is None and node.truth and all(node.counts.get(k,0)>2 for k in ('rgb','depth','scan','imu')):
                begin=node.now();node.record('phase',phase='observing_zero',started=begin)
                # Snapshot live participants rather than inferring segment sizes after shutdown.
                maps=Path('/proc/self/maps').read_text();files={line.split()[-1] for line in maps.splitlines() if '/dev/shm/' in line}
                live=dict(pid=os.getpid(),libraries=sorted({line.split()[-1] for line in maps.splitlines() if any(s in line for s in ('librmw_fastrtps','libfastrtps'))}),
                    segments=[dict(path=p,bytes=Path(p).stat().st_size) for p in files if Path(p).is_file()])
                qos={}
                for topic in ('/mooncar/rgb/image','/mooncar/depth/image'):
                    qos[topic]=[dict(node=e.node_name,gid=list(e.endpoint_gid),reliability=str(e.qos_profile.reliability),durability=str(e.qos_profile.durability),
                        history=None if e.qos_profile.history.value==3 else str(e.qos_profile.history),depth=None if e.qos_profile.history.value==3 else e.qos_profile.depth)
                        for e in node.get_publishers_info_by_topic(topic)]
                live['qos']=qos;(folder/'live-receiver.json').write_text(json.dumps(live,indent=2)+'\n')
            if begin is not None and node.now()-begin>=28:break
        else:raise TimeoutError('Formal launch collection timeout')
        service='/world/'+name+'/control'
        answer=command('gz','service','-s',service,'--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req','pause: true')
        if 'data: true' not in answer:raise RuntimeError('Pause request rejected')
        stats=None;until=min(deadline,time.monotonic()+5)
        while time.monotonic()<until:
            executor.spin_once(timeout_sec=.02)
            stats=last_message(command('gz','topic','-n','1','-e','--json-output','-t','/world/'+name+'/stats',timeout=3))
            if stats.get('paused'):break
        else:raise RuntimeError('Actual pause not confirmed')
        stamp=stats.get('simTime',stats.get('sim_time'));end=stamp.get('sec',0)+stamp.get('nsec',0)/1e9
        result.update(sim_started=begin,sim_ended=end,actual_pause=stats)
        until=min(deadline,time.monotonic()+5)
        while time.monotonic()<until:
            executor.spin_once(timeout_sec=.02)
            if (a.legacy or pause_marker(str(observer)+'.paused',int(stats['iterations']),end)) and node.truth and node.truth[-1]['stamp']>=end-.1:break
        else:raise RuntimeError('Pause flush/tail confirmation missing')
        result['observer_flush_confirmed']=a.legacy or pause_marker(str(observer)+'.paused',int(stats['iterations']),end)
        result['passed']=True
    except Exception as e:result['error']=repr(e)
    finally:
        try:
            subprocess.run(['gz','service','-s','/world/'+name+'/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','1000','--req','pause: true'],env=env,capture_output=True,timeout=2)
        except subprocess.SubprocessError:pass
        # Physics has paused and the observer has flushed before draining archive workers.
        result['capture']=node.close();executor.shutdown();rclpy.shutdown()
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(8)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close();shutil.copy2(source,installed)
    try:
        observations=rows(folder/'observations.jsonl');begin=result['sim_started'];end=result['sim_ended']
        sensors={k:coverage([r.get('stamp',r['sim']) for r in observations if r['kind']==k],begin,end,bound)
            for k,bound in (('rgb',1.5),('depth',1.5),('scan',.75),('imu',.100001),('joints',.100001),('truth',.100001),('safe_command',.100001))}
        selected=[r for r in observations if r['kind'] in ('rgb','depth')]
        images=all((folder/r['file']).is_file() and (folder/r['file']).stat().st_size>0 for r in selected)
        ids={r['event_id'] for r in observations};receipts=rows(folder/'writer-receipts.jsonl')
        drained=bool(result['capture']['drained'] and ids=={r['event_id'] for r in receipts} and len(ids)==len(observations)==len(receipts))
        safe=[r for r in observations if r['kind'] in ('safe_command','gz_command') and begin<=r['sim']<=end]
        zero=bool(safe and all(max(map(abs,r['values']))==0 for r in safe))
        ready=list((folder/'bridge-receipts').glob('*/ready.json'))
        bridge=json.loads(ready[0].read_text()) if len(ready)==1 else None
        libraries=live['libraries'];stock_receivers=bool(any(s['bytes']==549408 for s in live['segments']) and not any('image-dds' in p for p in libraries))
        result.update(sensors=sensors,images_complete=images,event_receipts_complete=drained,commands_zero=zero,stock_receiver_verified=stock_receivers,
            production_bridge=bridge,legacy_no_custom_library=a.legacy and bridge is None,wall_seconds=time.monotonic()-started_wall)
        result['passed']=bool(result['passed'] and all(c['passed'] for c in sensors.values()) and images and drained and zero and stock_receivers and (bridge is not None if not a.legacy else bridge is None))
        if not a.legacy:
            from r5_control_analysis import analyze_steps
            step=analyze_steps(observer,result);result['steps']=step;result['passed']=result['passed'] and step['complete']
        result['performance_samples']=rows(folder/'resources.jsonl')
        result['evidence_bytes']=sum(p.stat().st_size for p in folder.rglob('*') if p.is_file())
    except Exception as e:result.update(passed=False,analysis_error=repr(e))
    (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(name=a.name,passed=result['passed'],error=result.get('error'),sensors=result.get('sensors'))),flush=True)
    return 0 if result['passed'] else 1

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--name',required=True)
    p.add_argument('--scene',choices=('ramp','curve','survey'),default='ramp');p.add_argument('--legacy',action='store_true')
    raise SystemExit(run(p.parse_args()))
