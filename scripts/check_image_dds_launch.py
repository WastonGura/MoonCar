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
    from check_r5_physics import last_message
    from image_dds_pause import QueryWorker,wait_query,actual_pause
    folder=Path(a.output)/a.name;folder.mkdir();started_wall=time.monotonic();deadline=started_wall+580
    from image_dds_validation import observed_model,receiver_snapshot,image_files,stop_process,participants,paused_state,reception_tail,pause_flush
    if 'MOONCAR_IMAGE_DDS_PROFILE' in os.environ:raise RuntimeError('Formal default test must not override profile')
    env=dict(os.environ,GZ_PARTITION='mooncar_dds_launch_'+uuid.uuid4().hex,
        MOONCAR_IMAGE_DDS_RECEIPT_ROOT=str(folder/'bridge-receipts'))
    os.environ['GZ_PARTITION']=env['GZ_PARTITION']
    package=Path('/work/install/share/mooncar_gazebo');description=Path('/work/install/share/mooncar_description')
    model_name='mooncar.urdf.xacro' if a.legacy else 'mooncar_r5.urdf.xacro'
    installed=description/'urdf'/model_name;source=Path('/source/ros2_ws/src/mooncar_description/urdf')/model_name
    # These two plugins observe the spawned model; control/physical XML is unchanged.
    observer=folder/'physics-steps.csv'
    observed_model(source,installed,None if a.legacy else observer)
    world=package/'worlds'/('lunar_yard.sdf' if a.legacy else 'mooncar_'+a.scene+'_v2.sdf')
    name=ET.parse(world).find('world').get('name')
    (folder/'config.json').write_text(json.dumps(dict(scene=a.scene,legacy=a.legacy,world_sha256=sha(world),model_source_sha256=sha(source),
        model_instrumented_sha256=sha(installed),gimbal_default_changed=False,environment=env,observer_only=True),indent=2))
    log=(folder/'launch.log').open('x');process=subprocess.Popen(['ros2','launch','mooncar_gazebo','simulation.launch.py',
        'model_file:='+model_name,'world:='+str(world),'world_name:='+name,'spawn_z:=0.03',
        'gz_args:=-r -s -v 3 --headless-rendering --render-engine ogre2','software_rendering:=true'],env=env,
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    node=None;executor=None;initialized=False;begin=None;result={'passed':False};live=None
    worker=QueryWorker();next_heartbeat=0.
    def pump():
        nonlocal next_heartbeat
        executor.spin_once(timeout_sec=.005)
        now=time.monotonic()
        if now>=next_heartbeat:
            node.publisher.publish(Twist());node.record('source_command',values=[0.]*6)
            next_heartbeat=now+.02
        if process.poll() is not None or node.errors:raise RuntimeError('Runtime failed during continued reception')
    try:
        rclpy.init();initialized=True
        node=Witness(a.scene,folder,env,sample_resources=True,capture_profile='async',deadline=deadline,image_trace='headers')
        node.create_subscription(JointState,'/joint_states',lambda m:node.record('joints',stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
            names=list(m.name),positions=list(m.position),velocities=list(m.velocity)),qos_profile_sensor_data)
        executor=SingleThreadedExecutor();executor.add_node(node)
        while time.monotonic()<deadline:
            pump()
            if process.poll() is not None:raise RuntimeError('Formal launch exited')
            if node.errors:raise RuntimeError('Archive worker failed: '+repr(node.errors))
            if node.truth and max(abs(node.truth[-1]['roll']),abs(node.truth[-1]['pitch']))>.8:
                raise RuntimeError('Unexpected physical instability during acquisition check')
            if begin is None and node.truth and all(node.counts.get(k,0)>2 for k in ('rgb','depth','scan','imu')):
                begin=node.now();result['sim_started']=begin;node.record('phase',phase='observing_zero',started=begin)
                # Snapshot live participants rather than inferring segment sizes after shutdown.
                live=wait_query(worker,receiver_snapshot,pump,min(deadline,time.monotonic()+5))
                qos={}
                for topic in ('/mooncar/rgb/image','/mooncar/depth/image'):
                    qos[topic]=[dict(node=e.node_name,gid=list(e.endpoint_gid),reliability=str(e.qos_profile.reliability),durability=str(e.qos_profile.durability),
                        history=None if e.qos_profile.history.value==3 else str(e.qos_profile.history),depth=None if e.qos_profile.history.value==3 else e.qos_profile.depth)
                        for e in node.get_publishers_info_by_topic(topic)]
                live['qos']=qos;(folder/'live-receiver.json').write_text(json.dumps(live,indent=2)+'\n')
            if begin is not None and node.now()-begin>=28:break
        else:raise TimeoutError('Formal launch collection timeout')
        pause=actual_pause(worker,name,pump,min(deadline,time.monotonic()+5),paused_state,last_message,node.now())
        result['pause_request']=pause
        result['actual_pause']=pause['raw'];state=pause['state'];end=state['sim']
        result.update(sim_ended=end,paused_state=state)
        until=min(deadline,time.monotonic()+5)
        while time.monotonic()<until:
            pump()
            if process.poll() is not None or node.errors:raise RuntimeError('Runtime failed while confirming paused reception tail')
            with node.lock:tail=reception_tail(list(node.rows),end,node.now())
            result['reception_tail']=tail
            result['csv_flush']=dict(passed=True,legacy_no_step_observer=True) if a.legacy else pause_flush(observer,state)
            flushed=result['csv_flush']['passed']
            if flushed and tail['passed']:break
        else:raise RuntimeError('Pause flush/tail confirmation missing')
        result['observer_flush_confirmed']=flushed
        result['reception_confirmed_wall']=time.monotonic()
        node.record('pause_confirmed',state=state,tail=tail,observer_flush=flushed)
        result['passed']=True
    except BaseException as e:result['error']=repr(e)
    finally:
        # Physics has paused and the observer has flushed before draining archive workers.
        try:
            if executor and node:
                result['receiver_before_close']=wait_query(worker,receiver_snapshot,pump,min(deadline,time.monotonic()+5))
            ready=list((folder/'bridge-receipts').glob('*/ready.json'))
            proof=json.loads(ready[0].read_text()) if len(ready)==1 else {}
            roles={node.image_writer.process.pid:'archive-worker'} if node and node.image_writer else {}
            result['live_participants']=wait_query(worker,participants,pump,min(deadline,time.monotonic()+5),proof.get('pid'),roles) if executor and node else {'passed':False}
        except Exception as error:result.update(passed=False,cleanup_error=repr(error))
        # Ownership/query failure must not bypass archive drain or lose evidence.
        try:result['capture']=node.close() if node else {'drained':False,'reason':'Recorder never initialized'}
        except Exception as error:result.update(passed=False,archive_cleanup_error=repr(error),capture={'drained':False})
        result['query_timeline']=worker.timeline;worker.close()
        try:
            if executor:executor.shutdown()
            if initialized:rclpy.shutdown()
        finally:
            stop_process(process);log.close();shutil.copy2(source,installed)
    try:
        observations=rows(folder/'observations.jsonl');begin=result['sim_started'];end=result['sim_ended']
        sensors={k:coverage([r.get('stamp',r['sim']) for r in observations if r['kind']==k],begin,end,bound)
            for k,bound in (('rgb',1.5),('depth',1.5),('scan',.75),('imu',.100001),('joints',.100001),('truth',.100001),('safe_command',.100001))}
        selected=[r for r in observations if r['kind'] in ('rgb','depth')]
        content=image_files(folder,observations);images=content['passed'];result['image_content']=content
        ids={r['event_id'] for r in observations};receipts=rows(folder/'writer-receipts.jsonl')
        drained=bool(result['capture']['drained'] and ids=={r['event_id'] for r in receipts} and len(ids)==len(observations)==len(receipts))
        safe=[r for r in observations if r['kind'] in ('safe_command','gz_command') and begin<=r['sim']<=end]
        zero=bool(safe and all(max(map(abs,r['values']))==0 for r in safe))
        ready=list((folder/'bridge-receipts').glob('*/ready.json'))
        bridge=json.loads(ready[0].read_text()) if len(ready)==1 else None
        stock_receivers=bool(live['verified'] and result['receiver_before_close']['verified'] and result['live_participants']['passed'])
        result.update(sensors=sensors,images_complete=images,event_receipts_complete=drained,commands_zero=zero,stock_receiver_verified=stock_receivers,
            production_bridge=bridge,legacy_no_custom_library=a.legacy and bridge is None,wall_seconds=time.monotonic()-started_wall)
        result['passed']=bool(result['passed'] and all(c['passed'] for c in sensors.values()) and images and drained and zero and stock_receivers and (bridge is not None if not a.legacy else bridge is None))
        if not a.legacy:
            from r5_control_analysis import analyze_steps
            step=analyze_steps(observer,result);result['steps']=step;result['passed']=result['passed'] and step['complete']
        result['performance_samples']=rows(folder/'resources.jsonl')
        result['endpoint']=next((r for r in reversed(observations) if r['kind']=='truth'),None)
        result['runtime_speed']=dict(sim_seconds=end-begin,wall_seconds=result['wall_seconds'],real_time_factor=(end-begin)/result['wall_seconds'])
        result['evidence_bytes']=sum(p.stat().st_size for p in folder.rglob('*') if p.is_file())
    except Exception as e:result.update(passed=False,analysis_error=repr(e))
    (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(name=a.name,passed=result['passed'],error=result.get('error'),sensors=result.get('sensors'))),flush=True)
    return 0 if result['passed'] else 1

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--name',required=True)
    p.add_argument('--scene',choices=('ramp','curve','survey'),default='ramp');p.add_argument('--legacy',action='store_true')
    args=p.parse_args()
    try:code=run(args)
    except BaseException as error:
        folder=Path(args.output)/args.name;folder.mkdir(exist_ok=True)
        path=folder/'result.json'
        result=json.loads(path.read_text()) if path.is_file() else {}
        result.update(passed=False,startup_or_cleanup_error=repr(error))
        path.write_text(json.dumps(result,indent=2)+'\n');code=1
    raise SystemExit(code)
