#!/usr/bin/env python3
"""No Gazebo: real ROS image transmission and independent segment ownership."""
import argparse, importlib.util, json, os, signal, subprocess, sys, time
from pathlib import Path

def endpoint(mode, folder):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    if mode=='archive':
        # Import the same RMW as an archive worker, but do not create a participant.
        import ctypes
        ctypes.CDLL('/opt/ros/jazzy/lib/librmw_fastrtps_cpp.so')
        (folder/'archive-ready').write_text(str(os.getpid()))
        while True: time.sleep(.1)
    rclpy.init(); node=rclpy.create_node('boundary_'+mode); frames=[]
    if mode=='tx': publisher=node.create_publisher(Image,'/boundary/image',qos_profile_sensor_data)
    else:
        def receive(message):
            frames.append(dict(sec=message.header.stamp.sec,nanosec=message.header.stamp.nanosec,bytes=len(message.data),wall=time.monotonic()))
            (folder/'received.json').write_text(json.dumps(frames))
        node.create_subscription(Image,'/boundary/image',receive,qos_profile_sensor_data)
    (folder/(mode+'-ready')).write_text(str(os.getpid()))
    count=0
    try:
        while True:
            rclpy.spin_once(node,timeout_sec=.01)
            if mode=='tx':
                message=Image(); message.height=480; message.width=640; message.encoding='rgb8'; message.step=1920
                message.header.stamp.sec=count; message.data=bytes([count%255])*(480*1920)
                publisher.publish(message); count+=1; time.sleep(.05)
    finally: node.destroy_node(); rclpy.shutdown()

def run(folder):
    from image_dds_ownership import registry
    from image_dds_pause import QueryWorker,wait_query
    from image_dds_validation import stop_process
    folder.mkdir(); processes=[]; result=dict(passed=False,gazebo_attempts=0)
    path=Path('/source/ros2_ws/src/mooncar_gazebo/launch/image_dds_guard.py')
    spec=importlib.util.spec_from_file_location('guard',path); guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)
    try:
        guard.verify(os.environ['MOONCAR_IMAGE_DDS_RUNTIME'])
        for mode in ('rx','archive','tx'):
            env=dict(os.environ)
            if mode=='tx':env=guard.bridge_env(env,os.environ['MOONCAR_IMAGE_DDS_RUNTIME'],folder/'initialization.json')
            log=(folder/(mode+'.log')).open('x')
            child=subprocess.Popen([sys.executable,__file__,'--output',str(folder),'--mode',mode],env=env,
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True); processes.append((mode,child,log))
        until=time.monotonic()+12
        while time.monotonic()<until:
            if any(p.poll() is not None for _,p,_ in processes):raise RuntimeError('ROS preflight process exited')
            try:
                frames=json.loads((folder/'received.json').read_text())
                if len(frames)>=10 and (folder/'initialization.json').is_file() and (folder/'archive-ready').is_file():break
            except (FileNotFoundError,json.JSONDecodeError):pass
            time.sleep(.02)
        else:raise TimeoutError('Real ROS images not received')
        # Parent has no ROS participant. Do not add it as a required ROS node.
        from image_dds_ownership import inventory,snapshot
        data=inventory(); roles={p.pid:('bridge' if mode=='tx' else 'archive-worker' if mode=='archive' else 'ros-node') for mode,p,_ in processes}
        proof={str(pid):snapshot(pid,role,data,8388608 if role=='bridge' else 524288,role!='archive-worker') for pid,role in roles.items()}
        result['ownership']=proof; result['inventory']=data
        tx=next(r for r in proof.values() if r['role']=='bridge'); rx=next(r for r in proof.values() if r['role']=='ros-node')
        archive=next(r for r in proof.values() if r['role']=='archive-worker')
        result['real_ros']=dict(received_frames=len(frames),remote_bridge_mapping=any(s['owners'][0]['pid']==tx['pid'] for s in rx['remote_mappings']),
            archive_not_participant=not archive['participant'],initialization=json.loads((folder/'initialization.json').read_text()))
        if not all(r['verified'] for r in proof.values()) or not result['real_ros']['remote_bridge_mapping'] or archive['participant']:
            raise RuntimeError('Actual ownership method failed; no Gazebo allowed')
        # Run delayed subprocess queries while servicing a real ROS subscription
        # and a 20 ms heartbeat. This exercises the actual executor, not a mock.
        import rclpy
        from sensor_msgs.msg import Image
        from rclpy.qos import qos_profile_sensor_data
        rclpy.init(); node=rclpy.create_node('boundary_delay_observer'); receptions=[]; beats=[]
        node.create_subscription(Image,'/boundary/image',lambda _:receptions.append(time.monotonic()),qos_profile_sensor_data)
        from geometry_msgs.msg import Twist
        heartbeat=node.create_publisher(Twist,'/boundary/heartbeat',qos_profile_sensor_data)
        next_beat=time.monotonic()
        def pump():
            nonlocal next_beat
            rclpy.spin_once(node,timeout_sec=.005)
            now=time.monotonic()
            if now>=next_beat:heartbeat.publish(Twist()); beats.append(now); next_beat=now+.02
        warm=time.monotonic()+8
        while time.monotonic()<warm and len(receptions)<10:pump()
        if len(receptions)<10:raise RuntimeError('Delayed-query subscriber discovery not ready')
        delays=[]
        try:
            for delay,budget in ((.7,3),(2,4),(2,.4)):
                receptions.clear(); beats.clear(); worker=QueryWorker(); started=time.monotonic(); timed_out=False
                deadline=started+budget
                try:wait_query(worker,worker.command,pump,deadline,[sys.executable,'-c','import time; time.sleep('+str(delay)+')'],deadline)
                except TimeoutError:timed_out=True
                finally:worker.close()
                gaps=lambda values:max((b-a for a,b in zip(values,values[1:])),default=None)
                row=dict(delay_s=delay,budget_s=budget,timed_out=timed_out,received=len(receptions),heartbeats=len(beats),
                    receive_max_wall_gap_s=gaps(receptions),heartbeat_max_wall_gap_s=gaps(beats),query_timeline=worker.timeline)
                row['passed']=bool(timed_out==(budget<delay) and len(receptions)>2 and len(beats)>2 and row['heartbeat_max_wall_gap_s']<.1)
                delays.append(row)
            result['delayed_queries']=delays
            if not all(r['passed'] for r in delays):raise RuntimeError('Delayed query reception/heartbeat failed')
        finally:node.destroy_node();rclpy.shutdown()
        result['passed']=True
    except BaseException as error:result['error']=repr(error)
    finally:
        for _,child,log in reversed(processes):stop_process(child);log.close()
        (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return 0 if result['passed'] else 1

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);parser.add_argument('--mode',choices=('tx','rx','archive'))
    args=parser.parse_args();folder=Path(args.output)
    if args.mode:endpoint(args.mode,folder)
    else:raise SystemExit(run(folder))
