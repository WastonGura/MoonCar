#!/usr/bin/env python3
"""Real ROS participant lifecycle and stale remote mappings, without Gazebo."""
import argparse, json, os, subprocess, sys, time
from pathlib import Path

def sender(folder):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    rclpy.init();node=rclpy.create_node('short_lived_image_sender')
    publisher=node.create_publisher(Image,'/lifecycle/image',qos_profile_sensor_data)
    message=Image();message.width=640;message.height=480;message.encoding='rgb8';message.step=1920;message.data=bytes([100])*921600
    (folder/'sender-ready').write_text(str(os.getpid()))
    try:
        while not (folder/'release-sender').is_file():
            publisher.publish(message);rclpy.spin_once(node,timeout_sec=.02)
    finally:node.destroy_node();rclpy.shutdown()

def run(folder):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from image_dds_ownership import Lifecycle,snapshot,inventory
    from image_dds_validation import stop_process
    folder.mkdir();history=Lifecycle(folder/'lifecycle.jsonl');child=None;node=None;result=dict(passed=False,gazebo_attempts=0)
    try:
        rclpy.init();node=rclpy.create_node('lifecycle_image_receiver');received=[]
        node.create_subscription(Image,'/lifecycle/image',lambda _:received.append(time.monotonic()),qos_profile_sensor_data)
        log=(folder/'sender.log').open('x')
        child=subprocess.Popen([sys.executable,__file__,'--output',str(folder),'--sender'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        until=time.monotonic()+12;live=None
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.01)
            if child.poll() is not None:raise RuntimeError('Transient sender exited before observation')
            if len(received)<5:continue
            live=snapshot(history=history)
            if live['verified'] and any(s['owners'][0]['pid']==child.pid for s in live['remote_mappings']):break
        else:raise RuntimeError('Real ROS transient sender mapping not observed')
        result['live']=live;result['received_frames']=len(received)
        # Ensure the observation has archived an exact creator identity before exit.
        until=time.monotonic()+2
        while time.monotonic()<until:
            with history.lock:known=any(r['segment']['owners'][0]['pid']==child.pid for r in history.history.values())
            if known:break
            time.sleep(.01)
        if not known:raise RuntimeError('Transient creator ownership was not recorded')
        (folder/'release-sender').write_text('release');child.wait(5)
        if child.returncode:raise RuntimeError('Transient sender shutdown failed')
        # Immediately inspect the real receiving participant while remote cleanup runs.
        result['without_history']=snapshot()
        result['after_exit']=snapshot(history=history)
        if not result['after_exit']['verified']:raise RuntimeError('Exited sender mapping cannot be proven')
        result['stale_mapping_reproduced']=bool(result['after_exit']['retired_remote_mappings'])
        result['passed']=True
    except BaseException as error:result['error']=repr(error)
    finally:
        if child:stop_process(child);log.close()
        result['lifecycle_capture']=history.close()
        result['passed']=bool(result['passed'] and result['lifecycle_capture']['passed'])
        if node:node.destroy_node();rclpy.shutdown()
        (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return 0 if result['passed'] else 1

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);parser.add_argument('--sender',action='store_true');args=parser.parse_args()
    if args.sender:sender(Path(args.output))
    else:raise SystemExit(run(Path(args.output)))
