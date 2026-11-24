#!/usr/bin/env python3
"""Prove the calibration capacity gate runs before stopping DDS peer processes."""
import argparse, importlib.util, json, os, subprocess, sys, time
from pathlib import Path

def run(folder):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from image_dds_ownership import Lifecycle
    from image_dds_validation import receiver_snapshot,participants,active_receiver_gate,stop_process
    path=Path('/source/ros2_ws/src/mooncar_gazebo/launch/image_dds_guard.py')
    spec=importlib.util.spec_from_file_location('guard',path);guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)
    folder.mkdir();history=Lifecycle(folder/'lifecycle.jsonl');children=[];node=None;result=dict(passed=False,gazebo_attempts=0)
    try:
        rclpy.init();node=rclpy.create_node('teardown_primary_receiver');received=[]
        node.create_subscription(Image,'/boundary/image',lambda _:received.append(time.monotonic()),qos_profile_sensor_data)
        for mode in ('rx','tx'):
            env=dict(os.environ)
            if mode=='tx':env=guard.bridge_env(env,os.environ['MOONCAR_IMAGE_DDS_RUNTIME'],folder/'initialization.json')
            log=(folder/(mode+'.log')).open('x')
            child=subprocess.Popen([sys.executable,str(Path(__file__).with_name('preflight_image_dds_boundary.py')),
                '--output',str(folder),'--mode',mode],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            children.append((mode,child,log))
        rx=children[0][1];tx=children[1][1];until=time.monotonic()+12
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.01)
            if any(p.poll() is not None for _,p,_ in children):raise RuntimeError('Preflight peer exited')
            try:probe_frames=json.loads((folder/'received.json').read_text())
            except (FileNotFoundError,json.JSONDecodeError):continue
            if len(received)>5 and len(probe_frames)>5:break
        else:raise TimeoutError('Actual DDS peer receivers not ready')
        initial=dict(main=receiver_snapshot(history=history),**{'capture-probe':receiver_snapshot(rx.pid,history),
            'all_participants':participants(tx.pid,{rx.pid:'ros-node'},history)})
        final=participants(tx.pid,{rx.pid:'ros-node'},history)
        result.update(initial=initial,before_peer_shutdown=final,active_gate=active_receiver_gate(initial,final))
        if not result['active_gate']:raise RuntimeError('Active capacity proof failed')
        # Abrupt exit specifically exercises orphaned file/remote mapping cases.
        # No Gazebo is running; this does not alter a tested simulation process.
        tx.kill();tx.wait();stop_process(rx)
        result['after_peer_shutdown_diagnostic']=receiver_snapshot(history=history)
        result['post_shutdown_gate_not_used']=True
        result['passed']=True
    except BaseException as error:result['error']=repr(error)
    finally:
        for _,child,log in reversed(children):stop_process(child);log.close()
        result['lifecycle_capture']=history.close();result['passed']=result['passed'] and result['lifecycle_capture']['passed']
        if node:node.destroy_node();rclpy.shutdown()
        (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return 0 if result['passed'] else 1

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();raise SystemExit(run(Path(a.output)))
