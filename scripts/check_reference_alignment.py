"""Read-only, time-matched Gazebo model / ROS map pose spot check (no alignment fit)."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time

from survey_evaluation import write_json


def capture(output):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, TransformException
    if output.exists(): raise ValueError('Preserve existing alignment record; choose a new output')
    if not os.environ.get('GZ_PARTITION') or not os.environ.get('ROS_DOMAIN_ID'):
        raise ValueError('Select the experiment GZ_PARTITION and ROS_DOMAIN_ID explicitly')
    rclpy.init()
    node = Node('terrain_reference_alignment_check', parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer = Buffer(node=node); listener = TransformListener(buffer,node)
    rows = []
    try:
        deadline = time.monotonic()+5
        while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.1)
        for _ in range(3):
            process = subprocess.Popen(['gz','topic','-e','-t','/world/lunar_yard/pose/info','-n','1','--json-output'],
                                       stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            # Drain concurrently: a whole scene message can exceed the pipe capacity.
            import concurrent.futures
            with concurrent.futures.ThreadPoolExecutor(max_workers=1) as worker:
                future=worker.submit(process.communicate)
                deadline=time.monotonic()+15
                while not future.done() and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.05)
                if not future.done(): process.kill()
                stdout,stderr=future.result(timeout=5)
            if process.returncode: raise RuntimeError(stderr or 'Gazebo pose query failed')
            message=json.JSONDecoder().raw_decode(stdout.lstrip())[0]
            truth=next(p for p in message['pose'] if p.get('name')=='mooncar')
            stamp=message['header']['stamp']
            at=Time(seconds=int(stamp.get('sec',0)),nanoseconds=int(stamp.get('nsec',0)))
            deadline=time.monotonic()+5
            while True:
                try: pose=buffer.lookup_transform('map','base_footprint',at); break
                except TransformException:
                    if time.monotonic()>=deadline: raise
                    rclpy.spin_once(node,timeout_sec=.05)
            def yaw(q):
                return math.atan2(2*(q.get('w',0)*q.get('z',0)+q.get('x',0)*q.get('y',0)),
                                  1-2*(q.get('y',0)**2+q.get('z',0)**2))
            p,q=pose.transform.translation,pose.transform.rotation
            measured=dict(x=p.x,y=p.y,z=p.z,yaw=yaw(dict(x=q.x,y=q.y,z=q.z,w=q.w)))
            true=truth['position']; angle=measured['yaw']-yaw(truth['orientation'])
            rows.append(dict(stamp=stamp, gazebo_model=truth, map_base_footprint=measured,
                             xy_error_m=math.hypot(p.x-true.get('x',0),p.y-true.get('y',0)),
                             yaw_error_deg=math.degrees(math.atan2(math.sin(angle),math.cos(angle)))))
        output.parent.mkdir(parents=True,exist_ok=True)
        write_json(output,dict(status='synchronized_pose_spot_check_only', partition=os.environ['GZ_PARTITION'],
                   ros_domain=os.environ['ROS_DOMAIN_ID'], samples=rows, fitted_correction=False,
                   limitations=['Three samples at the current position do not validate archived trajectories or global alignment.',
                                'XY/yaw use URDF model-root base_footprint convention; Z is retained but not accepted as height calibration.']))
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('output',type=Path)
    capture(parser.parse_args().output)
