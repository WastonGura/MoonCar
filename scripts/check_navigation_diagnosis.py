"""Natural mission with raw costmaps, terrain and commands; no injected decisions."""
import argparse
import json
import os
import subprocess
from pathlib import Path
import check_survey_recovery_runtime as runner

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('output',type=Path)
    parser.add_argument('--profile',choices=('legacy','r5_support_v1'),default='legacy')
    args=parser.parse_args()
    from simulation_experiment import SimulationExperiment
    original_apply=SimulationExperiment.apply
    def apply(self, task):
        task=dict(task,gui=False)
        if args.profile!='legacy':task['simulation_profile']=args.profile
        return original_apply(self,task)
    SimulationExperiment.apply=apply
    original=runner.make_witness
    def witness(folder,partition):
        value=original(folder,partition)
        from nav_msgs.msg import OccupancyGrid
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import PointCloud2, JointState
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        from sensor_msgs_py import point_cloud2
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
        value.diagnostic_subscriptions=[]
        value.dedicated_executor=True
        clock_file=(folder/'native-clock.jsonl').open('w')
        clock_process=subprocess.Popen(['gz','topic','-e','-t','/clock','-n','1000','--json-output'],
            stdout=clock_file,stderr=value.gzlog,env=dict(os.environ,GZ_PARTITION=partition))
        old_close=value.close
        def close():
            try:
                if clock_process.poll() is None:clock_process.terminate()
                try:clock_process.wait(timeout=5)
                except subprocess.TimeoutExpired:clock_process.kill();clock_process.wait()
                clock_file.close()
            finally:old_close()
        value.close=close
        value.diagnostic_subscriptions.append(value.create_subscription(JointState,'/joint_states',
            lambda m:value.record('full_joints',stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
                names=list(m.name),positions=list(m.position),velocities=list(m.velocity)),qos_profile_sensor_data))
        value.diagnostic_subscriptions.append(value.create_subscription(String,'/robot_description',
            lambda m:(folder/'actual-robot.urdf').write_text(m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        value.diagnostic_subscriptions.append(value.create_subscription(TFMessage,'/tf',
            lambda m:value.record('tf',transforms=[dict(parent=t.header.frame_id,child=t.child_frame_id,
                stamp_ns=t.header.stamp.sec*10**9+t.header.stamp.nanosec,
                xyz=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]) for t in m.transforms]),10))
        for topic in ('/local_costmap/costmap','/global_costmap/costmap','/mooncar/terrain/traversability_map'):
            def grid(msg,topic=topic):
                value.record('costmap',topic=topic,frame=msg.header.frame_id,resolution=msg.info.resolution,
                    origin=[msg.info.origin.position.x,msg.info.origin.position.y],width=msg.info.width,
                    height=msg.info.height,data=list(msg.data))
            value.diagnostic_subscriptions.append(value.create_subscription(OccupancyGrid,topic,grid,
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        value.diagnostic_subscriptions.append(value.create_subscription(Twist,'/cmd_vel',
            lambda m:value.record('raw_command',linear=m.linear.x,angular=m.angular.z),10))
        def cloud(m):
            points=point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=True)
            value.record('terrain_obstacles',frame=m.header.frame_id,points=[[float(p[0]),float(p[1]),float(p[2])] for p in points])
        value.diagnostic_subscriptions.append(value.create_subscription(PointCloud2,'/mooncar/terrain/obstacles',cloud,qos_profile_sensor_data))
        return value
    runner.make_witness=witness
    passed=runner.trial(args.output,'natural')
    (args.output/'diagnostic-profile.json').write_text(json.dumps(dict(profile=args.profile,injected=False)))
    return 0 if passed else 1

if __name__=='__main__':raise SystemExit(main())
