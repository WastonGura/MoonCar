"""Exercise actual IMU ingestion and scan localization at nonzero spawn headings.

Analytic room geometry generates sensor fixtures only. No truth or correction is
passed into the node; the expected world pose uses the declared setup transform.
"""
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from laser_odometry_core import rotation
from laser_odometry_node import LaserOdometryNode


ANGLES=np.linspace(-math.pi,math.pi,720,endpoint=False)


def stamp(message,value):
    nanoseconds=round(value*1e9)
    message.header.stamp.sec=nanoseconds//1_000_000_000
    message.header.stamp.nanosec=nanoseconds%1_000_000_000


def room_scan(world_xy,world_yaw):
    origin=np.asarray(world_xy)+rotation(world_yaw)@np.array([.14,0.])
    directions=np.c_[np.cos(ANGLES+world_yaw),np.sin(ANGLES+world_yaw)]
    intersections=[]
    for axis,limits in ((0,(-4.,4.)),(1,(-3.,3.))):
        for limit in limits:
            with np.errstate(divide='ignore',invalid='ignore'):
                distance=(limit-origin[axis])/directions[:,axis]
            intersections.append(np.where(distance>0,distance,np.inf))
    return np.min(intersections,axis=0)


class HeadingReferenceTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p','direct_mode:=true',
                         '-p','heading_origin:=1.5707963267948966','-p','imu_heading_reference:=startup'],domain_id=95)
        self.node=LaserOdometryNode()
        for name in ('publisher','filtered_publisher','localized_scan_publisher','tf_broadcaster','status'):
            setattr(self.node,name,Mock())
        mount=TransformStamped();mount.header.frame_id='base_footprint';mount.child_frame_id='imu_link'
        mount.transform.rotation.z=math.sin(.2/2);mount.transform.rotation.w=math.cos(.2/2)
        self.node.buffer.set_transform_static(mount,'heading-reference-test')
        laser=TransformStamped();laser.header.frame_id='base_footprint';laser.child_frame_id='lidar_scan_frame'
        laser.transform.translation.x=.14;laser.transform.rotation.w=1.
        self.node.buffer.set_transform_static(laser,'heading-reference-test')

    def tearDown(self):
        self.node.destroy_node();rclpy.shutdown()

    def imu(self,t,body_yaw,angular_velocity=.0):
        msg=Imu();msg.header.frame_id='imu_link';stamp(msg,t)
        msg.orientation.z=math.sin((body_yaw+.2)/2);msg.orientation.w=math.cos((body_yaw+.2)/2)
        msg.angular_velocity.z=angular_velocity
        self.node.on_imu(msg)
        return msg

    def test_measured_startup_reference_removes_mount_once_and_ignores_world_spawn_heading(self):
        self.node.process_pending=Mock()
        self.imu(1.,.63)
        self.assertAlmostEqual(self.node.imu_heading_offset,.63,places=12)
        self.assertAlmostEqual(self.node.imu[-1][1],0.,places=12)
        self.imu(1.1,.88,.4)
        self.assertAlmostEqual(self.node.imu[-1][1],.25,places=12)
        self.assertAlmostEqual(self.node.imu[-1][2],.4,places=12)
        self.node.report()
        payload=json.loads(self.node.status.publish.call_args.args[0].data)
        self.assertEqual(payload['imu_heading_reference'],'startup')
        self.assertAlmostEqual(payload['imu_heading_offset'],.63)

    def test_invalid_first_orientation_or_gyro_cannot_establish_reference(self):
        self.node.process_pending=Mock()
        msg=Imu();msg.header.frame_id='imu_link';stamp(msg,1.)
        msg.orientation.w=0.
        self.node.on_imu(msg)  # All-zero quaternion is invalid.
        self.assertIsNone(self.node.imu_heading_offset)
        self.imu(1.1,1.,math.nan)
        self.assertIsNone(self.node.imu_heading_offset)
        self.assertIsNone(self.node.last_yaw)
        self.imu(1.2,.5)
        self.assertAlmostEqual(self.node.imu_heading_offset,.5)
        self.assertAlmostEqual(self.node.imu[-1][1],0.)

    def test_wrap_and_rejected_duplicate_never_recalibrate_reference(self):
        self.node.process_pending=Mock()
        self.imu(1.,3.)
        self.imu(1.1,-3.083185307179586)
        self.assertAlmostEqual(self.node.imu[-1][1],.2,places=12)
        self.imu(1.1,.5)  # Rejected duplicate cannot silently change the origin.
        self.imu(1.2,-2.983185307179586)
        self.assertAlmostEqual(self.node.imu_heading_offset,3.,places=12)
        self.assertAlmostEqual(self.node.imu[-1][1],.3,places=12)

    def test_actual_scan_pipeline_and_declared_setup_agree_at_nonzero_headings(self):
        # First heading reproduces the failed +90-degree slope experiment.
        # Other headings rule out a hard-coded quarter-turn correction.
        for case,initial_yaw in enumerate((math.pi/2,-math.pi/2,.7)):
            with self.subTest(initial_yaw=initial_yaw):
                node=self.node
                node.imu.clear();node.wheel.clear();node.pending.clear()
                node.core=None;node.last_yaw=None;node.imu_heading_offset=None;node.last_accepted=None
                node.quality=type(node.quality)()
                node.heading_origin=initial_yaw
                node.filtered_publisher.reset_mock()
                def feed(t,local_xy,wheel_vx=0.):
                    node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=t+.02,clock_type=rclpy.clock.ClockType.ROS_TIME))
                    for offset in (-.01,.01):
                        self.imu(t+offset,0.)  # Gazebo startup-relative heading, not world yaw.
                        wheel=Odometry();stamp(wheel,t+offset);wheel.twist.twist.linear.x=wheel_vx
                        node.on_wheel(wheel)
                    scan=LaserScan();scan.header.frame_id='lidar_scan_frame';stamp(scan,t)
                    scan.angle_min=float(ANGLES[0]);scan.angle_increment=float(2*math.pi/720)
                    scan.range_min=.08;scan.range_max=12.
                    scan.ranges=room_scan(rotation(initial_yaw)@np.asarray(local_xy),initial_yaw).astype(float).tolist()
                    node.on_scan(scan)
                base=case*10.
                for i in range(13):feed(base+1.+i*.1,[0.,0.])
                self.assertIsNotNone(node.core.tree)
                step=np.array([.04,.015])
                feed(base+2.3,step,.2)
                for i in range(1,13):feed(base+2.3+i*.1,step)
                measured=node.filtered_publisher.publish.call_args.args[0]
                local=np.array([measured.pose.pose.position.x,measured.pose.pose.position.y])
                world=np.array([-5.,3.])+rotation(initial_yaw)@local
                expected=np.array([-5.,3.])+rotation(initial_yaw)@step
                np.testing.assert_allclose(world,expected,atol=.003)
                self.assertAlmostEqual(measured.pose.pose.orientation.z,0.,places=12)


if __name__=='__main__':unittest.main(verbosity=2)
