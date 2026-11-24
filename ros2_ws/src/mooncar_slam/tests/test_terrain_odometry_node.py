"""Actual ROS node callbacks; no ROS executor, Gazebo or actuation is started."""
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo,Image,Imu
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from terrain_odometry_node import TerrainOdometryNode,imu_at,outside_rover


def stamp(msg,t):
    value=round(t*1e9);msg.header.stamp.sec=value//1_000_000_000;msg.header.stamp.nanosec=value%1_000_000_000


def quaternion(target,rotation):
    target.x,target.y,target.z,target.w=map(float,rotation.as_quat())


class DepthWrapperTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=['--ros-args','-p','use_sim_time:=true'],domain_id=96)
        self.core=Mock(references=[{}])
        with patch.object(TerrainOdometryNode,'create_core',return_value=self.core):
            self.node=TerrainOdometryNode()
        for name in ('publisher','filtered_publisher','tf_broadcaster','status'):
            setattr(self.node,name,Mock())
        self.node.core.update.return_value=dict(accepted=True,reason='accepted',stamp=2.,xyz=np.array([.02,-.03,.04]),
                                               covariance=np.eye(3)*.001,inliers=100,rmse_m=.001)
        self.node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=2.02,clock_type=rclpy.clock.ClockType.ROS_TIME))
        self.mount=Rotation.from_euler('xyz',[.1,.2,.3])
        mount=TransformStamped();mount.header.frame_id='base_footprint';mount.child_frame_id='imu_link'
        quaternion(mount.transform.rotation,self.mount)
        self.node.buffer.set_transform_static(mount,'depth-wrapper-test')
        self.camera_rotation=Rotation.from_matrix([[0,0,1],[-1,0,0],[0,-1,0]])
        for t,x in ((2.,.14),(2.1,.9)):
            camera=TransformStamped();camera.header.frame_id='base_footprint';camera.child_frame_id='camera_optical';stamp(camera,t)
            camera.transform.translation.x=x;camera.transform.translation.z=.3
            quaternion(camera.transform.rotation,self.camera_rotation)
            self.node.buffer.set_transform(camera,'depth-wrapper-test')
        self.info=CameraInfo();self.info.header.frame_id='camera_optical';self.info.width=8;self.info.height=8
        self.info.k=[4.,0.,3.5,0.,4.,3.5,0.,0.,1.]
        self.node.on_info(self.info)

    def tearDown(self):
        if self.node is not None:self.node.destroy_node()
        rclpy.shutdown()

    def synchronized(self,t=2.,roll=-.15,pitch=.1,yaw=.7):
        body=Rotation.from_euler('xyz',[roll,pitch,yaw])
        for offset in (-.01,.01):
            imu=Imu();imu.header.frame_id='imu_link';stamp(imu,t+offset)
            quaternion(imu.orientation,body*self.mount)
            imu.angular_velocity.z=.2;self.node.on_imu(imu)
            wheel=Odometry();stamp(wheel,t+offset);self.node.on_wheel(wheel)
        return Rotation.from_euler('z',-yaw)*body

    def image(self,t=2.):
        image=Image();image.header.frame_id='camera_optical';stamp(image,t)
        image.width=8;image.height=8;image.step=32;image.encoding='32FC1';image.is_bigendian=0
        image.data=np.full((8,8),2.,dtype='<f4').tobytes()
        return image

    def test_timestamped_gimbal_tf_full_tilt_and_lever_arm_reach_core(self):
        orientation=self.synchronized();self.node.on_depth(self.image())
        call=self.node.core.update.call_args.args
        # Half-pixel sample centres and stride=4: u,v=(0,4), optical depth=2.
        optical=np.array([[-1.5,-1.5,2.],[.5,-1.5,2.],[-1.5,.5,2.],[.5,.5,2.]])
        expected=orientation.apply(self.camera_rotation.apply(optical)+[.14,0.,.3])
        np.testing.assert_allclose(call[1],expected,atol=1e-12)
        self.assertEqual(call[0],2.)
        self.assertAlmostEqual(call[2],0.,places=12)
        transform=self.node.tf_broadcaster.sendTransform.call_args.args[0]
        self.assertEqual(transform.header.stamp.sec,2)
        self.assertAlmostEqual(transform.transform.translation.z,.04)
        self.assertEqual(transform.transform.rotation.x,0.)
        self.assertEqual(transform.transform.rotation.y,0.)

    def test_rejected_registration_and_heartbeat_never_publish_position(self):
        self.synchronized();self.node.core.update.return_value=dict(accepted=False,reason='unobservable_depth_translation')
        self.node.on_depth(self.image());self.node.report()
        self.node.filtered_publisher.publish.assert_not_called()
        self.node.tf_broadcaster.sendTransform.assert_not_called()
        self.assertEqual(len(self.node.pending),0)

    def test_missing_imu_bracket_cannot_extrapolate(self):
        self.synchronized()
        self.node.imu.clear()
        q=Rotation.identity().as_quat()
        self.node.imu.extend([(1.5,q,0.),(2.1,q,0.)])
        self.node.on_depth(self.image())
        self.node.core.update.assert_not_called()
        self.node.tf_broadcaster.sendTransform.assert_not_called()

    def test_camera_calibration_change_latches_and_clears_pending(self):
        self.node.pending.append((self.image(),0.))
        self.info.k[0]=5.
        self.node.on_info(self.info)
        self.assertEqual(self.node.quality.fault,'camera_calibration_changed')
        self.assertEqual(len(self.node.pending),0)
        self.assertFalse(json.loads(self.node.status.publish.call_args.args[0].data)['healthy'])
        self.info.k[0]=4.;self.node.on_info(self.info)
        self.synchronized();self.node.on_depth(self.image())
        self.assertEqual(self.node.quality.fault,'camera_calibration_changed')
        self.core.update.assert_not_called()
        self.node.tf_broadcaster.sendTransform.assert_not_called()

    def test_old_image_is_omitted_instead_of_refreshing_tf(self):
        self.synchronized();self.node.on_depth(self.image(1.))
        self.node.core.update.assert_not_called()
        self.node.tf_broadcaster.sendTransform.assert_not_called()
        self.assertEqual(len(self.node.pending),0)

    def test_rover_surface_is_filtered_before_attitude_rotation(self):
        points=np.array([[0.,0.,.2],[.49,.44,.59],[.51,0.,.2],[0.,.46,.2],[0.,0.,-.11],[0.,0.,.61]])
        np.testing.assert_array_equal(outside_rover(points),[False,False,True,True,True,True])

    def test_compute_finishing_after_sim_age_limit_does_not_publish(self):
        self.synchronized();result=self.core.update.return_value
        def update(*args):
            self.node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=2.301,
                clock_type=rclpy.clock.ClockType.ROS_TIME))
            return result
        self.core.update.side_effect=update
        self.node.on_depth(self.image())
        self.core.update.assert_called_once()
        self.assertEqual(self.node.counts['depth_observation_stale_after_compute'],1)
        self.node.tf_broadcaster.sendTransform.assert_not_called()
        self.node.filtered_publisher.publish.assert_not_called()

    def test_compute_finishing_after_wall_age_limit_does_not_publish(self):
        self.synchronized();result=self.core.update.return_value
        wall=SimpleNamespace(now=time.monotonic())
        timer=SimpleNamespace(monotonic=lambda:wall.now,perf_counter=time.perf_counter)
        def update(*args):
            wall.now+=.501
            return result
        self.core.update.side_effect=update
        with patch('terrain_odometry_node.time',timer):self.node.on_depth(self.image())
        self.assertEqual(self.node.counts['depth_observation_stale_after_compute'],1)
        self.node.tf_broadcaster.sendTransform.assert_not_called()
        self.node.filtered_publisher.publish.assert_not_called()

    def test_destroy_node_closes_the_factory_owned_core(self):
        self.node.destroy_node();self.node=None
        self.core.close.assert_called_once_with()


class AttitudeInterpolationTests(unittest.TestCase):
    def test_so3_interpolation_preserves_tilt_and_short_wrap(self):
        a=Rotation.from_euler('xyz',[.1,.2,3.1]);b=Rotation.from_euler('xyz',[.1,.2,-3.1])
        orientation,rate=imu_at([(1.,a.as_quat(),.2),(1.02,b.as_quat(),.4)],1.01)
        expected=Rotation.from_euler('xyz',[.1,.2,math.pi])
        self.assertLess((orientation.inv()*expected).magnitude(),1e-12)
        self.assertAlmostEqual(rate,.3)
        self.assertIsNone(imu_at([(1.,a.as_quat(),.2),(1.2,b.as_quat(),.4)],1.01))
        self.assertIsNone(imu_at([(1.,a.as_quat(),.2),(1.02,b.as_quat(),.4)],1.03))


if __name__=='__main__':unittest.main(verbosity=2)
