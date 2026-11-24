"""Actual ROS subscription contracts and a large reliable RGB/depth transfer."""
import json
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
from rclpy.exceptions import ParameterImmutableException, InvalidParameterTypeException
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(ROOT/'ros2_ws/src/mooncar_slam/scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
import test_rgbd_odometry_node as fixtures
from rgbd_odometry_node import RGBDOdometryNode
from terrain_odometry_node import TerrainOdometryNode

RGB='/mooncar/localization_rgb/image'
DEPTH='/mooncar/depth/image'
RGB_INFO='/mooncar/localization_rgb/camera_info'
DEPTH_INFO='/mooncar/depth/camera_info'


class RGBDImageQoSTests(unittest.TestCase):
    def setUp(self):
        self.nodes=[]

    def tearDown(self):
        for node in reversed(self.nodes):node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

    def initialize(self,policy=None):
        args=['--ros-args','-p','use_sim_time:=true']
        if policy is not None:args+=['-p','rgbd_image_reliability:='+policy]
        rclpy.init(args=args,domain_id=96)

    def make_node(self,node_class=RGBDOdometryNode):
        self.core=Mock(references=[])
        with patch.object(node_class,'create_core',return_value=self.core):
            node=node_class()
        self.nodes.append(node)
        for name in ('publisher','filtered_publisher','tf_broadcaster','status'):
            setattr(node,name,Mock())
        node.get_clock().set_ros_time_override(rclpy.time.Time(
            seconds=2.02,clock_type=rclpy.clock.ClockType.ROS_TIME))
        return node

    def camera_subscriptions(self,node):
        return {sub.topic_name:sub for sub in node.subscriptions if sub.topic_name in (RGB,DEPTH,RGB_INFO,DEPTH_INFO)}

    def assert_qos(self,subscriptions,topics,reliability,depth):
        self.assertEqual(set(subscriptions),set(topics))
        for topic,subscription in subscriptions.items():
            with self.subTest(topic=topic):
                qos=subscription.qos_profile
                self.assertEqual(qos.reliability,reliability)
                self.assertEqual(qos.history,HistoryPolicy.KEEP_LAST)
                self.assertEqual(qos.depth,depth)
                self.assertEqual(qos.durability,DurabilityPolicy.VOLATILE)

    def test_standalone_rgbd_keeps_best_effort_default_for_all_four_streams(self):
        self.initialize()
        node=self.make_node()
        self.assert_qos(self.camera_subscriptions(node),(RGB,DEPTH,RGB_INFO,DEPTH_INFO),ReliabilityPolicy.BEST_EFFORT,5)
        self.assertEqual(node.get_parameter('rgbd_image_reliability').value,'best_effort')

    def test_explicit_reliable_applies_to_both_images_and_both_calibrations(self):
        self.initialize('reliable')
        node=self.make_node()
        self.assert_qos(self.camera_subscriptions(node),(RGB,DEPTH,RGB_INFO,DEPTH_INFO),ReliabilityPolicy.RELIABLE,2)
        node.report()
        payload=json.loads(node.status.publish.call_args.args[0].data)
        self.assertEqual(payload['rgbd_image_reliability'],'reliable')
        self.assertEqual(payload['rgbd_image_queue_depth'],2)
        self.assertFalse(payload['healthy'])

    def test_explicit_best_effort_is_a_preserved_option(self):
        self.initialize('best_effort')
        node=self.make_node()
        self.assert_qos(self.camera_subscriptions(node),(RGB,DEPTH,RGB_INFO,DEPTH_INFO),ReliabilityPolicy.BEST_EFFORT,5)

    def test_legacy_depth_and_inertial_wheel_qos_remain_unchanged(self):
        self.initialize('reliable')
        node=self.make_node(TerrainOdometryNode)
        self.assert_qos(self.camera_subscriptions(node),(DEPTH,DEPTH_INFO),ReliabilityPolicy.BEST_EFFORT,5)
        subscriptions={sub.topic_name:sub for sub in node.subscriptions}
        self.assertEqual(subscriptions['/mooncar/imu'].qos_profile.reliability,ReliabilityPolicy.BEST_EFFORT)
        self.assertEqual(subscriptions['/mooncar/imu'].qos_profile.depth,5)
        self.assertEqual(subscriptions['/odom'].qos_profile.reliability,ReliabilityPolicy.RELIABLE)
        self.assertEqual(subscriptions['/odom'].qos_profile.depth,50)

    def test_reliability_is_read_only_and_cannot_silently_change_live_subscriptions(self):
        self.initialize('reliable')
        node=self.make_node()
        self.assertTrue(node.describe_parameter('rgbd_image_reliability').read_only)
        try:results=node.set_parameters([Parameter('rgbd_image_reliability',value='best_effort')])
        except ParameterImmutableException:pass
        else:self.assertTrue(all(not result.successful for result in results))
        self.assertEqual(node.get_parameter('rgbd_image_reliability').value,'reliable')
        self.assert_qos(self.camera_subscriptions(node),(RGB,DEPTH,RGB_INFO,DEPTH_INFO),ReliabilityPolicy.RELIABLE,2)

    def test_unknown_reliability_is_rejected_instead_of_falling_back(self):
        self.initialize('reliable_typo')
        node=RGBDOdometryNode.__new__(RGBDOdometryNode)
        self.nodes.append(node)
        with patch.object(RGBDOdometryNode,'create_core',return_value=Mock(references=[])):
            with self.assertRaisesRegex(ValueError,'best_effort or reliable'):
                node.__init__(asynchronous_processing=False)

    def test_nonstring_reliability_is_rejected_by_the_declared_parameter_type(self):
        self.initialize('true')
        node=RGBDOdometryNode.__new__(RGBDOdometryNode)
        self.nodes.append(node)
        with patch.object(RGBDOdometryNode,'create_core',return_value=Mock(references=[])):
            with self.assertRaises(InvalidParameterTypeException):
                node.__init__(asynchronous_processing=False)

    def spin_until(self,node,predicate,seconds=3.):
        deadline=time.monotonic()+seconds
        while not predicate() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        self.assertTrue(predicate(),'Reliable ROS transfer did not reach the expected callback state')

    def test_reliable_320x240_images_reach_real_callbacks_without_inventing_pose(self):
        self.initialize('reliable')
        node=self.make_node()
        qos=QoSProfile(depth=2,reliability=ReliabilityPolicy.RELIABLE)
        publishers={topic:node.create_publisher(Image if topic in (RGB,DEPTH) else CameraInfo,topic,qos)
                    for topic in (RGB,DEPTH,RGB_INFO,DEPTH_INFO)}
        self.spin_until(node,lambda:all(pub.get_subscription_count()>0 for pub in publishers.values()))
        rgb=fixtures.image(np.arange(240*320*3,dtype=np.uint8).reshape(240,320,3),'rgb8')
        depth=fixtures.image(np.full((240,320),2.,dtype=np.float32),'32FC1')
        self.assertEqual(len(rgb.data),230400)
        self.assertEqual(len(depth.data),307200)
        publishers[RGB].publish(rgb);publishers[DEPTH].publish(depth)
        self.spin_until(node,lambda:node.last_depth_stamp==2. and node.last_color_stamp==2.)
        self.assertEqual(bytes(node.colors[2.][0].data),bytes(rgb.data))
        self.assertEqual(bytes(node.pending[0][0].data),bytes(depth.data))
        self.assertIsNone(node.intrinsics)
        self.assertIsNone(node.color_info)
        self.core.update.assert_not_called()
        node.tf_broadcaster.sendTransform.assert_not_called()
        info=CameraInfo();info.header.frame_id='camera_optical';info.width=320;info.height=240
        fixtures.stamp(info,2.)
        info.k=[200.,0.,159.5,0.,200.,119.5,0.,0.,1.]
        publishers[DEPTH_INFO].publish(info);publishers[RGB_INFO].publish(info)
        self.spin_until(node,lambda:node.depth_info is not None and node.color_info is not None)
        self.assertFalse(node.imu)
        self.core.update.assert_not_called()
        node.tf_broadcaster.sendTransform.assert_not_called()
        node.publisher.publish.assert_not_called()
        node.filtered_publisher.publish.assert_not_called()
        node.report()
        payload=json.loads(node.status.publish.call_args.args[0].data)
        self.assertFalse(payload['healthy'])
        self.assertIsNone(payload['accepted_stamp'])


if __name__=='__main__':unittest.main(verbosity=2)
