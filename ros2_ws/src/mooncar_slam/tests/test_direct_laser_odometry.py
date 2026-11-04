"""The optional direct backend must never manufacture fresh positions on loss."""
from collections import deque
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, LaserScan

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from laser_odometry_node import LaserOdometryNode
from localization_quality import LocalizationQuality


class DirectOdometryTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=['--ros-args','-p','direct_mode:=true','-p','heading_origin:=0.3'],domain_id=115)
        self.node=LaserOdometryNode()
        self.node.publisher=Mock();self.node.filtered_publisher=Mock();self.node.tf_broadcaster=Mock()
        self.node.localized_scan_publisher=Mock()

    def tearDown(self):
        self.node.destroy_node();rclpy.shutdown()

    def result(self):
        return dict(accepted=True,reason='accepted',stamp=1.,xy=np.array([.12,-.03]),yaw=.2,
                    covariance=np.eye(2)*.001,angular_velocity_z=.4)

    def test_accepted_scan_is_the_only_direct_tf_and_filtered_pose_source(self):
        scan=LaserScan();scan.header.stamp.sec=1
        self.node.publish(scan,self.result())
        message=self.node.filtered_publisher.publish.call_args.args[0]
        transform=self.node.tf_broadcaster.sendTransform.call_args.args[0]
        self.assertEqual(message.header.stamp.sec,1)
        self.assertEqual(transform.header.stamp.sec,1)
        self.assertEqual(transform.header.frame_id,'odom');self.assertEqual(transform.child_frame_id,'base_footprint')
        self.assertEqual(transform.transform.translation.x,.12)
        self.assertEqual(transform.transform.translation.y,-.03)
        self.assertEqual(message.twist.twist.angular.z,.4)

    def test_rejected_match_and_wall_heartbeat_never_advance_direct_tf(self):
        node=self.node;node.imu=deque([(.99,0.,.4),(1.01,0.,.4)])
        node.wheel=deque([(.99,0.,.4),(1.01,0.,.4)])
        node.core=Mock();node.core.update.return_value=dict(accepted=False,reason='too_few_line_correspondences')
        scan=LaserScan();scan.header.stamp.sec=1;node.pending.append(scan)
        node.process_pending();node.report();node.reject_input('unsynchronized')
        node.core.update.assert_called_once()
        node.publisher.publish.assert_not_called();node.filtered_publisher.publish.assert_not_called()
        node.tf_broadcaster.sendTransform.assert_not_called()
        node.localized_scan_publisher.publish.assert_not_called()

    def test_slam_scans_are_forwarded_only_after_quality_recovery(self):
        scan=LaserScan();scan.header.stamp.sec=1
        self.node.publish(scan,self.result())
        self.node.localized_scan_publisher.publish.assert_not_called()
        self.node.quality.ready_since_stamp=1.
        self.node.publish(scan,self.result())
        self.node.localized_scan_publisher.publish.assert_called_once_with(scan)

    def test_imu_mount_and_declared_spawn_heading_are_applied_once(self):
        transform=TransformStamped();transform.header.frame_id='base_footprint';transform.child_frame_id='imu_link'
        transform.transform.rotation.z=math.sin(.2/2);transform.transform.rotation.w=math.cos(.2/2)
        node=self.node;node.buffer.set_transform_static(transform,'test')
        node.process_pending=Mock()
        msg=Imu();msg.header.frame_id='imu_link';msg.header.stamp.sec=1
        msg.orientation.z=math.sin(.8/2);msg.orientation.w=math.cos(.8/2);msg.angular_velocity.z=.4
        node.on_imu(msg)
        self.assertAlmostEqual(node.imu[-1][1],.8-.2-.3,places=12)
        self.assertAlmostEqual(node.imu[-1][2],.4,places=12)

    def test_gap_average_is_not_published_as_current_velocity(self):
        self.node.last_accepted=(.1,np.array([-.4,0.]))
        scan=LaserScan();scan.header.stamp.sec=1
        self.node.publish(scan,self.result())
        message=self.node.filtered_publisher.publish.call_args.args[0]
        self.assertEqual(message.twist.twist.linear.x,0.)
        self.assertEqual(message.twist.covariance[0],1e6)

    def test_new_fit_updates_quality_without_waiting_for_wall_heartbeat(self):
        node=self.node;node.status=Mock()
        node.get_clock=Mock(return_value=Mock(now=Mock(return_value=Mock(nanoseconds=1020000000))))
        for stamp in (.4,.6,.8):
            node.quality.observe(stamp,dict(accepted=True,reason='accepted'))
        node.imu=deque([(.99,0.,.4),(1.01,0.,.4)])
        node.wheel=deque([(.99,0.,.4),(1.01,0.,.4)])
        node.core=Mock();node.core.update.return_value=self.result()
        scan=LaserScan();scan.header.stamp.sec=1;node.pending.append(scan)
        # No executor spin or heartbeat timer invocation occurs here.
        node.process_pending()
        node.status.publish.assert_called_once()
        payload=json.loads(node.status.publish.call_args.args[0].data)
        self.assertEqual(payload['accepted_stamp'],1.)
        self.assertTrue(payload['healthy'])
        consumer=LocalizationQuality(required=True);consumer.observe(payload,10.)
        self.assertTrue(consumer.check(1.11,10.01)['healthy'])
        # The superseded .8 measurement would already be over the .3 s limit.
        self.assertGreater(1.11-.8,.3)


class LegacyOwnershipTests(unittest.TestCase):
    def test_default_candidate_still_leaves_tf_ownership_to_ekf(self):
        rclpy.init(domain_id=116)
        node=LaserOdometryNode()
        try:
            self.assertFalse(node.direct_mode)
            self.assertIsNone(node.filtered_publisher);self.assertIsNone(node.tf_broadcaster)
        finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
