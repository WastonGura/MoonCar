"""Real ROS node construction and message layout, no simulator or fake health."""
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import Mock
import unittest
import numpy as np
import rclpy
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from rgbd_window_odometry import WindowOdometryNode

class ROSAuthorityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory()
        self.node=WindowOdometryNode(SimpleNamespace(output=Path(self.folder.name)/'poses.jsonl',cached_query=True,
            image_reliability='reliable',readiness_wait_wall_s=.5))
        self.node.publisher=Mock();self.node.filtered_publisher=Mock();self.node.tf_broadcaster=Mock();self.node.status=Mock()
        self.node.sim_now=lambda:1_200_000_000
    def tearDown(self):
        self.node.finish();self.node.destroy_node();self.folder.cleanup()
    def write(self,stamp=1_000_000_000,**kw):
        self.node.sensor_for_publication=SimpleNamespace(stamp_ns=stamp,nominal_R_wb=np.eye(3));self.node.sensor_wz=.12
        row=dict(type='plan',stamp_ns=stamp,planned_stamp_ns=stamp,output=True,position=[.2,.3,.1],
            covariance_conditional=np.diag([.001,.002,.003]).tolist(),gauge_stamp_ns=900_000_000,
            received_wall_ns=time.monotonic_ns(),cache_age_s=.5,cache_version=1)
        row.update(kw);self.node.write_plan(row)
    def test_single_pose_tf_exposure_and_covariance_layout(self):
        self.write();msg=self.node.publisher.publish.call_args.args[0]
        self.assertEqual(msg.header.frame_id,'odom');self.assertEqual(msg.child_frame_id,'base_footprint')
        self.assertEqual(msg.header.stamp.sec,1);self.assertEqual(msg.header.stamp.nanosec,0)
        self.assertEqual(msg.pose.pose.position.x,.2);self.assertEqual(msg.pose.covariance[7],.002)
        self.assertEqual(msg.twist.twist.angular.z,.12)
        tf=self.node.tf_broadcaster.sendTransform.call_args.args[0]
        self.assertEqual(tf.header.stamp,msg.header.stamp);self.assertEqual(tf.transform.translation.y,.3)
    def test_health_requires_three_fresh_successes(self):
        self.write();self.assertFalse(self.node.quality.snapshot(1.2)['healthy'])
        self.write(1_100_000_000);self.assertFalse(self.node.quality.snapshot(1.2)['healthy'])
        self.write(1_200_000_000);self.assertTrue(self.node.quality.snapshot(1.2)['healthy'])
        self.assertFalse(self.node.quality.snapshot(1.6)['healthy'])
    def test_stale_or_failed_pose_does_not_publish_or_renew_health(self):
        self.write(800_000_000,gauge_stamp_ns=500_000_000)
        self.write(output=False)
        self.node.publisher.publish.assert_not_called();self.assertIsNone(self.node.quality.accepted_stamp)
    def test_source_fault_never_falls_back_to_a_legacy_pose(self):
        self.node.set_fault('test_source_fault');self.write()
        self.node.publisher.publish.assert_not_called();self.node.tf_broadcaster.sendTransform.assert_not_called()
        self.assertFalse(self.node.quality.snapshot(1.2)['healthy'])
    def test_closing_records_failures_without_publishing_after_context_stop(self):
        self.node.closing=True
        self.write();self.node.fail_plan(1_100_000_000,None,'missing_exposure_at_shutdown')
        self.node.report()
        self.node.publisher.publish.assert_not_called();self.node.status.publish.assert_not_called()

if __name__=='__main__':unittest.main()
