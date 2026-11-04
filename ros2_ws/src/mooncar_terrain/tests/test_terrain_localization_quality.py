"""Candidate localization faults must not enter terrain layers or raw observations."""
import json
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.clock import ClockType
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from terrain_node import TerrainNode
from localization_quality import LaserQualitySource


class TerrainLocalizationQualityTests(unittest.TestCase):
    def setUp(self):
        arguments = ['--ros-args', '-p', 'use_sim_time:=true']
        if self._testMethodName != 'test_default_disabled_retains_depth_processing':
            arguments += ['-p', 'require_laser_localization:=true']
        rclpy.init(args=arguments, domain_id=97)
        self.node = TerrainNode()
        self.set_time(10.)
        for name in ('cloud_pub', 'observation_pub', 'traversability_pub', 'diag_pub'):
            setattr(self.node, name, Mock())
        info = CameraInfo()
        info.header.frame_id = 'depth_optical'
        info.width = info.height = 48
        info.k = [100., 0., 24., 0., 100., 24., 0., 0., 1.]
        self.node.on_info(info)
        for child, x, z in (('base_footprint', 0., 0.), ('depth_optical', 1., .3)):
            transform = TransformStamped()
            transform.header.frame_id = 'odom'
            transform.child_frame_id = child
            transform.transform.translation.x = x
            transform.transform.translation.z = z
            transform.transform.rotation.w = 1.
            self.node.buffer.set_transform_static(transform, 'quality-test')
        self.source = LaserQualitySource(session_id='terrain-test')

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def set_time(self, stamp):
        self.node.get_clock().set_ros_time_override(Time(seconds=stamp, clock_type=ClockType.ROS_TIME))

    def observe(self, stamp):
        message = Image()
        message.header.stamp = Time(seconds=stamp).to_msg()
        message.header.frame_id = 'depth_optical'
        message.width = message.height = 48
        message.step = 48 * 4
        message.encoding = '32FC1'
        message.data = np.full((48, 48), 2., dtype='<f4').tobytes()
        imu = Imu()
        imu.header.stamp = message.header.stamp
        imu.header.frame_id = 'imu_link'
        imu.orientation.w = 1.
        self.node.on_imu(imu)
        self.node.on_depth(message)

    def healthy(self):
        for stamp in (9.8, 9.9, 10.):
            self.source.observe(stamp, dict(accepted=True, reason='accepted'))
        self.status(10.)

    def status(self, stamp):
        self.node.on_localization_quality(String(data=json.dumps(self.source.snapshot(stamp))))

    def assert_no_depth_mutation(self):
        self.assertIsNone(self.node.pending)
        self.assertEqual(self.node.last_processed, 0.)
        self.assertEqual(self.node.last_valid, 0.)
        self.assertEqual(self.node.traversability_cells, {(0, 0): (13, 1.)})
        self.node.cloud_pub.publish.assert_not_called()
        self.node.observation_pub.publish.assert_not_called()
        self.node.traversability_pub.publish.assert_not_called()

    def test_uninitialized_depth_cannot_mutate_existing_layers(self):
        self.node.traversability_cells = {(0, 0): (13, 1.)}
        self.observe(10.)
        self.node.process_pending()
        self.assert_no_depth_mutation()

    def test_pre_ready_and_beyond_measurement_depth_are_discarded(self):
        self.healthy()
        self.node.traversability_cells = {(0, 0): (13, 1.)}
        for stamp in (9.99, 10.11):
            with self.subTest(stamp=stamp):
                self.observe(stamp)
                self.node.process_pending()
                self.assert_no_depth_mutation()

    def test_stale_status_drops_depth_before_processing(self):
        self.healthy()
        self.node.traversability_cells = {(0, 0): (13, 1.)}
        self.observe(10.)
        with patch('terrain_node.time.monotonic', return_value=time.monotonic() + .6):
            self.node.process_pending()
        self.assert_no_depth_mutation()

    def test_match_fault_drops_pending_and_fresh_recovery_restores_processing(self):
        self.healthy()
        self.node.traversability_cells = {(0, 0): (13, 1.)}
        self.observe(10.)
        self.source.reject('too_few_line_correspondences')
        self.status(10.)
        self.assert_no_depth_mutation()
        for stamp in (10.02, 10.04, 10.06):
            self.source.observe(stamp, dict(accepted=True, reason='accepted'))
        self.set_time(10.06)
        self.status(10.06)
        self.observe(10.)  # Fault-time backlog cannot be admitted after recovery.
        self.node.process_pending()
        self.assert_no_depth_mutation()
        self.observe(10.06)
        self.node.process_pending()
        self.node.cloud_pub.publish.assert_called_once()
        self.node.observation_pub.publish.assert_called_once()
        self.node.traversability_pub.publish.assert_called_once()
        self.assertGreater(self.node.last_valid, 0.)

    def test_default_disabled_retains_depth_processing(self):
        self.assertFalse(self.node.require_laser_localization)
        self.assertFalse(any(s.topic_name == '/mooncar/laser_odometry/status'
                             for s in self.node.subscriptions))
        self.observe(10.)
        self.node.process_pending()
        self.node.cloud_pub.publish.assert_called_once()
        self.node.observation_pub.publish.assert_called_once()
        self.node.traversability_pub.publish.assert_called_once()


if __name__ == '__main__':
    unittest.main()
