"""Actual isolated ROS callbacks with a substituted registration core; no Gazebo."""
from collections import OrderedDict
import copy
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, Imu

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
sys.path.insert(0, str(ROOT/'ros2_ws/src/mooncar_slam/scripts'))
from rgbd_odometry_node import RGBDOdometryNode, calibration_signature, depth_grid, gray_image


def stamp(message, seconds):
    value = round(seconds*1e9)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(value, 1_000_000_000)


def quaternion(target, rotation):
    target.x, target.y, target.z, target.w = map(float, rotation.as_quat())


def image(values, encoding, *, at=2., padding=0, bigendian=False):
    values = np.asarray(values)
    height, width = values.shape[:2]
    dtype = {'32FC1': '>f4' if bigendian else '<f4',
             '16UC1': '>u2' if bigendian else '<u2'}.get(encoding, 'u1')
    values = np.asarray(values, dtype=dtype)
    message = Image()
    message.header.frame_id = 'camera_optical'
    stamp(message, at)
    message.width, message.height = width, height
    message.encoding, message.is_bigendian = encoding, int(bigendian)
    message.step = len(values[0].tobytes())+padding
    message.data = b''.join(row.tobytes()+b'\xa5'*padding for row in values)
    return message


class PixelDecoderTests(unittest.TestCase):
    def test_padded_rgb_bgr_and_mono_preserve_pixel_locations(self):
        rgb = np.array([[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                        [[0, 0, 0], [255, 255, 255], [50, 50, 50]]], dtype=np.uint8)
        expected = np.array([[76, 150, 29], [0, 255, 50]], dtype=np.uint8)
        for encoding, pixels in (('rgb8', rgb), ('bgr8', rgb[:, :, ::-1])):
            with self.subTest(encoding=encoding):
                result = gray_image(image(pixels, encoding, padding=7))
                np.testing.assert_array_equal(result, expected)
                self.assertEqual(result.dtype, np.uint8)
                self.assertTrue(result.flags.c_contiguous)
        mono = gray_image(image(expected, 'mono8', padding=5))
        np.testing.assert_array_equal(mono, expected)
        self.assertTrue(mono.flags.c_contiguous)

    def test_big_endian_float_and_padded_mm_depth_keep_unknown_pixels_nan(self):
        calibration = (2., 4., 1., 1.)
        floats = image([[2., np.nan, np.inf], [0., .05, 6.1]], '32FC1',
                       padding=9, bigendian=True)
        result = depth_grid(floats, calibration)
        self.assertEqual(result.shape, (2, 3, 3))
        np.testing.assert_allclose(result[0, 0], [-.5, -.25, 2.])
        self.assertTrue(np.isnan(result.reshape(-1, 3)[1:]).all())
        for endian in (False, True):
            with self.subTest(bigendian=endian):
                millimetres = image([[1000, 2000, 0]], '16UC1', padding=5, bigendian=endian)
                values = depth_grid(millimetres, calibration)
                np.testing.assert_allclose(values[0, 0], [-.25, -.125, 1.])
                np.testing.assert_allclose(values[0, 1], [.5, -.25, 2.])
                self.assertTrue(np.isnan(values[0, 2]).all())

    def test_invalid_encoding_short_rows_and_buffers_reject(self):
        for decoder, message in ((gray_image, image([[1, 2]], 'mono8')),
                                 (lambda msg: depth_grid(msg, (2., 2., 1., 1.)),
                                  image([[1., 2.]], '32FC1'))):
            message.data = message.data[:-1]
            with self.assertRaises(ValueError):
                decoder(message)
        invalid = image([[1, 2]], 'mono8')
        invalid.encoding = 'rgba8'
        with self.assertRaises(ValueError):
            gray_image(invalid)
        invalid = image([[1., 2.]], '32FC1')
        invalid.step -= 1
        with self.assertRaises(ValueError):
            depth_grid(invalid, (2., 2., 1., 1.))
        with self.assertRaises(ValueError):
            depth_grid(image([[1.]], '32FC1'), (0., 2., 1., 1.))

    def test_rectified_calibration_rejects_distortion_skew_binning_and_crop(self):
        valid = CameraInfo()
        valid.header.frame_id, valid.width, valid.height = 'camera_optical', 8, 8
        valid.k = [4., 0., 3.5, 0., 4., 3.5, 0., 0., 1.]
        # Unfilled R/P is the explicit ideal-producer contract, not automatic
        # real-camera rectification. Populated fields must agree with K.
        self.assertIsNotNone(calibration_signature(valid))
        for attribute, value in (('d', [.01, 0., 0., 0., 0.]), ('binning_x', 2)):
            invalid = copy.deepcopy(valid)
            setattr(invalid, attribute, value)
            with self.subTest(attribute=attribute), self.assertRaises(ValueError):
                calibration_signature(invalid)
        for change in ('skew', 'crop', 'zero_homogeneous_coordinate'):
            invalid = copy.deepcopy(valid)
            if change == 'skew': invalid.k[1] = .1
            if change == 'crop': invalid.roi.x_offset = 1
            if change == 'zero_homogeneous_coordinate': invalid.k[8] = 0.
            with self.subTest(change=change), self.assertRaises(ValueError):
                calibration_signature(invalid)

    def test_identical_streams_with_conflicting_projection_or_rotation_are_invalid(self):
        info = CameraInfo()
        info.header.frame_id, info.width, info.height = 'camera_optical', 8, 8
        info.k = [4., 0., 3.5, 0., 4., 3.5, 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [4., 0., 3.5, 0., 0., 4., 3.5, 0., 0., 0., 1., 0.]
        self.assertIsNotNone(calibration_signature(info))
        for change in ('projection_focal_length', 'projection_translation', 'rectification_rotation'):
            invalid = copy.deepcopy(info)
            if change == 'projection_focal_length': invalid.p[0] = 5.
            if change == 'projection_translation': invalid.p[3] = .05
            if change == 'rectification_rotation':
                invalid.r = Rotation.from_euler('z', .1).as_matrix().ravel().tolist()
            with self.subTest(change=change):
                for same_stream in (invalid, copy.deepcopy(invalid)):
                    with self.assertRaises(ValueError):
                        calibration_signature(same_stream)


class RGBDWrapperTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'], domain_id=96)
        self.core = Mock(references=[{}])
        self.core.update.return_value = dict(accepted=True, reason='accepted', stamp=2.,
            xyz=np.array([.02, -.03, .04]), covariance=np.eye(3)*.001, inliers=100, rmse_m=.001)
        with patch.object(RGBDOdometryNode, 'create_core', return_value=self.core):
            self.node = RGBDOdometryNode(asynchronous_processing=False)
        for name in ('publisher', 'filtered_publisher', 'tf_broadcaster', 'status'):
            setattr(self.node, name, Mock())
        self.clock(2.02)
        self.mount = Rotation.from_euler('xyz', [.1, .2, .3])
        mount = TransformStamped()
        mount.header.frame_id, mount.child_frame_id = 'base_footprint', 'imu_link'
        quaternion(mount.transform.rotation, self.mount)
        self.node.buffer.set_transform_static(mount, 'rgbd-wrapper-test')
        self.camera_rotation = Rotation.from_matrix([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        for at, x in ((2., .14), (2.1, .9)):
            camera = TransformStamped()
            camera.header.frame_id, camera.child_frame_id = 'base_footprint', 'camera_optical'
            stamp(camera, at)
            camera.transform.translation.x, camera.transform.translation.z = x, .3
            quaternion(camera.transform.rotation, self.camera_rotation)
            self.node.buffer.set_transform(camera, 'rgbd-wrapper-test')
        self.info = CameraInfo()
        self.info.header.frame_id, self.info.width, self.info.height = 'camera_optical', 8, 8
        self.info.k = [4., 0., 3.5, 0., 4., 3.5, 0., 0., 1.]
        self.info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        self.info.p = [4., 0., 3.5, 0., 0., 4., 3.5, 0., 0., 0., 1., 0.]
        self.node.on_info(self.info)

    def tearDown(self):
        if self.node is not None:
            self.node.destroy_node()
        rclpy.shutdown()

    def clock(self, at):
        self.node.get_clock().set_ros_time_override(rclpy.time.Time(
            seconds=at, clock_type=rclpy.clock.ClockType.ROS_TIME))

    def synchronized(self, at=2.):
        body = Rotation.from_euler('xyz', [-.15, .1, .7])
        for offset in (-.01, .01):
            imu = Imu()
            imu.header.frame_id = 'imu_link'
            stamp(imu, at+offset)
            quaternion(imu.orientation, body*self.mount)
            self.node.on_imu(imu)
            wheel = Odometry()
            stamp(wheel, at+offset)
            self.node.on_wheel(wheel)
        return Rotation.from_euler('z', -.7)*body

    def ready(self):
        self.node.on_color_info(copy.deepcopy(self.info))
        return self.synchronized()

    def color(self, at=2.):
        return image(np.arange(64, dtype=np.uint8).reshape(8, 8), 'mono8', at=at, padding=3)

    def depth(self, at=2.):
        return image(np.full((8, 8), 2.), '32FC1', at=at, padding=8)

    def no_pose(self):
        self.node.publisher.publish.assert_not_called()
        self.node.filtered_publisher.publish.assert_not_called()
        self.node.tf_broadcaster.sendTransform.assert_not_called()

    def seed_healthy_quality(self):
        for at in (1.7, 1.8, 1.9):
            self.node.quality.observe(at, dict(accepted=True, reason='accepted'))
        self.node.report()
        self.assertTrue(json.loads(self.node.status.publish.call_args.args[0].data)['healthy'])

    def test_exact_pair_waits_for_both_calibrations_before_update(self):
        self.synchronized()
        self.node.on_color(self.color())
        self.node.on_depth(self.depth())
        self.core.update.assert_not_called()
        self.no_pose()
        self.node.on_color_info(copy.deepcopy(self.info))
        self.core.update.assert_called_once()
        self.assertEqual(self.core.update.call_args.args[0], 2.)
        transform = self.node.tf_broadcaster.sendTransform.call_args.args[0]
        self.assertEqual(transform.header.stamp.sec, 2)
        self.assertEqual(transform.header.stamp.nanosec, 0)
        self.assertAlmostEqual(transform.transform.translation.z, .04)
        status = json.loads(self.node.status.publish.call_args.args[0].data)
        self.assertEqual(status['localization_backend'], 'metric_rgbd_3d')
        self.assertFalse(status['truth_inputs'])
        self.assertIn('Exact-stamp rectified RGB', status['input_contract'])

    def test_depth_before_color_waits_for_exact_matching_exposure(self):
        self.ready()
        self.node.on_depth(self.depth())
        self.core.update.assert_not_called()
        self.node.on_color(self.color())
        self.core.update.assert_called_once()

    def test_initialized_visual_measurement_does_not_wait_for_optional_wheel_prediction(self):
        self.ready()
        self.node.wheel.clear()
        self.node.on_color(self.color())
        self.node.on_depth(self.depth())
        self.core.update.assert_called_once()
        self.assertEqual(self.core.update.call_args.args[4:6],(None,None))
        self.assertEqual(self.node.counts['rgbd_processing_without_wheel_prediction'],1)
        self.node.tf_broadcaster.sendTransform.assert_called_once()

    def test_initialization_still_requires_a_real_bracketing_wheel_measurement(self):
        self.ready()
        self.core.references=[]
        self.node.wheel.clear()
        self.node.on_color(self.color())
        self.node.on_depth(self.depth())
        self.core.update.assert_not_called()
        self.no_pose()
        self.assertEqual(self.node.pending_wait_reason,'waiting_initial_wheel_bracket')
        self.assertIsNone(self.node.quality.accepted_stamp)

    def test_tracking_projection_uses_the_same_exposure_and_startup_axes_as_depth(self):
        orientation=self.ready()
        self.node.on_color(self.color())
        self.node.on_depth(self.depth())
        call=self.core.update.call_args
        measured=call.args[2]
        model=call.kwargs['camera_model']
        expected_rotation=(orientation*self.camera_rotation).as_matrix()
        np.testing.assert_allclose(model['rotation'],expected_rotation,atol=1e-12)
        # A later TF at 2.1 has x=.9. The 2.0 exposure must retain x=.14.
        np.testing.assert_allclose(model['translation'],orientation.apply([.14,0.,.3]),atol=1e-12)
        camera=(measured-model['translation'])@model['rotation']
        projected=camera@model['K'].T
        pixels=projected[:,:,:2]/projected[:,:,2,None]-.5
        rows,columns=np.mgrid[:8,:8]
        np.testing.assert_allclose(pixels,np.stack((columns,rows),axis=2),atol=1e-12)

    def test_color_cannot_substitute_for_missing_depth_calibration(self):
        self.node.intrinsics = self.node.camera_frame = self.node.camera_size = None
        self.ready()
        self.node.on_color(self.color())
        self.node.on_depth(self.depth())
        self.core.update.assert_not_called()
        self.no_pose()
        self.node.on_info(self.info)
        self.core.update.assert_called_once()

    def test_mismatched_color_stamp_or_missing_color_never_invents_pose(self):
        self.ready()
        self.node.on_color(self.color(1.999))
        self.node.on_depth(self.depth())
        self.node.report()
        self.core.update.assert_not_called()
        self.no_pose()
        self.clock(2.301)
        self.node.process_pending()
        self.assertEqual(self.node.counts['depth_observation_stale'], 1)
        self.assertFalse(self.node.pending)
        self.no_pose()

    def test_unpaired_drop_only_counts_without_renewing_accepted_measurement(self):
        self.ready()
        self.seed_healthy_quality()
        self.node.on_depth(self.depth())
        self.assertEqual(len(self.node.pending), 1, 'Missing RGB without a newer exposure must wait')
        self.node.on_color(self.color(2.1))
        self.assertFalse(self.node.pending)
        self.assertEqual(self.node.counts['unpaired_depth_omitted'], 1)
        self.assertEqual(self.node.quality.accepted_stamp, 1.9)
        self.assertEqual(self.node.quality.last_input_stamp, 1.9)
        self.assertEqual(self.node.quality.successful_streak, 3)
        self.core.update.assert_not_called()
        self.no_pose()
        self.clock(2.201)
        self.node.report()
        status = json.loads(self.node.status.publish.call_args.args[0].data)
        self.assertFalse(status['healthy'])
        self.assertEqual(status['reason'], 'observation_stale')
        self.assertEqual(status['accepted_stamp'], 1.9)

    def test_unpaired_oldest_does_not_block_the_next_exact_pair(self):
        self.ready()
        self.seed_healthy_quality()
        self.synchronized(2.1)
        self.clock(2.12)
        self.core.update.return_value['stamp'] = 2.1
        self.node.on_depth(self.depth())
        self.node.on_depth(self.depth(2.1))
        self.assertEqual(len(self.node.pending), 2)
        self.core.update.assert_not_called()
        self.node.on_color(self.color(2.1))
        self.assertEqual(self.node.counts['unpaired_depth_omitted'], 1)
        self.core.update.assert_called_once()
        self.assertEqual(self.core.update.call_args.args[0], 2.1)
        self.assertFalse(self.node.pending)
        self.assertEqual(self.node.quality.accepted_stamp, 2.1)
        transform = self.node.tf_broadcaster.sendTransform.call_args.args[0]
        self.assertEqual(transform.header.stamp.nanosec, 100_000_000)

    def test_unpaired_skip_does_not_accept_a_subsequent_bad_fit(self):
        self.ready()
        self.seed_healthy_quality()
        self.synchronized(2.1)
        self.clock(2.12)
        self.core.update.return_value = dict(accepted=False, reason='uncertain_rgbd_translation')
        self.node.on_depth(self.depth())
        self.node.on_depth(self.depth(2.1))
        self.node.on_color(self.color(2.1))
        self.assertEqual(self.node.counts['unpaired_depth_omitted'], 1)
        self.assertEqual(self.node.counts['uncertain_rgbd_translation'], 1)
        self.assertEqual(self.node.quality.accepted_stamp, 1.9)
        self.assertFalse(json.loads(self.node.status.publish.call_args.args[0].data)['healthy'])
        self.no_pose()

    def test_unpaired_path_cannot_clear_a_latched_backwards_clock(self):
        self.ready()
        self.seed_healthy_quality()
        self.clock(1.99)
        self.node.report()
        self.assertEqual(self.node.quality.fault, 'clock_moved_backwards')
        self.clock(2.12)
        self.node.on_color(self.color(2.1))
        self.node.on_depth(self.depth())
        self.node.on_depth(self.depth(2.1))
        self.node.report()
        self.assertEqual(self.node.quality.fault, 'clock_moved_backwards')
        self.assertEqual(self.node.quality.accepted_stamp, 1.9)
        self.core.update.assert_not_called()
        self.no_pose()

    def test_already_stale_unpaired_depth_is_skipped_without_discarding_a_fresh_pair(self):
        self.ready()
        self.seed_healthy_quality()
        self.synchronized(2.4)
        camera = TransformStamped()
        camera.header.frame_id, camera.child_frame_id = 'base_footprint', 'camera_optical'
        stamp(camera, 2.4)
        camera.transform.translation.x, camera.transform.translation.z = .14, .3
        quaternion(camera.transform.rotation, self.camera_rotation)
        self.node.buffer.set_transform(camera, 'rgbd-wrapper-test')
        self.node.on_depth(self.depth())
        self.clock(2.42)
        self.node.on_color(self.color(2.4))
        self.assertEqual(self.node.counts['unpaired_depth_omitted'], 1)
        self.assertEqual(self.node.quality.accepted_stamp, 1.9)
        self.core.update.assert_not_called()
        self.no_pose()
        self.node.report()
        self.assertFalse(json.loads(self.node.status.publish.call_args.args[0].data)['healthy'])
        self.core.update.return_value['stamp'] = 2.4
        self.node.on_depth(self.depth(2.4))
        self.core.update.assert_called_once()
        self.assertEqual(self.node.quality.accepted_stamp, 2.4, 'Only the genuine newer fit renews the stamp')
        status = json.loads(self.node.status.publish.call_args.args[0].data)
        self.assertFalse(status['healthy'], 'A gap over .3s still requires genuine fit recovery')
        self.assertEqual(status['reason'], 'recovering')

    def test_color_buffer_is_bounded_to_ten_latest_frames(self):
        for at in range(1, 31):
            self.node.on_color(self.color(float(at)))
        self.assertIsInstance(self.node.colors, OrderedDict)
        self.assertEqual(list(self.node.colors), list(map(float, range(21, 31))))
        self.core.update.assert_not_called()

    def test_timestamped_camera_pose_full_imu_and_nan_pixel_positions_reach_core(self):
        orientation = self.ready()
        depth = np.full((8, 8), 2.)
        depth[0, 0], depth[1, 2], depth[2, 2] = 0., np.nan, .2
        self.node.on_color(self.color())
        self.node.on_depth(image(depth, '32FC1', padding=7))
        args = self.core.update.call_args.args
        self.assertEqual(args[1].shape, (8, 8))
        self.assertEqual(args[2].shape, (8, 8, 3))
        np.testing.assert_array_equal(args[1], np.arange(64).reshape(8, 8))
        self.assertTrue(np.isnan(args[2][0, 0]).all())
        self.assertTrue(np.isnan(args[2][1, 2]).all())
        self.assertTrue(np.isnan(args[2][2, 2]).all(), 'A rover self return stays at its pixel as NaN')
        # Pixel (u=4,v=0): x=.5,y=-1.5,z=2 in optical coordinates.
        expected = orientation.apply(self.camera_rotation.apply([.5, -1.5, 2.])+[.14, 0., .3])
        np.testing.assert_allclose(args[2][0, 4], expected, atol=1e-12)
        self.assertAlmostEqual(args[3], 0., places=12)
        self.assertGreater(self.node.counts['self_depth_returns_omitted'], 0)

    def test_rgb_depth_calibration_mismatch_latches_and_never_recovers_implicitly(self):
        changed = copy.deepcopy(self.info)
        changed.k[0] = 5.
        changed.p[0] = 5.
        self.node.on_color_info(changed)
        self.synchronized()
        self.node.on_color(self.color())
        self.node.on_depth(self.depth())
        self.assertEqual(self.node.quality.fault, 'rgb_depth_calibration_mismatch')
        self.core.update.assert_not_called()
        self.no_pose()
        self.assertFalse(json.loads(self.node.status.publish.call_args.args[0].data)['healthy'])

    def test_info_mismatch_latches_immediately_without_any_image_pair(self):
        changed = copy.deepcopy(self.info)
        changed.k[0] = changed.p[0] = 5.
        self.node.on_color_info(changed)
        self.assertEqual(self.node.quality.fault, 'rgb_depth_calibration_mismatch')
        self.assertFalse(self.node.pending)
        self.assertFalse(self.node.colors)
        self.core.update.assert_not_called()
        self.no_pose()
        status = json.loads(self.node.status.publish.call_args.args[0].data)
        self.assertFalse(status['healthy'])
        self.assertEqual(status['reason'], 'rgb_depth_calibration_mismatch')

    def test_both_streams_changing_calibration_require_a_new_producer_epoch(self):
        self.ready()
        changed = copy.deepcopy(self.info)
        changed.k[0] = changed.p[0] = 5.
        self.node.on_info(changed)
        self.assertEqual(self.node.quality.fault, 'camera_calibration_changed')
        self.node.on_color_info(copy.deepcopy(changed))
        self.node.on_color(self.color())
        self.node.on_depth(self.depth())
        self.assertIsNotNone(self.node.quality.fault)
        self.core.update.assert_not_called()
        self.no_pose()

    def test_different_projection_matrices_do_not_count_as_matched_calibration(self):
        changed = copy.deepcopy(self.info)
        changed.p[3] = .05
        self.node.on_color_info(changed)
        self.synchronized()
        self.node.on_color(self.color())
        self.node.on_depth(self.depth())
        self.assertEqual(self.node.quality.fault, 'invalid_rectified_rgb_calibration')
        self.core.update.assert_not_called()
        self.no_pose()

    def test_different_distortion_descriptions_do_not_count_as_matched_calibration(self):
        changed = copy.deepcopy(self.info)
        changed.d = [0., 0., 0., 0., 0.]
        self.node.on_color_info(changed)
        self.synchronized()
        self.node.on_color(self.color())
        self.node.on_depth(self.depth())
        self.assertEqual(self.node.quality.fault, 'rgb_depth_calibration_mismatch')
        self.core.update.assert_not_called()
        self.no_pose()

    def test_latched_calibration_change_cannot_emit_tf_after_old_signature_returns(self):
        self.ready()
        changed = copy.deepcopy(self.info)
        changed.k[0] = 5.
        changed.p[0] = 5.
        self.node.on_color_info(changed)
        self.assertEqual(self.node.quality.fault, 'camera_calibration_changed')
        self.node.on_color_info(copy.deepcopy(self.info))
        self.node.on_color(self.color())
        self.node.on_depth(self.depth())
        self.assertIsNotNone(self.node.quality.fault)
        self.core.update.assert_not_called()
        self.no_pose()

    def test_rejected_registration_heartbeat_and_future_image_emit_no_tf(self):
        self.ready()
        self.core.update.return_value = dict(accepted=False, reason='ambiguous_rgbd_translation')
        self.node.on_color(self.color())
        self.node.on_depth(self.depth())
        self.node.report()
        self.no_pose()
        self.assertEqual(self.node.counts['ambiguous_rgbd_translation'], 1)
        self.node.on_color(self.color(3.))
        self.node.on_depth(self.depth(3.))
        self.assertEqual(self.node.counts['depth_observation_from_future'], 1)
        self.assertEqual(self.core.update.call_count, 1)
        self.no_pose()

    def test_compute_finishing_after_sim_age_limit_never_publishes(self):
        self.ready()
        result = self.core.update.return_value
        def update(*args,**kwargs):
            self.clock(2.301)
            return result
        self.core.update.side_effect = update
        self.node.on_color(self.color())
        self.node.on_depth(self.depth())
        self.core.update.assert_called_once()
        self.assertEqual(self.node.counts['depth_observation_stale_after_compute'], 1)
        self.no_pose()

    def test_compute_finishing_after_wall_age_limit_never_publishes(self):
        self.ready()
        result = self.core.update.return_value
        wall = SimpleNamespace(now=time.monotonic())
        timer = SimpleNamespace(monotonic=lambda: wall.now, perf_counter=time.perf_counter)
        def update(*args,**kwargs):
            wall.now += .501
            return result
        self.core.update.side_effect = update
        with patch('rgbd_odometry_node.time', timer):
            self.node.on_color(self.color())
            self.node.on_depth(self.depth())
        self.core.update.assert_called_once()
        self.assertEqual(self.node.counts['depth_observation_stale_after_compute'], 1)
        self.no_pose()

    def test_destroy_node_closes_the_factory_owned_core(self):
        self.node.destroy_node()
        self.node = None
        self.core.close.assert_called_once_with()


if __name__ == '__main__':
    unittest.main(verbosity=2)
