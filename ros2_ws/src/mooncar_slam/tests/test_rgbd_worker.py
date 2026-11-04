"""Isolated actual ROS worker lifecycle and fail-closed publication boundaries."""
import copy
from dataclasses import FrozenInstanceError
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from geometry_msgs.msg import TransformStamped

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_rgbd_odometry_node as fixtures
from test_rgbd_odometry_node import quaternion, stamp
from rgbd_odometry_node import RGBDOdometryNode


class RGBDWorkerTests(unittest.TestCase):
    clock = fixtures.RGBDWrapperTests.clock
    synchronized = fixtures.RGBDWrapperTests.synchronized
    ready = fixtures.RGBDWrapperTests.ready
    color = fixtures.RGBDWrapperTests.color
    depth = fixtures.RGBDWrapperTests.depth
    no_pose = fixtures.RGBDWrapperTests.no_pose
    seed_healthy_quality = fixtures.RGBDWrapperTests.seed_healthy_quality

    def setUp(self):
        fixtures.RGBDWrapperTests.setUp(self)
        self.entered = threading.Event()
        self.release = threading.Event()
        self.node.processing_async = True
        self.ready()
        self.seed_healthy_quality()

    def tearDown(self):
        self.release.set()
        fixtures.RGBDWrapperTests.tearDown(self)

    def wait_until(self, predicate, timeout=2.):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic()<deadline:
            threading.Event().wait(.005)
        self.assertTrue(predicate(), 'Worker did not reach the expected bounded state')

    def start_worker(self):
        self.node.processing_thread = threading.Thread(target=self.node._processing_loop,
            name='rgbd-isolated-test-worker', daemon=True)
        self.node.processing_thread.start()

    def block_first_fit(self):
        result = self.core.update.return_value
        def update(at, *args, **kwargs):
            if not self.entered.is_set():
                self.entered.set()
                if not self.release.wait(3.):
                    raise RuntimeError('Test did not release the blocked fit')
            return dict(result, stamp=at)
        self.core.update.side_effect = update
        self.start_worker()
        self.node.on_color(self.color())
        self.node.on_depth(self.depth())
        self.assertTrue(self.entered.wait(1.), 'Core never received the first exposure')
        self.assertTrue(self.node.processing_inflight)

    def queue(self, at):
        self.synchronized(at)
        camera = TransformStamped()
        camera.header.frame_id, camera.child_frame_id = 'base_footprint', 'camera_optical'
        stamp(camera, at)
        camera.transform.translation.x, camera.transform.translation.z = .14, .3
        quaternion(camera.transform.rotation, self.camera_rotation)
        self.node.buffer.set_transform(camera, 'rgbd-worker-test')
        color, depth = self.color(at), self.depth(at)
        self.node.on_color(color)
        self.node.on_depth(depth)
        return color, depth

    def test_blocked_compute_leaves_callbacks_free_and_keeps_only_latest_waiter(self):
        self.block_first_fit()
        self.clock(2.12)
        callbacks_done = threading.Event()
        def callbacks():
            self.queue(2.05)
            self.queue(2.1)
            callbacks_done.set()
        thread = threading.Thread(target=callbacks, daemon=True)
        thread.start()
        self.assertTrue(callbacks_done.wait(.7), 'ROS sensor callbacks blocked behind core fit')
        thread.join()
        self.assertEqual(self.node.processing_latest.stamp, 2.1)
        self.assertEqual(self.node.counts['rgbd_queued_exposures_omitted'], 1)
        self.assertEqual(self.node.quality.accepted_stamp, 1.9,
            'Queued/dropped raw exposures must not renew accepted measurement')
        self.assertEqual(self.core.update.call_count, 1)
        self.no_pose()
        self.release.set()
        self.wait_until(lambda: self.core.update.call_count==2 and not self.node.processing_inflight)
        self.assertEqual([call.args[0] for call in self.core.update.call_args_list], [2., 2.1])
        self.assertEqual(self.node.quality.accepted_stamp, 2.1)

    def test_heartbeat_reports_inflight_without_waiting_for_core(self):
        self.block_first_fit()
        reported = threading.Event()
        def report():
            self.node.report()
            reported.set()
        reporter = threading.Thread(target=report, daemon=True)
        reporter.start()
        self.assertTrue(reported.wait(.7), 'Status heartbeat blocked behind registration')
        reporter.join()
        payload = json.loads(self.node.status.publish.call_args.args[0].data)
        self.assertTrue(payload['rgbd_processing_async'])
        self.assertTrue(payload['rgbd_processing_inflight'])
        self.assertIsNone(payload['rgbd_pending_exposure_stamp'])
        self.assertEqual(payload['accepted_stamp'], 1.9)
        self.assertEqual(payload['rgbd_last_depth_stamp'], 2.)
        self.no_pose()

    def test_default_constructor_owns_and_stops_a_real_worker_thread(self):
        self.node.destroy_node()
        self.node = None
        self.core = Mock(references=[])
        with patch.object(RGBDOdometryNode, 'create_core', return_value=self.core):
            self.node = RGBDOdometryNode()
        self.assertTrue(self.node.processing_async)
        self.assertIsNotNone(self.node.processing_thread)
        self.assertTrue(self.node.processing_thread.is_alive())
        node = self.node
        node.destroy_node()
        self.assertFalse(node.processing_thread.is_alive())
        self.core.close.assert_called_once_with()
        self.node = None

    def test_waiting_bundle_copies_pixels_and_exposure_geometry(self):
        self.block_first_fit()
        self.clock(2.12)
        color, depth = self.queue(2.1)
        bundle = self.node.processing_latest
        self.assertIsInstance(bundle.depth.data, bytes)
        with self.assertRaises(FrozenInstanceError):
            bundle.stamp = 99.
        color.data = bytes(len(color.data))
        depth.data = bytes(len(depth.data))
        self.info.k[0] = 999.
        self.release.set()
        self.wait_until(lambda: self.core.update.call_count==2 and not self.node.processing_inflight)
        call = self.core.update.call_args_list[-1]
        np.testing.assert_array_equal(call.args[1], np.arange(64).reshape(8, 8))
        self.assertTrue(np.isfinite(call.args[2]).all())
        self.assertEqual(call.kwargs['camera_model']['K'][0, 0], 4.)

    def test_sim_clock_advances_during_fit_and_stale_result_never_publishes(self):
        self.block_first_fit()
        self.clock(2.301)
        self.release.set()
        self.wait_until(lambda: not self.node.processing_inflight)
        self.assertEqual(self.node.counts['depth_observation_stale_after_compute'], 1)
        self.assertEqual(self.node.quality.accepted_stamp, 1.9)
        self.no_pose()

    def test_wall_clock_ages_during_fit_and_stale_result_never_publishes(self):
        wall = SimpleNamespace(now=time.monotonic())
        timer = SimpleNamespace(monotonic=lambda: wall.now, perf_counter=time.perf_counter)
        with patch('rgbd_odometry_node.time', timer):
            self.block_first_fit()
            wall.now += .501
            self.release.set()
            self.wait_until(lambda: not self.node.processing_inflight)
        self.assertEqual(self.node.counts['depth_observation_stale_after_compute'], 1)
        self.assertEqual(self.node.quality.accepted_stamp, 1.9)
        self.no_pose()

    def test_clock_rollback_without_report_cannot_commit_inflight_pose(self):
        self.block_first_fit()
        self.clock(1.99)
        self.release.set()
        self.wait_until(lambda: not self.node.processing_inflight)
        self.assertEqual(self.node.quality.fault, 'clock_moved_backwards')
        self.assertEqual(self.node.quality.accepted_stamp, 1.9)
        self.no_pose()

    def test_calibration_fault_invalidates_inflight_and_old_info_cannot_restore_it(self):
        self.block_first_fit()
        changed = copy.deepcopy(self.info)
        changed.k[0] = changed.p[0] = 5.
        self.node.on_color_info(changed)
        self.assertEqual(self.node.quality.fault, 'camera_calibration_changed')
        self.node.on_color_info(copy.deepcopy(self.info))
        self.release.set()
        self.wait_until(lambda: not self.node.processing_inflight)
        self.assertEqual(self.node.counts['rgbd_results_invalidated'], 1)
        self.assertEqual(self.node.quality.accepted_stamp, 1.9)
        self.assertEqual(self.core.update.call_count, 1)
        self.no_pose()

    def test_reject_generation_discards_inflight_and_waiting_results(self):
        self.block_first_fit()
        self.clock(2.12)
        self.queue(2.1)
        self.node.reject('invalid_rgbd_input')
        self.assertIsNone(self.node.processing_latest)
        self.release.set()
        self.wait_until(lambda: not self.node.processing_inflight)
        self.assertEqual(self.node.counts['rgbd_results_invalidated'], 1)
        self.assertEqual(self.node.quality.accepted_stamp, 1.9)
        self.assertEqual(self.core.update.call_count, 1)
        self.no_pose()

    def test_already_stale_bundle_does_not_even_enter_core(self):
        self.node.on_color(self.color())
        self.node.on_depth(self.depth())
        bundle = self.node.processing_latest
        self.node.processing_latest = None
        self.clock(2.301)
        self.node._process_exposure(bundle)
        self.core.update.assert_not_called()
        self.assertEqual(self.node.counts['depth_observation_stale_before_compute'], 1)
        self.assertEqual(self.node.quality.accepted_stamp, 1.9)
        self.no_pose()

    def test_destroy_waits_for_worker_before_closing_owned_core(self):
        self.block_first_fit()
        node = self.node
        errors = []
        def close():
            try: node.destroy_node()
            except Exception as error: errors.append(error)
        closer = threading.Thread(target=close, daemon=True)
        closer.start()
        self.wait_until(lambda: node.processing_closing)
        self.core.close.assert_not_called()
        self.release.set()
        closer.join(2.)
        self.assertFalse(closer.is_alive())
        self.assertFalse(errors)
        self.assertFalse(node.processing_thread.is_alive())
        self.core.close.assert_called_once_with()
        self.no_pose()
        self.node = None

    def test_unexpected_worker_error_latches_and_preserves_live_shutdown(self):
        self.core.update.side_effect = RuntimeError('Deliberate registration exception')
        self.start_worker()
        self.node.on_color(self.color())
        self.node.on_depth(self.depth())
        self.wait_until(lambda: self.node.quality.fault is not None and not self.node.processing_inflight)
        self.assertEqual(self.node.quality.fault, 'rgbd_processing_failed')
        self.assertTrue(self.node.processing_thread.is_alive())
        self.assertEqual(self.node.quality.accepted_stamp, 1.9)
        self.no_pose()

    def test_worker_exception_cannot_overwrite_a_prior_calibration_fault(self):
        def update(*args, **kwargs):
            self.entered.set()
            self.release.wait(3.)
            raise RuntimeError('Exception after an independent calibration fault')
        self.core.update.side_effect = update
        self.start_worker()
        self.node.on_color(self.color())
        self.node.on_depth(self.depth())
        self.assertTrue(self.entered.wait(1.))
        changed = copy.deepcopy(self.info)
        changed.k[0] = changed.p[0] = 5.
        self.node.on_color_info(changed)
        self.release.set()
        self.wait_until(lambda: not self.node.processing_inflight)
        self.assertEqual(self.node.quality.fault, 'camera_calibration_changed')
        self.assertEqual(self.node.quality.accepted_stamp, 1.9)
        self.no_pose()


if __name__ == '__main__':
    unittest.main(verbosity=2)
