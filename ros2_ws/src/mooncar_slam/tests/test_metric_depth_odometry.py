"""Independent analytic surface and byte-layout regressions; no captured world."""
from pathlib import Path
import struct
import sys
import threading
import time
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from metric_depth_odometry import MetricDepthOdometry, project_depth, rotation


def surface(x, y):
    return .10*np.sin(1.1*x)+.08*np.cos(1.4*y)+.04*np.sin(.7*x+1.3*y)


def measured_surface(xyz, yaw=0., rng=None):
    # A moving camera samples fresh coordinates on an analytic surface; points
    # are already gravity aligned, as required by the core contract. This is not
    # the same stored point list merely translated to manufacture exact matches.
    forward, lateral = np.meshgrid(np.arange(.4, 3.21, .10), np.arange(-1.6, 1.61, .10))
    footprint = np.c_[forward.ravel(), lateral.ravel()] @ rotation(yaw).T
    xy = footprint+np.asarray(xyz)[:2]
    points = np.c_[footprint, surface(xy[:, 0], xy[:, 1])-xyz[2]]
    if rng is not None:
        points += rng.normal(0., .003, points.shape)
    return points


class DepthRegistrationTests(unittest.TestCase):
    def initialized(self, seed=982):
        core, rng = MetricDepthOdometry(), np.random.default_rng(seed)
        result = core.update(1., measured_surface([0., 0., 0.], rng=rng), 0., 0., 0.)
        self.assertTrue(result['accepted'], result)
        return core, rng

    def test_observed_translation_includes_height_and_lateral_motion(self):
        core, rng = self.initialized()
        target = np.array([.035, -.025, .022])
        result = core.update(1.1, measured_surface(target, rng=rng), 0., 0., 0.)
        self.assertTrue(result['accepted'], result)
        np.testing.assert_allclose(result['xyz'], target, atol=.008)
        self.assertGreater(result['xyz'][2], .01)
        self.assertTrue(np.linalg.eigvalsh(result['covariance']).min() > 0.)

    def test_same_observed_cloud_cannot_self_generate_translation(self):
        points = measured_surface([0., 0., 0.], rng=np.random.default_rng(512))
        core = MetricDepthOdometry()
        first = core.update(1., points, 0., 0., 0.)
        self.assertTrue(first['accepted'], first)
        for index in range(1, 21):
            result = core.update(1.+index*.1, points.copy(), 0., 0., 0.)
            self.assertTrue(result['accepted'], result)
            np.testing.assert_allclose(result['xyz'], 0., atol=1e-10)
        self.assertEqual(len(core.references), 1)

    def test_ten_metre_range_and_turn_handover_with_biased_wheel_prediction(self):
        core, rng = self.initialized(1171)
        original = core.references[0]
        initial_target = original['target'].copy()
        errors = []
        for index in range(1, 201):
            x = index*.05
            xyz = [x, .04*np.sin(x*.5), .06*np.sin(x*.3)]
            result = core.update(1.+index*.1, measured_surface(xyz, rng=rng), 0., .5*.98, 0.)
            self.assertTrue(result['accepted'], (xyz, result))
            errors.append(np.linalg.norm(result['xyz']-xyz))
        self.assertGreater(result['reference_id'], original['id'])
        self.assertLess(np.percentile(errors, 95), .03)
        self.assertLess(np.linalg.norm(result['xyz']-xyz), .03)
        self.assertLessEqual(len(core.references), core.max_frames)
        self.assertIs(core.references[0], original)
        stamp = core.stamp
        # Rotation changes the observed patch without supplying camera yaw to
        # translation fitting. Each point cloud has already used IMU attitude.
        for yaw in np.linspace(0., np.pi/2, 17)[1:]:
            stamp += .1
            result = core.update(stamp, measured_surface(xyz, yaw, rng), yaw, 0., 1.)
            self.assertTrue(result['accepted'], (yaw, result))
        self.assertLess(np.linalg.norm(result['xyz']-xyz), .03)
        np.testing.assert_array_equal(core.references[0]['target'], initial_target)
        ids = [row['id'] for row in core.references]
        self.assertEqual(len(ids), len(set(ids)))
        for row in core.references:
            if row['parent'] is not None:
                self.assertLess(row['parent'], row['id'])
                self.assertGreaterEqual(np.linalg.eigvalsh(row['anchor_covariance']).min(), 0.)

    def test_flat_and_tilted_plane_do_not_observe_xy_even_with_dense_returns(self):
        x, y = np.meshgrid(np.linspace(-2., 2., 41), np.linspace(-2., 2., 41))
        for tilt in (False, True):
            with self.subTest(tilt=tilt):
                z = .12*x-.08*y+1. if tilt else np.ones_like(x)
                points = np.c_[x.ravel(), y.ravel(), z.ravel()]
                core = MetricDepthOdometry()
                for stamp in (1., 1.1, 1.2):
                    result = core.update(stamp, points, 0., 0., 0.)
                    self.assertFalse(result['accepted'], result)
                    self.assertNotIn('xyz', result)
                self.assertFalse(core.references)

    def test_noisy_planes_and_one_direction_curvature_do_not_invent_observability(self):
        x, y = np.meshgrid(np.linspace(-2., 2., 41), np.linspace(-2., 2., 41))
        for shape in ('plane', 'tilted-plane', 'cylinder'):
            for seed in (73, 144, 217):
                with self.subTest(shape=shape, seed=seed):
                    z = (.12*x-.08*y+1. if shape == 'tilted-plane' else
                         .12*np.sin(x*1.1) if shape == 'cylinder' else np.ones_like(x))
                    points = np.c_[x.ravel(), y.ravel(), z.ravel()]
                    points += np.random.default_rng(seed).normal(0., .003, points.shape)
                    core = MetricDepthOdometry()
                    result = core.update(1., points, 0., 0., 0.)
                    self.assertFalse(result['accepted'], result)
                    self.assertNotIn('xyz', result)
                    self.assertFalse(core.references)

    def test_stationary_body_gimbal_views_handover_and_return_to_initial_reference(self):
        core, rng = self.initialized(711)
        original = core.references[0]
        position = np.zeros(3)
        stamp = 1.
        max_references = 1
        for camera_view in np.r_[np.linspace(0., np.pi, 41)[1:], np.linspace(np.pi, 0., 41)[1:]]:
            stamp += .1
            points = measured_surface(position, camera_view, rng)
            # No camera angle, body motion or wheel movement is supplied to the
            # core. View novelty is inferred only from accepted measured points.
            result = core.update(stamp, points, 0., 0., 0.)
            self.assertTrue(result['accepted'], (camera_view, result))
            self.assertLess(np.linalg.norm(result['xyz']), .03)
            max_references = max(max_references, len(core.references))
        self.assertGreater(max_references, 1)
        self.assertIs(core.references[0], original)
        self.assertEqual(result['reference_id'], original['id'])
        self.assertLessEqual(len(core.references), core.max_frames)

    def test_missing_invalid_and_clock_gap_never_emit_an_accepted_pose(self):
        core, rng = self.initialized()
        for stamp, points in ((1.1, np.empty((0, 3))),
                              (1.2, np.full((101, 3), np.nan))):
            result = core.update(stamp, points, 0., 0., 0.)
            self.assertFalse(result['accepted'], result)
            self.assertNotIn('xyz', result)
        for stamp in (2.5, 2.6, 3.):
            result = core.update(stamp, measured_surface([0., 0., 0.], rng=rng), 0., 0., 0.)
            self.assertFalse(result['accepted'], result)
            self.assertEqual(result['reason'], 'depth_gap_requires_new_session')
            self.assertNotIn('xyz', result)
        self.assertEqual(len(core.references), 1)

    def test_stationary_fresh_measurements_do_not_grow_the_reference_map(self):
        core, rng = self.initialized()
        initial = core.references[0]['target'].copy()
        stamps = []
        for index in range(1, 31):
            result = core.update(1.+index*.1, measured_surface([0., 0., 0.], rng=rng), 0., 0., 0.)
            self.assertTrue(result['accepted'], result)
            stamps.append(result['stamp'])
        self.assertTrue(all(a < b for a, b in zip(stamps, stamps[1:])))
        self.assertEqual(len(core.references), 1)
        np.testing.assert_array_equal(core.references[0]['target'], initial)


class DepthProjectionTests(unittest.TestCase):
    def test_padded_big_endian_float_depth_uses_metres_and_pixel_centres(self):
        row0 = struct.pack('>fff', 1., 2., float('inf'))+b'abcd'
        row1 = struct.pack('>fff', 0., 6.01, float('nan'))+b'efgh'
        points = project_depth(row0+row1, 3, 2, 16, '32FC1', True, [2., 4., .5, .5], stride=1)
        np.testing.assert_allclose(points, [[0., 0., 1.], [1., 0., 2.]])

    def test_padded_little_endian_unsigned_depth_converts_mm_to_metres(self):
        row0 = struct.pack('<HHH', 1000, 2000, 0)+b'aa'
        row1 = struct.pack('<HHH', 500, 6001, 0)+b'bb'
        points = project_depth(row0+row1, 3, 2, 8, '16UC1', False, [2., 4., .5, .5], stride=1)
        np.testing.assert_allclose(points, [[0., 0., 1.], [1., 0., 2.], [0., .125, .5]])

    def test_stride_is_applied_to_pixels_not_padding_and_bad_buffers_reject(self):
        row = struct.pack('<ffff', 1., 2., 3., 4.)+b'PAD!'
        data = row*3
        points = project_depth(data, 4, 3, 20, '32FC1', False, [1., 1., .5, .5], stride=2)
        np.testing.assert_allclose(points, [[0., 0., 1.], [6., 0., 3.], [0., 2., 1.], [6., 6., 3.]])
        for args in ((data[:-1], 4, 3, 20, '32FC1', False, [1., 1., 0., 0.]),
                     (data, 4, 3, 12, '32FC1', False, [1., 1., 0., 0.]),
                     (data, 4, 3, 20, '32FC1', False, [0., 1., 0., 0.])):
            with self.assertRaises(ValueError):
                project_depth(*args)


class AsynchronousReferenceTests(unittest.TestCase):
    def test_blocked_worker_does_not_block_fits_and_rejection_prevents_adoption(self):
        core = MetricDepthOdometry(asynchronous_references=True)
        rng = np.random.default_rng(815)
        entered, release = threading.Event(), threading.Event()
        calls = []
        original = core._build_reference_job
        def blocked(*args):
            calls.append(args)
            entered.set()
            if not release.wait(10):
                raise TimeoutError('Synthetic blocked worker did not receive its release')
            return original(*args)
        core._build_reference_job = blocked
        try:
            self.assertTrue(core.update(1., measured_surface([0., 0., 0.], rng=rng), 0., 0., 0.)['accepted'])
            position = [.35, 0., .01]
            result = core.update(1.1, measured_surface(position, rng=rng), 0., 3.43, 0.)
            self.assertTrue(result['accepted'], result)
            self.assertTrue(entered.wait(2))
            future = core._reference_future
            durations = []
            for stamp in (1.2, 1.3, 1.4):
                begin = time.monotonic()
                result = core.update(stamp, measured_surface(position, rng=rng), 0., 0., 0.)
                durations.append(time.monotonic()-begin)
                self.assertTrue(result['accepted'], result)
                self.assertEqual(result['stamp'], stamp)
                self.assertIs(core._reference_future, future)
                self.assertEqual(len(core.references), 1)
            self.assertLess(max(durations), 1.)
            self.assertEqual(len(calls), 1)
            self.assertFalse(core.update(1.5, np.empty((0, 3)), 0., 0., 0.)['accepted'])
            release.set()
            future.result(timeout=10)
            result = core.update(1.6, measured_surface(position, rng=rng), 0., 0., 0.)
            self.assertTrue(result['accepted'], result)
            self.assertEqual(len(core.references), 1)  # Invalidated job was discarded.
            self.assertIsNone(core._reference_future)
            # A subsequent genuine motion may request a new job; completion
            # alone cannot append until a fresh accepted measurement arrives.
            result = core.update(1.7, measured_surface(position, rng=rng), 0., .01, 0.)
            self.assertTrue(result['accepted'], result)
            core._reference_future.result(timeout=10)
            self.assertEqual(len(core.references), 1)
            result = core.update(1.8, measured_surface(position, rng=rng), 0., 0., 0.)
            self.assertTrue(result['accepted'], result)
            self.assertEqual(len(core.references), 2)
            self.assertEqual(core.references[1]['parent'], core.references[0]['id'])
            self.assertGreater(core.references[1]['id'], core.references[0]['id'])
        finally:
            release.set()
            core.close()
        self.assertFalse(core.update(1.9, measured_surface([0., 0., 0.]), 0., 0., 0.)['accepted'])


if __name__ == '__main__':
    unittest.main()
