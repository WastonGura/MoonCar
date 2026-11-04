"""Focused persistent-track tests; no ROS, recorded truth, or backend solve."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from offline_rgbd_window.frontend import TrackFrontend


def texture():
    rng = np.random.default_rng(89)
    return cv2.GaussianBlur(rng.integers(0, 256, (240, 320), dtype=np.uint8), (3, 3), .6)


def sensor(image, shift=0.):
    gray = cv2.warpAffine(image, np.array([[1., 0., shift], [0., 1., 0.]]),
                          (image.shape[1], image.shape[0]), flags=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_REFLECT)
    v, u = np.indices(gray.shape)
    grid = np.stack(((u+.5-160.)*.01, (v+.5-120.)*.01,
                     np.full(gray.shape, 2.)), axis=2)
    return SimpleNamespace(gray=gray, xyz_grid=grid)


class PhotometricSamplingTests(unittest.TestCase):
    @staticmethod
    def original(image, pixels):
        offset = np.stack(np.meshgrid(np.arange(-10, 11), np.arange(-10, 11)), axis=-1).reshape(-1, 2)
        return TrackFrontend._sample(image, pixels[:, None]+offset)

    def test_standard_patches_are_bit_exact_at_fractional_and_integer_boundaries(self):
        rng = np.random.default_rng(731)
        image = rng.integers(0, 256, (240, 320), dtype=np.uint8)
        points = rng.uniform([12., 12.], [307., 227.], (96, 2)).astype(np.float32)
        boundary = np.array([[12., 12.], [32., 32.],
            [np.nextafter(np.float32(32.), np.float32(0.)), 32.5],
            [np.nextafter(np.float32(32.), np.float32(np.inf)), 32.25],
            [np.nextafter(np.float32(308.), np.float32(0.)),
             np.nextafter(np.float32(228.), np.float32(0.))]], dtype=np.float32)
        points = np.concatenate((points, boundary))
        expected = self.original(image, points)
        original_points = points.copy()
        original_image = image.copy()
        with patch.object(TrackFrontend, '_sample', side_effect=AssertionError('standard fast path')):
            actual = TrackFrontend._sample_patch(image, points)
        self.assertEqual(actual.dtype, np.float64)
        self.assertEqual(actual.shape, (len(points), 441))
        np.testing.assert_array_equal(actual, expected)
        np.testing.assert_array_equal(points, original_points)
        np.testing.assert_array_equal(image, original_image)

    def test_patch_layout_handles_strided_inputs_and_empty_standard_centers(self):
        rng = np.random.default_rng(732)
        image = rng.integers(0, 256, (480, 640), dtype=np.uint8)[::2, ::2]
        centers = rng.uniform(12., 220., (48, 4)).astype(np.float32)[:, ::2]
        for pixels in (centers, np.empty((0, 2), dtype=np.float32)):
            with self.subTest(count=len(pixels)):
                np.testing.assert_array_equal(TrackFrontend._sample_patch(image, pixels),
                                              self.original(image, pixels))

    def test_patch_batches_preserve_exact_values_across_batch_boundaries(self):
        rng = np.random.default_rng(734)
        image = rng.integers(0, 256, (480, 640), dtype=np.uint8)[::2, ::2]
        for count in (1, 63, 64, 65, 127, 128, 129, 1400):
            centers = rng.uniform([12., 12.], [307., 227.], (count, 2)).astype(np.float32)
            with self.subTest(count=count):
                np.testing.assert_array_equal(TrackFrontend._sample_patch(image, centers),
                                              self.original(image, centers))

    def test_nonstandard_dtype_and_border_keep_the_original_sampler(self):
        image = np.arange(240*320, dtype=np.uint8).reshape(240, 320)
        precise = np.array([[np.nextafter(32., 0.), np.nextafter(64., np.inf)]])
        cases = [(image, precise), (image, precise.astype(np.float16)),
                 (image, np.array([[10.25, 10.75], [308., 228.]], dtype=np.float32)),
                 (image.astype(np.float64), precise.astype(np.float32))]
        for values, pixels in cases:
            with self.subTest(dtype=pixels.dtype, image_dtype=values.dtype):
                expected = self.original(values, pixels)
                with patch.object(TrackFrontend, '_sample', wraps=TrackFrontend._sample) as fallback:
                    actual = TrackFrontend._sample_patch(values, pixels)
                self.assertEqual(fallback.call_count, 1)
                np.testing.assert_array_equal(actual, expected)
        # Nonfinite sites must reach the original validation/indexing behavior,
        # rather than being converted into a plausible finite measurement.
        invalid = np.array([[np.nan, 32.], [32., np.inf]], dtype=np.float32)
        sentinel = object()
        with patch.object(TrackFrontend, '_sample', return_value=sentinel) as fallback:
            self.assertIs(TrackFrontend._sample_patch(image, invalid), sentinel)
        self.assertEqual(fallback.call_count, 1)
        self.assertTrue(np.isnan(fallback.call_args.args[1][0, :, 0]).all())
        self.assertTrue(np.isinf(fallback.call_args.args[1][1, :, 1]).all())

    def test_photometric_costs_improvement_and_unrefined_decisions_are_bit_exact(self):
        rng = np.random.default_rng(733)
        old = rng.integers(0, 256, (240, 320), dtype=np.uint8)
        current = rng.integers(0, 256, old.shape, dtype=np.uint8)
        p0 = rng.uniform([20., 20.], [290., 210.], (96, 2)).astype(np.float32)
        q = p0+rng.uniform(-.75, .75, p0.shape).astype(np.float32)
        q[:8] = p0[:8]  # Exact ties retain the original <= and epsilon rule.
        costs = []
        for sampler in (self.original, TrackFrontend._sample_patch):
            target, before, after = sampler(old, p0), sampler(current, p0), sampler(current, q)
            before_cost = np.mean((target-before)**2, axis=1)
            after_cost = np.mean((target-after)**2, axis=1)
            improves = after_cost <= before_cost+1e-12
            unrefined = ~improves & (np.linalg.norm(q-p0, axis=1) <= .5)
            costs.append((before_cost, after_cost, improves, unrefined))
        for actual, expected in zip(costs[1], costs[0]):
            np.testing.assert_array_equal(actual, expected)
        self.assertTrue(costs[1][2][:8].all())
        self.assertTrue((~costs[1][2]).any())


class PersistentTrackTests(unittest.TestCase):
    def test_recent_measured_reference_recovers_only_verified_map_ids_with_new_depth(self):
        frontend=TrackFrontend(track_budget=576)
        initial=sensor(self.image);initial.stamp_ns=100_000_000
        first=frontend.process(initial);selected,_=frontend.select(first);frontend.commit(first,selected)
        active=set(map(int,first['track_ids'][selected]));before={int(t):p.copy() for t,p in zip(first['track_ids'],first['points'])}
        current=sensor(self.image);current.stamp_ns=200_000_000;current.xyz_grid[:,:,0]+=.005
        original=frontend._track;calls=[]
        def once(*args):
            calls.append(True)
            return None if len(calls)==1 else original(*args)
        with patch.object(frontend,'_track',side_effect=once):result=frontend.process(current)
        self.assertTrue(frontend.last_tracking['recovery_used'])
        self.assertTrue(set(map(int,result['track_ids'][:frontend.last_tracking['surviving']])).issubset(active))
        self.assertTrue(frontend.select(result)[1]['adequate'])
        for t,p in zip(result['track_ids'],result['points']):
            if int(t) in active:np.testing.assert_allclose(p,before[int(t)]+[.005,0,0],atol=1e-7)

    def test_reference_recovery_cannot_override_bad_roundtrip_or_expired_reference(self):
        for stamp in (200_000_000,1_200_000_000):
            with self.subTest(stamp=stamp):
                frontend=TrackFrontend(track_budget=576)
                initial=sensor(self.image);initial.stamp_ns=100_000_000
                first=frontend.process(initial);selected,_=frontend.select(first);frontend.commit(first,selected)
                changed=sensor(self.image,1.);changed.stamp_ns=stamp
                def bad_flow(left,right,points,initial,**kwargs):
                    result=initial.copy();result[:,0,0]+=1.
                    return result,np.ones((len(points),1),np.uint8),np.zeros((len(points),1),np.float32)
                with patch('cv2.calcOpticalFlowPyrLK',side_effect=bad_flow):result=frontend.process(changed)
                self.assertFalse(frontend.last_tracking.get('recovery_used',False))
                self.assertFalse(frontend.select(result)[1]['adequate'])
                if stamp==1_200_000_000:self.assertNotIn('recovery_attempts',frontend.last_tracking)

    def test_recovery_reference_memory_is_bounded_to_two_measured_frames(self):
        frontend=TrackFrontend(track_budget=576)
        for i in range(1,5):
            item=sensor(self.image);item.stamp_ns=i*100_000_000
            features=frontend.process(item);selected,_=frontend.select(features);frontend.commit(features,selected)
        self.assertEqual(len(frontend._recovery_frames),2)
        self.assertEqual([f['_exposure_stamp_ns'] for f in frontend._recovery_frames],[300_000_000,400_000_000])

    def test_spatial_failure_retains_verified_ids_but_forbids_pose_until_recovery(self):
        frontend,first,active=self.initialized()
        with patch.object(frontend.measurements,'_spatial_support',return_value=(False,{})):
            weak=frontend.process(sensor(self.image))
        self.assertIsNotNone(weak)
        self.assertTrue(active.issubset(set(map(int,weak['track_ids']))))
        self.assertFalse(weak['tracking_pose_adequate'])
        self.assertFalse(frontend.select(weak)[1]['adequate'])
        restored=frontend.process(sensor(self.image))
        self.assertTrue(restored['tracking_pose_adequate'])
        self.assertTrue(frontend.select(restored)[1]['adequate'])
        self.assertGreaterEqual(frontend.select(restored)[1]['connected'],24)

    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(2)
        cls.image = texture()

    def initialized(self):
        frontend = TrackFrontend()
        first = frontend.process(sensor(self.image))
        selected, diagnostic = frontend.select(first)
        self.assertTrue(diagnostic['adequate'])
        frontend.commit(first, selected)
        return frontend, first, set(map(int, first['track_ids'][selected]))

    def test_identical_rgb_avoids_patch_sampling_but_uses_fresh_depth(self):
        frontend, first, _ = self.initialized()
        current = sensor(self.image)
        current.xyz_grid[:, :, 0] += .005
        with patch.object(frontend, '_sample_patch', side_effect=AssertionError('duplicate patch')):
            result = frontend.process(current)
        self.assertTrue(frontend.last_tracking['identical_image_zero_flow'])
        self.assertGreater(frontend.last_tracking['surviving'], 24)
        old = {int(t): p for t, p in zip(first['track_ids'], first['points'])}
        for track, point in zip(result['track_ids'], result['points']):
            if int(track) in old:
                np.testing.assert_allclose(point, old[int(track)]+[.005, 0., 0.], atol=1e-7, rtol=0)

    def test_identical_rgb_cannot_reuse_old_depth_when_current_depth_is_invalid(self):
        frontend, _, _ = self.initialized()
        current = sensor(self.image)
        current.xyz_grid[:] = np.nan
        with patch.object(frontend, '_sample_patch', side_effect=AssertionError('duplicate patch')):
            self.assertIsNone(frontend.process(current))
        self.assertTrue(frontend.last_tracking['identical_image_zero_flow'])
        self.assertEqual(frontend.last_tracking['depth_valid'], 0)

    def test_changed_rgb_keeps_photometric_checks(self):
        frontend, _, _ = self.initialized()
        with patch.object(frontend, '_sample_patch', wraps=frontend._sample_patch) as sampled:
            frontend.process(sensor(self.image, .5))
        self.assertFalse(frontend.last_tracking['identical_image_zero_flow'])
        self.assertEqual(sampled.call_count, 3)

    def test_real_lk_preserves_submitted_identity_without_orb_redetection(self):
        frontend, first, active = self.initialized()
        next_track = frontend.next_track
        # New detections are unavailable; all surviving identities must come
        # from real image tracking, not descriptor reassignment.
        with patch.object(frontend.measurements, '_features', return_value=None):
            for shift in (1.5, 3., 4.5):
                current = frontend.process(sensor(self.image, shift))
                self.assertIsNotNone(current)
                selected, diagnostic = frontend.select(current)
                self.assertTrue(diagnostic['adequate'])
                self.assertGreaterEqual(diagnostic['connected'], 24)
                self.assertGreaterEqual(len(active & set(map(int, current['track_ids']))), 24)
                self.assertEqual(frontend.last_tracking['new_tracks'], 0)
                self.assertFalse(current['descriptor_valid'].any())
        self.assertEqual(frontend.next_track, next_track)

    def test_identical_real_image_uses_exact_zero_flow_and_rechecks_new_depth(self):
        frontend, _, active = self.initialized()
        with patch.object(frontend.measurements, '_features', return_value=None), \
                patch('cv2.calcOpticalFlowPyrLK', side_effect=AssertionError('no iterative drift for exact image')):
            for _ in range(3):
                current = frontend.process(sensor(self.image))
                self.assertTrue(active.issubset(set(map(int, current['track_ids']))))
                self.assertTrue(frontend.last_tracking['identical_image_zero_flow'])
                self.assertTrue(frontend.select(current)[1]['adequate'])
            invalid = sensor(self.image)
            invalid.xyz_grid[:] = np.nan
            self.assertIsNone(frontend.process(invalid))

    def test_nonimproving_tiny_refinement_retains_real_unrefined_measurement(self):
        frontend, _, active = self.initialized()
        changed = self.image.copy()
        changed[0, 0] = 255-int(changed[0, 0])
        old_pixels = {int(t): p.copy() for t, p in
                      zip(frontend.previous[0]['track_ids'], frontend.previous[0]['pixels'])}
        calls = []
        def tiny_flow(left, right, points, initial, **kwargs):
            answer = initial.copy()
            if not calls:
                answer[:, 0, 0] += .001
            calls.append(True)
            return answer, np.ones((len(points), 1), np.uint8), np.zeros((len(points), 1), np.float32)
        with patch.object(frontend.measurements, '_features', return_value=None), \
                patch('cv2.calcOpticalFlowPyrLK', side_effect=tiny_flow):
            current = frontend.process(sensor(changed))
        self.assertEqual(len(calls), 2)
        self.assertFalse(frontend.last_tracking['identical_image_zero_flow'])
        self.assertGreaterEqual(frontend.last_tracking['photometric_unrefined'], 24)
        self.assertTrue(frontend.select(current)[1]['adequate'])
        self.assertGreaterEqual(len(active & set(map(int, current['track_ids']))), 24)
        unchanged = sum(np.array_equal(p, old_pixels[int(t)]) for t, p in
                        zip(current['track_ids'], current['pixels']))
        self.assertGreaterEqual(unchanged, 24)
        # Retaining an image location does not retain its old depth sample.
        bad = sensor(changed)
        bad.xyz_grid[:] = np.nan
        with patch.object(frontend.measurements, '_features', return_value=None):
            self.assertIsNone(frontend.process(bad))

    def test_nonimproving_flow_outside_original_pixel_budget_is_still_rejected(self):
        frontend, _, _ = self.initialized()
        changed = self.image.copy()
        changed[0, 0] = 255-int(changed[0, 0])
        calls = []
        def large_flow(left, right, points, initial, **kwargs):
            answer = initial.copy()
            if not calls:
                answer[:, 0, 0] += 1.
            calls.append(True)
            return answer, np.ones((len(points), 1), np.uint8), np.zeros((len(points), 1), np.float32)
        with patch.object(frontend.measurements, '_features', return_value=None), \
                patch('cv2.calcOpticalFlowPyrLK', side_effect=large_flow):
            self.assertIsNone(frontend.process(sensor(changed)))
        self.assertEqual(frontend.last_tracking['photometric_unrefined'], 0)

    def test_search_projection_handles_rotation_lever_half_pixel_and_invalid_points(self):
        frontend = TrackFrontend()
        R = Rotation.from_euler('xyz', [.03, .08, -.02]).as_matrix()
        K = np.array([[200., 0., 160.], [0., 200., 120.], [0., 0., 1.]])
        lever = np.array([.12, -.03, .2])
        optical = np.array([[.2, -.1, 2.], [0., 0., -1.], [8., 0., 1.]])
        points = optical @ R.T+lever
        p0 = np.array([[20., 40.], [30., 50.], [40., 60.]], dtype=np.float32)
        seed = frontend._search_seed(dict(points=points), p0,
            dict(K=K, rotation=R, translation=lever), (240, 320))
        np.testing.assert_allclose(seed[0, 0], [179.5, 109.5], atol=1e-5)
        np.testing.assert_array_equal(seed[1:, 0], p0[1:])
        self.assertEqual(seed.dtype, np.float32)
        self.assertTrue(seed.flags.c_contiguous)
        np.testing.assert_array_equal(frontend._search_seed(dict(points=points), p0, None,
                                                            (240, 320))[:, 0], p0)

    def rotated_plane(self):
        R = Rotation.from_euler('y', 8., degrees=True).as_matrix()
        lever = np.array([.08, .01, .02])
        K = np.array([[200., 0., 160.], [0., 200., 120.], [0., 0., 1.]])
        v, u = np.indices(self.image.shape)
        rays = np.stack(((u+.5-160.)/200., (v+.5-120.)/200., np.ones(u.shape)), axis=2) @ R.T
        depth = (2.-lever[2])/rays[:, :, 2]
        world = lever+depth[:, :, None]*rays
        mapx = (world[:, :, 0]/2.*200.+160.-.5).astype(np.float32)
        mapy = (world[:, :, 1]/2.*200.+120.-.5).astype(np.float32)
        gray = cv2.remap(self.image, mapx, mapy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        world[(mapx < 0.) | (mapx+1 >= 320) | (mapy < 0.) | (mapy+1 >= 240)] = np.nan
        return SimpleNamespace(gray=gray, xyz_grid=world, stamp_ns=200000000,
            camera_model=dict(K=K, rotation=R, translation=lever))

    def test_real_rotating_camera_and_lever_track_from_measured_projection_seed(self):
        frontend, _, active = self.initialized()
        current_sensor = self.rotated_plane()
        with patch.object(frontend.measurements, '_features', return_value=None):
            current = frontend.process(current_sensor)
        self.assertIsNotNone(current)
        self.assertGreaterEqual(frontend.last_tracking['camera_projection_seed_count'], 24)
        self.assertTrue(frontend.select(current)[1]['adequate'])
        self.assertGreaterEqual(len(active & set(map(int, current['track_ids']))), 24)
        stored = current['_search_camera_model']['translation'].copy()
        current_sensor.camera_model['translation'][:] = 99.
        np.testing.assert_array_equal(current['_search_camera_model']['translation'], stored)

    def test_valid_camera_projection_cannot_override_failed_real_image_roundtrip(self):
        frontend, _, _ = self.initialized()
        current_sensor = self.rotated_plane()
        calls = []
        def failed_roundtrip(left, right, points, initial, **kwargs):
            answer = initial.copy()
            if calls:
                answer[:, 0, 0] += 1.
            calls.append(True)
            return answer, np.ones((len(points), 1), np.uint8), np.zeros((len(points), 1), np.float32)
        with patch.object(frontend.measurements, '_features', return_value=None), \
                patch('cv2.calcOpticalFlowPyrLK', side_effect=failed_roundtrip):
            self.assertIsNone(frontend.process(current_sensor))
        self.assertGreaterEqual(frontend.last_tracking['camera_projection_seed_count'], 24)
        self.assertEqual(frontend.last_tracking['forward_backward_valid'], 0)

    def test_forward_backward_above_half_pixel_never_preserves_ids(self):
        frontend, _, _ = self.initialized()
        calls = []
        def bad_flow(left, right, points, initial, **kwargs):
            calls.append(True)
            answer = initial.copy()
            answer[:, 0, 0] += 1.
            return answer, np.ones((len(points), 1), np.uint8), np.zeros((len(points), 1), np.float32)
        with patch.object(frontend.measurements, '_features', return_value=None), \
                patch('cv2.calcOpticalFlowPyrLK', side_effect=bad_flow):
            self.assertIsNone(frontend.process(sensor(self.image, 1.)))
        self.assertEqual(len(calls), 2)
        self.assertEqual(frontend.last_tracking['forward_backward_valid'], 0)

    def test_depth_unknown_discontinuity_and_bilinear_covariance(self):
        frontend = TrackFrontend()
        grid = sensor(self.image).xyz_grid.copy()
        pixels = np.array([[80.25, 70.75]])
        which, points, C = frontend._sample_tracks(grid, pixels, np.array([2.]))
        self.assertEqual(which.tolist(), [0])
        np.testing.assert_allclose(points[0], ((80.75-160.)*.01, (71.25-120.)*.01, 2.))
        np.testing.assert_allclose(C[0], np.diag([.003**2+.5**2*4*.01**2,
                                                 .003**2+.5**2*4*.01**2, .003**2]))
        grid[70, 80] = np.nan
        self.assertEqual(len(frontend._sample_tracks(grid, pixels, np.ones(1))[0]), 0)
        grid = sensor(self.image).xyz_grid.copy()
        grid[70, 80, 2] += .151
        self.assertEqual(len(frontend._sample_tracks(grid, pixels, np.ones(1))[0]), 0)
        self.assertEqual(len(frontend._sample_tracks(grid, np.array([[.5, .5]]), np.ones(1))[0]), 0)

    def test_orb_only_adds_spaced_new_rows_and_track_budget_is_bounded(self):
        frontend, _, active = self.initialized()
        current = frontend.process(sensor(self.image, 2.))
        self.assertGreaterEqual(frontend.last_tracking['surviving'], 24)
        ids = current['track_ids']
        self.assertEqual(len(ids), len(set(map(int, ids))))
        self.assertLessEqual(len(ids), frontend.measurements._orb.getMaxFeatures())
        points = current['pixels']
        distances = np.sum((points[:, None]-points[None])**2, axis=2)
        np.fill_diagonal(distances, np.inf)
        self.assertGreaterEqual(float(distances.min()), frontend.MIN_TRACK_SPACING_PX**2-1e-6)
        self.assertFalse(current['descriptor_valid'][:frontend.last_tracking['surviving']].any())
        selected, diagnostic = frontend.select(current)
        self.assertLessEqual(len(selected), 96)
        self.assertTrue(diagnostic['adequate'])
        self.assertGreaterEqual(len(active & set(map(int, ids[selected]))), 24)

    def test_original_three_dimensional_consensus_rejects_split_modes(self):
        frontend = TrackFrontend()
        rng = np.random.default_rng(11)
        previous = dict(points=rng.uniform(-1., 1., (72, 3)),
                        covariance=np.tile(np.eye(3)*.003**2, (72, 1, 1)))
        current = dict(points=previous['points'].copy(), covariance=previous['covariance'].copy())
        current['points'][:, 0] += np.repeat([0., .2, .4], 24)
        pairs = np.column_stack((np.arange(72), np.arange(72)))
        self.assertEqual(len(frontend._consensus(current, previous, pairs)), 0)

    def test_twenty_three_shared_ids_do_not_pass_twenty_four_gate(self):
        frontend, first, _ = self.initialized()
        changed = dict(first)
        ids = np.arange(100000, 100000+len(first['track_ids']), dtype=np.int64)
        chosen = frontend.keyframes[-1][0]['submitted_ids'][:23]
        for at, track in enumerate(chosen):
            ids[at] = track
        changed['track_ids'] = ids
        _, diagnostic = frontend.select(changed)
        self.assertLess(diagnostic['connected'], 24)
        self.assertFalse(diagnostic['adequate'])


if __name__ == '__main__':
    unittest.main()
