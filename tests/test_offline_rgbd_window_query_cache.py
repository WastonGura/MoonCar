"""Independent cached-query MAP equivalence; no ROS, truth archive, or timing benchmark."""
from pathlib import Path
import pickle
import sys
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from offline_rgbd_window.core import Frame, Observation, WindowEstimator, WindowError
from offline_rgbd_window.query_cache import (freeze_query_cache, query_cached,
    _batch_residuals, _checked_query)


def scene(count=96):
    rng = np.random.default_rng(2193)
    return np.column_stack((rng.uniform(-1., 1., count), rng.uniform(-.7, .7, count),
                            rng.uniform(2., 4., count)))


def frame(number, points, ids=None, position=None, dynamic=False, attitude_sigma=.001,
          point_sigma=.004, noise=0.):
    ids = list(range(len(points))) if ids is None else list(ids)
    position = np.array((.02*number, -.005*number, .002*number)) if position is None else np.asarray(position)
    body = Rotation.from_euler('xyz', [.003*number, -.002*number, .01*number]).as_matrix() if dynamic else np.eye(3)
    camera = Rotation.from_euler('yz', [.03*number, -.02*number]).as_matrix() if dynamic else np.eye(3)
    lever = np.array((.15, .02, .35)) if dynamic else np.zeros(3)
    measured = ((points[ids]-position)@body-lever)@camera
    rng = np.random.default_rng(number+1983)
    measured += rng.normal(0., noise, measured.shape)
    return Frame(number*100_000_000, body, camera, lever,
        [Observation(i, z, np.diag([1., 1.3, 1.7])*point_sigma**2) for i, z in zip(ids, measured)],
        np.eye(3)*attitude_sigma**2)


class QueryCacheTests(unittest.TestCase):
    def test_commit_handoff_reuses_checked_lu_through_full_window_and_churn(self):
        points = scene(36)
        estimator, reference = WindowEstimator(3), WindowEstimator(3)
        for number in range(1, 8):
            ids = range(24) if number % 2 else range(6, 30)
            measured = frame(number, points, ids, dynamic=True, noise=.0001)
            expected = reference.add_frame(measured)
            answer, posterior = estimator.add_frame_with_posterior(measured)
            np.testing.assert_array_equal(answer.position, expected.position)
            np.testing.assert_array_equal(answer.covariance, expected.covariance)
            self.assertEqual(answer.diagnostics, expected.diagnostics)
            self.assertEqual(pickle.dumps(estimator.__dict__), pickle.dumps(reference.__dict__))
            persistent = pickle.dumps(estimator.__dict__)
            decomposition = posterior._decomposition
            with patch.object(estimator, '_assemble', side_effect=AssertionError('duplicate assembly')), \
                    patch('offline_rgbd_window.query_cache.splu',
                          side_effect=AssertionError('duplicate LU')):
                cache = freeze_query_cache(estimator, number, posterior=posterior)
            self.assertIs(cache._decomposition, decomposition)
            self.assertEqual(pickle.dumps(estimator.__dict__), persistent)
            ordinary = freeze_query_cache(reference, number)
            np.testing.assert_array_equal(cache._information.toarray(), ordinary._information.toarray())
            np.testing.assert_array_equal(cache._eta, ordinary._eta)
            np.testing.assert_array_equal(cache._mean, ordinary._mean)
            query = frame(number+1, points, range(3, 27), dynamic=True, noise=.0001)
            a, b = query_cached(query, cache, number), query_cached(query, ordinary, number)
            np.testing.assert_array_equal(a.position, b.position)
            np.testing.assert_array_equal(a.covariance, b.covariance)

    def test_commit_handoff_owner_expiry_mutation_and_one_use_are_checked(self):
        points = scene(12)
        estimator = WindowEstimator(3)
        _, posterior = estimator.add_frame_with_posterior(frame(1, points))
        other = WindowEstimator(3)
        other.add_frame(frame(1, points))
        with self.assertRaisesRegex(WindowError, 'owner mismatch'):
            freeze_query_cache(other, 1, posterior=posterior)
        freeze_query_cache(estimator, 1, posterior=posterior)
        with self.assertRaisesRegex(WindowError, 'already consumed'):
            freeze_query_cache(estimator, 1, posterior=posterior)
        for mutation in ('next_commit', 'stamp', 'count', 'configuration', 'measurement', 'prior'):
            with self.subTest(mutation=mutation):
                estimator = WindowEstimator(3)
                _, posterior = estimator.add_frame_with_posterior(frame(1, points))
                if mutation == 'next_commit':
                    estimator.add_frame(frame(2, points))
                elif mutation == 'stamp':
                    estimator._last_stamp += 1
                elif mutation == 'count':
                    estimator._total_frames += 1
                elif mutation == 'configuration':
                    estimator.shared_calibration_sigma_m *= 2
                elif mutation == 'measurement':
                    estimator._frames[0].observations[0].point_camera[0] += .01
                else:
                    estimator._prior_eta = np.ones(3)
                with patch.object(estimator, '_assemble', side_effect=AssertionError('invalid fallback')), \
                        patch('offline_rgbd_window.query_cache.splu',
                              side_effect=AssertionError('invalid fallback')):
                    with self.assertRaisesRegex(WindowError, 'stale or mutated'):
                        freeze_query_cache(estimator, 1, posterior=posterior)
        with self.assertRaisesRegex(WindowError, 'expected a committed posterior'):
            freeze_query_cache(other, 1, posterior=object())

    def test_failed_explicit_commit_exports_no_posterior_and_rolls_back(self):
        points, estimator = scene(12), WindowEstimator(2)
        estimator.add_frame(frame(1, points))
        persistent = pickle.dumps(estimator.__dict__)
        with patch('offline_rgbd_window.core.splu', side_effect=RuntimeError('failed LU')):
            with self.assertRaisesRegex(WindowError, 'factorization failed'):
                estimator.add_frame_with_posterior(frame(2, points))
        self.assertEqual(pickle.dumps(estimator.__dict__), persistent)
        with patch.object(estimator, '_posterior_signature', side_effect=WindowError('publish failed')):
            with self.assertRaisesRegex(WindowError, 'publish failed'):
                estimator.add_frame_with_posterior(frame(2, points))
        self.assertEqual(pickle.dumps(estimator.__dict__), persistent)
        answer, posterior = estimator.add_frame_with_posterior(frame(3, points))
        self.assertEqual(answer.diagnostics['total_frames'], 2)
        self.assertIsNotNone(freeze_query_cache(estimator, 2, posterior=posterior))
        pickle.dumps(estimator.__dict__)

    def test_handoff_cache_owns_arrays_and_survives_later_source_changes(self):
        points, estimator = scene(24), WindowEstimator(3)
        for number in range(1, 6):
            _, posterior = estimator.add_frame_with_posterior(frame(number, points, dynamic=True))
        cache = freeze_query_cache(estimator, 5, posterior=posterior)
        query = frame(6, points, dynamic=True)
        expected = query_cached(query, cache, 5)
        for current in estimator._frames:
            current.nominal_R_wb[:] = 0.
            for observation in current.observations:
                observation.point_camera[:] = np.nan
                observation.covariance_camera[:] = np.nan
        estimator._prior_H[:] = 0.
        estimator._prior_eta[:] = 0.
        for _, compiled in estimator._frame_terms.values():
            for blocks, constant, _ in compiled[-1]:
                constant[:] = np.nan
                for matrix in blocks.values():
                    matrix[:] = np.nan
        estimator._solution.clear()
        answer = query_cached(query, cache, 5)
        np.testing.assert_array_equal(answer.position, expected.position)
        np.testing.assert_array_equal(answer.covariance, expected.covariance)
        self.assertFalse(cache._information.data.flags.writeable)
        self.assertFalse(cache._eta.flags.writeable)
        self.assertFalse(cache._mean.flags.writeable)

    def equivalent(self, estimator, cache, query):
        original = pickle.dumps(estimator.__dict__)
        expected = estimator.query_frame(query)
        answer = query_cached(query, cache, cache.version)
        np.testing.assert_allclose(answer.position, expected.position, atol=2e-8, rtol=1e-7)
        np.testing.assert_allclose(answer.covariance, expected.covariance, atol=2e-11, rtol=1e-6)
        self.assertAlmostEqual(answer.diagnostics['maximum_attitude_correction_rad'],
                               expected.diagnostics['maximum_attitude_correction_rad'], delta=2e-8)
        self.assertAlmostEqual(answer.diagnostics['metric_residual_rms_m'],
                               expected.diagnostics['metric_residual_rms_m'], delta=2e-8)
        self.assertEqual(pickle.dumps(estimator.__dict__), original)
        return answer

    def test_partial_window_dynamic_gimbal_and_lever_matches_joint_reference(self):
        points = scene(40)
        estimator = WindowEstimator(6)
        for i in range(1, 4):
            estimator.add_frame(frame(i, points, range(24), dynamic=True, noise=.0002))
        cache = freeze_query_cache(estimator, 3)
        answer = self.equivalent(estimator, cache,
            frame(4, points, range(36), dynamic=True, noise=.0002))
        self.assertEqual(answer.diagnostics['observed_existing_landmarks'], 24)
        self.assertEqual(answer.diagnostics['observed_new_landmarks'], 12)
        self.assertEqual(answer.diagnostics['frame_count'], 4)

    def test_full_window_oldest_only_tracks_and_new_landmarks_match_temporary_schur(self):
        points = scene(42)
        estimator = WindowEstimator(6)
        estimator.add_frame(frame(1, points, range(30), dynamic=True))
        for i in range(2, 7):
            estimator.add_frame(frame(i, points, range(6, 30), dynamic=True, noise=.0002))
        cache = freeze_query_cache(estimator, 6)
        answer = self.equivalent(estimator, cache,
            frame(7, points, list(range(18))+list(range(30, 42)), dynamic=True, noise=.0002))
        self.assertEqual(answer.diagnostics['frame_count'], 6)
        self.assertEqual(cache.keyframe_stamps_ns[0], 100_000_000)
        self.assertEqual(estimator._marginalized_frames, 0)

    def test_prior_cross_blocks_after_landmark_churn_are_retained(self):
        points = scene(96)
        estimator = WindowEstimator(3)
        for i in range(1, 9):
            ids = list(range(12))+list(range(12+6*(i-1), 18+6*(i-1)))
            estimator.add_frame(frame(i, points, ids, dynamic=True, noise=.0003))
        cache = freeze_query_cache(estimator, 8)
        self.equivalent(estimator, cache, frame(9, points,
            list(range(12))+list(range(60, 66)), dynamic=True, noise=.0003))
        columns = np.column_stack((cache._columns[0], cache._columns[1]))
        at = cache._offsets[('l', 0)]
        self.assertGreater(np.linalg.norm(columns[at:at+3, 3:6]), 0.)
        self.assertGreater(np.linalg.norm(estimator._prior_H), 0.)

    def test_new_single_exposure_landmarks_supply_no_extra_pose_information(self):
        points = scene(48)
        estimator = WindowEstimator(6)
        estimator.add_frame(frame(1, points, range(18)))
        estimator.add_frame(frame(2, points, range(18)))
        cache = freeze_query_cache(estimator, 2)
        first = self.equivalent(estimator, cache, frame(3, points, range(18)))
        second = self.equivalent(estimator, cache, frame(3, points, range(48)))
        np.testing.assert_allclose(first.position, second.position, atol=2e-8, rtol=1e-7)
        np.testing.assert_allclose(first.covariance, second.covariance, atol=2e-11, rtol=1e-6)

    def test_snapshot_ownership_version_and_query_do_not_pollute_next_commit(self):
        points = scene(24)
        estimator = WindowEstimator(3)
        estimator.add_frame(frame(1, points))
        estimator.add_frame(frame(2, points))
        before = pickle.dumps(estimator.__dict__)
        cache = freeze_query_cache(estimator, 2)
        self.assertEqual(pickle.dumps(estimator.__dict__), before)
        with self.assertRaises(ValueError):
            cache._mean[0] = 123.
        self.equivalent(estimator, cache, frame(3, points))
        self.equivalent(estimator, cache, frame(3, points))
        expected = estimator.query_frame(frame(3, points))
        committed = estimator.add_frame(frame(3, points))
        np.testing.assert_array_equal(committed.position, expected.position)
        self.assertEqual(estimator._total_frames, 3)
        with self.assertRaisesRegex(WindowError, 'version mismatch'):
            query_cached(frame(4, points), cache, 3)
        newer = freeze_query_cache(estimator, 3)
        self.assertNotEqual(newer.keyframe_stamps_ns, cache.keyframe_stamps_ns)
        self.equivalent(estimator, newer, frame(4, points))

    def test_memoized_columns_only_grow_for_new_existing_ids_and_memory_is_reported(self):
        points = scene(36)
        estimator = WindowEstimator()
        estimator.add_frame(frame(1, points, range(30)))
        cache = freeze_query_cache(estimator, 1)
        before = cache.estimated_bytes()
        a = query_cached(frame(2, points, range(12)), cache, 1)
        self.assertEqual(a.diagnostics['new_columns'], 36)
        self.assertEqual(cache.cached_columns, 36)
        self.assertGreater(cache.estimated_bytes(), before)
        b = query_cached(frame(2, points, range(12)), cache, 1)
        self.assertEqual(b.diagnostics['new_columns'], 0)
        self.assertEqual(b.diagnostics['cache_bytes'], cache.estimated_bytes())
        c = query_cached(frame(2, points, range(6, 24)), cache, 1)
        self.assertEqual(c.diagnostics['new_columns'], 36)
        self.assertEqual(cache.cached_columns, 72)

    def test_invalid_frame_and_angle_failure_do_not_commit_pending_columns(self):
        points = scene(48)
        estimator = WindowEstimator(3)
        estimator.add_frame(frame(1, points, attitude_sigma=.00001))
        cache = freeze_query_cache(estimator, 1)
        bad = frame(2, points)
        bad = Frame(bad.stamp_ns, bad.nominal_R_wb, bad.R_bc, bad.t_bc,
            [Observation(o.track_id+1000, o.point_camera, o.covariance_camera) for o in bad.observations], bad.attitude_cov)
        for query in (bad, frame(1, points)):
            with self.assertRaises(WindowError):
                estimator.query_frame(query)
            with self.assertRaises(WindowError):
                query_cached(query, cache, 1)
            self.assertEqual(cache.cached_columns, 0)
        actual = Rotation.from_euler('z', 8., degrees=True).as_matrix()
        rotated = (points-np.array((.04, -.01, .004)))@actual
        query = Frame(200_000_000, np.eye(3), np.eye(3), np.zeros(3),
            [Observation(i, z, np.eye(3)*.004**2) for i, z in enumerate(rotated)], np.eye(3)*.1**2)
        with self.assertRaisesRegex(WindowError, '3 degree'):
            estimator.query_frame(query)
        with self.assertRaisesRegex(WindowError, '3 degree'):
            query_cached(query, cache, 1)
        self.assertEqual(cache.cached_columns, 0)
        self.assertEqual(cache.version, 1)

    def test_retained_history_angles_are_updated_not_just_current_attitude(self):
        points = scene(48)
        estimator = WindowEstimator(6)
        estimator.add_frame(frame(1, points, attitude_sigma=.1, position=np.zeros(3)))
        cache = freeze_query_cache(estimator, 1)
        actual = Rotation.from_euler('z', 8., degrees=True).as_matrix()
        query = Frame(200_000_000, np.eye(3), np.eye(3), np.zeros(3),
            [Observation(i, z, np.eye(3)*.004**2) for i, z in enumerate(points@actual)], np.eye(3)*.00001**2)
        with self.assertRaisesRegex(WindowError, '3 degree'):
            estimator.query_frame(query)
        with self.assertRaisesRegex(WindowError, '3 degree'):
            query_cached(query, cache, 1)
        self.assertEqual(cache.cached_columns, 0)

    def test_query_never_calls_legacy_solve_and_conditioning_basis_is_explicit(self):
        points = scene(18)
        estimator = WindowEstimator()
        estimator.add_frame(frame(1, points))
        cache = freeze_query_cache(estimator, 1)
        with patch.object(WindowEstimator, 'query_frame', side_effect=AssertionError('no hidden legacy fallback')), \
                patch.object(WindowEstimator, 'add_frame', side_effect=AssertionError('no hidden commit')), \
                patch.object(WindowEstimator, '_assemble', side_effect=AssertionError('no whole-window assembly')):
            result = query_cached(frame(2, points), cache, 1)
        self.assertEqual(result.diagnostics['query_dimension'], 9)
        self.assertIsNone(result.diagnostics['legacy_query_pivot_ratio'])
        self.assertFalse(result.diagnostics['legacy_query_conditioning_checked'])
        self.assertEqual(result.diagnostics['old_window_pivot_ratio'], cache.old_window_pivot_ratio)
        self.assertGreater(result.diagnostics['query_information_eigenvalue_ratio'], 0.)

    def test_empty_cache_invalid_version_and_mutated_source_are_not_silently_reused(self):
        with self.assertRaises(WindowError):
            freeze_query_cache(WindowEstimator(), 0)
        estimator = WindowEstimator()
        points = scene(18)
        estimator.add_frame(frame(1, points))
        for version in (True, -1, 1.5, '1'):
            with self.assertRaises(WindowError):
                freeze_query_cache(estimator, version)
        cache = freeze_query_cache(estimator, 1)
        original = query_cached(frame(2, points), cache, 1)
        estimator._frames[0].observations[0].point_camera[:] = np.nan
        result = query_cached(frame(2, points), cache, 1)
        np.testing.assert_array_equal(result.position, original.position)
        np.testing.assert_array_equal(result.covariance, original.covariance)

    def test_freeze_does_not_deepcopy_and_only_fills_isolated_factor_cache(self):
        points = scene(24)
        estimator = WindowEstimator(3)
        for i in range(1, 5):
            estimator.add_frame(frame(i, points, dynamic=True))
        # Force one real compilation during freeze; it may fill only the
        # temporary dictionary, without touching the committed estimator.
        del estimator._frame_terms[estimator._frames[-1].stamp_ns]
        before = pickle.dumps(estimator.__dict__)
        with patch('offline_rgbd_window.query_cache.copy.deepcopy',
                   side_effect=AssertionError('whole-estimator deepcopy forbidden')):
            cache = freeze_query_cache(estimator, 4)
        self.assertEqual(pickle.dumps(estimator.__dict__), before)
        self.equivalent(estimator, cache, frame(5, points, dynamic=True))

    def test_frozen_posterior_owns_prior_factors_and_measurements_after_source_mutation(self):
        points = scene(30)
        estimator = WindowEstimator(3)
        for i in range(1, 6):
            estimator.add_frame(frame(i, points, dynamic=True, noise=.0002))
        self.assertGreater(estimator._prior_H.size, 0)
        before = pickle.dumps(estimator.__dict__)
        cache = freeze_query_cache(estimator, 5)
        self.assertEqual(pickle.dumps(estimator.__dict__), before)
        query = frame(6, points, dynamic=True, noise=.0002)
        expected = query_cached(query, cache, 5)
        estimator._prior_H[:] = np.nan
        estimator._prior_eta[:] = np.nan
        estimator._prior_keys.clear()
        for old, compiled in estimator._frame_terms.values():
            for blocks, constant, weight in compiled[-1]:
                for value in blocks.values():
                    value[:] = np.nan
                constant[:] = np.nan
                weight[:] = np.nan
            old.nominal_R_wb[:] = np.nan
            old.R_bc[:] = np.nan
            old.t_bc[:] = np.nan
            old.attitude_cov[:] = np.nan
            for observation in old.observations:
                observation.point_camera[:] = np.nan
                observation.covariance_camera[:] = np.nan
        for value in estimator._solution.values():
            value[:] = np.nan
        estimator._frames.clear()
        estimator._frame_terms.clear()
        actual = query_cached(query, cache, 5)
        np.testing.assert_array_equal(actual.position, expected.position)
        np.testing.assert_array_equal(actual.covariance, expected.covariance)
        self.assertEqual(actual.diagnostics['metric_residual_rms_m'],
                         expected.diagnostics['metric_residual_rms_m'])

    def test_standard_batch_validation_matches_reference_and_owns_observations(self):
        points = scene(24)
        estimator = WindowEstimator()
        estimator.add_frame(frame(1, points, dynamic=True))
        cache = freeze_query_cache(estimator, 1)
        query = frame(2, points, dynamic=True)
        expected = estimator._checked_frame(query)
        with patch.object(WindowEstimator, '_checked_frame',
                          side_effect=AssertionError('valid standard input must use batch checks')):
            actual = _checked_query(query, cache)
        for a, b in zip(actual.observations, expected.observations):
            self.assertEqual(a.track_id, b.track_id)
            np.testing.assert_array_equal(a.point_camera, b.point_camera)
            np.testing.assert_array_equal(a.covariance_camera, b.covariance_camera)
            self.assertTrue(a.point_camera.flags.owndata)
            self.assertTrue(a.covariance_camera.flags.owndata)
        query.observations[0].point_camera[:] = np.nan
        query.observations[0].covariance_camera[:] = np.nan
        query.nominal_R_wb[:] = np.nan
        np.testing.assert_array_equal(actual.observations[0].point_camera,
                                      expected.observations[0].point_camera)
        np.testing.assert_array_equal(actual.observations[0].covariance_camera,
                                      expected.observations[0].covariance_camera)
        np.testing.assert_array_equal(actual.nominal_R_wb, expected.nominal_R_wb)

    def test_covariance_symmetry_boundary_and_nonstandard_fallback_match_reference(self):
        points = scene(18)
        estimator = WindowEstimator()
        estimator.add_frame(frame(1, points))
        cache = freeze_query_cache(estimator, 1)
        for asymmetry in (0., .5e-12, 2.e-12):
            query = frame(2, points)
            covariance = query.observations[0].covariance_camera
            covariance[0, 1] = 1.e-6
            covariance[1, 0] = 1.e-6+asymmetry
            with self.subTest(asymmetry=asymmetry):
                if asymmetry < 2.e-12:
                    expected = estimator._checked_frame(query)
                    actual = _checked_query(query, cache)
                    np.testing.assert_array_equal(actual.observations[0].covariance_camera,
                                                  expected.observations[0].covariance_camera)
                else:
                    with self.assertRaises(WindowError) as reference:
                        estimator._checked_frame(query)
                    with self.assertRaises(WindowError) as candidate:
                        _checked_query(query, cache)
                    self.assertEqual(str(candidate.exception), str(reference.exception))
        for convert in (lambda a: a.astype(np.float32), lambda a: a.tolist()):
            query = frame(2, points)
            observations = [Observation(o.track_id, convert(o.point_camera),
                                        convert(o.covariance_camera)) for o in query.observations]
            query = Frame(query.stamp_ns, query.nominal_R_wb, query.R_bc,
                          query.t_bc, observations, query.attitude_cov)
            expected = estimator._checked_frame(query)
            original_validator = WindowEstimator._checked_frame
            with patch.object(WindowEstimator, '_checked_frame', autospec=True,
                              side_effect=original_validator) as fallback:
                actual = _checked_query(query, cache)
            self.assertEqual(fallback.call_count, 1)
            for a, b in zip(actual.observations, expected.observations):
                np.testing.assert_array_equal(a.point_camera, b.point_camera)
                np.testing.assert_array_equal(a.covariance_camera, b.covariance_camera)

    def test_invalid_observation_order_preserves_original_first_error(self):
        points = scene(18)
        estimator = WindowEstimator()
        estimator.add_frame(frame(1, points))
        cache = freeze_query_cache(estimator, 1)
        cases = []
        query = frame(1, points)
        query.observations[0].point_camera[2] = -1.
        cases.append(query)
        query = frame(2, points)
        query.observations[0].point_camera[2] = -1.
        query.observations[0].covariance_camera[0, 1] = 1.
        cases.append(query)
        query = frame(2, points)
        query.observations[0].covariance_camera[0, 1] = 1.
        query.observations[1] = Observation(True, np.zeros(3), np.eye(3))
        cases.append(query)
        query = frame(2, points)
        query.observations[0].covariance_camera[0, 0] = -1.
        query.observations[1] = query.observations[0]
        cases.append(query)
        query = frame(2, points)
        query.observations[0].point_camera[0] = np.nan
        query.observations[0].covariance_camera[0, 0] = -1.
        cases.append(query)
        query = frame(2, points)
        query.nominal_R_wb[0, 0] = 2.
        cases.append(query)
        query = frame(2, points)
        query.attitude_cov[0, 0] = -1.
        cases.append(query)
        for index, query in enumerate(cases):
            with self.subTest(case=index):
                with self.assertRaises(WindowError) as reference:
                    estimator._checked_frame(query)
                with self.assertRaises(WindowError) as candidate:
                    _checked_query(query, cache)
                self.assertEqual(str(candidate.exception), str(reference.exception))
        self.assertEqual(cache.cached_columns, 0)

    def test_batched_residuals_keep_factor_and_block_accumulation_order(self):
        points = scene(30)
        estimator = WindowEstimator(6)
        for i in range(1, 4):
            estimator.add_frame(frame(i, points, dynamic=True, noise=.0003))
        query = estimator._checked_frame(frame(4, points, dynamic=True, noise=.0003))
        rng = np.random.default_rng(372)
        for current in (*estimator._frames, query):
            factors = tuple(estimator._factor(current, o, estimator._first_stamp)
                            for o in current.observations)
            state = {key: rng.normal(0., 2., 3) for blocks, _, _ in factors for key in blocks}
            expected = np.asarray([constant+sum(J@state[key] for key, J in blocks.items())
                                   for blocks, constant, _ in factors])
            actual = _batch_residuals(factors, state)
            np.testing.assert_allclose(actual, expected, atol=2e-15, rtol=1e-14)
            self.assertAlmostEqual(float(np.sqrt(np.mean(np.sum(actual**2, axis=1)))),
                                   float(np.sqrt(np.mean(np.sum(expected**2, axis=1)))), delta=2e-15)

    def test_nonfinite_batched_residual_still_rejects_without_committing_columns(self):
        points = scene(18)
        estimator = WindowEstimator()
        estimator.add_frame(frame(1, points))
        cache = freeze_query_cache(estimator, 1)
        with patch('offline_rgbd_window.query_cache._batch_residuals',
                   return_value=np.full((18, 3), np.nan)):
            with self.assertRaisesRegex(WindowError, 'nonfinite back-substituted'):
                query_cached(frame(2, points), cache, 1)
        self.assertEqual(cache.cached_columns, 0)


if __name__ == '__main__':
    unittest.main()
