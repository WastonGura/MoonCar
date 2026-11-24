"""Small model tests; no ROS, campaign files, truth inputs, or production edits."""
import importlib.util
import copy
from pathlib import Path
import pickle
import sys
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation


CORE = Path(__file__).resolve().parents[1] / "scripts" / "offline_rgbd_window" / "core.py"
SPEC = importlib.util.spec_from_file_location("offline_rgbd_window_core", CORE)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
Observation, Frame = MODULE.Observation, MODULE.Frame
WindowEstimator, WindowError = MODULE.WindowEstimator, MODULE.WindowError


def scene(count=12):
    rng = np.random.default_rng(17)
    return np.column_stack((rng.uniform(-1., 1., count),
                            rng.uniform(-.7, .7, count), rng.uniform(2., 4., count)))


def exposure(number, landmarks, position, body=None, camera=None, lever=None,
             point_sigma=.004, attitude_sigma=.001):
    body = np.eye(3) if body is None else body
    camera = np.eye(3) if camera is None else camera
    lever = np.zeros(3) if lever is None else lever
    measured = ((landmarks - position) @ body - lever) @ camera
    observations = [Observation(i, z, np.eye(3)*point_sigma**2)
                    for i, z in enumerate(measured)]
    return Frame(number*100_000_000, body, camera, lever, observations,
                 np.eye(3)*attitude_sigma**2)


class WindowCoreTests(unittest.TestCase):
    def test_shared_landmarks_recover_translation_with_dynamic_gimbal_and_lever(self):
        estimator = WindowEstimator(max_frames=3)
        landmarks = scene()
        estimator.add_frame(exposure(1, landmarks, np.zeros(3)))
        for i in range(2, 7):
            p = np.array((.03*i, -.01*i, .005*i))
            body = Rotation.from_euler("xyz", (.01*i, -.008*i, .02*i)).as_matrix()
            camera = Rotation.from_euler("yz", (.05*i, -.04*i)).as_matrix()
            result = estimator.add_frame(exposure(i, landmarks, p, body, camera,
                                                 np.array((.15, 0., .35))))
            np.testing.assert_allclose(result.position, p, atol=2e-9)
            self.assertLessEqual(result.diagnostics["frame_count"], 3)
        self.assertEqual(result.diagnostics["marginalized_frames"], 3)
        self.assertGreater(result.diagnostics["prior_cross_information_norm"], 0.)

    def test_schur_window_matches_batch_position_and_covariance(self):
        small, batch = WindowEstimator(2), WindowEstimator(10)
        landmarks = scene()
        rng = np.random.default_rng(88)
        for i in range(1, 9):
            frame = exposure(i, landmarks, np.array((.025*(i-1), .003*(i-1), 0.)))
            noisy = [Observation(o.track_id, o.point_camera+rng.normal(0., .0006, 3),
                                 o.covariance_camera) for o in frame.observations]
            frame = Frame(frame.stamp_ns, frame.nominal_R_wb, frame.R_bc, frame.t_bc,
                          noisy, frame.attitude_cov)
            a, b = small.add_frame(frame), batch.add_frame(frame)
            np.testing.assert_allclose(a.position, b.position, atol=2e-8, rtol=1e-7)
            np.testing.assert_allclose(a.covariance, b.covariance, atol=2e-11, rtol=1e-6)
        self.assertEqual(a.diagnostics["frame_count"], 2)
        self.assertEqual(a.diagnostics["total_frames"], 8)

    def test_common_error_does_not_average_away_with_point_count(self):
        answers = []
        for count in (6, 96):
            estimator = WindowEstimator()
            landmarks = scene(count)
            estimator.add_frame(exposure(1, landmarks, np.zeros(3), point_sigma=.0001,
                                         attitude_sigma=.00001))
            result = estimator.add_frame(exposure(2, landmarks, np.array((.04, 0., 0.)),
                                                  point_sigma=.0001, attitude_sigma=.00001))
            answers.append(np.diag(result.covariance))
            self.assertGreater(np.linalg.eigvalsh(result.covariance).min(), 1.99*.003**2)
            self.assertEqual(result.diagnostics["output_floor_added_variance_m2"], 0.)
        self.assertLess(answers[0].max()/answers[1].min(), 1.01)

    def test_attitude_prior_is_shared_uncertainty_not_per_feature_average(self):
        covariances = []
        for sigma in (.0001, .01):
            estimator = WindowEstimator()
            landmarks = scene(24)
            estimator.add_frame(exposure(1, landmarks, np.zeros(3), attitude_sigma=sigma))
            covariances.append(estimator.add_frame(exposure(2, landmarks,
                np.array((.02, 0., 0.)), attitude_sigma=sigma)).covariance)
        self.assertGreater(np.trace(covariances[1]), np.trace(covariances[0])*1.05)

    def test_disconnected_reject_is_atomic_and_next_valid_frame_works(self):
        estimator = WindowEstimator(2)
        landmarks = scene()
        estimator.add_frame(exposure(1, landmarks, np.zeros(3)))
        bad = exposure(2, landmarks, np.zeros(3))
        bad = Frame(bad.stamp_ns, bad.nominal_R_wb, bad.R_bc, bad.t_bc,
                    [Observation(o.track_id+1000, o.point_camera, o.covariance_camera)
                     for o in bad.observations], bad.attitude_cov)
        with self.assertRaisesRegex(WindowError, "three existing"):
            estimator.add_frame(bad)
        self.assertEqual(estimator._total_frames, 1)
        self.assertEqual(estimator._first_stamp, 100_000_000)
        result = estimator.add_frame(exposure(3, landmarks, np.array((.04, 0., 0.))))
        np.testing.assert_allclose(result.position, (.04, 0., 0.), atol=1e-9)
        self.assertEqual(result.diagnostics["total_frames"], 2)

    def test_inputs_are_owned_and_invalid_measurements_do_not_reset(self):
        estimator = WindowEstimator()
        frame = exposure(1, scene(), np.zeros(3))
        estimator.add_frame(frame)
        frame.observations[0].point_camera[:] = np.nan
        frame.nominal_R_wb[:] = 0.
        self.assertTrue(np.isfinite(estimator._frames[0].observations[0].point_camera).all())
        for bad in (frame, exposure(1, scene(), np.zeros(3))):
            with self.assertRaises(WindowError):
                estimator.add_frame(bad)
        too_many = exposure(2, scene(97), np.zeros(3))
        with self.assertRaisesRegex(WindowError, "3..96"):
            estimator.add_frame(too_many)
        self.assertEqual(estimator._total_frames, 1)

    def test_dead_landmarks_are_eliminated_without_reanchoring(self):
        estimator = WindowEstimator(2)
        initial = scene()
        estimator.add_frame(exposure(1, initial, np.zeros(3)))
        for i in range(2, 7):
            # Keep six original shared tracks; remaining tracks are genuinely new.
            frame = exposure(i, initial, np.array((.03*(i-1), 0., 0.)))
            observations = [Observation(o.track_id if o.track_id < 6 else o.track_id+i*100,
                                        o.point_camera, o.covariance_camera)
                            for o in frame.observations]
            result = estimator.add_frame(Frame(frame.stamp_ns, frame.nominal_R_wb,
                frame.R_bc, frame.t_bc, observations, frame.attitude_cov))
            np.testing.assert_allclose(result.position, (.03*(i-1), 0., 0.), atol=1e-8)
            self.assertLessEqual(result.diagnostics["landmark_count"], 18)
        self.assertEqual(estimator._first_stamp, 100_000_000)

    def test_small_angle_model_guard_rolls_back(self):
        estimator = WindowEstimator()
        landmarks = scene(48)
        estimator.add_frame(exposure(1, landmarks, np.zeros(3), attitude_sigma=.00001))
        # Strongly known first attitude but permissive second prior: a true
        # 8-degree difference is outside this prototype's linearized contract.
        actual = Rotation.from_euler("z", 8., degrees=True).as_matrix()
        bad = exposure(2, landmarks, np.zeros(3), actual, attitude_sigma=.1)
        bad = Frame(bad.stamp_ns, np.eye(3), bad.R_bc, bad.t_bc,
                    bad.observations, bad.attitude_cov)
        with self.assertRaisesRegex(WindowError, "3 degree"):
            estimator.add_frame(bad)
        self.assertEqual(estimator._total_frames, 1)
        self.assertEqual(estimator._last_stamp, 100_000_000)

    def test_fixed_factor_cache_is_bounded_and_does_not_recompute_retained_measurements(self):
        estimator = WindowEstimator(3)
        landmarks = scene(96)
        with patch.object(estimator, '_factor', wraps=estimator._factor) as factor:
            for i in range(1, 8):
                estimator.add_frame(exposure(i, landmarks, np.array((.02*i, 0., 0.))))
                self.assertEqual(factor.call_count, i*96)
                self.assertEqual(set(estimator._frame_terms), {f.stamp_ns for f in estimator._frames})
                self.assertLessEqual(len(estimator._frame_terms), 3)
            before = factor.call_count
            estimator._solve()
            self.assertEqual(factor.call_count, before)

    def test_query_equals_committed_joint_map_and_restores_every_persistent_field(self):
        estimator = WindowEstimator(3)
        landmarks = scene(32)
        for i in range(1, 5):
            estimator.add_frame(exposure(i, landmarks, np.array((.03*i, 0., 0.))))
        before = pickle.dumps(estimator.__dict__)
        original_fields = estimator.__dict__.copy()
        query = exposure(5, landmarks, np.array((.15, .01, 0.)),
            Rotation.from_euler('z', .03).as_matrix(),
            Rotation.from_euler('y', .1).as_matrix(), np.array((.15, 0., .35)))
        expected = copy.deepcopy(estimator).add_frame(query)
        for _ in range(2):
            answer = estimator.query_frame(query)
            np.testing.assert_array_equal(answer.position, expected.position)
            np.testing.assert_array_equal(answer.covariance, expected.covariance)
            self.assertEqual(answer.diagnostics, expected.diagnostics)
            self.assertEqual(pickle.dumps(estimator.__dict__), before)
            for key, value in original_fields.items():
                self.assertIs(estimator.__dict__[key], value)
        # A previous query did not consume its stamp or double-count its data.
        committed = estimator.add_frame(query)
        np.testing.assert_array_equal(committed.position, expected.position)
        np.testing.assert_array_equal(committed.covariance, expected.covariance)

    def test_query_exception_and_failed_post_schur_transaction_restore_cache_and_prior(self):
        estimator = WindowEstimator(2)
        landmarks = scene(48)
        for i in range(1, 4):
            estimator.add_frame(exposure(i, landmarks, np.zeros(3), attitude_sigma=.00001))
        actual = Rotation.from_euler('z', 8., degrees=True).as_matrix()
        bad = exposure(4, landmarks, np.zeros(3), actual, attitude_sigma=.1)
        bad = Frame(bad.stamp_ns, np.eye(3), bad.R_bc, bad.t_bc, bad.observations, bad.attitude_cov)
        before = pickle.dumps(estimator.__dict__)
        for operation in (estimator.query_frame, estimator.add_frame):
            with self.assertRaisesRegex(WindowError, '3 degree'):
                operation(bad)
            self.assertEqual(pickle.dumps(estimator.__dict__), before)

    def test_sparse_assembly_keeps_dense_prior_cross_information_and_matches_direct_normal_equations(self):
        estimator = WindowEstimator(2)
        landmarks = scene(18)
        for i in range(1, 5):
            estimator.add_frame(exposure(i, landmarks, np.array((.02*i, .005*i, 0.)),
                Rotation.from_euler('z', .01*i).as_matrix(),
                Rotation.from_euler('y', .02*i).as_matrix(), np.array((.15, .02, .35))))
        keys = estimator._keys()
        H, eta = estimator._assemble(keys, estimator._frames)
        direct, rhs = np.zeros(H.shape), np.zeros(len(eta))
        offsets = {key: 3*i for i, key in enumerate(keys)}
        indices = np.array([offsets[key]+j for key in estimator._prior_keys for j in range(3)])
        direct[np.ix_(indices, indices)] += estimator._prior_H
        rhs[indices] += estimator._prior_eta
        for f in estimator._frames:
            a, b = offsets[('a', f.stamp_ns)], offsets[('b', f.stamp_ns)]
            direct[a:a+3, a:a+3] += np.linalg.inv(f.attitude_cov)
            direct[b:b+3, b:b+3] += np.eye(3)/estimator.shared_calibration_sigma_m**2
            for o in f.observations:
                blocks, constant, W = estimator._factor(f, o, estimator._first_stamp)
                J = np.zeros((3, len(eta)))
                for key, block in blocks.items():
                    at = offsets[key]
                    J[:, at:at+3] = block
                direct += J.T@W@J
                rhs -= J.T@W@constant
        np.testing.assert_allclose(H.toarray(), direct, rtol=2e-13, atol=2e-8)
        np.testing.assert_allclose(eta, rhs, rtol=2e-13, atol=2e-8)


if __name__ == "__main__":
    unittest.main()
