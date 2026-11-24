"""Unit tests for R5 calibrated odometry drift uncertainty and R7 weak-geometry reselection."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

from offline_rgbd_window.uncertainty import (
    CALIBRATION_MODEL_ID,
    CHI2_2D_95_SCALE,
    DEFAULT_BASE_SIGMA_M,
    DEFAULT_DRIFT_SIGMA_M_PER_SQRT_M,
    OdometryDriftUncertainty,
    evaluate_nees_coverage,
)


class R5UncertaintyCalibrationTests(unittest.TestCase):
    def test_parked_jitter_does_not_inflate_travel_or_drift(self):
        tracker = OdometryDriftUncertainty()
        cov = np.diag([0.005 ** 2, 0.005 ** 2, 0.005 ** 2])
        first = tracker.update([0.0, 0.0, 0.0], cov)
        self.assertAlmostEqual(first['cumulative_travel_m'], 0.0, places=9)
        expected_0 = math.hypot(0.005, DEFAULT_BASE_SIGMA_M)
        self.assertAlmostEqual(first['calibrated_xy_std_m'], expected_0, places=6)
        self.assertAlmostEqual(first['calibrated_xy_bound_95_m'], CHI2_2D_95_SCALE * expected_0, places=6)
        self.assertEqual(first['uncertainty_calibration'], CALIBRATION_MODEL_ID)

        # Sub-2mm parked noise over 200 updates must not inflate cumulative travel.
        for i in range(200):
            dx = 0.0005 if i % 2 == 0 else -0.0005
            res = tracker.update([dx, 0.0004, 0.0], cov)
        self.assertAlmostEqual(res['cumulative_travel_m'], 0.0, places=9)
        self.assertAlmostEqual(res['calibrated_xy_std_m'], expected_0, places=6)

    def test_uncertainty_grows_monotonically_with_estimated_travel(self):
        tracker = OdometryDriftUncertainty()
        cov = np.diag([0.006 ** 2, 0.006 ** 2, 0.006 ** 2])
        sigmas = []
        for step in range(0, 61):
            res = tracker.update([float(step), 0.0, 0.0], cov)
            sigmas.append(res['calibrated_xy_std_m'])
        self.assertAlmostEqual(res['cumulative_travel_m'], 60.0, places=6)
        self.assertTrue(all(b > a for a, b in zip(sigmas, sigmas[1:])))
        # At 60 m travel, calibrated 1-sigma should be ~4.1 cm (matching observed 5-6 cm XY P95)
        expected_60 = math.sqrt(0.006 ** 2 + DEFAULT_BASE_SIGMA_M ** 2 + (DEFAULT_DRIFT_SIGMA_M_PER_SQRT_M ** 2) * 60.0)
        self.assertAlmostEqual(res['calibrated_xy_std_m'], expected_60, places=6)
        self.assertGreater(res['calibrated_xy_bound_95_m'], 0.09)
        self.assertLess(res['calibrated_xy_bound_95_m'], 0.12)

        tracker.reset()
        after_reset = tracker.update([10.0, 5.0, 0.0], cov)
        self.assertAlmostEqual(after_reset['cumulative_travel_m'], 0.0, places=9)

    def test_rejects_invalid_inputs(self):
        tracker = OdometryDriftUncertainty()
        with self.assertRaises(ValueError):
            OdometryDriftUncertainty(base_sigma_m=-0.01)
        with self.assertRaises(ValueError):
            tracker.update([float('nan'), 0.0], np.eye(2) * 1e-4)
        with self.assertRaises(ValueError):
            tracker.update([0.0, 0.0], np.array([[float('inf'), 0.0], [0.0, 1e-4]]))

    def test_nees_evaluation_distinguishes_calibrated_vs_overconfident_sigma(self):
        rng = np.random.default_rng(42)
        true_sigma = 0.04
        # Draw 20,000 samples from 2D isotropic N(0, true_sigma^2 I_2)
        err_xy = np.hypot(rng.normal(0.0, true_sigma, 20000), rng.normal(0.0, true_sigma, 20000))

        cal = evaluate_nees_coverage(err_xy, np.full_like(err_xy, true_sigma))
        self.assertAlmostEqual(cal['mean_nees'], 2.0, delta=0.08)
        self.assertAlmostEqual(cal['empirical_95_coverage'], 0.95, delta=0.01)

        overconf = evaluate_nees_coverage(err_xy, np.full_like(err_xy, 0.006))
        self.assertGreater(overconf['mean_nees'], 50.0)
        self.assertLess(overconf['empirical_95_coverage'], 0.15)


class R7WeakGeometryReselectionTests(unittest.TestCase):
    def test_metric_spread_reselection_recovers_lateral_support_on_collinear_strip(self):
        try:
            from offline_rgbd_window.frontend import TrackFrontend
        except ImportError as exc:
            self.skipTest(f'OpenCV/SciPy frontend dependencies unavailable on host: {exc}')

        fe = TrackFrontend(max_observations=48, max_frames=6)
        rng = np.random.default_rng(7)
        # Create 60 high-age collinear points along X (y ~ 0) + 36 younger lateral points (y = +-1.2 m).
        n_col, n_lat = 60, 36
        pts_col = np.column_stack([np.linspace(0.5, 4.5, n_col), rng.normal(0.0, 0.01, n_col), np.zeros(n_col)])
        pts_lat = np.column_stack([
            np.linspace(0.8, 3.8, n_lat),
            np.where(np.arange(n_lat) % 2 == 0, 1.2, -1.2),
            np.zeros(n_lat),
        ])
        points = np.vstack([pts_col, pts_lat])
        # Spread pixels across image ROI [0, 640] x [0, 480]
        pix_col = np.column_stack([np.linspace(40, 600, n_col), np.linspace(220, 260, n_col)])
        pix_lat = np.column_stack([
            np.linspace(60, 580, n_lat),
            np.where(np.arange(n_lat) % 2 == 0, 80.0, 400.0),
        ])
        pixels = np.vstack([pix_col, pix_lat])
        track_ids = np.arange(1, n_col + n_lat + 1, dtype=np.int64)
        # Old collinear tracks have higher age so initial selection picks mostly collinear strip
        fe.age = {int(t): (20 if t <= n_col else 3) for t in track_ids}
        # Seed keyframe with all tracks active so lateral tracks are also genuine map landmarks
        fe.keyframes.append((dict(submitted_ids=track_ids.copy()), track_ids.copy()))

        features = dict(
            pixels=pixels,
            points=points,
            covariance=np.broadcast_to(np.eye(3) * (0.01 ** 2), (len(points), 3, 3)).copy(),
            track_ids=track_ids,
            image_shape=(480, 640),
            measured_roi=np.array([0.0, 0.0, 640.0, 480.0]),
            tracking_pose_adequate=True,
        )
        selected, diag = fe.select(features)
        self.assertTrue(diag['adequate'])
        self.assertGreaterEqual(len(selected), 24)
        # Verify either initial spread or metric_spread_retry achieved valid 2D eigenvalue support
        self.assertGreaterEqual(diag['metric_support_eigenvalues_m2'][1], 0.0025)


if __name__ == '__main__':
    unittest.main()
