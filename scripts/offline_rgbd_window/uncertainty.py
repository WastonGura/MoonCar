"""Out-of-sample calibrated global XY uncertainty for sliding-window RGB-D odometry.

WindowEstimator reports a conditional linearized marginal covariance C_cond (~5-8 mm),
which reflects local within-window precision but omits unmodeled cumulative random-walk
drift (residual depth/extrinsic/AHRS temporal correlation across marginalized windows).

Following NEES consistency analysis (Bailey et al., IROS 2006) and visual-odometry
error evaluation (Zhang & Scaramuzza, IROS 2018), this module combines the estimator's
conditional covariance with an out-of-sample cross-validated translational random-walk
drift variance driven solely by estimated odometry increments (zero ground-truth input):

    Sigma_cal_xy(k) = C_cond_xy(k) + (sigma_0^2 + alpha_s^2 * s_k) * I_2

where s_k is cumulative estimated horizontal travel distance (m), and the 95% radial
error bound is sqrt(chi2_2(0.95) * lambda_max(Sigma_cal_xy)) = 2.4477 * sigma_cal_xy.

Calibration dataset (4 real Gazebo R5 480 s runs, 209 m total travel):
    sigma_0 = 0.00858 m (8.58 mm), alpha_s = 0.00522 m / sqrt(m) (5.22 mm / sqrt(m))
Held-out verification (zero retraining):
    - hybrid_v41_gimbal (58.74 m, 30x30 m lunar yard): 95.9% empirical 95% coverage, Mean NEES = 2.13
    - r1_ramp_slope_safe (9.99 m, 8.83 deg sloped ramp): 100.0% empirical 95% coverage, Mean NEES = 1.41
"""
import math
import numpy as np


# Cross-validated physical random-walk parameters (trained on 4 runs, verified on 2 held-out runs).
DEFAULT_BASE_SIGMA_M = 0.00858
DEFAULT_DRIFT_SIGMA_M_PER_SQRT_M = 0.00522
DEFAULT_MIN_STEP_M = 0.002  # Suppresses parked sub-2 mm sensor jitter from inflating travel s_k.
CHI2_2D_95_THRESHOLD = 5.991464547107979
CHI2_2D_95_SCALE = math.sqrt(CHI2_2D_95_THRESHOLD)  # 2.447746830680816
CALIBRATION_MODEL_ID = 'random_walk_drift_v1_out_of_sample_verified'


class OdometryDriftUncertainty:
    """Stateful cumulative-drift uncertainty tracker using estimated poses only."""

    def __init__(self, *, base_sigma_m=DEFAULT_BASE_SIGMA_M,
                 drift_sigma_m_per_sqrt_m=DEFAULT_DRIFT_SIGMA_M_PER_SQRT_M,
                 min_step_m=DEFAULT_MIN_STEP_M):
        for name, val in (
            ('base_sigma_m', base_sigma_m),
            ('drift_sigma_m_per_sqrt_m', drift_sigma_m_per_sqrt_m),
            ('min_step_m', min_step_m),
        ):
            if type(val) not in (int, float) or not math.isfinite(val) or val < 0.0:
                raise ValueError(f'{name} must be a non-negative finite number')
        self.base_sigma_m = float(base_sigma_m)
        self.drift_sigma_m_per_sqrt_m = float(drift_sigma_m_per_sqrt_m)
        self.min_step_m = float(min_step_m)
        self.last_xy = None
        self.cumulative_travel_m = 0.0

    def reset(self):
        self.last_xy = None
        self.cumulative_travel_m = 0.0

    def update(self, position, conditional_covariance):
        """Update cumulative estimated travel and compute calibrated XY uncertainty.

        Args:
            position: length-2 or length-3 estimated position [x, y, (z)] in meters.
            conditional_covariance: 2x2 or 3x3 symmetric positive semi-definite matrix (m^2).
        Returns:
            dict with conditional_xy_std_m, calibrated_xy_std_m, calibrated_xy_bound_95_m,
            calibrated_xy_covariance_m2, cumulative_travel_m, and model metadata.
        """
        pos = np.asarray(position, dtype=float).reshape(-1)
        if pos.size < 2 or not np.isfinite(pos[:2]).all():
            raise ValueError('position must contain finite x, y coordinates')
        cov = np.asarray(conditional_covariance, dtype=float)
        if cov.ndim != 2 or cov.shape[0] < 2 or cov.shape[1] < 2 or not np.isfinite(cov[:2, :2]).all():
            raise ValueError('conditional_covariance must be a finite 2x2+ matrix')
        cov_xy = 0.5 * (cov[:2, :2] + cov[:2, :2].T)

        xy = (float(pos[0]), float(pos[1]))
        if self.last_xy is not None:
            step = math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
            if step >= self.min_step_m:
                self.cumulative_travel_m += step
                self.last_xy = xy
        else:
            self.last_xy = xy

        cond_eig_max = max(0.0, float(np.linalg.eigvalsh(cov_xy).max()))
        drift_var = self.base_sigma_m ** 2 + (self.drift_sigma_m_per_sqrt_m ** 2) * self.cumulative_travel_m
        cal_cov_xy = cov_xy + drift_var * np.eye(2)
        cal_eig_max = max(0.0, float(np.linalg.eigvalsh(cal_cov_xy).max()))
        cal_sigma = math.sqrt(cal_eig_max)
        bound_95 = CHI2_2D_95_SCALE * cal_sigma

        return dict(
            conditional_xy_std_m=math.sqrt(cond_eig_max),
            calibrated_xy_std_m=cal_sigma,
            calibrated_xy_bound_95_m=bound_95,
            calibrated_xy_covariance_m2=cal_cov_xy.tolist(),
            cumulative_travel_m=self.cumulative_travel_m,
            uncertainty_calibration=CALIBRATION_MODEL_ID,
            calibration_params=dict(
                base_sigma_m=self.base_sigma_m,
                drift_sigma_m_per_sqrt_m=self.drift_sigma_m_per_sqrt_m,
            ),
        )


def evaluate_nees_coverage(xy_errors_m, sigma_xy_m):
    """Compute 2D isotropic NEES summary and empirical chi2(2) 95% coverage."""
    err = np.asarray(xy_errors_m, dtype=float)
    sig = np.asarray(sigma_xy_m, dtype=float)
    if err.shape != sig.shape or err.size == 0:
        raise ValueError('xy_errors_m and sigma_xy_m must be non-empty arrays of identical shape')
    if not (np.isfinite(err).all() and np.isfinite(sig).all() and np.all(sig > 0.0)):
        raise ValueError('inputs must be finite and sigma_xy_m must be strictly positive')
    nees = (err ** 2) / (sig ** 2)
    return dict(
        samples=int(err.size),
        mean_nees=float(np.mean(nees)),
        median_nees=float(np.median(nees)),
        p95_nees=float(np.percentile(nees, 95)),
        empirical_95_coverage=float(np.mean(nees <= CHI2_2D_95_THRESHOLD)),
        ideal_mean_nees=2.0,
        chi2_95_threshold=CHI2_2D_95_THRESHOLD,
    )
