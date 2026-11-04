"""Experimental sensor-feedback allocation. No wheel odometry or simulator truth.

The fixed flat-load ratio is only a bounded initial estimate. An independent
laser registration must qualify each update; loss of evidence inhibits turns.
This does not change the WheelSlip contact model or certify terrain performance.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial import cKDTree


def rotation(angle):
    return np.array([[math.cos(angle), -math.sin(angle)],
                     [math.sin(angle), math.cos(angle)]])


def scan_points(ranges, angle_min, angle_increment, range_min, range_max):
    data = np.array([float('nan') if x is None else x for x in ranges], dtype=float)
    angles = angle_min + np.arange(len(data)) * angle_increment
    good = np.isfinite(data) & (data > range_min) & (data < range_max)
    points = np.column_stack((data[good]*np.cos(angles[good]), data[good]*np.sin(angles[good])))
    if len(points) > 512:
        points = points[np.linspace(0, len(points)-1, 512).astype(int)]
    return points


def fit_scan_motion(previous, current, seed_yaw=0.):
    """Point-to-line ICP; maps current laser points into previous laser frame.

    Normal-equation rank rejects a single wall/corridor; low residual alone
    must not be treated as observable translation. Covariance is an engineering
    registration diagnostic, not a calibrated real-sensor uncertainty.
    """
    a, b = np.asarray(previous, float), np.asarray(current, float)
    if (a.ndim != 2 or b.ndim != 2 or a.shape[1:] != (2,) or b.shape[1:] != (2,)
            or len(a) < 40 or len(b) < 40 or not np.isfinite(a).all()
            or not np.isfinite(b).all() or not math.isfinite(seed_yaw)):
        return dict(valid=False, reason='insufficient_or_nonfinite_scan')
    tree = cKDTree(a)
    distances, neighbours = tree.query(a, k=7)
    local = a[neighbours] - a[neighbours].mean(axis=1, keepdims=True)
    eigenvalues, eigenvectors = np.linalg.eigh(np.einsum('nki,nkj->nij', local, local)/7)
    normals = eigenvectors[:, :, 0]
    good_normals = (eigenvalues[:, 1] > 1e-7) & (eigenvalues[:, 0] < .15*eigenvalues[:, 1]) & (distances[:, -1] < .5)
    theta = seed_yaw
    translation = np.zeros(2)
    converged = False
    for _ in range(15):
        rotated = b @ rotation(theta).T
        mapped = rotated + translation
        distance, index = tree.query(mapped)
        selected = (distance < .15) & good_normals[index]
        if selected.sum() < 40 or selected.mean() < .45:
            return dict(valid=False, reason='insufficient_overlap', inliers=int(selected.sum()))
        n = normals[index[selected]]
        q = rotated[selected]
        residual = np.sum(n*(mapped[selected]-a[index[selected]]), axis=1)
        selected_residual = abs(residual) < .06
        n, q, residual = n[selected_residual], q[selected_residual], residual[selected_residual]
        jacobian = np.column_stack((n, n[:, 0]*(-q[:, 1])+n[:, 1]*q[:, 0]))
        hessian = jacobian.T @ jacobian
        spectrum = np.linalg.eigvalsh(hessian)
        ratio = float(spectrum[0]/max(spectrum[-1], 1e-30))
        if len(residual) < 40 or ratio < .002:
            return dict(valid=False, reason='unobservable_scan_geometry', condition_ratio=ratio)
        delta = np.linalg.solve(hessian, -jacobian.T @ residual)
        if not np.isfinite(delta).all() or np.linalg.norm(delta[:2]) > .1 or abs(delta[2]) > .1:
            return dict(valid=False, reason='registration_jump')
        translation += delta[:2]
        theta += delta[2]
        if np.linalg.norm(delta[:2]) < .0002 and abs(delta[2]) < .0002:
            converged = True
            break
    # Recompute evidence at the FINAL transform, rather than stale residuals.
    rotated = b @ rotation(theta).T
    mapped = rotated+translation
    distances, index = tree.query(mapped)
    selected = (distances < .15) & good_normals[index]
    n, q = normals[index[selected]], rotated[selected]
    residual = np.sum(n*(mapped[selected]-a[index[selected]]), axis=1)
    good = abs(residual) < .06
    n, q, residual = n[good], q[good], residual[good]
    jacobian = np.column_stack((n, -n[:, 0]*q[:, 1]+n[:, 1]*q[:, 0]))
    hessian = jacobian.T @ jacobian
    spectrum = np.linalg.eigvalsh(hessian)
    if len(residual) < 40 or spectrum[0]/max(spectrum[-1], 1e-30) < .002:
        return dict(valid=False, reason='final_geometry_unobservable')
    rms = float(np.sqrt(np.mean(residual**2)))
    covariance = np.linalg.inv(hessian)*max(rms*rms, .005**2)
    position_sigma = float(np.sqrt(np.linalg.eigvalsh(covariance[:2, :2]).max()))
    valid = converged and rms <= .02 and position_sigma <= .004 and abs(theta-seed_yaw) <= .03
    return dict(valid=bool(valid), reason='' if valid else 'registration_quality',
                translation=translation.tolist(), yaw=float(theta), residual_rms_m=rms,
                position_sigma_m=position_sigma, inliers=len(residual),
                condition_ratio=float(spectrum[0]/spectrum[-1]), converged=converged)


def body_delta(registration, lidar_xy=(.11, 0.)):
    if not registration.get('valid'):
        raise ValueError('Unqualified laser registration')
    offset = np.asarray(lidar_xy, float)
    # T_base,laser * T_prevlaser,currentlaser * inverse(T_base,laser).
    return np.asarray(registration['translation']) + offset - rotation(registration['yaw']) @ offset


@dataclass
class AdaptiveAllocation:
    initial_front: float = .96026822178862603
    gain: float = .1             # 1/(m*s), fixed candidate, not a tuning search
    filter_seconds: float = .5
    maximum_rate: float = .02    # ratio/s
    minimum_omega: float = .05
    front: float = None
    filtered_lateral: float = 0.
    last_ns: int = None
    low_yaw_seconds: float = 0.

    def __post_init__(self):
        if not .95 <= self.initial_front <= 1.05:
            raise ValueError('Initial estimate outside preserved 5% envelope')
        self.front = self.initial_front

    def reset(self):
        self.front = self.initial_front
        self.filtered_lateral = 0.
        self.last_ns = None
        self.low_yaw_seconds = 0.

    def update(self, stamp_ns, dt, lateral_velocity, omega, requested_linear,
               requested_angular, valid=True):
        values = (dt, lateral_velocity, omega, requested_linear, requested_angular)
        if not isinstance(stamp_ns, int) or stamp_ns < 0 or not all(math.isfinite(x) for x in values):
            return dict(valid=False, reason='nonfinite_feedback', front=1., rear=1.)
        if self.last_ns is not None and stamp_ns <= self.last_ns:
            return dict(valid=False, reason='duplicate_or_reversed_feedback', front=1., rear=1.)
        if requested_angular == 0 or abs(requested_linear) > 1e-9:
            self.reset()
            return dict(valid=True, reason='inactive', front=1., rear=1.)
        if (not valid or not .04 <= dt <= .25 or abs(lateral_velocity) > .15):
            return dict(valid=False, reason='feedback_unqualified', front=1., rear=1.)
        self.last_ns = stamp_ns
        if abs(omega) < self.minimum_omega:
            self.low_yaw_seconds += dt
            # Permit the original acceleration ramp, but not perpetual stalled motion.
            return dict(valid=self.low_yaw_seconds <= .75,
                        reason='settling_low_yaw' if self.low_yaw_seconds <= .75 else 'yaw_stalled',
                        front=self.front, rear=2-self.front)
        if omega*requested_angular <= 0:
            return dict(valid=False, reason='wrong_yaw_direction', front=1., rear=1.)
        self.low_yaw_seconds = 0.
        fraction = 1-math.exp(-dt/self.filter_seconds)
        self.filtered_lateral += fraction*(lateral_velocity-self.filtered_lateral)
        # Positive v_y / omega corresponds to a rearward instantaneous pivot.
        # Decrease front/rear allocation difference until measured v_y approaches 0.
        rate = float(np.clip(-self.gain*self.filtered_lateral/omega,
                             -self.maximum_rate, self.maximum_rate))
        before = self.front
        self.front = float(np.clip(before+rate*dt, .95, 1.05))
        return dict(valid=True, reason='adaptive', front=self.front, rear=2-self.front,
                    filtered_lateral_m_s=self.filtered_lateral,
                    pivot_x_estimate_m=-self.filtered_lateral/omega,
                    rate_ratio_s=rate, saturated=self.front in (.95, 1.05))


def wheel_allocation(native, front, fresh):
    targets = np.asarray(native, float)
    if targets.shape != (4,) or not np.isfinite(targets).all():
        raise ValueError('Missing native wheel targets')
    if abs(targets[0]-targets[2]) > 1e-12 or abs(targets[1]-targets[3]) > 1e-12:
        raise ValueError('Unexpected DiffDrive front/rear ordering')
    pure = abs(targets[0]+targets[1]) <= 1e-12 and abs(targets[0]-targets[1]) > 1e-12
    if not pure:
        return targets.copy(), 'inactive'
    if not fresh or not math.isfinite(front) or not .95 <= front <= 1.05:
        return np.zeros(4), 'inhibited'
    return targets*np.array([front, front, 2-front, 2-front]), 'adaptive'
