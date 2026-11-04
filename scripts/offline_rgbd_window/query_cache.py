"""Independent offline queries against a fixed, correlated linear MAP window.

The legacy ``WindowEstimator.query_frame`` remains the reference. This module
does not submit query factors, change gauges, average landmark uncertainties,
or replace the production/online estimator. Its 9D conditioning diagnostics
are not the legacy full-window LU pivot test. Offline acceptance comparisons
must check the reference's accepted/rejected status separately.
"""
from dataclasses import dataclass, field
from itertools import islice
import copy
import numbers
from types import MappingProxyType
import threading
import time

import numpy as np
from scipy.linalg import cho_factor, cho_solve
from scipy.sparse.linalg import splu

from .core import (Estimate, Frame, Observation, WindowEstimator, WindowError,
                   _CommittedPosterior, _array, _covariance, _rotation)


def _version(value):
    if isinstance(value, bool) or not isinstance(value, numbers.Integral) or value < 0:
        raise WindowError('cache version must be a nonnegative integer')
    return int(value)


def _readonly(array):
    answer = np.array(array, dtype=float, copy=True)
    answer.setflags(write=False)
    return answer


def _owned_frame(frame):
    return Frame(int(frame.stamp_ns), _readonly(frame.nominal_R_wb),
        _readonly(frame.R_bc), _readonly(frame.t_bc),
        tuple(Observation(int(o.track_id), _readonly(o.point_camera),
            _readonly(o.covariance_camera)) for o in frame.observations),
        _readonly(frame.attitude_cov))


def _storage_bytes(value, seen=None):
    """Array/sparse storage estimate, not a claim of total process RSS."""
    seen = set() if seen is None else seen
    if id(value) in seen:
        return 0
    seen.add(id(value))
    if isinstance(value, np.ndarray):
        return value.nbytes
    if isinstance(value, dict) or isinstance(value, MappingProxyType):
        return sum(_storage_bytes(v, seen) for v in value.values())
    if isinstance(value, (list, tuple)):
        return sum(_storage_bytes(v, seen) for v in value)
    if isinstance(value, (Frame, Observation)):
        return _storage_bytes(vars(value), seen)
    if hasattr(value, 'indptr') and hasattr(value, 'indices') and hasattr(value, 'data'):
        return sum(_storage_bytes(v, seen) for v in (value.data, value.indices, value.indptr))
    return 0


@dataclass(frozen=True, eq=False)
class QueryCache:
    version: int
    gauge_stamp_ns: int
    keyframe_stamps_ns: tuple
    max_frames: int
    shared_calibration_sigma_m: float
    old_window_pivot_ratio: float
    build_wall_s: float
    _frames: tuple = field(repr=False)
    _residual_factors: object = field(repr=False)
    _keys: tuple = field(repr=False)
    _offsets: object = field(repr=False)
    _information: object = field(repr=False)
    _eta: np.ndarray = field(repr=False)
    _mean: np.ndarray = field(repr=False)
    _decomposition: object = field(repr=False)
    _base_bytes: int = field(repr=False)
    _columns: dict = field(default_factory=dict, repr=False)
    _lock: object = field(default_factory=threading.RLock, repr=False)

    @property
    def cached_columns(self):
        with self._lock:
            return 3*len(self._columns)

    def estimated_bytes(self):
        with self._lock:
            return int(self._base_bytes + sum(column.nbytes for column in self._columns.values()))


def freeze_query_cache(estimator, version, posterior=None):
    """Own a versioned fixed-window posterior without changing the estimator.

    The caller increments versions after successful keyframe commits. This
    cache is not linked to later estimator mutations. Selecting an obsolete
    cache requires an explicit old expected_version; no latest-cache fallback
    or implicit refresh occurs here. As with the estimator's other methods,
    the caller serializes commits/mutations during this call.

    An explicit successful-commit handoff transfers that same checked solve;
    it must match this owner and current source and can be consumed once.
    Invalid/obsolete handoffs are rejected, never rebuilt or silently reused.
    """
    version = _version(version)
    if not isinstance(estimator, WindowEstimator) or not estimator._frames:
        raise WindowError('a nonempty committed WindowEstimator is required')
    began = time.perf_counter()
    # Assembly only reads committed factors/prior and may fill _frame_terms.
    # Isolate its mutable containers without recursively copying unrelated
    # solutions or copying the measurements/factors twice. Every array kept
    # by the returned cache is still independently owned below.
    snapshot = copy.copy(estimator)
    snapshot._frames = list(estimator._frames)
    snapshot._frame_terms = dict(estimator._frame_terms)
    keys = tuple(snapshot._keys())
    if posterior is None:
        H, eta = snapshot._assemble(keys, snapshot._frames)
        decomposition, mean = None, None
    else:
        if not isinstance(posterior, _CommittedPosterior):
            raise WindowError('expected a committed posterior handoff')
        solved_keys, H, eta, mean, decomposition = posterior._take(estimator)
        if solved_keys != keys:
            raise WindowError('committed posterior key mismatch')
    try:
        if posterior is None:
            decomposition = splu(H, permc_spec='MMD_AT_PLUS_A', diag_pivot_thresh=0.)
        pivots = decomposition.U.diagonal()
        if (not np.isfinite(pivots).all() or np.min(pivots) <= 0
                or np.min(pivots) <= 1e-12*np.max(pivots)):
            raise WindowError('rank deficient or ill-conditioned cached window')
        if posterior is None:
            mean = decomposition.solve(eta)
        if not np.isfinite(mean).all():
            raise WindowError('nonfinite cached window mean')
    except (RuntimeError, np.linalg.LinAlgError) as error:
        raise WindowError('cached window factorization failed') from error
    frames = tuple(_owned_frame(f) for f in snapshot._frames)
    residual_factors = MappingProxyType({int(f.stamp_ns): tuple(
        (MappingProxyType({key: _readonly(J) for key, J in blocks.items()}),
         _readonly(constant))
        for blocks, constant, _ in snapshot._compiled_frame(f)[-1])
        for f in snapshot._frames})
    eta, mean = _readonly(eta), _readonly(mean)
    for array in (H.data, H.indices, H.indptr):
        array.setflags(write=False)
    # SuperLU keeps native factors; include their exposed sparse storage in
    # the estimate. No dense all-landmark covariance is constructed upfront.
    base_bytes = (_storage_bytes((frames, residual_factors, H, eta, mean))
                  + _storage_bytes((decomposition.L, decomposition.U)))
    return QueryCache(version=version, gauge_stamp_ns=int(snapshot._first_stamp),
        keyframe_stamps_ns=tuple(f.stamp_ns for f in frames), max_frames=snapshot.max_frames,
        shared_calibration_sigma_m=snapshot.shared_calibration_sigma_m,
        old_window_pivot_ratio=float(np.min(pivots)/np.max(pivots)),
        build_wall_s=time.perf_counter()-began, _frames=frames,
        _residual_factors=residual_factors, _keys=keys,
        _offsets=MappingProxyType({key: 3*i for i, key in enumerate(keys)}),
        _information=H, _eta=eta, _mean=mean, _decomposition=decomposition,
        _base_bytes=base_bytes)


def _checked_query(frame, cache):
    checked = _checked_standard_query(frame, cache)
    if checked is not None:
        return checked
    # Reuse every legacy input/association/chronology check on an isolated
    # validator, not on the source estimator or a mutable cached estimator.
    validator = WindowEstimator(cache.max_frames, cache.shared_calibration_sigma_m)
    validator._frames = cache._frames
    validator._first_stamp = cache.gauge_stamp_ns
    validator._last_stamp = cache.keyframe_stamps_ns[-1]
    return validator._checked_frame(frame)


def _checked_standard_query(frame, cache):
    """Batch ordinary f64 point checks; errors use the reference validator.

    The fast path accepts only plain arrays and valid scalar metadata. Any
    nonstandard/invalid observation goes through _checked_frame, preserving
    its original first-error order. Rotations, lever and attitude retain the
    same original helpers even on this path.
    """
    if (type(frame) is not Frame or isinstance(frame.stamp_ns, bool)
            or not isinstance(frame.stamp_ns, numbers.Integral)
            or frame.stamp_ns < 0 or frame.stamp_ns <= cache.keyframe_stamps_ns[-1]
            or type(frame.observations) not in (list, tuple)
            or not 3 <= len(frame.observations) <= WindowEstimator.MAX_OBSERVATIONS_PER_FRAME):
        return None
    numbers_seen = set()
    for observation in frame.observations:
        if (type(observation) is not Observation or isinstance(observation.track_id, bool)
                or not isinstance(observation.track_id, numbers.Integral)
                or observation.track_id < 0 or observation.track_id in numbers_seen):
            return None
        numbers_seen.add(int(observation.track_id))
        for value, shape in ((observation.point_camera, (3,)),
                             (observation.covariance_camera, (3, 3))):
            if (type(value) is not np.ndarray or value.dtype != np.dtype('float64')
                    or value.shape != shape):
                return None
    points = np.stack([o.point_camera for o in frame.observations])
    covariances = np.stack([o.covariance_camera for o in frame.observations])
    if (not np.isfinite(points).all() or (points[:, 2] <= 0).any()
            or not np.isfinite(covariances).all()
            or not np.allclose(covariances, covariances.transpose(0, 2, 1),
                               atol=1e-12, rtol=1e-7)):
        return None
    try:
        np.linalg.cholesky(covariances)
    except np.linalg.LinAlgError:
        return None
    existing = {o.track_id for old in cache._frames for o in old.observations}
    if len(existing & numbers_seen) < 3:
        return None
    observations = [Observation(int(o.track_id), points[i].copy(), covariances[i].copy())
                    for i, o in enumerate(frame.observations)]
    return Frame(int(frame.stamp_ns), _rotation(frame.nominal_R_wb, 'R_wb'),
        _rotation(frame.R_bc, 'R_bc'), _array(frame.t_bc, (3,), 't_bc'),
        observations, _covariance(frame.attitude_cov, 'attitude covariance'))


def _batch_residuals(factors, state):
    """Same factor/block accumulation order with batched small matvecs."""
    factors = tuple(factors)
    items = [tuple(factor[0].items()) for factor in factors]
    total = np.zeros((len(items), 3))
    for slot in range(len(items[0])):
        matrices = np.stack([row[slot][1] for row in items])
        values = np.stack([state[row[slot][0]] for row in items])
        total += (matrices @ values[:, :, None])[:, :, 0]
    return np.stack([factor[1] for factor in factors])+total


def _missing_columns(cache, track_ids):
    missing = [number for number in track_ids if number not in cache._columns]
    pending = {}
    if missing:
        dimension = len(cache._mean)
        indices = [cache._offsets[('l', number)]+j for number in missing for j in range(3)]
        selector = np.zeros((dimension, len(indices)))
        selector[indices, np.arange(len(indices))] = 1.
        values = cache._decomposition.solve(selector)
        if not np.isfinite(values).all():
            raise WindowError('nonfinite selected posterior columns')
        pending = {number: _readonly(values[:, 3*i:3*i+3]) for i, number in enumerate(missing)}
    return pending


def _pending_columns(cache, track_ids):
    pending = _missing_columns(cache, track_ids)
    columns = np.concatenate([cache._columns[number] if number in cache._columns else pending[number]
                              for number in track_ids], axis=1)
    return columns, pending


def prewarm_query_columns(cache, track_ids, expected_version, max_landmarks=96):
    """Prepare bounded derived covariance columns on an unpublished cache.

    Examine only the first max_landmarks candidate IDs, ignoring duplicates
    and IDs absent from this version. No measurement, mean or prior changes.
    The publisher owns this cache until the complete warmup succeeds.
    """
    if not isinstance(cache, QueryCache) or _version(expected_version) != cache.version:
        raise WindowError('query cache version mismatch')
    if type(max_landmarks) is not int or not 0 <= max_landmarks <= 96:
        raise WindowError('prewarm budget must be between 0 and 96')
    ids = []
    seen = set()
    for value in islice(iter(track_ids), max_landmarks):
        number = _version(value)
        if number not in seen and ('l', number) in cache._offsets:
            ids.append(number)
            seen.add(number)
    began = time.perf_counter()
    with cache._lock:
        pending = {}
        if ids:
            try:
                pending = _missing_columns(cache, ids)
            except (RuntimeError, np.linalg.LinAlgError) as error:
                raise WindowError('cache prewarm factorization failed') from error
        cache._columns.update(pending)
    return dict(wall_s=time.perf_counter()-began, selected_landmarks=len(ids),
                added_columns=3*len(pending), cache_bytes=cache.estimated_bytes())


def query_cached(frame, cache, expected_version):
    """Current measured 9D MAP against correlated landmark marginals.

    Successful queries may memoize derived H^-1 columns; rejected queries do
    not. No query observation or state is committed. Ordinary outputs retain
    the caller's actual exposure stamp and must not be restamped by consumers.
    """
    if not isinstance(cache, QueryCache) or _version(expected_version) != cache.version:
        raise WindowError('query cache version mismatch')
    checked = _checked_query(frame, cache)
    with cache._lock:
        try:
            return _query_checked(checked, cache)
        except (RuntimeError, np.linalg.LinAlgError) as error:
            raise WindowError('cached query factorization failed') from error


def _query_checked(frame, cache):
    existing = [o for o in frame.observations if ('l', o.track_id) in cache._offsets]
    track_ids = [o.track_id for o in existing]
    columns, pending = _pending_columns(cache, track_ids)
    indices = np.array([cache._offsets[('l', number)]+j for number in track_ids for j in range(3)])
    C_LL = columns[indices]
    C_LL = (C_LL+C_LL.T)*.5
    mu_L = cache._mean[indices]
    A = np.zeros((len(indices), 9))
    constant, Sigma = np.zeros(len(indices)), C_LL.copy()
    current_keys = [('p', frame.stamp_ns), ('a', frame.stamp_ns), ('b', frame.stamp_ns)]
    current_factors = {}
    for observation in frame.observations:
        current_factors[observation.track_id] = WindowEstimator._factor(frame, observation, cache.gauge_stamp_ns)
    for i, observation in enumerate(existing):
        blocks, c, W = current_factors[observation.track_id]
        at = slice(3*i, 3*i+3)
        A[at] = np.column_stack([blocks[key] for key in current_keys])
        constant[at] = c
        # Invert the actual legacy W; do not assume the checked nominal
        # rotation is arithmetically perfectly orthogonal.
        Sigma[at, at] += np.linalg.inv(W)
    Sigma = (Sigma+Sigma.T)*.5
    if not np.isfinite(Sigma).all():
        raise WindowError('nonfinite correlated observation covariance')
    sigma_factor = cho_factor(Sigma, lower=True, check_finite=False)
    inverse_A = cho_solve(sigma_factor, A, check_finite=False)
    N = A.T@inverse_A
    N[3:6, 3:6] += np.linalg.inv(frame.attitude_cov)
    N[6:9, 6:9] += np.eye(3)/cache.shared_calibration_sigma_m**2
    N = (N+N.T)*.5
    rhs = A.T@cho_solve(sigma_factor, mu_L-constant, check_finite=False)
    if not np.isfinite(N).all() or not np.isfinite(rhs).all():
        raise WindowError('nonfinite current query system')
    factor = cho_factor(N, lower=True, check_finite=False)
    y = cho_solve(factor, rhs, check_finite=False)
    C_y = cho_solve(factor, np.eye(9), check_finite=False)
    C_y = (C_y+C_y.T)*.5
    if not np.isfinite(y).all() or not np.isfinite(C_y).all():
        raise WindowError('nonfinite current query solution')
    updated_mean = cache._mean + columns@cho_solve(sigma_factor,
        A@y+constant-mu_L, check_finite=False)
    retained = cache._frames if len(cache._frames) < cache.max_frames else cache._frames[1:]
    angles = [float(np.linalg.norm(updated_mean[cache._offsets[('a', f.stamp_ns)]:
        cache._offsets[('a', f.stamp_ns)]+3])) for f in retained]
    angles.append(float(np.linalg.norm(y[3:6])))
    if max(angles) > WindowEstimator.MAX_SMALL_ANGLE_RAD:
        raise WindowError('attitude correction exceeds first-order 3 degree model')
    C = C_y[:3, :3].copy()
    if not np.isfinite(C).all() or np.linalg.eigvalsh(C).min() <= 0:
        raise WindowError('invalid position marginal covariance')
    floor_added = max(0., WindowEstimator.OUTPUT_SIGMA_FLOOR_M**2-float(np.linalg.eigvalsh(C).min()))
    C += floor_added*np.eye(3)

    # Recover means only for residual diagnostics, including each new landmark
    # with its one free observation. These points add no information to y.
    state = {key: updated_mean[3*i:3*i+3] for i, key in enumerate(cache._keys)}
    state.update({key: y[3*i:3*i+3] for i, key in enumerate(current_keys)})
    for observation in frame.observations:
        key = ('l', observation.track_id)
        if key not in cache._offsets:
            blocks, c, _ = current_factors[observation.track_id]
            state[key] = c+sum(blocks[k]@state[k] for k in current_keys)
    residuals = np.concatenate([
        *(_batch_residuals(cache._residual_factors[old.stamp_ns], state) for old in retained),
        _batch_residuals(current_factors.values(), state)])
    if not np.isfinite(updated_mean).all() or not np.isfinite(residuals).all():
        raise WindowError('nonfinite back-substituted query state')
    eigenvalues = np.linalg.eigvalsh(N)
    if eigenvalues.min() <= 0:
        raise WindowError('nonpositive current query information')
    # Only derived columns become persistent, after every solve/angle/C gate.
    cache._columns.update(pending)
    diagnostics = dict(model='first_order_rgbd_landmark_map',
        covariance_model='conditional_linearized_marginal_with_minimum_floor',
        attitude_temporal_correlation='independent_priors_assumption',
        factor_basis='correlated_cached_landmark_marginal_plus_current_9d',
        old_window_pivot_ratio=cache.old_window_pivot_ratio,
        query_information_eigenvalue_ratio=float(eigenvalues.min()/eigenvalues.max()),
        legacy_query_pivot_ratio=None, legacy_query_conditioning_checked=False,
        query_dimension=9, cache_version=cache.version,
        cache_columns=cache.cached_columns, new_columns=3*len(pending), cache_bytes=cache.estimated_bytes(),
        cache_build_wall_s=cache.build_wall_s, stamp_ns=frame.stamp_ns,
        gauge_stamp_ns=cache.gauge_stamp_ns, observed_existing_landmarks=len(existing),
        observed_new_landmarks=len(frame.observations)-len(existing),
        frame_count=len(retained)+1,
        maximum_attitude_correction_rad=max(angles),
        metric_residual_rms_m=float(np.sqrt(np.mean(np.sum(np.asarray(residuals)**2, axis=1)))),
        shared_calibration_sigma_m=cache.shared_calibration_sigma_m,
        output_floor_added_variance_m2=floor_added,
        position_xy_sigma_max_m=float(np.sqrt(np.linalg.eigvalsh(C[:2, :2]).max())))
    return Estimate(y[:3].copy(), C.copy(), diagnostics)
