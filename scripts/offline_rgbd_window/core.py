"""Offline, bounded RGB-D landmark/window prototype.

This is a first-order MAP model, not nonlinear BA or tightly coupled VIO.
For each frame it estimates translation p, a *body-local* small attitude
correction a, and one camera-local common measurement error b.  A track has
one shared world landmark L.  With q = R_bc z + t_bc, its residual is

    p + R_wb q - R_wb [q]x a + R_wb R_bc b - L.

The attitude and common error have Gaussian priors; neither is averaged as
independent pixel noise.  Independent point covariance is supplied in camera
coordinates.  Attitude priors are independent between frames in this explicit
prototype model; unknown AHRS temporal correlations are NOT calibrated here.

Old frame factors and the previous information prior are Schur-marginalized.
Retained frame factors are not folded into that prior a second time.  The
nominal rotations/linearization stay fixed, so its dense cross blocks exactly
retain correlation for this linear model.  Reported C is a conditional,
linearized model covariance, with a 3 mm minimum eigenvalue applied only when
missing.  It is not a claim of field-calibrated navigation accuracy.

Associations must already be verified by the caller.  No truth, map, wheel
measurement, pose averaging, re-identification, or covariance reset is used.
"""

from dataclasses import dataclass
import hashlib
import math
import numbers

import numpy as np
from scipy import sparse
from scipy.sparse.linalg import splu


@dataclass(frozen=True)
class Observation:
    track_id: int
    point_camera: np.ndarray
    covariance_camera: np.ndarray


@dataclass(frozen=True)
class Frame:
    stamp_ns: int
    nominal_R_wb: np.ndarray
    R_bc: np.ndarray
    t_bc: np.ndarray
    observations: list
    attitude_cov: np.ndarray


@dataclass(frozen=True)
class Estimate:
    position: np.ndarray
    covariance: np.ndarray
    diagnostics: dict


class WindowError(ValueError):
    """Invalid/disconnected observation or a failed model solve; no reset."""


@dataclass(eq=False)
class _CommittedPosterior:
    """One successful commit's local solve, never persistent estimator state."""
    _owner: object
    _signature: object
    _keys: tuple
    _information: object
    _eta: np.ndarray
    _mean: np.ndarray
    _decomposition: object
    _consumed: bool = False

    def _take(self, estimator):
        if self._consumed:
            raise WindowError("committed posterior already consumed")
        if self._owner is not estimator:
            raise WindowError("committed posterior owner mismatch")
        try:
            signature = estimator._posterior_signature()
        except (AttributeError, TypeError, ValueError) as error:
            raise WindowError("stale or mutated committed posterior") from error
        if self._signature != signature:
            raise WindowError("stale or mutated committed posterior")
        self._consumed = True
        answer = (self._keys, self._information, self._eta, self._mean,
                  self._decomposition)
        self._owner = self._information = self._eta = None
        self._mean = self._decomposition = None
        return answer


def _skew(v):
    x, y, z = v
    return np.array(((0., -z, y), (z, 0., -x), (-y, x, 0.)))


def _array(value, shape, label):
    answer = np.asarray(value, dtype=float)
    if answer.shape != shape or not np.isfinite(answer).all():
        raise WindowError("invalid " + label)
    return answer.copy()


def _rotation(value, label):
    answer = _array(value, (3, 3), label)
    if (not np.allclose(answer.T @ answer, np.eye(3), atol=1e-7, rtol=0.)
            or not math.isclose(float(np.linalg.det(answer)), 1., abs_tol=1e-7)):
        raise WindowError("invalid rotation " + label)
    return answer


def _covariance(value, label):
    answer = _array(value, (3, 3), label)
    if not np.allclose(answer, answer.T, atol=1e-12, rtol=1e-7):
        raise WindowError("asymmetric " + label)
    try:
        np.linalg.cholesky(answer)
    except np.linalg.LinAlgError as error:
        raise WindowError("nonpositive " + label) from error
    return answer


class WindowEstimator:
    MAX_OBSERVATIONS_PER_FRAME = 96
    MAX_FRAMES = 12
    MAX_SMALL_ANGLE_RAD = math.radians(3.)
    OUTPUT_SIGMA_FLOOR_M = .003

    def __init__(self, max_frames=6, shared_calibration_sigma_m=.003):
        if (isinstance(max_frames, bool) or not isinstance(max_frames, numbers.Integral)
                or not 2 <= max_frames <= self.MAX_FRAMES):
            raise WindowError("max_frames must be between 2 and 12")
        if (not np.isfinite(shared_calibration_sigma_m)
                or shared_calibration_sigma_m <= 0):
            raise WindowError("common measurement sigma must be positive")
        self.max_frames = int(max_frames)
        # API spelling retained as agreed.  This is a per-exposure shared
        # camera-coordinate error, not a calibrated all-time sensor bias.
        self.shared_calibration_sigma_m = float(shared_calibration_sigma_m)
        self._frames = []
        self._first_stamp = None
        self._last_stamp = None
        self._prior_keys = []
        self._prior_H = np.empty((0, 0))
        self._prior_eta = np.empty(0)
        self._solution = {}
        self._total_frames = 0
        self._marginalized_frames = 0
        # Owned frames have fixed nominal linearization. Cache only their
        # unchanged factor coefficients, never an approximate solution/prior.
        self._frame_terms = {}

    def _checked_frame(self, frame):
        if not isinstance(frame, Frame):
            raise WindowError("expected Frame")
        if (isinstance(frame.stamp_ns, bool)
                or not isinstance(frame.stamp_ns, numbers.Integral)
                or frame.stamp_ns < 0
                or (self._last_stamp is not None and frame.stamp_ns <= self._last_stamp)):
            raise WindowError("nonincreasing or invalid exposure stamp")
        if not 3 <= len(frame.observations) <= self.MAX_OBSERVATIONS_PER_FRAME:
            raise WindowError("expected 3..96 verified observations per frame")
        observations, seen = [], set()
        for observation in frame.observations:
            if (not isinstance(observation, Observation)
                    or isinstance(observation.track_id, bool)
                    or not isinstance(observation.track_id, numbers.Integral)
                    or observation.track_id < 0 or observation.track_id in seen):
                raise WindowError("invalid or duplicate track ID")
            seen.add(int(observation.track_id))
            z = _array(observation.point_camera, (3,), "camera point")
            if z[2] <= 0:
                raise WindowError("camera point must be in front of camera")
            observations.append(Observation(int(observation.track_id), z,
                _covariance(observation.covariance_camera, "point covariance")))
        if self._frames:
            existing = {o.track_id for old in self._frames for o in old.observations}
            if len(existing & seen) < 3:
                raise WindowError("fewer than three existing verified landmark tracks")
        return Frame(int(frame.stamp_ns), _rotation(frame.nominal_R_wb, "R_wb"),
            _rotation(frame.R_bc, "R_bc"), _array(frame.t_bc, (3,), "t_bc"),
            observations, _covariance(frame.attitude_cov, "attitude covariance"))

    def _keys(self):
        keys = []
        for frame in self._frames:
            if frame.stamp_ns != self._first_stamp:
                keys.append(("p", frame.stamp_ns))
            keys.extend((("a", frame.stamp_ns), ("b", frame.stamp_ns)))
        landmarks = {o.track_id for frame in self._frames for o in frame.observations}
        keys.extend(("l", number) for number in sorted(landmarks))
        return keys

    @staticmethod
    def _factor(frame, observation, first_stamp):
        R, camera_R = frame.nominal_R_wb, frame.nominal_R_wb @ frame.R_bc
        q = frame.R_bc @ observation.point_camera + frame.t_bc
        blocks = {("a", frame.stamp_ns): -R @ _skew(q),
                  ("b", frame.stamp_ns): camera_R,
                  ("l", observation.track_id): -np.eye(3)}
        if frame.stamp_ns != first_stamp:
            blocks[("p", frame.stamp_ns)] = np.eye(3)
        W = camera_R @ np.linalg.inv(observation.covariance_camera) @ camera_R.T
        return blocks, R @ q, W

    def _compiled_frame(self, frame):
        cached = self._frame_terms.get(frame.stamp_ns)
        if cached is not None and cached[0] is frame:
            return cached[1]
        factors = [self._factor(frame, observation, self._first_stamp)
                   for observation in frame.observations]
        keys = list(dict.fromkeys(key for blocks, _, _ in factors for key in blocks))
        index = {key: 3*i for i, key in enumerate(keys)}
        lefts, rights, values, eta_at, eta_values = [], [], [], [], []

        def block(left, right, value):
            lefts.append(index[left]); rights.append(index[right])
            values.append(value.ravel())

        block(("a", frame.stamp_ns), ("a", frame.stamp_ns),
              np.linalg.inv(frame.attitude_cov))
        block(("b", frame.stamp_ns), ("b", frame.stamp_ns),
              np.eye(3) / self.shared_calibration_sigma_m**2)
        for blocks, constant, W in factors:
            for left, J in blocks.items():
                # Retain contribution order for the information vector.
                eta_at.extend((index[left], index[left]+1, index[left]+2))
                eta_values.extend(-(J.T @ W @ constant))
                for right, K in blocks.items():
                    block(left, right, J.T @ W @ K)
        rows = (np.asarray(lefts)[:, None] + np.repeat(np.arange(3), 3)).ravel()
        columns = (np.asarray(rights)[:, None] + np.tile(np.arange(3), 3)).ravel()
        answer = (keys, rows, columns, np.concatenate(values),
                  np.asarray(eta_at), np.asarray(eta_values), factors)
        self._frame_terms[frame.stamp_ns] = (frame, answer)
        return answer

    def _assemble(self, keys, frames, include_prior=True):
        index = {key: 3*i for i, key in enumerate(keys)}
        rows, columns, values = [], [], []
        eta = np.zeros(3*len(keys))
        if include_prior and self._prior_keys:
            mapping = (np.asarray([index[key] for key in self._prior_keys])[:, None]
                       + np.arange(3)).ravel()
            # Preserve every nonzero cross block of the exact dense Schur
            # prior. Zero entries need no Python block/COO bookkeeping.
            r, c = np.nonzero(self._prior_H)
            rows.append(mapping[r]); columns.append(mapping[c])
            values.append(self._prior_H[r, c])
            eta[mapping] += self._prior_eta
        for frame in frames:
            local, r, c, value, at, rhs, _ = self._compiled_frame(frame)
            mapping = (np.asarray([index[key] for key in local])[:, None]
                       + np.arange(3)).ravel()
            rows.append(mapping[r]); columns.append(mapping[c]); values.append(value)
            np.add.at(eta, mapping[at], rhs)
        H = sparse.coo_matrix((np.concatenate(values),
            (np.concatenate(rows), np.concatenate(columns))),
            shape=(len(eta), len(eta))).tocsc()
        return (H + H.T) * .5, eta

    def _solve(self, posterior_out=None):
        keys = self._keys()
        H, eta = self._assemble(keys, self._frames)
        try:
            decomposition = splu(H, permc_spec="MMD_AT_PLUS_A", diag_pivot_thresh=0.)
            pivots = decomposition.U.diagonal()
            if (not np.isfinite(pivots).all() or np.min(pivots) <= 0
                    or np.min(pivots) <= 1e-12 * np.max(pivots)):
                raise WindowError("rank deficient or ill-conditioned window")
            x = decomposition.solve(eta)
            if not np.isfinite(x).all():
                raise WindowError("nonfinite window solution")
            state = {key: x[3*i:3*i+3].copy() for i, key in enumerate(keys)}
            angles = [np.linalg.norm(state[("a", f.stamp_ns)]) for f in self._frames]
            if max(angles) > self.MAX_SMALL_ANGLE_RAD:
                raise WindowError("attitude correction exceeds first-order 3 degree model")
            current = ("p", self._frames[-1].stamp_ns)
            if current in state:
                at = 3 * keys.index(current)
                unit = np.zeros((len(x), 3)); unit[at:at+3] = np.eye(3)
                C = decomposition.solve(unit)[at:at+3]
                C = (C + C.T) * .5
                if not np.isfinite(C).all() or np.linalg.eigvalsh(C).min() <= 0:
                    raise WindowError("invalid position marginal covariance")
                position = state[current].copy()
            else:
                position, C = np.zeros(3), np.zeros((3, 3))
        except (RuntimeError, np.linalg.LinAlgError) as error:
            raise WindowError("window factorization failed") from error
        residuals = []
        for frame in self._frames:
            for blocks, constant, _ in self._compiled_frame(frame)[-1]:
                residuals.append(constant + sum(J @ state[key] for key, J in blocks.items()))
        diagnostics = dict(
            maximum_attitude_correction_rad=float(max(angles)),
            metric_residual_rms_m=float(np.sqrt(np.mean(np.sum(np.asarray(residuals)**2, axis=1)))),
            state_dimension=len(x), information_nonzeros=H.nnz,
            information_pivot_ratio=float(np.min(pivots) / np.max(pivots)))
        if posterior_out is not None:
            posterior_out.append((tuple(keys), H, eta, x, decomposition))
        return position, C, state, diagnostics

    def _posterior_signature(self):
        """Bind an explicit handoff to the exact committed source it solved."""
        digest = hashlib.sha256()
        def include(array):
            digest.update(repr((array.dtype.str, array.shape)).encode('ascii'))
            digest.update(array.tobytes())
        metadata = (self._first_stamp, self._last_stamp, self._total_frames,
                    self._marginalized_frames, self.max_frames,
                    self.shared_calibration_sigma_m, tuple(self._keys()),
                    tuple(self._prior_keys))
        digest.update(repr(metadata).encode('ascii'))
        for array in (self._prior_H, self._prior_eta):
            include(array)
        for frame in self._frames:
            digest.update(repr((frame.stamp_ns,
                tuple(o.track_id for o in frame.observations))).encode('ascii'))
            for array in (frame.nominal_R_wb, frame.R_bc, frame.t_bc, frame.attitude_cov):
                include(array)
            for observation in frame.observations:
                include(observation.point_camera)
                include(observation.covariance_camera)
        return metadata, digest.digest()

    def _marginalize_oldest(self):
        old, remaining = self._frames[0], self._frames[1:]
        keys = self._keys()
        # Only disappearing factors + old prior enter this Schur complement.
        # All retained-frame observations and their nuisance priors remain live.
        H, eta = self._assemble(keys, [old])
        live_tracks = {o.track_id for frame in remaining for o in frame.observations}
        drop = {key for key in keys if
                (key[0] in ("p", "a", "b") and key[1] == old.stamp_ns)
                or (key[0] == "l" and key[1] not in live_tracks)}
        touched = np.asarray(abs(H).sum(axis=1)).ravel() > 0
        kept_keys = [key for i, key in enumerate(keys)
                     if key not in drop and touched[3*i:3*i+3].any()]
        drop_indices = np.array([3*i+j for i, key in enumerate(keys) if key in drop
                                 for j in range(3)], dtype=int)
        keep_indices = np.array([3*i+j for i, key in enumerate(keys) if key in kept_keys
                                 for j in range(3)], dtype=int)
        Hdd = H[drop_indices][:, drop_indices].tocsc()
        Hdk = H[drop_indices][:, keep_indices].toarray()
        Hkk = H[keep_indices][:, keep_indices].toarray()
        try:
            eliminated = splu(Hdd, permc_spec="MMD_AT_PLUS_A", diag_pivot_thresh=0.)
            prior_H = Hkk - Hdk.T @ eliminated.solve(Hdk)
            prior_eta = eta[keep_indices] - Hdk.T @ eliminated.solve(eta[drop_indices])
        except RuntimeError as error:
            raise WindowError("marginalization failed") from error
        self._prior_keys = kept_keys
        self._prior_H = (prior_H + prior_H.T) * .5
        self._prior_eta = prior_eta
        self._frames = remaining
        self._solution = {key: value for key, value in self._solution.items() if key not in drop}
        self._marginalized_frames += 1

    def query_frame(self, frame):
        """Solve one measured exposure without retaining it or its factors.

        This is the same joint MAP/marginal as ``add_frame``, including exact
        temporary Schur elimination when the persistent window is full. No
        query observation, pose, covariance, stamp or factor cache survives.
        The caller serializes access, as it does for ``add_frame``.
        """
        persistent = self.__dict__.copy()
        try:
            # add_frame appends before it replaces the window during Schur.
            # Protect that original list as well as all replaced state fields.
            self._frames = self._frames.copy()
            return self.add_frame(frame)
        finally:
            self.__dict__.clear()
            self.__dict__.update(persistent)

    def add_frame(self, frame):
        """Add a verified measurement transaction, or raise without state change."""
        return self._add_frame(frame, False)

    def add_frame_with_posterior(self, frame):
        """Explicit cached-query commit; return its one-use solve handoff too.

        The normal add/query API and persistent state keep no native LU. The
        caller must serialize the immediate cache build, as for add_frame.
        """
        return self._add_frame(frame, True)

    def _add_frame(self, frame, retain_posterior):
        checked = self._checked_frame(frame)
        snapshot = (list(self._frames), self._first_stamp, self._last_stamp,
                    list(self._prior_keys), self._prior_H, self._prior_eta,
                    self._solution, self._total_frames, self._marginalized_frames,
                    self._frame_terms)
        try:
            self._frame_terms = self._frame_terms.copy()
            if self._first_stamp is None:
                self._first_stamp = checked.stamp_ns
            self._frames.append(checked)
            if len(self._frames) > self.max_frames:
                # This model has fixed nominal linearization, so elimination
                # before solving is exact and bounds the solve to max_frames.
                self._marginalize_oldest()
            posterior_out = [] if retain_posterior else None
            position, C, self._solution, diagnostics = (self._solve(posterior_out)
                if retain_posterior else self._solve())
            self._last_stamp = checked.stamp_ns
            self._total_frames += 1
            minimum = float(np.linalg.eigvalsh(C).min())
            floor_added = max(0., self.OUTPUT_SIGMA_FLOOR_M**2 - minimum)
            C = C + floor_added * np.eye(3)
            off_diagonal = self._prior_H.copy()
            for i in range(len(self._prior_keys)):
                off_diagonal[3*i:3*i+3, 3*i:3*i+3] = 0.
            diagnostics.update(model="first_order_rgbd_landmark_map",
                covariance_model="conditional_linearized_marginal_with_minimum_floor",
                attitude_temporal_correlation="independent_priors_assumption",
                frame_count=len(self._frames), landmark_count=sum(k[0] == "l" for k in self._keys()),
                observation_count=sum(len(f.observations) for f in self._frames),
                total_frames=self._total_frames, marginalized_frames=self._marginalized_frames,
                prior_dimension=len(self._prior_eta),
                prior_cross_information_norm=float(np.linalg.norm(off_diagonal)),
                shared_calibration_sigma_m=self.shared_calibration_sigma_m,
                output_floor_added_variance_m2=floor_added,
                position_xy_sigma_max_m=float(np.sqrt(np.linalg.eigvalsh(C[:2, :2]).max())))
            self._frame_terms = {f.stamp_ns: self._frame_terms[f.stamp_ns]
                                 for f in self._frames}
            answer = Estimate(position.copy(), C.copy(), diagnostics)
            if retain_posterior:
                keys, H, eta, mean, decomposition = posterior_out[0]
                handoff = _CommittedPosterior(self, self._posterior_signature(),
                    keys, H, eta, mean, decomposition)
                return answer, handoff
            return answer
        except Exception:
            (self._frames, self._first_stamp, self._last_stamp, self._prior_keys,
             self._prior_H, self._prior_eta, self._solution, self._total_frames,
             self._marginalized_frames, self._frame_terms) = snapshot
            raise
