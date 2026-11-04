"""Sensor-only local metric-support matching with a fresh bounded estimate.

Opt-in research candidate composed from the frozen offline metric-v1 sources.
No truth/map pose, wheel lateral-zero constraint or stale synthetic observation.
The initialized reference remains immutable. Bounded local references extend
coverage only through observable overlap with already registered measurements.
"""
from collections import deque
import math
import numpy as np
from laser_odometry_core import LaserOdometry as BaselineLaserOdometry


class FreshWindowEstimate:
    window_seconds = .4  # Five samples at the declared 10 Hz laser cadence.
    model_threshold = -2 * math.log(.01)  # Chi-square(2), false-motion rate 1%.

    def __init__(self):
        self.rows = deque(maxlen=8)

    def clear(self):
        self.rows.clear()

    def update(self, stamp, xy, covariance):
        xy = np.asarray(xy, dtype=float)
        covariance = np.asarray(covariance, dtype=float)
        if (not np.isfinite(stamp) or xy.shape != (2,) or covariance.shape != (2, 2)
                or not np.isfinite(xy).all() or not np.isfinite(covariance).all()
                or np.linalg.eigvalsh(covariance).min() <= 0):
            raise ValueError('Finite new position and positive covariance required')
        if self.rows and stamp <= self.rows[-1][0]:
            raise ValueError('A new fit with an increasing timestamp is required')
        while self.rows and stamp - self.rows[0][0] > self.window_seconds + 1e-9:
            self.rows.popleft()
        self.rows.append((float(stamp), xy.copy(), covariance.copy()))
        identity = np.eye(2)
        information = np.zeros((2, 2))
        rhs = np.zeros(2)
        linear_information = np.zeros((4, 4))
        linear_rhs = np.zeros(4)
        for t, position, cov in self.rows:
            weight = np.linalg.inv(cov)
            information += weight
            rhs += weight @ position
            design = np.c_[identity, (t - stamp) * identity]
            linear_information += design.T @ weight @ design
            linear_rhs += design.T @ weight @ position
        estimate_cov = np.linalg.inv(information)
        estimate = estimate_cov @ rhs
        model, velocity, improvement = 'constant_position', np.zeros(2), 0.
        if len(self.rows) >= 3 and self.rows[-1][0] - self.rows[0][0] >= .1 - 1e-9:
            linear_covariance = np.linalg.inv(linear_information)
            linear = linear_covariance @ linear_rhs
            velocity = linear[2:]
            static_cost = linear_cost = 0.
            for t, position, cov in self.rows:
                weight = np.linalg.inv(cov)
                static_error = position - estimate
                linear_error = position - linear[:2] - (t - stamp) * velocity
                static_cost += float(static_error @ weight @ static_error)
                linear_cost += float(linear_error @ weight @ linear_error)
            improvement = max(0., static_cost - linear_cost)
            if improvement > self.model_threshold:
                estimate, estimate_cov = linear[:2], linear_covariance[:2, :2]
                model = 'constant_velocity'
            else:
                # Explicit uncertainty for unresolved motion during averaging.
                shift = velocity * (stamp - self.rows[0][0]) / 2
                estimate_cov = estimate_cov + np.outer(shift, shift)
        # A sudden real movement must not be absorbed into old observations.
        if np.linalg.norm(estimate - xy) > 3 * math.sqrt(np.linalg.eigvalsh(covariance).max()):
            estimate, estimate_cov, model = xy.copy(), covariance.copy(), 'current_fit_motion_boundary'
            self.rows.clear()
            self.rows.append((float(stamp), xy.copy(), covariance.copy()))
        return dict(xy=estimate, covariance=estimate_cov, raw_xy=xy,
                    window_samples=len(self.rows), window_span_s=stamp - self.rows[0][0],
                    newest_measurement_stamp=stamp, motion_model=model,
                    velocity=velocity, model_improvement=improvement)


class _MetricNormals(BaselineLaserOdometry):
    # Scan range noise is 1 cm. At least 2.5 sigma of tangent spread prevents
    # fitting a normal to a tiny noise blob; bounded radii never join objects
    # farther apart than the existing 25 cm correspondence neighborhood.
    NOISE_M = .01
    SUPPORT_RADII_M = (.06, .09, .135, .20, .25)
    MIN_TANGENT_STD_M = .025
    MIN_SUPPORT_POINTS = 8

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.normal_means = None
        self.normals = None
        self.normal_valid = None
        self.normal_support_m = None
        self.last_measurement_covariance = None

    def reference(self, world):
        # Prevent repeated observations of the same small surface patch from
        # shrinking the k-nearest support to the measurement-noise scale.
        if self.tree is not None:
            self.last_reference = (self.xy.copy(), self.yaw)
            return
        super().reference(world)
        self.normal_means = np.zeros_like(self.target)
        self.normals = np.zeros_like(self.target)
        self.normal_valid = np.zeros(len(self.target), dtype=bool)
        self.normal_support_m = np.zeros(len(self.target))
        for index, anchor in enumerate(self.target):
            delta = self.target - anchor
            squared_distance = np.sum(delta * delta, axis=1)
            for radius in self.SUPPORT_RADII_M:
                support = self.target[squared_distance <= radius * radius]
                if len(support) < self.MIN_SUPPORT_POINTS:
                    continue
                mean = support.mean(axis=0)
                centered = support - mean
                covariance = centered.T @ centered / len(support)
                eigenvalues, eigenvectors = np.linalg.eigh(covariance)
                if eigenvalues[1] < self.MIN_TANGENT_STD_M ** 2:
                    continue
                if eigenvalues[0] >= .2 * eigenvalues[1]:
                    continue
                self.normal_means[index] = mean
                self.normals[index] = eigenvectors[:, 0]
                self.normal_valid[index] = True
                self.normal_support_m[index] = radius
                break

    def fit(self, world, prediction):
        translation = prediction.copy()
        information = residual = None
        used = 0
        for _ in range(12):
            moved = world + translation
            distances, indices = self.tree.query(moved, k=1)
            valid = (distances < .15) & self.normal_valid[indices]
            n = self.normals[indices[valid]]
            mean = self.normal_means[indices[valid]]
            r = np.sum(n * (moved[valid] - mean), axis=1)
            used = len(r)
            if used < 6:
                return None, 'too_few_line_correspondences'
            weights = np.minimum(1., .02 / np.maximum(abs(r), 1e-9))
            information = n.T @ (weights[:, None] * n)
            eigenvalues = np.linalg.eigvalsh(information)
            if eigenvalues[0] < .02 * eigenvalues[-1]:
                return None, 'unobservable_translation'
            step = -np.linalg.solve(information, n.T @ (weights * r))
            translation += step
            residual = r
            if np.linalg.norm(translation - prediction) > .1:
                return None, 'correction_jump'
            if np.linalg.norm(step) < 1e-5:
                break
        rmse = float(np.sqrt(np.mean(residual ** 2)))
        if rmse > .03:
            return self._rescue_residual_failure(world, prediction, translation)
        self.last_measurement_covariance = max(rmse ** 2, self.NOISE_M ** 2) * np.linalg.inv(information)
        covariance = self.last_measurement_covariance + np.eye(2) * .02 ** 2
        return dict(xy=translation, covariance=covariance,
                    rmse_m=rmse, inliers=used), 'accepted'


class MetricLaserOdometry(_MetricNormals):
    # Reuse the original bounded 20-frame submap and keyframe cadence. These
    # are storage limits, not relaxed registration/observability thresholds.
    MAX_LOCAL_REFERENCES = 20
    MIN_REFERENCE_SCANS = 10  # Same independent-scan count as initial bootstrap.
    _REFERENCE_FIELDS = ('target', 'tree', 'normal_means', 'normals',
                         'normal_valid', 'normal_support_m')

    def __init__(self, *args):
        super().__init__(*args)
        self.fresh_window = FreshWindowEstimate()
        self.local_references = deque(maxlen=self.MAX_LOCAL_REFERENCES)
        self.measured_frames = deque(maxlen=20)
        self.reference_sequence = 0
        self.active_reference_id = 0
        self._pending_reference_stamp = None

    def _using_reference(self, reference, operation):
        """Temporarily select an immutable measured reference, restoring anchor."""
        previous = {name: getattr(self, name) for name in self._REFERENCE_FIELDS}
        try:
            for name in self._REFERENCE_FIELDS:
                setattr(self, name, reference[name])
            return operation()
        finally:
            for name, value in previous.items():
                setattr(self, name, value)

    def _local_model(self, measured):
        """Same metric normal gates, batched to bound keyframe rebuild work."""
        model = _MetricNormals(self.laser_xy, self.laser_yaw)
        BaselineLaserOdometry.reference(model, measured)
        model.normal_means = np.zeros_like(model.target)
        model.normals = np.zeros_like(model.target)
        model.normal_valid = np.zeros(len(model.target), dtype=bool)
        model.normal_support_m = np.zeros(len(model.target))
        for radius in self.SUPPORT_RADII_M:
            queries = np.flatnonzero(~model.normal_valid)
            if not len(queries):
                break
            neighborhoods = model.tree.query_ball_point(model.target[queries], radius, return_sorted=True)
            counts = np.fromiter((len(row) for row in neighborhoods), dtype=int)
            usable = np.flatnonzero(counts >= self.MIN_SUPPORT_POINTS)
            if not len(usable):
                continue
            counts = counts[usable]
            neighbors = np.concatenate([neighborhoods[index] for index in usable])
            starts = np.r_[0, np.cumsum(counts)[:-1]]
            points = model.target[neighbors]
            means = np.add.reduceat(points, starts, axis=0)/counts[:, None]
            centered = points-np.repeat(means, counts, axis=0)
            products = centered[:, :, None]*centered[:, None, :]
            covariance = np.add.reduceat(products, starts, axis=0)/counts[:, None, None]
            eigenvalues, eigenvectors = np.linalg.eigh(covariance)
            valid = ((eigenvalues[:, 1] >= self.MIN_TANGENT_STD_M**2)
                     & (eigenvalues[:, 0] < .2*eigenvalues[:, 1]))
            indices = queries[usable[valid]]
            model.normal_means[indices] = means[valid]
            model.normals[indices] = eigenvectors[valid, :, 0]
            model.normal_valid[indices] = True
            model.normal_support_m[indices] = radius
        return model

    def reference(self, world):
        if self.tree is None:
            return super().reference(world)
        # Only the inherited update(), immediately after an actual accepted fit,
        # may request a keyframe. Manual/stale/stationary calls cannot grow it.
        if (self._pending_reference_stamp != self.stamp
                or len(self.measured_frames) < self.MIN_REFERENCE_SCANS):
            return
        self._pending_reference_stamp = None
        measured = np.concatenate([row['points'] for row in self.measured_frames])
        model = self._local_model(measured)
        reference = {name: getattr(model, name) for name in self._REFERENCE_FIELDS}
        latest = self.measured_frames[-1]
        system, _ = self._using_reference(reference, lambda: self._noise_inlier_system(
            latest['world'], latest['xy']))
        if system is None:
            return
        # Anchoring uncertainty is shared by subsequent fits: it must not shrink
        # as though repeated observations of the same keyframe were independent.
        variance = max(np.linalg.eigvalsh(row['covariance']).max() for row in self.measured_frames)
        self.reference_sequence += 1
        reference.update(id=self.reference_sequence, xy=latest['xy'].copy(),
            anchor_covariance=np.eye(2)*variance,
            parent_ids=tuple(sorted({row['reference_id'] for row in self.measured_frames})),
            first_stamp=self.measured_frames[0]['stamp'], last_stamp=self.measured_frames[-1]['stamp'])
        self.local_references.append(reference)
        self.last_reference = (self.xy.copy(), self.yaw)

    def update(self, *args, **kwargs):
        result = super().update(*args, **kwargs)
        # The inherited input guards can reject before fit() is reached. Old
        # samples must not cross that missing-measurement boundary either.
        if not result['accepted'] and result['reason'] not in ('bootstrap', 'stationary_hold'):
            self.fresh_window.clear()
            self.measured_frames.clear()
            self._pending_reference_stamp = None
        return result

    def fit(self, world, prediction):
        self._pending_reference_stamp = None
        # Initial measured geometry always has priority, including on return.
        # Ordinary anchor-supported updates retain the original exact outputs.
        result, reason = super().fit(world, prediction)
        reference_id, anchor_covariance = 0, np.zeros((2, 2))
        support = None
        if result is not None:
            support, _ = self._noise_inlier_system(world, result['xy'])
            if self.local_references and support is None:
                result, reason = None, 'insufficient_reference_overlap'
        if result is None:
            original_reason = reason
            references = sorted(self.local_references,
                key=lambda row: np.linalg.norm(row['xy']-prediction))
            for reference in references:
                candidate, candidate_reason = self._using_reference(reference,
                    lambda: _MetricNormals.fit(self, world, prediction))
                if candidate is None:
                    continue
                candidate_support, _ = self._using_reference(reference,
                    lambda: self._noise_inlier_system(world, candidate['xy']))
                # Six constraints alone can be a small accidental match in new
                # geometry. A handoff also needs the unchanged majority 3-sigma
                # inlier and two-direction rank checks used by residual rescue.
                if candidate_support is None:
                    continue
                result, reason, support = candidate, candidate_reason, candidate_support
                reference_id = reference['id']
                anchor_covariance = reference['anchor_covariance']
                break
            if result is None:
                reason = original_reason
        if result is None:
            self.fresh_window.clear()
            return result, reason
        if reference_id != self.active_reference_id:
            self.fresh_window.clear()
            self.active_reference_id = reference_id
        measurement_covariance = self.last_measurement_covariance.copy()
        raw_xy = result['xy'].copy()
        filtered = self.fresh_window.update(self.stamp, result['xy'], self.last_measurement_covariance)
        result.update(filtered)
        result['covariance'] = filtered['covariance'] + anchor_covariance + np.eye(2) * .02 ** 2
        result.update(reference_id=reference_id, local_reference_count=len(self.local_references),
                      anchor_covariance=anchor_covariance.copy())
        # Only genuine moving, overlapping, observable scans enter a future
        # local reference. Failed fits and stationary repetition add no geometry.
        if support is not None and self.last_motion == self.stamp:
            self.measured_frames.append(dict(stamp=self.stamp, world=world.copy(), xy=raw_xy,
                points=world+raw_xy, reference_id=reference_id,
                covariance=measurement_covariance+anchor_covariance+np.eye(2)*.02**2))
            self._pending_reference_stamp = self.stamp
        return result, reason

    def _noise_inlier_system(self,world,translation):
        moved=world+translation;distances,indices=self.tree.query(moved,k=1)
        valid=(distances<.15)&self.normal_valid[indices]
        normals=self.normals[indices[valid]];means=self.normal_means[indices[valid]]
        residual=np.sum(normals*(moved[valid]-means),axis=1)
        inlier=abs(residual)<=3*self.NOISE_M
        used=int(inlier.sum())
        if used<6 or 2*used<=len(world):return None,'insufficient_noise_inliers'
        n=normals[inlier];r=residual[inlier]
        weights=np.minimum(1.,.02/np.maximum(abs(r),1e-9));information=n.T@(weights[:,None]*n)
        eigenvalues=np.linalg.eigvalsh(information)
        if eigenvalues[0]<.02*eigenvalues[-1]:return None,'unobservable_translation'
        return dict(normals=n,residual=r,weights=weights,information=information,
                    eigenvalues=eigenvalues,inliers=used,correspondences=len(residual),
                    all_residual=residual),'accepted'

    def _rescue_residual_failure(self,world,prediction,translation):
        system,reason=self._noise_inlier_system(world,translation)
        if system is None:return None,reason
        # One bounded closed-form refit of the selected tangent constraints,
        # after the original (at most twelve) correspondence iterations.
        step=-np.linalg.solve(system['information'],system['normals'].T@(system['weights']*system['residual']))
        translation=translation+step
        if np.linalg.norm(translation-prediction)>.1:return None,'correction_jump'
        system,reason=self._noise_inlier_system(world,translation)
        if system is None:return None,reason
        rmse=float(np.sqrt(np.mean(system['residual']**2)))
        if rmse>.03:return None,'registration_residual'
        self.last_measurement_covariance=max(rmse**2,self.NOISE_M**2)*np.linalg.inv(system['information'])
        return dict(xy=translation,covariance=self.last_measurement_covariance+np.eye(2)*.02**2,
                    rmse_m=rmse,inliers=system['inliers'],
                    discarded_correspondences=system['correspondences']-system['inliers'],
                    usable_returns=len(world),
                    all_correspondence_rmse_m=float(np.sqrt(np.mean(system['all_residual']**2))),
                    information_eigenvalue_ratio=float(system['eigenvalues'][0]/system['eigenvalues'][-1]),
                    rescued_registration=True),'accepted'
