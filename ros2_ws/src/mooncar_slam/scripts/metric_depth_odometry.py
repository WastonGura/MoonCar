"""Measured depth surface registration with IMU attitude and bounded keyframes.

All points are sensor measurements in a gravity-aligned odom orientation, about
the current body origin. Wheel velocity only predicts a correspondence search.
Planar/degenerate geometry cannot create an accepted XY observation.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.stats import chi2


def project_depth(data, width, height, step, encoding, bigendian, intrinsics,
                  stride=4, pixel_offset=.5):
    if width <= 0 or height <= 0 or stride < 1:
        raise ValueError('Invalid image dimensions or stride')
    if encoding not in ('32FC1', '16UC1'):
        raise ValueError('Unsupported depth encoding')
    dtype = np.dtype(('>' if bigendian else '<')+('f4' if encoding == '32FC1' else 'u2'))
    if step < width*dtype.itemsize or len(data) < step*height:
        raise ValueError('Truncated depth image')
    fx, fy, cx, cy = intrinsics
    if not np.isfinite([fx, fy, cx, cy, pixel_offset]).all() or min(fx, fy) <= 0:
        raise ValueError('Invalid camera intrinsics')
    depth = np.ndarray((height, width), dtype=dtype, buffer=data,
                       strides=(step, dtype.itemsize))[::stride, ::stride].astype(float)
    if encoding == '16UC1':
        depth *= .001
    v, u = np.mgrid[0:height:stride, 0:width:stride]
    valid = np.isfinite(depth) & (depth >= .1) & (depth <= 6.)
    z = depth[valid]
    return np.column_stack(((u[valid]+pixel_offset-cx)*z/fx,
                            (v[valid]+pixel_offset-cy)*z/fy, z))


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


class MetricDepthOdometry:
    noise_m = .003  # Engineering uncertainty; real depth noise requires calibration.
    max_frames = 20

    def __init__(self, asynchronous_references=False):
        self.references = []
        self.xyz = np.zeros(3)
        self.yaw = None
        self.stamp = None
        self.last_reference = None
        self.last_reference_view = None
        self.last_reference_overlap = None
        self.reference_number = 0
        self.initialized_moving = False
        self.asynchronous_references = bool(asynchronous_references)
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='measured-depth-reference') if self.asynchronous_references else None
        self._reference_future = None
        self._reference_job_epoch = None
        self._reference_epoch = 0
        self.reference_build_failures = 0
        self.last_reference_build_error = None
        self._closed = False

    def close(self, wait=True):
        self._closed = True
        self._reference_epoch += 1
        if self._executor is not None:
            self._executor.shutdown(wait=wait, cancel_futures=True)
            self._executor = None

    def _reject(self, reason):
        # An intervening measurement failure invalidates a pending handoff. Its
        # immutable worker can finish, but it cannot supply a pose or be adopted.
        self._reference_epoch += 1
        if self._reference_future is not None:
            self._reference_future.cancel()
        return dict(accepted=False, reason=reason)

    @staticmethod
    def _build_reference_job(points, xyz, covariance, parent, yaw, noise_m):
        builder = MetricDepthOdometry()
        builder.yaw, builder.noise_m = yaw, noise_m
        reference = builder._reference(points, xyz, covariance, parent)
        if reference is None:
            return None
        system, _ = builder._system(points, xyz, reference)
        if system is None:
            return None
        return reference, system['overlap'], points.mean(axis=0)

    def _install_reference(self, reference, overlap, view):
        self.references.append(reference)
        if len(self.references) > self.max_frames:
            del self.references[1]
        self.last_reference = (reference['xyz'].copy(), reference['yaw'])
        self.last_reference_view = view.copy()
        self.last_reference_overlap = overlap

    def _adopt_pending_reference(self):
        future = self._reference_future
        if future is None or not future.done():
            return
        self._reference_future = None
        epoch = self._reference_job_epoch
        self._reference_job_epoch = None
        if future.cancelled() or epoch != self._reference_epoch or self._closed:
            return
        try:
            built = future.result()
        except Exception as error:
            self.reference_build_failures += 1
            self.last_reference_build_error = repr(error)
            return
        if built is not None:
            reference, overlap, view = built
            # IDs belong to the main measurement thread, not the worker's local
            # temporary builder. Parents refer only to earlier registered IDs.
            self.reference_number += 1
            reference['id'] = self.reference_number
            self._install_reference(reference, overlap, view)

    def _request_reference(self, points, fitted, yaw):
        if self.asynchronous_references:
            if self._reference_future is None and not self._closed:
                self._reference_job_epoch = self._reference_epoch
                self._reference_future = self._executor.submit(self._build_reference_job,
                    points.copy(), self.xyz.copy(), fitted['covariance'].copy(),
                    int(fitted['reference_id']), float(yaw), float(self.noise_m))
            return
        reference = self._reference(points, self.xyz, fitted['covariance'], fitted['reference_id'])
        if reference is not None:
            system, _ = self._system(points, self.xyz, reference)
            if system is not None:
                self._install_reference(reference, system['overlap'], points.mean(axis=0))

    def _reference(self, points, xyz, covariance, parent):
        world = points+xyz
        _, indices = np.unique(np.floor(world/.02).astype(np.int64), axis=0, return_index=True)
        target = world[np.sort(indices)]
        if len(target) < 80:
            return None
        tree = cKDTree(target)
        # A plane through a neighborhood centroid has a deterministic offset on
        # curved ground. Repeated keyframe handoffs would integrate that offset.
        # Fit the measured local surface once, validating every spatial scale
        # against held-out real returns rather than assuming a terrain shape.
        distances, neighbors = tree.query(target, k=min(512, len(target)))
        count = len(target)
        means, normals = np.zeros_like(target), np.zeros_like(target)
        basis, coefficients = np.zeros((count, 3, 3)), np.zeros((count, 6))
        coefficient_covariance = np.zeros((count, 6, 6))
        valid, support_count = np.zeros(count, dtype=bool), np.zeros(count, dtype=int)
        support_scale = np.zeros(count)
        # Actual spatial support, never synthetic temporal duplicates. Dense near
        # pixels need more neighbors to span a metric patch; sparse far pixels
        # retain smaller supports inside the same original 30 cm radius.
        for scale, k in ((1., 512), (1., 256), (1., 128), (.6, 256), (.6, 128), (.6, 64),
                         (.3, 128), (.3, 96), (.3, 64), (.3, 48), (.3, 32), (.3, 24)):
            if k > neighbors.shape[1]:
                continue
            selected = np.flatnonzero((~valid) & (distances[:, k-1] < scale))
            if not len(selected):
                continue
            patches = target[neighbors[selected, :k]]
            holdout = np.arange(k) % 4 == 0
            train = patches[:, ~holdout]
            center = train.mean(axis=1)
            centered_train = train-center[:, None, :]
            cov = np.einsum('nki,nkj->nij', centered_train, centered_train)/train.shape[1]
            ev, vectors = np.linalg.eigh(cov)
            usable = (ev[:, 1] > .0001) & (ev[:, 0] < .15*ev[:, 1])
            axes = vectors[:, :, [2, 1, 0]]
            local = np.einsum('nki,nij->nkj', patches-center[:, None, :], axes)
            u, v, w = local[:, :, 0]/scale, local[:, :, 1]/scale, local[:, :, 2]
            design = np.stack((u*u, u*v, v*v, u, v, np.ones_like(u)), axis=-1)
            training_design = design[:, ~holdout]
            training_information = np.einsum('nki,nkj->nij', training_design, training_design)
            spectrum = np.linalg.eigvalsh(training_information)
            # Two scan rows can appear 2D to PCA yet leave quadratic curvature
            # essentially unconstrained. Refuse those fits rather than allowing
            # an arbitrarily large gradient covariance to swamp all good patches.
            usable &= spectrum[:, 0] > spectrum[:, -1]*1e-6
            if not usable.any():
                continue
            candidate = np.flatnonzero(usable)
            training_inverse = np.linalg.inv(training_information[candidate])
            training_rhs = np.einsum('nki,nk->ni', training_design[candidate], w[candidate][:, ~holdout])
            training_coefficient = np.einsum('nij,nj->ni', training_inverse, training_rhs)
            training_residual = w[candidate][:, ~holdout]-np.einsum('nki,ni->nk', training_design[candidate], training_coefficient)
            variance = np.maximum(self.noise_m**2,
                np.sum(training_residual**2, axis=1)/(training_design.shape[1]-6))
            training_covariance = variance[:, None, None]*training_inverse
            validation_design = design[candidate][:, holdout]
            validation_residual = w[candidate][:, holdout]-np.einsum('nki,ni->nk', validation_design, training_coefficient)
            prediction_variance = np.einsum('nki,nij,nkj->nk', validation_design, training_covariance, validation_design)
            # A larger support is permitted only when held-out measured returns
            # agree with the local surface at the unchanged sensor-noise scale.
            # No world shape, extrapolated fill, or pose truth is supplied.
            statistic = np.sum(validation_residual**2/(self.noise_m**2+prediction_variance), axis=1)
            usable[candidate[statistic > chi2.ppf(.99, int(holdout.sum()))]] = False
            if not usable.any():
                continue
            chosen = selected[usable]
            inverse = np.linalg.inv(np.einsum('nki,nkj->nij', design[usable], design[usable]))
            means[chosen], normals[chosen], basis[chosen] = center[usable], vectors[usable, :, 0], axes[usable]
            rhs = np.einsum('nki,nk->ni', design[usable], w[usable])
            coefficients[chosen] = np.einsum('nij,nj->ni', inverse, rhs)
            fit_residual = w[usable]-np.einsum('nki,ni->nk', design[usable], coefficients[chosen])
            variance = np.maximum(self.noise_m**2, np.sum(fit_residual**2, axis=1)/(k-6))
            coefficient_covariance[chosen] = variance[:, None, None]*inverse
            support_count[chosen], valid[chosen] = k, True
            support_scale[chosen] = scale
        if valid.sum() < 60:
            return None
        self.reference_number += 1
        radii, radius_counts = np.unique(support_scale[valid], return_counts=True)
        summary = dict(valid_patches=int(valid.sum()), target_points=len(target),
            support_radius_m={str(float(radius)): int(number) for radius, number in zip(radii, radius_counts)},
            minimum_support_points=int(support_count[valid].min()),
            maximum_support_points=int(support_count[valid].max()),
            minimum_sensor_noise_m=self.noise_m, holdout_fraction=.25, holdout_chi_square_confidence=.99)
        return dict(id=self.reference_number, parent=parent, xyz=xyz.copy(), yaw=self.yaw,
                    target=target, tree=tree, means=means, normals=normals, valid=valid,
                    surface_basis=basis, surface_coefficients=coefficients, surface_scale=support_scale,
                    surface_support_count=support_count,
                    support_summary=summary,
                    surface_coefficient_covariance=coefficient_covariance,
                    anchor_covariance=covariance.copy())

    def _system(self, points, xyz, reference):
        moved = points+xyz
        distance, index = reference['tree'].query(moved, k=1)
        valid = (distance < .15) & reference['valid'][index]
        selected = index[valid]
        basis = reference['surface_basis'][selected]
        local = np.einsum('ni,nij->nj', moved[valid]-reference['means'][selected], basis)
        scale = reference['surface_scale'][selected]
        u, v, w = local[:, 0]/scale, local[:, 1]/scale, local[:, 2]
        coefficients = reference['surface_coefficients'][selected]
        design = np.column_stack((u*u, u*v, v*v, u, v, np.ones_like(u)))
        fitted = np.sum(design*coefficients, axis=1)
        target_local = np.einsum('ni,nij->nj', reference['target'][selected]-reference['means'][selected], basis)
        tu, tv = target_local[:, 0]/scale, target_local[:, 1]/scale
        target_design = np.column_stack((tu*tu, tu*tv, tv*tv, tu, tv, np.ones_like(tu)))
        target_residual = target_local[:, 2]-np.sum(target_design*coefficients, axis=1)
        du = (2*coefficients[:, 0]*u+coefficients[:, 1]*v+coefficients[:, 3])/scale
        dv = (coefficients[:, 1]*u+2*coefficients[:, 2]*v+coefficients[:, 4])/scale
        n = basis[:, :, 2]-du[:, None]*basis[:, :, 0]-dv[:, None]*basis[:, :, 1]
        length = np.linalg.norm(n, axis=1)
        n /= length[:, None]
        # Curvature corrects interpolation between measured returns. A surface
        # approximation may never pull an unchanged measured cloud away from its
        # defining origin just because the smooth polynomial misses that target.
        residual = (w-fitted-target_residual)/length
        du_coeff = np.column_stack((2*u, v, np.zeros_like(u), np.ones_like(u),
                                     np.zeros_like(u), np.zeros_like(u)))/scale[:, None]
        dv_coeff = np.column_stack((np.zeros_like(v), u, 2*v, np.zeros_like(v),
                                     np.ones_like(v), np.zeros_like(v)))/scale[:, None]
        normal_jacobian = (-basis[:, :, 0, None]*du_coeff[:, None, :]
                           -basis[:, :, 1, None]*dv_coeff[:, None, :])
        normal_jacobian -= n[:, :, None]*np.einsum('ni,nik->nk', n, normal_jacobian)[:, None, :]
        normal_jacobian /= length[:, None, None]
        normal_covariance = np.einsum('nik,nkl,njl->nij', normal_jacobian,
            reference['surface_coefficient_covariance'][selected], normal_jacobian)
        difference_design = design-target_design
        reference_variance = (self.noise_m**2+np.einsum('ni,nij,nj->n', difference_design,
            reference['surface_coefficient_covariance'][selected], difference_design))/length**2
        inlier = abs(residual) < .03
        if inlier.sum() < max(60, len(points)*.25) or inlier.sum()*2 < len(residual):
            return None, 'insufficient_depth_overlap'
        n, residual, reference_variance = n[inlier], residual[inlier], reference_variance[inlier]
        normal_covariance = normal_covariance[inlier]
        weight = np.minimum(1., .01/np.maximum(abs(residual), 1e-9))
        # Reference polynomial coefficients came from measured noisy points.
        # Their uncertainty cannot be treated as an exact geometric ground truth.
        weight /= 1.+reference_variance/self.noise_m**2
        information = n.T@(weight[:, None]*n)
        # Eliminate height: a tilted single plane still supplies no XY position.
        if information[2, 2] <= 1e-9:
            return None, 'unobservable_depth_translation'
        xy_information = (information[:2, :2] - np.outer(information[:2, 2], information[2, :2])
                          /information[2, 2])
        projection = np.c_[np.eye(2), -information[:2, 2]/information[2, 2]]
        noise_information = np.einsum('ai,nij,bj,n->ab', projection, normal_covariance, projection, weight)
        # Neighborhoods share their 24 measured points. Do not count their noisy
        # fitted gradients as independent observed curvature on a planar floor.
        # The three-sigma variance margin is conservative for that overlap.
        independent_patches = max(1., len(n)/max(reference['surface_support_count'][selected[inlier]]))
        reliable_xy_information = xy_information-(1.+3.*math.sqrt(2./independent_patches))*noise_information
        eigenvalues = np.linalg.eigvalsh(reliable_xy_information)
        if eigenvalues[0] <= 1e-7 or eigenvalues[0] < .02*eigenvalues[-1]:
            return None, 'unobservable_depth_translation'
        try:
            covariance = self.noise_m**2*np.linalg.inv(information)
            extra_xy = self.noise_m**2*(np.linalg.inv(reliable_xy_information)-np.linalg.inv(xy_information))
            marginal_to_full = np.vstack((np.eye(2), -information[2, :2]/information[2, 2]))
            covariance += marginal_to_full@extra_xy@marginal_to_full.T
        except np.linalg.LinAlgError:
            return None, 'unobservable_depth_translation'
        if (not np.isfinite(covariance).all() or np.linalg.eigvalsh(covariance).min() <= 0
                or np.linalg.eigvalsh(covariance[:2, :2]).max() > .05**2):
            return None, 'uncertain_depth_translation'
        return dict(information=information, residual=residual, normals=n, weights=weight,
                    covariance=covariance, inliers=len(residual), overlap=len(residual)/len(points)), 'accepted'

    def _fit(self, points, prediction, reference):
        xyz = prediction.copy()
        for _ in range(15):
            system, reason = self._system(points, xyz, reference)
            if system is None:
                return None, reason
            step = -np.linalg.solve(system['information'],
                                    system['normals'].T@(system['weights']*system['residual']))
            xyz += step
            if np.linalg.norm(xyz-prediction) > .1:
                return None, 'depth_correction_jump'
            if np.linalg.norm(step) < 1e-5:
                break
        system, reason = self._system(points, xyz, reference)
        if system is None:
            return None, reason
        rmse = float(np.sqrt(np.mean(system['residual']**2)))
        if rmse > .015:
            return None, 'depth_registration_residual'
        return dict(xyz=xyz, covariance=system['covariance']+reference['anchor_covariance'],
                    measurement_covariance=system['covariance'], inliers=system['inliers'],
                    overlap=system['overlap'], rmse_m=rmse, reference_id=reference['id'],
                    reference_support=reference['support_summary']), 'accepted'

    def update(self, stamp, points, yaw, wheel_vx, wheel_wz):
        if self._closed:
            return self._reject('depth_core_closed')
        points = np.asarray(points, dtype=float)
        if (not np.isfinite([stamp, yaw, wheel_vx, wheel_wz]).all() or points.ndim != 2
                or points.shape[1] != 3 or not np.isfinite(points).all()):
            return self._reject('invalid_depth_input')
        if len(points) < 100:
            return self._reject('too_few_depth_returns')
        if self.stamp is not None and stamp <= self.stamp:
            return self._reject('nonincreasing_depth_stamp')
        moving = abs(wheel_vx) > .005 or abs(wheel_wz) > .03
        if not self.references and moving:
            self.initialized_moving = True
        if self.initialized_moving:
            return self._reject('initialization_motion_requires_new_session')
        dt = 0. if self.stamp is None else stamp-self.stamp
        if dt > 1.:
            return self._reject('depth_gap_requires_new_session')
        prediction = self.xyz.copy()
        if self.yaw is not None:
            prediction[:2] += rotation(self.yaw)@np.array([wheel_vx*dt, 0.])
        self.stamp, self.yaw = float(stamp), float(yaw)
        if not self.references:
            reference = self._reference(points, self.xyz, np.zeros((3, 3)), None)
            if reference is None:
                return self._reject('insufficient_depth_surface')
            system, reason = self._system(points, self.xyz, reference)
            if system is None:
                return self._reject(reason)
            self.references.append(reference)
            self.last_reference = (self.xyz.copy(), yaw)
            self.last_reference_view = points.mean(axis=0)
            self.last_reference_overlap = system['overlap']
            return dict(accepted=True, reason='initialized', stamp=stamp, yaw=yaw,
                        xyz=self.xyz.copy(), covariance=system['covariance'],
                        inliers=system['inliers'], rmse_m=0., reference_id=reference['id'],
                        reference_frames=len(self.references), reference_support=reference['support_summary'])
        # Fixed origin gets priority; registered local references bridge its range.
        candidates = [self.references[0]]+sorted(self.references[1:],
                      key=lambda r: np.linalg.norm(r['xyz']-prediction))
        # Retain the origin anchor, but do not prefer its weak sliver of overlap
        # over a more informative already registered view. Compare conservative
        # predicted covariance for the anchor and nearby measured references.
        scored = []
        for order, reference in enumerate(candidates[:4]):
            system, _ = self._system(points, prediction, reference)
            score = (np.linalg.eigvalsh((system['covariance']+reference['anchor_covariance'])[:2, :2]).max()
                     if system is not None else math.inf)
            scored.append((score, order, reference))
        candidates = [row[2] for row in sorted(scored, key=lambda row: (row[0], row[1]))]+candidates[4:]
        fitted, reason = None, 'insufficient_depth_overlap'
        for reference in candidates:
            fitted, reason = self._fit(points, prediction, reference)
            if fitted is not None:
                break
        if fitted is None:
            self.xyz = prediction
            return self._reject(reason)
        self.xyz = fitted['xyz'].copy()
        # A completed worker is committed only here, at a genuinely accepted new
        # measurement. The returned pose remains this fit, never the worker pose.
        self._adopt_pending_reference()
        position, heading = self.last_reference
        turn = abs(math.atan2(math.sin(yaw-heading), math.cos(yaw-heading)))
        view_shift = np.linalg.norm(points.mean(axis=0)-self.last_reference_view)
        new_measured_view = (view_shift >= .3 and fitted['overlap'] < self.last_reference_overlap)
        if (moving and (np.linalg.norm(self.xyz-position) >= .3 or turn >= .25)) or new_measured_view:
            self._request_reference(points, fitted, yaw)
        return dict(accepted=True, reason='accepted', stamp=stamp, yaw=yaw,
                    reference_frames=len(self.references), reference_pending=self._reference_future is not None,
                    reference_build_failures=self.reference_build_failures, **fitted)
