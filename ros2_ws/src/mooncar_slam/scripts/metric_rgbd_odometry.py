"""Metric translation from identified RGB features and measured aligned depth.

The caller supplies synchronized grayscale pixels and an HxWx3 measured XYZ
grid in startup-oriented odom axes, relative to the body origin. Invalid depth
stays NaN. IMU orientation has already been applied; wheel velocity predicts a
search position only. No geometry nearest-neighbor matching, map, or truth is
used to invent feature identity. Texture can constrain translation on a plane.
"""
import math
import cv2
import numpy as np
from scipy.stats import chi2


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


class _MetricRGBDRegistration:
    noise_m = .003
    pixel_noise = .5
    max_frames = 20
    minimum_matches = 24
    residual_chi_square = float(chi2.ppf(.9973, 3))

    def __init__(self, attitude_noise_rad=math.radians(.1)):
        if not np.isfinite(attitude_noise_rad) or attitude_noise_rad < math.radians(.1):
            raise ValueError('Attitude uncertainty must include the 0.1 degree engineering floor')
        self.attitude_noise_rad = float(attitude_noise_rad)
        self.references = []
        self.xyz = np.zeros(3)
        self.yaw = None
        self.stamp = None
        self.reference_number = 0
        self.initialized_moving = False
        self.last_reference = None
        self.last_reference_view = None
        self.last_reference_overlap = None
        self._closed = False
        self.last_camera_model = None
        self._wheel_prediction_used = False
        self._attempted_references = []
        # Native moon-surface returns have real texture at one or two intensity
        # levels per pixel. Enhance only measured RGB contrast; descriptor and
        # metric consensus still establish identity on independently timed data.
        self._contrast = cv2.createCLAHE(clipLimit=2., tileGridSize=(8, 8))
        self._orb = cv2.ORB_create(nfeatures=1400, edgeThreshold=15,
                                  patchSize=31, fastThreshold=4, nlevels=6)
        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

    def close(self, wait=True):
        self._closed = True

    @staticmethod
    def _reject(reason):
        return dict(accepted=False, reason=reason)

    @staticmethod
    def _camera_model(model):
        """Validate optional measured calibration; it supplies no accepted pose.

        This metadata never produces a pose. It qualifies measured camera-view
        admission only; all correspondences still require real RGB identities
        and unchanged metric registration gates. Copies prevent a caller changing a past measurement.
        """
        if model is None:
            return None
        try:
            result = {name:np.array(model[name], dtype=float, copy=True)
                      for name in ('K', 'rotation', 'translation')}
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError('Invalid RGBD camera model') from error
        K, orientation, lever = result['K'], result['rotation'], result['translation']
        if (K.shape != (3,3) or orientation.shape != (3,3) or lever.shape != (3,)
                or not all(np.isfinite(value).all() for value in result.values())
                or min(K[0,0], K[1,1]) <= 0
                or not np.allclose(K[2], [0.,0.,1.], atol=1e-9, rtol=0)
                or abs(K[0,1])+abs(K[1,0]) > 1e-9
                or not np.allclose(orientation.T@orientation, np.eye(3), atol=1e-6, rtol=0)
                or not math.isclose(np.linalg.det(orientation), 1., abs_tol=1e-6)):
            raise ValueError('Invalid RGBD camera model')
        for value in result.values():
            value.setflags(write=False)
        return result

    def _features(self, gray, grid):
        measured_contrast=self._contrast.apply(gray)
        keypoints, descriptors = self._orb.detectAndCompute(measured_contrast, None)
        if descriptors is None:
            return None
        height, width = gray.shape
        pixels = np.asarray([feature.pt for feature in keypoints])
        xy = np.floor(pixels).astype(int)
        valid = ((xy[:, 0] >= 1) & (xy[:, 1] >= 1)
                 & (xy[:, 0]+2 < width) & (xy[:, 1]+2 < height))
        kept = np.flatnonzero(valid)
        x, y = xy[kept].T
        # Bilinear interpolation uses four actual valid returns. The central
        # gradient propagates finite pixel location uncertainty in metric units;
        # a depth edge, missing pixel or discontinuity cannot be filled in.
        patch = grid[y[:, None, None]+np.arange(-1, 3)[None, :, None],
                     x[:, None, None]+np.arange(-1, 3)[None, None, :]]
        valid = (np.isfinite(patch).all(axis=(1, 2, 3))
                 & (np.max(np.linalg.norm(patch-patch[:, 1:2, 1:2], axis=3), axis=(1, 2)) <= .15))
        kept, x, y = kept[valid], x[valid], y[valid]
        if len(kept) < self.minimum_matches:
            return None
        a, b = (pixels[kept]-np.column_stack((x, y))).T
        points = (((1-a)*(1-b))[:, None]*grid[y, x]+(a*(1-b))[:, None]*grid[y, x+1]
                  +((1-a)*b)[:, None]*grid[y+1, x]+(a*b)[:, None]*grid[y+1, x+1])
        du, dv = (grid[y, x+1]-grid[y, x-1])*.5, (grid[y+1, x]-grid[y-1, x])*.5
        covariance = (self.noise_m**2*np.eye(3)[None, :, :]+self.pixel_noise**2*
                      (np.einsum('ni,nj->nij', du, du)+np.einsum('ni,nj->nij', dv, dv)))
        # ORB pyramid levels have coarser pixel localization; uncertainty must
        # grow with their measured scale rather than pretending subpixel LK ran.
        scales = np.asarray([feature.size/31. for feature in keypoints])[kept]
        covariance = (self.noise_m**2*np.eye(3)[None, :, :]
                      +(covariance-self.noise_m**2*np.eye(3)[None, :, :])*scales[:, None, None]**2)
        valid_y, valid_x = np.nonzero(np.isfinite(grid).all(axis=2))
        roi = np.array([valid_x.min(), valid_y.min(), valid_x.max()+1, valid_y.max()+1])
        return dict(pixels=pixels[kept], points=points,
                    covariance=covariance, pixel_scales=scales.copy(), descriptors=descriptors[kept].copy(),
                    image_shape=gray.shape, measured_roi=roi,
                    _measured_gray=measured_contrast,_measured_grid=grid,_measurement_token=object())

    @staticmethod
    def _spatial_support(features, selected):
        pixels, points = features['pixels'][selected], features['points'][selected]
        height, width = features['image_shape']
        roi = features['measured_roi']
        dimensions = roi[2:]-roi[:2]
        cells = np.clip(((pixels-roi[:2])/dimensions*np.array([4, 3])).astype(int), 0, [3, 2])
        occupied = len(np.unique(cells, axis=0))
        span = np.ptp(pixels, axis=0)/dimensions
        eigenvalues = np.linalg.eigvalsh(np.cov(points.T))
        adequate = (occupied >= 6 and np.all(span >= .30) and eigenvalues[1] >= .0025
                    and eigenvalues[1] >= .02*eigenvalues[2]
                    and np.prod(dimensions) >= .05*width*height
                    and np.all(dimensions >= .10*np.array([width, height])))
        return adequate, dict(image_grid_cells=occupied, image_span=span.tolist(),
                              image_span_denominator='actual_finite_depth_roi', measured_roi=roi.tolist(),
                              metric_support_eigenvalues_m2=eigenvalues.tolist())

    def _matches(self,current,reference):
        left,right=current['descriptors'],reference['descriptors']
        identical=(isinstance(left,np.ndarray) and isinstance(right,np.ndarray)
                   and left.dtype==np.uint8 and right.dtype==np.uint8
                   and left.ndim==2 and left.shape==right.shape and left.shape[1]==32
                   and np.array_equal(left,right))
        if identical:
            # Exact full-table equivalence to the original two-sided Hamming
            # KNN/ratio rule, never an independent identity or pose shortcut.
            # Unique rows have only their own zero-distance match and a strictly
            # positive second neighbor. Repeated rows have two zero neighbors
            # and fail the original strict ratio. Keep original ordering/cap.
            if len(left)<2:return np.empty((0,2),dtype=int)
            _,inverse,counts=np.unique(left,axis=0,return_inverse=True,return_counts=True)
            unique=np.flatnonzero(counts[np.asarray(inverse).reshape(-1)]==1)
            if not len(unique):return np.empty((0,2),dtype=int)
            pairs=[cv2.DMatch(int(i),int(i),0,0.) for i in unique]
        else:
            forward=self._matcher.knnMatch(current['descriptors'],reference['descriptors'],k=2)
            preliminary=[pair[0] for pair in forward if len(pair)==2 and pair[0].distance<=48
                         and pair[0].distance<.70*pair[1].distance]
            if not preliminary:return np.empty((0,2),dtype=int)
            targets=np.unique([match.trainIdx for match in preliminary])
            backward=self._matcher.knnMatch(reference['descriptors'][targets],current['descriptors'],k=2)
            reverse={int(targets[pair[0].queryIdx]):pair[0].trainIdx for pair in backward
                     if len(pair)==2 and pair[0].distance<=48 and pair[0].distance<.70*pair[1].distance}
            pairs=[match for match in preliminary if reverse.get(match.trainIdx)==match.queryIdx]
        pairs.sort(key=lambda match:(match.distance,match.queryIdx))
        roi=current['measured_roi'];width,height=roi[2:]-roi[:2]
        counts,kept={},[]
        for match in pairs:
            u,v=current['pixels'][match.queryIdx]-roi[:2]
            cell=(min(3,int(u/width*4)),min(2,int(v/height*3)))
            if counts.get(cell,0)<8:
                counts[cell]=counts.get(cell,0)+1;kept.append((match.queryIdx,match.trainIdx))
        return np.asarray(kept,dtype=int).reshape(-1,2)

    def _precision_support(self, current, reference, current_indices, target_indices, inlier):
        """Select one fixed metric-precision tier, without testing geometry.

        Shared attitude errors are used only to rank genuine measurement
        support here. Their correlated covariance remains outside the feature
        average in the original estimator below.
        """
        ci, ri = current_indices, target_indices
        covariance = current['covariance'][ci]+reference['covariance'][ri]
        for points in (current['points'][ci], reference['points'][ri]):
            covariance += self.attitude_noise_rad**2*(
                np.einsum('ni,ni->n', points, points)[:, None, None]*np.eye(3)
                -np.einsum('ni,nj->nij', points, points))
        maximum_variance = np.linalg.eigvalsh(covariance)[:, -1]
        limit = 4.*float(maximum_variance.min())
        selected = np.flatnonzero(inlier & (maximum_variance <= limit))
        return selected, dict(policy='pair_max_variance_within_4x_best_identity',
                              candidates=len(current_indices), metric_inliers=int(inlier.sum()), selected=len(selected),
                              minimum_pair_sigma_m=float(np.sqrt(maximum_variance.min())),
                              pair_sigma_limit_m=float(np.sqrt(limit)),
                              selected_pair_sigma_max_m=None if not len(selected) else float(np.sqrt(maximum_variance[selected].max())))

    def _refine_identified_measurements(self,features,reference,pairs):
        return features

    def _fit(self, features, prediction, reference):
        pairs = self._matches(features, reference)
        if len(pairs) < self.minimum_matches:
            return None, 'insufficient_rgbd_identity_matches'
        features=self._refine_identified_measurements(features,reference,pairs)
        current_indices, target_indices = pairs.T
        differences = reference['world'][target_indices]-features['points'][current_indices]
        variance = reference['covariance'][target_indices]+features['covariance'][current_indices]
        information = np.linalg.inv(variance)
        # Translation-only RANSAC: each hypothesis comes from a measured RGB
        # identity pair. Wheels neither create correspondences nor select modes.
        seeds = np.linspace(0, len(pairs)-1, min(96, len(pairs)), dtype=int)
        hypotheses = np.vstack((np.median(differences, axis=0), differences[seeds]))
        residual = differences[None, :, :]-hypotheses[:, None, :]
        distance = np.einsum('hni,nij,hnj->hn', residual, information, residual)
        masks = distance <= self.residual_chi_square
        support = masks.sum(axis=1)
        best = int(np.argmax(support))
        inlier = masks[best]
        if inlier.sum() < self.minimum_matches or inlier.mean() < .55:
            return None, 'rgbd_correspondence_consensus'
        for _ in range(3):
            normal = information[inlier].sum(axis=0)
            measurement_covariance = np.linalg.inv(normal)
            xyz = measurement_covariance@np.einsum('nij,nj->i', information[inlier], differences[inlier])
            residual = differences-xyz
            distance = np.einsum('ni,nij,nj->n', residual, information, residual)
            inlier = distance <= self.residual_chi_square
            if inlier.sum() < self.minimum_matches or inlier.mean() < .55:
                return None, 'rgbd_correspondence_consensus'
        measurement_covariance = np.linalg.inv(information[inlier].sum(axis=0))
        xyz = measurement_covariance@np.einsum('nij,nj->i', information[inlier], differences[inlier])
        # A repeated pattern may have two well supported metric translations.
        # A close wheel prediction must never resolve that visual ambiguity.
        separated = np.linalg.norm(hypotheses-xyz, axis=1) > max(.03, 6*math.sqrt(np.linalg.eigvalsh(measurement_covariance).max()))
        alternatives = np.flatnonzero(separated & (support >= max(self.minimum_matches, .60*inlier.sum())))
        # A noisy single-pair seed is not a second translation mode. Refine its
        # own measured consensus first; tails of one mode converge to that same
        # mode, while a repeated-pattern displacement retains a distinct center.
        for alternative in sorted(alternatives, key=lambda index: -support[index])[:8]:
            alternate_inlier = masks[alternative].copy()
            for _ in range(3):
                if alternate_inlier.sum() < self.minimum_matches:
                    break
                alternate_covariance = np.linalg.inv(information[alternate_inlier].sum(axis=0))
                alternate_xyz = alternate_covariance@np.einsum('nij,nj->i', information[alternate_inlier], differences[alternate_inlier])
                alternate_residual = differences-alternate_xyz
                alternate_inlier = np.einsum('ni,nij,nj->n', alternate_residual, information, alternate_residual) <= self.residual_chi_square
            delta = alternate_xyz-xyz
            separation_covariance = alternate_covariance+measurement_covariance+2*self.noise_m**2*np.eye(3)
            if (alternate_inlier.sum() >= max(self.minimum_matches, .60*inlier.sum())
                    and np.linalg.norm(delta) > .03
                    and delta@np.linalg.solve(separation_covariance, delta) > self.residual_chi_square):
                return None, 'ambiguous_rgbd_translation'
        current_good, current_support = self._spatial_support(features, current_indices[inlier])
        target_good, target_support = self._spatial_support(reference, target_indices[inlier])
        support_layer = 'full_identity_inliers'
        precision_support = None
        if not (current_good and target_good):
            # Two predeclared sufficient evidence scales: the original full
            # identity consensus, OR one fixed metric-precision tier. Both
            # frames must qualify together at the SAME scale. Long, uncertain
            # depth levers cannot negate an already adequate precise support,
            # nor may that support borrow image cells from the full set.
            # The tier threshold is fixed from all measured identity pairs
            # before metric consensus and never searched against PCA or pose.
            support_indices, precision_support = self._precision_support(
                features, reference, current_indices, target_indices, inlier)
            if len(support_indices) < self.minimum_matches:
                return None, 'insufficient_rgbd_spatial_support'
            current_good, current_support = self._spatial_support(features, current_indices[support_indices])
            target_good, target_support = self._spatial_support(reference, target_indices[support_indices])
            if not (current_good and target_good):
                return None, 'insufficient_rgbd_spatial_support'
            support_layer = 'fixed_precision_identity_inliers'
        if np.linalg.norm(xyz-prediction) > .10:
            return None, 'rgbd_correction_jump'
        rmse = float(np.sqrt(np.mean(np.sum((differences[inlier]-xyz)**2, axis=1))))
        if rmse > .015:
            return None, 'rgbd_registration_residual'
        # Shared metric calibration is not averaged down by feature count.
        # Registered reference pose uncertainty is common to all its points and
        # is added once, never divided by the number of correspondences.
        measurement_covariance += self.noise_m**2*np.eye(3)
        lever = features['points'][current_indices[inlier]].mean(axis=0)
        reference_lever = reference['points'][target_indices[inlier]].mean(axis=0)
        # Both frames share an attitude measurement error across their features.
        # Keep this term outside the feature-count average, just like calibration.
        for measured_lever in (lever, reference_lever):
            measurement_covariance += self.attitude_noise_rad**2*(np.dot(measured_lever, measured_lever)*np.eye(3)-np.outer(measured_lever, measured_lever))
        covariance = measurement_covariance+reference['anchor_covariance']
        if (not np.isfinite(covariance).all() or np.linalg.eigvalsh(covariance).min() <= 0
                or np.linalg.eigvalsh(covariance[:2, :2]).max() > .05**2):
            return None, 'uncertain_rgbd_translation'
        return dict(xyz=xyz, covariance=covariance, measurement_covariance=measurement_covariance,
                    inliers=int(inlier.sum()), identity_matches=len(pairs),
                    overlap=float(inlier.sum()/min(len(features['points']), 96)), rmse_m=rmse,
                    reference_id=reference['id'], reference_support=dict(current=current_support,
                    reference=target_support, spatial_support_policy='full_or_fixed_4x_best_pair_variance',
                    spatial_support_layer=support_layer, precision_support=precision_support, sensor_noise_floor_m=self.noise_m, pixel_noise_px=self.pixel_noise,
                    attitude_noise_floor_rad=self.attitude_noise_rad,
                    identity_pixel_scale_max=dict(
                        current=float(np.max(features['pixel_scales'][current_indices])) if 'pixel_scales' in features else 1.,
                        reference=float(np.max(reference['pixel_scales'][target_indices])) if 'pixel_scales' in reference else 1.),
                    measured_rgb_preprocessing='clahe_clip2_tiles8x8_fast4')), 'accepted'

    def _reference(self, features, covariance, parent):
        self.reference_number += 1
        return dict(id=self.reference_number, parent=parent, xyz=self.xyz.copy(), yaw=self.yaw,
                    world=features['points'].copy()+self.xyz,
                    anchor_covariance=covariance.copy(),
                    **{name: value.copy() if isinstance(value, np.ndarray) else value
                       for name, value in features.items()})

    def _install_reference(self, features, covariance, parent):
        reference = self._reference(features, covariance, parent)
        validation, _ = self._fit(features, self.xyz, reference)
        if validation is None:
            return
        self.references.append(reference)
        if len(self.references) > self.max_frames:
            del self.references[1]
        self.last_reference = (self.xyz.copy(), self.yaw)
        self.last_reference_view = features['points'].mean(axis=0)
        self.last_reference_overlap = validation['overlap']

    def _reference_candidates(self,prediction):
        latest=self.references[-1:]
        local=sorted(self.references[1:-1],key=lambda ref:np.linalg.norm(ref['xyz']-prediction))
        return [self.references[0]]+(latest if len(self.references)>1 else [])+local

    def update(self, stamp, gray_image, points_odom_orientation, yaw, wheel_vx=None, wheel_wz=None,
               camera_model=None):
        self._wheel_prediction_used = False
        self._attempted_references = []
        if self._closed:
            return self._reject('rgbd_core_closed')
        wheel_hint_available = wheel_vx is not None and wheel_wz is not None
        if ((wheel_vx is None) != (wheel_wz is None) or
                (wheel_hint_available and not np.isfinite([wheel_vx, wheel_wz]).all())):
            return self._reject('invalid_rgbd_input')
        gray, grid = np.asarray(gray_image), np.asarray(points_odom_orientation, dtype=float)
        if (not np.isfinite([stamp, yaw]).all() or gray.ndim != 2
                or min(gray.shape, default=0) < 40 or gray.dtype != np.uint8 or grid.shape != gray.shape+(3,)):
            return self._reject('invalid_rgbd_input')
        try:
            measured_camera_model = self._camera_model(camera_model)
        except ValueError:
            return self._reject('invalid_rgbd_camera_model')
        if self.stamp is not None and stamp <= self.stamp:
            return self._reject('nonincreasing_rgbd_stamp')
        moving = wheel_hint_available and (abs(wheel_vx) > .005 or abs(wheel_wz) > .03)
        if not self.references and moving:
            self.initialized_moving = True
        if self.initialized_moving:
            return self._reject('initialization_motion_requires_new_session')
        if not self.references and not wheel_hint_available:
            return self._reject('missing_wheel_hint_for_initialization')
        dt = 0. if self.stamp is None else stamp-self.stamp
        if dt > 1.:
            return self._reject('rgbd_gap_requires_new_session')
        features = self._features(np.ascontiguousarray(gray), grid)
        if features is None:
            return self._reject('insufficient_rgbd_texture_depth')
        prediction = self.xyz.copy()
        if wheel_hint_available and self.yaw is not None and self.references:
            self._wheel_prediction_used = True
            prediction[:2] += rotation(self.yaw)@np.array([wheel_vx*dt, 0.])
        self.stamp, self.yaw = float(stamp), float(yaw)
        self.last_camera_model = measured_camera_model
        if not self.references:
            adequate, support = self._spatial_support(features, np.arange(len(features['points'])))
            if not adequate:
                return self._reject('insufficient_rgbd_spatial_support')
            # Self matching must be distinct: uniform/repeated descriptors are
            # not valid identities even for the initial coordinate anchor.
            initial = self._reference(features, np.zeros((3, 3)), None)
            fitted, reason = self._fit(features, self.xyz, initial)
            self._attempted_references.append(dict(reference_id=initial['id'],reason=reason,accepted=fitted is not None))
            if fitted is None:
                return self._reject(reason)
            self.references.append(initial)
            self.last_reference = (self.xyz.copy(), yaw)
            self.last_reference_view = features['points'].mean(axis=0)
            self.last_reference_overlap = fitted['overlap']
            return dict(accepted=True, reason='initialized', stamp=float(stamp), yaw=yaw,
                        reference_frames=1, **fitted)
        # The latest measured view matters when a stationary gimbal changes its
        # field of view: all registered body positions can then be identical.
        candidates = self._reference_candidates(prediction)
        fits, reason = [], 'insufficient_rgbd_identity_matches'
        # Check origin and the latest measured view first. A nearby fallback is
        # needed only if neither passes; comparing five overlapping references
        # every frame would spend the source's freshness budget on duplicate work.
        for number, reference in enumerate(candidates[:5]):
            if number >= 2 and fits:
                break
            fitted, failure = self._fit(features, prediction, reference)
            self._attempted_references.append(dict(reference_id=reference['id'],reason=failure,accepted=fitted is not None))
            if fitted is not None:
                fits.append(fitted)
            else:
                reason = failure
        if not fits:
            # A search hint is never a registered pose, including internally:
            # the next hint-free measurement must start at the last real fit.
            return self._reject(reason)
        for left in fits:
            for right in fits:
                delta = left['xyz']-right['xyz']
                covariance = left['covariance']+right['covariance']
                if (min(left['inliers'], right['inliers']) >= .60*max(fit['inliers'] for fit in fits)
                        and delta@np.linalg.solve(covariance, delta) > self.residual_chi_square):
                    return self._reject('ambiguous_rgbd_reference_translation')
        fitted = min(fits, key=lambda fit: np.linalg.eigvalsh(fit['covariance'][:2, :2]).max())
        self.xyz = fitted['xyz'].copy()
        position, heading = self.last_reference
        turn = abs(math.atan2(math.sin(yaw-heading), math.cos(yaw-heading)))
        view_shift = np.linalg.norm(features['points'].mean(axis=0)-self.last_reference_view)
        new_view = view_shift >= .30 and fitted['overlap'] < self.last_reference_overlap
        if np.linalg.norm(self.xyz-position) >= .30 or turn >= .25 or new_view:
            self._install_reference(features, fitted['covariance'], fitted['reference_id'])
        return dict(accepted=True, reason='accepted', stamp=float(stamp), yaw=yaw,
                    reference_frames=len(self.references), reference_pending=False,
                    reference_build_failures=0, **fitted)


class _ConflictingRegisteredView(Exception):
    pass


class _MetricRGBDReferenceMaintenance(_MetricRGBDRegistration):
    _PATCH_OFFSETS=np.stack(np.meshgrid(np.arange(-10,11),np.arange(-10,11)),axis=-1).reshape(-1,2)

    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self._active_view_id=None
        self._frame_fits={}
        self._frame_prediction=None
        self._defer_reference_admission=False
        self._reference_admission_requested=False
        self._entry_reference_ids=set()
        self._anchor_adoption={}
        self._reference_build_attempts=[]
        self._entry_latest_reference_id=None
        self._refresh_latest_template=False
        # Only owned registered reference geometry is cached across frames.
        # Dynamic current-view projections in adoption remain uncached.
        self._retention_coverage={}
        self._retention_references={}
        self._view_motion_cache=None

    def _features(self,gray,grid):
        features=super()._features(gray,grid)
        self._current_features=features
        return features

    def _reference(self,features,covariance,parent):
        reference=super()._reference(features,covariance,parent)
        reference['camera_model']=self.last_camera_model
        return reference

    def _fit(self,features,prediction,reference):
        if self._frame_prediction is None:self._frame_prediction=prediction.copy()
        self._last_measurement_refinement=None
        fitted,reason=super()._fit(features,prediction,reference)
        if fitted is not None and self._last_measurement_refinement is not None:
            fitted['reference_support']['measurement_refinement']=dict(self._last_measurement_refinement)
        self._frame_fits[reference['id']]=fitted
        return fitted,reason

    def _refine_identified_measurements(self,features,reference,pairs):
        """Refine only measured locations of the already frozen ORB identities.

        Density selection remains at the original ORB seed pixels. Geometry
        sees the per-reference refined pixels/points; these never modify the
        current raw features, another reference's fit, or a future template.
        Every failed refinement keeps its original valid RGBD measurement.
        """
        required=('_measured_gray','_measured_grid','_measurement_token','pixel_scales')
        if not all(key in features and key in reference for key in required):return features
        if (features['_measurement_token'] is not None
                and features['_measurement_token'] is reference['_measurement_token']):return features
        ci,ri=pairs.T
        left,right=reference['_measured_gray'],features['_measured_gray']
        grid=features['_measured_grid']
        if left.shape!=right.shape or grid.shape!=right.shape+(3,):return features
        p0=np.asarray(reference['pixels'][ri],dtype=np.float32)
        seed=np.asarray(features['pixels'][ci],dtype=np.float32)
        budgets=3*self.pixel_noise*np.hypot(features['pixel_scales'][ci],reference['pixel_scales'][ri])
        h,w=right.shape
        def inside(points,radius):
            return (np.isfinite(points).all(axis=1)&(points[:,0]-radius>=0)&(points[:,1]-radius>=0)
                    &(points[:,0]+radius+1<w)&(points[:,1]+radius+1<h))
        # All accepted endpoint/photo samples lie in real image pixels. OpenCV
        # does not expose transient internal iteration paths; the full original
        # seed uncertainty neighborhood plus gradient margin is guarded too.
        eligible=inside(p0,12+budgets)&inside(seed,12+budgets)&np.isfinite(budgets)&(budgets>0)
        diag=dict(policy='original_orb_identities_clahe_bidirectional_lk',identity_pairs=len(pairs),
                  eligible=int(eligible.sum()),applied=0,all_original_identities_retained=True)
        self._last_measurement_refinement=diag
        which=np.flatnonzero(eligible)
        if not len(which):return features
        parameters=dict(winSize=(21,21),maxLevel=0,
                        criteria=(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,30,1e-3),
                        flags=cv2.OPTFLOW_USE_INITIAL_FLOW,minEigThreshold=1e-4)
        try:
            forward,fs,_=cv2.calcOpticalFlowPyrLK(left,right,p0[which,None,:],seed[which,None,:].copy(),**parameters)
            if forward is None or fs is None:return features
            q=forward[:,0,:]
            safe=np.where(np.isfinite(q),q,seed[which])
            backward,bs,_=cv2.calcOpticalFlowPyrLK(right,left,safe[:,None,:],p0[which,None,:].copy(),**parameters)
            if backward is None or bs is None:return features
        except cv2.error:
            return features
        back=backward[:,0,:]
        valid=(fs.ravel().astype(bool)&bs.ravel().astype(bool)&np.isfinite(q).all(axis=1)&np.isfinite(back).all(axis=1)
               &(np.linalg.norm(q-seed[which],axis=1)<=budgets[which])
               &(np.linalg.norm(back-p0[which],axis=1)<=self.pixel_noise)&inside(q,12.))
        which,q=which[valid],q[valid].astype(float)
        if not len(which):return features
        target=self._sample_identity_patch(left,p0[which])
        before=self._sample_identity_patch(right,seed[which])
        after=self._sample_identity_patch(right,q)
        improves=np.mean((target-after)**2,axis=1)<=np.mean((target-before)**2,axis=1)+1e-12
        which,q=which[improves],q[improves]
        if not len(which):return features
        xy=np.floor(q).astype(int);x,y=xy.T
        valid=(x>=1)&(y>=1)&(x+2<w)&(y+2<h)
        which,q,x,y=which[valid],q[valid],x[valid],y[valid]
        if not len(which):return features
        patch=grid[y[:,None,None]+np.arange(-1,3)[None,:,None],x[:,None,None]+np.arange(-1,3)[None,None,:]]
        valid=(np.isfinite(patch).all(axis=(1,2,3))
               &(np.max(np.linalg.norm(patch-patch[:,1:2,1:2],axis=3),axis=(1,2))<=.15))
        which,q,x,y=which[valid],q[valid],x[valid],y[valid]
        if not len(which):return features
        a,b=(q-np.column_stack((x,y))).T
        points=(((1-a)*(1-b))[:,None]*grid[y,x]+(a*(1-b))[:,None]*grid[y,x+1]
                +((1-a)*b)[:,None]*grid[y+1,x]+(a*b)[:,None]*grid[y+1,x+1])
        du,dv=(grid[y,x+1]-grid[y,x-1])*.5,(grid[y+1,x]-grid[y-1,x])*.5
        scales=features['pixel_scales'][ci[which]]
        covariance=(self.noise_m**2*np.eye(3)[None,:,:]+self.pixel_noise**2*scales[:,None,None]**2*
                    (np.einsum('ni,nj->nij',du,du)+np.einsum('ni,nj->nij',dv,dv)))
        original=features['covariance'][ci[which]]
        delta=covariance-original
        values,vectors=np.linalg.eigh((delta+delta.transpose(0,2,1))*.5)
        upper=original+(vectors*np.maximum(values,0)[:,None,:])@vectors.transpose(0,2,1)
        refined=dict(features)
        for key in ('pixels','points','covariance'):refined[key]=features[key].copy()
        refined['pixels'][ci[which]]=q
        refined['points'][ci[which]]=points
        refined['covariance'][ci[which]]=upper
        diag['applied']=len(which)
        return refined

    @classmethod
    def _sample_identity_patch(cls,image,pixels):
        """Keep all441 samples and arithmetic order, sharing each center's weights.

        LK sites are float32 (sometimes exactly widened to float64). Integer
        offsets preserve their fractional parts exactly. Other inputs use the
        original site-wise sampler, including its rounding and boundary rules.
        """
        offset=cls._PATCH_OFFSETS
        standard=(type(image) is np.ndarray and image.dtype==np.uint8 and image.ndim==2
            and max(image.shape)<=2**31 and type(pixels) is np.ndarray
            and pixels.dtype in (np.dtype('float32'),np.dtype('float64'))
            and pixels.ndim==2 and pixels.shape[1]==2 and np.isfinite(pixels).all()
            and np.all(pixels>=12.) and np.all(pixels[:,0]+12.<image.shape[1])
            and np.all(pixels[:,1]+12.<image.shape[0])
            and (pixels.dtype==np.float32 or np.array_equal(pixels,pixels.astype(np.float32).astype(np.float64))))
        if not standard:
            sites=pixels[:,None,:]+offset
            xy=np.floor(sites).astype(int);x,y=xy[...,0],xy[...,1]
            a,b=(sites-xy)[...,0],(sites-xy)[...,1]
            return ((1-a)*(1-b)*image[y,x]+a*(1-b)*image[y,x+1]
                    +(1-a)*b*image[y+1,x]+a*b*image[y+1,x+1])
        result=np.empty((len(pixels),len(offset)),dtype=np.float64)
        for start in range(0,len(pixels),32):
            centers=pixels[start:start+32]
            xy=np.floor(centers).astype(int)
            x,y=xy[:,:1]+offset[:,0],xy[:,1:]+offset[:,1]
            fraction=centers.astype(np.float64)-xy
            a,b=fraction[:,:1],fraction[:,1:]
            result[start:start+len(centers)]=(
                (1-a)*(1-b)*image[y,x]+a*(1-b)*image[y,x+1]
                +(1-a)*b*image[y+1,x]+a*b*image[y+1,x+1])
        return result

    @staticmethod
    def _angle(left,right):
        return math.acos(float(np.clip((np.trace(left.T@right)-1)*.5,-1.,1.)))

    def _activate(self,reference):
        self._active_view_id=reference['id']
        self.last_reference=(reference['xyz'].copy(),reference['yaw'])
        self.last_reference_view=reference['points'].mean(axis=0)
        self.last_reference_overlap=reference['self_overlap']

    def _view_rotation_limit(self,model):
        # A measured view should hand off before rotation shifts the image by
        # half the smallest ORB descriptor patch. Calibration sets that angular
        # footprint; this controls reference admission, never accepted identity.
        focal=max(float(model['K'][0,0]),float(model['K'][1,1]))
        return min(.10,math.atan2(.5*self._orb.getPatchSize(),focal))

    def _reference_candidates(self,prediction):
        candidates=super()._reference_candidates(prediction)
        if self.last_camera_model is None:return candidates
        leading=2 if len(candidates)>1 else 1
        def priority(reference):
            motion=self._projected_view_motion(reference,body_xyz=prediction)
            visible=motion['visible_reference_features']>=self.minimum_matches
            shift=motion['projected_median_shift_px']
            model=reference.get('camera_model')
            angle=(math.inf if model is None else
                   self._angle(model['rotation'],self.last_camera_model['rotation']))
            fraction=motion['visible_reference_features']/max(1,motion['reference_features'])
            # The activated view was independently fitted on an accepted frame.
            # Projection only orders bounded searches; original RGB identities
            # and all metric gates must still succeed before any pose is used.
            return (0 if reference['id']==self._active_view_id and visible else 1,
                    0 if visible else 1,math.inf if shift is None else shift,
                    -fraction,angle,float(np.linalg.norm(reference['xyz']-prediction)))
        return candidates[:leading]+sorted(candidates[leading:],key=priority)

    def _angular_near_view(self,reference):
        model=reference.get('camera_model')
        return (model is not None and self.last_camera_model is not None
            and self._angle(model['rotation'],self.last_camera_model['rotation'])<min(
                self._view_rotation_limit(model),self._view_rotation_limit(self.last_camera_model))
            and np.linalg.norm(reference['xyz']-self.xyz)<.30)

    def _projected_view_motion(self,reference,body_xyz=None):
        cache=self._view_motion_cache
        key=None
        if cache is not None and self.last_camera_model is not None and self._current_features is not None:
            position=np.asarray(self.xyz if body_xyz is None else body_xyz)
            if position.shape==(3,) and position.dtype==np.float64 and np.isfinite(position).all():
                key=(id(reference),id(self.last_camera_model),tuple(self._current_features['image_shape']),position.tobytes())
                entry=cache.get(key)
                if entry is not None:return dict(entry[1])
        result=self._compute_projected_view_motion(reference,body_xyz)
        if key is not None and len(cache)<128:
            # Keep the actual reference alive until this update ends: an ID
            # cannot be reused after retention removes it. Never cache fits.
            cache[key]=(reference,dict(result))
        return result

    def _compute_projected_view_motion(self,reference,body_xyz=None):
        # Reproject only already measured, registered reference points. This
        # selects when to retain another measured view; it never supplies a
        # feature identity, a pose constraint, or an accepted translation.
        model=self.last_camera_model
        result=dict(visible_reference_features=0,reference_features=len(reference['points']),
                    projected_median_shift_px=None)
        if model is None or self._current_features is None:return result
        # Maintenance uses the accepted pose. An optional search prediction
        # may order candidates but can never establish identity or a pose.
        position=self.xyz if body_xyz is None else body_xyz
        optical=(reference['world']-position-model['translation'])@model['rotation']
        # Match the caller's measured depth range, rather than RGB FOV alone.
        usable=np.isfinite(optical).all(axis=1)&(optical[:,2]>.1)&(optical[:,2]<=6.)
        indices=np.flatnonzero(usable)
        if len(indices)==0:return result
        projected=optical[indices]@model['K'].T
        pixels=projected[:,:2]/projected[:,2:]-.5
        height,width=self._current_features['image_shape']
        visible=(np.isfinite(pixels).all(axis=1)&(pixels[:,0]>=0)&(pixels[:,0]<width)
                 &(pixels[:,1]>=0)&(pixels[:,1]<height))
        indices,pixels=indices[visible],pixels[visible]
        result['visible_reference_features']=len(indices)
        if len(indices)>=self.minimum_matches:
            result['projected_median_shift_px']=float(np.median(np.linalg.norm(
                pixels-reference['pixels'][indices],axis=1)))
        return result

    def _support_handoff(self,reference):
        """Retain a qualified view before measured span or residual reserve runs out.

        This changes bounded reference maintenance only. A real accepted RGBD
        fit must already prove the original spatial/covariance gates. Half a
        descriptor patch is reserved above the 0.30 span boundary in each
        exposure's own measured ROI, and at least a quarter patch of measured
        parallax is needed for span reserve. Residual reserve is one 3mm floor
        below the original 15mm gate, with genuine parallax exceeding the
        two-exposure 3-sigma pixel budget at their coarsest identified scales.
        Neither policy can create a pose or relax registration acceptance.
        """
        result=dict(needed=False,minimum_span_margin_px=None,
                    span_margin_budget_px=.5*self._orb.getPatchSize(),
                    minimum_parallax_px=.25*self._orb.getPatchSize())
        fit=self._frame_fits.get(reference['id'])
        if fit is None:return result
        support=fit.get('reference_support',{})
        margins=[]
        for side in ('current','reference'):
            item=support.get(side,{})
            span=np.asarray(item.get('image_span',[]),dtype=float)
            roi=np.asarray(item.get('measured_roi',[]),dtype=float)
            if span.shape!=(2,) or roi.shape!=(4,):return result
            dimensions=roi[2:]-roi[:2]
            if not np.isfinite(np.r_[span,dimensions]).all() or np.any(dimensions<=0):return result
            margins.extend(((span-.30)*dimensions).tolist())
        result['minimum_span_margin_px']=float(min(margins))
        span_tight=min(margins)<=result['span_margin_budget_px']
        rmse=fit.get('rmse_m')
        result['residual_margin_m']=None if rmse is None else .015-float(rmse)
        result['residual_margin_budget_m']=self.noise_m
        scales=support.get('identity_pixel_scale_max',{})
        pixel_scales=np.array([scales.get('current',np.nan),scales.get('reference',np.nan)],dtype=float)
        valid_scales=np.isfinite(pixel_scales).all() and np.all(pixel_scales>=1.)
        result['residual_minimum_parallax_px']=(float(3*self.pixel_noise*np.linalg.norm(pixel_scales))
                                                if valid_scales else None)
        residual_tight=(rmse is not None and np.isfinite(rmse) and valid_scales
                        and result['residual_margin_m']<=result['residual_margin_budget_m'])
        if not span_tight and not residual_tight:return result
        motion=self._projected_view_motion(reference)
        shift=motion['projected_median_shift_px']
        result['projected_reference_motion']=motion
        result['span_triggered']=bool(span_tight and shift is not None and shift>=result['minimum_parallax_px'])
        result['residual_triggered']=bool(residual_tight and shift is not None
                                          and shift>=result['residual_minimum_parallax_px'])
        result['needed']=result['span_triggered'] or result['residual_triggered']
        return result

    def _latest_template_handoff(self,reference):
        """Refresh a failed newest template only after real resolvable motion.

        A successful older fit supplies the pose. A latest tracking attempt rejected specifically for metric residual
        supplies no pose/covariance, merely evidence that its measured image
        template no longer registers consistently. Ordinary FOV/identity or
        spatial-support failures retain the original reuse policy. Both the accepted anchor and latest
        template must have changed beyond the existing two-exposure 3-sigma
        pixel budget; this cannot repeatedly refresh a stationary old view.
        """
        result=dict(needed=False,latest_reference_id=self._entry_latest_reference_id)
        if reference is None or reference['id']==self._entry_latest_reference_id:return result
        latest=next((item for item in self.references
                     if item['id']==self._entry_latest_reference_id),None)
        failed=next((item for item in self._attempted_references
                     if item['reference_id']==self._entry_latest_reference_id and not item['accepted']
                     and item['reason']=='rgbd_registration_residual'),None)
        fit=self._frame_fits.get(reference['id'])
        if latest is None or failed is None or fit is None:return result
        scales=fit.get('reference_support',{}).get('identity_pixel_scale_max',{})
        pair_scales=np.array([scales.get('current',np.nan),scales.get('reference',np.nan)])
        if not np.isfinite(pair_scales).all() or np.any(pair_scales<1.):return result
        current_scales=self._current_features.get('pixel_scales',np.array([]))
        latest_scales=latest.get('pixel_scales',np.array([]))
        if not len(current_scales) or not len(latest_scales):return result
        latest_pair_scales=np.array([np.max(current_scales),np.max(latest_scales)])
        if not np.isfinite(latest_pair_scales).all() or np.any(latest_pair_scales<1.):return result
        accepted_budget=float(3*self.pixel_noise*np.linalg.norm(pair_scales))
        latest_budget=float(3*self.pixel_noise*np.linalg.norm(latest_pair_scales))
        accepted_motion=self._projected_view_motion(reference)
        latest_motion=self._projected_view_motion(latest)
        accepted_shift=accepted_motion['projected_median_shift_px']
        latest_shift=latest_motion['projected_median_shift_px']
        result.update(latest_tracking_reason=failed['reason'],accepted_reference_id=reference['id'],
            accepted_reference_motion=accepted_motion,latest_reference_motion=latest_motion,
            accepted_minimum_parallax_px=accepted_budget,latest_minimum_parallax_px=latest_budget)
        result['needed']=(accepted_shift is not None and accepted_shift>=accepted_budget
                          and latest_shift is not None and latest_shift>=latest_budget)
        return result

    def _near_view(self,reference):
        if self._refresh_latest_template and self._latest_template_handoff(reference)['needed']:return False
        if not self._angular_near_view(reference):return False
        motion=self._projected_view_motion(reference)
        shift=motion['projected_median_shift_px']
        # Invalid/behind-camera or insufficient projections never prove that a
        # previously measured view still covers the current physical image.
        return (shift is not None and shift<.5*self._orb.getPatchSize()
                and not self._support_handoff(reference)['needed'])

    def _registered_view(self,features,current):
        selected=next((ref for ref in self.references if ref['id']==current['reference_id']),None)
        if selected is not None and self._near_view(selected):return selected
        # Angular neighbors are still independently fitted for conflicting
        # anchors even if parallax makes them unsuitable for simple reuse.
        nearby=[ref for ref in self.references if self._angular_near_view(ref)]
        def proximity(reference):
            shift=self._projected_view_motion(reference)['projected_median_shift_px']
            return (math.inf if shift is None else shift,
                    self._angle(reference['camera_model']['rotation'],self.last_camera_model['rotation']))
        nearby.sort(key=proximity)
        for reference in nearby[:2]:
            if reference['id'] in self._frame_fits:
                fit=self._frame_fits[reference['id']]
            else:
                fit,_=self._fit(features,self._frame_prediction,reference)
            if fit is None:continue
            delta=fit['xyz']-current['xyz']
            statistic=delta@np.linalg.solve(fit['covariance']+current['covariance'],delta)
            if statistic>self.residual_chi_square:
                if min(fit['inliers'],current['inliers'])>=.60*max(fit['inliers'],current['inliers']):
                    raise _ConflictingRegisteredView()
                continue
            if self._near_view(reference):return reference
        return None

    @staticmethod
    def _anchor_risk(reference):
        return float(np.linalg.eigvalsh(reference['anchor_covariance'][:2,:2]).max())

    @staticmethod
    def _sparse_visible_fraction(source,target):
        """Measured-view retention proxy only, never a match or pose proof."""
        model=target.get('camera_model')
        if model is None or not len(source['world']):return 0.
        indices=np.linspace(0,len(source['world'])-1,min(48,len(source['world'])),dtype=int)
        optical=(source['world'][indices]-target['xyz']-model['translation'])@model['rotation']
        finite=np.isfinite(optical).all(axis=1)&(optical[:,2]>.1)&(optical[:,2]<=6.)
        pixels=np.zeros((len(indices),2))
        image=optical[finite]@model['K'].T
        pixels[finite]=image[:,:2]/image[:,2:]-.5
        height,width=target['image_shape']
        finite &= (pixels[:,0]>=0)&(pixels[:,0]<width)&(pixels[:,1]>=0)&(pixels[:,1]<height)
        return float(finite.mean())

    def _prune_retention_cache(self,library):
        references={ref['id']:ref for ref in library}
        # IDs are monotonic in a session. Also invalidate a replaced object in
        # case a rollback or an explicit test fixture reuses an old ID.
        valid={number for number,ref in references.items()
               if self._retention_references.get(number) is ref}
        self._retention_coverage={key:value for key,value in self._retention_coverage.items()
                                  if key[0] in valid and key[1] in valid}
        self._retention_references=references

    def close(self,wait=True):
        self._retention_coverage={};self._retention_references={}
        return super().close(wait=wait)

    def _retire_redundant_reference(self,parent,incoming=None):
        # Score a provisional library without exposing a 21st reference while
        # NumPy releases the GIL. A single slice commits the bounded result.
        library=list(self.references)+([] if incoming is None else [incoming])
        self._prune_retention_cache(library)
        if len(library)<=self.max_frames:
            self.references[:]=library
            return
        # Pin the coordinate origin, new latest and this exposure's actual
        # parent. Keep the active registered view as well if it differs.
        pin_order=list(dict.fromkeys((library[0]['id'],library[-1]['id'],parent,self._active_view_id)))
        pinned=set(pin_order[:self.max_frames])
        pool=[ref for ref in library if ref['id'] not in pinned]
        if not pool:raise RuntimeError('No unpinned RGBD reference slot')
        coverage=self._retention_coverage
        risks={ref['id']:self._anchor_risk(ref) for ref in library}
        def mutual(left,right):
            key=tuple(sorted((left['id'],right['id'])))
            value=coverage.get(key)
            if value is None:
                value=min(self._sparse_visible_fraction(left,right),
                          self._sparse_visible_fraction(right,left))
                coverage[key]=value
            return value
        def redundancy(reference):
            alternatives=[other for other in library
                          if other['id']!=reference['id'] and risks[other['id']]<=risks[reference['id']]]
            covered=max((mutual(reference,other) for other in alternatives),default=0.)
            # Prefer discarding a weak anchor covered in both measured views by
            # an equally or more precise one. If all views are distinct, retain
            # the lower-risk anchors; no finite library promises global cover.
            return (covered>=.60,covered if covered>=.60 else 0.,risks[reference['id']],-reference['id'])
        retired=max(pool,key=redundancy)
        self.references[:]=[ref for ref in library if ref['id']!=retired['id']]
        self._prune_retention_cache(self.references)
        self._anchor_adoption['retired_reference_id']=retired['id']

    def _prefer_measured_anchor(self,features,current):
        """At a genuine admission boundary, try at most one extra old view."""
        current_risk=float(np.linalg.eigvalsh(current['covariance'][:2,:2]).max())
        entry=[ref for ref in self.references if ref['id'] in self._entry_reference_ids]
        candidates=[]
        for reference in entry:
            fit=self._frame_fits.get(reference['id'])
            if fit is not None:candidates.append((reference,fit))
        # Projections only rank an eligible search. Original RGB identities,
        # metric support, residual, ambiguity and covariance gates still decide.
        current_view=dict(world=features['points']+self.xyz,xyz=self.xyz,
                          image_shape=features['image_shape'],camera_model=self.last_camera_model)
        eligible=[]
        attempted_ids={item['reference_id'] for item in self._attempted_references}
        if self.last_camera_model is not None:
            for reference in entry:
                if (reference['id'] in attempted_ids or reference['id'] in self._frame_fits or reference.get('camera_model') is None
                        or self._anchor_risk(reference)>=current_risk):continue
                mutual=min(self._sparse_visible_fraction(reference,current_view),
                           self._sparse_visible_fraction(current_view,reference))
                if mutual<.60:continue
                motion=self._projected_view_motion(reference)
                if motion['visible_reference_features']<self.minimum_matches:continue
                eligible.append((self._anchor_risk(reference),-mutual,
                    motion['projected_median_shift_px'],reference['id'],reference))
        self._anchor_adoption=dict(additional_fit_attempts=0,from_reference_id=current['reference_id'])
        if eligible:
            reference=min(eligible,key=lambda item:item[:4])[-1]
            fit,reason=self._fit(features,self._frame_prediction,reference)
            self._anchor_adoption.update(additional_fit_attempts=1,
                additional_reference_id=reference['id'],additional_reason=reason)
            if fit is not None:candidates.append((reference,fit))
        # A newly found fit must not conceal a strong mode already seen in
        # this exposure. Statistical consistency with only the selected fit is
        # not transitive, so preserve the original pairwise ambiguity rule.
        evidence=[fit for _,fit in candidates]
        maximum_inliers=max((fit['inliers'] for fit in evidence),default=current['inliers'])
        for number,left in enumerate(evidence):
            for right in evidence[number+1:]:
                delta=left['xyz']-right['xyz']
                if (min(left['inliers'],right['inliers'])>=.60*maximum_inliers
                        and delta@np.linalg.solve(left['covariance']+right['covariance'],delta)>self.residual_chi_square):
                    raise _ConflictingRegisteredView()
        consistent=[]
        for reference,fit in candidates:
            delta=fit['xyz']-current['xyz']
            statistic=float(delta@np.linalg.solve(fit['covariance']+current['covariance'],delta))
            if statistic>self.residual_chi_square:
                if min(fit['inliers'],current['inliers'])>=.60*max(fit['inliers'],current['inliers']):
                    raise _ConflictingRegisteredView()
                continue
            consistent.append((reference,fit))
        if consistent:
            reference,fit=min(consistent,key=lambda item:np.linalg.eigvalsh(item[1]['covariance'][:2,:2]).max())
            if np.linalg.eigvalsh(fit['covariance'][:2,:2]).max()<current_risk:
                # Adopt one complete genuine fit atomically, never covariance
                # from a different pose and never a self-validation result.
                current.update(fit)
                self.xyz=fit['xyz'].copy()
                self._anchor_adoption['adopted_reference_id']=reference['id']
        self._anchor_adoption['selected_reference_id']=current['reference_id']
        return current

    def _install_reference(self,features,covariance,parent):
        if self._defer_reference_admission:
            self._reference_admission_requested=True
            return
        fitted=self._frame_fits.get(parent)
        existing=self._registered_view(features,fitted) if fitted is not None else None
        if existing is not None:
            self._activate(existing)
            return
        reference=self._reference(features,covariance,parent)
        validation,reason=self._fit(features,self.xyz,reference)
        self._reference_build_attempts.append(dict(reference_id=reference['id'],parent=parent,reason=reason))
        if validation is None:return
        self._retire_redundant_reference(parent,incoming=reference)
        self.last_reference=(self.xyz.copy(),self.yaw)
        self.last_reference_view=features['points'].mean(axis=0)
        self.last_reference_overlap=validation['overlap']
        reference['self_overlap']=validation['overlap']
        self._active_view_id=reference['id']

    def update(self, stamp, gray_image, points_odom_orientation, yaw, wheel_vx=None, wheel_wz=None,
               camera_model=None):
        snapshot=(self.xyz.copy(),list(self.references),self.reference_number,
            self.last_reference,self.last_reference_view,self.last_reference_overlap,self._active_view_id)
        self._current_features=None;self._frame_fits={};self._frame_prediction=None
        self._entry_reference_ids={ref['id'] for ref in self.references}
        self._entry_latest_reference_id=self.references[-1]['id'] if self.references else None
        self._refresh_latest_template=False
        self._reference_admission_requested=False;self._reference_build_attempts=[];self._anchor_adoption={}
        self._defer_reference_admission=True
        try:
            result=super().update(stamp,gray_image,points_odom_orientation,yaw,wheel_vx,wheel_wz,
                                  camera_model=camera_model)
            self._defer_reference_admission=False
            result['wheel_prediction_used']=bool(self._wheel_prediction_used)
            result['reference_attempts']=[dict(attempt) for attempt in self._attempted_references]
            if not result['accepted'] or not self.references:return result
            features=self._current_features
            if self.last_camera_model is None:
                if self._reference_admission_requested:
                    result=self._prefer_measured_anchor(features,result)
                    self._install_reference(features,result['covariance'],result['reference_id'])
                    result['reference_frames']=len(self.references)
                    result['reference_build_failures']=sum(item['reason']!='accepted' for item in self._reference_build_attempts)
                return result
            for reference in self.references:
                if 'self_overlap' not in reference:reference['self_overlap']=result['overlap']
            selected=next((ref for ref in self.references+snapshot[1] if ref['id']==result['reference_id']),None)
            template_handoff=self._latest_template_handoff(selected)
            self._refresh_latest_template=template_handoff['needed']
            if selected is not None and self._near_view(selected):self._activate(selected)
            # All maintenance evidence belongs to this exposure's actual fit.
            # An unfitted/stale active view is eligible only after the existing
            # independent registered-view consistency check proves reuse.
            previous=selected if selected is not None else self.references[-1]
            model=previous.get('camera_model')
            if model is None or 'self_overlap' not in previous:
                if self._reference_admission_requested:
                    result=self._prefer_measured_anchor(features,result)
                    self._install_reference(features,result['covariance'],result['reference_id'])
                    result['reference_frames']=len(self.references)
                    result['reference_build_failures']=sum(item['reason']!='accepted' for item in self._reference_build_attempts)
                return result
            reference_overlap=previous['self_overlap']
            angle=self._angle(model['rotation'],self.last_camera_model['rotation'])
            motion=self._projected_view_motion(previous)
            shift=motion['projected_median_shift_px']
            parallax_changed=shift is not None and shift>=.5*self._orb.getPatchSize()
            support_handoff=self._support_handoff(previous)
            support_handoff['reference_id']=previous['id']
            rotation_limit=min(self._view_rotation_limit(model),self._view_rotation_limit(self.last_camera_model))
            if (angle<rotation_limit and not parallax_changed and not support_handoff['needed']
                    and not self._reference_admission_requested and not self._refresh_latest_template):return result
            optical=(features['points']+self.xyz-previous['xyz']-model['translation'])@model['rotation']
            image=optical@model['K'].T
            pixels=image[:,:2]/image[:,2:]-.5
            height,width=previous['image_shape']
            outside=((optical[:,2]<=.1)|(pixels[:,0]<0)|(pixels[:,0]>=width)
                |(pixels[:,1]<0)|(pixels[:,1]>=height))
            novelty=int(outside.sum())
            changed=(self._reference_admission_requested or self._refresh_latest_template or parallax_changed or support_handoff['needed'] or result['overlap']<reference_overlap or
                     (novelty>=24 and novelty>=.10*len(features['points'])))
            result['view_admission']=dict(angle_rad=angle,projected_reference_motion=motion,
                support_handoff=support_handoff,latest_template_handoff=template_handoff,
                new_measured_features_outside_old_rgb=novelty,
                feature_count=len(features['points']),overlap_declined=result['overlap']<reference_overlap)
            if changed:
                result=self._prefer_measured_anchor(features,result)
                self._install_reference(features,result['covariance'],result['reference_id'])
                result['reference_frames']=len(self.references)
                result['reference_build_failures']=sum(item['reason']!='accepted' for item in self._reference_build_attempts)
                result['view_admission']['anchor_adoption']=dict(self._anchor_adoption)
                result['view_admission']['reference_build_attempts']=[dict(item) for item in self._reference_build_attempts]
            return result
        except _ConflictingRegisteredView:
            # A rolled-back reference number can later be reused. Cached
            # geometry from the abandoned transaction must never survive.
            self._retention_coverage={};self._retention_references={}
            (self.xyz,refs,self.reference_number,self.last_reference,self.last_reference_view,
                self.last_reference_overlap,self._active_view_id)=snapshot
            self.references[:]=refs
            result=self._reject('ambiguous_rgbd_reference_translation')
            result['wheel_prediction_used']=bool(self._wheel_prediction_used)
            result['reference_attempts']=[dict(attempt) for attempt in self._attempted_references]
            return result



class MetricRGBDOdometry(_MetricRGBDReferenceMaintenance):
    """Reserve a measured template when consecutive identity support is fading.

    This is bounded reference maintenance only. It never changes the original
    identity, metric, uncertainty, conflict, or source quality acceptance rules.
    History means a complete core accepted transaction, not a wrapper publish ACK.
    """
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self._identity_history=None
        self._identity_previous=None
        self._identity_motion_cache=None
        self._identity_refresh=False
        self._identity_signal_suspended=False
        self._identity_cancelled_after_adoption=False
        self._identity_skip_admission=False
        self._identity_only_admission=False
        self._identity_trigger_evidence=None
        self._identity_trigger_handoff=None
        self._identity_adjacent_match_count=0
        self._identity_depleted_reference_id=None
        self._recent_measurement=None
        self._recent_fit_calls=0

    def _fit(self,features,prediction,reference):
        self._recent_fit_calls+=1
        return super()._fit(features,prediction,reference)

    def _retain_recent_measurement(self,result,features,stamp_ns):
        required=('descriptors','pixels','points','covariance','pixel_scales','_measured_gray','_measured_grid','_measurement_token')
        if self.last_camera_model is None or not all(key in features for key in required):
            self._recent_measurement=None;return
        # Feature arrays and enhanced gray are produced/owned internally. The
        # measured grid may alias caller input, so detach that one raw array.
        owned=dict(features);owned['_measured_grid']=features['_measured_grid'].copy()
        self._recent_measurement=dict(stamp_ns=stamp_ns,features=owned,xyz=result['xyz'].copy(),
            covariance=result['covariance'].copy(),yaw=float(result['yaw']),camera_model=self.last_camera_model,
            reference_id=result['reference_id'])

    def _try_recent_measurement(self,rejected,features,stamp_ns,entry_ns,cadence):
        record=self._recent_measurement;attempts=rejected.get('reference_attempts',[])
        if (record is None or features is None or not cadence or record['stamp_ns']!=entry_ns
                or len(attempts)!=5 or any(item['accepted'] for item in attempts)
                or any('ambiguous' in item['reason'] for item in attempts)
                or not self._identity_same_calibration(dict(image_shape=tuple(record['features']['image_shape']),
                    K=record['camera_model']['K']),features,self.last_camera_model)):
            return rejected
        allowed={'insufficient_rgbd_identity_matches','rgbd_correspondence_consensus','insufficient_rgbd_spatial_support',
                 'rgbd_correction_jump','rgbd_registration_residual','uncertain_rgbd_translation'}
        if any(item['reason'] not in allowed for item in attempts):return rejected
        predicted=self._frame_prediction.copy();number=self.reference_number+1
        # Explicit past measurement fields: the normal constructor uses the
        # current yaw/model and must not relabel this record as a current view.
        reference=dict(id=number,parent=record['reference_id'],xyz=record['xyz'].copy(),yaw=record['yaw'],
            world=record['features']['points']+record['xyz'],anchor_covariance=record['covariance'].copy(),
            camera_model=record['camera_model'],measurement_stamp_ns=record['stamp_ns'],**record['features'])
        evidence=dict(measurement_exposure_stamp=record['stamp_ns']/1e9,current_exposure_stamp=stamp_ns/1e9,
                      candidate_reference_id=number,registered=False)
        rejected['recent_reference']=evidence
        validation,reason=self._fit(record['features'],record['xyz'],reference)
        evidence['self_validation_reason']=reason
        if validation is None:
            self._frame_fits.pop(number,None);evidence['reason']='previous_self_rejected';return rejected
        fitted,reason=self._fit(features,predicted,reference)
        evidence['cross_frame_reason']=reason
        if fitted is None:
            self._frame_fits.pop(number,None);evidence['reason']='current_previous_fit_rejected';return rejected
        old_xyz=self.xyz.copy();self.xyz=fitted['xyz'].copy()
        try:
            nearby=[ref for ref in self.references if self._angular_near_view(ref)]
            def proximity(ref):
                shift=self._projected_view_motion(ref)['projected_median_shift_px']
                return (math.inf if shift is None else shift,
                        self._angle(ref['camera_model']['rotation'],self.last_camera_model['rotation']))
            nearby.sort(key=proximity);complete={number:fitted};checked=[]
            for ref in nearby[:2]:
                if ref['id'] in self._frame_fits:fit=self._frame_fits[ref['id']]
                else:fit,_=self._fit(features,predicted,ref)
                checked.append(ref['id'])
                if fit is not None:complete[ref['id']]=fit
            maximum=max(fit['inliers'] for fit in complete.values());values=list(complete.values())
            for index,left in enumerate(values):
                for right in values[index+1:]:
                    delta=left['xyz']-right['xyz']
                    if (min(left['inliers'],right['inliers'])>=.60*maximum
                            and delta@np.linalg.solve(left['covariance']+right['covariance'],delta)>self.residual_chi_square):
                        self.xyz=old_xyz;self._frame_fits.pop(number,None)
                        failed=self._reject('ambiguous_rgbd_reference_translation')
                        failed.update(wheel_prediction_used=bool(self._wheel_prediction_used),reference_attempts=attempts,
                                      recent_reference=dict(evidence,reason='registered_view_conflict'))
                        return failed
            reference['self_overlap']=validation['overlap']
            self._retire_redundant_reference(record['reference_id'],incoming=reference)
            self.reference_number=number;self._activate(reference)
            evidence.update(reason='accepted',registered=True,registered_conflict_reference_ids=checked,
                            total_fit_calls=self._recent_fit_calls,retired_reference_id=self._anchor_adoption.get('retired_reference_id'))
            self._attempted_references.append(dict(reference_id=number,reason='accepted',accepted=True))
            return dict(accepted=True,reason='accepted',stamp=stamp_ns/1e9,yaw=self.yaw,
                        reference_frames=len(self.references),reference_pending=False,reference_build_failures=0,
                        wheel_prediction_used=bool(self._wheel_prediction_used),reference_attempts=[dict(item) for item in self._attempted_references],
                        recent_reference=evidence,view_admission=dict(anchor_adoption=dict(self._anchor_adoption),
                            recent_reference_registration=dict(reference_id=number,measurement_exposure_stamp=record['stamp_ns']/1e9,
                                                               parent=record['reference_id'])),**fitted)
        except Exception:
            self.xyz=old_xyz;self._frame_fits.pop(number,None);raise

    @staticmethod
    def _identity_same_calibration(history,features,model):
        return (model is not None and tuple(features['image_shape'])==history['image_shape']
                and np.array_equal(model['K'],history['K']))

    def _identity_primary_tracking_fit(self):
        # Use the same genuine tracking selection as the original estimator;
        # exclude maintenance probes and this exposure's self-validation.
        fits=[self._frame_fits.get(attempt['reference_id']) for attempt in self._attempted_references
              if attempt['accepted'] and attempt['reference_id'] in self._entry_reference_ids]
        fits=[fit for fit in fits if fit is not None]
        return min(fits,key=lambda fit:np.linalg.eigvalsh(fit['covariance'][:2,:2]).max()) if fits else None

    def _identity_support_reserve(self,reference):
        evidence=dict(needed=False,stage='original_post_density_identity_matches',reference_id=reference['id'],
                      minimum_matches=self.minimum_matches)
        if self._identity_cancelled_after_adoption:return evidence
        previous=self._identity_previous
        fitted=self._identity_primary_tracking_fit()
        if (previous is None or fitted is None or self._current_features is None
                or reference['id']!=previous['reference_id'] or fitted['reference_id']!=reference['id']):
            return evidence
        features=self._current_features
        if not self._identity_same_calibration(previous,features,self.last_camera_model):return evidence
        count=int(fitted['identity_matches']);last=previous['identity_matches'];decline=last-count
        evidence.update(previous_identity_matches=last,current_identity_matches=count,
                        observed_decline=decline,one_more_same_decline_count=count-decline)
        if decline<=0 or count-decline>=self.minimum_matches:return evidence
        if self._identity_motion_cache is None:
            # One original raw-ORB adjacent match only, after reserve demand.
            # No geometry prediction or future image supplies feature identity.
            pairs=self._matches(features,previous['features'])
            self._identity_adjacent_match_count+=1
            motion=dict(adjacent_identity_pairs=len(pairs),movement_resolved=False,
                        policy='median_actual_pixel_step_over_pair_3sigma_rss')
            if len(pairs)>=self.minimum_matches:
                ci,pi=pairs.T
                step=np.linalg.norm(features['pixels'][ci]-previous['features']['pixels'][pi],axis=1)
                scales=np.column_stack((features['pixel_scales'][ci],previous['features']['pixel_scales'][pi]))
                if np.isfinite(step).all() and np.isfinite(scales).all() and np.all(scales>=1.):
                    budget=3*self.pixel_noise*np.linalg.norm(scales,axis=1)
                    ratio=step/budget
                    motion.update(median_step_px=float(np.median(step)),median_pair_3sigma_px=float(np.median(budget)),
                                  median_step_over_pair_3sigma=float(np.median(ratio)),
                                  pairs_over_pair_3sigma=int((ratio>1.).sum()),
                                  movement_resolved=bool(np.median(ratio)>1.))
            self._identity_motion_cache=motion
        evidence['adjacent_motion']=dict(self._identity_motion_cache)
        evidence['needed']=self._identity_motion_cache['movement_resolved']
        if evidence['needed']:
            self._identity_refresh=True
            self._identity_depleted_reference_id=reference['id']
            self._identity_only_admission=not self._identity_original_maintenance(reference,fitted)
        return evidence

    def _identity_original_maintenance(self,reference,fitted):
        """Whether the unchanged policy already requests this admission."""
        model=reference.get('camera_model')
        if model is None or self.last_camera_model is None:return self._reference_admission_requested
        angle=self._angle(model['rotation'],self.last_camera_model['rotation'])
        motion=self._projected_view_motion(reference)
        shift=motion['projected_median_shift_px']
        parallax=shift is not None and shift>=.5*self._orb.getPatchSize()
        support=super()._support_handoff(reference)['needed']
        rotation_limit=min(self._view_rotation_limit(model),self._view_rotation_limit(self.last_camera_model))
        outer=(angle>=rotation_limit or parallax or support or self._reference_admission_requested or self._refresh_latest_template)
        features=self._current_features
        optical=(features['points']+self.xyz-reference['xyz']-model['translation'])@model['rotation']
        image=optical@model['K'].T
        pixels=image[:,:2]/image[:,2:]-.5
        height,width=reference['image_shape']
        novelty=int(((optical[:,2]<=.1)|(pixels[:,0]<0)|(pixels[:,0]>=width)|(pixels[:,1]<0)|(pixels[:,1]>=height)).sum())
        changed=(self._reference_admission_requested or self._refresh_latest_template or parallax or support
                 or fitted['overlap']<reference['self_overlap'] or (novelty>=24 and novelty>=.10*len(features['points'])))
        return bool(outer and changed)

    def _support_handoff(self,reference):
        result=super()._support_handoff(reference)
        if self._identity_signal_suspended:return result
        identity=self._identity_support_reserve(reference)
        result['identity_support_reserve']=identity
        if identity['needed']:
            self._identity_trigger_evidence=identity
            self._identity_trigger_handoff=result
            identity['original_support_handoff_needed']=result['needed']
        result['needed']=result['needed'] or identity['needed']
        return result

    def _prefer_measured_anchor(self,features,current):
        result=super()._prefer_measured_anchor(features,current)
        previous=self._identity_previous
        if self._identity_refresh and previous is not None and result['reference_id']!=previous['reference_id']:
            # The final genuinely adopted reference is a different measurement.
            # Cancel A's trend, never attach A's count or cause to final B.
            self._identity_cancelled_after_adoption=True
            self._identity_refresh=False
            self._identity_skip_admission=self._identity_only_admission
            if self._identity_trigger_evidence is not None:
                self._identity_trigger_evidence.update(needed=False,cancelled_after_anchor_adoption=True,
                    final_adopted_reference_id=result['reference_id'])
            if self._identity_trigger_handoff is not None:
                self._identity_trigger_handoff['needed']=self._identity_trigger_evidence['original_support_handoff_needed']
        return result

    def _install_reference(self,features,covariance,parent):
        if not self._defer_reference_admission and self._identity_skip_admission:return
        return super()._install_reference(features,covariance,parent)

    def _identity_current_template(self,reference):
        # A truly current, independently accepted existing template may still
        # be reused. Do not globally forbid all old views when reserve triggers.
        self._identity_signal_suspended=True
        try:near=super()._near_view(reference)
        finally:self._identity_signal_suspended=False
        if not near or reference['id'] not in self._entry_reference_ids:return False
        fitted=self._frame_fits.get(reference['id'])
        if fitted is None:return False
        scales=fitted.get('reference_support',{}).get('identity_pixel_scale_max',{})
        values=np.array([scales.get('current',np.nan),scales.get('reference',np.nan)],dtype=float)
        if not np.isfinite(values).all() or np.any(values<1.):return False
        motion=self._projected_view_motion(reference)
        shift=motion['projected_median_shift_px']
        return shift is not None and shift<=3*self.pixel_noise*np.linalg.norm(values)

    def _near_view(self,reference):
        if self._identity_refresh and reference['id']==self._identity_depleted_reference_id:
            return self._identity_current_template(reference)
        result=super()._near_view(reference)
        # The original call can discover reserve through _support_handoff.
        return (self._identity_current_template(reference)
                if self._identity_refresh and reference['id']==self._identity_depleted_reference_id else result)

    def close(self,wait=True):
        self._identity_history=None;self._identity_previous=None;self._identity_motion_cache=None
        self._identity_refresh=False
        self._identity_cancelled_after_adoption=False;self._identity_skip_admission=False
        self._recent_measurement=None
        return super().close(wait=wait)

    def _update_identity_transaction(self,stamp,gray_image,points_odom_orientation,yaw,wheel_vx=None,wheel_wz=None,camera_model=None):
        previous=self._identity_history
        self._identity_previous=None;self._identity_motion_cache=None;self._identity_refresh=False
        self._identity_signal_suspended=False
        self._identity_cancelled_after_adoption=False;self._identity_skip_admission=False
        self._identity_only_admission=False;self._identity_trigger_evidence=None;self._identity_trigger_handoff=None
        self._identity_adjacent_match_count=0
        self._identity_depleted_reference_id=None
        stamp_ns=round(float(stamp)*1e9) if np.isfinite(stamp) else None
        entry_ns=round(float(self.stamp)*1e9) if self.stamp is not None else None
        consecutive=(previous is not None and bool(self.references) and stamp_ns is not None
                     and entry_ns==previous['stamp_ns'] and stamp_ns>entry_ns)
        interval=stamp_ns-entry_ns if consecutive else None
        cadence=(consecutive and previous['interval_ns'] is not None and interval==previous['interval_ns'])
        next_interval=interval if consecutive and (previous['interval_ns'] is None or cadence) else None
        if cadence:self._identity_previous=previous
        try:
            result=super().update(stamp,gray_image,points_odom_orientation,yaw,wheel_vx,wheel_wz,camera_model=camera_model)
        except Exception:
            self._identity_history=None;self._identity_previous=None;self._identity_motion_cache=None
            raise
        features=self._current_features
        if not result['accepted']:
            result=self._try_recent_measurement(result,features,stamp_ns,entry_ns,cadence)
        if not result['accepted'] or features is None or self.last_camera_model is None:
            self._identity_history=None;self._identity_previous=None
            self._recent_measurement=None
            return result
        if previous is not None and not self._identity_same_calibration(previous,features,self.last_camera_model):next_interval=None
        count=result.get('identity_matches')
        if count is None or int(count)<self.minimum_matches or not all(key in features for key in ('descriptors','pixels','pixel_scales')):
            self._identity_history=None;self._identity_previous=None
            self._recent_measurement=None
            return result
        self._retain_recent_measurement(result,features,stamp_ns)
        # Commit only the complete final adopted fit. Never mix the primary
        # tracking count with an adopted reference ID, or preserve a reject.
        raw={key:features[key].copy() for key in ('descriptors','pixels','pixel_scales')}
        self._identity_history=dict(stamp_ns=stamp_ns,interval_ns=next_interval,
            reference_id=result['reference_id'],identity_matches=int(count),features=raw,
            image_shape=tuple(features['image_shape']),K=self.last_camera_model['K'].copy())
        self._identity_previous=None
        return result

    def update(self,stamp,gray_image,points_odom_orientation,yaw,wheel_vx=None,wheel_wz=None,camera_model=None):
        self._recent_fit_calls=0
        # Geometry is immutable inside this serial transaction. Different pose,
        # model or reference identities have distinct keys; no cross-frame reuse.
        self._view_motion_cache={}
        snapshot=(self.xyz.copy(),list(self.references),self.reference_number,self.last_reference,
                  self.last_reference_view,self.last_reference_overlap,self._active_view_id)
        try:
            return self._update_identity_transaction(stamp,gray_image,points_odom_orientation,yaw,wheel_vx,wheel_wz,camera_model=camera_model)
        except Exception:
            # Also covers input normalization, calibration checks and the final
            # owned snapshot, not only exceptions thrown inside the old update.
            self._identity_history=None;self._identity_previous=None;self._identity_motion_cache=None
            self._identity_refresh=False;self._identity_depleted_reference_id=None
            (self.xyz,refs,self.reference_number,self.last_reference,self.last_reference_view,self.last_reference_overlap,self._active_view_id)=snapshot
            self.references[:]=refs;self._retention_coverage={};self._retention_references={}
            self._recent_measurement=None
            raise
        finally:
            self._view_motion_cache=None
