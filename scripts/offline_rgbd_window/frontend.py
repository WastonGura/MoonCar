"""Bounded persistent pyramidal forward/backward KLT tracks for offline replay.

ORB detects only new points outside surviving tracks' sampling neighborhoods.
Identity is propagated by measured image motion, not repeated ORB detection or
the production ORB-seed refinement model. Production depth interpolation/noise,
metric consensus and spatial support remain guarded. Neither production poses
nor truth are used to invent identity or constrain the new backend.
"""
from collections import Counter, deque
import heapq
from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.spatial import cKDTree

SLAM_SCRIPTS = Path(__file__).resolve().parents[2] / 'ros2_ws/src/mooncar_slam/scripts'
sys.path.insert(0, str(SLAM_SCRIPTS))
from metric_rgbd_odometry import _MetricRGBDReferenceMaintenance


class TrackFrontend:
    MIN_TRACK_SPACING_PX = 4.  # Exclusion radius from the 4x4 sampling footprint.
    RECOVERY_MAX_AGE_NS = 1_000_000_000
    _PATCH_OFFSETS = np.stack(np.meshgrid(np.arange(-10, 11), np.arange(-10, 11)), axis=-1).reshape(-1, 2)
    _PATCH_OFFSETS.flags.writeable = False
    _PATCH_BATCH = 64

    def __init__(self, max_observations=96, max_frames=6, track_budget=None):
        self.measurements = _MetricRGBDReferenceMaintenance()
        self.max_observations = int(max_observations)
        if track_budget is not None and (type(track_budget) is not int
                or not self.max_observations+24 <= track_budget <= self.measurements._orb.getMaxFeatures()):
            raise ValueError('track budget must leave observation and replenishment capacity')
        self.track_budget = track_budget
        self._tracking_submissions = {}
        self._recovery_frames = deque(maxlen=2)
        self.previous = None
        self.keyframes = deque(maxlen=max_frames)
        self.next_track = 0
        self.age = {}
        self.stats = Counter()
        self.last_tracking = {}

    def protect_submission(self, request, track_ids):
        """Protect accepted running/latest-pending identities, never invent them."""
        self._remember_reference()
        if self.track_budget is None:
            return
        self._tracking_submissions[int(request)] = frozenset(int(t) for t in track_ids)
        # A failed job can hand off to its pending successor between poll and
        # offer without advancing the cache version. Keep one extra generation
        # so that this newly running job is not mistaken for a replaced pending
        # job. Completion notices retire stale protection on the next frame.
        if len(self._tracking_submissions) > 3:
            ordered = sorted(self._tracking_submissions)
            keep = (ordered[0], *ordered[-2:])
            self._tracking_submissions = {r:self._tracking_submissions[r] for r in keep}

    def complete_submissions(self, request):
        self._tracking_submissions = {r:ids for r,ids in self._tracking_submissions.items() if r > request}

    def _remember_reference(self, features=None):
        features=features if features is not None else (None if self.previous is None else self.previous[0])
        if features is None or features.get('_exposure_stamp_ns') is None:return
        if self._recovery_frames and self._recovery_frames[-1]['_exposure_stamp_ns']==features['_exposure_stamp_ns']:return
        # Arrays are measured snapshots: subsequent tracking/budgeting creates
        # new arrays, never writes into these images or feature tables.
        self._recovery_frames.append(dict(features))

    def _recover_tracks(self, gray, grid, camera_model, stamp_ns):
        if stamp_ns is None:return None
        active={int(t) for frame,_ in self.keyframes for t in frame['submitted_ids']}
        ordinary=self.last_tracking;attempts=[];previous=self.previous
        try:
            for reference in reversed(self._recovery_frames):
                age=stamp_ns-reference['_exposure_stamp_ns']
                if not 0<age<=self.RECOVERY_MAX_AGE_NS:continue
                selected=np.array([i for i,t in enumerate(reference['track_ids']) if int(t) in active],dtype=int)
                if len(selected)<24:continue
                bounded=dict(reference)
                for name in ('pixels','points','covariance','pixel_scales','track_ids','descriptors','descriptor_valid'):
                    bounded[name]=reference[name][selected]
                self.previous=(bounded,bounded['track_ids'])
                self.last_tracking=dict(attempted=0,consensus=0,spatial_adequate=False)
                recovered=self._track(gray,grid,camera_model)
                diagnostic=self.last_tracking
                success=recovered is not None and recovered['tracking_pose_adequate'] and len(recovered['track_ids'])>=24
                attempts.append(dict(reference_stamp_ns=reference['_exposure_stamp_ns'],map_tracks=len(selected),
                    valid_tracks=0 if recovered is None else len(recovered['track_ids']),accepted=bool(success)))
                if success:
                    diagnostic.update({k:v for k,v in ordinary.items() if k.startswith('budget_')})
                    diagnostic.update(recovery_used=True,recovery_attempts=attempts,
                        initial_spatial_adequate=ordinary.get('spatial_adequate'),recovery_reference_stamp_ns=reference['_exposure_stamp_ns'])
                    self.last_tracking=diagnostic
                    return recovered
        finally:
            self.previous=previous
            if not self.last_tracking.get('recovery_used'):
                self.last_tracking=ordinary
                if attempts:self.last_tracking.update(recovery_used=False,recovery_attempts=attempts)
        return None

    def _budget_previous(self):
        """Bound old points before LK/patch/depth work, retaining real row data."""
        previous = self.previous[0]
        ids = previous['track_ids']
        priorities = {}
        for rank, (frame, _) in enumerate(reversed(self.keyframes)):
            for track in frame['submitted_ids']:
                priorities.setdefault(int(track), rank)
        for request in sorted(self._tracking_submissions):
            for track in self._tracking_submissions[request]:
                priorities[int(track)] = -1
        protected = [i for i,t in enumerate(ids) if int(t) in priorities]
        reserve = self.max_observations
        target = max(self.track_budget-reserve, min(len(protected), self.track_budget))
        roi = previous['measured_roi']
        cells = np.clip(((previous['pixels']-roi[:2])/(roi[2:]-roi[:2])*[8,6]).astype(int), 0, [7,5])
        cell_ids = cells[:,1]*8+cells[:,0]
        traces = np.trace(previous['covariance'], axis1=1, axis2=2)
        order = lambda i: (priorities.get(int(ids[i]), len(self.keyframes)),
            -self.age.get(int(ids[i]),0), float(traces[i]), int(ids[i]))
        chosen, occupancy = [], Counter()
        for pool in (protected, [i for i,t in enumerate(ids) if int(t) not in priorities]):
            buckets = {}
            for i in sorted(pool, key=order):
                buckets.setdefault(int(cell_ids[i]), deque()).append(i)
            heap = [(occupancy[c],c) for c in buckets]
            heapq.heapify(heap)
            while heap and len(chosen) < target:
                count, cell = heapq.heappop(heap)
                chosen.append(buckets[cell].popleft())
                occupancy[cell] += 1
                if buckets[cell]:
                    heapq.heappush(heap, (count+1,cell))
        indices = np.asarray(sorted(chosen), dtype=int)
        bounded = dict(previous)
        for key in ('pixels','points','covariance','pixel_scales','track_ids','descriptors','descriptor_valid'):
            bounded[key] = previous[key][indices]
        self.last_tracking.update(budget_limit=self.track_budget, budget_input=len(ids),
            budget_kept=len(indices), budget_protected=len(protected),
            budget_kept_protected=sum(int(ids[i]) in priorities for i in indices),
            budget_replenishment_capacity=self.track_budget-len(indices))
        return bounded, bounded['track_ids'].copy()

    def _pairs(self, current, previous):
        """The same strict two-sided ORB identity rule, before density capping."""
        # Persistent LK rows intentionally have no current ORB descriptor.
        # They may never accidentally be matched as a zero-filled descriptor.
        left_ids = np.flatnonzero(current.get('descriptor_valid', np.ones(len(current['descriptors']), dtype=bool)))
        right_ids = np.flatnonzero(previous.get('descriptor_valid', np.ones(len(previous['descriptors']), dtype=bool)))
        left, right = current['descriptors'][left_ids], previous['descriptors'][right_ids]
        if min(len(left), len(right)) < 2:
            return np.empty((0, 2), dtype=int)
        forward = self.measurements._matcher.knnMatch(left, right, k=2)
        backward = self.measurements._matcher.knnMatch(right, left, k=2)
        reverse = {p[0].queryIdx: p[0].trainIdx for p in backward
                   if len(p) == 2 and p[0].distance <= 48 and p[0].distance < .70*p[1].distance}
        pairs = [(int(left_ids[p[0].queryIdx]), int(right_ids[p[0].trainIdx])) for p in forward
                 if len(p) == 2 and p[0].distance <= 48 and p[0].distance < .70*p[1].distance
                 and reverse.get(p[0].trainIdx) == p[0].queryIdx]
        return np.asarray(pairs, dtype=int).reshape(-1, 2)

    def _consensus(self, current, previous, pairs):
        if len(pairs) < 24:
            return np.empty((0, 2), dtype=int)
        ci, pi = pairs.T
        delta = previous['points'][pi] - current['points'][ci]
        covariance = previous['covariance'][pi] + current['covariance'][ci]
        # Attitude error is shared in the backend. Here it only widens an
        # outlier gate; this gate supplies no independent pose measurement.
        for points in (previous['points'][pi], current['points'][ci]):
            covariance += np.deg2rad(.1)**2 * (
                np.sum(points**2, axis=1)[:, None, None]*np.eye(3)
                - np.einsum('ni,nj->nij', points, points))
        covariance += 2*.003**2*np.eye(3)
        information = np.linalg.inv(covariance)
        hypotheses = np.vstack((np.median(delta, axis=0), delta[np.linspace(0, len(delta)-1, min(64, len(delta)), dtype=int)]))
        residual = delta[None] - hypotheses[:, None]
        masks = np.einsum('hni,nij,hnj->hn', residual, information, residual) <= self.measurements.residual_chi_square
        inlier = masks[np.argmax(masks.sum(axis=1))]
        if inlier.sum() < 24 or inlier.mean() < .55:
            return np.empty((0, 2), dtype=int)
        normal = information[inlier].sum(axis=0)
        center = np.linalg.solve(normal, np.einsum('nij,nj->i', information[inlier], delta[inlier]))
        residual = delta-center
        inlier = np.einsum('ni,nij,nj->n', residual, information, residual) <= self.measurements.residual_chi_square
        if inlier.sum() < 24 or inlier.mean() < .55:
            return np.empty((0, 2), dtype=int)
        self.stats['geometric_outliers'] += int((~inlier).sum())
        return pairs[inlier]

    @staticmethod
    def _sample(image, sites):
        xy = np.floor(sites).astype(int)
        x, y = xy[..., 0], xy[..., 1]
        a, b = (sites-xy)[..., 0], (sites-xy)[..., 1]
        return ((1-a)*(1-b)*image[y, x]+a*(1-b)*image[y, x+1]
                +(1-a)*b*image[y+1, x]+a*b*image[y+1, x+1])

    @classmethod
    def _sample_patch(cls, image, pixels):
        """Sample the unchanged 21x21 photometric footprint with shared weights.

        Standard LK centers are float32 and already inside the twelve-pixel
        border. Their integer offsets are exact in float64, so all 441 sites
        have the center's fractional coordinates. Keep the original sampler
        for other inputs rather than changing its rounding or boundary rules.
        """
        offset = cls._PATCH_OFFSETS
        if not (type(image) is np.ndarray and image.dtype == np.uint8 and image.ndim == 2
                and max(image.shape) <= 2**31
                and type(pixels) is np.ndarray and pixels.dtype == np.float32
                and pixels.ndim == 2 and pixels.shape[1] == 2
                and np.isfinite(pixels).all() and np.all(pixels >= 12.)
                and np.all(pixels[:, 0]+12. < image.shape[1])
                and np.all(pixels[:, 1]+12. < image.shape[0])):
            return cls._sample(image, pixels[:, None]+offset)
        # Bound the gather/arithmetic working set independently of track count.
        # The element-wise float64 operations and their order stay unchanged;
        # only the batch size changes, not interpolation or patch footprint.
        result = np.empty((len(pixels), len(offset)), dtype=np.float64)
        for start in range(0, len(pixels), cls._PATCH_BATCH):
            centers = pixels[start:start+cls._PATCH_BATCH]
            xy = np.floor(centers).astype(int)
            x = xy[:, :1]+offset[:, 0]
            y = xy[:, 1:]+offset[:, 1]
            fraction = centers.astype(np.float64)-xy
            a, b = fraction[:, :1], fraction[:, 1:]
            result[start:start+len(centers)] = (
                (1-a)*(1-b)*image[y, x]+a*(1-b)*image[y, x+1]
                +(1-a)*b*image[y+1, x]+a*b*image[y+1, x+1])
        return result

    def _sample_tracks(self, grid, pixels, scales):
        """The production 4x4/continuity/bilinear/gradient measurement model."""
        height, width = grid.shape[:2]
        finite = np.isfinite(pixels).all(axis=1)
        safe = np.where(finite[:, None], pixels, 0.)
        xy = np.floor(safe).astype(int)
        x, y = xy.T
        valid = finite & (x >= 1) & (y >= 1) & (x+2 < width) & (y+2 < height)
        which = np.flatnonzero(valid)
        x, y = x[which], y[which]
        patch = grid[y[:, None, None]+np.arange(-1, 3)[None, :, None],
                     x[:, None, None]+np.arange(-1, 3)[None, None, :]]
        continuous = (np.isfinite(patch).all(axis=(1, 2, 3))
            & (np.max(np.linalg.norm(patch-patch[:, 1:2, 1:2], axis=3), axis=(1, 2)) <= .15))
        which, x, y = which[continuous], x[continuous], y[continuous]
        a, b = (pixels[which]-np.column_stack((x, y))).T
        points = (((1-a)*(1-b))[:, None]*grid[y, x]+(a*(1-b))[:, None]*grid[y, x+1]
                  +((1-a)*b)[:, None]*grid[y+1, x]+(a*b)[:, None]*grid[y+1, x+1])
        du, dv = (grid[y, x+1]-grid[y, x-1])*.5, (grid[y+1, x]-grid[y-1, x])*.5
        covariance = (self.measurements.noise_m**2*np.eye(3)[None]
            + self.measurements.pixel_noise**2*scales[which, None, None]**2*
            (np.einsum('ni,nj->nij', du, du)+np.einsum('ni,nj->nij', dv, dv)))
        return which, points, covariance

    def _spaced(self, pixels, order, initial=()):
        kept, initial = [], [np.asarray(p, dtype=float) for p in initial]
        limit = self.measurements._orb.getMaxFeatures()
        if self.track_budget is not None:
            limit = min(limit, self.track_budget)
        count = len(initial)
        if count >= limit:
            return np.empty(0, dtype=int)
        # Preserve the original greedy order, arithmetic dtype and strict
        # distance test. Only avoid rebuilding every occupied row per query.
        dtype = np.result_type(pixels.dtype, np.float64) if count else pixels.dtype
        occupied = np.empty((limit, pixels.shape[1]), dtype=dtype)
        if count:
            occupied[:count] = initial
        if (pixels.ndim == 2 and pixels.shape[1] == 2
                and pixels.dtype in (np.dtype('float32'), np.dtype('float64'))
                and self.MIN_TRACK_SPACING_PX == 4.
                and np.isfinite(pixels).all()
                and (not count or np.isfinite(occupied[:count]).all())):
            # The axis-aligned four-pixel box is only a conservative broad
            # phase. Every possible edge is checked by the original dtype,
            # coordinate sum order and strict squared-distance comparison.
            all_points = np.concatenate((occupied[:count], pixels.astype(dtype, copy=False)))
            possible = cKDTree(all_points).query_pairs(4., p=np.inf, output_type='ndarray')
            difference = all_points[possible[:, 0]]-all_points[possible[:, 1]]
            pairs = possible[np.sum(difference**2, axis=1) < self.MIN_TRACK_SPACING_PX**2]
            neighbors = [[] for _ in range(len(all_points))]
            for left, right in pairs:
                neighbors[left].append(right)
                neighbors[right].append(left)
            active = np.zeros(len(all_points), dtype=bool)
            active[:count] = True
            initial_count = count
            for index in order:
                # Preserve NumPy's original out-of-range/noninteger errors
                # before canonicalizing a valid negative row index.
                _ = pixels[index]
                slot = initial_count + (int(index) % len(pixels))
                if active[slot] or any(active[other] for other in neighbors[slot]):
                    continue
                kept.append(index)
                active[slot] = True
                count += 1
                if count >= limit:
                    break
            return np.asarray(kept, dtype=int)
        # Keep the original behavior for unusual dtype or nonfinite input.
        for index in order:
            if count and np.min(np.sum((occupied[:count]-pixels[index])**2, axis=1)) < self.MIN_TRACK_SPACING_PX**2:
                continue
            kept.append(index)
            occupied[count] = pixels[index]
            count += 1
            if count >= limit:
                break
        return np.asarray(kept, dtype=int)

    def _search_seed(self, previous, p0, camera_model, image_shape):
        """Project a measured old point for search, assuming zero translation.

        The points already include their old nominal attitude. Only the current
        calibrated camera transform is inverted here. This is an initialization
        of image search, never a correspondence, visibility or pose constraint.
        """
        self.last_tracking['camera_projection_seed_count'] = 0
        if camera_model is None:
            return p0[:, None].copy()
        optical = (previous['points']-camera_model['translation']) @ camera_model['rotation']
        projected = optical @ camera_model['K'].T
        with np.errstate(divide='ignore', invalid='ignore'):
            # depth_grid uses u+.5/v+.5, so array pixel coordinates subtract .5.
            pixels = projected[:, :2]/projected[:, 2:]-.5
        height, width = image_shape
        valid = (np.isfinite(pixels).all(axis=1) & (optical[:, 2] > .1)
                 & (pixels[:, 0] >= 0.) & (pixels[:, 0] < width)
                 & (pixels[:, 1] >= 0.) & (pixels[:, 1] < height))
        self.last_tracking['camera_projection_seed_count'] = int(valid.sum())
        # A point projected outside the image remains eligible for the original
        # zero-flow search. Projection is not a physical rejection/acceptance.
        return np.where(valid[:, None], pixels, p0).astype(np.float32)[:, None].copy()

    def _track(self, gray, grid, camera_model=None):
        previous = self.previous[0]
        p0 = previous['pixels'].astype(np.float32)
        height, width = gray.shape
        parameters = dict(winSize=(21, 21), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 1e-3),
            flags=cv2.OPTFLOW_USE_INITIAL_FLOW, minEigThreshold=1e-4)
        self.last_tracking['attempted'] = len(p0)
        identical = np.array_equal(previous['_measured_gray'], gray)
        self.last_tracking['identical_image_zero_flow'] = identical
        if identical:
            # Exact measured image equality makes zero flow an exact minimum
            # of the patch cost. Avoid an iterative solver's rounding drift;
            # all new depth/metric/spatial checks below still run.
            q, back = p0.copy(), p0.copy()
            fs = bs = np.ones((len(p0), 1), dtype=np.uint8)
        else:
            try:
                forward, fs, _ = cv2.calcOpticalFlowPyrLK(previous['_measured_gray'], gray,
                    p0[:, None], self._search_seed(previous, p0, camera_model, gray.shape), **parameters)
                if forward is None or fs is None:
                    return None
                q = forward[:, 0]
                safe = np.where(np.isfinite(q), q, p0)
                backward, bs, _ = cv2.calcOpticalFlowPyrLK(gray, previous['_measured_gray'],
                    safe[:, None], p0[:, None].copy(), **parameters)
                if backward is None or bs is None:
                    return None
            except cv2.error:
                return None
            back = backward[:, 0]
        def inside(points):
            return (np.isfinite(points).all(axis=1) & (points[:, 0] >= 12.)
                    & (points[:, 1] >= 12.) & (points[:, 0]+12. < width)
                    & (points[:, 1]+12. < height))
        good = (fs.ravel().astype(bool) & bs.ravel().astype(bool) & inside(p0) & inside(q)
                & np.isfinite(back).all(axis=1)
                & (np.linalg.norm(back-p0, axis=1) <= self.measurements.pixel_noise))
        previous_indices = np.flatnonzero(good)
        self.last_tracking['forward_backward_valid'] = len(previous_indices)
        if not len(previous_indices):
            return None
        # Improvement decides whether to use the subpixel refinement; it is not
        # itself an identity-lifetime test.  A round-trip-validated displacement
        # inside the original pixel localization uncertainty can retain the
        # unrefined *current-image* pixel instead of killing a stationary track.
        # Both choices below are real pixels and get fresh depth/covariance and
        # the unchanged metric/spatial checks; neither supplies a zero-pose factor.
        if identical:
            # Exact measured-image equality above also sets q == p0. All
            # three finite uint8 patches would therefore be identical, so
            # both photometric residuals are exactly zero. Keep the original
            # decisions without allocating/sampling those duplicate patches.
            # Fresh depth, covariance, consensus and spatial gates below still
            # run even when the RGB image has not changed.
            improves = np.ones(len(previous_indices), dtype=bool)
            unrefined = np.zeros(len(previous_indices), dtype=bool)
        else:
            target = self._sample_patch(previous['_measured_gray'], p0[previous_indices])
            before = self._sample_patch(gray, p0[previous_indices])
            after = self._sample_patch(gray, q[previous_indices])
            improves = np.mean((target-after)**2, axis=1) <= np.mean((target-before)**2, axis=1)+1e-12
            unrefined = (~improves & (np.linalg.norm(q[previous_indices]-p0[previous_indices], axis=1)
                                     <= self.measurements.pixel_noise))
        q[previous_indices[unrefined]] = p0[previous_indices[unrefined]]
        self.last_tracking['photometric_unrefined'] = int(unrefined.sum())
        self.stats['photometric_unrefined_measurements'] += int(unrefined.sum())
        previous_indices = previous_indices[improves | unrefined]
        self.last_tracking['photometric_valid'] = len(previous_indices)
        if not len(previous_indices):
            return None
        which, points, covariance = self._sample_tracks(grid, q[previous_indices].astype(float),
                                                       previous['pixel_scales'][previous_indices])
        previous_indices = previous_indices[which]
        self.last_tracking['depth_valid'] = len(previous_indices)
        if not len(previous_indices):
            return None
        tracked = dict(pixels=q[previous_indices].astype(float), points=points, covariance=covariance,
            pixel_scales=previous['pixel_scales'][previous_indices].copy(),
            image_shape=gray.shape, measured_roi=np.array([0., 0., width, height]))
        valid_y, valid_x = np.nonzero(np.isfinite(grid).all(axis=2))
        if not len(valid_x):
            return None
        tracked['measured_roi'] = np.array([valid_x.min(), valid_y.min(), valid_x.max()+1, valid_y.max()+1])
        active = {int(t) for frame, _ in self.keyframes for t in frame['submitted_ids']}
        priority = sorted(range(len(points)), key=lambda i: (
            int(previous['track_ids'][previous_indices[i]]) not in active,
            -self.age.get(int(previous['track_ids'][previous_indices[i]]), 0),
            float(np.trace(covariance[i])), int(previous['track_ids'][previous_indices[i]])))
        spaced = self._spaced(tracked['pixels'], priority)
        for name in ('pixels', 'points', 'covariance', 'pixel_scales'):
            tracked[name] = tracked[name][spaced]
        previous_indices = previous_indices[spaced]
        pairs = np.column_stack((np.arange(len(previous_indices)), previous_indices))
        pairs = self._consensus(tracked, previous, pairs)
        self.last_tracking['consensus'] = len(pairs)
        if not len(pairs):
            return None
        current_good, _ = self.measurements._spatial_support(tracked, pairs[:, 0])
        previous_good, _ = self.measurements._spatial_support(previous, pairs[:, 1])
        if not (current_good and previous_good):
            self.stats['klt_spatial_rejected'] += 1
            self.last_tracking['spatial_adequate'] = False
        else:
            self.last_tracking['spatial_adequate'] = True
        # Identity has already passed measured flow, photometric, depth and
        # metric checks. Weak spatial support forbids a pose, but must not
        # erase those associations from the next measured image.
        tracked['tracking_pose_adequate'] = bool(current_good and previous_good)
        selected = pairs[:, 0]
        for name in ('pixels', 'points', 'covariance', 'pixel_scales'):
            tracked[name] = tracked[name][selected]
        tracked['track_ids'] = previous['track_ids'][pairs[:, 1]].copy()
        self.stats['klt_surviving_tracks'] += len(selected)
        return tracked

    def process(self, sensor):
        raw = self.measurements._features(sensor.gray, sensor.xyz_grid)
        gray = raw['_measured_gray'] if raw is not None else self.measurements._contrast.apply(sensor.gray)
        try:
            search_model = self.measurements._camera_model(getattr(sensor, 'camera_model', None))
        except (ValueError, TypeError, KeyError):
            # Missing/invalid optional search metadata cannot create a match.
            # Measured-image zero-flow search and all its guards still apply.
            search_model = None
        self.last_tracking = dict(attempted=0, consensus=0, spatial_adequate=False)
        tracked = None
        if self.previous is not None and self.previous[0]['image_shape'] == gray.shape:
            original = self.previous
            try:
                if self.track_budget is not None:
                    self.previous = self._budget_previous()
                tracked = self._track(gray, sensor.xyz_grid, search_model)
            finally:
                self.previous = original
        active={int(t) for frame,_ in self.keyframes for t in frame['submitted_ids']}
        connected=0 if tracked is None else sum(int(t) in active for t in tracked['track_ids'])
        if active and (tracked is None or not tracked['tracking_pose_adequate'] or connected<24):
            recovered=self._recover_tracks(gray,sensor.xyz_grid,search_model,getattr(sensor,'stamp_ns',None))
            if recovered is not None:tracked=recovered
        tracked_count = 0 if tracked is None else len(tracked['points'])
        new = np.empty(0, dtype=int) if raw is None else self._spaced(raw['pixels'],
            range(len(raw['points'])), () if tracked is None else tracked['pixels'])
        if tracked_count+len(new) < 24:
            self.stats['no_features'] += 1
            return None
        features = dict(image_shape=gray.shape, _measured_gray=gray,
            _measured_grid=sensor.xyz_grid, _measurement_token=object(),
            _search_camera_model=search_model,
            _exposure_stamp_ns=getattr(sensor, 'stamp_ns', None),
            tracking_pose_adequate=True if tracked is None else tracked['tracking_pose_adequate'])
        if raw is not None:
            features['measured_roi'] = raw['measured_roi'].copy()
        else:
            features['measured_roi'] = tracked['measured_roi'].copy()
        for name in ('pixels', 'points', 'covariance', 'pixel_scales'):
            pieces = ([] if tracked is None else [tracked[name]])+([] if raw is None else [raw[name][new]])
            features[name] = np.concatenate(pieces, axis=0)
        new_ids = np.arange(self.next_track, self.next_track+len(new), dtype=np.int64)
        self.next_track += len(new)
        ids = np.concatenate((np.empty(0, dtype=np.int64) if tracked is None else tracked['track_ids'], new_ids))
        # ORB is only a source of new tracks. No old-descriptor/current-pixel
        # association or historical ORB recovery is claimed for tracked rows.
        features['descriptors'] = np.concatenate((np.zeros((tracked_count, 32), dtype=np.uint8),
            np.empty((0, 32), dtype=np.uint8) if raw is None else raw['descriptors'][new]))
        features['descriptor_valid'] = np.r_[np.zeros(tracked_count, dtype=bool), np.ones(len(new), dtype=bool)]
        self.stats['orb_new_tracks'] += len(new)
        self.last_tracking.update(surviving=tracked_count, new_tracks=len(new))
        self.age = {int(track): self.age.get(int(track), 0)+1 for track in ids}
        features['track_ids'] = ids
        self.previous = (features, ids.copy())
        self.stats['frames_with_features'] += 1
        return features

    def select(self, features):
        active = {int(track) for frame, _ in self.keyframes for track in frame['submitted_ids']}
        ids = features['track_ids']
        roi = features['measured_roi']
        cells = np.clip(((features['pixels']-roi[:2])/(roi[2:]-roi[:2])*[4, 3]).astype(int), 0, [3, 2])
        old = [i for i, track in enumerate(ids) if int(track) in active]
        new = [i for i, track in enumerate(ids) if int(track) not in active]
        order = lambda i: (-self.age.get(int(ids[i]), 0), float(np.trace(features['covariance'][i])), int(ids[i]))
        old.sort(key=order)
        new.sort(key=order)

        selected = self._select_observations(old, new, cells)
        connected = sum(int(ids[index]) in active for index in selected)
        enough_connections = not self.keyframes or connected >= 24
        tracking_adequate = features.get('tracking_pose_adequate', True)
        adequate, support = (self.measurements._spatial_support(features, selected)
                             if len(selected) >= 24 else (False, {}))
        retry = None
        if not adequate and enough_connections and tracking_adequate and support:
            eigenvalues = support['metric_support_eigenvalues_m2']
            if eigenvalues[1] < .02*eigenvalues[2]:
                candidate = self._select_for_metric_spread(features, selected, old, new, cells)
                candidate_adequate, candidate_support = self.measurements._spatial_support(features, candidate)
                known = candidate[np.isin(ids[candidate], tuple(active))]
                # New landmarks can enter a keyframe, but alone they cannot
                # constrain this pose in the coordinates of the existing map.
                shared_adequate = (not self.keyframes or
                                   (len(known) >= 24 and self.measurements._spatial_support(features, known)[0]))
                retry = dict(candidate_support, accepted=bool(candidate_adequate and shared_adequate),
                             connected_spatial_adequate=bool(shared_adequate),
                             original_metric_support_eigenvalues_m2=eigenvalues)
                if retry['accepted']:
                    selected, connected, adequate = candidate, len(known), True
                    support = candidate_support
        diagnostic = dict(selected=len(selected), connected=int(connected),
                          adequate=bool(adequate and enough_connections and tracking_adequate), **support)
        if not diagnostic['adequate']:
            diagnostic['tracking_pose_adequate'] = bool(tracking_adequate)
        if retry is not None:
            diagnostic['metric_spread_retry'] = retry
        if self.track_budget is not None:
            diagnostic['tracking_budget'] = {k:v for k,v in self.last_tracking.items() if k.startswith('budget_')}
        return selected, diagnostic

    def _select_observations(self, old, new, cells):
        chosen, counts = [], Counter()
        # Reserve a quarter of the measurement budget for incoming landmarks;
        # identities seen by older frames retain priority among the rest.
        for pool, limit in ((old, self.max_observations*3//4), (new, self.max_observations), (old, self.max_observations)):
            for index in pool:
                if len(chosen) >= limit:
                    break
                cell = tuple(cells[index])
                if index in chosen or counts[cell] >= 8:
                    continue
                chosen.append(index)
                counts[cell] += 1
        return np.asarray(chosen, dtype=int)

    def _select_for_metric_spread(self, features, selected, old, new, cells):
        """Re-rank a rejected frame once using its weak measured direction."""
        points = features['points']
        center = points[selected].mean(axis=0)
        eigenvalues, axes = np.linalg.eigh(np.cov(points[selected].T))
        lateral, longitudinal = ((points-center) @ axes[:, 1:]).T
        # Spend the existing per-cell budget on measured points that improve
        # lateral spread instead of adding leverage along the dominant axis.
        # Neither the ranks nor the eigenvectors enter the pose optimizer.
        score = lateral**2/max(eigenvalues[1], .0025) - longitudinal**2/eigenvalues[2]
        order = lambda i: -score[i]
        return self._select_observations(sorted(old, key=order), sorted(new, key=order), cells)

    def commit(self, features, selected):
        # Keep full measured tables for descriptor re-identification, but only
        # submitted IDs count as active map landmarks in subsequent selection.
        features['submitted_ids'] = features['track_ids'][selected].copy()
        self.keyframes.append((features, features['track_ids'].copy()))
        self._remember_reference(features)
