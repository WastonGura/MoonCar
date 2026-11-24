"""Independent offline asynchronous adapter; no ROS or default integration.

The foreground owns tracking and queries. One worker owns window commits.
Only successful, complete cache versions cross the publication boundary.
"""
from collections import Counter
from dataclasses import dataclass
import copy
import threading
import time

import numpy as np

from .core import Frame, Observation, WindowEstimator, WindowError
from .frontend import TrackFrontend
from .online import ShadowProcessor, plain
from .query_cache import _owned_frame, freeze_query_cache, prewarm_query_columns, query_cached


@dataclass(frozen=True)
class CommitJob:
    request: int
    base_version: int
    frame: Frame
    offered_wall_ns: int


@dataclass(frozen=True)
class Publication:
    version: int
    request: int
    cache: object
    source_stamp_ns: int
    finished_wall_ns: int
    service_s: float


class CommitWorker:
    """One running and one replaceable pending job, one latest publication.

    The shallow candidate copies every container mutated by WindowEstimator;
    its array priors/solutions are read-only inputs to the next transaction.
    A failed solve OR freeze never advances the committed estimator/version.
    """
    def __init__(self, estimator=None, freezer=freeze_query_cache, prewarm_landmarks=96):
        if type(prewarm_landmarks) is not int or not 0 <= prewarm_landmarks <= 96:
            raise ValueError('invalid prewarm budget')
        self._estimator = estimator if estimator is not None else WindowEstimator(6)
        self._freezer = freezer
        self.prewarm_landmarks = prewarm_landmarks
        self._condition = threading.Condition()
        self._pending = None
        self._running = None
        self._publication = None
        # SciPy 1.11 SuperLU tracks native allocations per creator thread.
        # Retain ownership here until the foreground releases its old lease;
        # final destruction must happen on this worker, not in _install.
        self._owned = {}
        self._acknowledged = 0
        self._retiring = False
        self._notice = None
        self._version = 0
        self._closed = False
        self._fault = None
        self._counts = Counter()
        self._thread = threading.Thread(target=self._work, name='rgbd-window-commit', daemon=True)
        self._thread.start()

    def offer(self, request, base_version, frame):
        # The copy is measured foreground work; no image/grid crosses here.
        job = CommitJob(int(request), int(base_version), _owned_frame(frame), time.monotonic_ns())
        with self._condition:
            if self._closed or self._fault:
                self._counts['offer_unavailable'] += 1
                return False
            if job.base_version != self._version:
                self._counts['offer_stale'] += 1
                return False
            self._acknowledged = max(self._acknowledged, job.base_version)
            if self._pending is not None:
                self._counts['pending_replaced'] += 1
            self._pending = job
            self._counts['offered'] += 1
            self._condition.notify_all()
            return True

    def poll(self):
        with self._condition:
            return self._publication, dict(version=self._version, running=self._running is not None,
                pending=self._pending is not None, closed=self._closed, fault=self._fault,
                owned_versions=len(self._owned), acknowledged_version=self._acknowledged,
                notice=None if self._notice is None else dict(self._notice), counts=dict(self._counts))

    def acknowledge(self, version):
        """Foreground has dropped all older cache references before this call.

        Native caches are borrowed within process(); callers must not retain
        publications after acknowledging a newer version or closing.
        """
        with self._condition:
            if not 0 <= version <= self._version:
                raise WindowError('invalid publication acknowledgement')
            self._acknowledged = max(self._acknowledged, version)
            self._condition.notify_all()

    def wait_idle(self, timeout=5.):
        """Testing/drain only: never called on the foreground process path."""
        with self._condition:
            return self._condition.wait_for(lambda: self._pending is None and self._running is None
                and not self._retiring and not any(v < self._acknowledged for v in self._owned), timeout)

    def close(self, timeout=5.):
        with self._condition:
            self._closed = True
            if self._pending is not None:
                self._counts['pending_cancelled'] += 1
            self._pending = None
            self._condition.notify_all()
        self._thread.join(timeout)
        if self._thread.is_alive():
            raise RuntimeError('commit worker did not stop within close timeout')

    def _work(self):
        cache = candidate = posterior = None
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._pending is not None
                    or any(v < self._acknowledged for v in self._owned))
                retired = [self._owned.pop(v) for v in list(self._owned)
                           if v < self._acknowledged]
                self._retiring = bool(retired)
            # No consumer holds these leases. Release native factors on the
            # creator thread, outside the brief publication lock.
            retired.clear()
            with self._condition:
                self._retiring = False
                self._condition.notify_all()
                if self._closed:
                    self._publication = None
                    self._owned.clear()
                    cache = candidate = posterior = None
                    return
                if self._pending is None:
                    continue
                job, self._pending = self._pending, None
                self._running = job
            began = time.monotonic_ns()
            cpu_began = time.thread_time_ns()
            candidate = cache = None
            warm = dict(wall_s=0., selected_landmarks=0, added_columns=0, cache_bytes=0)
            reason = fault = None
            try:
                if job.base_version != self._version:
                    raise WindowError('candidate base version is obsolete')
                candidate = copy.copy(self._estimator)
                candidate._frames = list(self._estimator._frames)
                candidate._frame_terms = dict(self._estimator._frame_terms)
                candidate._prior_keys = list(self._estimator._prior_keys)
                candidate._solution = dict(self._estimator._solution)
                _, posterior = candidate.add_frame_with_posterior(job.frame)
                cache = self._freezer(candidate, self._version+1, posterior=posterior)
                if self.prewarm_landmarks:
                    warm = prewarm_query_columns(cache, (o.track_id for o in job.frame.observations),
                        self._version+1, self.prewarm_landmarks)
            except WindowError as error:
                reason = str(error)
            except Exception as error:
                reason = 'backend_exception: '+repr(error)
                fault = reason
            finished = time.monotonic_ns()
            with self._condition:
                self._counts['completed'] += 1
                self._notice = dict(request=job.request, stamp_ns=job.frame.stamp_ns,
                    base_version=job.base_version, success=reason is None,
                    reason=reason, service_s=(finished-began)/1e9,
                    thread_cpu_s=(time.thread_time_ns()-cpu_began)/1e9,
                    prewarm=warm,
                    queue_s=(began-job.offered_wall_ns)/1e9)
                if self._closed:
                    self._counts['completed_after_close'] += 1
                elif reason is None:
                    self._estimator = candidate
                    self._version += 1
                    self._owned[self._version] = cache
                    self._publication = Publication(self._version, job.request, cache,
                        job.frame.stamp_ns, finished, (finished-began)/1e9)
                    self._counts['committed'] += 1
                else:
                    self._counts['rejected'] += 1
                if fault:
                    self._fault = fault
                    if self._pending is not None:
                        self._counts['pending_cancelled'] += 1
                    self._pending = None
                self._running = None
                self._condition.notify_all()


class AsyncProcessor:
    """Sequential foreground API backed by a separate bounded commit worker.

    Cache TTL is distinct from output exposure age: default two KF periods.
    Callers must still score original exposure-to-delivery latency externally.
    """
    # Keep one minimum-support set in reserve while the next measured
    # keyframe is built. This schedules work; it never relaxes the 24-point
    # association/spatial gates or turns an uncommitted frame into a map.
    OVERLAP_KEYFRAME_TRACKS = 48
    OVERLAP_KEYFRAME_INTERVAL_NS = 100_000_000
    def __init__(self, frontend=None, worker=None, keyframe_interval_ns=500_000_000,
                 max_cache_age_ns=1_000_000_000, query=query_cached, prewarm_landmarks=96):
        if type(keyframe_interval_ns) is not int or keyframe_interval_ns <= 0:
            raise ValueError('invalid keyframe interval')
        if type(max_cache_age_ns) is not int or max_cache_age_ns <= 0:
            raise ValueError('invalid cache age limit')
        self.frontend = frontend if frontend is not None else TrackFrontend(96, 6)
        self.worker = worker if worker is not None else CommitWorker(prewarm_landmarks=prewarm_landmarks)
        self.keyframe_interval_ns = keyframe_interval_ns
        self.max_cache_age_ns = max_cache_age_ns
        self.query = query
        self.query_cache = None
        self.cache_version = 0
        self.gauge_stamp_ns = None
        self.last_requested_stamp_ns = None
        self.last_stamp_ns = None
        self.request = 0

    def close(self, timeout=5.):
        # Drop the consumer lease while the worker still retains its native
        # ownership; the worker then performs the final resource destruction.
        self.query_cache = None
        self.worker.close(timeout)

    def _install(self, publication, stamp):
        if (publication is None or publication.version <= self.cache_version
                or publication.source_stamp_ns >= stamp):
            return
        cache = publication.cache
        if self.gauge_stamp_ns is not None and cache.gauge_stamp_ns != self.gauge_stamp_ns:
            raise WindowError('publication changed gauge')
        # Tracking/selection only need submitted IDs from confirmed frames,
        # not mutable full image/depth feature tables from the past.
        metadata = []
        for frame in cache._frames:
            ids = np.array([o.track_id for o in frame.observations], dtype=np.int64)
            ids.setflags(write=False)
            metadata.append((dict(submitted_ids=ids), ids))
        self.frontend.keyframes.clear()
        self.frontend.keyframes.extend(metadata)
        self.query_cache, self.cache_version = cache, publication.version
        self.gauge_stamp_ns = cache.gauge_stamp_ns
        self.worker.acknowledge(self.cache_version)

    @staticmethod
    def observation_frame(sensor, features, selected):
        R = sensor.nominal_R_wb @ sensor.R_bc
        lever = sensor.nominal_R_wb @ sensor.t_bc
        observations = [Observation(int(features['track_ids'][i]),
            R.T @ (features['points'][i]-lever),
            R.T @ features['covariance'][i] @ R) for i in selected]
        return Frame(sensor.stamp_ns, sensor.nominal_R_wb, sensor.R_bc, sensor.t_bc,
                     observations, np.deg2rad(.1)**2*np.eye(3))

    def process(self, sensor, planned_stamp_ns, received_wall_ns=0, started_wall_ns=0,
                sim_now_ns=None, wall_now_ns=time.monotonic_ns):
        if sim_now_ns is None:
            sim_now_ns = lambda: sensor.stamp_ns
        row = ShadowProcessor.failure(self, planned_stamp_ns, sensor.stamp_ns, 'initializing',
            received_wall_ns, started_wall_ns, 0, 0)
        begin = time.perf_counter()
        cpu_begin = time.thread_time_ns()
        if self.last_stamp_ns is not None and sensor.stamp_ns <= self.last_stamp_ns:
            raise WindowError('foreground exposure stamps must increase')
        publication, status = self.worker.poll()
        self._install(publication, sensor.stamp_ns)
        if status['notice'] is not None and hasattr(self.frontend, 'complete_submissions'):
            self.frontend.complete_submissions(status['notice']['request'])
        self.last_stamp_ns = sensor.stamp_ns
        before = time.perf_counter()
        features = self.frontend.process(sensor)
        row['frontend_s'] = time.perf_counter()-before
        due = (self.last_requested_stamp_ns is None
               or sensor.stamp_ns-self.last_requested_stamp_ns >= self.keyframe_interval_ns)
        cache = self.query_cache  # One whole foreground frame uses one version.
        row.update(estimation_mode='async_query', keyframe_requested=False, request_accepted=False,
            keyframe_trigger=None,
            cache_version=self.cache_version, cache_available=cache is not None,
            cache_age_s=None if cache is None else (sensor.stamp_ns-cache.keyframe_stamps_ns[-1])/1e9,
            gauge_stamp_ns=self.gauge_stamp_ns, backend_status=status)
        if status['fault'] or status['closed']:
            row['reason'] = 'backend_unavailable'
        elif features is None:
            row['reason'] = 'no_features'
        else:
            selected, diagnostic = self.frontend.select(features)
            row['frontend'] = diagnostic
            if not diagnostic['adequate']:
                row['reason'] = 'insufficient_shared_or_spatial_track_support'
            else:
                overlap_due = (cache is not None
                    and diagnostic.get('connected', self.OVERLAP_KEYFRAME_TRACKS) < self.OVERLAP_KEYFRAME_TRACKS
                    and self.last_requested_stamp_ns is not None
                    and sensor.stamp_ns-self.last_requested_stamp_ns >= self.OVERLAP_KEYFRAME_INTERVAL_NS
                    and not status['running'] and not status['pending'])
                frame = self.observation_frame(sensor, features, selected)
                if cache is not None:
                    if sensor.stamp_ns-cache.keyframe_stamps_ns[-1] > self.max_cache_age_ns:
                        row['reason'] = 'cache_expired'
                    else:
                        before = time.perf_counter()
                        try:
                            answer = self.query(frame, cache, self.cache_version)
                            row.update(output=True, reason='solved', position=answer.position,
                                covariance_conditional=answer.covariance, diagnostics=answer.diagnostics)
                        except WindowError as error:
                            row['reason'] = 'window_error: '+str(error)
                        row['backend_s'] = time.perf_counter()-before
                if due or overlap_due:
                    self.request += 1
                    self.last_requested_stamp_ns = sensor.stamp_ns
                    row['keyframe_requested'] = True
                    row['keyframe_trigger'] = 'interval' if due else 'declining_overlap'
                    row['request_accepted'] = self.worker.offer(self.request, self.cache_version, frame)
                    if row['request_accepted'] and hasattr(self.frontend, 'protect_submission'):
                        self.frontend.protect_submission(self.request, (o.track_id for o in frame.observations))
        row['finished_wall_ns'] = wall_now_ns()
        row['ready_sim_ns'] = int(sim_now_ns())
        row['age_sim_s'] = (row['ready_sim_ns']-sensor.stamp_ns)/1e9
        row['foreground_s'] = time.perf_counter()-begin
        row['foreground_thread_cpu_s'] = (time.thread_time_ns()-cpu_begin)/1e9
        return plain(row)
