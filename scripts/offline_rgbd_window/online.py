"""Pure sensor-to-plan adapter for the independent live shadow experiment.

No ROS, baseline, truth, navigation, TF publication or health authority lives
here. Processing every available input is attempted, not guaranteed: the live
node owns a single latest pending slot and reports every replacement explicitly.
"""
from dataclasses import dataclass, replace
from contextlib import contextmanager
from pathlib import Path
import json
import threading
import time

import numpy as np

from .core import Frame, Observation, WindowEstimator, WindowError
from .data import SensorFrame
from .frontend import TrackFrontend
from .query_cache import freeze_query_cache, query_cached


class _PhaseTimings:
    """Optional diagnostic clocks; process CPU also includes other threads."""
    def __init__(self, enabled):
        self.enabled = enabled
        self.values = {}

    @contextmanager
    def phase(self, name):
        if not self.enabled:
            yield
            return
        wall, thread, process = time.perf_counter_ns(), time.thread_time_ns(), time.process_time_ns()
        try:
            yield
        finally:
            self.values[name] = dict(wall_s=(time.perf_counter_ns()-wall)/1e9,
                thread_cpu_s=(time.thread_time_ns()-thread)/1e9,
                process_cpu_s=(time.process_time_ns()-process)/1e9)


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    return value


class JsonlWriter:
    def __init__(self, path, epoch):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open('x', encoding='utf-8')
        self.epoch = epoch
        self.lock = threading.Lock()

    def write(self, row):
        encoded = json.dumps(plain(dict(row, epoch=self.epoch)), allow_nan=False)+'\n'
        with self.lock:
            self.stream.write(encoded)
            self.stream.flush()

    def close(self):
        with self.lock:
            self.stream.close()


class PlanScheduler:
    """Fixed slots advanced by real paired exposures, with a complete tail.

    Live callers use observe's exposure watermark to identify skipped slots.
    The ROS clock is not a delivery watermark: expiring slots against it before
    sensor callbacks arrive would suppress genuinely late measured solves.
    At closed capture, expire(final=True) accounts for the unobserved tail.
    """
    def __init__(self, interval_ns=500_000_000):
        if not isinstance(interval_ns, int) or interval_ns <= 0:
            raise ValueError('interval_ns must be positive')
        self.interval_ns = interval_ns
        self.origin_ns = None
        self.next_due_ns = None
        self.last_exposure_ns = None

    def observe(self, stamp_ns):
        if (not isinstance(stamp_ns, int) or stamp_ns < 0
                or (self.last_exposure_ns is not None and stamp_ns <= self.last_exposure_ns)):
            raise ValueError('nonincreasing paired exposure stamp')
        self.last_exposure_ns = stamp_ns
        if self.origin_ns is None:
            self.origin_ns = self.next_due_ns = stamp_ns
        due = []
        while self.next_due_ns <= stamp_ns:
            due.append(self.next_due_ns)
            self.next_due_ns += self.interval_ns
        # One physical observation cannot masquerade as several new solutions.
        return (due[-1] if due else None), due[:-1]

    def expire(self, now_sim_ns, final=False):
        missing = []
        boundary = now_sim_ns if final else now_sim_ns-self.interval_ns
        while self.next_due_ns is not None and self.next_due_ns <= boundary:
            missing.append(self.next_due_ns)
            self.next_due_ns += self.interval_ns
        return missing


class LatestMailbox:
    """Exactly one queued item plus, externally, one item being processed."""
    def __init__(self):
        self.condition = threading.Condition()
        self.latest = None
        self.closed = False

    def offer(self, item):
        with self.condition:
            if self.closed:
                raise RuntimeError('mailbox is closed')
            previous, self.latest = self.latest, item
            self.condition.notify()
            return previous

    def take(self, timeout=.1):
        with self.condition:
            self.condition.wait_for(lambda: self.closed or self.latest is not None, timeout)
            item, self.latest = self.latest, None
            return item

    def close(self):
        with self.condition:
            self.closed = True
            previous, self.latest = self.latest, None
            self.condition.notify_all()
            return previous


def transfer_pending_plan(previous, current, interval_ns):
    """Keep a pending slot on its latest real exposure, never on a new stamp.

    Only queued work can transfer. The caller holds the mailbox condition so a
    worker cannot claim the old item between inspection and replacement.
    Cross-slot replacement still fails the old plan explicitly.
    """
    planned = None if previous is None else previous.planned_stamp_ns
    if (planned is not None and current.planned_stamp_ns is None
            and planned <= current.stamp_ns < planned+interval_ns):
        return replace(current, planned_stamp_ns=planned), True
    return current, False


def make_sensor_frame(stamp_ns, gray, optical, nominal_R_wb, R_bc, t_bc, K, outside_rover):
    """Use the same measured optical depth/mask and per-exposure transforms."""
    measured = np.full(optical.shape, np.nan, dtype=float)
    valid = np.flatnonzero(np.isfinite(optical).all(axis=2).ravel())
    body = optical.reshape(-1, 3)[valid] @ R_bc.T+t_bc
    external = outside_rover(body)
    measured.reshape(-1, 3)[valid[external]] = body[external] @ nominal_R_wb.T
    # Metric float depth is another representation of the SAME input samples.
    return SensorFrame(int(stamp_ns), np.ascontiguousarray(gray), measured,
        np.ascontiguousarray(optical[:, :, 2]), np.asarray(nominal_R_wb).copy(),
        np.asarray(R_bc).copy(), np.asarray(t_bc).copy(), np.asarray(K).copy(), None, None)


class ShadowProcessor:
    def __init__(self, frontend=None, estimator=None, keyframe_interval_ns=500_000_000,
                 cached_query=False, timing_diagnostics=False):
        if type(keyframe_interval_ns) is not int or keyframe_interval_ns <= 0:
            raise ValueError('keyframe interval must be positive')
        if type(cached_query) is not bool:
            raise ValueError('cached_query must be boolean')
        if type(timing_diagnostics) is not bool:
            raise ValueError('timing_diagnostics must be boolean')
        self.frontend = frontend if frontend is not None else TrackFrontend(96, 6)
        self.estimator = estimator if estimator is not None else WindowEstimator(6)
        self.gauge_stamp_ns = None
        self.last_keyframe_stamp_ns = None
        self.keyframe_interval_ns = keyframe_interval_ns
        self.cached_query = cached_query
        self.timing_diagnostics = timing_diagnostics
        self.query_cache = None
        self.cache_version = 0

    def failure(self, planned_stamp_ns, stamp_ns, reason, received_wall_ns,
                started_wall_ns, finished_wall_ns, ready_sim_ns):
        return dict(type='plan', planned_stamp_ns=planned_stamp_ns, stamp_ns=stamp_ns,
            output=False, reason=reason, gauge_stamp_ns=self.gauge_stamp_ns,
            received_wall_ns=received_wall_ns, started_wall_ns=started_wall_ns,
            finished_wall_ns=finished_wall_ns, ready_sim_ns=ready_sim_ns,
            age_sim_s=None if stamp_ns is None else (ready_sim_ns-stamp_ns)/1e9,
            queue_wait_wall_s=None if started_wall_ns is None or received_wall_ns is None
                else (started_wall_ns-received_wall_ns)/1e9,
            frontend_s=0., backend_s=0., frontend={})

    def process(self, sensor, planned_stamp_ns, received_wall_ns, started_wall_ns,
                sim_now_ns, wall_now_ns=time.monotonic_ns):
        phases = _PhaseTimings(self.timing_diagnostics)
        begin = time.perf_counter()
        with phases.phase('frontend'):
            features = self.frontend.process(sensor)
        frontend_s = time.perf_counter()-begin
        if planned_stamp_ns is None:
            return None
        row = self.failure(planned_stamp_ns, sensor.stamp_ns, 'no_features',
            received_wall_ns, started_wall_ns, 0, 0)
        row['frontend_s'] = frontend_s
        keyframe = (self.last_keyframe_stamp_ns is None
                    or sensor.stamp_ns-self.last_keyframe_stamp_ns >= self.keyframe_interval_ns)
        row.update(keyframe=keyframe, estimation_mode='keyframe_commit' if keyframe else 'joint_query')
        if self.cached_query:
            # A due keyframe must never leave its preceding cache usable after
            # a rejected commit/build. Only a newly committed version can
            # restore cached queries; there is no legacy-query fallback.
            if keyframe:
                self.query_cache = None
            elif self.query_cache is None:
                row['reason'] = 'query_cache_unavailable'
            row.update(cache_version=self.cache_version, cache_available=self.query_cache is not None,
                       cache_build_s=0.)
            if not keyframe:
                row['estimation_mode'] = 'cached_query'
        if features is not None:
            with phases.phase('selection'):
                selected, diagnostic = self.frontend.select(features)
            row['frontend'] = diagnostic
            if not diagnostic['adequate']:
                row['reason'] = 'insufficient_shared_or_spatial_track_support'
            else:
                with phases.phase('observation_build'):
                    R = sensor.nominal_R_wb @ sensor.R_bc
                    lever = sensor.nominal_R_wb @ sensor.t_bc
                    observations = [Observation(int(features['track_ids'][i]),
                        R.T @ (features['points'][i]-lever),
                        R.T @ features['covariance'][i] @ R) for i in selected]
                    frame = Frame(sensor.stamp_ns, sensor.nominal_R_wb, sensor.R_bc, sensor.t_bc,
                        observations, np.deg2rad(.1)**2*np.eye(3))
                begin = time.perf_counter()
                try:
                    with phases.phase('solve'):
                        if keyframe:
                            if self.cached_query:
                                answer, posterior = self.estimator.add_frame_with_posterior(frame)
                            else:
                                answer = self.estimator.add_frame(frame)
                        elif self.cached_query:
                            if self.query_cache is None:
                                raise WindowError('query cache unavailable after unsuccessful keyframe/cache build')
                            answer = query_cached(frame, self.query_cache, self.cache_version)
                        else:
                            answer = self.estimator.query_frame(frame)
                    if keyframe:
                        with phases.phase('commit'):
                            self.frontend.commit(features, selected)
                        self.last_keyframe_stamp_ns = sensor.stamp_ns
                    if self.gauge_stamp_ns is None:
                        self.gauge_stamp_ns = sensor.stamp_ns
                    if keyframe and self.cached_query:
                        self.cache_version += 1
                        cache_begin = time.perf_counter()
                        try:
                            with phases.phase('freeze'):
                                self.query_cache = freeze_query_cache(self.estimator, self.cache_version,
                                                                     posterior=posterior)
                        finally:
                            # This is a subset of backend_s, including failed
                            # builds; it is never shifted to an untimed phase.
                            row['cache_build_s'] = time.perf_counter()-cache_begin
                    row.update(output=True, reason='solved', position=answer.position,
                        covariance_conditional=answer.covariance,
                        diagnostics=answer.diagnostics, gauge_stamp_ns=self.gauge_stamp_ns)
                except WindowError as error:
                    row['reason'] = 'window_error: '+str(error)
                row['backend_s'] = time.perf_counter()-begin
        if self.cached_query:
            row.update(cache_version=self.cache_version, cache_available=self.query_cache is not None,
                       gauge_stamp_ns=self.gauge_stamp_ns)
        row['finished_wall_ns'] = wall_now_ns()
        row['ready_sim_ns'] = int(sim_now_ns())
        row['age_sim_s'] = (row['ready_sim_ns']-sensor.stamp_ns)/1e9
        # Slow but measured solutions remain recorded. No .3 s censoring here.
        with phases.phase('plain'):
            result = plain(row)
        if self.timing_diagnostics:
            result['timing_phases'] = phases.values
        return result
