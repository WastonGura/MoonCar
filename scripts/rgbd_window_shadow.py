#!/usr/bin/env python3
"""Independent RGB-D window shadow: sensor subscriptions and JSONL only.

Run in the sourced ROS environment, e.g. --output /evidence/shadow.jsonl
--ros-args -p use_sim_time:=true. Never publishes TF, poses, health or control.
"""
import argparse
from collections import Counter, OrderedDict, deque
from dataclasses import dataclass, replace
import json
from pathlib import Path
import sys
import threading
import time
import uuid

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, CameraInfo, Imu
from tf2_ros import Buffer, TransformListener, TransformException
from ament_index_python.packages import get_package_prefix

# Reuse the installed sensor decoders, not a second interpretation of pixels.
sys.path.insert(0, str(Path(get_package_prefix('mooncar_slam'))/'lib'/'mooncar_slam'))
from rgbd_odometry_node import ExposureImage, gray_image, depth_grid, calibration_signature
from terrain_odometry_node import imu_at, outside_rover
from offline_rgbd_window.online import (JsonlWriter, LatestMailbox, PlanScheduler,
                                      ShadowProcessor, make_sensor_frame, transfer_pending_plan)


@dataclass(frozen=True)
class Pair:
    stamp_ns: int
    rgb: ExposureImage
    depth: ExposureImage
    frame_id: str
    received_wall_ns: int
    planned_stamp_ns: int | None
    offer_wall_ns: int | None = None


def stamp_ns(msg):
    return int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)


class ShadowNode(Node):
    def __init__(self, args):
        super().__init__('mooncar_rgbd_window_shadow')
        cv2.setNumThreads(2)
        self.args = args
        self.lock = threading.RLock()
        self.counts = Counter()
        self.epoch = str(uuid.uuid4())
        self.writer = JsonlWriter(args.output, self.epoch)
        self.processor = ShadowProcessor(cached_query=args.cached_query, timing_diagnostics=True)
        self.scheduler = PlanScheduler(interval_ns=100_000_000)
        self.mailbox = LatestMailbox()
        self.images = dict(rgb=OrderedDict(), depth=OrderedDict())
        self.info = dict(rgb=None, depth=None)
        self.imu = deque(maxlen=500)
        # Header/callback tracing is bounded and never flushes per IMU sample.
        self.imu_trace = deque(maxlen=256)
        self.imu_trace_total = 0
        self.imu_header_summary = dict(callbacks_received=0, callbacks_finished=0,
            accepted=0, rejected=0, first_stamp_ns=None, last_stamp_ns=None,
            maximum_gap_ns=0, gaps_over_30ms=0, gaps_over_60ms=0)
        self.initial_imu_yaw_rad = None
        self.last_sim_ns = None
        self.fault = None
        self.closing = False
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE
            if args.image_reliability == 'reliable' else ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/mooncar/localization_rgb/image',
            lambda msg: self.on_image('rgb', msg), qos)
        self.create_subscription(Image, '/mooncar/depth/image',
            lambda msg: self.on_image('depth', msg), qos)
        self.create_subscription(CameraInfo, '/mooncar/localization_rgb/camera_info',
            lambda msg: self.on_info('rgb', msg), qos)
        self.create_subscription(CameraInfo, '/mooncar/depth/camera_info',
            lambda msg: self.on_info('depth', msg), qos)
        imu_qos = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Imu, '/mooncar/imu', self.on_imu, imu_qos)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.writer.write(dict(type='run_start', sim_ns=self.sim_now(), started_wall_ns=time.monotonic_ns(),
            axes='initial_body_yaw', initial_imu_yaw_rad=None, gauge_stamp_ns=None,
            config=dict(interval_ns=100_000_000, keyframe_interval_ns=500_000_000,
                query='current measured frame joint MAP with persistent keyframes; query factors are discarded',
                cached_query=args.cached_query,
                window=6, max_observations=96,
                latest_pending=1, raw_pair_buffer_per_stream=8, imu_buffer=500,
                readiness_wait_wall_s=args.readiness_wait_wall_s,
                image_reliability=args.image_reliability, opencv_threads=cv2.getNumThreads(),
                imu_qos=dict(depth=200, history='keep_last', reliability='reliable', durability='volatile'),
                imu_trace=dict(max_rows=256, fields=['sequence', 'stamp_ns', 'received_wall_ns',
                    'finished_wall_ns', 'accepted', 'reason'], emission='bracket neighbors on success; recent ring on failure'),
                covariance='conditional only; no baseline C0 input',
                acceptance='unchanged image/depth/geometry gates; no online age cutoff',
                axes='Rz(-initial_imu_yaw_rad) times mounted IMU world axes',
                topics=['/mooncar/localization_rgb/image', '/mooncar/depth/image',
                        '/mooncar/localization_rgb/camera_info', '/mooncar/depth/camera_info', '/mooncar/imu'],
                outputs='JSONL only; no ROS publisher or control')))
        self.worker = threading.Thread(target=self.work, name='rgbd-window-shadow', daemon=False)
        self.worker.start()
        print(json.dumps(dict(shadow_ready=True, epoch=self.epoch, output=str(args.output))), flush=True)

    def sim_now(self):
        return int(self.get_clock().now().nanoseconds)

    def event(self, **values):
        self.writer.write(values)

    def set_fault(self, reason):
        with self.lock:
            if self.fault is None:
                self.fault = reason
                self.event(type='fault', reason=reason, sim_ns=self.sim_now(), wall_ns=time.monotonic_ns())

    def guard_clock(self):
        now = self.sim_now()
        with self.lock:
            if self.last_sim_ns is not None and now < self.last_sim_ns:
                self.set_fault('simulation_clock_regressed')
            self.last_sim_ns = now
        return now

    def write_plan(self, row):
        with self.lock:
            self.counts['plan_outputs' if row['output'] else 'plan_failures'] += 1
        self.writer.write(row)

    def fail_plan(self, planned, stamp, reason, received=None, started=None, diagnostics=None):
        if planned is not None:
            row = self.processor.failure(planned, stamp, reason, received,
                started, time.monotonic_ns(), self.sim_now())
            if diagnostics is not None:
                row.update(diagnostics)
            self.write_plan(row)
            return row

    def exposure(self, pair, disposition, reason=None):
        with self.lock:
            self.counts[disposition] += 1
            counts = dict(self.counts)
        self.event(type='exposure', stamp_ns=pair.stamp_ns, planned_stamp_ns=pair.planned_stamp_ns,
            received_wall_ns=pair.received_wall_ns, wall_ns=time.monotonic_ns(), sim_ns=self.sim_now(),
            offer_wall_ns=getattr(pair, 'offer_wall_ns', None),
            disposition=disposition, reason=reason, counters=counts)

    def on_info(self, kind, msg):
        try:
            incoming = calibration_signature(msg)
        except (ValueError, TypeError):
            self.set_fault('invalid_camera_info')
            return
        with self.lock:
            if self.info[kind] is not None and self.info[kind] != incoming:
                self.set_fault('camera_calibration_changed')
                return
            self.info[kind] = incoming

    def on_imu(self, msg):
        received = time.monotonic_ns()
        stamp, accepted, reason = None, False, 'callback_exception'
        with self.lock:
            sequence = getattr(self, 'imu_trace_total', 0)+1
            self.imu_trace_total = sequence
            summary = getattr(self, 'imu_header_summary', None)
            if summary is not None:
                summary['callbacks_received'] += 1
        try:
            stamp = stamp_ns(msg)
            try:
                tf = self.buffer.lookup_transform('base_footprint', msg.header.frame_id,
                                                  Time.from_msg(msg.header.stamp))
                q, m = msg.orientation, tf.transform.rotation
                mount = Rotation.from_quat([m.x, m.y, m.z, m.w])
                body = Rotation.from_quat([q.x, q.y, q.z, q.w])*mount.inv()
                rate = mount.apply([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
                if not np.isfinite(rate).all():
                    raise ValueError('invalid IMU rate')
            except (TransformException, ValueError):
                reason = 'invalid_or_missing_tf'
                with self.lock:
                    self.counts['imu_invalid_or_missing_tf'] += 1
                return
            with self.lock:
                if self.imu and stamp/1e9 <= self.imu[-1][0]:
                    reason = 'nonincreasing'
                    self.counts['imu_nonincreasing'] += 1
                    return
                if self.initial_imu_yaw_rad is None:
                    matrix = body.as_matrix()
                    self.initial_imu_yaw_rad = float(np.arctan2(matrix[1, 0], matrix[0, 0]))
                    self.event(type='imu_reference', stamp_ns=stamp,
                        initial_imu_yaw_rad=self.initial_imu_yaw_rad, axes='initial_body_yaw',
                        received_wall_ns=time.monotonic_ns(), sim_ns=self.sim_now())
                normalized = Rotation.from_rotvec([0., 0., -self.initial_imu_yaw_rad])*body
                self.imu.append((stamp/1e9, normalized.as_quat(), float(rate[2])))
                self.counts['imu_samples'] += 1
                accepted, reason = True, 'accepted'
                if summary is not None:
                    previous = summary['last_stamp_ns']
                    if previous is None:
                        summary['first_stamp_ns'] = stamp
                    else:
                        gap = stamp-previous
                        summary['maximum_gap_ns'] = max(summary['maximum_gap_ns'], gap)
                        summary['gaps_over_30ms'] += int(gap > 30_000_000)
                        summary['gaps_over_60ms'] += int(gap > 60_000_000)
                    summary['last_stamp_ns'] = stamp
                    summary['accepted'] += 1
        finally:
            with self.lock:
                trace = getattr(self, 'imu_trace', None)
                if trace is not None:
                    trace.append((sequence, stamp, received,
                                  time.monotonic_ns(), accepted, reason))
                if summary is not None:
                    summary['callbacks_finished'] += 1
                    summary['rejected'] += int(not accepted)

    def on_image(self, kind, msg):
        stamp = stamp_ns(msg)
        received = time.monotonic_ns()
        self.guard_clock()
        try:
            captured = ExposureImage.capture(msg)
        except Exception:
            with self.lock:
                self.counts['image_capture_failed'] += 1
            return
        other = 'depth' if kind == 'rgb' else 'rgb'
        with self.lock:
            self.counts[kind+'_messages'] += 1
            if self.closing:
                return
            if stamp in self.images[kind]:
                self.counts[kind+'_duplicate'] += 1
                return
            self.images[kind][stamp] = (captured, received, msg.header.frame_id)
            while len(self.images[kind]) > 8:
                old_stamp, _ = self.images[kind].popitem(last=False)
                self.counts[kind+'_unmatched_dropped'] += 1
                self.event(type='input_reject', stream=kind, stamp_ns=old_stamp,
                    reason='bounded_unmatched_buffer', wall_ns=received)
            if stamp not in self.images[other]:
                return
            rgb, rgb_wall, rgb_frame = self.images['rgb'].pop(stamp)
            depth, depth_wall, depth_frame = self.images['depth'].pop(stamp)
            self.counts['paired_exposures'] += 1
            try:
                planned, missing = self.scheduler.observe(stamp)
            except ValueError:
                pair = Pair(stamp, rgb, depth, rgb_frame, max(rgb_wall, depth_wall), None)
                self.exposure(pair, 'rejected', 'nonincreasing_paired_stamp')
                return
            for due in missing:
                self.fail_plan(due, None, 'missing_exposure_for_interval')
            pair = Pair(stamp, rgb, depth, rgb_frame, max(rgb_wall, depth_wall), planned)
            if rgb_frame != depth_frame or self.fault:
                reason = self.fault or 'rgb_depth_frame_mismatch'
                self.exposure(pair, 'rejected', reason)
                self.fail_plan(planned, stamp, reason, pair.received_wall_ns)
                return
            with self.mailbox.condition:
                pair, transferred = transfer_pending_plan(self.mailbox.latest, pair,
                                                          self.scheduler.interval_ns)
                pair = replace(pair, offer_wall_ns=time.monotonic_ns())
                replaced = self.mailbox.offer(pair)
            self.exposure(pair, 'queued')
            if replaced is not None:
                self.exposure(replaced, 'replaced', 'latest_pending_replacement')
                if transferred:
                    self.counts['planned_reassignments'] += 1
                    self.event(type='plan_reassigned', planned_stamp_ns=pair.planned_stamp_ns,
                        from_stamp_ns=replaced.stamp_ns, to_stamp_ns=pair.stamp_ns)
                else:
                    self.fail_plan(replaced.planned_stamp_ns, replaced.stamp_ns,
                        'exposure_replaced_before_processing', replaced.received_wall_ns)

    def _imu_bracket_diagnostics(self, rows, stamp, available, failed):
        """Describe the actual sampled buffer; never supplies attitude or a gate."""
        target = stamp/1e9
        before = next((row[0] for row in reversed(rows) if row[0] <= target), None)
        after = next((row[0] for row in rows if row[0] >= target), None)
        bracket = next(((a[0], b[0]) for a, b in zip(rows, rows[1:])
                        if a[0] <= target <= b[0]), None)
        if bracket is None and rows and target == rows[-1][0]:
            bracket = (target, target)
        ns = lambda value: None if value is None else round(value*1e9)
        with self.lock:
            trace = list(getattr(self, 'imu_trace', ()))
            total = getattr(self, 'imu_trace_total', 0)
        if failed:
            selected = trace
        else:
            neighbors = {ns(before), ns(after)}
            if bracket is not None:
                neighbors.update(map(ns, bracket))
            selected = list({row[0]: row for row in
                [row for row in trace if row[1] in neighbors][-4:]+trace[-2:]}.values())
        return dict(buffer_size=len(rows), first_stamp_ns=ns(rows[0][0]) if rows else None,
            last_stamp_ns=ns(rows[-1][0]) if rows else None,
            nearest_before_ns=ns(before), nearest_after_ns=ns(after),
            bracket_left_ns=None if bracket is None else ns(bracket[0]),
            bracket_right_ns=None if bracket is None else ns(bracket[1]),
            bracket_gap_s=None if bracket is None else bracket[1]-bracket[0],
            nearest_endpoint_gap_s=None if bracket is None else
                min(target-bracket[0], bracket[1]-target),
            imu_at_available=available, trace_total=total, trace_retained=len(trace),
            trace_rows=selected)

    def prepare(self, pair):
        began = time.monotonic()
        wait_reason, polls, waits = 'waiting_calibration', 0, Counter()
        imu, available, succeeded, sampled = [], None, False, None
        try:
            while time.monotonic()-began <= self.args.readiness_wait_wall_s:
                polls += 1
                with self.lock:
                    if self.closing or self.fault:
                        raise ValueError(self.fault or 'shutdown_before_preparation')
                    rgb_info, depth_info = self.info['rgb'], self.info['depth']
                    imu = list(self.imu)
                    sampled = time.monotonic_ns()
                if rgb_info is None or depth_info is None:
                    wait_reason = 'waiting_calibration'
                elif rgb_info != depth_info:
                    self.set_fault('rgb_depth_calibration_mismatch')
                    raise ValueError('rgb_depth_calibration_mismatch')
                else:
                    frame, width, height, raw_K = rgb_info[:4]
                    if (pair.frame_id != frame or (pair.rgb.width, pair.rgb.height) != (width, height)
                            or (pair.depth.width, pair.depth.height) != (width, height)):
                        raise ValueError('rgb_depth_calibration_frame_or_shape_mismatch')
                    attitude = imu_at(imu, pair.stamp_ns/1e9)
                    available = attitude is not None
                    if attitude is None:
                        wait_reason = 'waiting_exact_imu_bracket'
                    else:
                        try:
                            tf = self.buffer.lookup_transform('base_footprint', frame,
                                Time(nanoseconds=pair.stamp_ns))
                        except TransformException:
                            wait_reason = 'waiting_exact_exposure_camera_tf'
                        else:
                            q, t = tf.transform.rotation, tf.transform.translation
                            Rbc = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                            lever = np.array([t.x, t.y, t.z])
                            if not np.isfinite(lever).all():
                                raise ValueError('nonfinite_camera_transform')
                            K = np.asarray(raw_K).reshape(3, 3)
                            optical = depth_grid(pair.depth, (K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
                            sensor = make_sensor_frame(pair.stamp_ns, gray_image(pair.rgb), optical,
                                attitude[0].as_matrix(), Rbc, lever, K, outside_rover)
                            succeeded, wait_reason = True, 'ready'
                            return sensor
                waits[wait_reason] += 1
                time.sleep(.005)
            with self.lock:
                self.counts['readiness_timeouts'] += 1
            raise ValueError('readiness_timeout: '+wait_reason)
        finally:
            self.preparation_diagnostics = dict(wait_reason=wait_reason, polls=polls,
                wait_polls=dict(waits), buffer_snapshot_wall_ns=sampled,
                imu=self._imu_bracket_diagnostics(imu, pair.stamp_ns, available, not succeeded))

    @staticmethod
    def _timing_mark():
        return time.monotonic_ns(), time.thread_time_ns(), time.process_time_ns()

    @staticmethod
    def _timing_delta(start, end):
        return dict(zip(('wall_s', 'thread_cpu_s', 'process_cpu_s'),
                        ((b-a)/1e9 for a, b in zip(start, end))))

    def work(self):
        worker_id = threading.get_native_id()
        while True:
            pair = self.mailbox.take()
            if pair is None:
                if self.mailbox.closed:
                    return
                continue
            started = time.monotonic_ns()
            preparation_begin = self._timing_mark()
            preparation_end = processing_begin = processing_end = row = None
            self.preparation_diagnostics = {}
            try:
                sensor = self.prepare(pair)
                prepared = time.monotonic_ns()
                preparation_end = self._timing_mark()
                before_gauge = self.processor.gauge_stamp_ns
                processing_begin = self._timing_mark()
                row = self.processor.process(sensor, pair.planned_stamp_ns,
                    pair.received_wall_ns, started, self.sim_now)
                processing_end = self._timing_mark()
                if row is not None:
                    row['preparation_s'] = (prepared-started)/1e9
                    row.update(offer_wall_ns=getattr(pair, 'offer_wall_ns', None),
                        take_wall_ns=started, worker_native_id=worker_id,
                        preparation_diagnostics=self.preparation_diagnostics)
                    row.setdefault('timing_phases', {}).update(
                        node_prepare=self._timing_delta(preparation_begin, preparation_end),
                        node_processor=self._timing_delta(processing_begin, processing_end))
                    with self.lock:
                        fault = self.fault
                    if fault:
                        row.update(output=False, reason='source_fault_during_processing: '+fault)
                        row.pop('position', None)
                        row.pop('covariance_conditional', None)
                    self.write_plan(row)
                if self.processor.gauge_stamp_ns != before_gauge:
                    self.event(type='gauge', gauge_stamp_ns=self.processor.gauge_stamp_ns,
                        initial_imu_yaw_rad=self.initial_imu_yaw_rad, axes='initial_body_yaw')
                self.exposure(pair, 'processed')
            except Exception as error:
                ended = self._timing_mark()
                if preparation_end is None:
                    preparation_end = ended
                if processing_end is None:
                    processing_end = ended
                reason = str(error) if isinstance(error, ValueError) else 'processing_exception: '+repr(error)
                if not isinstance(error, ValueError):
                    self.set_fault('shadow_processing_exception')
                self.exposure(pair, 'rejected', reason)
                phases = dict(node_prepare=self._timing_delta(preparation_begin, preparation_end))
                if processing_begin is not None:
                    phases['node_processor'] = self._timing_delta(processing_begin, processing_end)
                row = self.fail_plan(pair.planned_stamp_ns, pair.stamp_ns, reason,
                    pair.received_wall_ns, started, diagnostics=dict(
                        offer_wall_ns=getattr(pair, 'offer_wall_ns', None), take_wall_ns=started,
                        worker_native_id=worker_id, preparation_diagnostics=self.preparation_diagnostics,
                        preparation_s=(preparation_end[0]-started)/1e9, timing_phases=phases))
            # Preserve the existing finished_wall_ns contract. This separate
            # event marks completion of the original plan/exposure writes.
            after_write = self._timing_mark()
            write_begin = processing_end if processing_begin is not None else preparation_end
            self.event(type='worker_complete', stamp_ns=pair.stamp_ns,
                planned_stamp_ns=pair.planned_stamp_ns,
                offer_wall_ns=getattr(pair, 'offer_wall_ns', None), take_wall_ns=started,
                original_finished_wall_ns=None if row is None else row['finished_wall_ns'],
                after_write_wall_ns=after_write[0], worker_native_id=worker_id,
                worker_thread_cpu_s=(after_write[1]-preparation_begin[1])/1e9,
                timing_phases=dict(node_after_write=self._timing_delta(write_begin, after_write)))

    def tick(self):
        # /clock can precede delivery of the corresponding RGB/depth messages.
        # Only a later real exposure proves skipped slots; finish records the
        # unobserved tail. Never consume a late image's plan in this timer.
        self.guard_clock()

    def finish(self):
        with self.lock:
            self.closing = True
        pending = self.mailbox.close()
        if pending is not None:
            self.exposure(pending, 'rejected', 'shutdown_before_processing')
            self.fail_plan(pending.planned_stamp_ns, pending.stamp_ns,
                           'shutdown_before_processing', pending.received_wall_ns)
        self.worker.join(timeout=10.)
        if self.worker.is_alive():
            self.event(type='shutdown_incomplete', reason='worker_not_terminal_after_10_wall_s',
                       sim_ns=self.sim_now(), wall_ns=time.monotonic_ns())
            raise RuntimeError('shadow worker failed to stop; no complete run_end written')
        now = self.sim_now()
        with self.lock:
            for due in self.scheduler.expire(now, final=True):
                self.fail_plan(due, None, 'missing_exposure_at_shutdown')
            counts = dict(self.counts)
            imu_header_summary = dict(getattr(self, 'imu_header_summary', {}))
        self.event(type='run_end', complete=True, sim_ns=now, finished_wall_ns=time.monotonic_ns(),
            counts=counts, fault=self.fault, axes='initial_body_yaw',
            gauge_stamp_ns=self.processor.gauge_stamp_ns,
            initial_imu_yaw_rad=self.initial_imu_yaw_rad,
            imu_header_summary=imu_header_summary,
            remaining_unmatched={k: len(v) for k, v in self.images.items()}, worker_terminal=True)
        self.writer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--image-reliability', choices=('best_effort', 'reliable'), default='best_effort')
    parser.add_argument('--readiness-wait-wall-s', type=float, default=.5)
    parser.add_argument('--cached-query', action='store_true',
        help='explicit shadow-only versioned correlated query cache; no legacy fallback')
    args, ros_args = parser.parse_known_args()
    if not 0. < args.readiness_wait_wall_s <= 2.:
        parser.error('readiness wait must be >0 and <=2 wall seconds')
    rclpy.init(args=ros_args)
    node = ShadowNode(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.finish()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
