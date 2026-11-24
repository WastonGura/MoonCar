"""Streaming, passive Gazebo/ROS witness for the simulation boundary campaign.

Run in the experiment's ROS_DOMAIN_ID and GZ_PARTITION, after sourcing ROS.
Nothing in this program publishes a ROS message, changes physics, or commands a
vehicle. Readiness deliberately does not require localization to be healthy.
Raw truth is the evaluation denominator, including truth without a matching TF.
"""
import argparse
from collections import Counter, deque
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time
import traceback


# Audited against experiment_motion.py, laser_odometry_node.py,
# level_scan_filter.py, wheel_odometry.py, map_recorder.py and bridge YAML.
# A subscription is a ROS graph participant even when it never publishes.
TOPIC_TYPES = {
    '/clock': 'rosgraph_msgs/msg/Clock',
    '/mooncar/clock_raw': 'rosgraph_msgs/msg/Clock',
    '/mooncar/scan': 'sensor_msgs/msg/LaserScan',
    '/mooncar/scan_level': 'sensor_msgs/msg/LaserScan',
    '/mooncar/scan_localized': 'sensor_msgs/msg/LaserScan',
    '/mooncar/imu': 'sensor_msgs/msg/Imu',
    '/cmd_vel': 'geometry_msgs/msg/Twist',
    '/mooncar/cmd_vel_safe': 'geometry_msgs/msg/Twist',
    '/mooncar/laser_odometry/status': 'std_msgs/msg/String',
    '/mooncar/workbench/permit': 'std_msgs/msg/Bool',
    '/mooncar/laser_odometry': 'nav_msgs/msg/Odometry',
    '/odometry/filtered': 'nav_msgs/msg/Odometry',
    '/odom': 'nav_msgs/msg/Odometry',
    '/mooncar/map/status': 'diagnostic_msgs/msg/DiagnosticArray',
    '/mooncar/map/quality': 'std_msgs/msg/String',
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
}


def survey_map_payload(text):
    """Keep real reconstructed cells unchanged; reject a status message as a map."""
    value=json.loads(text)
    if (not isinstance(value,dict) or not isinstance(value.get('cells'),list)
            or type(value.get('frames')) is not int or value['frames']<0
            or type(value.get('resolution')) not in (int,float) or not math.isfinite(value['resolution']) or value['resolution']<=0
            or type(value.get('stamp')) not in (int,float) or not math.isfinite(value['stamp']) or value['stamp']<0):
        raise ValueError('Invalid reconstructed survey map header')
    return value


def finite_json(value):
    """Keep nonfinite sensor metadata JSON-safe without inventing a measurement."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(item) for item in value]
    return value


def write_json(path, value):
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(finite_json(value), ensure_ascii=False,
                                    indent=2, allow_nan=False) + '\n', encoding='utf-8')
    temporary.replace(path)


def angles(q):
    """Return Euler radians; retain the original quaternion separately."""
    if len(q) != 4 or not all(math.isfinite(value) for value in q):
        return None, None, None
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1e-12:
        return None, None, None
    x, y, z, w = (value / norm for value in q)
    return (math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)),
            math.asin(max(-1., min(1., 2 * (w*y - z*x)))),
            math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)))


def wrapped(value):
    return math.atan2(math.sin(value), math.cos(value))


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def make_node(args, folder, stopped):
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.clock import ClockType
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                          qos_profile_sensor_data)
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Bool, String
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformException, TransformListener

    class Observer(Node):
        def __init__(self):
            super().__init__('simulation_boundary_observer', parameter_overrides=[
                Parameter('use_sim_time', value=True)])
            self.buffer = Buffer(cache_time=Duration(seconds=20.), node=self)
            self.listener = TransformListener(self.buffer, self)
            self.lock = threading.Lock()
            self.files = {name: (folder / (name + '.jsonl')).open('x', encoding='utf-8', buffering=1)
                          for name in ('truth', 'alignment', 'samples', 'tf', 'missing','survey-map')}
            self.counts = Counter()
            self.pending = deque()
            self.pending_limit = 4096
            self.match_wait_wall_s = 5.
            self.errors = []
            self.last = {}
            self.distinct = Counter()
            self.last_stamps = {}
            self.quality = None
            self.last_survey_map_key = None
            self.first_truth_stamp = self.last_truth_stamp = None
            self.first_clock_stamp = self.last_clock_stamp = None
            self.ready_written = False
            self.started_wall = time.monotonic()
            self.started_epoch_ns = time.time_ns()
            self.closing = False
            self.gz = self.reader = self.stderr = None

            def subscribe(message_type, topic, handler, qos):
                declared_type = message_type.__module__.split('.')[0] + '/msg/' + message_type.__name__
                if TOPIC_TYPES.get(topic) != declared_type:
                    raise ValueError(f'Observer topic/type contract mismatch: {topic} -> {declared_type}')
                def guarded(message):
                    try:
                        handler(topic, message)
                    except BaseException as error:
                        self.fail('ros-callback:' + topic, error)
                self.create_subscription(message_type, topic, guarded, qos)

            subscribe(Clock, '/clock', self.clock_seen, qos_profile_sensor_data)
            subscribe(Clock, '/mooncar/clock_raw', self.clock_seen, qos_profile_sensor_data)
            for topic in ('/mooncar/scan', '/mooncar/scan_level', '/mooncar/scan_localized'):
                subscribe(LaserScan, topic, self.scan_seen, qos_profile_sensor_data)
            subscribe(Imu, '/mooncar/imu', self.imu_seen, qos_profile_sensor_data)
            for topic in ('/cmd_vel', '/mooncar/cmd_vel_safe'):
                subscribe(Twist, topic, self.command_seen, QoSProfile(depth=10))
            subscribe(String, '/mooncar/laser_odometry/status', self.string_seen, QoSProfile(depth=10))
            subscribe(String, '/mooncar/map/quality', self.survey_map_seen, QoSProfile(depth=1))
            subscribe(Bool, '/mooncar/workbench/permit', self.permit_seen, QoSProfile(depth=1))
            for topic in ('/mooncar/laser_odometry', '/odometry/filtered', '/odom'):
                subscribe(Odometry, topic, self.odom_seen, qos_profile_sensor_data)
            subscribe(DiagnosticArray, '/mooncar/map/status', self.map_seen, QoSProfile(depth=10))
            subscribe(TFMessage, '/tf', self.tf_seen, QoSProfile(depth=100))
            subscribe(TFMessage, '/tf_static', self.tf_seen, QoSProfile(
                depth=100, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))

            self.stderr = (folder / 'truth-stderr.log').open('x', encoding='utf-8')
            self.gz = subprocess.Popen([
                'gz', 'topic', '-e', '-t', f'/world/{args.world}/dynamic_pose/info', '--json-output'],
                stdout=subprocess.PIPE, stderr=self.stderr, text=True, bufsize=1,
                start_new_session=True)
            self.reader = threading.Thread(target=self.receive_truth, name='passive-gazebo-truth', daemon=True)
            self.reader.start()

        def fail(self, where, error):
            with self.lock:
                self.errors.append(dict(where=where, error=repr(error),
                                        traceback=traceback.format_exc(), wall=time.monotonic()))
            stopped.set()

        def emit(self, name, row):
            line = json.dumps(finite_json(row), ensure_ascii=False, allow_nan=False) + '\n'
            with self.lock:
                self.files[name].write(line)

        def envelope(self, kind, topic):
            wall = time.monotonic()
            self.counts[topic] += 1
            self.last[topic] = wall
            return dict(kind=kind, topic=topic, wall=wall, wall_epoch_ns=time.time_ns(),
                        sim=self.get_clock().now().nanoseconds / 1e9)

        def measurement(self, topic, value):
            previous = self.last_stamps.get(topic)
            if previous is None or value != previous:
                self.distinct[topic] += 1
            if previous is not None and value < previous:
                self.counts['stamp_backwards:' + topic] += 1
            self.last_stamps[topic] = value

        def clock_seen(self, topic, msg):
            value = stamp_ns(msg.clock)
            self.measurement(topic, value)
            if topic == '/clock':
                if self.first_clock_stamp is None:
                    self.first_clock_stamp = value
                self.last_clock_stamp = value
            self.emit('samples', dict(self.envelope('clock', topic), stamp=value/1e9, stamp_ns=value))

        def scan_seen(self, topic, msg):
            value = stamp_ns(msg.header.stamp)
            self.measurement(topic, value)
            finite = [float(item) for item in msg.ranges if math.isfinite(item)]
            valid = sum(msg.range_min < item < msg.range_max for item in finite)
            self.emit('samples', dict(self.envelope('scan', topic), stamp=value/1e9, stamp_ns=value,
                frame=msg.header.frame_id, range_count=len(msg.ranges), finite_ranges=len(finite),
                valid_ranges=valid, range_min=float(msg.range_min), range_max=float(msg.range_max),
                finite_min=min(finite) if finite else None, finite_max=max(finite) if finite else None))

        def imu_seen(self, topic, msg):
            value = stamp_ns(msg.header.stamp)
            q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            roll, pitch, yaw = angles(q)
            self.emit('samples', dict(self.envelope('imu', topic), stamp=value/1e9, stamp_ns=value,
                q=q, roll=roll, pitch=pitch, yaw=yaw,
                angular_velocity=[msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                linear_acceleration=[msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]))

        def command_seen(self, topic, msg):
            self.emit('samples', dict(self.envelope('command', topic), linear=msg.linear.x,
                angular=msg.angular.z, components=[msg.linear.x, msg.linear.y, msg.linear.z,
                                                  msg.angular.x, msg.angular.y, msg.angular.z]))

        def string_seen(self, topic, msg):
            row = self.envelope('quality', topic)
            row['payload'] = json.loads(msg.data)
            self.quality = row['payload']
            self.emit('samples', row)

        def permit_seen(self, topic, msg):
            self.emit('samples', dict(self.envelope('permit', topic), data=bool(msg.data)))

        def survey_map_seen(self,topic,msg):
            received=self.envelope('reconstructed-survey-map',topic)
            payload=survey_map_payload(msg.data)
            key=(payload['stamp'],payload['frames'])
            if key!=self.last_survey_map_key:
                self.emit('survey-map',dict(received,payload=payload))
                self.last_survey_map_key=key

        def odom_seen(self, topic, msg):
            value = stamp_ns(msg.header.stamp)
            self.measurement(topic, value)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.emit('samples', dict(self.envelope('odometry', topic), stamp=value/1e9, stamp_ns=value,
                frame=msg.header.frame_id, child=msg.child_frame_id,
                xyz=[p.x, p.y, p.z], q=[q.x, q.y, q.z, q.w]))

        def map_seen(self, topic, msg):
            entries = []
            for item in msg.status:
                level = item.level
                if isinstance(level, (bytes, bytearray, memoryview)):
                    level = int.from_bytes(level, 'little')
                entries.append(dict(name=item.name, level=int(level), message=item.message,
                                    values={value.key: value.value for value in item.values}))
            self.emit('samples', dict(self.envelope('map-status', topic), status=entries))

        def tf_seen(self, topic, msg):
            base = self.envelope('tf-static' if topic == '/tf_static' else 'tf', topic)
            for transform in msg.transforms:
                value = stamp_ns(transform.header.stamp)
                p, q = transform.transform.translation, transform.transform.rotation
                self.emit('tf', dict(base, parent=transform.header.frame_id,
                    child=transform.child_frame_id, stamp=value/1e9, stamp_ns=value,
                    xyz=[p.x, p.y, p.z], q=[q.x, q.y, q.z, q.w]))

        def receive_truth(self):
            try:
                for line in self.gz.stdout:
                    if not line.strip():
                        continue
                    data = json.loads(line)
                    body = next((item for item in data.get('pose', []) if item.get('name') == 'mooncar'), None)
                    if body is None:
                        continue
                    stamp = data['header']['stamp']
                    value = int(stamp.get('sec', 0)) * 1_000_000_000 + int(stamp.get('nsec', 0))
                    position, orientation = body.get('position', {}), body.get('orientation', {})
                    xyz = [float(position.get(key, 0.)) for key in ('x', 'y', 'z')]
                    q = [float(orientation.get(key, 1. if key == 'w' else 0.)) for key in ('x', 'y', 'z', 'w')]
                    if not all(math.isfinite(item) for item in xyz + q):
                        raise ValueError('Gazebo truth contains nonfinite position/quaternion')
                    roll, pitch, yaw = angles(q)
                    if yaw is None:
                        raise ValueError('Gazebo truth quaternion is invalid')
                    wall = time.monotonic()
                    with self.lock:
                        self.counts['truth'] += 1
                        sequence = self.counts['truth']
                        self.last['truth'] = wall
                        previous = self.last_truth_stamp
                        if previous is not None and value < previous:
                            self.counts['stamp_backwards:truth'] += 1
                        if previous is None or previous != value:
                            self.distinct['truth'] += 1
                        if self.first_truth_stamp is None:
                            self.first_truth_stamp = value
                        self.last_truth_stamp = value
                    row = dict(sequence=sequence, stamp=value/1e9, stamp_ns=value, wall=wall,
                               wall_epoch_ns=time.time_ns(), xyz=xyz, q=q, roll=roll, pitch=pitch, yaw=yaw)
                    self.emit('truth', row)
                    with self.lock:
                        evicted = self.pending.popleft() if len(self.pending) >= self.pending_limit else None
                        self.pending.append(dict(row=row, last_tf_error=None))
                    if evicted is not None:
                        self.missing(evicted, 'pending_capacity_eviction')
                if not self.closing:
                    raise RuntimeError(f'Gazebo truth stream ended unexpectedly; exit={self.gz.poll()}')
            except BaseException as error:
                if not self.closing:
                    self.fail('gazebo-truth-reader', error)

        def missing(self, item, reason):
            row = item['row']
            self.counts['missing:' + reason] += 1
            self.emit('missing', dict(truth_sequence=row['sequence'], stamp=row['stamp'], stamp_ns=row['stamp_ns'],
                reason=reason, last_tf_error=item['last_tf_error'], wait_wall_s=time.monotonic()-row['wall']))

        def match_pending(self, terminal=False):
            with self.lock:
                pending = list(self.pending)
                self.pending.clear()
            retained = []
            for item in pending:
                row = item['row']
                try:
                    # ROS time is used explicitly, never wall time or latest TF.
                    tf = self.buffer.lookup_transform('map', 'base_footprint',
                        Time(nanoseconds=row['stamp_ns'], clock_type=ClockType.ROS_TIME))
                except TransformException as error:
                    item['last_tf_error'] = str(error)
                    if terminal:
                        self.missing(item, 'capture_ended_without_matching_tf')
                    elif time.monotonic()-row['wall'] >= self.match_wait_wall_s:
                        self.missing(item, 'tf_wait_wall_timeout')
                    else:
                        retained.append(item)
                    continue
                p, q = tf.transform.translation, tf.transform.rotation
                xyz, orientation = [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]
                roll, pitch, yaw = angles(orientation)
                if yaw is None or not all(math.isfinite(value) for value in xyz):
                    item['last_tf_error'] = 'nonfinite or invalid map pose'
                    self.missing(item, 'invalid_transform')
                    continue
                latest_stamp = None
                try:
                    latest_stamp = stamp_ns(self.buffer.lookup_transform('map', 'base_footprint', Time()).header.stamp)
                except TransformException:
                    pass
                self.emit('alignment', dict(truth_sequence=row['sequence'], stamp=row['stamp'], stamp_ns=row['stamp_ns'],
                    wall=time.monotonic(), truth_xyz=row['xyz'], truth_q=row['q'],
                    truth_roll=row['roll'], truth_pitch=row['pitch'], truth_yaw=row['yaw'],
                    map_xyz=xyz, map_q=orientation, map_roll=roll, map_pitch=pitch, map_yaw=yaw,
                    xy_error_m=math.hypot(p.x-row['xyz'][0], p.y-row['xyz'][1]),
                    z_difference_m=p.z-row['xyz'][2], yaw_error_deg=math.degrees(wrapped(yaw-row['yaw'])),
                    lookup_tf_stamp_ns=stamp_ns(tf.header.stamp), latest_map_base_stamp_ns=latest_stamp,
                    match_delay_wall_s=time.monotonic()-row['wall']))
                self.counts['matched'] += 1
            with self.lock:
                # The reader may have added new rows during TF lookups.
                combined = deque(retained)
                combined.extend(self.pending)
                overflow = []
                while len(combined) > self.pending_limit:
                    overflow.append(combined.popleft())
                self.pending = combined
            for item in overflow:
                self.missing(item, 'pending_capacity_eviction')

        def transform_state(self):
            result = {}
            for target, source in (('map', 'survey_odom'), ('survey_odom', 'odom'), ('map', 'base_footprint')):
                key = target + '/' + source
                try:
                    tf = self.buffer.lookup_transform(target, source, Time())
                    p, q = tf.transform.translation, tf.transform.rotation
                    orientation = [q.x, q.y, q.z, q.w]
                    roll, pitch, yaw = angles(orientation)
                    result[key] = dict(stamp_ns=stamp_ns(tf.header.stamp), xyz=[p.x, p.y, p.z],
                                       q=orientation, roll=roll, pitch=pitch, yaw=yaw)
                    if key == 'survey_odom/odom' and yaw is not None:
                        result[key]['setup_xy_error_m'] = math.hypot(p.x-args.start_x, p.y-args.start_y)
                        result[key]['setup_yaw_error_rad'] = wrapped(yaw-args.start_yaw)
                except TransformException as error:
                    result[key] = dict(error=str(error))
            return result

        def snapshot(self):
            wall = time.monotonic()
            with self.lock:
                counts, distinct, last = dict(self.counts), dict(self.distinct), dict(self.last)
                pending, errors = len(self.pending), list(self.errors)
                first_truth, last_truth = self.first_truth_stamp, self.last_truth_stamp
            live = {key: dict(receipt_age_wall_s=wall-last[key], distinct_stamps=distinct.get(key, 0),
                             live=wall-last[key] <= 3. and distinct.get(key, 0) >= 3)
                    for key in ('truth', '/mooncar/scan', '/clock') if key in last}
            raw_ready = (len(live) == 3 and all(value['live'] for value in live.values())
                         and self.first_clock_stamp is not None and self.last_clock_stamp > self.first_clock_stamp)
            result = dict(schema_version=1, wall=wall, elapsed_wall_s=wall-self.started_wall,
                sim=self.get_clock().now().nanoseconds/1e9, counts=counts, distinct_stamp_counts=distinct,
                truth_count=counts.get('truth', 0), matched_count=counts.get('matched', 0),
                missing_count=sum(value for key, value in counts.items() if key.startswith('missing:')),
                pending_truth=pending, pending_limit=self.pending_limit, match_wait_wall_s=self.match_wait_wall_s,
                first_truth_stamp_ns=first_truth, last_truth_stamp_ns=last_truth,
                first_clock_stamp_ns=self.first_clock_stamp, last_clock_stamp_ns=self.last_clock_stamp,
                last_stamps_ns=dict(self.last_stamps), live=live, raw_observation_ready=raw_ready,
                quality=self.quality, transforms=self.transform_state(), errors=errors)
            if raw_ready and not self.ready_written:
                write_json(folder/'ready.json', dict(result,
                    definition='At least three distinct advancing raw truth/scan/clock stamps with live receipt; health and TF are not prerequisites',
                    health_required=False, truth_used_by_control=False))
                self.ready_written = True
            write_json(folder/'status.json', result)
            return result

        def close_truth(self):
            self.closing = True
            if self.gz is not None and self.gz.poll() is None:
                for sig, timeout in ((signal.SIGINT, 3), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
                    try:
                        os.killpg(self.gz.pid, sig)
                    except ProcessLookupError:
                        break
                    try:
                        self.gz.wait(timeout=timeout)
                        break
                    except subprocess.TimeoutExpired:
                        continue
            if self.reader is not None:
                self.reader.join(timeout=5)
                if self.reader.is_alive():
                    raise RuntimeError('Gazebo reader did not finish; do not close files under its thread')
            if self.gz is not None and self.gz.stdout is not None:
                self.gz.stdout.close()
            if self.stderr is not None:
                self.stderr.close()

        def close_files(self):
            for stream in self.files.values():
                stream.flush()
                stream.close()

    return Observer()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', required=True, type=Path)
    parser.add_argument('--world', required=True)
    parser.add_argument('--start-x', type=float, default=0.)
    parser.add_argument('--start-y', type=float, default=0.)
    parser.add_argument('--start-yaw', type=float, default=0.)
    parser.add_argument('--duration-wall', type=float)
    args = parser.parse_args(argv)
    if not re.fullmatch(r'[A-Za-z0-9_]+', args.world):
        parser.error('world must be an SDF world name')
    if not all(math.isfinite(value) for value in (args.start_x, args.start_y, args.start_yaw)):
        parser.error('known setup coordinates must be finite')
    if args.duration_wall is not None and (not math.isfinite(args.duration_wall) or args.duration_wall <= 0):
        parser.error('duration-wall must be finite and positive')
    folder = args.folder.resolve()
    folder.mkdir(parents=False, exist_ok=False)
    write_json(folder/'observer-contract.json', dict(schema_version=1,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), world=args.world,
        topic_types=TOPIC_TYPES,
        ros_domain_id=os.environ.get('ROS_DOMAIN_ID'), gz_partition=os.environ.get('GZ_PARTITION'),
        setup=dict(start_x=args.start_x, start_y=args.start_y, start_yaw=args.start_yaw),
        frame_method='navigation.launch.py publishes survey_odom->odom at declared initial XY/Z/yaw; map->base lookup already includes it; world/map identity, no extra or fitted correction',
        expected_survey_odom_to_odom_xy_yaw=[args.start_x, args.start_y, args.start_yaw],
        coordinate_alignment_fitted=False, truth_used_by_control=False,
        no_ros_publishers_or_control_calls=True,
        missing_truth_policy='Every raw truth row is retained; terminal unmatched rows receive explicit missing records. Truth transport itself may drop messages; inspect timestamp gaps.',
        nonfinite_policy='Nonfinite scan/metadata floats become JSON null; invalid truth is a fatal instrumentation error',
        duration_wall_s=args.duration_wall))

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    stopped = threading.Event()
    reason = {'value': 'duration-wall' if args.duration_wall is not None else 'requested-stop'}

    def stop(signum, _frame):
        reason['value'] = signal.Signals(signum).name
        stopped.set()

    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = executor = None
    failures = []
    initialized = False
    started = time.monotonic()
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = make_node(args, folder, stopped)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        next_match = next_snapshot = time.monotonic()
        while not stopped.is_set():
            executor.spin_once(timeout_sec=.05)
            now = time.monotonic()
            if now >= next_match:
                node.match_pending()
                next_match = now + .1
            if now >= next_snapshot:
                node.snapshot()
                next_snapshot = now + 1.
            if args.duration_wall is not None and now-started >= args.duration_wall:
                break
    except BaseException as error:
        failures.append(dict(where='main', error=repr(error), traceback=traceback.format_exc()))
        reason['value'] = 'instrumentation-error'
    finally:
        final = None
        if node is not None:
            try:
                node.close_truth()
                # Consume already received ROS data while the truth stream is closed.
                until = time.monotonic() + .5
                while executor is not None and time.monotonic() < until:
                    executor.spin_once(timeout_sec=.02)
                node.match_pending(terminal=True)
                final = node.snapshot()
            except BaseException as error:
                failures.append(dict(where='capture-close', error=repr(error), traceback=traceback.format_exc()))
            try:
                if executor is not None:
                    if not executor.shutdown(timeout_sec=5):
                        raise RuntimeError('ROS executor shutdown timed out')
                node.listener.unregister()
                node.destroy_node()
            except BaseException as error:
                failures.append(dict(where='ros-close', error=repr(error), traceback=traceback.format_exc()))
            try:
                if node.reader is None or not node.reader.is_alive():
                    node.close_files()
                else:
                    raise RuntimeError('Reader still running; output handles kept open until process exit')
            except BaseException as error:
                failures.append(dict(where='files-close', error=repr(error), traceback=traceback.format_exc()))
        if initialized:
            try:
                rclpy.shutdown()
            except BaseException as error:
                failures.append(dict(where='rclpy-shutdown', error=repr(error)))
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        errors = (list(node.errors) if node is not None else []) + failures
        matched = node.counts['matched'] if node is not None else 0
        missing = sum(value for key, value in node.counts.items() if key.startswith('missing:')) if node is not None else 0
        truths = node.counts['truth'] if node is not None else 0
        conservation = truths == matched + missing
        result = dict(schema_version=1, capture_completed=not errors and conservation and node is not None,
            stop_reason=reason['value'], errors=errors, elapsed_wall_s=time.monotonic()-started,
            ready=bool(node is not None and node.ready_written), truth_rows=truths,
            matched_rows=matched, missing_rows=missing, truth_rows_accounted_for=conservation,
            reader_alive=bool(node is not None and node.reader is not None and node.reader.is_alive()),
            gz_reader_exit_code=node.gz.poll() if node is not None and node.gz is not None else None,
            start_sim_s=node.first_truth_stamp/1e9 if node is not None and node.first_truth_stamp is not None else None,
            end_sim_s=node.last_truth_stamp/1e9 if node is not None and node.last_truth_stamp is not None else None,
            final_status=final, truth_used_by_control=False, coordinate_alignment_fitted=False,
            note='Capture completion is an instrumentation result; it does not imply healthy localization, accurate poses, or mission scope completion.')
        write_json(folder/'complete.json', result)
        print(json.dumps({key: result[key] for key in ('capture_completed', 'ready', 'truth_rows', 'matched_rows', 'missing_rows', 'errors')}, ensure_ascii=False), flush=True)
    return 0 if result['capture_completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
