"""Pure IMU subscription/readiness contracts, executing actual node methods.

ROS transport and image decoding are fakes. The node constructor, IMU callback,
preparation loop, and installed-source imu_at implementation are real AST bodies.
These tests do not claim to exercise DDS delivery or start a worker/world.
"""
import ast
from collections import Counter, OrderedDict, deque
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


ROOT = Path(__file__).resolve().parents[1]
NODE_SOURCE = ROOT / 'scripts/rgbd_window_shadow.py'
IMU_SOURCE = ROOT / 'ros2_ws/src/mooncar_slam/scripts/terrain_odometry_node.py'


def real_imu_at():
    tree = ast.parse(IMU_SOURCE.read_text(encoding='utf-8'))
    function = next(n for n in tree.body
                    if isinstance(n, ast.FunctionDef) and n.name == 'imu_at')
    namespace = dict(Rotation=Rotation, Slerp=Slerp)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])),
                 str(IMU_SOURCE), 'exec'), namespace)
    return namespace['imu_at']


class FakeTime:
    def __init__(self, *, nanoseconds=0):
        self.nanoseconds = nanoseconds

    @classmethod
    def from_msg(cls, stamp):
        return cls(nanoseconds=stamp.sec * 1_000_000_000 + stamp.nanosec)


class FakeWallClock:
    def __init__(self):
        self.ns = 0
        self.on_sleep = None

    def monotonic(self):
        return self.ns / 1e9

    def monotonic_ns(self):
        return self.ns

    perf_counter = monotonic
    perf_counter_ns = monotonic_ns

    @staticmethod
    def thread_time_ns():
        return 0

    @staticmethod
    def process_time_ns():
        return 0

    def sleep(self, seconds):
        self.ns += round(seconds * 1e9)
        if self.on_sleep is not None:
            self.on_sleep()


class FakeQoS:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        for key, value in kwargs.items():
            setattr(self, key, value)


class FakeWorker:
    def __init__(self, **kwargs):
        self.target = kwargs['target']
        self.started = False

    def start(self):
        # Capture the real target, without executing it or starting a thread.
        self.started = True


class FakeWriter:
    def __init__(self, *args):
        self.rows = []

    def write(self, row):
        self.rows.append(row)


class FakeBuffer:
    def __init__(self, **kwargs):
        self.calls = []

    def lookup_transform(self, parent, child, stamp):
        self.calls.append((parent, child, stamp.nanoseconds))
        return SimpleNamespace(transform=SimpleNamespace(
            rotation=SimpleNamespace(x=0., y=0., z=0., w=1.),
            translation=SimpleNamespace(x=0., y=0., z=0.)))


class FakeNode:
    def __init__(self, name):
        self.subscriptions = []
        self.clock_sim_ns = 2_748_000_000

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions.append((message_type, topic, callback, qos))

    def create_timer(self, *args, **kwargs):
        return None

    def get_clock(self):
        return SimpleNamespace(now=lambda: FakeTime(nanoseconds=self.clock_sim_ns))


def actual_node():
    clock = FakeWallClock()
    calls = []

    def capture_sensor(stamp, *args):
        calls.append((stamp, args))
        return SimpleNamespace(stamp_ns=stamp)

    tree = ast.parse(NODE_SOURCE.read_text(encoding='utf-8'))
    bodies = [n for n in tree.body
              if (isinstance(n, ast.FunctionDef) and n.name == 'stamp_ns')
              or (isinstance(n, ast.ClassDef) and n.name == 'ShadowNode')]
    namespace = dict(
        Node=FakeNode, Counter=Counter, OrderedDict=OrderedDict, deque=deque,
        threading=SimpleNamespace(RLock=threading.RLock, Thread=FakeWorker),
        cv2=SimpleNamespace(setNumThreads=lambda n: None, getNumThreads=lambda: 2),
        uuid=SimpleNamespace(uuid4=lambda: 'pure-imu-test'), JsonlWriter=FakeWriter,
        ShadowProcessor=lambda **kwargs: SimpleNamespace(gauge_stamp_ns=None),
        PlanScheduler=lambda **kwargs: object(), LatestMailbox=lambda: object(),
        Buffer=FakeBuffer, TransformListener=lambda *args: None,
        QoSProfile=FakeQoS,
        HistoryPolicy=SimpleNamespace(KEEP_LAST='keep_last'),
        ReliabilityPolicy=SimpleNamespace(RELIABLE='reliable', BEST_EFFORT='best_effort'),
        DurabilityPolicy=SimpleNamespace(VOLATILE='volatile'),
        qos_profile_sensor_data=object(), Image=object(), CameraInfo=object(), Imu=object(),
        Clock=lambda **kwargs: object(), ClockType=SimpleNamespace(STEADY_TIME=object()),
        time=clock, json=__import__('json'), np=np, Rotation=Rotation,
        Time=FakeTime, TransformException=type('TransformException', (Exception,), {}),
        imu_at=real_imu_at(),
        depth_grid=lambda depth, intrinsics: np.ones((2, 2, 3)),
        gray_image=lambda rgb: np.zeros((2, 2), dtype=np.uint8),
        make_sensor_frame=capture_sensor, outside_rover=lambda points: True)
    exec(compile(ast.fix_missing_locations(ast.Module(body=bodies, type_ignores=[])),
                 str(NODE_SOURCE), 'exec'), namespace)
    args = SimpleNamespace(output='not-opened.jsonl', cached_query=True,
                           image_reliability='reliable', readiness_wait_wall_s=.5)
    node = namespace['ShadowNode'](args)
    return node, clock, calls


def imu_message(stamp_ns):
    seconds, nanos = divmod(stamp_ns, 1_000_000_000)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=seconds, nanosec=nanos),
                               frame_id='imu_frame'),
        orientation=SimpleNamespace(x=0., y=0., z=0., w=1.),
        angular_velocity=SimpleNamespace(x=0., y=0., z=.02))


def calibrated_pair(node, stamp_ns=2_900_000_000):
    info = ('camera_optical', 2, 2, (100., 0., 1., 0., 100., 1., 0., 0., 1.))
    node.info = dict(rgb=info, depth=info)
    return SimpleNamespace(stamp_ns=stamp_ns, frame_id=info[0],
                           rgb=SimpleNamespace(width=2, height=2),
                           depth=SimpleNamespace(width=2, height=2))


class ShadowImuReadinessTests(unittest.TestCase):
    def test_real_constructor_captures_bounded_reliable_imu_subscription(self):
        node, _, _ = actual_node()
        subscriptions = {topic: qos for _, topic, _, qos in node.subscriptions}
        imu = subscriptions['/mooncar/imu']
        self.assertEqual(imu.kwargs, dict(depth=200, history='keep_last',
                                         reliability='reliable', durability='volatile'))
        self.assertEqual(len(subscriptions), 5)
        for topic, qos in subscriptions.items():
            if topic != '/mooncar/imu':
                self.assertEqual(qos.depth, 2)
                self.assertEqual(qos.reliability, 'reliable')
        self.assertEqual(node.imu.maxlen, 500)
        self.assertEqual(node.args.readiness_wait_wall_s, .5)
        self.assertIs(node.worker.target.__self__, node)

    def test_two_hundred_real_callbacks_preserve_headers_and_prepare_true_bracket(self):
        node, clock, sensors = actual_node()
        stamps = [2_000_000_000 + i * 10_000_000 for i in range(200)]
        for stamp in stamps:
            node.on_imu(imu_message(stamp))
        self.assertEqual(node.counts['imu_samples'], 200)
        self.assertEqual([round(row[0] * 1e9) for row in node.imu], stamps)
        self.assertEqual(node.imu.maxlen, 500)
        pair = calibrated_pair(node)
        self.assertIsNotNone(real_imu_at()(list(node.imu), pair.stamp_ns / 1e9))
        sensor = node.prepare(pair)
        self.assertEqual(sensor.stamp_ns, 2_900_000_000)
        self.assertEqual(pair.stamp_ns, 2_900_000_000)
        self.assertNotEqual(node.sim_now(), pair.stamp_ns)  # Clock lag cannot rewrite exposure.
        self.assertEqual(sensors[0][0], 2_900_000_000)
        self.assertEqual(clock.ns, 0)
        self.assertEqual(node.buffer.calls[-1],
                         ('base_footprint', 'camera_optical', 2_900_000_000))
        self.assertEqual(node.counts['readiness_timeouts'], 0)

    def test_missing_historical_bracket_is_not_repaired_by_future_imu_or_longer_wait(self):
        node, clock, sensors = actual_node()
        for stamp in [2_840_000_000, 2_910_000_000]:
            node.on_imu(imu_message(stamp))
        pair = calibrated_pair(node)
        self.assertIsNone(real_imu_at()(list(node.imu), pair.stamp_ns / 1e9))
        next_stamp = iter(range(3_200_000_000, 5_000_000_000, 10_000_000))
        clock.on_sleep = lambda: node.on_imu(imu_message(next(next_stamp)))
        with self.assertRaisesRegex(ValueError, 'readiness_timeout: waiting_exact_imu_bracket'):
            node.prepare(pair)
        self.assertGreater(node.counts['imu_samples'], 2)
        self.assertEqual(node.counts['readiness_timeouts'], 1)
        self.assertGreaterEqual(clock.ns, 500_000_000)
        self.assertLessEqual(clock.ns, 505_001_000)  # At most the original 5ms polling overshoot.
        self.assertIsNone(real_imu_at()(list(node.imu), pair.stamp_ns / 1e9))
        self.assertEqual(pair.stamp_ns, 2_900_000_000)
        self.assertEqual(sensors, [])
        self.assertFalse(any(child == 'camera_optical' for _, child, _ in node.buffer.calls))


if __name__ == '__main__':
    unittest.main()
