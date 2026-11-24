"""Exercise the laser gate through real ROS topics, without Gazebo or hardware.

Run in the sourced ROS environment. The two cases use isolated domains 96/95
and a frozen, published ROS clock so wall freshness is tested independently of
the accepted observation's simulation age.
"""
import json
from pathlib import Path
import sys
import time
import unittest

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts'))
from cmd_vel_safety_relay import CmdVelSafetyRelay
from localization_quality import LaserQualitySource


class LaserInterlockTransportTests(unittest.TestCase):
    def setUp(self):
        self.relay = self.peer = self.executor = None
        domain = 95 if 'heartbeat' in self._testMethodName else 96
        rclpy.init(domain_id=domain, args=[
            '--ros-args', '-p', 'use_sim_time:=true',
            '-p', 'require_laser_localization:=true',
            '-p', 'input_topic:=/laser_interlock_test/input',
            '-p', 'output_topic:=/laser_interlock_test/output',
            # Keep a held command fresh long enough to detect replay on recovery.
            '-p', 'timeout_s:=2.0', '-p', 'publish_rate_hz:=50.0',
        ])
        self.addCleanup(self.cleanup_ros)
        self.relay = CmdVelSafetyRelay()
        self.peer = Node('laser_interlock_transport_peer')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.relay)
        self.executor.add_node(self.peer)
        self.outputs = []
        self.collector = self.peer.create_subscription(
            Twist, '/laser_interlock_test/output',
            lambda msg: self.outputs.append((time.monotonic(), msg.linear.x, msg.angular.z)), 100)
        self.commands = self.peer.create_publisher(Twist, '/laser_interlock_test/input', 10)
        self.statuses = self.peer.create_publisher(String, '/mooncar/laser_odometry/status', 10)
        self.clock = self.peer.create_publisher(
            Clock, '/clock', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.source = LaserQualitySource()
        self.sim = 10.
        self.wait_for(lambda: self.commands.get_subscription_count() > 0
                      and self.statuses.get_subscription_count() > 0
                      and self.relay.publisher.get_subscription_count() > 0,
                      'ROS topic discovery', timeout=4.)
        self.set_clock(self.sim)

    def cleanup_ros(self):
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2.)
        if self.peer is not None:
            self.peer.destroy_node()
        if self.relay is not None:
            self.relay.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def wait_for(self, condition, description, timeout=1., drive=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if drive is not None:
                drive()
            self.executor.spin_once(timeout_sec=.005)
            if condition():
                return
        self.fail('Timed out waiting for ' + description)

    def pump(self, seconds, drive=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if drive is not None:
                drive()
            self.executor.spin_once(timeout_sec=.005)

    def set_clock(self, stamp):
        self.sim = stamp
        msg = Clock()
        msg.clock.sec = int(stamp)
        msg.clock.nanosec = round((stamp - int(stamp)) * 1e9)
        self.wait_for(lambda: self.relay.get_clock().now().nanoseconds == round(stamp * 1e9)
                      and self.peer.get_clock().now().nanoseconds == round(stamp * 1e9),
                      'published ROS clock', drive=lambda: self.clock.publish(msg))

    def command(self, velocity=.2):
        msg = Twist()
        msg.linear.x = velocity
        self.commands.publish(msg)

    def publish_status(self):
        payload = self.source.snapshot(self.sim)
        self.statuses.publish(String(data=json.dumps(payload, allow_nan=False)))
        return payload['sequence']

    def deliver_status(self):
        sequence = self.publish_status()
        self.wait_for(lambda: self.relay.localization_quality.sequence == sequence
                      or self.relay.localization_quality.fault is not None,
                      'quality status delivery')

    def heartbeat_driver(self, commands=False):
        next_status = next_command = 0.

        def drive():
            nonlocal next_status, next_command
            now = time.monotonic()
            if now >= next_status:
                self.publish_status()
                next_status = now + .05
            if commands and now >= next_command:
                self.command()
                next_command = now + .025
        return drive

    def assert_zero_outputs(self, start, duration=.06, drive=None):
        self.pump(duration, drive=drive)
        rows = self.outputs[start:]
        self.assertTrue(rows, 'Expected real output messages from the relay')
        self.assertTrue(all(x == 0. and z == 0. for _, x, z in rows), rows)

    def initial_fits(self, assert_closed=False, expect_healthy=True):
        self.source.observe(self.sim - .2, dict(accepted=True, reason='initialized'))
        self.deliver_status()
        for index, offset in enumerate((.15, .10, .05), 1):
            self.source.observe(self.sim - offset, dict(accepted=True, reason='matched'))
            start = len(self.outputs)
            self.deliver_status()
            if assert_closed and index < 3:
                self.command()
                self.assert_zero_outputs(start)
                self.assertFalse(self.relay.localization_healthy())
        self.assertEqual(self.relay.localization_healthy(), expect_healthy)

    def test_real_topics_require_three_fits_stop_and_discard_commands(self):
        start = len(self.outputs)
        self.command()
        self.assert_zero_outputs(start)
        self.initial_fits(assert_closed=True)

        start = len(self.outputs)
        self.command()
        self.wait_for(lambda: any(x == .2 for _, x, _ in self.outputs[start:]),
                      'healthy command forwarding', drive=self.heartbeat_driver())

        self.source.observe(self.sim, dict(accepted=False, reason='unobservable'))
        start = len(self.outputs)
        rejected_at = time.monotonic()
        self.deliver_status()
        self.wait_for(lambda: any(x == 0. and z == 0. for _, x, z in self.outputs[start:]),
                      'zero output after rejected fit', timeout=.3)
        first_zero = next(t for t, x, z in self.outputs[start:] if x == 0. and z == 0.)
        self.assertLess(first_zero - rejected_at, .3)
        print(f'Rejected-fit zero output latency: {first_zero - rejected_at:.3f}s', flush=True)
        self.assertFalse(self.relay.localization_healthy())
        # A fresh command arriving during loss must also be discarded.
        self.command(.3)
        self.assert_zero_outputs(len(self.outputs))

        self.set_clock(10.2)
        for stamp in (10.1, 10.15, 10.2):
            self.source.observe(stamp, dict(accepted=True, reason='matched'))
            start = len(self.outputs)
            self.deliver_status()
            self.assert_zero_outputs(start, drive=self.heartbeat_driver())
        self.assertTrue(self.relay.localization_healthy())
        # Neither the last healthy command nor the command during loss replays.
        self.assert_zero_outputs(len(self.outputs), duration=.1, drive=self.heartbeat_driver())
        start = len(self.outputs)
        self.command(.25)
        self.wait_for(lambda: any(x == .25 for _, x, _ in self.outputs[start:]),
                      'new command after recovery', drive=self.heartbeat_driver())

        # Even an already healthy replacement producer cannot clear the latch.
        self.set_clock(10.4)
        self.source = LaserQualitySource()
        self.initial_fits(expect_healthy=False)
        self.assertEqual(self.relay.localization_quality.fault, 'producer_session_changed')
        self.pump(.05)
        self.assert_zero_outputs(len(self.outputs), duration=.15,
                                 drive=self.heartbeat_driver(commands=True))

    def test_status_heartbeat_loss_stops_a_fresh_command_stream(self):
        self.initial_fits()
        start = len(self.outputs)
        self.wait_for(lambda: any(x == .2 for _, x, _ in self.outputs[start:]),
                      'initial streamed command', drive=self.heartbeat_driver(commands=True))
        self.deliver_status()
        last_status_wall = self.relay.localization_quality.received_wall
        start = len(self.outputs)
        next_command = 0.

        def fresh_commands():
            nonlocal next_command
            now = time.monotonic()
            if now >= next_command:
                self.command()
                next_command = now + .025

        self.wait_for(lambda: self.relay.localization_quality.check(
            self.sim, time.monotonic())['reason'] == 'status_stale'
            and any(x == 0. for _, x, _ in self.outputs[start:]),
            'wall watchdog despite fresh commands', timeout=.9, drive=fresh_commands)
        first_zero = next(t for t, x, _ in self.outputs[start:] if x == 0.)
        self.assertGreaterEqual(first_zero - last_status_wall, .49)
        self.assertLess(first_zero - last_status_wall, .75)
        print(f'Heartbeat-loss zero output after last status: {first_zero - last_status_wall:.3f}s', flush=True)
        self.assertLess(time.monotonic() - self.relay.last_cmd_wall, .1)
        self.assertEqual(self.relay.get_clock().now().nanoseconds, 10_000_000_000)
        self.assert_zero_outputs(len(self.outputs), duration=.15, drive=fresh_commands)


if __name__ == '__main__':
    unittest.main(verbosity=2)
