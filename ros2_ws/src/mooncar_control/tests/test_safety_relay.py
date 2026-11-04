from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from cmd_vel_safety_relay import CmdVelSafetyRelay


class RelayTests(unittest.TestCase):
    def test_laser_gate_starts_closed_and_recovers_without_replaying_command(self):
        rclpy.init(args=['--ros-args', '-p', 'require_laser_localization:=true'])
        node = CmdVelSafetyRelay()
        try:
            node.publisher = Mock()
            command = Twist()
            command.linear.x = .2
            node.on_cmd_vel(command)
            node.on_timer()
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, 0.)
            # Drive monitor state independently of the status-schema unit tests.
            monitor = Mock()
            monitor.check.return_value = dict(healthy=True)
            node.localization_quality = monitor
            node.on_localization_quality(String(data='{}'))
            node.on_timer()
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, 0.)
            node.on_cmd_vel(command)
            node.on_timer()
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, .2)
            monitor.check.return_value = dict(healthy=False)
            node.on_localization_quality(String(data='{}'))
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, 0.)
            monitor.check.return_value = dict(healthy=True)
            node.on_localization_quality(String(data='{}'))
            node.on_timer()
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, 0.)
            node.on_cmd_vel(command)
            node.on_timer()
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, .2)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_laser_watchdog_discards_command_when_status_disappears(self):
        rclpy.init()
        node = CmdVelSafetyRelay()
        try:
            node.publisher = Mock()
            monitor = Mock()
            monitor.check.return_value = dict(healthy=True)
            node.localization_quality = monitor
            command = Twist()
            command.angular.z = .3
            node.on_cmd_vel(command)
            monitor.check.return_value = dict(healthy=False)
            node.on_timer()
            self.assertEqual(node.publisher.publish.call_args.args[0].angular.z, 0.)
            node.on_cmd_vel(command)  # Commands arriving during loss are discarded.
            monitor.check.return_value = dict(healthy=True)
            node.on_timer()
            self.assertEqual(node.publisher.publish.call_args.args[0].angular.z, 0.)
            node.on_cmd_vel(command)
            node.on_timer()
            self.assertEqual(node.publisher.publish.call_args.args[0].angular.z, .3)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_malformed_laser_status_fails_closed_when_required(self):
        rclpy.init(args=['--ros-args', '-p', 'require_laser_localization:=true'])
        node = CmdVelSafetyRelay()
        try:
            node.publisher = Mock()
            node.last_cmd.linear.x = .2
            node.on_localization_quality(String(data='{invalid json'))
            self.assertFalse(node.localization_healthy())
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, 0.)
            self.assertEqual(node.last_cmd.linear.x, 0.)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_disabled_laser_gate_preserves_baseline_command(self):
        rclpy.init()
        node = CmdVelSafetyRelay()
        try:
            node.publisher = Mock()
            command = Twist()
            command.linear.x = .2
            node.on_cmd_vel(command)
            node.on_localization_quality(String(data='{invalid json'))
            node.on_timer()
            self.assertTrue(node.localization_healthy())
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, .2)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_terrain_gate_and_invalid_command(self):
        rclpy.init()
        node = CmdVelSafetyRelay()
        try:
            node.publisher = Mock()
            node.require_terrain = True
            command = Twist()
            command.linear.x = 5.0
            node.on_cmd_vel(command)
            node.on_timer()
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, 0.0)
            node.on_terrain(DiagnosticArray(status=[DiagnosticStatus(
                name='terrain_geometry', level=DiagnosticStatus.OK)]))
            node.on_timer()
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, 0.35)
            node.terrain_time = time.monotonic() - 6
            node.on_timer()
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, 0.0)
            self.assertEqual(node.clamp_value(float('nan'), 0.35), 0.0)
            node.require_terrain = False
            node.require_workbench = True
            node.on_cmd_vel(command)
            node.on_timer()
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, 0.)
            node.on_workbench(Bool(data=True))
            node.on_timer()
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, .35)
            node.workbench_time = time.monotonic() - 1.1
            node.on_timer()
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, 0.)
            node.require_workbench = False
            node.last_cmd_wall = time.monotonic() - 1.
            node.on_timer()
            self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, 0.)
        finally:
            node.destroy_node()
            rclpy.shutdown()
