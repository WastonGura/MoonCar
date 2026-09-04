from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from cmd_vel_safety_relay import CmdVelSafetyRelay


class RelayTests(unittest.TestCase):
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
        finally:
            node.destroy_node()
            rclpy.shutdown()
