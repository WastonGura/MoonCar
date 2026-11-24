from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from pan_tilt_controller import PanTiltController


class PanTiltControllerTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = PanTiltController()
        self.node.pub_pan = Mock()
        self.node.pub_tilt = Mock()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_clamping(self):
        # Normal in-range values
        pan, tilt = self.node.clamp(0.5, -0.2)
        self.assertAlmostEqual(pan, 0.5)
        self.assertAlmostEqual(tilt, -0.2)

        # Exceeding upper limits
        pan, tilt = self.node.clamp(3.0, 1.0)
        self.assertAlmostEqual(pan, 2.35619)
        self.assertAlmostEqual(tilt, 0.610865)

        # Exceeding lower limits
        pan, tilt = self.node.clamp(-3.0, -1.0)
        self.assertAlmostEqual(pan, -2.35619)
        self.assertAlmostEqual(tilt, -0.610865)

    def test_smooth_interpolation(self):
        # Target far away
        self.node.current_pan = 0.0
        self.node.target_pan = 1.0
        # dt = 1/30, max_velocity = 2.0 -> max_step = 2.0 / 30 = 0.0667
        self.node.on_control_loop()
        self.assertAlmostEqual(self.node.current_pan, 2.0 / 30.0, places=4)
        self.node.pub_pan.publish.assert_called()

    def test_joint_state_tracking(self):
        msg = JointState()
        msg.name = ["front_left_wheel_joint", "camera_pan_joint", "camera_tilt_joint"]
        msg.position = [0.0, 0.785, -0.261]
        self.node.on_joint_states(msg)
        self.assertTrue(self.node.measured_valid)
        self.assertAlmostEqual(self.node.measured_pan, 0.785)
        self.assertAlmostEqual(self.node.measured_tilt, -0.261)
