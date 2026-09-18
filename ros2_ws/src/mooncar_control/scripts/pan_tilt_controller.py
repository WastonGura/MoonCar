#!/usr/bin/env python3
"""Pan-Tilt Gimbal Controller for MoonCar.

Provides joint position command smoothing, safety limit enforcement,
homing, and ROS 2 topic/service interfaces for both simulation (Gazebo)
and real serial bus servos.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


class PanTiltController(Node):
    def __init__(self):
        super().__init__("mooncar_pan_tilt_controller")

        self.declare_parameter("pan_min", -2.35619)    # -135 deg
        self.declare_parameter("pan_max", 2.35619)     # +135 deg
        self.declare_parameter("tilt_min", -0.610865)  # -35 deg
        self.declare_parameter("tilt_max", 0.610865)   # +35 deg
        self.declare_parameter("default_pan", 0.0)
        self.declare_parameter("default_tilt", -0.2618)  # -15 deg (cruise look-down)
        self.declare_parameter("max_velocity", 2.0)     # rad/s
        self.declare_parameter("update_rate_hz", 30.0)

        self.pan_min = float(self.get_parameter("pan_min").value)
        self.pan_max = float(self.get_parameter("pan_max").value)
        self.tilt_min = float(self.get_parameter("tilt_min").value)
        self.tilt_max = float(self.get_parameter("tilt_max").value)
        self.default_pan = float(self.get_parameter("default_pan").value)
        self.default_tilt = float(self.get_parameter("default_tilt").value)
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        update_rate_hz = float(self.get_parameter("update_rate_hz").value)

        # Current commanded positions (for interpolation)
        self.current_pan = self.default_pan
        self.current_tilt = self.default_tilt

        # Target commanded positions
        self.target_pan = self.default_pan
        self.target_tilt = self.default_tilt

        # Measured positions from /joint_states
        self.measured_pan = 0.0
        self.measured_tilt = 0.0
        self.measured_valid = False

        # Publishers to Gazebo bridge
        self.pub_pan = self.create_publisher(Float64, "/mooncar/cmd_camera_pan", 10)
        self.pub_tilt = self.create_publisher(Float64, "/mooncar/cmd_camera_tilt", 10)

        # Subscriber for direct commands: Vector3(x=pan_rad, y=tilt_rad, z=0)
        self.sub_cmd = self.create_subscription(
            Vector3, "/mooncar/pan_tilt/cmd", self.on_cmd, 10
        )

        # Subscriber to joint_states for actual feedback
        self.sub_joint_states = self.create_subscription(
            JointState, "/joint_states", self.on_joint_states, 10
        )

        # Services
        self.srv_home = self.create_service(
            Trigger, "/mooncar/pan_tilt/home", self.on_home
        )

        # Control loop timer
        self.dt = 1.0 / update_rate_hz
        self.timer = self.create_timer(self.dt, self.on_control_loop)
        self.get_logger().info("MoonCar Pan-Tilt Controller initialized.")

    def clamp(self, pan, tilt):
        clamped_pan = max(self.pan_min, min(self.pan_max, float(pan)))
        clamped_tilt = max(self.tilt_min, min(self.tilt_max, float(tilt)))
        return clamped_pan, clamped_tilt

    def on_cmd(self, msg: Vector3):
        clamped_pan, clamped_tilt = self.clamp(msg.x, msg.y)
        self.target_pan = clamped_pan
        self.target_tilt = clamped_tilt

    def on_joint_states(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if "camera_pan_joint" in name:
                self.measured_pan = pos
                self.measured_valid = True
            elif "camera_tilt_joint" in name:
                self.measured_tilt = pos
                self.measured_valid = True

    def on_home(self, request, response):
        self.target_pan = self.default_pan
        self.target_tilt = self.default_tilt
        response.success = True
        response.message = f"Homed to pan={self.default_pan:.2f}, tilt={self.default_tilt:.2f}"
        return response

    def on_control_loop(self):
        # Step interpolation towards target respecting max_velocity
        max_step = self.max_velocity * self.dt

        d_pan = self.target_pan - self.current_pan
        if abs(d_pan) <= max_step:
            self.current_pan = self.target_pan
        else:
            self.current_pan += math.copysign(max_step, d_pan)

        d_tilt = self.target_tilt - self.current_tilt
        if abs(d_tilt) <= max_step:
            self.current_tilt = self.target_tilt
        else:
            self.current_tilt += math.copysign(max_step, d_tilt)

        # Publish to Gazebo
        msg_pan = Float64()
        msg_pan.data = self.current_pan
        self.pub_pan.publish(msg_pan)

        msg_tilt = Float64()
        msg_tilt.data = self.current_tilt
        self.pub_tilt.publish(msg_tilt)


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
