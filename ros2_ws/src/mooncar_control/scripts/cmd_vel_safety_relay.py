#!/usr/bin/env python3
import math
import json
from pathlib import Path
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.clock import Clock, ClockType

try:
    from localization_quality import LocalizationQuality
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts'))
    from localization_quality import LocalizationQuality


class CmdVelSafetyRelay(Node):
    def __init__(self):
        super().__init__("mooncar_cmd_vel_safety_relay")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/mooncar/cmd_vel_safe")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("timeout_s", 0.5)
        self.declare_parameter("max_linear_x", 0.35)
        self.declare_parameter("max_angular_z", 0.8)
        self.declare_parameter("require_terrain", False)
        self.declare_parameter("require_workbench", False, ParameterDescriptor(read_only=True))
        self.declare_parameter("require_laser_localization", False, ParameterDescriptor(read_only=True))

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_cmd_wall = time.monotonic()
        self.require_terrain = self.get_parameter("require_terrain").value
        self.require_workbench = self.get_parameter("require_workbench").value
        self.require_laser_localization = self.get_parameter("require_laser_localization").value
        self.localization_quality = LocalizationQuality(required=self.require_laser_localization)
        if self.require_laser_localization:
            self.create_subscription(String, "/mooncar/laser_odometry/status", self.on_localization_quality, 10)
        self.workbench_permit = False
        self.workbench_time = 0.0
        self.create_subscription(Bool, "/mooncar/workbench/permit", self.on_workbench, 1)
        self.terrain_ok = False
        self.terrain_time = 0.0
        self.create_subscription(DiagnosticArray, "/mooncar/terrain/status", self.on_terrain, 10)

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(Twist, input_topic, self.on_cmd_vel, 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.on_timer,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f"Relaying {input_topic} to {output_topic} with timeout {self.timeout_s:.2f}s")

    def on_cmd_vel(self, msg):
        if not self.localization_healthy():
            self.last_cmd = Twist()
            return
        self.last_cmd = self.clamp(msg)
        self.last_cmd_time = self.get_clock().now()
        self.last_cmd_wall = time.monotonic()

    def on_timer(self):
        localization_ok = self.localization_healthy()
        if not localization_ok:
            # A recovered status must never replay a command from before the fault.
            self.last_cmd = Twist()
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        terrain_stale = not self.terrain_ok or time.monotonic() - self.terrain_time > 5.0
        workbench_stale = not self.workbench_permit or time.monotonic() - self.workbench_time > 1.0
        if (age < 0 or age > self.timeout_s or time.monotonic() - self.last_cmd_wall > self.timeout_s
                or (self.require_terrain and terrain_stale)
                or (self.require_workbench and workbench_stale)
                or not localization_ok):
            self.publisher.publish(Twist())
        else:
            self.publisher.publish(self.last_cmd)

    def localization_healthy(self):
        return self.localization_quality.check(
            self.get_clock().now().nanoseconds * 1e-9, time.monotonic())['healthy']

    def on_localization_quality(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            payload = {}
        if not isinstance(payload, dict):
            payload = {}
        self.localization_quality.observe(payload, time.monotonic())
        if not self.localization_healthy():
            self.last_cmd = Twist()
            # Revoke output in this callback as well as the independent watchdog.
            self.publisher.publish(Twist())

    def on_workbench(self, msg):
        self.workbench_permit = msg.data
        self.workbench_time = time.monotonic()

    def on_terrain(self, msg):
        for status in msg.status:
            if status.name == "terrain_geometry":
                self.terrain_ok = status.level == DiagnosticStatus.OK
                self.terrain_time = time.monotonic()

    def clamp(self, msg):
        cmd = Twist()
        cmd.linear.x = self.clamp_value(msg.linear.x, self.max_linear_x)
        cmd.angular.z = self.clamp_value(msg.angular.z, self.max_angular_z)
        return cmd

    @staticmethod
    def clamp_value(value, limit):
        if not math.isfinite(value):
            return 0.0
        return max(-limit, min(limit, value))


def main():
    rclpy.init()
    node = CmdVelSafetyRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
