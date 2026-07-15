#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """MoonCar keyboard teleop

w/s: increase/decrease forward velocity
a/d: increase/decrease yaw rate
space or x: stop
q: quit
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("mooncar_keyboard_teleop")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_step", 0.04)
        self.declare_parameter("angular_step", 0.12)
        self.declare_parameter("max_linear_x", 0.35)
        self.declare_parameter("max_angular_z", 0.8)
        self.declare_parameter("rate_hz", 10.0)
        self.publisher = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.linear_x = 0.0
        self.angular_z = 0.0

    def run(self):
        if not sys.stdin.isatty():
            self.get_logger().error("keyboard_teleop.py requires an interactive terminal")
            return 2

        original_settings = termios.tcgetattr(sys.stdin)
        print(HELP)
        try:
            tty.setcbreak(sys.stdin.fileno())
            period = 1.0 / float(self.get_parameter("rate_hz").value)
            while rclpy.ok():
                key = self.read_key(period)
                if key == "q":
                    break
                self.apply_key(key)
                self.publish_current()
                rclpy.spin_once(self, timeout_sec=0.0)
        finally:
            self.linear_x = 0.0
            self.angular_z = 0.0
            self.publish_current()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)

        return 0

    def read_key(self, timeout_sec):
        ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if not ready:
            return None
        return sys.stdin.read(1)

    def apply_key(self, key):
        if key is None:
            return
        if key == "w":
            self.linear_x += float(self.get_parameter("linear_step").value)
        elif key == "s":
            self.linear_x -= float(self.get_parameter("linear_step").value)
        elif key == "a":
            self.angular_z += float(self.get_parameter("angular_step").value)
        elif key == "d":
            self.angular_z -= float(self.get_parameter("angular_step").value)
        elif key in (" ", "x"):
            self.linear_x = 0.0
            self.angular_z = 0.0

        self.linear_x = self.clamp(
            self.linear_x,
            -float(self.get_parameter("max_linear_x").value),
            float(self.get_parameter("max_linear_x").value),
        )
        self.angular_z = self.clamp(
            self.angular_z,
            -float(self.get_parameter("max_angular_z").value),
            float(self.get_parameter("max_angular_z").value),
        )

    def publish_current(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.publisher.publish(msg)

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
