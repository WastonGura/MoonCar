#!/usr/bin/env python3
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class DriveSmokeTest(Node):
    def __init__(self):
        super().__init__("mooncar_drive_smoke_test")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_x", 0.16)
        self.declare_parameter("angular_z", 0.35)
        self.declare_parameter("rate_hz", 10.0)
        self.publisher = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)

    def run(self):
        self.get_logger().info("Waiting for command subscribers")
        deadline = time.monotonic() + 5.0
        while self.publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.publish_for(linear_x=float(self.get_parameter("linear_x").value), angular_z=0.0, duration_s=4.0)
        self.publish_for(linear_x=0.0, angular_z=0.0, duration_s=1.0)
        self.publish_for(linear_x=0.0, angular_z=float(self.get_parameter("angular_z").value), duration_s=3.0)
        self.publish_for(linear_x=0.0, angular_z=0.0, duration_s=1.0)
        self.get_logger().info("Drive smoke test complete")

    def publish_for(self, linear_x, angular_z, duration_s):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        period = 1.0 / float(self.get_parameter("rate_hz").value)
        end_time = time.monotonic() + duration_s
        while time.monotonic() < end_time:
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)


def main():
    rclpy.init()
    node = DriveSmokeTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
