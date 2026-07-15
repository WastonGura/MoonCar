#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class WheelOdometry(Node):
    def __init__(self):
        super().__init__("mooncar_wheel_odometry")
        self.declare_parameter("wheel_radius", 0.16)
        self.declare_parameter("track_width", 0.59)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", True)

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.track_width = float(self.get_parameter("track_width").value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.left_joints = ("front_left_wheel_joint", "rear_left_wheel_joint")
        self.right_joints = ("front_right_wheel_joint", "rear_right_wheel_joint")
        self.last_left = None
        self.last_right = None
        self.last_time = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.subscription = self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)

    def on_joint_state(self, msg):
        left = self.average_position(msg, self.left_joints)
        right = self.average_position(msg, self.right_joints)
        if left is None or right is None:
            return

        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if self.last_left is None:
            self.last_left = left
            self.last_right = right
            self.last_time = stamp
            return

        dt = (stamp - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        delta_left = (left - self.last_left) * self.wheel_radius
        delta_right = (right - self.last_right) * self.wheel_radius
        delta_distance = 0.5 * (delta_left + delta_right)
        delta_yaw = (delta_right - delta_left) / self.track_width

        mid_yaw = self.yaw + 0.5 * delta_yaw
        self.x += delta_distance * math.cos(mid_yaw)
        self.y += delta_distance * math.sin(mid_yaw)
        self.yaw = self.normalize_angle(self.yaw + delta_yaw)

        linear_x = delta_distance / dt
        angular_z = delta_yaw / dt

        self.last_left = left
        self.last_right = right
        self.last_time = stamp
        self.publish_odom(msg.header.stamp, linear_x, angular_z)

    def publish_odom(self, stamp, linear_x, angular_z):
        orientation = self.yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = orientation
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.angular.z = angular_z
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = orientation
            self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def average_position(msg, joint_names):
        positions = []
        for joint_name in joint_names:
            if joint_name in msg.name:
                index = msg.name.index(joint_name)
                if index < len(msg.position):
                    positions.append(msg.position[index])
        if len(positions) != len(joint_names):
            return None
        return sum(positions) / len(positions)

    @staticmethod
    def yaw_to_quaternion(yaw):
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main():
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
