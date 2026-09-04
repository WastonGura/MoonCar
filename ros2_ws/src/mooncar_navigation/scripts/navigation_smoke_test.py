#!/usr/bin/env python3
import math
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class NavigationSmokeTest(Node):
    def __init__(self):
        super().__init__('mooncar_navigation_smoke_test')
        self.declare_parameter('goal_x', 1.5)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('server_timeout_s', 30.0)
        self.declare_parameter('goal_timeout_s', 120.0)
        self.declare_parameter('wall_timeout_s', 600.0)
        self.declare_parameter('clock_stall_timeout_s', 30.0)
        self.declare_parameter('min_displacement_m', 0.5)
        self.position = None
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odometry, 10)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def on_odometry(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def wait_for_odometry(self, timeout_s=10.0):
        deadline = time.monotonic() + timeout_s
        while self.position is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.position is None:
            raise RuntimeError('Timed out waiting for /odometry/filtered')

    def run(self):
        self.wait_for_odometry()
        start = self.position
        server_timeout = float(self.get_parameter('server_timeout_s').value)
        if not self.client.wait_for_server(timeout_sec=server_timeout):
            raise RuntimeError('Timed out waiting for /navigate_to_pose')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.get_parameter('goal_x').value)
        goal.pose.pose.position.y = float(self.get_parameter('goal_y').value)
        yaw = float(self.get_parameter('goal_yaw').value)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Sending goal x={goal.pose.pose.position.x:.2f}, '
            f'y={goal.pose.pose.position.y:.2f}, yaw={yaw:.2f}'
        )
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=server_timeout)
        if not send_future.done() or send_future.result() is None:
            raise RuntimeError('Timed out sending navigation goal')
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError('Navigation goal was rejected')

        result_future = goal_handle.get_result_async()
        timeout = float(self.get_parameter('goal_timeout_s').value)
        wall_timeout = float(self.get_parameter('wall_timeout_s').value)
        stall_timeout = float(self.get_parameter('clock_stall_timeout_s').value)
        started = self.get_clock().now().nanoseconds
        wall_started = last_tick = last_report = time.monotonic()
        previous = started
        failure = None
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if result_future.done():
                break
            now = self.get_clock().now().nanoseconds
            wall_now = time.monotonic()
            elapsed = (now - started) * 1e-9
            wall_elapsed = wall_now - wall_started
            if now != previous:
                last_tick = wall_now
            if now < previous:
                failure = 'Navigation clock moved backwards'
            elif elapsed >= timeout:
                failure = f'Navigation goal timed out after {elapsed:.1f} ROS seconds'
            elif wall_elapsed >= wall_timeout:
                failure = f'Navigation exceeded {wall_timeout:.1f} wall seconds'
            elif wall_now - last_tick >= stall_timeout:
                failure = f'Navigation clock stopped for {stall_timeout:.1f} seconds'
            previous = now
            if failure:
                break
            if wall_now - last_report >= 15.0:
                self.get_logger().info(
                    f'Navigating: ROS elapsed={elapsed:.1f}/{timeout:.1f} s, '
                    f'wall elapsed={wall_elapsed:.1f} s, '
                    f'odom=({self.position[0]:.2f}, {self.position[1]:.2f})'
                )
                last_report = wall_now
        if failure or not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            raise RuntimeError(failure or 'Navigation interrupted')

        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Navigation failed with action status {status}')

        displacement = math.hypot(self.position[0] - start[0], self.position[1] - start[1])
        minimum = float(self.get_parameter('min_displacement_m').value)
        if displacement < minimum:
            raise RuntimeError(
                f'Navigation succeeded but displacement {displacement:.2f} m '
                f'is below {minimum:.2f} m'
            )
        self.get_logger().info(
            f'Navigation smoke test complete; displacement={displacement:.2f} m'
        )


def main():
    rclpy.init()
    node = NavigationSmokeTest()
    exit_code = 0
    try:
        node.run()
    except Exception as error:
        node.get_logger().error(str(error))
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
