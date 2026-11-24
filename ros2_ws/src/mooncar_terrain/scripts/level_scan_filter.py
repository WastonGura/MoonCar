#!/usr/bin/env python3
"""Keep tilted 2D lidar planes out of planar SLAM; Nav2 keeps the raw scan."""
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

from terrain_geometry import tilt_degrees


MAX_TILT_DEG = 1.5
MAX_STAMP_GAP = 0.03


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class LevelScanFilter(Node):
    def __init__(self):
        super().__init__('mooncar_level_scan_filter')
        self.imu = deque(maxlen=50)
        self.pending = deque(maxlen=10)
        self.published = 0
        self.omitted = 0
        self.publisher = self.create_publisher(LaserScan, '/mooncar/scan_level', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/mooncar/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/mooncar/imu', self.on_imu, qos_profile_sensor_data)
        self.create_timer(5.0, self.report)

    def on_scan(self, msg):
        if len(self.pending) == self.pending.maxlen:
            self.omitted += 1
        self.pending.append(msg)
        self.process_pending()

    def on_imu(self, msg):
        if msg.header.frame_id != 'imu_link':
            return
        q = msg.orientation
        try:
            tilt = tilt_degrees((q.x, q.y, q.z, q.w))
        except ValueError:
            return
        self.imu.append((stamp_seconds(msg.header.stamp), tilt))
        self.process_pending()

    def process_pending(self):
        if not self.imu:
            return
        latest = self.imu[-1][0]
        while self.pending:
            scan = self.pending[0]
            stamp = stamp_seconds(scan.header.stamp)
            nearest = min(self.imu, key=lambda row: abs(row[0] - stamp))
            if abs(nearest[0] - stamp) <= MAX_STAMP_GAP:
                self.pending.popleft()
                if nearest[1] <= MAX_TILT_DEG:
                    self.publisher.publish(scan)
                    self.published += 1
                else:
                    self.omitted += 1
            elif latest > stamp + MAX_STAMP_GAP:
                self.pending.popleft()
                self.omitted += 1
            else:
                break

    def report(self):
        self.get_logger().info(f'Planar SLAM scans: published={self.published}, omitted={self.omitted}')


def main():
    rclpy.init()
    node = LevelScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
