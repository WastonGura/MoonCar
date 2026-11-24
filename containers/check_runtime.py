"""Read-only smoke check for a running MoonCar simulation; issues no motion commands."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=30.)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('mooncar_environment_probe')
    report = dict(scope='simulation startup and sensor smoke check', physical_hardware=False,
                  counts=dict(clock=0, scan=0, depth=0, imu=0, map=0, points=0, safe_command=0),
                  clock_span_seconds=0., finite_scan_returns=0, finite_depth_returns=0,
                  observed_map_cells=0, point_cloud_width=0, nonzero_safe_commands=0,
                  finite_imu=False, point_cloud_topic='/mooncar/terrain/observations')
    first_clock = None

    def clock(msg):
        nonlocal first_clock
        stamp = msg.clock.sec + msg.clock.nanosec / 1e9
        if first_clock is None:
            first_clock = stamp
        report['counts']['clock'] += 1
        report['clock_span_seconds'] = stamp - first_clock

    def scan(msg):
        report['counts']['scan'] += 1
        values = np.asarray(msg.ranges)
        report['finite_scan_returns'] = int((np.isfinite(values) & (values >= msg.range_min)
                                             & (values <= msg.range_max)).sum())

    def depth(msg):
        report['counts']['depth'] += 1
        report['depth_encoding'] = msg.encoding
        if msg.encoding == '32FC1':
            values = np.frombuffer(msg.data, dtype='>f4' if msg.is_bigendian else '<f4')
            report['finite_depth_returns'] = int((np.isfinite(values) & (values > 0)).sum())

    def imu(msg):
        report['counts']['imu'] += 1
        report['finite_imu'] = bool(np.isfinite([
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]).all())

    def grid(msg):
        report['counts']['map'] += 1
        report['observed_map_cells'] = int((np.asarray(msg.data) >= 0).sum())

    def points(msg):
        report['counts']['points'] += 1
        report['point_cloud_width'] = msg.width

    def safe(msg):
        report['counts']['safe_command'] += 1
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        if not np.isfinite(values).all() or any(abs(v) > 1e-6 for v in values):
            report['nonzero_safe_commands'] += 1

    subscriptions = [
        node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/mooncar/scan', scan, qos_profile_sensor_data),
        node.create_subscription(Image, '/mooncar/depth/image', depth, qos_profile_sensor_data),
        node.create_subscription(Imu, '/mooncar/imu', imu, qos_profile_sensor_data),
        node.create_subscription(OccupancyGrid, '/map', grid,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(PointCloud2, report['point_cloud_topic'], points, qos_profile_sensor_data),
        node.create_subscription(Twist, '/mooncar/cmd_vel_safe', safe, 10),
    ]
    def passed():
        return bool(all(report['counts'].values()) and report['clock_span_seconds'] >= 1.
                    and report['finite_scan_returns'] >= 10 and report['finite_depth_returns'] >= 100
                    and report['finite_imu'] and report['observed_map_cells'] > 0
                    and report['point_cloud_width'] >= 100 and report['counts']['safe_command'] >= 3
                    and report['nonzero_safe_commands'] == 0)
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if passed():
                break
        report['passed'] = passed()
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report, indent=2))
        return 0 if report['passed'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
