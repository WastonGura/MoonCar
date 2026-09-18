#!/usr/bin/env python3
from collections import deque
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from terrain_geometry import (
    classify_cells,
    compute_traversability_cost,
    correct_camera_pose,
    project_depth,
    tilt_degrees,
    transform_points,
)


class TerrainNode(Node):
    def __init__(self):
        super().__init__('mooncar_terrain')
        for name, value in [('frame', 'odom'), ('resolution', 0.3), ('slope_deg', 18.0),
                            ('roughness', 0.04), ('step_height', 0.18), ('drop_height', 0.12),
                            ('depth_pixel_offset', 0.0), ('traversability_resolution', 0.20),
                            ('slope_mild', 5.0), ('roughness_mild', 0.015),
                            ('costmap_width', 24.0), ('costmap_height', 24.0),
                            ('cell_persistence', 60.0)]:
            self.declare_parameter(name, value)
        self.frame = self.get_parameter('frame').value
        self.intrinsics = None
        self.camera_frame = None
        self.camera_size = None
        self.last_processed = 0.0
        self.last_valid = 0.0
        self.pending = None
        self.imu = deque(maxlen=400)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.cloud_pub = self.create_publisher(PointCloud2, '/mooncar/terrain/obstacles', 10)
        self.observation_pub = self.create_publisher(
            PointCloud2, '/mooncar/terrain/observations', qos_profile_sensor_data)
        grid_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.traversability_pub = self.create_publisher(
            OccupancyGrid, '/mooncar/terrain/traversability_map', grid_qos)
        self.traversability_cells = {}
        self.diag_pub = self.create_publisher(DiagnosticArray, '/mooncar/terrain/status', 10)
        self.create_subscription(CameraInfo, '/mooncar/depth/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/mooncar/depth/image', self.on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, '/mooncar/imu', self.on_imu,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self.check_stale)
        self.create_timer(0.1, self.process_pending)

    def on_info(self, msg):
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.camera_frame = msg.header.frame_id
        self.camera_size = (msg.width, msg.height)

    def status(self, level, message, values=()):
        status = DiagnosticStatus(level=level, name='terrain_geometry',
                                  hardware_id='depth_camera', message=message)
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values]
        self.diag_pub.publish(DiagnosticArray(
            header=Header(stamp=self.get_clock().now().to_msg()), status=[status]))

    def check_stale(self):
        if time.monotonic() - self.last_valid > 5.0:
            self.status(DiagnosticStatus.STALE, 'No recent valid depth/TF; terrain unknown')

    def on_depth(self, msg):
        if self.pending is None:
            self.pending = (msg, time.monotonic())

    def on_imu(self, msg):
        if msg.header.frame_id != 'imu_link':
            self.status(DiagnosticStatus.WARN, 'Unexpected IMU frame; depth pose quality unknown')
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        q = msg.orientation
        self.imu.append((stamp, (q.x, q.y, q.z, q.w)))

    def process_pending(self):
        if self.pending is None:
            return
        msg, received = self.pending
        if self.intrinsics is None or time.monotonic() - self.last_processed < 0.5:
            return
        # Wait for the timestamped EKF transform without blocking its subscription.
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        imu = min(self.imu, key=lambda row: abs(row[0] - stamp.nanoseconds / 1e9), default=None)
        if imu is None or abs(imu[0] - stamp.nanoseconds / 1e9) > .03:
            if time.monotonic() - received > 3.0:
                self.pending = None
                self.status(DiagnosticStatus.WARN, 'Synchronized IMU unavailable; depth omitted')
            return
        if (not self.buffer.can_transform(self.frame, msg.header.frame_id, stamp)
                or not self.buffer.can_transform(self.frame, 'base_footprint', stamp)):
            if time.monotonic() - received > 3.0:
                self.pending = None
                self.status(DiagnosticStatus.WARN, 'Depth transform unavailable within 3 seconds')
            return
        self.pending = None
        self.last_processed = time.monotonic()
        try:
            if msg.header.frame_id != self.camera_frame:
                raise ValueError('Depth and CameraInfo frames differ')
            if (msg.width, msg.height) != self.camera_size:
                raise ValueError('Depth and CameraInfo dimensions differ')
            points = project_depth(msg.data, msg.width, msg.height, msg.step,
                                   msg.encoding, msg.is_bigendian, self.intrinsics,
                                   pixel_offset=self.get_parameter('depth_pixel_offset').value)
            if len(points) < 100:
                raise ValueError('Too few depth returns; terrain unknown')
            sensor_points = points
            transform = self.buffer.lookup_transform(
                self.frame, msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp))
            robot_transform = self.buffer.lookup_transform(
                self.frame, 'base_footprint', rclpy.time.Time.from_msg(msg.header.stamp))
            t, q = transform.transform.translation, transform.transform.rotation
            rt, rq = robot_transform.transform.translation, robot_transform.transform.rotation
            corrected_t, corrected_q = correct_camera_pose(
                (rt.x, rt.y, rt.z), (rq.x, rq.y, rq.z, rq.w),
                (t.x, t.y, t.z), (q.x, q.y, q.z, q.w), imu[1])
            points = transform_points(points, corrected_t, corrected_q)
            cells = classify_cells(points, **{
                name: self.get_parameter(name).value for name in
                ('resolution', 'slope_deg', 'roughness', 'step_height', 'drop_height')})
            hazards = [(x, y, 0.2) for x, y, _, _, reason in cells if reason]
            cloud = point_cloud2.create_cloud_xyz32(
                Header(stamp=msg.header.stamp, frame_id=self.frame),
                np.asarray(hazards, dtype=np.float32).reshape(-1, 3))
            self.cloud_pub.publish(cloud)

            # Update 2.5D continuous traversability grid
            if getattr(self, 'traversability_cells', None) is None:
                self.traversability_cells = {}
            now = time.monotonic()
            slope_mild = float(self.get_parameter('slope_mild').value) if hasattr(self, 'get_parameter') else 5.0
            slope_max = float(self.get_parameter('slope_deg').value) if hasattr(self, 'get_parameter') else 18.0
            roughness_mild = float(self.get_parameter('roughness_mild').value) if hasattr(self, 'get_parameter') else 0.015
            roughness_max = float(self.get_parameter('roughness').value) if hasattr(self, 'get_parameter') else 0.040
            t_res = float(self.get_parameter('traversability_resolution').value) if hasattr(self, 'get_parameter') else 0.20

            for row in cells:
                cx, cy, slope, residual, reason = row[:5]
                cost = compute_traversability_cost(
                    slope, residual, reason,
                    slope_mild=slope_mild, slope_max=slope_max,
                    roughness_mild=roughness_mild, roughness_max=roughness_max
                )
                k = (int(math.floor(cx / t_res)), int(math.floor(cy / t_res)))
                old_cost, old_stamp = self.traversability_cells.get(k, (0, 0.0))
                new_cost = max(cost, old_cost) if now - old_stamp < 5.0 else cost
                self.traversability_cells[k] = (new_cost, now)

            self.publish_traversability_map(rt.x, rt.y)

            # Preserve measured XYZ in the optical frame; never archive flattened hazards.
            self.observation_pub.publish(point_cloud2.create_cloud_xyz32(
                msg.header, sensor_points.astype(np.float32)))
            self.last_valid = time.monotonic()
            self.status(DiagnosticStatus.OK, 'Geometry observed with synchronized IMU attitude',
                        [('points', len(points)), ('cells', len(cells)),
                         ('hazard_cells', len(hazards)),
                         ('traversability_cells', len(self.traversability_cells)),
                         ('tilt_deg', round(tilt_degrees(imu[1]), 3))])
        except (ValueError, TransformException) as error:
            if time.monotonic() - self.last_valid > 2.0:
                self.status(DiagnosticStatus.WARN, str(error))

    def publish_traversability_map(self, robot_x=0.0, robot_y=0.0):
        if not hasattr(self, 'traversability_pub') or not hasattr(self, 'traversability_cells'):
            return
        now = time.monotonic()
        persistence = float(self.get_parameter('cell_persistence').value) if hasattr(self, 'get_parameter') else 60.0
        r = float(self.get_parameter('traversability_resolution').value) if hasattr(self, 'get_parameter') else 0.20
        w_m = float(self.get_parameter('costmap_width').value) if hasattr(self, 'get_parameter') else 24.0
        h_m = float(self.get_parameter('costmap_height').value) if hasattr(self, 'get_parameter') else 24.0
        width = int(round(w_m / r))
        height = int(round(h_m / r))

        ox = math.floor((robot_x - w_m / 2.0) / r) * r
        oy = math.floor((robot_y - h_m / 2.0) / r) * r
        origin_ix = int(round(ox / r))
        origin_iy = int(round(oy / r))

        grid = np.full((height, width), -1, dtype=np.int8)

        expired = [k for k, (_, stamp) in self.traversability_cells.items() if now - stamp > persistence]
        for k in expired:
            del self.traversability_cells[k]

        for (k_ix, k_iy), (cost, stamp) in self.traversability_cells.items():
            col = k_ix - origin_ix
            row = k_iy - origin_iy
            if 0 <= col < width and 0 <= row < height:
                grid[row, col] = int(cost)

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.info.resolution = r
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = float(ox)
        msg.info.origin.position.y = float(oy)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()
        self.traversability_pub.publish(msg)



def main():
    rclpy.init()
    node = TerrainNode()
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
