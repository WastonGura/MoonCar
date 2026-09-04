#!/usr/bin/env python3
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from terrain_geometry import classify_cells, project_depth, transform_points


class TerrainNode(Node):
    def __init__(self):
        super().__init__('mooncar_terrain')
        for name, value in [('frame', 'odom'), ('resolution', 0.3), ('slope_deg', 18.0),
                            ('roughness', 0.04), ('step_height', 0.18), ('drop_height', 0.12)]:
            self.declare_parameter(name, value)
        self.frame = self.get_parameter('frame').value
        self.intrinsics = None
        self.camera_frame = None
        self.camera_size = None
        self.last_processed = 0.0
        self.last_valid = 0.0
        self.pending = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.cloud_pub = self.create_publisher(PointCloud2, '/mooncar/terrain/obstacles', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/mooncar/terrain/status', 10)
        self.create_subscription(CameraInfo, '/mooncar/depth/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/mooncar/depth/image', self.on_depth,
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

    def process_pending(self):
        if self.pending is None:
            return
        msg, received = self.pending
        if self.intrinsics is None or time.monotonic() - self.last_processed < 0.5:
            return
        # Wait for the timestamped EKF transform without blocking its subscription.
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if not self.buffer.can_transform(self.frame, msg.header.frame_id, stamp):
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
                                   msg.encoding, msg.is_bigendian, self.intrinsics)
            if len(points) < 100:
                raise ValueError('Too few depth returns; terrain unknown')
            transform = self.buffer.lookup_transform(
                self.frame, msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp))
            t, q = transform.transform.translation, transform.transform.rotation
            points = transform_points(points, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
            cells = classify_cells(points, **{
                name: self.get_parameter(name).value for name in
                ('resolution', 'slope_deg', 'roughness', 'step_height', 'drop_height')})
            hazards = [(x, y, 0.2) for x, y, _, _, reason in cells if reason]
            cloud = point_cloud2.create_cloud_xyz32(
                Header(stamp=msg.header.stamp, frame_id=self.frame),
                np.asarray(hazards, dtype=np.float32).reshape(-1, 3))
            self.cloud_pub.publish(cloud)
            self.last_valid = time.monotonic()
            self.status(DiagnosticStatus.OK, 'Geometry observed; unobserved space remains unknown',
                        [('points', len(points)), ('cells', len(cells)),
                         ('hazard_cells', len(hazards))])
        except (ValueError, TransformException) as error:
            if time.monotonic() - self.last_valid > 2.0:
                self.status(DiagnosticStatus.WARN, str(error))


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
