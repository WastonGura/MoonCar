#!/usr/bin/env python3
"""Record timestamped depth observations separately from M6 safety decisions."""
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_msgs.msg import String
import json
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from terrain_map import TerrainMap
from terrain_geometry import correct_camera_pose
from survey_uplink import SurveyUplink
try:
    from localization_quality import LocalizationQuality
except ModuleNotFoundError:
    sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
    from localization_quality import LocalizationQuality


def keyframe_needed(frames, translation, quaternion, distance, angle):
    if len(frames) < 3:
        return True
    previous_t, previous_q = frames[-1][2:]
    q = np.asarray(quaternion, dtype=float)
    cosine = abs(np.dot(previous_q, q) / (np.linalg.norm(previous_q) * np.linalg.norm(q)))
    rotation = np.degrees(2 * np.arccos(np.clip(cosine, 0., 1.)))
    return np.linalg.norm(np.asarray(translation)-previous_t) >= distance or rotation >= angle


class MapRecorder(Node):
    def __init__(self):
        super().__init__('mooncar_map_recorder')
        for name, value in [('frame', 'map'), ('resolution', .25), ('voxel_size', .05),
                            ('output_directory', str(Path.home() / '.local/share/mooncar/maps')),
                            ('source', 'ROS depth observations; physical/simulation provenance unspecified'),
                            ('save_interval', 5.), ('keyframe_distance', .05), ('keyframe_angle', 5.),
                            ('require_laser_localization', False),
                            ('uplink_url', ''), ('uplink_token_file', ''), ('uplink_task_file', '')]:
            self.declare_parameter(name, value)
        self.localization_quality=LocalizationQuality(required=self.get_parameter('require_laser_localization').value)
        self.directory = Path(self.get_parameter('output_directory').value).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.terrain = TerrainMap(self.get_parameter('resolution').value,
                                  self.get_parameter('voxel_size').value,
                                  self.get_parameter('frame').value,
                                  self.get_parameter('source').value, spool_directory=self.directory)
        self.keyframe_distance = self.get_parameter('keyframe_distance').value
        self.keyframe_angle = self.get_parameter('keyframe_angle').value
        if not all(np.isfinite(v) and v > 0 for v in (self.keyframe_distance, self.keyframe_angle)):
            raise ValueError('Keyframe thresholds must be finite and positive')
        self.path = self.directory / f'survey-{datetime.now(timezone.utc):%Y%m%dT%H%M%S%fZ}.npz'
        self.uplink = None
        uplink = [self.get_parameter(name).value for name in ('uplink_url', 'uplink_token_file', 'uplink_task_file')]
        if any(uplink):
            if not all(uplink): raise ValueError('All three uplink parameters are required')
            task = json.loads(Path(uplink[2]).expanduser().read_text(encoding='utf-8'))
            expected_clock = 'simulation' if self.get_parameter('use_sim_time').value else 'utc'
            if (task['coordinates']['frame'] != self.terrain.frame or task['grid']['cell_m'] != self.terrain.resolution
                    or task['grid']['voxel_m'] != self.terrain.voxel_size or task['time']['clock'] != expected_clock
                    or task['source']['description'] != self.terrain.source):
                raise ValueError('Uplink task must match recorder frame/grid/clock/source description')
            self.uplink = SurveyUplink(uplink[0], Path(uplink[1]).expanduser(), task,
                                       self.directory/(self.path.stem+'.uplink'))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending = None
        self.imu = deque(maxlen=2000)
        self.dirty = False
        self.last_observation = 0.
        self.last_stamp = None
        self.reset_detected = False
        self.cloud_pub = self.create_publisher(PointCloud2, '/mooncar/map/points', 1)
        self.status_pub = self.create_publisher(DiagnosticArray, '/mooncar/map/status', 1)
        self.quality_pub = self.create_publisher(String, '/mooncar/map/quality', 1)
        if self.get_parameter('require_laser_localization').value:
            self.create_subscription(String, '/mooncar/laser_odometry/status', self.on_localization_quality, 1)
        self.create_timer(1.,lambda:self.quality_pub.publish(String(data=json.dumps(self.terrain.quality(with_surface=True)))))
        self.create_subscription(PointCloud2, '/mooncar/terrain/observations', self.on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, '/mooncar/imu', self.on_imu,
                                 qos_profile_sensor_data)
        self.create_timer(.1, self.process_pending)
        interval = self.get_parameter('save_interval').value
        if interval < 1. or not np.isfinite(interval):
            raise ValueError('save_interval must be finite and >= 1 second')
        self.create_timer(interval, self.save_periodic)
        self.create_service(Trigger, '/mooncar/map/save', self.save_service)
        self.create_service(Trigger, '/mooncar/map/export_dem', self.export_dem_service)
        self.create_service(Trigger, '/mooncar/map/export_mesh', self.export_mesh_service)
        self.get_logger().info(f'Observation archive: {self.path}')

    def status(self, level, message):
        status = DiagnosticStatus(name='map_recorder', hardware_id='depth_camera',
                                  level=level, message=message,
                                  values=[KeyValue(key='archive', value=str(self.path)),
                                          KeyValue(key='frames', value=str(len(self.terrain.frames)))])
        self.status_pub.publish(DiagnosticArray(
            header=Header(stamp=self.get_clock().now().to_msg()), status=[status]))

    def on_cloud(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.last_stamp is not None and stamp < self.last_stamp:
            # Do not mix coordinate epochs after a simulation reset or bag rewind.
            self.reset_detected = True
            self.pending = None
            self.status(DiagnosticStatus.ERROR, 'Clock moved backwards; save and restart recorder for a new session')
            return
        self.last_stamp = stamp
        if not self.reset_detected and self.pending is None:
            self.pending = (msg, time.monotonic())

    def on_localization_quality(self, msg):
        try:value=json.loads(msg.data)
        except (ValueError,TypeError):value={}
        self.localization_quality.observe(value,time.monotonic())
        health=self.localization_quality.check(self.get_clock().now().nanoseconds/1e9,time.monotonic())
        if not health['healthy']:
            self.pending=None
            self.status(DiagnosticStatus.WARN,'Laser localization unavailable; new map frames suspended: '+health['reason'])

    def on_imu(self, msg):
        if msg.header.frame_id != 'imu_link':
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        q = msg.orientation
        self.imu.append((stamp, (q.x, q.y, q.z, q.w)))

    def process_pending(self):
        if self.pending is None:
            return
        msg, received = self.pending
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        health=self.localization_quality.check(self.get_clock().now().nanoseconds/1e9,time.monotonic(),
                                               observation_stamp=stamp.nanoseconds/1e9)
        if not health['healthy']:
            self.pending=None
            self.status(DiagnosticStatus.WARN,'Laser localization unavailable; depth frame omitted: '+health['reason'])
            return
        imu = min(self.imu, key=lambda row: abs(row[0] - stamp.nanoseconds / 1e9), default=None)
        if imu is None or abs(imu[0] - stamp.nanoseconds / 1e9) > .03:
            if time.monotonic() - received > 3.:
                self.pending = None
                self.status(DiagnosticStatus.WARN, 'Synchronized IMU unavailable; frame omitted')
            return
        if (not self.buffer.can_transform(self.terrain.frame, msg.header.frame_id, stamp)
                or not self.buffer.can_transform(self.terrain.frame, 'base_footprint', stamp)):
            if time.monotonic() - received > 3.:
                self.pending = None
                self.status(DiagnosticStatus.WARN, 'Timestamped pose unavailable; frame omitted')
            return
        self.pending = None
        try:
            points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            transform = self.buffer.lookup_transform(self.terrain.frame, msg.header.frame_id, stamp)
            robot_transform = self.buffer.lookup_transform(self.terrain.frame, 'base_footprint', stamp)
            t, q = transform.transform.translation, transform.transform.rotation
            rt, rq = robot_transform.transform.translation, robot_transform.transform.rotation
            corrected_t, corrected_q = correct_camera_pose(
                (rt.x, rt.y, rt.z), (rq.x, rq.y, rq.z, rq.w),
                (t.x, t.y, t.z), (q.x, q.y, q.z, q.w), imu[1])
            self.last_observation = time.monotonic()
            if not keyframe_needed(self.terrain.frames, corrected_t, corrected_q,
                                   self.keyframe_distance, self.keyframe_angle):
                self.status(DiagnosticStatus.OK, 'Depth active; pose unchanged, redundant frame omitted')
                return
            self.terrain.add_frame(points, stamp.nanoseconds / 1e9, corrected_t, corrected_q,
                                   (rt.x, rt.y, rt.z), (rq.x, rq.y, rq.z, rq.w))
            self.dirty = True
            if self.uplink is not None:
                try:
                    fs, fp, ft, fq = self.terrain.frames[-1]
                    rp = self.terrain.robot_poses[-1]
                    self.uplink.enqueue(fp, fs, ft, fq, rp[:3] if rp is not None else None,
                                        rp[3:] if rp is not None else None)
                except (ValueError, OSError) as error:
                    self.get_logger().error(f'Live uplink queue failed; local archive retained: {error}')
            self.last_observation = time.monotonic()
            self.status(DiagnosticStatus.OK, 'Depth frame recorded; historical pose errors are not corrected by loop closure')
        except OSError as error:
            self.status(DiagnosticStatus.ERROR, f'Observation segment write failed: {error}')
        except (ValueError, TransformException) as error:
            self.status(DiagnosticStatus.WARN, str(error))

    def save_periodic(self):
        if self.dirty:
            try:
                self.save()
            except OSError as error:
                self.status(DiagnosticStatus.ERROR, f'Archive save failed: {error}')
        if self.reset_detected:
            self.status(DiagnosticStatus.ERROR, 'Clock reset; restart recorder with a new session')
        elif time.monotonic() - self.last_observation > 5.:
            self.status(DiagnosticStatus.STALE, 'No recent observations; saved map is historical')
        elif self.uplink is not None and self.uplink.status()['error']:
            self.status(DiagnosticStatus.WARN, 'Live uplink disconnected/rejected; queued frames retained locally')

    def save(self):
        self.terrain.save(self.path)
        try:
            self.terrain.export_dem(self.path.with_suffix('.asc'))
            self.terrain.export_mesh(self.path.with_suffix('.obj'))
            self.terrain.export_ply(self.path.with_suffix('.ply'))
        except Exception as error:
            self.get_logger().warn(f'Secondary format export warning: {error}')
        self.dirty = False
        stamp = self.get_clock().now().to_msg()
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(
            Header(stamp=stamp, frame_id=self.terrain.frame), self.terrain.cloud()))

    def save_service(self, request, response):
        try:
            self.save()
            response.success, response.message = True, str(self.path)
        except OSError as error:
            response.success, response.message = False, str(error)
        return response

    def export_dem_service(self, request, response):
        try:
            meta = self.terrain.export_dem(self.path.with_suffix('.asc'))
            response.success, response.message = True, json.dumps(meta)
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def export_mesh_service(self, request, response):
        try:
            v_count, f_count = self.terrain.export_mesh(self.path.with_suffix('.obj'))
            response.success, response.message = True, f"Vertices: {v_count}, Faces: {f_count}"
        except Exception as error:
            response.success, response.message = False, str(error)
        return response


def main():
    rclpy.init()
    node = MapRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.uplink is not None: node.uplink.close()
        if node.dirty:
            try:
                node.terrain.save(node.path)
            except OSError as error:
                node.get_logger().error(f'Final map save failed: {error}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
