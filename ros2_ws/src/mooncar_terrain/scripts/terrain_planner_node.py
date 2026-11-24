#!/usr/bin/env python3
"""Nav2 ComputePathToPose action adapter for recorded terrain, opt-in only.

Returns a research path. A motion coordinator must execute its stop/turn
primitives; feeding this path blindly to a smoothing controller is not supported.
"""
from dataclasses import fields, replace
import math
from pathlib import Path
import threading
import time
import zipfile

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path as RosPath
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformException, TransformListener

from terrain_map import TerrainMap
from terrain_planner import MODES, TerrainPlanner, Vehicle


class TerrainPlannerNode(Node):
    def __init__(self):
        super().__init__('mooncar_terrain_planner')
        for name, value in [('archive', ''), ('frame', 'map'), ('robot_frame', 'base_footprint'),
                            ('require_costmap', True), ('search_timeout', 5.), ('tracking_margin', .15)]:
            self.declare_parameter(name, value)
        # Match the existing simulation Nav2 footprint, not the proposed physical chassis.
        vehicle = Vehicle(length=.86, width=.72)
        for field in fields(vehicle):
            self.declare_parameter('vehicle.' + field.name, getattr(vehicle, field.name))
        self.vehicle = Vehicle(**{f.name: self.get_parameter('vehicle.' + f.name).value for f in fields(vehicle)})
        margin = self.get_parameter('tracking_margin').value
        if not math.isfinite(margin) or not 0. <= margin <= .5:
            raise ValueError('Tracking margin must be between zero and 0.5 metre')
        # Reserve clearance for controller/localization error; monitoring uses the actual footprint.
        self.planning_vehicle = replace(self.vehicle, length=self.vehicle.length + 2 * margin,
                                        width=self.vehicle.width + 2 * margin)
        self.frame = self.get_parameter('frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.costmap = None
        self.costmap_received = 0.
        self.lock = threading.Lock()
        self.active = False
        self.cached = None
        self.cached_version = None
        self.cache_lock = threading.Lock()
        self.group = ReentrantCallbackGroup()
        self.path_pub = self.create_publisher(RosPath, '/mooncar/terrain/plan', 1)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                                 callback_group=self.group)
        self.action = ActionServer(self, ComputePathToPose, '/mooncar/compute_terrain_path',
                                   execute_callback=self.execute, goal_callback=self.accept,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.group)

    def on_costmap(self, msg):
        self.costmap, self.costmap_received = msg, time.monotonic()

    def accept(self, request):
        with self.lock:
            if self.active or request.planner_id not in MODES:
                return GoalResponse.REJECT
            self.active = True
        return GoalResponse.ACCEPT

    def elevation(self):
        path = Path(self.get_parameter('archive').value).expanduser()
        with self.cache_lock:
            version = (str(path), path.stat().st_mtime_ns)
            if self.cached_version != version:
                terrain = TerrainMap.load(path)
                if terrain.frame != self.frame:
                    raise ValueError('Archive and planning frame differ')
                self.cached, self.cached_version = terrain.elevation(), (str(path), terrain.archive_updated_ns)
                self.cached_resolution = terrain.resolution
            cells = [dict(cell) for cell in self.cached]
            resolution = self.cached_resolution
        costmap = self.costmap
        if not self.get_parameter('require_costmap').value:
            return cells, resolution
        if costmap is None or time.monotonic() - self.costmap_received > 3.:
            raise ValueError('Global costmap is unavailable or stale')
        if costmap.header.frame_id != self.frame:
            raise ValueError('Global costmap frame differs')
        info = costmap.info
        q = info.origin.orientation
        if abs(q.x) + abs(q.y) + abs(q.z) > 1e-6 or info.resolution <= 0:
            raise ValueError('Expected an axis-aligned global costmap')
        if len(costmap.data) != info.width * info.height:
            raise ValueError('Invalid costmap dimensions')
        for cell in cells:
            r = resolution
            left = math.floor((cell['ix'] * r - info.origin.position.x) / info.resolution)
            bottom = math.floor((cell['iy'] * r - info.origin.position.y) / info.resolution)
            right = math.ceil(((cell['ix'] + 1) * r - info.origin.position.x) / info.resolution)
            top = math.ceil(((cell['iy'] + 1) * r - info.origin.position.y) / info.resolution)
            if (left < 0 or bottom < 0 or right > info.width or top > info.height
                    # 99 is already inflated by Nav2's inscribed vehicle radius.
                    # Our planner sweeps the full rectangle against actual (100) obstacles.
                    or any(costmap.data[y * info.width + x] < 0 or costmap.data[y * info.width + x] >= 100
                           for y in range(bottom, top) for x in range(left, right))):
                cell['valid'] = False
        return cells, resolution

    @staticmethod
    def yaw(q):
        values = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(x) for x in values) or abs(sum(x*x for x in values) - 1.) > .01:
            raise ValueError('Invalid pose quaternion')
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))

    def execute(self, handle):
        begin = time.monotonic()
        result = ComputePathToPose.Result()
        try:
            request = handle.request
            if request.goal.header.frame_id != self.frame:
                raise ValueError('Goal must use the configured map frame')
            if request.use_start:
                if request.start.header.frame_id != self.frame:
                    raise ValueError('Start must use the configured map frame')
                p, q = request.start.pose.position, request.start.pose.orientation
            else:
                transform = self.buffer.lookup_transform(self.frame, self.robot_frame, rclpy.time.Time())
                p, q = transform.transform.translation, transform.transform.rotation
                age = (self.get_clock().now().nanoseconds - rclpy.time.Time.from_msg(transform.header.stamp).nanoseconds) / 1e9
                if not -.1 <= age <= 2.:
                    raise ValueError('Robot pose is stale')
            start = (p.x, p.y, self.yaw(q))
            goal = (request.goal.pose.position.x, request.goal.pose.position.y)
            cells, resolution = self.elevation()
            planner = TerrainPlanner(cells, resolution, self.planning_vehicle)
            plan = planner.plan(start, goal, mode=request.planner_id,
                                timeout=self.get_parameter('search_timeout').value,
                                cancelled=lambda: handle.is_cancel_requested,
                                goal_yaw=self.yaw(request.goal.pose.orientation))
            if handle.is_cancel_requested or plan['status'] == 'cancelled':
                handle.canceled()
                result.error_code, result.error_msg = result.UNKNOWN, 'Planning cancelled'
                return result
            if plan['status'] != 'ok':
                result.error_code = {'invalid_start': result.START_OCCUPIED, 'invalid_goal': result.GOAL_OCCUPIED,
                                     'budget_exceeded': result.TIMEOUT}.get(plan['status'], result.NO_VALID_PATH)
                result.error_msg = plan['status']
                handle.abort()
                return result
            result.path.header.frame_id = self.frame
            result.path.header.stamp = self.get_clock().now().to_msg()
            for point in plan['path']:
                pose = PoseStamped()
                pose.header = result.path.header
                pose.pose.position.x, pose.pose.position.y = point['x'], point['y']
                pose.pose.orientation.z = math.sin(point['yaw'] / 2)
                pose.pose.orientation.w = math.cos(point['yaw'] / 2)
                result.path.poses.append(pose)
            elapsed = time.monotonic() - begin
            result.planning_time.sec, result.planning_time.nanosec = int(elapsed), int(elapsed % 1 * 1e9)
            self.path_pub.publish(result.path)
            handle.succeed()
        except (ValueError, OSError, TransformException, zipfile.BadZipFile) as error:
            result.error_code, result.error_msg = result.UNKNOWN, str(error)
            handle.abort()
        finally:
            with self.lock:
                self.active = False
        return result


def main():
    rclpy.init()
    node = TerrainPlannerNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
