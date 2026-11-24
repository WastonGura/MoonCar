"""Opt-in Nav2 execution with a browser lease and independent velocity permission.

The relay must be started with require_workbench=true. This module never writes
motor commands. It executes checked stationary turns and straight FollowPath goals.
"""
import math
from pathlib import Path
import secrets
import threading
import time

from nav2_msgs.action import ComputePathToPose, FollowPath, Spin
from nav_msgs.msg import Path as RosPath
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from std_msgs.msg import Bool

from terrain_planner_node import TerrainPlannerNode
from terrain_planner import TerrainPlanner


class ReplanNeeded(RuntimeError):
    pass


def motion_segments(poses):
    """Keep stop/turn boundaries; merge only contiguous straight translations."""
    segments, drive = [], []
    for a, b in zip(poses, poses[1:]):
        pa, pb = a.pose.position, b.pose.position
        yaw_a, yaw_b = TerrainPlannerNode.yaw(a.pose.orientation), TerrainPlannerNode.yaw(b.pose.orientation)
        delta = math.atan2(math.sin(yaw_b - yaw_a), math.cos(yaw_b - yaw_a))
        distance = math.hypot(pb.x - pa.x, pb.y - pa.y)
        if distance < 1e-7:
            if drive:
                segments.append(('drive', drive))
                drive = []
            if abs(delta) > 1e-6:
                segments.append(('turn', b))
        else:
            bearing = math.atan2(pb.y - pa.y, pb.x - pa.x)
            if abs(delta) > 1e-6 or abs(math.atan2(math.sin(bearing - yaw_a), math.cos(bearing - yaw_a))) > 1e-6:
                raise ValueError('Path contains unchecked curved or reversing motion')
            if not drive:
                drive = [a]
            drive.append(b)
    if drive:
        segments.append(('drive', drive))
    return segments


class NavigationGateway(Node):
    def __init__(self, planner_node):
        super().__init__('mooncar_workbench_navigation')
        self.planner_node = planner_node
        self.archive = Path(planner_node.get_parameter('archive').value).resolve()
        self.action_clients = {
            'plan': ActionClient(self, ComputePathToPose, '/mooncar/compute_terrain_path'),
            'drive': ActionClient(self, FollowPath, '/follow_path'),
            'turn': ActionClient(self, Spin, '/spin'),
        }
        self.relay_parameters = AsyncParameterClient(self, '/mooncar_cmd_vel_safety_relay')
        self.permit_pub = self.create_publisher(Bool, '/mooncar/workbench/permit', 1)
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.active, self.fault = False, False
        self.phase, self.error = 'idle', None
        self.token = ''
        self.last_heartbeat = 0.
        self.started = 0.
        self.active_handle, self.pending_send, self.worker = None, None, None
        self.planned, self.track = [], []
        self.runtime_checker = None
        self.watched_motion = []
        self.last_environment_check = 0.
        self.replans = 0
        self.request = None
        self.last_trace = 0.
        self.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def pose(self):
        transform = self.planner_node.buffer.lookup_transform(
            self.planner_node.frame, self.planner_node.robot_frame, rclpy.time.Time())
        age = (self.get_clock().now().nanoseconds - rclpy.time.Time.from_msg(transform.header.stamp).nanoseconds) / 1e9
        if not -.1 <= age <= 2.:
            raise ValueError('Vehicle pose is stale')
        t, q = transform.transform.translation, transform.transform.rotation
        return dict(x=t.x, y=t.y, z=t.z, yaw=TerrainPlannerNode.yaw(q))

    def tick(self):
        with self.lock:
            leased = self.active and time.monotonic() - self.last_heartbeat < 3. and not self.stop_event.is_set()
            if self.active and not leased:
                self.stop_event.set()
            if self.active and time.monotonic() - self.last_trace >= .5:
                try:
                    self.track.append(dict(**self.pose(), elapsed=time.monotonic() - self.started))
                except Exception:
                    self.stop_event.set()
                self.last_trace = time.monotonic()
            if leased and self.phase == 'executing' and self.runtime_checker is not None:
                try:
                    pose = self.pose()
                    if self.runtime_checker.assess(pose['x'], pose['y'], pose['yaw']) is None:
                        self.stop_event.set()
                except Exception:
                    self.stop_event.set()
            self.permit_pub.publish(Bool(data=bool(leased and not self.stop_event.is_set()
                                                  and self.phase == 'executing' and not self.fault)))

    def state(self):
        with self.lock:
            try:
                pose = self.pose()
            except Exception:
                pose = None
            return dict(enabled=True, session=self.archive.name, active=self.active, phase=self.phase,
                        error=self.error, fault=self.fault, pose=pose, path=list(self.planned),
                        track=list(self.track), lease_seconds=3.,
                        replans=self.replans,
                        request=self.request,
                        servers_ready=all(client.server_is_ready() for client in self.action_clients.values()))

    def start(self, session, goal, yaw=0., mode='full'):
        if (session != self.archive.name or len(goal) != 2
                or not all(isinstance(x, (int, float)) and math.isfinite(x) and abs(x) < 10000 for x in (*goal, yaw))
                or mode not in ('fixed', 'direction', 'uncertainty', 'full')):
            raise ValueError('Invalid live navigation request')
        with self.lock:
            if self.active or self.fault:
                raise ValueError('Navigation busy or cancellation unconfirmed; inspect state')
            if not all(client.server_is_ready() for client in self.action_clients.values()):
                raise ValueError('Nav2 action servers are not ready')
            try:
                self.pose()
            except Exception as error:
                raise ValueError(f'Current vehicle pose unavailable: {error}') from error
            self.token = secrets.token_urlsafe(24)
            self.started = self.last_heartbeat = time.monotonic()
            self.active, self.phase, self.error = True, 'planning', None
            self.stop_event.clear()
            self.planned, self.track = [], []
            self.runtime_checker = None
            self.watched_motion, self.replans = [], 0
            self.request = dict(goal=list(goal), yaw=yaw, mode=mode)
            self.worker = threading.Thread(target=self.run_mission, args=(goal, yaw, mode), daemon=True)
            self.worker.start()
            return dict(token=self.token, lease_seconds=3.)

    def heartbeat(self, token):
        with self.lock:
            if not isinstance(token, str) or not secrets.compare_digest(token, self.token):
                raise ValueError('No matching active navigation lease')
            if self.active:
                self.last_heartbeat = time.monotonic()
            return dict(ok=True, active=self.active)

    def cancel(self, token):
        with self.lock:
            if not isinstance(token, str) or not secrets.compare_digest(token, self.token):
                raise ValueError('Navigation lease does not match')
            self.stop_event.set()
            self.permit_pub.publish(Bool(data=False))
        return dict(ok=True)

    def wait(self, future, timeout, cancellable=True):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError('Nav2 response deadline exceeded')
            if cancellable and (self.stop_event.is_set() or time.monotonic() - self.last_heartbeat > 3.
                                or time.monotonic() - self.started > 900.):
                self.stop_event.set()
                raise RuntimeError('Navigation cancelled or browser lease expired')
            if cancellable and self.watched_motion and time.monotonic() - self.last_environment_check >= 1.:
                self.last_environment_check = time.monotonic()
                self.refresh_motion()
            time.sleep(.02)
        return future.result()

    def refresh_motion(self):
        cells, resolution = self.planner_node.elevation()
        checker = TerrainPlanner(cells, resolution, self.planner_node.vehicle)
        self.runtime_checker = checker
        actual = self.pose()
        if checker.assess(actual['x'], actual['y'], actual['yaw']) is None:
            raise RuntimeError('Current vehicle footprint is no longer traversable')
        if any(checker.motion_cost(a, b) is None for a, b in self.watched_motion):
            raise ReplanNeeded('Terrain or obstacle map changed')

    def call_action(self, kind, goal, timeout):
        self.pending_send = self.action_clients[kind].send_goal_async(goal)

        def cancel_late(future):
            if self.stop_event.is_set():
                try:
                    handle = future.result()
                    if handle and handle.accepted:
                        handle.cancel_goal_async()
                except Exception as error:
                    self.get_logger().error(f'Late goal cancellation failed: {error}')

        self.pending_send.add_done_callback(cancel_late)
        handle = self.wait(self.pending_send, 5.)
        self.active_handle = handle if handle.accepted else None
        self.pending_send = None
        if not handle.accepted:
            raise RuntimeError(f'Nav2 rejected {kind}')
        result = self.wait(handle.get_result_async(), timeout)
        self.active_handle = None
        if result.status != 4 or result.result.error_code != 0:
            if self.watched_motion:
                self.refresh_motion()
            raise RuntimeError(f'{kind}: {result.result.error_msg or result.status}')
        return result.result

    def cancel_action(self):
        self.watched_motion = []
        self.permit_pub.publish(Bool(data=False))
        if self.pending_send is not None:
            handle = self.wait(self.pending_send, 5., cancellable=False)
            self.active_handle = handle if handle.accepted else None
            self.pending_send = None
        if self.active_handle is not None:
            self.wait(self.active_handle.cancel_goal_async(), 5., cancellable=False)
            result = self.wait(self.active_handle.get_result_async(), 5., cancellable=False)
            if result.status not in (4, 5, 6):
                raise RuntimeError('Cancellation did not reach a terminal state')
            self.active_handle = None

    def execute_route(self, goal_xy, goal_yaw, mode):
        request = ComputePathToPose.Goal()
        request.planner_id, request.use_start = mode, False
        request.goal.header.frame_id = self.planner_node.frame
        request.goal.pose.position.x, request.goal.pose.position.y = map(float, goal_xy)
        request.goal.pose.orientation.z, request.goal.pose.orientation.w = math.sin(goal_yaw / 2), math.cos(goal_yaw / 2)
        path = self.call_action('plan', request, 30.).path
        with self.lock:
            self.planned = [dict(x=p.pose.position.x, y=p.pose.position.y, z=0.,
                                 yaw=TerrainPlannerNode.yaw(p.pose.orientation)) for p in path.poses]
        for kind, item in motion_segments(path.poses):
            if self.stop_event.is_set():
                raise RuntimeError('Navigation cancelled')
            # Re-read the terrain/costmap before each primitive, not just at first planning.
            cells, resolution = self.planner_node.elevation()
            checker = TerrainPlanner(cells, resolution, self.planner_node.vehicle)
            self.runtime_checker = checker
            actual = self.pose()
            if checker.assess(actual['x'], actual['y'], actual['yaw']) is None:
                raise RuntimeError('Current vehicle footprint is no longer traversable')
            self.phase = 'executing'
            if kind == 'turn':
                yaw = TerrainPlannerNode.yaw(item.pose.orientation)
                if checker.motion_cost((actual['x'], actual['y'], actual['yaw']),
                                       (actual['x'], actual['y'], yaw)) is None:
                    raise ReplanNeeded('Turn is no longer traversable')
                self.watched_motion = [((actual['x'], actual['y'], actual['yaw']),
                                        (actual['x'], actual['y'], yaw))]
                request = Spin.Goal()
                request.target_yaw = math.atan2(math.sin(yaw - actual['yaw']), math.cos(yaw - actual['yaw']))
                request.time_allowance.sec = 30
                self.call_action('turn', request, 120.)
            else:
                for a, b in zip(item, item[1:]):
                    if checker.motion_cost((a.pose.position.x, a.pose.position.y, TerrainPlannerNode.yaw(a.pose.orientation)),
                                           (b.pose.position.x, b.pose.position.y, TerrainPlannerNode.yaw(b.pose.orientation))) is None:
                        raise ReplanNeeded('Segment is no longer traversable')
                self.watched_motion = [((a.pose.position.x, a.pose.position.y, TerrainPlannerNode.yaw(a.pose.orientation)),
                                        (b.pose.position.x, b.pose.position.y, TerrainPlannerNode.yaw(b.pose.orientation)))
                                       for a, b in zip(item, item[1:])]
                request = FollowPath.Goal()
                request.path = RosPath(header=path.header, poses=item)
                request.controller_id, request.goal_checker_id = 'FollowPath', 'goal_checker'
                request.progress_checker_id = 'progress_checker'
                self.call_action('drive', request, 600.)
            self.phase = 'checking'
            self.watched_motion = []

    def run_mission(self, goal_xy, goal_yaw, mode):
        try:
            parameters = self.wait(self.relay_parameters.get_parameters(['require_workbench']), 5.)
            if len(parameters.values) != 1 or not parameters.values[0].bool_value:
                raise RuntimeError('Start simulation with require_workbench:=true before navigation')
            while True:
                try:
                    self.phase = 'planning'
                    self.execute_route(goal_xy, goal_yaw, mode)
                    break
                except ReplanNeeded:
                    self.phase = 'checking'
                    try:
                        self.cancel_action()
                    except Exception:
                        self.fault = True
                        raise
                    # ponytail: three replans per mission; repeated instability needs operator inspection.
                    if self.replans >= 3:
                        raise RuntimeError('Terrain remained unstable after three replans')
                    self.replans += 1
            actual = self.pose()
            if math.hypot(actual['x'] - goal_xy[0], actual['y'] - goal_xy[1]) > .12:
                raise RuntimeError('Nav2 completed but final localization error exceeds 0.12 m')
            self.phase = 'succeeded'
        except Exception as error:
            self.error = str(error)
            self.phase = 'cancelled' if self.stop_event.is_set() else 'failed'
        finally:
            self.stop_event.set()
            self.permit_pub.publish(Bool(data=False))
            try:
                self.cancel_action()
            except Exception as error:
                self.fault = True
                self.phase, self.error = 'fault', f'Cancellation unconfirmed; motion permission withheld: {error}'
            self.active = False

    def close(self):
        self.stop_event.set()
        self.permit_pub.publish(Bool(data=False))
        if self.worker:
            self.worker.join(timeout=12.)
