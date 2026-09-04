#!/usr/bin/env python3
"""M6 integration acceptance: prior zones, new obstacle, and two real goals."""
import json
import math
import os
from pathlib import Path
import subprocess
import time
import uuid

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy

from navigation_smoke_test import NavigationSmokeTest


class MissionTest(NavigationSmokeTest):
    def __init__(self):
        super().__init__()
        self.terrain = None
        self.costmap = None
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.create_subscription(DiagnosticArray, '/mooncar/terrain/status', self.on_status, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.on_costmap, QoSProfile(depth=1,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.evidence = {}

    def on_status(self, msg):
        for status in msg.status:
            if status.name == 'terrain_geometry' and status.level == DiagnosticStatus.OK:
                self.terrain = {v.key: v.value for v in status.values}

    def on_costmap(self, msg):
        self.costmap = msg

    def wait(self, predicate, message, timeout=45):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not predicate():
            raise RuntimeError(message)

    def plan(self, start, end):
        if not self.planner.wait_for_server(timeout_sec=30):
            raise RuntimeError('Planner unavailable')
        goal = ComputePathToPose.Goal()
        goal.use_start = True
        goal.planner_id = 'GridBased'
        for field, xy in [('start', start), ('goal', end)]:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = xy
            pose.pose.orientation.w = 1.0
            setattr(goal, field, pose)
        future = self.planner.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30)
        if not future.done() or not future.result().accepted:
            raise RuntimeError('Planning request rejected or timed out')
        result = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=30)
        if not result.done() or result.result().status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Planning failed')
        points = [(p.pose.position.x, p.pose.position.y) for p in result.result().result.path.poses]
        if not points:
            raise RuntimeError('Planner returned empty path')
        return points

    def occupied(self, x, y):
        if self.costmap is None:
            return False
        info = self.costmap.info
        col, row = int((x-info.origin.position.x)/info.resolution), int((y-info.origin.position.y)/info.resolution)
        return (0 <= col < info.width and 0 <= row < info.height and
                self.costmap.data[row*info.width+col] >= 99)

    def obstacle_surface_observed(self, x, y):
        # A laser sees the near surface, not the unobserved box interior.
        return any(self.occupied(x+dx, y+dy) for dx in (-0.4, -0.3, 0, 0.3, 0.4)
                   for dy in (-0.4, -0.3, 0, 0.3, 0.4))

    def run_mission(self):
        self.wait(lambda: self.terrain is not None, 'No valid depth terrain observation')
        self.wait(lambda: self.occupied(-1.5, -0.8), 'Known soil zone absent from global costmap')
        self.evidence['live_depth'] = self.terrain
        path = self.plan((0.5, -0.8), (-3.0, -0.8))
        clearance = min(math.hypot(x+1.5, y+0.8) for x, y in path)
        if clearance < 0.65:
            raise RuntimeError('Planned path crossed keepout zone')
        self.evidence['keepout_center_clearance_m'] = clearance
        self.get_logger().info(f'Keepout detour passed; center clearance={clearance:.2f} m')

        start, end, center = (0.5, 2.5), (3.5, 2.5), (1.5, 2.5)
        before = self.plan(start, end)
        baseline = min(math.hypot(x-center[0], y-center[1]) for x, y in before)
        if baseline > 0.4:
            raise RuntimeError('Dynamic-obstacle baseline is already obstructed')
        name = 'm6_obstacle_' + uuid.uuid4().hex[:8]
        sdf = (f'<sdf version="1.9"><model name="{name}"><static>true</static>'
               '<pose>1.5 2.5 0.4 0 0 0</pose><link name="box">'
               '<collision name="c"><geometry><box><size>0.8 0.8 0.8</size></box></geometry></collision>'
               '<visual name="v"><geometry><box><size>0.8 0.8 0.8</size></box></geometry></visual>'
               '</link></model></sdf>')
        try:
            subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'lunar_yard',
                            '-string', sdf], check=True, timeout=30)
            self.wait(lambda: self.obstacle_surface_observed(*center), 'New obstacle absent from costmap', 60)
            after = self.plan(start, end)
            clearance = min(math.hypot(x-center[0], y-center[1]) for x, y in after)
            if clearance < 0.5:
                raise RuntimeError('Replanned path did not avoid the new obstacle')
            self.evidence['new_obstacle'] = {'baseline_center_distance_m': baseline,
                                           'replanned_center_distance_m': clearance}
            self.get_logger().info(f'New obstacle replanning passed; clearance={clearance:.2f} m')
        finally:
            result = subprocess.run(['gz', 'service', '-s', '/world/lunar_yard/remove',
                '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000', '--req', f'name: "{name}" type: MODEL'],
                capture_output=True, text=True, timeout=10)
            if result.returncode or 'true' not in result.stdout:
                raise RuntimeError('Failed to remove acceptance obstacle: ' + result.stdout + result.stderr)

        self.evidence['goals'] = []
        for x, y, minimum in [(1.5, 0.0, 0.5), (3.5, 3.0, 2.0)]:
            self.set_parameters([Parameter('goal_x', value=x), Parameter('goal_y', value=y),
                                 Parameter('min_displacement_m', value=minimum)])
            self.run()
            self.evidence['goals'].append({'x': x, 'y': y, 'status': 'SUCCEEDED',
                                           'final_odom': self.position})
        output = Path(os.environ.get('MOONCAR_M6_REPORT', str(Path.home()/'mooncar_m6_acceptance.json')))
        output.write_text(json.dumps(self.evidence, indent=2) + '\n')
        self.get_logger().info(f'M6 acceptance passed. Evidence: {output}')


def main():
    output = Path(os.environ.get('MOONCAR_M6_REPORT', str(Path.home()/'mooncar_m6_acceptance.json')))
    output.unlink(missing_ok=True)
    rclpy.init()
    node = MissionTest()
    try:
        node.run_mission()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
