"""Real in-process ROS action transport, with a generated map (not Gazebo)."""
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from terrain_map import TerrainMap
from terrain_planner_node import TerrainPlannerNode
from terrain_planner import TerrainPlanner
from navigation_gateway import NavigationGateway


class PlannerActionTests(unittest.TestCase):
    def test_action_result_and_missing_costmap_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            x, y = np.meshgrid(np.arange(.025, 4., .05), np.arange(.025, 3., .05))
            terrain = TerrainMap(source='ROS action test generated planar samples')
            terrain.add_frame(np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size))), 1.)
            archive = Path(directory) / 'flat.npz'
            terrain.save(archive)
            rclpy.init()
            server = TerrainPlannerNode()
            server.set_parameters([Parameter('archive', value=str(archive)), Parameter('require_costmap', value=False)])
            gateway = NavigationGateway(server)
            client_node = Node('terrain_action_test_client')
            client = ActionClient(client_node, ComputePathToPose, '/mooncar/compute_terrain_path')
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(server)
            executor.add_node(client_node)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()

            def wait(future):
                limit = time.monotonic() + 10.
                while not future.done() and time.monotonic() < limit:
                    time.sleep(.02)
                self.assertTrue(future.done(), 'ROS action transport timed out')
                return future.result()

            try:
                self.assertTrue(client.wait_for_server(timeout_sec=5.))
                self.assertEqual(set(gateway.action_clients), {'plan', 'drive', 'turn'})
                goal = ComputePathToPose.Goal()
                goal.planner_id, goal.use_start = 'full', True
                goal.start.header.frame_id = goal.goal.header.frame_id = 'map'
                goal.start.pose.position.x, goal.start.pose.position.y = 1.125, 1.375
                goal.goal.pose.position.x, goal.goal.pose.position.y = 2.875, 1.375
                goal.start.pose.orientation.w = goal.goal.pose.orientation.w = 1.
                handle = wait(client.send_goal_async(goal))
                self.assertTrue(handle.accepted)
                result = wait(handle.get_result_async())
                self.assertEqual(result.status, 4)
                self.assertEqual(result.result.error_code, 0)
                self.assertGreater(len(result.result.path.poses), 2)
                self.assertEqual(result.result.path.header.frame_id, 'map')
                self.assertAlmostEqual(server.planning_vehicle.width - server.vehicle.width, .3)
                # A real controller need not follow the exact centerline to remain traversable.
                monitor = TerrainPlanner(terrain.elevation(), terrain.resolution, server.vehicle)
                for pose in result.result.path.poses:
                    for dx, dy, yaw in [(.1, .1, .05), (-.1, -.1, -.05)]:
                        self.assertIsNotNone(monitor.assess(pose.pose.position.x + dx,
                                                            pose.pose.position.y + dy, yaw))
                server.set_parameters([Parameter('require_costmap', value=True)])
                handle = wait(client.send_goal_async(goal))
                result = wait(handle.get_result_async())
                self.assertEqual(result.status, 6)
                self.assertIn('costmap', result.result.error_msg)
                grid = OccupancyGrid()
                grid.header.frame_id = 'map'
                grid.info.width, grid.info.height, grid.info.resolution = 16, 12, .25
                grid.info.origin.orientation.w = 1.
                grid.data = [0] * (16 * 12)
                for value, valid in [(99, True), (100, False), (-1, False)]:
                    grid.data[5 * 16 + 5] = value
                    server.on_costmap(grid)
                    overlaid, _ = server.elevation()
                    cell = next(c for c in overlaid if (c['ix'], c['iy']) == (5, 5))
                    self.assertEqual(cell['valid'], valid)
            finally:
                executor.shutdown()
                thread.join(timeout=3.)
                client.destroy()
                client_node.destroy_node()
                gateway.destroy_node()
                server.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
