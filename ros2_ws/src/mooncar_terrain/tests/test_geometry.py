import pathlib
import math
import sys
import time
import unittest
from collections import deque
from unittest.mock import Mock

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
from terrain_geometry import classify_cells, correct_camera_pose, project_depth, tilt_degrees, transform_points  # noqa: E402
from keepout_node import KeepoutNode
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from terrain_node import TerrainNode


class GeometryTest(unittest.TestCase):
    def test_keepout_does_not_clear_unknown_and_respects_map_rotation(self):
        grid = OccupancyGrid()
        grid.info.width, grid.info.height, grid.info.resolution = 2, 1, 1.0
        grid.info.origin.position.x = 3.0
        grid.info.origin.orientation.z = np.sin(np.pi/4)
        grid.info.origin.orientation.w = np.cos(np.pi/4)
        node = Mock(zones=[{'x': 2.5, 'y': 0.5, 'radius': 0.4}])
        KeepoutNode.on_map(node, grid)
        mask = node.publisher.publish.call_args.args[0]
        self.assertEqual(list(mask.data), [100, -1])

    def test_keepout_extends_past_slam_map_boundary(self):
        grid = OccupancyGrid()
        grid.info.width, grid.info.height, grid.info.resolution = 2, 1, 1.0
        grid.info.origin.orientation.w = 1.0
        node = Mock(zones=[{'x': 2.5, 'y': 0.5, 'radius': 0.6}])
        KeepoutNode.on_map(node, grid)
        mask = node.publisher.publish.call_args.args[0]
        self.assertGreaterEqual(mask.info.width, 4)
        ix = math.floor((2.5-mask.info.origin.position.x)/mask.info.resolution)
        iy = math.floor((0.5-mask.info.origin.position.y)/mask.info.resolution)
        self.assertEqual(mask.data[iy*mask.info.width+ix], 100)
        self.assertEqual(mask.data[0], -1)

    def surface(self, z):
        x, y = np.meshgrid(np.linspace(0.01, 0.28, 12), np.linspace(0.01, 0.28, 12))
        return np.column_stack((x.ravel(), y.ravel(), z(x, y).ravel()))

    def test_flat_slope_roughness_step_and_pit(self):
        for surface, bit in [
                (lambda x, y: x*0, 0), (lambda x, y: x*0.6, 1),
                (lambda x, y: 0.09*np.sin(x*150), 2)]:
            reason = classify_cells(self.surface(surface))[0][4]
            self.assertEqual(reason, 0) if bit == 0 else self.assertTrue(reason & bit)
        # Height hazards require measured neighboring ground. An isolated
        # translated plane does not establish a step relative to map zero.
        for height, bit in ((.25, 4), (-.25, 8)):
            ground = self.surface(lambda x, y: x*0)
            raised = ground+[.3, 0., height]
            cell = next(c for c in classify_cells(np.vstack((ground, raised))) if c[0]>.3)
            self.assertTrue(cell[4] & bit)
        self.assertEqual(classify_cells([[0, 0, float('nan')]]), [])

    def test_scanline_is_unknown_not_a_vertical_obstacle(self):
        x = np.linspace(0.01, 0.28, 20)
        for y in [x*0+0.1, 0.1+0.00001*np.sin(x*100)]:
            points = np.column_stack((x, y, 0.002+0.0001*np.cos(x*100)))
            self.assertEqual(classify_cells(points), [])

    def test_depth_padding_endian_units_and_invalid_input(self):
        data = np.array([[1000, 2000, 0], [3000, 0, 0]], dtype='>u2').tobytes()
        points = project_depth(data, 2, 2, 6, '16UC1', True, (1, 1, 0, 0), stride=1)
        np.testing.assert_allclose(points, [[0, 0, 1], [2, 0, 2], [0, 3, 3]])
        with self.assertRaises(ValueError):
            project_depth(b'', 2, 2, 8, '32FC1', False, (1, 1, 0, 0))

    def test_optical_transform(self):
        np.testing.assert_allclose(
            transform_points([[0, 0, 1]], [0.4, 0, 0.3], [-0.5, 0.5, -0.5, 0.5]),
            [[1.4, 0, 0.3]], atol=1e-9)

    def test_imu_tilt_is_yaw_invariant_and_rejects_bad_orientation(self):
        for yaw in (0, 137, -92):
            half_yaw = math.radians(yaw / 2)
            half_pitch = math.radians(2.8 / 2)
            level = (0, 0, math.sin(half_yaw), math.cos(half_yaw))
            tilted = (-math.sin(half_yaw) * math.sin(half_pitch),
                      math.cos(half_yaw) * math.sin(half_pitch),
                      math.sin(half_yaw) * math.cos(half_pitch),
                      math.cos(half_yaw) * math.cos(half_pitch))
            self.assertAlmostEqual(tilt_degrees(level), 0, places=8)
            self.assertAlmostEqual(tilt_degrees(tilted), 2.8, places=8)
        with self.assertRaises(ValueError):
            tilt_degrees((0, 0, 0, 0))

    def test_depth_waits_for_synchronized_imu(self):
        depth = PointCloud2()
        depth.header.stamp.sec = 10
        node = Mock(intrinsics=(1, 1, 0, 0), last_processed=0, frame='odom')
        node.pending = (depth, time.monotonic()-4)
        node.imu = deque()
        TerrainNode.process_pending(node)
        self.assertIsNone(node.pending)
        self.assertIn('Synchronized IMU unavailable', node.status.call_args.args[1])
        node.buffer.lookup_transform.assert_not_called()
        node.observation_pub.publish.assert_not_called()

    def test_imu_attitude_corrects_planar_camera_about_robot(self):
        def yaw_pitch(yaw, pitch):
            a, b = math.radians(yaw/2), math.radians(pitch/2)
            return (-math.sin(a)*math.sin(b), math.cos(a)*math.sin(b),
                    math.sin(a)*math.cos(b), math.cos(a)*math.cos(b))
        robot = yaw_pitch(90, 0)
        imu = yaw_pitch(45, 2.8)  # IMU yaw is relative to the initial 45-degree heading.
        position, orientation = correct_camera_pose((1, 2, 0), robot, (1, 3, 1), robot, imu)
        pitch = math.radians(2.8)
        np.testing.assert_allclose(position, (1, 2+math.cos(pitch)+math.sin(pitch),
                                              math.cos(pitch)-math.sin(pitch)), atol=1e-9)
        np.testing.assert_allclose(transform_points([[1, 0, 0]], (0, 0, 0), orientation),
                                   [[0, math.cos(pitch), -math.sin(pitch)]], atol=1e-9)
        with self.assertRaises(ValueError):
            correct_camera_pose((1, 2, 0), imu, (1, 3, 1), robot, imu)

    def test_explicit_gazebo_pixel_centers_preserve_default_camera_convention(self):
        data = np.array([[1, 2], [3, 4]], dtype='<f4').tobytes()
        args = (data, 2, 2, 8, '32FC1', False, (2, 4, 1, 1))
        nominal = project_depth(*args, stride=1)
        centered = project_depth(*args, stride=1, pixel_offset=0.5)
        np.testing.assert_allclose(nominal, [[-.5, -.25, 1], [0, -.5, 2], [-1.5, 0, 3], [0, 0, 4]])
        np.testing.assert_allclose(centered, [[-.25, -.125, 1], [.5, -.25, 2], [-.75, .375, 3], [1, .5, 4]])
        for value in (float('nan'), float('inf'), 2):
            with self.assertRaises(ValueError): project_depth(*args, pixel_offset=value)


if __name__ == '__main__':
    unittest.main()
