import pathlib
import sys
import unittest
from unittest.mock import Mock

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
from terrain_geometry import classify_cells, project_depth, transform_points  # noqa: E402
from keepout_node import KeepoutNode
from nav_msgs.msg import OccupancyGrid


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

    def surface(self, z):
        x, y = np.meshgrid(np.linspace(0.01, 0.28, 12), np.linspace(0.01, 0.28, 12))
        return np.column_stack((x.ravel(), y.ravel(), z(x, y).ravel()))

    def test_flat_slope_roughness_step_and_pit(self):
        for surface, bit in [
                (lambda x, y: x*0, 0), (lambda x, y: x*0.6, 1),
                (lambda x, y: 0.09*np.sin(x*150), 2),
                (lambda x, y: x*0+0.25, 4), (lambda x, y: x*0-0.25, 8)]:
            reason = classify_cells(self.surface(surface))[0][4]
            self.assertEqual(reason, 0) if bit == 0 else self.assertTrue(reason & bit)
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


if __name__ == '__main__':
    unittest.main()
