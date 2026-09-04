import pathlib
import sys
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
from terrain_geometry import classify_cells, project_depth, transform_points  # noqa: E402


class GeometryTest(unittest.TestCase):
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
