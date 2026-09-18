import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from terrain_map import TerrainMap


def plane_cloud(gx=0.1, gy=0.0, z0=0.5, span=1.0, step=0.05):
    """Generate dense point cloud representing an inclined plane."""
    x, y = np.meshgrid(np.arange(0.0, span, step), np.arange(0.0, span, step))
    z = z0 + gx * x + gy * y
    return np.column_stack((x.ravel(), y.ravel(), z.ravel()))


class TerrainExportTests(unittest.TestCase):
    def test_export_dem_empty_map(self):
        terrain = TerrainMap()
        with tempfile.TemporaryDirectory() as folder:
            dem_path = Path(folder) / 'empty_map.asc'
            meta = terrain.export_dem(dem_path)
            self.assertTrue(dem_path.exists())
            self.assertEqual(meta['valid_cells'], 0)
            self.assertEqual(meta['total_cells'], 1)
            content = dem_path.read_text(encoding='ascii')
            self.assertIn('ncols        1', content)
            self.assertIn('nrows        1', content)
            self.assertIn('NODATA_value -9999.0', content)
            json_path = Path(folder) / 'empty_map.json'
            self.assertTrue(json_path.exists())
            loaded_meta = json.loads(json_path.read_text(encoding='utf-8'))
            self.assertEqual(loaded_meta['valid_cells'], 0)

    def test_export_mesh_empty_map(self):
        terrain = TerrainMap()
        with tempfile.TemporaryDirectory() as folder:
            obj_path = Path(folder) / 'empty_map.obj'
            n_v, n_f = terrain.export_mesh(obj_path)
            self.assertEqual(n_v, 0)
            self.assertEqual(n_f, 0)
            self.assertTrue(obj_path.exists())
            content = obj_path.read_text(encoding='ascii')
            self.assertIn('Empty mesh', content)

    def test_export_dem_with_inclined_plane(self):
        terrain = TerrainMap(resolution=0.25)
        # Add 4 frames to ensure sufficient voxel support and stability
        cloud = plane_cloud(gx=0.2, gy=0.1, z0=1.0, span=1.0, step=0.04)
        for stamp in range(1, 5):
            terrain.add_frame(cloud, float(stamp))

        with tempfile.TemporaryDirectory() as folder:
            dem_path = Path(folder) / 'terrain.asc'
            meta = terrain.export_dem(dem_path)

            self.assertTrue(dem_path.exists())
            json_path = Path(folder) / 'terrain.json'
            self.assertTrue(json_path.exists())

            self.assertGreater(meta['valid_cells'], 0)
            self.assertIsNotNone(meta['min_z'])
            self.assertIsNotNone(meta['max_z'])
            self.assertAlmostEqual(meta['cellsize'], 0.25, places=4)

            # Check ASCII Grid format
            lines = [l.strip() for l in dem_path.read_text(encoding='ascii').splitlines() if l.strip()]
            self.assertTrue(lines[0].startswith('ncols'))
            self.assertTrue(lines[1].startswith('nrows'))
            self.assertTrue(lines[2].startswith('xllcorner'))
            self.assertTrue(lines[3].startswith('yllcorner'))
            self.assertTrue(lines[4].startswith('cellsize'))
            self.assertTrue(lines[5].startswith('NODATA_value'))

            # Parse grid rows
            grid_lines = lines[6:]
            self.assertEqual(len(grid_lines), meta['nrows'])
            first_row_vals = [float(v) for v in grid_lines[0].split()]
            self.assertEqual(len(first_row_vals), meta['ncols'])

    def test_export_mesh_topology(self):
        terrain = TerrainMap(resolution=0.25)
        # Create a 4x4 cell region (spanning 1.0m x 1.0m)
        cloud = plane_cloud(gx=0.0, gy=0.0, z0=0.0, span=1.0, step=0.03)
        for stamp in range(1, 5):
            terrain.add_frame(cloud, float(stamp))

        with tempfile.TemporaryDirectory() as folder:
            obj_path = Path(folder) / 'terrain.obj'
            n_v, n_f = terrain.export_mesh(obj_path)

            self.assertGreater(n_v, 0)
            self.assertGreater(n_f, 0)

            content = obj_path.read_text(encoding='ascii').splitlines()
            v_lines = [l for l in content if l.startswith('v ')]
            f_lines = [l for l in content if l.startswith('f ')]

            self.assertEqual(len(v_lines), n_v)
            self.assertEqual(len(f_lines), n_f)

            # Verify vertex format and face index ranges
            for vl in v_lines:
                parts = vl.split()
                self.assertEqual(len(parts), 4)
                vx, vy, vz = float(parts[1]), float(parts[2]), float(parts[3])
                self.assertFalse(math.isnan(vx) or math.isnan(vy) or math.isnan(vz))

            for fl in f_lines:
                parts = fl.split()
                self.assertEqual(len(parts), 4)
                idx1, idx2, idx3 = int(parts[1]), int(parts[2]), int(parts[3])
                # 1-indexed and bounded by n_v
                self.assertTrue(1 <= idx1 <= n_v)
                self.assertTrue(1 <= idx2 <= n_v)
                self.assertTrue(1 <= idx3 <= n_v)
                # Triangle face has distinct vertices
                self.assertEqual(len({idx1, idx2, idx3}), 3)


if __name__ == '__main__':
    unittest.main()
