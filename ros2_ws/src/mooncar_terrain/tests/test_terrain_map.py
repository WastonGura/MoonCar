import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from terrain_map import TerrainMap


def plane(gx=0., gy=0.):
    x, y = np.meshgrid(np.arange(-.475, .5, .05), np.arange(-.475, .5, .05))
    return np.column_stack((x.ravel(), y.ravel(), (gx * x + gy * y).ravel()))


class TerrainMapTests(unittest.TestCase):
    def test_voxel_centroid_does_not_cross_observed_cell_boundary(self):
        terrain = TerrainMap()
        points = np.array([[0., 0., 0.], [2e-8, 0., 0.], [0., 2e-8, 0.]])
        terrain.add_frame(points, 1., translation=(-15.5000001, -17.5000001, 0.))
        cells = terrain.quality(with_surface=True)['cells']
        self.assertEqual([(cell['ix'], cell['iy']) for cell in cells], [(-63, -71)])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'boundary.npz'
            terrain.save(path)
            self.assertEqual(TerrainMap.load(path).quality(with_surface=True),
                             terrain.quality(with_surface=True))

    def test_cached_surface_matches_fresh_fit_after_new_observations(self):
        terrain = TerrainMap()
        for stamp in range(1, 5):
            terrain.add_frame(plane(.05 * stamp), float(stamp))
            cached = terrain.snapshot()
            terrain._elevation_cache.clear()
            terrain._cloud = None
            self.assertEqual(cached, terrain.snapshot())
            # Callers cannot mutate the next cached result.
            terrain.cloud()[:] = 100
            terrain.elevation()[0]['valid'] = None
            self.assertEqual(cached, terrain.snapshot())

    def test_atomic_replacement_during_fusion_keeps_one_snapshot(self):
        with tempfile.TemporaryDirectory() as folder:
            path, replacement = Path(folder)/'map.npz', Path(folder)/'next.npz'
            old, new = TerrainMap(source='old'), TerrainMap(source='new')
            old.add_frame(plane(), 1.)
            new.add_frame(plane(gx=.1), 2.)
            old.save(path); new.save(replacement)
            version = path.stat().st_mtime_ns
            original = TerrainMap.add_frame
            def replacing(instance, *args, **kwargs):
                replacement.replace(path)
                return original(instance, *args, **kwargs)
            with patch.object(TerrainMap, 'add_frame', replacing):
                loaded = TerrainMap.load(path)
            self.assertEqual(loaded.source, 'old')
            self.assertEqual(loaded.archive_updated_ns, version)
            self.assertEqual(TerrainMap.inspect(path)['source'], 'new')
            np.testing.assert_array_equal(loaded.cloud(), old.cloud())

    def test_robot_pose_is_distinct_from_camera_and_survives_reload(self):
        terrain = TerrainMap()
        terrain.add_frame(plane(), 1., translation=(1., 0., .4),
                          robot_translation=(.7, 0., 0.), robot_quaternion=(0., 0., 0., 1.))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'pose.npz'
            terrain.save(path)
            snapshot = TerrainMap.load(path).snapshot()
            self.assertEqual(snapshot['robot']['position'], [.7, 0., 0.])
            self.assertEqual(snapshot['trajectory'][0]['position'], [1., 0., .4])
        with self.assertRaises(ValueError):
            terrain.add_frame(plane(), 2., robot_translation=(0., 0., 0.))

    def test_transform_fusion_archive_replay_and_export(self):
        terrain = TerrainMap()
        for stamp in (1., 2., 3.):
            terrain.add_frame(plane(.2, -.1), stamp, (1., 2., .3))
        cells = terrain.elevation()
        self.assertTrue(all(c['valid'] for c in cells))
        self.assertAlmostEqual(cells[0]['gx'], .2, places=5)
        self.assertAlmostEqual(cells[0]['gy'], -.1, places=5)
        self.assertGreater(cells[0]['support'], .8)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'survey.npz'
            terrain.save(path)
            loaded = TerrainMap.load(path)
            np.testing.assert_allclose(loaded.cloud(), terrain.cloud(), atol=1e-6)
            self.assertEqual(len(TerrainMap.load(path, until=1).frames), 1)
            self.assertEqual(len(TerrainMap.load(path, until=0).cloud()), 0)
            loaded.export_ply(Path(folder) / 'survey.ply')
            self.assertIn('element vertex', (Path(folder) / 'survey.ply').read_text())
            # Truncation must fail instead of returning a partly trusted map.
            path.write_bytes(path.read_bytes()[:100])
            with self.assertRaises(Exception):
                TerrainMap.load(path)

    def test_unknown_degenerate_multilevel_and_rejected_input(self):
        terrain = TerrainMap()
        line = np.column_stack((np.arange(20) * .05, np.zeros(20), np.zeros(20)))
        terrain.add_frame(line, 1.)
        self.assertFalse(any(c['valid'] for c in terrain.elevation()))
        self.assertTrue(all(c['iy'] == 0 for c in terrain.elevation()))
        with self.assertRaises(ValueError):
            terrain.add_frame(line, 1.)
        with self.assertRaises(ValueError):
            terrain.add_frame(line * math.nan, 2.)
        multi = TerrainMap()
        samples = plane()
        multi.add_frame(np.vstack((samples, samples + (0, 0, .5))), 1.)
        self.assertFalse(any(c['valid'] for c in multi.elevation()))

    def test_rotation_negative_cells_and_empty_archive(self):
        terrain = TerrainMap()
        terrain.add_frame(plane(.2), 1., quaternion=(0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)))
        self.assertTrue(any(c['ix'] < 0 for c in terrain.elevation()))
        for cell in terrain.elevation():
            self.assertAlmostEqual(cell['gx'], 0., places=5)
            self.assertAlmostEqual(cell['gy'], .2, places=5)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'empty.npz'
            TerrainMap().save(path)
            self.assertEqual(TerrainMap.load(path).snapshot()['points'], [])


if __name__ == '__main__':
    unittest.main()
