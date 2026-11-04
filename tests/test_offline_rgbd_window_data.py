"""Tiny synthetic file tests for the offline-only sensor/truth boundary."""
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from offline_rgbd_window.data import (SensorFrame, describe_manifest, interpolate_truth,
                                      iter_sensor_frames, load_truth, sha256_file)


class OfflineRGBDDataTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)

    def arrays(self, stamps):
        count = len(stamps)
        K = np.array([[4., 0., 1.5], [0., 5., 1.], [0., 0., 1.]])
        angle = .2
        R_wb = np.array([[math.cos(angle), -math.sin(angle), 0.],
                         [math.sin(angle), math.cos(angle), 0.], [0., 0., 1.]])
        R_bc = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
        transform = np.eye(4);transform[:3, :3] = R_bc;transform[:3, 3] = [.2, -.1, .3]
        depth = np.array([[.5, .8, 1.], [1.2, 1.5, 2.]], np.float32)
        v, u = np.indices(depth.shape)
        optical = np.stack(((u + .5 - K[0, 2]) * depth / K[0, 0],
                            (v + .5 - K[1, 2]) * depth / K[1, 1], depth), axis=2)
        grid = (optical @ R_bc.T + transform[:3, 3]) @ R_wb.T
        grid[0, 0] = np.nan  # Original masked rover/unknown pixel stays masked.
        return dict(stamp_ns=np.asarray(stamps, np.int64), stamp=np.asarray(stamps) / 1e9,
            gray=np.full((count, 2, 3), 25, np.uint8), depth_raw=np.tile(depth, (count, 1, 1)),
            xyz_grid=np.tile(grid, (count, 1, 1, 1)), body_rotation=np.tile(R_wb, (count, 1, 1)),
            camera_tf_matrix=np.tile(transform, (count, 1, 1)), camera_K=np.tile(K, (count, 1, 1)),
            camera_rotation=np.tile(R_wb @ R_bc, (count, 1, 1)),
            camera_translation=np.tile(R_wb @ transform[:3, 3], (count, 1)), yaw=np.full(count, angle),
            wheel=np.tile([.1, -.2], (count, 1)), wheel_available=np.ones(count, bool))

    def write(self, groups, mutate=None):
        chunks = [];total = 0
        for index, stamps in enumerate(groups):
            arrays = self.arrays(stamps)
            if mutate is not None:mutate(index, arrays)
            filename = f'frames-{index:04}.npz'
            np.savez_compressed(self.path / filename, **arrays)
            chunks.append(dict(index=index, frames_file=filename, count=len(stamps),
                first_stamp_ns=stamps[0], last_stamp_ns=stamps[-1], first_global_index=total,
                last_global_index=total + len(stamps) - 1, frames_sha256=sha256_file(self.path / filename)))
            total += len(stamps)
        manifest = self.path / 'manifest.json'
        manifest.write_text(json.dumps(dict(complete=True, case='synthetic', extracted_frames=total,
                                            omission_count=1, chunks=chunks)))
        return manifest

    def test_sensor_geometry_half_pixel_original_mask_and_camera_model(self):
        path = self.write([[100_000_000]])
        frame = next(iter_sensor_frames(path, verify_hashes=True))
        self.assertIsInstance(frame, SensorFrame)
        self.assertEqual(frame.stamp, .1);self.assertAlmostEqual(frame.yaw, .2)
        self.assertEqual((frame.wheel_vx, frame.wheel_wz), (.1, -.2))
        self.assertTrue(np.isnan(frame.optical_grid[0, 0]).all())
        self.assertAlmostEqual(frame.optical_grid[0, 1, 0], 0.)
        self.assertAlmostEqual(frame.optical_grid[1, 2, 0], .5)
        model = frame.camera_model
        expected = frame.optical_grid @ model['rotation'].T + model['translation']
        np.testing.assert_allclose(expected, frame.xyz_grid, equal_nan=True)
        self.assertFalse(frame.gray.flags.writeable)
        self.assertFalse(frame.xyz_grid.flags.writeable)

    def test_lazy_chunks_keep_gaps_and_never_load_a_future_chunk_early(self):
        path = self.write([[100_000_000, 200_000_000], [500_000_000]])
        original = np.load;opened = []
        def counted(source, *args, **kwargs):
            opened.append(Path(source).name)
            return original(source, *args, **kwargs)
        with patch('offline_rgbd_window.data.np.load', side_effect=counted):
            frames = iter_sensor_frames(path)
            self.assertEqual(next(frames).stamp_ns, 100_000_000)
            self.assertEqual(opened, ['frames-0000.npz'])
            self.assertEqual(next(frames).stamp_ns, 200_000_000)
            self.assertEqual(opened, ['frames-0000.npz'])
            self.assertEqual(next(frames).stamp_ns, 500_000_000)
            self.assertEqual(opened, ['frames-0000.npz', 'frames-0001.npz'])
            with self.assertRaises(StopIteration):next(frames)

    def test_bad_time_shape_transform_and_geometry_are_rejected(self):
        for error in ('time', 'shape', 'rotation', 'translation', 'xyz', 'yaw', 'stamp'):
            with self.subTest(error=error):
                def change(index, arrays):
                    if index != 1:return
                    if error == 'time':arrays['stamp_ns'][0] = 100_000_000
                    elif error == 'shape':arrays['xyz_grid'] = arrays['xyz_grid'][:, :, :2]
                    elif error == 'rotation':arrays['camera_tf_matrix'][0, 0, 0] = .25
                    elif error == 'translation':arrays['camera_translation'][0, 0] += .2
                    elif error == 'xyz':arrays['xyz_grid'][0, 1, 2, 0] += .2
                    elif error == 'yaw':arrays['yaw'][0] += .2
                    else:arrays['stamp'][0] += .2
                path = self.write([[100_000_000], [200_000_000]], change)
                with self.assertRaises(ValueError):list(iter_sensor_frames(path))

    def test_missing_wheel_stays_none_and_available_nonfinite_is_invalid(self):
        def absent(index, arrays):
            arrays['wheel_available'][:] = False;arrays['wheel'][:] = np.nan
        frame = next(iter_sensor_frames(self.write([[100_000_000]], absent)))
        self.assertEqual((frame.wheel_vx, frame.wheel_wz), (None, None))
        def invalid(index, arrays):arrays['wheel'][0, 0] = np.nan
        with self.assertRaises(ValueError):list(iter_sensor_frames(self.write([[100_000_000]], invalid)))

    def test_changed_calibration_and_manifest_count_or_hash_do_not_pass(self):
        def change(index, arrays):
            if index == 1:arrays['camera_K'][0, 0, 0] += 1.
        path = self.write([[100_000_000], [200_000_000]], change)
        with self.assertRaisesRegex(ValueError, 'calibration'):list(iter_sensor_frames(path))
        path = self.write([[100_000_000]])
        manifest = json.loads(path.read_text());manifest['extracted_frames'] = 2
        path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, 'count'):list(iter_sensor_frames(path))
        manifest['extracted_frames'] = 1;manifest['chunks'][0]['frames_sha256'] = '0' * 64
        path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, 'SHA256'):list(iter_sensor_frames(path, verify_hashes=True))

    def test_metadata_hash_and_original_files_remain_unchanged(self):
        path = self.write([[100_000_000], [200_000_000]])
        before = {p.name:sha256_file(p) for p in self.path.iterdir()}
        description = describe_manifest(path)
        self.assertEqual((description['count'], description['chunks'], description['duration_s']), (2, 2, .1))
        self.assertEqual(description['sha256'], hashlib.sha256(path.read_bytes()).hexdigest())
        list(iter_sensor_frames(path))
        self.assertEqual(before, {p.name:sha256_file(p) for p in self.path.iterdir()})

    def test_truth_is_separate_and_exact_interpolation_never_extrapolates(self):
        truth = [dict(stamp_ns=100_000_000, xyz=[0., 1., 2.], q=[0., 0., 0., 1.]),
                 dict(stamp_ns=200_000_000, xyz=[2., 3., 4.]),
                 dict(stamp_ns=400_000_000, xyz=[4., 5., 6.])]
        path = self.path / 'truth.jsonl'
        path.write_text('\n'.join(json.dumps(row) for row in truth))
        loaded = load_truth(path)
        self.assertEqual(loaded, truth)
        xyz, valid = interpolate_truth(loaded, [0, 100_000_000, 150_000_000, 300_000_000, 400_000_000, 500_000_000])
        np.testing.assert_array_equal(valid, [False, True, True, False, True, False])
        np.testing.assert_allclose(xyz[[1, 2, 4]], [[0., 1., 2.], [1., 2., 3.], [4., 5., 6.]])
        self.assertTrue(np.isnan(xyz[~valid]).all())
        self.assertNotIn('truth', SensorFrame.__dataclass_fields__)

    def test_truth_recording_gaps_bad_rows_and_invalid_queries(self):
        truth = [dict(stamp_ns=0, xyz=[0., 0., 0.]), dict(stamp_ns=100_000_001, xyz=[1., 1., 1.])]
        xyz, valid = interpolate_truth(truth, [50_000_000])
        self.assertFalse(valid[0]);self.assertTrue(np.isnan(xyz[0]).all())
        for bad in (truth[::-1], [truth[0], truth[0]], [dict(stamp_ns=0, xyz=[np.nan, 0., 0.])]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):interpolate_truth(bad, [0])
        with self.assertRaises(ValueError):interpolate_truth(truth, [.5])
        with self.assertRaises(ValueError):interpolate_truth(truth, [0], max_gap=0.)
        self.assertEqual(interpolate_truth([], [1])[1].tolist(), [False])


if __name__ == '__main__':
    unittest.main()
