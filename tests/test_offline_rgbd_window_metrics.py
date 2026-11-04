"""Availability must not disappear merely because evaluation truth is absent."""
from pathlib import Path
import json
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from offline_rgbd_window.run import score
from offline_rgbd_window import run as replay


class MetricsContracts(unittest.TestCase):
    def test_truth_gap_does_not_remove_uncertainty_pass_or_failed_plan(self):
        rows = [dict(stamp_ns=1, output=True, position=[0., 0., 0.], covariance=.003**2*np.eye(3)),
                dict(stamp_ns=2, output=False, reason='no_features')]
        metrics = score(rows, np.full((2, 3), np.nan), [False, False], np.zeros(3))
        self.assertEqual(metrics['planned'], 2)
        self.assertEqual(metrics['outputs'], 1)
        self.assertEqual(metrics['output_within_50mm'], 1)
        self.assertEqual(metrics['output_within_50mm_fraction_of_all_plans'], .5)
        self.assertEqual(metrics['scored_outputs'], 0)
        self.assertIsNone(metrics['xy_95pct_ellipse_coverage'])


class BaselineCacheContracts(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)
        self.serial = 0

    def fixture(self, mutation=None, reseal=False):
        self.serial += 1
        folder = self.path/str(self.serial);folder.mkdir()
        manifest = folder/'manifest.json'
        replay.write_json(manifest, dict(extracted_frames=2))
        rows = [dict(stamp_ns=stamp, output=True, reason='accepted', position=[12., 5., 3.],
                     covariance=(.003**2*np.eye(3)).tolist(), baseline_s=.25, frontend_s=99.)
                for stamp in (100_000_000, 600_000_000)]
        baseline = folder/'baseline.jsonl'
        baseline.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        source = replay.SLAM_SCRIPTS/'metric_rgbd_odometry.py'
        bound = {str(source):replay.digest(source)}
        record = dict(source_before=bound, source_after=dict(bound), sensor_frames=2,
            config=dict(max_sensor_frames=0), manifest_sha256=replay.digest(manifest),
            output_sha256={'baseline.jsonl':replay.digest(baseline)})
        if mutation is not None:
            mutation(rows, record, manifest)
            baseline.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        if reseal:record['output_sha256']['baseline.jsonl'] = replay.digest(baseline)
        replay.write_json(folder/'run.json', record)
        return folder, manifest, rows

    def test_cache_main_skips_old_core_keeps_final_C_out_of_estimator_and_retimes_frontend(self):
        folder, manifest, rows = self.fixture()
        output = self.path/'output'
        sensors = [NS(stamp_ns=row['stamp_ns'], stamp=row['stamp_ns']/1e9,
            nominal_R_wb=np.eye(3), R_bc=np.eye(3), t_bc=np.zeros(3)) for row in rows]
        features = dict(points=np.array([[1., 0., 1.], [0., 1., 1.], [1., 1., 1.]]),
                        covariance=np.tile(.001*np.eye(3), (3, 1, 1)), track_ids=np.array([1, 2, 3]))
        frontend = Mock(stats={});frontend.process.return_value=features
        frontend.select.return_value=(range(3), dict(adequate=True))
        estimator = Mock();estimator.add_frame.return_value=NS(position=np.zeros(3), covariance=.00001*np.eye(3), diagnostics={})
        def evaluate(output, baseline, plans, stamp, origin, seed, args):
            np.testing.assert_array_equal(origin, [12., 5., 3.])
            np.testing.assert_array_equal(seed, .003**2*np.eye(3))
            self.assertTrue(all(row['frontend_s'] != 99. for row in baseline))
            self.assertTrue(all(row['baseline_s'] == .25 and row['baseline_cached'] for row in baseline))
            self.assertEqual(args.baseline_cache_metadata['historical_baseline_total_compute_seconds'], .5)
            metrics = dict(planned_window_with_start_prior=dict(outputs=2), planned_baseline={}, timing={})
            replay.write_json(output/'metrics.json', metrics)
            return metrics
        argv = ['run', '--manifest', str(manifest), '--truth', str(manifest), '--output', str(output), '--baseline-run', str(folder)]
        with patch.object(sys, 'argv', argv), patch.object(replay, 'MetricRGBDOdometry') as old_core, \
                patch.object(replay, 'TrackFrontend', return_value=frontend), \
                patch.object(replay, 'WindowEstimator', return_value=estimator), \
                patch.object(replay, 'iter_sensor_frames', return_value=iter(sensors)), \
                patch.object(replay, 'evaluate', side_effect=evaluate), patch('builtins.print'):
            replay.main()
        old_core.assert_not_called()
        self.assertEqual(estimator.add_frame.call_count, 2)
        np.testing.assert_array_equal(estimator.add_frame.call_args.args[0].observations[0].point_camera, [1., 0., 1.])
        result = json.loads((output/'run.json').read_text())
        self.assertIsNotNone(result['baseline_cache'])
        self.assertEqual((folder/'baseline.jsonl').read_text().count('99.0'), 2)

    def test_changed_cache_covariance_source_or_manifest_seal_is_rejected(self):
        for kind in ('covariance', 'core', 'manifest', 'invalid_covariance'):
            with self.subTest(kind=kind):
                def mutate(rows, record, manifest):
                    if kind == 'covariance':rows[0]['covariance'][0][0] *= 2.
                    elif kind == 'core':
                        record['source_before'] = {key:'0'*64 for key in record['source_before']}
                        record['source_after'] = dict(record['source_before'])
                    elif kind == 'manifest':manifest.write_text('{"extracted_frames": 2, "changed": true}')
                    else:rows[0]['covariance'][0][0] = -1.
                folder, manifest, _ = self.fixture(mutate, reseal=kind == 'invalid_covariance')
                with self.assertRaises(ValueError):replay.BaselineCache(folder, manifest)

    def test_missing_duplicate_disordered_sensor_mismatch_and_unused_tail_cannot_pass(self):
        for kind in ('missing', 'duplicate', 'disordered', 'extra'):
            with self.subTest(kind=kind):
                def mutate(rows, record, manifest):
                    if kind == 'missing':rows.pop()
                    elif kind == 'extra':rows.append(dict(rows[-1], stamp_ns=700_000_000))
                    elif kind == 'duplicate':rows[1]['stamp_ns'] = rows[0]['stamp_ns']
                    else:rows.reverse()
                folder, manifest, _ = self.fixture(mutate, reseal=True)
                with self.assertRaises(ValueError):replay.BaselineCache(folder, manifest)
        folder, manifest, _ = self.fixture()
        cache = replay.BaselineCache(folder, manifest)
        with self.assertRaises(ValueError):cache.row_for(200_000_000)
        row = cache.row_for(100_000_000);row['covariance'][0][0] = 9.
        self.assertNotEqual(cache.rows[0]['covariance'][0][0], 9.)
        with self.assertRaisesRegex(ValueError, 'tail'):cache.finish()
        cache.row_for(600_000_000);cache.finish()
        with self.assertRaises(ValueError):cache.row_for(700_000_000)


if __name__ == '__main__':
    unittest.main()
