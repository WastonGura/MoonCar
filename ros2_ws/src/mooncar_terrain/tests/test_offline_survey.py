"""Hardware-free task, reconstruction, provenance, quality and configuration boundaries."""
import copy
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
from offline_survey import create_example, read_json, run_task, task_cells, write_json
from survey_contract import load_input, validate_task, TerrainMap, digest
from survey_configuration import audit_configuration, camera_fov, project_configuration


class OfflineSurveyTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        create_example(self.root/'example')
        self.task_path = self.root/'example/task.json'
        self.task = read_json(self.task_path)

    def tearDown(self):
        self.temp.cleanup()

    def save_task(self):
        write_json(self.task_path, self.task)

    def test_coverage_threshold_removed_but_archived_task_still_loads(self):
        self.assertNotIn('minimum_coverage',self.task['targets'])
        validate_task(self.task)
        original=run_task(self.task_path,self.root/'current')
        self.task['targets']['minimum_coverage']=1.
        self.save_task();validate_task(self.task)
        legacy=run_task(self.task_path,self.root/'legacy')
        self.assertEqual(original['accuracy'],legacy['accuracy'])
        report=(self.root/'legacy/report.html').read_text(encoding='utf-8')
        self.assertIn('覆盖率仅作统计，不作为通过条件',report)
        self.assertNotIn('覆盖／达标率目标',report)

    def test_known_plane_bias_missing_and_mixed_cells(self):
        result = run_task(self.task_path, self.root/'result')
        self.assertEqual(result['execution_status'], 'completed')
        self.assertEqual(result['coverage']['task_cells'], 64)
        counts = result['coverage']['state_counts']
        self.assertEqual(counts['unobserved'], 4)
        self.assertGreaterEqual(counts['invalid_fit'], 1)
        self.assertEqual(counts['error_exceeds_threshold'], 4)
        self.assertGreater(counts['qualified'], 0)
        self.assertEqual(result['frames']['outside_time_window'], 1)
        with (self.root/'result/cells.csv').open(encoding='utf-8-sig') as handle:
            rows = list(csv.DictReader(handle))
        good = [r for r in rows if r['state'] == 'qualified']
        self.assertLess(max(abs(float(r['height_error_m'])) for r in good), 1e-5)
        self.assertLess(max(abs(float(r['slope_error_deg'])) for r in good), 1e-3)

    def test_no_reference_reports_unknown_accuracy_not_false_success(self):
        self.task.pop('reference'); self.task.pop('configuration'); self.save_task()
        result = run_task(self.task_path, self.root/'result')
        self.assertIsNone(result['accuracy']['P']); self.assertIsNone(result['accuracy']['S'])
        self.assertIsNone(result['accuracy']['geometry_pass'])
        self.assertGreater(result['coverage']['state_counts']['fitted_accuracy_unknown'], 0)
        self.assertFalse(result['physical_accuracy_certified'])
        self.assertEqual(result['configuration_status'], 'not_checked')

    def test_reference_missing_cells_cannot_reduce_denominator(self):
        ref_path = self.task_path.parent/'reference.json'; ref = read_json(ref_path)
        ref['cells'].pop(); write_json(ref_path, ref)
        with self.assertRaisesRegex(ValueError, 'every declared cell'):
            run_task(self.task_path, self.root/'result')

    def test_reference_unverified_never_publishes_pass_colours(self):
        ref_path = self.task_path.parent/'reference.json'; ref = read_json(ref_path)
        ref['alignment']['status'] = 'unverified'; write_json(ref_path, ref)
        result = run_task(self.task_path, self.root/'result')
        self.assertFalse(result['accuracy']['evaluable']); self.assertIsNone(result['accuracy']['P'])
        self.assertEqual(result['coverage']['state_counts']['qualified'], 0)

    def test_no_synthetic_exact_reference_for_physical_data(self):
        self.task['source']['kind'] = 'physical'; self.save_task()
        path = self.task_path.parent/'data/manifest.json'; data = read_json(path)
        data['contract']['source_kind'] = 'physical'; write_json(path, data)
        with self.assertRaisesRegex(ValueError, 'Synthetic exact'):
            run_task(self.task_path, self.root/'result')

    def test_provided_alignment_requires_unchanged_evidence(self):
        ref_path = self.task_path.parent/'reference.json'; ref = read_json(ref_path)
        ref['alignment']['status'] = 'provided_verified'
        evidence = self.task_path.parent/'alignment.txt'; evidence.write_text('independently supplied fixed alignment')
        ref['alignment'].update(evidence_file='alignment.txt', sha256=digest(evidence)); write_json(ref_path, ref)
        result = run_task(self.task_path, self.root/'result')
        self.assertTrue(result['accuracy']['evaluable'])
        evidence.write_text('changed')
        with self.assertRaisesRegex(ValueError, 'evidence hash'):
            run_task(self.task_path, self.root/'second')

    def test_input_contract_rejects_units_clock_frame_and_versions(self):
        path = self.task_path.parent/'data/manifest.json'; original = read_json(path)
        for field in ('unit', 'clock', 'frame', 'pose_version', 'calibration_version'):
            with self.subTest(field=field):
                data = copy.deepcopy(original); data['contract'][field] = 'wrong'; write_json(path, data)
                with self.assertRaisesRegex(ValueError, 'mismatch'):
                    run_task(self.task_path, self.root/('bad-'+field))

    def test_duplicate_or_invalid_future_timestamps_are_rejected(self):
        path = self.task_path.parent/'data/manifest.json'; data = read_json(path)
        data['frames'][-1]['stamp_s'] = 2.; write_json(path, data)
        with self.assertRaisesRegex(ValueError, 'strictly increase'):
            run_task(self.task_path, self.root/'bad')

    def test_nonfinite_points_and_bad_quaternion_are_rejected(self):
        points = self.task_path.parent/'data/points.npy'; value = np.load(points); value[0, 0] = np.nan
        np.save(points, value)
        with self.assertRaisesRegex(ValueError, 'Invalid or oversized'):
            run_task(self.task_path, self.root/'bad')
        path = self.task_path.parent/'data/manifest.json'; data = read_json(path)
        data['frames'][0]['pose']['quaternion_xyzw'] = [0, 0, 0, 2]; write_json(path, data)
        with self.assertRaisesRegex(ValueError, 'unit quaternion'):
            run_task(self.task_path, self.root/'bad2')

    def test_folder_path_escape_is_rejected(self):
        np.save(self.task_path.parent/'elsewhere.npy', np.zeros((3, 3)))
        path = self.task_path.parent/'data/manifest.json'; data = read_json(path)
        data['frames'][0]['points_file'] = '../elsewhere.npy'; write_json(path, data)
        with self.assertRaisesRegex(ValueError, 'escaping'):
            run_task(self.task_path, self.root/'bad')

    def test_rectified_depth_adapter_uses_declared_units_intrinsics_and_pose(self):
        path = self.task_path.parent/'data/manifest.json'; data = read_json(path)
        depth = np.full((40, 40), 1000, dtype=np.uint16); np.save(path.parent/'depth.npy', depth)
        data['camera'] = dict(width=40, height=40, encoding='16UC1', intrinsics=[40, 40, 20, 20], pixel_offset=0, rectified=True)
        data['frames'] = [dict(stamp_s=1, depth_file='depth.npy', pose=dict(translation_m=[1, 1, 1], quaternion_xyzw=[1, 0, 0, 0]))]
        write_json(path, data)
        self.task.pop('reference'); self.save_task()
        result = run_task(self.task_path, self.root/'result')
        model = TerrainMap.load(self.root/'result/observations.npz')
        self.assertGreater(result['coverage']['fitted_cells'], 0)
        self.assertLess(np.max(np.abs(model.cloud()[:, 2])), 1e-7)

    def test_missing_camera_info_rejected_for_depth(self):
        path = self.task_path.parent/'data/manifest.json'; data = read_json(path)
        np.save(path.parent/'depth.npy', np.ones((40, 40), dtype=np.float32))
        data['frames'][0].pop('points_file'); data['frames'][0]['depth_file'] = 'depth.npy'; write_json(path, data)
        with self.assertRaises(ValueError): run_task(self.task_path, self.root/'bad')

    def test_empty_depth_and_sparse_task_points_preserve_missingness(self):
        path = self.task_path.parent/'data/manifest.json'; data = read_json(path)
        # Two task points plus one outside still constitute a valid raw frame.
        np.save(path.parent/'sparse.npy', np.array([[.1,.1,0], [.12,.12,0], [10,10,0]], dtype=float))
        data['frames'] = [dict(stamp_s=1, points_file='sparse.npy', pose=dict(translation_m=[0,0,0], quaternion_xyzw=[0,0,0,1]))]
        write_json(path, data); self.task.pop('reference'); self.save_task()
        result = run_task(self.task_path, self.root/'result')
        self.assertEqual(result['coverage']['observed_cells'], 1)
        self.assertEqual(result['coverage']['state_counts']['invalid_fit'], 1)
        self.assertEqual(result['coverage']['fitted_cells'], 0)
        values = np.loadtxt(self.root/'result/dem.asc', skiprows=6)
        self.assertTrue(np.all(values == -9999))

    def test_task_area_remains_full_in_dem_and_unknown_cells(self):
        run_task(self.task_path, self.root/'result')
        values = np.loadtxt(self.root/'result/dem.asc', skiprows=6)
        self.assertEqual(values.shape, (8, 8)); self.assertEqual(values[0, -1], -9999)
        meta = read_json(self.root/'result/dem.json')
        self.assertEqual(meta['coordinates']['frame'], 'survey_local')
        self.assertEqual(meta['total_cells'], 64)

    def test_roi_assignment_matches_baseline_float32_precision(self):
        path = self.task_path.parent/'data/manifest.json'; data = read_json(path)
        # All values round to the next cell in baseline sensor-point storage.
        np.save(path.parent/'boundary.npy', np.array([[.2499999999,.1,0], [.2499999999,.12,0], [.2499999999,.14,0]]))
        data['frames'] = [dict(stamp_s=1, points_file='boundary.npy', pose=dict(translation_m=[0,0,0], quaternion_xyzw=[0,0,0,1]))]
        write_json(path, data); self.task['region']['bounds_m'] = [0,0,.25,.25]
        self.task.pop('reference'); self.save_task()
        result = run_task(self.task_path, self.root/'result')
        self.assertEqual(result['coverage']['observed_cells'], 0)

    def test_packaged_replay_preserves_geometry_and_configuration(self):
        first = run_task(self.task_path, self.root/'result')
        second = run_task(self.root/'result/task-replay.json', self.root/'replayed')
        self.assertEqual(first, second)
        np.testing.assert_array_equal(TerrainMap.load(self.root/'result/observations.npz').cloud(),
                                      TerrainMap.load(self.root/'replayed/observations.npz').cloud())
        self.assertEqual(read_json(self.root/'replayed/configuration-audit.json')['source_mismatches'], [])

    def test_npz_adapter_requires_contract_and_matches_folder(self):
        frames, contract, _, _ = load_input(self.task, self.task_path)
        model = TerrainMap(frame=self.task['coordinates']['frame'])
        for frame in frames:
            model.add_frame(frame.points, frame.stamp, frame.translation, frame.quaternion)
        model.save(self.task_path.parent/'raw.npz'); write_json(self.task_path.parent/'contract.json', contract)
        first = run_task(self.task_path, self.root/'folder-result')
        self.task['input'] = dict(format='observations-v2', path='raw.npz', contract='contract.json'); self.save_task()
        second = run_task(self.task_path, self.root/'npz-result')
        self.assertEqual(first['coverage'], second['coverage'])
        self.assertEqual(first['accuracy'], second['accuracy'])
        replay = run_task(self.root/'npz-result/task-replay.json', self.root/'npz-replay')
        self.assertEqual(replay['accuracy'], second['accuracy'])

    def test_inputs_and_existing_output_are_not_modified(self):
        paths = list(self.task_path.parent.rglob('*')); before = {p: digest(p) for p in paths if p.is_file()}
        run_task(self.task_path, self.root/'result')
        with self.assertRaisesRegex(ValueError, 'already exists'):
            run_task(self.task_path, self.root/'result')
        self.assertEqual(before, {p: digest(p) for p in before})
        with self.assertRaisesRegex(ValueError, 'inside a folder'):
            run_task(self.task_path, self.task_path.parent/'data/result')

    def test_task_boundaries_and_unsupported_shapes(self):
        for key, value in (('region', dict(type='polygon', bounds_m=[0,0,1,1])),
                           ('grid', dict(cell_m=.00001, voxel_m=.000001)),
                           ('time', dict(clock='simulation', start_s=3, end_s=1))):
            with self.subTest(key=key):
                task = copy.deepcopy(self.task); task[key] = value
                with self.assertRaises(ValueError): validate_task(task)
        cells = task_cells(self.task)
        self.assertEqual(min(c['x'] for c in cells.values()), .125)
        self.assertEqual(max(c['x'] for c in cells.values()), 1.875)

    def test_report_escapes_user_text_and_records_state_legend(self):
        self.task['id'] = '<script>alert(1)</script>'; self.save_task()
        run_task(self.task_path, self.root/'result')
        report = (self.root/'result/report.html').read_text(encoding='utf-8')
        self.assertNotIn('<script>alert', report); self.assertIn('&lt;script&gt;', report)
        for label in ('未观测', '无法有效拟合', '精度未知', '误差超限', '源码声明'):
            self.assertIn(label, report)

    def test_project_audit_detects_fov_speed_and_unknown_runtime(self):
        result = audit_configuration(project_configuration())
        checks = {c['field']: c for c in result['checks']}
        self.assertEqual(result['scope'], 'source_declarations')
        self.assertEqual(checks['horizontal_fov_deg']['state'], 'difference')
        self.assertAlmostEqual(checks['horizontal_fov_deg']['observed'], 73)
        self.assertEqual(checks['linear_speed_m_s']['state'], 'difference')
        self.assertEqual(checks['footprint_size_m']['state'], 'unknown')

    def test_runtime_camera_info_and_source_hash_checks(self):
        camera = dict(width=640, height=480, k=[320/math.tan(math.radians(73/2)),0,320,0,500,240,0,0,1])
        self.assertAlmostEqual(camera_fov(camera), 73)
        file = self.root/'snapshot.txt'; file.write_text('original')
        config = dict(format='mooncar-survey-configuration-v1', expected=dict(horizontal_fov_deg=73),
                      observed=dict(camera_info=camera), scope='runtime_snapshot', source_files=[dict(path=str(file),sha256=digest(file))])
        self.assertEqual(audit_configuration(config)['checks'][0]['state'], 'consistent')
        file.write_text('changed'); self.assertEqual(audit_configuration(config)['status'], 'source_changed')

    def test_duplicate_json_keys_and_nonfinite_targets_rejected(self):
        file = self.root/'duplicate.json'; file.write_text('{"id":1,"id":2}')
        with self.assertRaisesRegex(ValueError, 'Duplicate'): read_json(file)
        task = copy.deepcopy(self.task); task['targets']['height_tolerance_m'] = float('nan')
        with self.assertRaises(ValueError): validate_task(task)

    def test_cli_validates_and_generates_without_ros(self):
        result = subprocess.run([sys.executable, '-B', str(ROOT/'scripts/offline_survey.py'), 'run',
                                 str(self.task_path), '--output', str(self.root/'cli')],
                                capture_output=True, encoding='utf-8')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)['execution_status'], 'completed')
        self.assertNotIn('rclpy', sys.modules)


if __name__ == '__main__':
    unittest.main()
