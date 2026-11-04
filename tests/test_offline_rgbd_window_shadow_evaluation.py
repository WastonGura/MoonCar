"""Pure closed-record evaluator contracts; no ROS, images or estimator runs."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from offline_rgbd_window import evaluate_shadow as evaluation

NS = evaluation.NS


def truth(stop=1., *, axis='x', turning=False):
    result = []
    for n in range(round(stop*100)+1):
        stamp = n*10_000_000
        t = stamp/NS
        xyz = [7.+(.1*t if axis == 'x' else 0.),
               8.+(.1*t if axis == 'y' else 0.), 0.]
        if axis == 'static':
            xyz = [7., 8., 0.]
        result.append(dict(stamp_ns=stamp, xyz=xyz, yaw=.2*t if turning else 0.))
    return result


def fixture(stamps=(0, NS//2, NS), *, end=NS, yaw=0., gauge=0):
    rows = [dict(type='run_start', epoch='e', axes='initial_body_yaw',
                 initial_imu_yaw_rad=yaw, config=dict(interval_ns=500_000_000))]
    for stamp in stamps:
        rows.append(dict(type='exposure', epoch='e', stamp_ns=stamp,
                         received_wall_ns=10*NS+stamp, disposition='processed'))
        rows.append(dict(type='plan', epoch='e', planned_stamp_ns=stamp,
            stamp_ns=stamp, output=True, reason='solved', gauge_stamp_ns=gauge,
            position=[.1*(stamp-gauge)/NS, 0., 0.],
            covariance_conditional=(.003**2*np.eye(3)).tolist(),
            received_wall_ns=10*NS+stamp, started_wall_ns=10*NS+stamp+10_000_000,
            finished_wall_ns=10*NS+stamp+20_000_000,
            ready_sim_ns=stamp+20_000_000, age_sim_s=.02,
            frontend_s=.005, backend_s=.005))
    rows.append(dict(type='run_end', epoch='e', axes='initial_body_yaw',
                     initial_imu_yaw_rad=yaw, gauge_stamp_ns=gauge, sim_ns=end,
                     finished_wall_ns=12*NS, complete=True, worker_terminal=True))
    return rows


def plans(rows):
    return [r for r in rows if r['type'] == 'plan']


class ShadowEvaluationTests(unittest.TestCase):
    def assess(self, rows, actual=None, baseline=(), **kwargs):
        return evaluation.evaluate(rows, truth() if actual is None else actual, baseline,
            imu_world_yaw_offset_rad=0., **kwargs)

    def test_missing_plan_raw_drop_and_high_covariance_remain_in_all_slots(self):
        rows = fixture()
        rows.remove(plans(rows)[1])
        rows.insert(-1, dict(type='exposure', epoch='e', stamp_ns=NS//2, disposition='replaced'))
        plans(rows)[-1]['covariance_conditional'] = (.06**2*np.eye(3)).tolist()
        result, records = self.assess(rows)
        self.assertEqual(result['denominator']['expected_plans'], 3)
        self.assertEqual(result['denominator']['missing_plan_records'], 1)
        self.assertEqual(result['shadow']['outputs'], 2)
        self.assertEqual(result['shadow']['conditional_sigma_within_50mm'], 1)
        self.assertEqual(result['raw_rgbd']['unique_observed_pairs'], 3)
        self.assertEqual(result['raw_rgbd']['dropped_replaced_or_rejected'], 1)
        self.assertEqual(records[1]['reason'], 'missing_plan_record')

    def test_truth_gap_does_not_censor_output_uncertainty_or_timing(self):
        result, records = self.assess(fixture(), truth(.5))
        self.assertEqual(result['shadow']['outputs'], 3)
        self.assertEqual(result['shadow']['conditional_sigma_within_50mm'], 3)
        self.assertEqual(result['shadow']['truth_scored_outputs'], 2)
        self.assertIsNone(records[-1]['error_xy_m'])
        self.assertEqual(result['timing']['compute_wall_s']['count'], 3)

    def test_declared_yaw_only_and_exact_tf_common_comparison(self):
        rows = fixture(yaw=math.pi/2)
        exact = [dict(stamp_ns=s, lookup_tf_stamp_ns=s,
                      map_xyz=[7., 8.+.1*s/NS, 0.]) for s in (0, NS//2, NS)]
        exact[1]['lookup_tf_stamp_ns'] += 1
        result, records = self.assess(rows, truth(axis='y'), exact)
        self.assertLess(result['shadow']['conditional_error_xy_m']['maximum'], 1e-12)
        self.assertEqual(result['common_exact_baseline']['samples'], 2)
        self.assertEqual(result['baseline']['rejected_records']['not_exact_lookup_stamp'], 1)
        self.assertFalse(records[1]['baseline_exact_tf'])
        self.assertAlmostEqual(result['coordinate_contract']['yaw_to_world_rad'], math.pi/2)

    def test_missing_declared_yaw_offset_never_fits_truth_yaw(self):
        result, _ = evaluation.evaluate(fixture(), truth())
        self.assertFalse(result['coordinate_contract']['available'])
        self.assertEqual(result['shadow']['outputs'], 3)
        self.assertEqual(result['shadow']['truth_scored_outputs'], 0)

    def test_late_gauge_keeps_dropped_first_plan_without_reanchoring(self):
        rows = fixture(gauge=NS//2)
        first = plans(rows)[0]
        first.clear()
        first.update(type='plan', epoch='e', planned_stamp_ns=0, stamp_ns=0,
                     output=False, reason='exposure_replaced_before_processing')
        result, _ = self.assess(rows)
        self.assertEqual(result['denominator']['expected_plans'], 3)
        self.assertEqual(result['shadow']['outputs'], 2)
        self.assertEqual(result['coordinate_contract']['gauge_stamp_ns'], NS//2)
        self.assertLess(result['shadow']['conditional_error_xy_m']['maximum'], 1e-12)

    def test_stale_results_keep_error_and_latency_is_not_cpu(self):
        rows = fixture()
        last = plans(rows)[-1]
        last.update(ready_sim_ns=NS+301_000_000, age_sim_s=.301,
                    started_wall_ns=11*NS+100_000_000,
                    finished_wall_ns=11*NS+600_000_000)
        last['position'] = [.3, 0., 0.]
        result, records = self.assess(rows)
        self.assertEqual(result['shadow']['truth_scored_outputs'], 3)
        self.assertEqual(result['shadow']['fresh_sim_at_delivery'], 2)
        self.assertAlmostEqual(result['shadow']['stale_output_error_xy_m']['maximum'], .2)
        self.assertAlmostEqual(records[-1]['queue_wait_wall_s'], .1)
        self.assertAlmostEqual(records[-1]['compute_wall_s'], .5)
        self.assertAlmostEqual(records[-1]['end_to_end_wall_s'], .6)
        self.assertNotIn('cpu_s', result['timing'])

    def test_two_hz_delivery_is_not_continuously_fresh_and_stamp_is_not_republished(self):
        rows = fixture()
        for row in plans(rows):
            row.update(ready_sim_ns=row['stamp_ns'], age_sim_s=0.)
        result, _ = self.assess(rows)
        self.assertEqual(result['shadow']['fresh_sim_at_delivery'], 3)
        self.assertAlmostEqual(result['online_freshness']['fresh_sim_fraction'], .6)
        self.assertAlmostEqual(result['online_freshness']['longest_not_fresh_sim_seconds'], .2)
        self.assertAlmostEqual(result['gaps']['result_delivery_sim']['longest_no_new_output_sim_s'], .5)

    def test_future_phase_boundary_and_point_three_age_are_explicit(self):
        rows = fixture()
        p = plans(rows)
        p[0].update(ready_sim_ns=300_000_000, age_sim_s=.3)
        p[1].update(ready_sim_ns=450_000_000, age_sim_s=-.05)
        p[2].update(ready_sim_ns=949_999_999, age_sim_s=-.050000001)
        result, _ = self.assess(rows)
        self.assertEqual(result['shadow']['fresh_sim_at_delivery'], 2)

    def test_body_turn_coverage_does_not_require_translation(self):
        rows = fixture()
        for row in plans(rows):
            row['position'] = [0., 0., 0.]
        result, _ = self.assess(rows, truth(axis='static', turning=True))
        self.assertEqual(result['moving_plans']['plans'], 0)
        self.assertEqual(result['turning_plans']['plans'], 2)
        self.assertEqual(result['pure_turn_plans']['plans'], 2)
        self.assertAlmostEqual(result['physical']['path_xy_m'], 0.)
        self.assertAlmostEqual(result['physical']['absolute_yaw_change_rad'], .2)

    def test_missing_tail_slots_extend_denominator_and_no_output_gap(self):
        result, _ = self.assess(fixture(end=2*NS))
        self.assertEqual(result['denominator']['expected_plans'], 5)
        self.assertEqual(result['denominator']['missing_plan_records'], 2)
        self.assertEqual(result['shadow']['maximum_consecutive_failed_plans'], 2)
        self.assertAlmostEqual(result['gaps']['result_delivery_sim']['longest_no_new_output_sim_s'], .98)

    def test_post_window_delivery_keeps_error_but_is_not_online_available(self):
        rows = fixture()
        result, _ = self.assess(rows)
        self.assertEqual(result['shadow']['outputs'], 3)
        self.assertEqual(result['shadow']['truth_scored_outputs'], 3)
        self.assertEqual(result['shadow']['outputs_delivered_within_window'], 2)
        self.assertEqual(result['shadow']['outputs_delivered_after_window'], 1)
        self.assertEqual(result['shadow']['fresh_sim_at_delivery'], 3)
        self.assertEqual(result['shadow']['fresh_deliveries_within_window'], 2)

    def test_forced_or_incomplete_shutdown_does_not_claim_closed_capture(self):
        for field in ('complete', 'worker_terminal'):
            with self.subTest(field=field):
                rows = fixture()
                rows[-1][field] = False
                result, _ = self.assess(rows)
                self.assertFalse(result['capture_closed'])
                self.assertEqual(result['shadow']['outputs'], 3)

    def test_hundred_ms_queries_and_five_hundred_ms_keyframes_each_count_once(self):
        stamps = tuple(n*100_000_000 for n in range(6))
        rows = fixture(stamps, end=NS//2)
        rows[0]['config'].update(interval_ns=100_000_000, keyframe_interval_ns=500_000_000)
        for row in plans(rows):
            row['keyframe'] = row['stamp_ns'] % 500_000_000 == 0
        result, records = self.assess(rows)
        self.assertEqual(result['denominator']['expected_plans'], 6)
        self.assertEqual(result['shadow']['outputs'], 6)
        self.assertEqual(result['denominator']['interval_sim_s'], .1)
        self.assertEqual(result['denominator']['declared_keyframe_interval_sim_s'], .5)
        self.assertEqual(sum(r['keyframe'] for r in records), 2)
        self.assertEqual(result['shadow']['fresh_sim_at_delivery'], 6)
        self.assertAlmostEqual(result['online_freshness']['fresh_sim_fraction'], .96)
        self.assertAlmostEqual(result['online_freshness']['longest_not_fresh_sim_seconds'], .02)

    def test_hundred_ms_missing_and_failed_queries_remain_in_declared_grid(self):
        stamps = tuple(n*100_000_000 for n in range(6))
        rows = fixture(stamps, end=NS//2)
        rows[0]['config'].update(interval_ns=100_000_000, keyframe_interval_ns=500_000_000)
        rows.remove(plans(rows)[2])
        failed = next(r for r in plans(rows) if r['stamp_ns'] == 400_000_000)
        failed.update(output=False, reason='query_insufficient_shared_support')
        result, _ = self.assess(rows)
        self.assertEqual(result['denominator']['expected_plans'], 6)
        self.assertEqual(result['denominator']['missing_plan_records'], 1)
        self.assertEqual(result['shadow']['outputs'], 4)
        self.assertEqual(result['shadow']['no_output'], 2)
        self.assertAlmostEqual(result['shadow']['output_fraction_of_all_plans'], 4/6)

    def test_legacy_records_default_to_half_second_and_invalid_cadence_rejects(self):
        rows = fixture()
        rows[0]['config'].pop('interval_ns')
        result, _ = self.assess(rows)
        self.assertEqual(result['denominator']['interval_sim_s'], .5)
        self.assertEqual(result['denominator']['expected_plans'], 3)
        for interval in (0, -1, True, .1):
            with self.subTest(interval=interval):
                rows = fixture()
                rows[0]['config']['interval_ns'] = interval
                with self.assertRaises(ValueError):
                    self.assess(rows)

    def test_reanchoring_duplicate_outputs_fabricated_exposures_and_ages_fail_closed(self):
        for kind in ('gauge', 'duplicate_plan', 'duplicate_output', 'fake_exposure', 'age'):
            with self.subTest(kind=kind):
                rows = fixture()
                p = plans(rows)
                if kind == 'gauge':
                    p[1]['gauge_stamp_ns'] = NS//2
                elif kind == 'duplicate_plan':
                    rows.append(dict(p[0]))
                elif kind == 'duplicate_output':
                    p[1]['stamp_ns'] = 0
                elif kind == 'fake_exposure':
                    p[1]['stamp_ns'] = 123
                else:
                    p[1]['age_sim_s'] = .29
                with self.assertRaises(ValueError):
                    self.assess(rows)

    def test_cli_only_reads_inputs_and_writes_new_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            shadow, actual = root/'shadow.jsonl', root/'truth.jsonl'
            shadow.write_text(''.join(json.dumps(r)+'\n' for r in fixture()), encoding='utf-8')
            actual.write_text(''.join(json.dumps(r)+'\n' for r in truth()), encoding='utf-8')
            before = [evaluation.digest(p) for p in (shadow, actual)]
            output = root/'new'
            argv = ['evaluate_shadow', '--shadow', str(shadow), '--truth', str(actual),
                    '--imu-world-yaw-offset-rad', '0', '--output', str(output)]
            with patch.object(sys, 'argv', argv), patch('builtins.print'):
                evaluation.main()
            self.assertEqual(before, [evaluation.digest(p) for p in (shadow, actual)])
            result = json.loads((output/'summary.json').read_text())
            self.assertEqual(result['shadow']['outputs'], 3)
            self.assertTrue(json.loads((output/'integrity.json').read_text())['unchanged'])
            with patch.object(sys, 'argv', argv), patch('sys.stderr'):
                with self.assertRaises(SystemExit):
                    evaluation.main()


if __name__ == '__main__':
    unittest.main()
