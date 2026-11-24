"""Focused live-adapter contracts; no world, navigation, or recorded truth."""
import ast
from collections import Counter
from dataclasses import dataclass
import json
import pickle
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from offline_rgbd_window.online import (JsonlWriter, LatestMailbox, PlanScheduler,
    ShadowProcessor, make_sensor_frame, transfer_pending_plan)
from offline_rgbd_window.core import WindowError, WindowEstimator

NODE_TREE = ast.parse((ROOT/'scripts/rgbd_window_shadow.py').read_text(encoding='utf-8'))


def extracted(name, namespace, method=True):
    owner = next(n for n in NODE_TREE.body if isinstance(n, ast.ClassDef) and n.name == 'ShadowNode') if method else NODE_TREE
    node = next(n for n in owner.body if isinstance(n, ast.FunctionDef) and n.name == name)
    scope = dict(namespace)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), '<real-shadow-method>', 'exec'), scope)
    return scope[name]


def sensor(stamp=1_000_000_000):
    return make_sensor_frame(stamp, np.zeros((2, 2), np.uint8),
        np.full((2, 2, 3), 1.), np.eye(3), np.eye(3), np.zeros(3), np.eye(3),
        lambda p: np.ones(len(p), bool))


def frontend():
    features = dict(track_ids=np.array([7]), points=np.array([[1., 2., 3.]]),
                    covariance=np.array([np.diag([.01, .02, .03])]))
    f = Mock()
    f.process.return_value = features
    f.select.return_value = (np.array([0]), dict(adequate=True, connected=24))
    return f


class ShadowAdapterTests(unittest.TestCase):
    def test_optional_phase_diagnostics_preserve_real_cached_results_and_committed_state(self):
        rng = np.random.default_rng(981)
        features = dict(track_ids=np.arange(12),
            points=np.column_stack((rng.uniform(-1., 1., 12), rng.uniform(-.8, .8, 12),
                                    rng.uniform(2., 4., 12))),
            covariance=np.repeat((.004**2*np.eye(3))[None], 12, axis=0))
        processors = []
        for enabled in (False, True):
            f = Mock()
            f.process.return_value = features
            f.select.return_value = (np.arange(12), dict(adequate=True, connected=12))
            processors.append(ShadowProcessor(f, WindowEstimator(6), cached_query=True,
                                             timing_diagnostics=enabled))
        for stamp in range(1_000_000_000, 1_700_000_000, 100_000_000):
            rows = [p.process(sensor(stamp), stamp, 10, 20, lambda: stamp+20_000_000,
                              lambda: 30) for p in processors]
            self.assertTrue(all(row['output'] for row in rows))
            self.assertNotIn('timing_phases', rows[0])
            self.assertEqual(pickle.dumps(processors[0].estimator.__dict__),
                             pickle.dumps(processors[1].estimator.__dict__))
            for key in ('position', 'covariance_conditional', 'reason', 'stamp_ns',
                        'gauge_stamp_ns', 'keyframe', 'cache_version', 'finished_wall_ns',
                        'ready_sim_ns', 'age_sim_s'):
                self.assertEqual(rows[0][key], rows[1][key], key)
            phases = rows[1]['timing_phases']
            self.assertTrue({'frontend', 'selection', 'observation_build', 'solve', 'plain'} <= phases.keys())
            self.assertEqual('freeze' in phases, rows[1]['keyframe'])
            for measured in phases.values():
                self.assertEqual(set(measured), {'wall_s', 'thread_cpu_s', 'process_cpu_s'})
                self.assertTrue(all(value >= 0. for value in measured.values()))
        with self.assertRaises(ValueError):
            ShadowProcessor(timing_diagnostics=1)

    def test_cache_is_opt_in_and_default_path_does_not_construct_or_query_it(self):
        f, e = frontend(), Mock()
        answer = SimpleNamespace(position=np.zeros(3), covariance=.004*np.eye(3), diagnostics={})
        e.add_frame.return_value = e.query_frame.return_value = answer
        with patch('offline_rgbd_window.online.freeze_query_cache') as build, \
                patch('offline_rgbd_window.online.query_cached') as query:
            processor = ShadowProcessor(f, e)
            processor.process(sensor(), 1_000_000_000, 1, 2, lambda: 1_020_000_000)
            row = processor.process(sensor(1_100_000_000), 1_100_000_000, 1, 2,
                                    lambda: 1_120_000_000)
        self.assertEqual(row['estimation_mode'], 'joint_query')
        self.assertNotIn('cache_version', row)
        self.assertEqual(e.query_frame.call_count, 1)
        e.add_frame_with_posterior.assert_not_called()
        build.assert_not_called()
        query.assert_not_called()
        with self.assertRaises(ValueError):
            ShadowProcessor(f, e, cached_query=1)
        main = next(n for n in NODE_TREE.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        flag = next(n for n in ast.walk(main) if isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Attribute) and n.func.attr == 'add_argument'
                    and n.args and isinstance(n.args[0], ast.Constant)
                    and n.args[0].value == '--cached-query')
        self.assertEqual(next(k.value.value for k in flag.keywords if k.arg == 'action'), 'store_true')

    def test_cached_keyframes_replace_version_and_queries_keep_actual_stamp_without_commits(self):
        f, e = frontend(), Mock()
        answer = SimpleNamespace(position=np.zeros(3), covariance=.004*np.eye(3), diagnostics={})
        posterior = object()
        e.add_frame_with_posterior.return_value = answer, posterior
        old, new = object(), object()
        with patch('offline_rgbd_window.online.freeze_query_cache', side_effect=[old, new]) as build, \
                patch('offline_rgbd_window.online.query_cached', return_value=answer) as query:
            processor = ShadowProcessor(f, e, cached_query=True)
            first = processor.process(sensor(), 1_000_000_000, 1, 2, lambda: 1_020_000_000)
            measured = processor.process(sensor(1_100_000_000), 1_100_000_000, 3, 4,
                                         lambda: 1_420_000_000)
            second = processor.process(sensor(1_500_000_000), 1_500_000_000, 5, 6,
                                       lambda: 1_520_000_000)
            processor.process(sensor(1_600_000_000), 1_600_000_000, 7, 8, lambda: 1_620_000_000)
        self.assertTrue(first['output'] and second['output'])
        self.assertEqual((first['cache_version'], second['cache_version']), (1, 2))
        self.assertTrue(first['cache_available'] and second['cache_available'])
        self.assertEqual([call.args for call in build.call_args_list], [(e, 1), (e, 2)])
        self.assertTrue(all(call.kwargs == dict(posterior=posterior)
                            for call in build.call_args_list))
        self.assertEqual(e.add_frame_with_posterior.call_count, 2)
        e.add_frame.assert_not_called()
        self.assertIs(query.call_args_list[0].args[1], old)
        self.assertIs(query.call_args_list[1].args[1], new)
        self.assertEqual([call.args[2] for call in query.call_args_list], [1, 2])
        self.assertEqual(measured['estimation_mode'], 'cached_query')
        self.assertEqual(measured['stamp_ns'], 1_100_000_000)
        self.assertAlmostEqual(measured['age_sim_s'], .32)
        self.assertEqual(f.commit.call_count, 2)
        e.query_frame.assert_not_called()

    def test_failed_keyframe_discards_old_cache_and_never_reuses_or_falls_back(self):
        f, e = frontend(), Mock()
        answer = SimpleNamespace(position=np.zeros(3), covariance=.004*np.eye(3), diagnostics={})
        e.add_frame_with_posterior.side_effect = [(answer, object()),
            WindowError('rejected keyframe'), WindowError('still rejected')]
        with patch('offline_rgbd_window.online.freeze_query_cache', return_value=object()) as build, \
                patch('offline_rgbd_window.online.query_cached') as query:
            processor = ShadowProcessor(f, e, cached_query=True)
            processor.process(sensor(), 1_000_000_000, 1, 2, lambda: 1_020_000_000)
            failed = processor.process(sensor(1_500_000_000), 1_500_000_000, 1, 2,
                                       lambda: 1_520_000_000)
            retry = processor.process(sensor(1_600_000_000), 1_600_000_000, 1, 2,
                                      lambda: 1_620_000_000)
        self.assertFalse(failed['output'] or retry['output'])
        self.assertFalse(failed['cache_available'])
        self.assertIsNone(processor.query_cache)
        self.assertEqual(processor.cache_version, 1)
        self.assertEqual(processor.last_keyframe_stamp_ns, 1_000_000_000)
        self.assertEqual(build.call_count, 1)
        self.assertEqual(f.commit.call_count, 1)
        query.assert_not_called()
        e.query_frame.assert_not_called()

    def test_failed_build_records_committed_version_and_cache_unavailable_until_next_keyframe(self):
        f, e = frontend(), Mock()
        answer = SimpleNamespace(position=np.zeros(3), covariance=.004*np.eye(3), diagnostics={})
        e.add_frame_with_posterior.return_value = answer, object()
        with patch('offline_rgbd_window.online.freeze_query_cache',
                   side_effect=[object(), WindowError('failed cache build'), object()]) as build, \
                patch('offline_rgbd_window.online.query_cached') as query:
            processor = ShadowProcessor(f, e, cached_query=True)
            processor.process(sensor(), 1_000_000_000, 1, 2, lambda: 1_020_000_000)
            failed = processor.process(sensor(1_500_000_000), 1_500_000_000, 1, 2,
                                       lambda: 1_520_000_000)
            unavailable = processor.process(sensor(1_600_000_000), 1_600_000_000, 1, 2,
                                            lambda: 1_620_000_000)
            recovered = processor.process(sensor(2_000_000_000), 2_000_000_000, 1, 2,
                                          lambda: 2_020_000_000)
        self.assertFalse(failed['output'] or unavailable['output'])
        self.assertEqual(failed['cache_version'], 2)
        self.assertFalse(failed['cache_available'])
        self.assertEqual(unavailable['estimation_mode'], 'cached_query')
        self.assertIn('query cache unavailable', unavailable['reason'])
        self.assertNotIn('position', unavailable)
        self.assertTrue(recovered['output'] and recovered['cache_available'])
        self.assertEqual(recovered['cache_version'], 3)
        self.assertEqual(f.commit.call_count, 3)
        self.assertEqual([call.args[1] for call in build.call_args_list], [1, 2, 3])
        query.assert_not_called()
        e.query_frame.assert_not_called()

    def test_due_keyframe_with_insufficient_features_also_invalidates_preceding_cache(self):
        f, e = frontend(), Mock()
        e.add_frame_with_posterior.return_value = (SimpleNamespace(position=np.zeros(3),
            covariance=.004*np.eye(3), diagnostics={}), object())
        with patch('offline_rgbd_window.online.freeze_query_cache', return_value=object()) as build:
            processor = ShadowProcessor(f, e, cached_query=True)
            processor.process(sensor(), 1_000_000_000, 1, 2, lambda: 1_020_000_000)
            f.select.return_value = (np.array([0]), dict(adequate=False, connected=1))
            failed = processor.process(sensor(1_500_000_000), 1_500_000_000, 1, 2,
                                       lambda: 1_520_000_000)
        self.assertFalse(failed['output'] or failed['cache_available'])
        self.assertEqual(failed['reason'], 'insufficient_shared_or_spatial_track_support')
        self.assertIsNone(processor.query_cache)
        self.assertEqual(processor.last_keyframe_stamp_ns, 1_000_000_000)
        self.assertEqual(build.call_count, 1)
        self.assertEqual(e.add_frame_with_posterior.call_count, 1)
        self.assertEqual(f.commit.call_count, 1)

    def test_cache_build_time_is_inside_backend_time_including_failure(self):
        for build_result in (object(), WindowError('failed build')):
            with self.subTest(failed=isinstance(build_result, Exception)):
                f, e = frontend(), Mock()
                e.add_frame_with_posterior.return_value = (SimpleNamespace(position=np.zeros(3),
                    covariance=.004*np.eye(3), diagnostics={}), object())
                outcome = dict(side_effect=build_result) if isinstance(build_result, Exception) else dict(return_value=build_result)
                with patch('offline_rgbd_window.online.freeze_query_cache', **outcome), \
                        patch('offline_rgbd_window.online.time.perf_counter',
                              side_effect=[0., 1., 2., 3., 5., 8.]):
                    processor = ShadowProcessor(f, e, cached_query=True)
                    row = processor.process(sensor(), 1_000_000_000, 1, 2, lambda: 1_020_000_000)
                self.assertEqual(row['cache_build_s'], 2.)
                self.assertEqual(row['backend_s'], 6.)
                self.assertEqual(row['output'], not isinstance(build_result, Exception))

    def test_real_cached_query_failure_keeps_committed_model_and_memo_transactional(self):
        rng = np.random.default_rng(2193)
        points = np.column_stack((rng.uniform(-1., 1., 12), rng.uniform(-.7, .7, 12),
                                  rng.uniform(2., 4., 12)))
        features = dict(track_ids=np.arange(12), points=points,
                        covariance=np.repeat((.004**2*np.eye(3))[None], 12, axis=0))
        f = Mock()
        f.process.return_value = features
        f.select.return_value = (np.arange(12), dict(adequate=True, connected=12))
        e = WindowEstimator(6)
        processor = ShadowProcessor(f, e, cached_query=True)
        self.assertTrue(processor.process(sensor(), 1_000_000_000, 1, 2,
                                        lambda: 1_020_000_000)['output'])
        cache = processor.query_cache
        self.assertEqual(cache.cached_columns, 0)
        frames = tuple(e._frames)
        with patch.object(e, 'query_frame', side_effect=AssertionError('legacy fallback')), \
                patch('offline_rgbd_window.query_cache._batch_residuals',
                      return_value=np.full((12, 3), np.nan)):
            failed = processor.process(sensor(1_100_000_000), 1_100_000_000, 1, 2,
                                       lambda: 1_120_000_000)
        self.assertFalse(failed['output'])
        self.assertIn('nonfinite back-substituted query state', failed['reason'])
        self.assertEqual(cache.cached_columns, 0)
        self.assertIs(processor.query_cache, cache)
        self.assertEqual(tuple(e._frames), frames)
        self.assertEqual(processor.cache_version, 1)
        self.assertEqual(f.commit.call_count, 1)
        recovered = processor.process(sensor(1_200_000_000), 1_200_000_000, 1, 2,
                                      lambda: 1_220_000_000)
        self.assertTrue(recovered['output'])
        self.assertEqual(cache.cached_columns, 36)
        self.assertEqual(f.commit.call_count, 1)

    def test_live_clock_ahead_of_sensor_delivery_does_not_consume_real_late_plans(self):
        scheduler = PlanScheduler(interval_ns=100_000_000)
        f, e = frontend(), Mock()
        answer = SimpleNamespace(position=np.array([.1, .2, .3]),
                                 covariance=.004*np.eye(3), diagnostics={})
        e.add_frame.return_value = e.query_frame.return_value = answer
        processor = ShadowProcessor(f, e)
        planned, missing = scheduler.observe(1_000_000_000)
        self.assertEqual(missing, [])
        self.assertTrue(processor.process(sensor(), planned, 1, 2, lambda: 1_020_000_000)['output'])
        node = SimpleNamespace(scheduler=scheduler, guard_clock=Mock(return_value=1_420_000_000),
                               fail_plan=Mock(), lock=threading.RLock())
        tick = extracted('tick', {})
        for _ in range(3):
            tick(node)
        self.assertEqual(node.guard_clock.call_count, 3)
        node.fail_plan.assert_not_called()
        self.assertEqual(scheduler.next_due_ns, 1_100_000_000)
        # The genuine late exposure gets a new solve, not a stamped copy of
        # the previous pose. Its actual .32s age remains visible to evaluation.
        planned, missing = scheduler.observe(1_100_000_000)
        self.assertEqual((planned, missing), (1_100_000_000, []))
        row = processor.process(sensor(1_100_000_000), planned, 3, 4, lambda: 1_420_000_000)
        self.assertTrue(row['output'])
        self.assertEqual(row['stamp_ns'], 1_100_000_000)
        self.assertEqual(row['planned_stamp_ns'], 1_100_000_000)
        self.assertEqual(row['ready_sim_ns'], 1_420_000_000)
        self.assertAlmostEqual(row['age_sim_s'], .32)
        self.assertEqual(e.query_frame.call_count, 1)
        self.assertEqual(e.query_frame.call_args.args[0].stamp_ns, 1_100_000_000)

    def test_real_exposure_watermark_and_closed_tail_keep_every_plan_once(self):
        scheduler = PlanScheduler(interval_ns=100_000_000)
        terminals = []
        for stamp in (1_000_000_000, 1_100_000_000, 1_400_000_000):
            planned, missing = scheduler.observe(stamp)
            terminals.extend(missing)
            terminals.append(planned)
            if stamp == 1_400_000_000:
                self.assertEqual(missing, [1_200_000_000, 1_300_000_000])
        tail = scheduler.expire(1_650_000_000, final=True)
        self.assertEqual(tail, [1_500_000_000, 1_600_000_000])
        terminals.extend(tail)
        self.assertEqual(terminals, list(range(1_000_000_000, 1_700_000_000, 100_000_000)))
        self.assertEqual(len(terminals), len(set(terminals)))
        self.assertEqual(scheduler.expire(1_650_000_000, final=True), [])

    def test_current_measured_queries_keep_keyframes_and_do_not_double_commit(self):
        f, e = frontend(), Mock()
        answer = SimpleNamespace(position=np.array([.1,.2,.3]), covariance=.004*np.eye(3), diagnostics={})
        e.add_frame.return_value = answer
        e.query_frame.return_value = answer
        p = ShadowProcessor(f,e)
        first=p.process(sensor(1_000_000_000),1_000_000_000,1,2,lambda:1_020_000_000)
        query=p.process(sensor(1_100_000_000),1_100_000_000,3,4,lambda:1_120_000_000)
        self.assertTrue(first['keyframe'])
        self.assertFalse(query['keyframe'])
        self.assertEqual(query['estimation_mode'],'joint_query')
        self.assertEqual(query['stamp_ns'],1_100_000_000)
        self.assertEqual(query['gauge_stamp_ns'],1_000_000_000)
        self.assertAlmostEqual(query['age_sim_s'],.02)
        self.assertEqual(e.add_frame.call_count,1)
        self.assertEqual(e.query_frame.call_count,1)
        self.assertEqual(f.commit.call_count,1)
        np.testing.assert_equal(query['covariance_conditional'],answer.covariance)
        e.query_frame.side_effect=WindowError('query support failed')
        failed=p.process(sensor(1_200_000_000),1_200_000_000,5,6,lambda:1_220_000_000)
        self.assertFalse(failed['output'])
        self.assertEqual(p.last_keyframe_stamp_ns,1_000_000_000)
        self.assertEqual(f.commit.call_count,1)
        committed=p.process(sensor(1_500_000_000),1_500_000_000,7,8,lambda:1_520_000_000)
        self.assertTrue(committed['keyframe'])
        self.assertEqual(e.add_frame.call_count,2)
        self.assertEqual(f.commit.call_count,2)
        self.assertEqual(p.last_keyframe_stamp_ns,1_500_000_000)

    def test_pending_plan_transfer_uses_latest_real_stamp_in_same_slot_only(self):
        @dataclass(frozen=True)
        class Pending:
            stamp_ns:int
            planned_stamp_ns:int|None
            received_wall_ns:int
        old=Pending(1_000,1_000,10)
        raw=Pending(1_050,None,20)
        latest,inherited=transfer_pending_plan(old,raw,100)
        self.assertTrue(inherited)
        self.assertEqual(latest,Pending(1_050,1_000,20))
        self.assertIsNone(raw.planned_stamp_ns)
        self.assertEqual(old,Pending(1_000,1_000,10))
        for other in (Pending(1_100,None,30),Pending(1_150,1_100,40)):
            latest,inherited=transfer_pending_plan(old,other,100)
            self.assertFalse(inherited)
            self.assertIs(latest,other)

    def test_inflight_plan_cannot_be_transferred_or_completed_twice(self):
        @dataclass(frozen=True)
        class Pending:
            stamp_ns:int
            planned_stamp_ns:int|None
        box=LatestMailbox();first=Pending(1_000,1_000)
        box.offer(first)
        self.assertIs(box.take(),first)
        with box.condition:
            latest,inherited=transfer_pending_plan(box.latest,Pending(1_050,None),100)
            self.assertFalse(inherited)
            self.assertIsNone(box.offer(latest))
        self.assertIsNone(box.take().planned_stamp_ns)
        self.assertIsNone(box.close())

    def test_fixed_plan_denominator_includes_missing_slots_without_reusing_exposure(self):
        s = PlanScheduler()
        self.assertEqual(s.observe(100_000_000), (100_000_000, []))
        self.assertEqual(s.observe(200_000_000), (None, []))
        self.assertEqual(s.observe(1_700_000_000), (1_600_000_000, [600_000_000, 1_100_000_000]))
        self.assertEqual(s.expire(2_500_000_000), [])
        self.assertEqual(s.expire(2_600_000_000), [2_100_000_000])
        self.assertEqual(s.expire(2_600_000_000, final=True), [2_600_000_000])
        self.assertEqual(s.expire(2_600_000_000, final=True), [])
        with self.assertRaises(ValueError):
            s.observe(1_700_000_000)

    def test_latest_mailbox_replaces_one_item_and_close_wakes_worker(self):
        box = LatestMailbox()
        self.assertIsNone(box.offer('a'))
        self.assertEqual(box.offer('b'), 'a')
        self.assertEqual(box.take(), 'b')
        returned = []
        worker = threading.Thread(target=lambda: returned.append(box.take(timeout=2.)))
        worker.start()
        self.assertIsNone(box.close())
        worker.join(timeout=1.)
        self.assertFalse(worker.is_alive())
        self.assertEqual(returned, [None])
        with self.assertRaises(RuntimeError):
            box.offer('c')

    def test_dynamic_extrinsic_and_real_depth_mask_are_not_pose_inputs(self):
        optical = np.array([[[1., 0., 2.], [2., 0., 3.]], [[np.nan]*3, [0., 1., 4.]]])
        Rwb, Rbc = Rotation.from_euler('z', 25, degrees=True).as_matrix(), Rotation.from_euler('y', 10, degrees=True).as_matrix()
        lever = np.array([.2, -.1, .3])
        row = make_sensor_frame(123, np.zeros((2, 2), np.uint8), optical, Rwb, Rbc, lever,
            np.diag([100., 100., 1.]), lambda p: np.array([True, False, True]))
        np.testing.assert_allclose(row.xyz_grid[0, 0], Rwb@(Rbc@optical[0, 0]+lever))
        self.assertTrue(np.isnan(row.xyz_grid[0, 1]).all())
        self.assertTrue(np.isnan(row.xyz_grid[1, 0]).all())
        np.testing.assert_equal(row.depth_raw, optical[:, :, 2])
        self.assertIsNone(row.wheel_vx)

    def test_slow_measured_output_is_retained_and_original_exposure_stamp_is_preserved(self):
        f, e = frontend(), Mock()
        e.add_frame.return_value = SimpleNamespace(position=np.array([.1, .2, .3]), covariance=.004*np.eye(3), diagnostics={'conditional': True})
        p = ShadowProcessor(f, e)
        row = p.process(sensor(), 900_000_000, 10, 20, lambda: 2_000_000_000, lambda: 30)
        self.assertTrue(row['output'])
        self.assertEqual(row['stamp_ns'], 1_000_000_000)
        self.assertEqual(row['planned_stamp_ns'], 900_000_000)
        self.assertEqual(row['age_sim_s'], 1.)
        self.assertEqual(row['gauge_stamp_ns'], 1_000_000_000)
        np.testing.assert_equal(row['covariance_conditional'], .004*np.eye(3))
        self.assertNotIn('baseline', row)
        frame = e.add_frame.call_args.args[0]
        np.testing.assert_allclose(frame.observations[0].point_camera, [1., 2., 3.])
        self.assertEqual(f.commit.call_count, 1)

    def test_frontend_only_and_failed_plan_do_not_commit_or_reset_estimator(self):
        f, e = frontend(), Mock()
        p = ShadowProcessor(f, e)
        self.assertIsNone(p.process(sensor(), None, 1, 2, lambda: 1_100_000_000))
        e.add_frame.assert_not_called()
        e.add_frame.side_effect = WindowError('actual shared support missing')
        row = p.process(sensor(), 1_000_000_000, 1, 2, lambda: 1_500_000_000)
        self.assertFalse(row['output'])
        self.assertIn('window_error', row['reason'])
        self.assertIsNone(p.gauge_stamp_ns)
        self.assertIs(p.estimator, e)
        f.commit.assert_not_called()
        self.assertNotIn('position', row)

    def test_jsonl_epoch_exclusive_output_and_no_nan(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'shadow.jsonl'
            w = JsonlWriter(path, 'test-epoch')
            w.write(dict(type='plan', position=np.ones(3), output=True))
            with self.assertRaises(ValueError):
                w.write(dict(value=np.nan))
            w.close()
            self.assertEqual(json.loads(path.read_text())['epoch'], 'test-epoch')
            with self.assertRaises(FileExistsError):
                JsonlWriter(path, 'another-epoch')

    def test_sensor_only_subscriptions_and_exact_exposure_transform(self):
        calls = [n for n in ast.walk(NODE_TREE) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
        subscriptions = [n for n in calls if n.func.attr == 'create_subscription']
        self.assertEqual({n.args[1].value for n in subscriptions}, {
            '/mooncar/localization_rgb/image', '/mooncar/depth/image',
            '/mooncar/localization_rgb/camera_info', '/mooncar/depth/camera_info', '/mooncar/imu'})
        imu = next(n for n in subscriptions if n.args[1].value == '/mooncar/imu')
        self.assertEqual(ast.unparse(imu.args[3]), 'imu_qos')
        self.assertFalse({'create_publisher', 'publish', 'send_goal_async', 'sendTransform'} & {n.func.attr for n in calls})
        prepare = next(n for n in next(n for n in NODE_TREE.body if isinstance(n, ast.ClassDef) and n.name == 'ShadowNode').body if isinstance(n, ast.FunctionDef) and n.name == 'prepare')
        tf = next(n for n in ast.walk(prepare) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'lookup_transform')
        self.assertEqual(ast.unparse(tf.args[2]), 'Time(nanoseconds=pair.stamp_ns)')

    def test_finish_joins_worker_records_pending_failure_and_complete_run_end(self):
        events, failures = [], []
        box = LatestMailbox()
        pair = SimpleNamespace(planned_stamp_ns=10, stamp_ns=10, received_wall_ns=100)
        box.offer(pair)
        writer, worker = Mock(), Mock()
        worker.is_alive.return_value = False
        stub = SimpleNamespace(lock=threading.RLock(), closing=False, mailbox=box,
            exposure=lambda *a: events.append(('exposure', a)), fail_plan=lambda *a: failures.append(a),
            worker=worker, event=lambda **row: events.append(row), sim_now=lambda: 10,
            scheduler=PlanScheduler(), counts=Counter(), fault=None, writer=writer,
            processor=SimpleNamespace(gauge_stamp_ns=None), initial_imu_yaw_rad=.2,
            images=dict(rgb={}, depth={}))
        finish = extracted('finish', {'time': __import__('time')})
        finish(stub)
        worker.join.assert_called_once_with(timeout=10.)
        self.assertEqual(len(failures), 1)
        self.assertEqual(failures[0][2], 'shutdown_before_processing')
        self.assertEqual(events[-1]['type'], 'run_end')
        self.assertTrue(events[-1]['worker_terminal'])
        writer.close.assert_called_once()

    def test_incomplete_worker_never_claims_run_end(self):
        events = []
        worker = Mock()
        worker.is_alive.return_value = True
        stub = SimpleNamespace(lock=threading.RLock(), closing=False, mailbox=LatestMailbox(),
            worker=worker, event=lambda **row: events.append(row), sim_now=lambda: 10)
        with self.assertRaises(RuntimeError):
            extracted('finish', {'time': __import__('time')})(stub)
        self.assertEqual([e['type'] for e in events], ['shutdown_incomplete'])

    def test_ctrl_c_or_external_rclpy_shutdown_still_finishes_and_destroys(self):
        class Shutdown(Exception):
            pass
        for interruption in (KeyboardInterrupt, Shutdown):
            with self.subTest(interruption=interruption):
                ros, node = Mock(), Mock()
                ros.spin.side_effect = interruption()
                ros.ok.return_value = False  # SIGINT may already have shut the context down.
                main = extracted('main', dict(argparse=__import__('argparse'), Path=Path,
                    rclpy=ros, ShadowNode=lambda args: node, ExternalShutdownException=Shutdown,
                    __doc__='test'), method=False)
                from unittest.mock import patch
                with patch.object(sys, 'argv', ['shadow', '--output', 'unused.jsonl']):
                    main()
                node.finish.assert_called_once()
                node.destroy_node.assert_called_once()


if __name__ == '__main__':
    unittest.main()
