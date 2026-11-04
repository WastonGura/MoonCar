"""Offline ownership, publication and failure contracts with a real solver."""
from collections import deque
from dataclasses import replace
from pathlib import Path
import gc
import pickle
import sys
import threading
from types import SimpleNamespace
import unittest
import weakref
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from offline_rgbd_window.async_processor import AsyncProcessor, CommitWorker
from offline_rgbd_window.core import Frame, Observation, WindowError
from offline_rgbd_window.query_cache import freeze_query_cache, prewarm_query_columns, query_cached


POINTS = np.random.default_rng(991).uniform([-1., -.7, 2.], [1., .7, 4.], (32, 3))


def sensor(number):
    return SimpleNamespace(stamp_ns=number*100_000_000, nominal_R_wb=np.eye(3),
                           R_bc=np.eye(3), t_bc=np.zeros(3))


def frame(number):
    return Frame(number*100_000_000, np.eye(3), np.eye(3), np.zeros(3),
        [Observation(i, p-[.002*number, 0., 0.], np.eye(3)*.004**2)
         for i, p in enumerate(POINTS)], np.eye(3)*.001**2)


class Frontend:
    def __init__(self):
        self.keyframes = deque(maxlen=6)

    def process(self, s):
        f = frame(s.stamp_ns//100_000_000)
        return dict(track_ids=np.arange(32), points=np.array([o.point_camera for o in f.observations]),
                    covariance=np.array([o.covariance_camera for o in f.observations]))

    def select(self, features):
        return np.arange(32), dict(adequate=True, selected=32, connected=32 if self.keyframes else 0)


class AsyncContracts(unittest.TestCase):
    def test_declining_overlap_commits_before_period_without_changing_gauge(self):
        w=self.worker();p=AsyncProcessor(frontend=Frontend(),worker=w)
        self.addCleanup(p.close)
        p.process(sensor(1),100_000_000);self.assertTrue(w.wait_idle())
        row=p.process(sensor(2),200_000_000)
        self.assertTrue(row['output'])
        self.assertEqual(row['frontend']['connected'],32)
        self.assertTrue(row['request_accepted'])
        self.assertEqual(row['keyframe_trigger'],'declining_overlap')
        self.assertEqual(row['stamp_ns'],200_000_000)
        self.assertEqual(row['gauge_stamp_ns'],100_000_000)
        self.assertTrue(w.wait_idle())
        self.assertEqual(w.poll()[0].cache.gauge_stamp_ns,100_000_000)

    def test_inadequate_support_never_bootstraps_a_new_map_or_keyframe(self):
        f=Frontend();w=self.worker();p=AsyncProcessor(frontend=f,worker=w)
        self.addCleanup(p.close)
        p.process(sensor(1),100_000_000);self.assertTrue(w.wait_idle())
        with patch.object(f,'select',return_value=(np.arange(23),dict(adequate=False,selected=23,connected=23))), \
                patch.object(w,'offer',side_effect=AssertionError('unsafe new map')):
            row=p.process(sensor(2),200_000_000)
        self.assertFalse(row['output']);self.assertFalse(row['keyframe_requested'])
        self.assertEqual(row['reason'],'insufficient_shared_or_spatial_track_support')
        self.assertEqual(p.gauge_stamp_ns,100_000_000)

    def test_overlap_request_is_rate_limited_and_does_not_duplicate_inflight_job(self):
        w=self.worker();p=AsyncProcessor(frontend=Frontend(),worker=w)
        self.addCleanup(p.close)
        p.process(sensor(1),100_000_000);self.assertTrue(w.wait_idle())
        with patch.object(w,'offer',side_effect=AssertionError('too soon')):
            row=p.process(sensor(1.5),150_000_000)
        self.assertFalse(row['keyframe_requested'])
        publication,status=w.poll();status=dict(status,running=True)
        with patch.object(w,'poll',return_value=(publication,status)), \
                patch.object(w,'offer',side_effect=AssertionError('duplicate commit')):
            row=p.process(sensor(2),200_000_000)
        self.assertTrue(row['output']);self.assertFalse(row['keyframe_requested'])

    def test_accepted_submission_ids_are_protected_and_completion_precedes_tracking(self):
        events = []
        class Observed(Frontend):
            def protect_submission(self,request,ids): events.append(('protect',request,tuple(ids)))
            def complete_submissions(self,request): events.append(('complete',request))
            def process(self,s):
                events.append(('track',s.stamp_ns))
                return super().process(s)
        w = self.worker()
        p = AsyncProcessor(frontend=Observed(),worker=w)
        self.addCleanup(p.close)
        p.process(sensor(1),100_000_000)
        self.assertTrue(w.wait_idle())
        self.assertEqual(events[1],('protect',1,tuple(range(32))))
        p.process(sensor(2),200_000_000)
        self.assertEqual(events[2],('complete',1))
        self.assertEqual(events[3],('track',200_000_000))

    def test_partial_prewarm_matches_cold_queries_after_marginalization(self):
        w = self.worker(prewarm_landmarks=0)
        for n in range(1, 10):
            cold = self.commit(w, n, n-1).cache
            warm = freeze_query_cache(w._estimator, n)
            mean = warm._mean.copy()
            result = prewarm_query_columns(warm, range(32), n, 16)
            self.assertEqual(result['added_columns'], 48)
            np.testing.assert_array_equal(warm._mean, mean)
            self.assertEqual((cold._information-warm._information).nnz, 0)
            a, b = query_cached(frame(n+1), cold, n), query_cached(frame(n+1), warm, n)
            np.testing.assert_allclose(a.position, b.position, rtol=0, atol=1e-12)
            np.testing.assert_allclose(a.covariance, b.covariance, rtol=0, atol=1e-16)

    def test_prewarm_is_bounded_and_nonfinite_solve_does_not_memoize(self):
        w = self.worker(prewarm_landmarks=0)
        cache = self.commit(w, 1, 0).cache
        consumed = []
        def ids():
            for n in range(1_000_000):
                consumed.append(n)
                yield n
        result = prewarm_query_columns(cache, ids(), 1, 7)
        self.assertEqual(consumed, list(range(7)))
        self.assertEqual(result['added_columns'], 21)
        self.assertEqual(prewarm_query_columns(cache, [9999, 1, 1, 8], 1, 4)['added_columns'], 3)
        saved = dict(cache._columns)
        class Nonfinite:
            def solve(self, selector):
                return np.full_like(selector, np.nan)
        broken = replace(cache, _decomposition=Nonfinite())
        with self.assertRaises(WindowError):
            prewarm_query_columns(broken, [9], 1)
        self.assertEqual(cache._columns.keys(), saved.keys())
        for version, budget in ((2, 8), (1, 97), (1, True)):
            with self.assertRaises(WindowError):
                prewarm_query_columns(cache, [9], version, budget)
        self.assertEqual(cache._columns.keys(), saved.keys())

    def test_prewarm_failure_is_transactional_and_recovers(self):
        w = self.worker()
        first = self.commit(w, 1, 0)
        saved = pickle.dumps(w._estimator.__dict__)
        with patch('offline_rgbd_window.async_processor.prewarm_query_columns',
                   side_effect=WindowError('injected prewarm failure')):
            self.commit(w, 2, 1)
        self.assertIs(w.poll()[0], first)
        self.assertEqual(pickle.dumps(w._estimator.__dict__), saved)
        self.assertFalse(w.poll()[1]['notice']['success'])
        recovered = self.commit(w, 3, 1)
        self.assertEqual(recovered.version, 2)
        self.assertEqual(recovered.cache.cached_columns, 96)

    def test_prewarm_runs_on_native_owner_before_publication(self):
        w = self.worker()
        seen = []
        def checked(cache, ids, version, budget):
            seen.append((threading.get_native_id(), w.poll()[1]['version']))
            return prewarm_query_columns(cache, ids, version, budget)
        with patch('offline_rgbd_window.async_processor.prewarm_query_columns', checked):
            publication = self.commit(w, 1, 0)
        self.assertEqual(seen, [(w._thread.native_id, 0)])
        self.assertEqual(publication.cache.cached_columns, 96)
        self.assertEqual(w.poll()[1]['notice']['prewarm']['added_columns'], 96)

    def worker(self, **kwargs):
        w = CommitWorker(**kwargs)
        self.addCleanup(w.close)
        return w

    def commit(self, w, number, version):
        self.assertTrue(w.offer(number, version, frame(number)))
        self.assertTrue(w.wait_idle())
        return w.poll()[0]

    def test_solve_and_freeze_are_one_transaction_and_next_commit_recovers(self):
        fail = False
        def freezer(*a, **kw):
            if fail:
                raise WindowError('injected freeze failure')
            return freeze_query_cache(*a, **kw)
        w = self.worker(freezer=freezer)
        first = self.commit(w, 1, 0)
        saved = pickle.dumps(w._estimator.__dict__)
        fail = True
        self.commit(w, 2, 1)
        self.assertIs(w.poll()[0], first)
        self.assertEqual(pickle.dumps(w._estimator.__dict__), saved)
        self.assertEqual(w.poll()[1]['version'], 1)
        self.assertFalse(w.poll()[1]['notice']['success'])
        fail = False
        self.assertEqual(self.commit(w, 3, 1).version, 2)

    def test_mutating_source_after_offer_cannot_mutate_publication(self):
        w = self.worker()
        source = frame(1)
        expected = source.observations[0].point_camera.copy()
        self.assertTrue(w.offer(1, 0, source))
        source.observations[0].point_camera[:] = 0.
        self.assertTrue(w.wait_idle())
        actual = w.poll()[0].cache._frames[0].observations[0].point_camera
        np.testing.assert_array_equal(actual, expected)
        self.assertFalse(actual.flags.writeable)

    def test_freeze_failure_after_full_window_marginalization_preserves_prior(self):
        w = self.worker()
        for n in range(1, 7):
            self.commit(w, n, n-1)
        saved = pickle.dumps(w._estimator.__dict__)
        old = w.poll()[0]
        w._freezer = lambda *a, **kw: (_ for _ in ()).throw(WindowError('freeze after Schur'))
        self.commit(w, 7, 6)
        self.assertEqual(pickle.dumps(w._estimator.__dict__), saved)
        self.assertIs(w.poll()[0], old)
        w._freezer = freeze_query_cache
        self.assertEqual(self.commit(w, 8, 6).version, 7)

    def test_publication_gauge_change_is_rejected_before_metadata_switch(self):
        w = self.worker()
        publication = self.commit(w, 1, 0)
        p = AsyncProcessor(frontend=Frontend(), worker=w)
        p._install(publication, 200_000_000)
        bad_cache = replace(publication.cache, gauge_stamp_ns=999)
        bad = replace(publication, version=2, cache=bad_cache)
        with self.assertRaisesRegex(WindowError, 'changed gauge'):
            p._install(bad, 300_000_000)
        self.assertEqual(p.cache_version, 1)

    def test_stale_offer_is_rejected_without_advancing_version(self):
        w = self.worker()
        self.commit(w, 1, 0)
        self.assertFalse(w.offer(2, 0, frame(2)))
        self.assertEqual(w.poll()[1]['counts']['offer_stale'], 1)

    def test_pending_replacement_and_obsolete_dependency_are_accounted(self):
        entered, release = threading.Event(), threading.Event()
        def freezer(*a, **kw):
            entered.set()
            self.assertTrue(release.wait(3))
            return freeze_query_cache(*a, **kw)
        w = self.worker(freezer=freezer)
        try:
            w.offer(1, 0, frame(1))
            self.assertTrue(entered.wait(3))
            w.offer(2, 0, frame(2)); w.offer(3, 0, frame(3))
            self.assertTrue(w.poll()[1]['pending'])
        finally:
            release.set()
        self.assertTrue(w.wait_idle())
        publication, status = w.poll()
        self.assertEqual(publication.request, 1)
        self.assertEqual(status['version'], 1)
        self.assertEqual(status['counts']['pending_replaced'], 1)
        self.assertEqual(status['counts']['rejected'], 1)
        self.assertIn('obsolete', status['notice']['reason'])

    def test_unexpected_backend_exception_is_latched_and_offers_stop(self):
        def broken(*a, **kw):
            raise RuntimeError('injected backend crash')
        w = self.worker(freezer=broken)
        self.commit(w, 1, 0)
        self.assertIn('backend_exception', w.poll()[1]['fault'])
        self.assertFalse(w.offer(2, 0, frame(2)))

    def test_close_cancels_pending_discards_running_result_and_reports_timeout(self):
        entered, release = threading.Event(), threading.Event()
        def freezer(*a, **kw):
            entered.set(); release.wait(3)
            return freeze_query_cache(*a, **kw)
        w = self.worker(freezer=freezer)
        try:
            w.offer(1, 0, frame(1)); self.assertTrue(entered.wait(3))
            w.offer(2, 0, frame(2))
            with self.assertRaisesRegex(RuntimeError, 'close timeout'):
                w.close(.01)
        finally:
            release.set(); w.close()
        self.assertIsNone(w.poll()[0])
        self.assertEqual(w.poll()[1]['counts']['pending_cancelled'], 1)
        self.assertFalse(w._thread.is_alive())

    def test_initialization_never_waits_for_stalled_backend_or_relabels_old_pose(self):
        entered, release = threading.Event(), threading.Event()
        def freezer(*a, **kw):
            entered.set(); release.wait(3)
            return freeze_query_cache(*a, **kw)
        w = self.worker(freezer=freezer)
        p = AsyncProcessor(frontend=Frontend(), worker=w)
        try:
            self.assertFalse(p.process(sensor(1), sensor(1).stamp_ns)['output'])
            self.assertTrue(entered.wait(3))
            with patch.object(w, 'wait_idle', side_effect=AssertionError('foreground waited')):
                row = p.process(sensor(2), sensor(2).stamp_ns)
            self.assertFalse(row['output'])
            self.assertEqual(row['stamp_ns'], 200_000_000)
            self.assertIsNone(p.query_cache)
        finally:
            release.set()
        self.assertTrue(w.wait_idle())
        row = p.process(sensor(3), sensor(3).stamp_ns)
        self.assertTrue(row['output'])
        self.assertEqual(row['stamp_ns'], 300_000_000)
        self.assertEqual(row['cache_age_s'], .2)
        self.assertEqual(row['gauge_stamp_ns'], 100_000_000)

    def test_expired_cache_is_recorded_as_no_output(self):
        w = self.worker(); self.commit(w, 1, 0)
        p = AsyncProcessor(frontend=Frontend(), worker=w)
        row = p.process(sensor(12), sensor(12).stamp_ns)
        self.assertFalse(row['output'])
        self.assertEqual(row['reason'], 'cache_expired')

    def test_future_and_same_exposure_cache_are_not_queried(self):
        w = self.worker(); self.commit(w, 5, 0)
        p = AsyncProcessor(frontend=Frontend(), worker=w)
        publication = w.poll()[0]
        p._install(publication, 400_000_000)
        self.assertIsNone(p.query_cache)
        p._install(publication, 500_000_000)
        self.assertIsNone(p.query_cache)
        p._install(publication, 600_000_000)
        self.assertEqual(p.cache_version, 1)

    def test_foreground_same_snapshot_query_is_exact_and_metadata_is_consistent(self):
        w = self.worker(); self.commit(w, 1, 0)
        seen = []
        def checked(f, cache, version):
            clone = replace(cache, _columns=dict(cache._columns), _lock=threading.RLock())
            expected = query_cached(f, clone, version)
            actual = query_cached(f, cache, version)
            np.testing.assert_array_equal(actual.position, expected.position)
            np.testing.assert_array_equal(actual.covariance, expected.covariance)
            seen.append((f.stamp_ns, version))
            return actual
        p = AsyncProcessor(frontend=Frontend(), worker=w, query=checked)
        row = p.process(sensor(2), 200_000_000)
        self.assertTrue(row['output'])
        self.assertEqual(seen, [(200_000_000, 1)])
        np.testing.assert_array_equal(p.frontend.keyframes[0][0]['submitted_ids'], np.arange(32))
        with self.assertRaisesRegex(WindowError, 'must increase'):
            p.process(sensor(2), 200_000_000)

    def test_rejected_update_is_visible_while_valid_old_snapshot_can_be_queried(self):
        w = self.worker(); self.commit(w, 1, 0)
        w._freezer = lambda *a, **kw: (_ for _ in ()).throw(WindowError('freeze failed'))
        self.commit(w, 2, 1)
        p = AsyncProcessor(frontend=Frontend(), worker=w)
        row = p.process(sensor(3), 300_000_000)
        self.assertTrue(row['output'])
        self.assertEqual(row['cache_version'], 1)
        self.assertFalse(row['backend_status']['notice']['success'])

    def test_successive_commits_keep_both_windows_bounded(self):
        w = self.worker()
        for n in range(1, 10):
            publication = self.commit(w, n, n-1)
            self.assertLessEqual(len(publication.cache._frames), 6)
            self.assertLessEqual(len(w._estimator._frames), 6)
            self.assertLessEqual(len(w._estimator._frame_terms), 6)

    def test_replaced_cache_is_released_after_foreground_switch(self):
        w = self.worker()
        publication = self.commit(w, 1, 0)
        p = AsyncProcessor(frontend=Frontend(), worker=w)
        p._install(publication, 200_000_000)
        old = weakref.ref(publication.cache)
        new = self.commit(w, 3, 1)
        p._install(new, 400_000_000)
        del publication
        self.assertTrue(w.wait_idle())
        gc.collect()
        self.assertIsNone(old())

    def test_backend_fault_stops_foreground_output_even_with_valid_old_cache(self):
        w = self.worker(); self.commit(w, 1, 0)
        def broken(*a, **kw):
            raise RuntimeError('backend crash')
        w._freezer = broken
        self.commit(w, 2, 1)
        p = AsyncProcessor(frontend=Frontend(), worker=w)
        row = p.process(sensor(3), 300_000_000)
        self.assertFalse(row['output'])
        self.assertEqual(row['reason'], 'backend_unavailable')

    def test_native_factor_final_release_happens_on_creator_thread(self):
        releases = []
        class Traced:
            def __init__(self, factor):
                self.factor = factor
                self.creator = threading.get_native_id()
            def __getattr__(self, name):
                return getattr(self.factor, name)
            def __del__(self):
                releases.append((self.creator, threading.get_native_id()))
        def freeze(*a, **kw):
            cache = freeze_query_cache(*a, **kw)
            return replace(cache, _decomposition=Traced(cache._decomposition))
        w = self.worker(freezer=freeze)
        p = AsyncProcessor(frontend=Frontend(), worker=w)
        # Drive submissions through the actual foreground: offering with a
        # newer base version promises that older leases were released.
        for n in range(1, 21):
            row = p.process(sensor(n), n*100_000_000)
            self.assertEqual(row['output'], n != 1)
            self.assertTrue(w.wait_idle())
            self.assertLessEqual(w.poll()[1]['owned_versions'], 2)
        committed = w.poll()[1]['counts']['committed']
        p.close()
        self.assertEqual(len(releases), committed)
        self.assertTrue(all(creator == destructor for creator, destructor in releases))


if __name__ == '__main__':
    unittest.main()
