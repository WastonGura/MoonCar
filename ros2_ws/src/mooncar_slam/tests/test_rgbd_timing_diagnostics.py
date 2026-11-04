"""Real isolated ROS worker timing; no Gazebo, API or sensor-motion claim."""
import json
import threading
import time
import unittest

import test_rgbd_worker as worker


class RGBDTimingDiagnosticsTests(unittest.TestCase):
    clock=worker.RGBDWorkerTests.clock
    synchronized=worker.RGBDWorkerTests.synchronized
    ready=worker.RGBDWorkerTests.ready
    color=worker.RGBDWorkerTests.color
    depth=worker.RGBDWorkerTests.depth
    no_pose=worker.RGBDWorkerTests.no_pose
    seed_healthy_quality=worker.RGBDWorkerTests.seed_healthy_quality
    wait_until=worker.RGBDWorkerTests.wait_until
    start_worker=worker.RGBDWorkerTests.start_worker
    block_first_fit=worker.RGBDWorkerTests.block_first_fit
    queue=worker.RGBDWorkerTests.queue

    def setUp(self):
        worker.RGBDWorkerTests.setUp(self)

    def tearDown(self):
        worker.RGBDWorkerTests.tearDown(self)

    def test_async_blocked_fit_records_wall_and_worker_cpu_without_pose_refresh(self):
        self.block_first_fit()
        self.no_pose()
        self.assertEqual(self.node.quality.accepted_stamp,1.9)
        threading.Event().wait(.04)
        self.release.set()
        self.wait_until(lambda:not self.node.processing_inflight)
        timing=self.node.runtime_diagnostics()['last_per_exposure_timing']
        self.assertEqual(timing['exposure_stamp'],2.)
        self.assertEqual(timing['stage'],'result_observed')
        self.assertAlmostEqual(timing['prepared_sim_age_s'],.02)
        self.assertAlmostEqual(timing['start_sim_age_s'],.02)
        self.assertGreaterEqual(timing['core_wall_s'],.04)
        self.assertGreater(timing['core_wall_s'],timing['worker_thread_cpu_s'])
        for key in ('decode_wall_s','worker_thread_cpu_s','end_lock_wait_wall_s',
                    'input_wall_age_start_s','ready_wait_wall_s'):
            self.assertGreaterEqual(timing[key],0.,key)
        self.assertAlmostEqual(timing['decode_core_wall_s'],timing['decode_wall_s']+timing['core_wall_s'])
        self.node.report()
        payload=json.loads(self.node.status.publish.call_args.args[0].data)
        self.assertEqual(payload['last_per_exposure_timing'],timing)
        self.assertEqual(payload['accepted_stamp'],2.)
        self.assertTrue(payload['healthy'])

    def test_old_snapshot_boundary_remains_recovering_despite_fresh_current_fit(self):
        original=self.node.quality.snapshot;calls=[]
        def snapshot(now):
            calls.append((now,self.node.quality.accepted_stamp))
            return original(now)
        self.node.quality.snapshot=snapshot
        self.block_first_fit()
        self.clock(2.201)
        self.release.set()
        self.wait_until(lambda:not self.node.processing_inflight)
        timing=self.node.runtime_diagnostics()['last_per_exposure_timing']
        self.assertAlmostEqual(timing['old_accepted_stamp_before_snapshot'],1.9)
        self.assertAlmostEqual(timing['old_accepted_age_before_snapshot_s'],.301)
        self.assertAlmostEqual(timing['new_exposure_age_after_s'],.201)
        self.assertEqual(calls,[(2.02,1.9),(2.201,1.9),(2.201,2.)])
        payload=json.loads(self.node.status.publish.call_args.args[0].data)
        self.assertEqual(payload['reason'],'recovering')
        self.assertFalse(payload['healthy'])
        self.assertEqual(payload['successful_streak'],1)
        self.assertEqual(payload['accepted_stamp'],2.)
        self.node.publisher.publish.assert_called_once()

    def test_postcompute_processing_lock_wait_is_separate_from_core_wall(self):
        returned=threading.Event();result=self.core.update.return_value
        def update(at,*args,**kwargs):
            self.entered.set()
            self.assertTrue(self.release.wait(2.))
            returned.set()
            return dict(result,stamp=at)
        self.core.update.side_effect=update
        self.start_worker()
        self.node.on_color(self.color());self.node.on_depth(self.depth())
        self.assertTrue(self.entered.wait(1.))
        with self.node.processing_lock:
            self.release.set()
            self.assertTrue(returned.wait(1.))
            threading.Event().wait(.04)
        self.wait_until(lambda:not self.node.processing_inflight)
        timing=self.node.runtime_diagnostics()['last_per_exposure_timing']
        self.assertGreaterEqual(timing['end_lock_wait_wall_s'],.03)
        self.assertGreaterEqual(timing['core_wall_s'],0.)
        self.assertEqual(self.node.quality.accepted_stamp,2.)
        self.node.publisher.publish.assert_called_once()
