"""Non-ROS contracts for the opt-in real sensor transport adapter."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from offline_rgbd_window.transport_async import AsyncTransportProcessor, install_async_transport
from offline_rgbd_window.core import WindowError
from test_offline_rgbd_window_async import Frontend, sensor


class TransportContracts(unittest.TestCase):
    def test_requires_opt_in_before_starting_worker(self):
        with self.assertRaisesRegex(ValueError, 'explicit'):
            AsyncTransportProcessor()

    def test_unplanned_frame_tracks_without_query_job_or_output(self):
        p = AsyncTransportProcessor(cached_query=True, frontend=Frontend())
        self.addCleanup(p.close)
        with patch.object(p.worker, 'offer', side_effect=AssertionError('unplanned KF')):
            self.assertIsNone(p.process(sensor(1), None))
        self.assertEqual(p.last_stamp_ns, 100_000_000)
        self.assertEqual(p.worker.poll()[1]['counts'], {})
        row = p.process(sensor(2), 200_000_000)
        self.assertFalse(row['output'])
        self.assertTrue(p.worker.wait_idle())
        row = p.process(sensor(3), 300_000_000)
        self.assertTrue(row['output'])
        self.assertEqual(row['gauge_stamp_ns'], 200_000_000)

    def test_unplanned_clock_regression_does_not_modify_frontend_state(self):
        p = AsyncTransportProcessor(cached_query=True, frontend=Frontend())
        self.addCleanup(p.close)
        p.process(sensor(2), None)
        with self.assertRaises(WindowError):
            p.process(sensor(1), None)
        self.assertEqual(p.last_stamp_ns, 200_000_000)

    def test_existing_readiness_and_shutdown_failures_keep_original_fields(self):
        p = AsyncTransportProcessor(cached_query=True, frontend=Frontend())
        self.addCleanup(p.close)
        row = p.failure(100, 110, 'readiness_timeout', 10, 20, 30, 120)
        self.assertEqual(row['planned_stamp_ns'], 100)
        self.assertEqual(row['stamp_ns'], 110)
        self.assertFalse(row['output'])
        self.assertEqual(row['reason'], 'readiness_timeout')

    def test_worker_closes_before_transport_finish_even_on_foreground_error(self):
        events = []
        processor = SimpleNamespace(max_cache_age_ns=1_000_000_000,
            worker=SimpleNamespace(poll=lambda: (None, dict(closed=bool(events))),
                                   _thread=SimpleNamespace(is_alive=lambda: False)),
            close=lambda timeout: events.append('close'))
        module = SimpleNamespace()
        class Node:
            def __init__(self, args):self.processor = module.ShadowProcessor()
            def event(self, **row):events.append(row)
            def set_fault(self, reason):events.append(reason)
            def work(self):raise WindowError('foreground failed')
        module.ShadowNode = Node
        with patch('offline_rgbd_window.transport_async.AsyncTransportProcessor', return_value=processor):
            install_async_transport(module)
            node = module.ShadowNode(None)
        self.assertIsNot(module.ShadowNode, Node)
        with self.assertRaisesRegex(WindowError, 'foreground failed'):
            node.work()
        close = next(r for r in events if isinstance(r, dict) and r['type']=='async_backend_close')
        self.assertTrue(close['complete'])
        self.assertFalse(close['backend_thread_alive'])
        self.assertLess(events.index('close'), events.index(close))


if __name__ == '__main__':unittest.main()
