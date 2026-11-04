"""Opt-in adapter for an isolated copy of the passive ROS shadow transport.

No default node or workbench imports this module. Keep raw sensor pairing,
readiness, scheduling and logging in the existing transport unchanged.
"""
import time

from .async_processor import AsyncProcessor
from .core import WindowError
from .online import ShadowProcessor


class AsyncTransportProcessor(AsyncProcessor):
    def __init__(self, cached_query=False, timing_diagnostics=False, **kwargs):
        if cached_query is not True:
            raise ValueError('async shadow requires explicit --cached-query')
        super().__init__(**kwargs)
        self.timing_diagnostics = bool(timing_diagnostics)

    failure = ShadowProcessor.failure

    def process(self, sensor, planned_stamp_ns, received_wall_ns=0, started_wall_ns=0,
                sim_now_ns=None, wall_now_ns=time.monotonic_ns):
        if planned_stamp_ns is not None:
            return super().process(sensor, planned_stamp_ns, received_wall_ns, started_wall_ns,
                                   sim_now_ns, wall_now_ns)
        # An unplanned exposure still maintains real track identity, just as
        # the synchronous processor does. It creates no pose or KF request.
        if self.last_stamp_ns is not None and sensor.stamp_ns <= self.last_stamp_ns:
            raise WindowError('foreground exposure stamps must increase')
        publication, _ = self.worker.poll()
        self._install(publication, sensor.stamp_ns)
        self.last_stamp_ns = sensor.stamp_ns
        self.frontend.process(sensor)
        return None


def install_async_transport(isolated_node_module):
    """Install only into a freshly loaded, task-local node module namespace."""
    original = isolated_node_module.ShadowNode

    class AsyncShadowNode(original):
        def __init__(self, args):
            super().__init__(args)
            self.event(type='async_configuration', execution='foreground_query_background_commit',
                cache_ttl_sim_s=self.processor.max_cache_age_ns/1e9,
                max_running_commits=1, max_pending_commits=1,
                native_cache_release='creator_thread_after_foreground_ack')

        def work(self):
            try:
                return super().work()
            finally:
                before = self.processor.worker.poll()[1]
                try:
                    self.processor.close(timeout=5.)
                    after = self.processor.worker.poll()[1]
                    self.event(type='async_backend_close', complete=True, before=before,
                        after=after, backend_thread_alive=self.processor.worker._thread.is_alive())
                except Exception as error:
                    self.set_fault('async_backend_close_failed: '+repr(error))
                    self.event(type='async_backend_close', complete=False, before=before, error=repr(error))
                    raise

    isolated_node_module.ShadowProcessor = AsyncTransportProcessor
    isolated_node_module.ShadowNode = AsyncShadowNode
