"""Execute candidate motion methods with real quality policy, without ROS transport.

AST extraction executes the actual source methods (including reset and generation
fences), rather than a second implementation. ROS entities alone are substituted.
The ordinary ROS integration test also exercises this boundary after application.
"""
import ast
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
from localization_quality import LaserQualitySource,LocalizationQuality,MAX_STATUS_WALL_AGE,LocalizationDispatchRejected


def extracted_motion():
    path=ROOT/'scripts/experiment_motion.py'
    tree=ast.parse(path.read_text(encoding='utf-8'))
    source=next(node for node in tree.body if isinstance(node,ast.ClassDef) and node.name=='ExperimentMotion')
    names={'subscribe_localization_quality','on_localization_quality','_accept_new_epoch_locked',
           '_localization_state_locked','localization_state','activate_localization_action','reset'}
    methods=[node for node in source.body if isinstance(node,ast.FunctionDef) and node.name in names]
    if {node.name for node in methods}!=names:raise AssertionError('Required actual candidate methods missing')
    extracted=ast.ClassDef(name='CandidateMotion',bases=[],keywords=[],body=methods,decorator_list=[])
    module=ast.fix_missing_locations(ast.Module(body=[extracted],type_ignores=[]))
    namespace=dict(json=json,time=time,LocalizationQuality=LocalizationQuality,LocalizationDispatchRejected=LocalizationDispatchRejected,
        MAX_STATUS_WALL_AGE=MAX_STATUS_WALL_AGE,String=SimpleNamespace,
        Buffer=Mock(side_effect=lambda **kw:Mock()),TransformListener=Mock(side_effect=lambda *a:Mock()),
        ActionClient=Mock(side_effect=lambda *a,**kw:Mock()),NavigateToPose=object(),Spin=object())
    exec(compile(module,str(path),'exec'),namespace)
    return namespace['CandidateMotion']


Motion=extracted_motion()


def ready_source(session,stamp):
    source=LaserQualitySource(session)
    source.observe(stamp-.3,dict(accepted=True,reason='initialized'))
    for value in (stamp-.2,stamp-.1,stamp):source.observe(value,dict(accepted=True,reason='accepted'))
    return source


def motion(stamp=900.,required=True):
    node=Motion()
    node.lock=threading.RLock();node.localization_lock=threading.RLock()
    node.localization_quality=LocalizationQuality(required=required)
    node.localization_fault_counter=0;node.localization_action_fault_counter=0
    node.localization_was_healthy=False;node.localization_generation=0
    node.localization_subscription=None;node.localization_epoch_pending=False
    node.localization_epoch_candidate=None;node.localization_previous_session=None
    node.stamp=stamp
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=round(node.stamp*1e9)))
    node.clock_node=Mock();node.cancel=Mock();node.buffer=Mock();node.listener=Mock()
    node.destroy_service=Mock();node.client=Mock();node.spin_client=Mock()
    node.control_group=object();node.active=False
    return node


def send(node,source,stamp=None,generation=None):
    stamp=node.stamp if stamp is None else stamp
    node.on_localization_quality(SimpleNamespace(data=json.dumps(source.snapshot(stamp))),
        node.localization_generation if generation is None else generation)


def bound_old_motion():
    node=motion();source=ready_source('previous',900.)
    send(node,source)
    assert node.localization_state()['healthy']
    return node,source


def complete_epoch(node,source=None):
    source=source or ready_source('replacement',.3)
    node.stamp=.3;send(node,source)
    node.stamp=.4;source.observe(.4,dict(accepted=True,reason='accepted'));send(node,source)
    return source


class ClockEpochTests(unittest.TestCase):
    def test_warm_reset_polling_old_clock_cannot_bind_new_consumer(self):
        node,old=bound_old_motion();previous_generation=node.localization_generation
        node.reset()
        self.assertEqual(node.localization_generation,previous_generation+1)
        for _ in range(4):
            self.assertEqual(node.localization_state()['reason'],'awaiting_new_clock_epoch')
            self.assertIsNone(node.localization_quality.last_sim)
        self.assertEqual(node.stamp,900.)
        node.stamp=0.
        self.assertEqual(node.localization_state()['reason'],'awaiting_new_clock_epoch')
        self.assertIsNone(node.localization_quality.fault)
        complete_epoch(node)
        self.assertTrue(node.localization_state()['healthy'])
        self.assertEqual(node.localization_quality.session_id,'replacement')
        self.assertIsNone(node.localization_quality.fault)
        self.assertEqual(node.localization_quality.last_sim,.4)

    def test_no_action_before_new_clock_and_distinct_fresh_measurement(self):
        node,_=bound_old_motion();node.reset()
        with self.assertRaisesRegex(ValueError,'awaiting_new_clock_epoch'):node.activate_localization_action()
        source=ready_source('replacement',.3);node.stamp=.3;send(node,source)
        with self.assertRaisesRegex(ValueError,'awaiting_new_clock_epoch'):node.activate_localization_action()
        self.assertFalse(node.active)
        node.stamp=.4;source.observe(.4,dict(accepted=True,reason='accepted'));send(node,source)
        node.activate_localization_action();self.assertTrue(node.active)

    def test_running_backwards_clock_still_latches_without_auto_rearm(self):
        node,source=bound_old_motion();node.stamp=899.9
        self.assertEqual(node.localization_state()['reason'],'clock_moved_backwards')
        node.stamp=900.1;source.observe(900.1,dict(accepted=True,reason='accepted'));send(node,source)
        self.assertEqual(node.localization_state()['reason'],'clock_moved_backwards')
        self.assertFalse(node.localization_epoch_pending)
        with self.assertRaisesRegex(ValueError,'clock_moved_backwards'):node.activate_localization_action()

    def test_running_producer_change_still_latches(self):
        node,_=bound_old_motion();replacement=ready_source('unrequested-change',900.)
        send(node,replacement)
        self.assertEqual(node.localization_state()['reason'],'producer_session_changed')
        self.assertFalse(node.localization_epoch_pending)

    def test_old_callback_generation_and_previous_producer_both_rejected(self):
        node,old=bound_old_motion();generation=node.localization_generation;node.reset()
        replacement=ready_source('replacement',.3);node.stamp=.3
        send(node,replacement,generation=generation)
        self.assertIsNone(node.localization_epoch_candidate)
        node.stamp=900.;send(node,old)
        self.assertIsNone(node.localization_epoch_candidate)
        self.assertIsNone(node.localization_quality.session_id)
        complete_epoch(node,replacement)
        self.assertTrue(node.localization_state()['healthy'])

    def test_heartbeats_without_new_accepted_measurement_do_not_open_barrier(self):
        node,_=bound_old_motion();node.reset();source=ready_source('replacement',.3)
        node.stamp=.3;send(node,source)
        for stamp in (.31,.32,.33):node.stamp=stamp;send(node,source)
        self.assertTrue(node.localization_epoch_pending)
        self.assertIsNone(node.localization_quality.session_id)
        node.stamp=.4;source.observe(.4,dict(accepted=True,reason='accepted'));send(node,source)
        self.assertTrue(node.localization_state()['healthy'])

    def test_paused_clock_cannot_open_barrier_even_with_new_source_timestamps(self):
        node,_=bound_old_motion();node.reset();source=ready_source('replacement',.3)
        node.stamp=.3;send(node,source)
        source.observe(.31,dict(accepted=True,reason='accepted'));send(node,source,stamp=.31)
        self.assertTrue(node.localization_epoch_pending)
        node.stamp=.4;source.observe(.4,dict(accepted=True,reason='accepted'));send(node,source)
        self.assertTrue(node.localization_state()['healthy'])

    def test_stale_or_future_status_cannot_open_barrier_against_old_clock(self):
        node,_=bound_old_motion();node.reset();source=ready_source('replacement',.3)
        send(node,source,stamp=.3)
        self.assertIsNone(node.localization_epoch_candidate)
        node.stamp=0.;send(node,source,stamp=.3)
        self.assertIsNone(node.localization_epoch_candidate)
        self.assertIsNone(node.localization_quality.last_sim)
        self.assertIsNone(node.localization_quality.fault)
        complete_epoch(node,source)
        self.assertTrue(node.localization_state()['healthy'])

    def test_nonrequired_offline_mode_does_not_wait_for_absent_quality_source(self):
        node=motion(required=False);node.reset()
        self.assertFalse(node.localization_epoch_pending)
        self.assertTrue(node.localization_state()['healthy'])
        self.assertEqual(node.localization_state()['reason'],'disabled')

    def test_unhealthy_new_epoch_never_binds_consumer_or_enables_motion(self):
        node,_=bound_old_motion();node.reset();source=LaserQualitySource('replacement')
        for stamp in (.1,.2,.3,1.,10.):
            node.stamp=stamp
            source.observe(stamp,dict(accepted=False,reason='unobservable_depth_translation'))
            send(node,source)
            self.assertEqual(node.localization_state()['reason'],'awaiting_new_clock_epoch')
            self.assertIsNone(node.localization_epoch_candidate)
            self.assertIsNone(node.localization_quality.session_id)
            self.assertIsNone(node.localization_quality.last_sim)
            self.assertIsNone(node.localization_quality.fault)
            with self.assertRaisesRegex(ValueError,'awaiting_new_clock_epoch'):
                node.activate_localization_action()
            self.assertFalse(node.active)
        self.assertEqual(node.localization_fault_counter,0)

    def test_unhealthy_between_healthy_samples_requires_a_new_pair(self):
        node,_=bound_old_motion();node.reset();source=ready_source('replacement',.3)
        node.stamp=.3;send(node,source)
        self.assertIsNotNone(node.localization_epoch_candidate)
        node.stamp=.31;source.reject('unobservable_depth_translation');send(node,source)
        self.assertIsNone(node.localization_epoch_candidate)
        self.assertIsNone(node.localization_quality.session_id)
        for stamp in (.4,.5,.6):
            node.stamp=stamp;source.observe(stamp,dict(accepted=True,reason='accepted'));send(node,source)
            self.assertTrue(node.localization_epoch_pending)
        with self.assertRaisesRegex(ValueError,'awaiting_new_clock_epoch'):
            node.activate_localization_action()
        node.stamp=.7;source.observe(.7,dict(accepted=True,reason='accepted'));send(node,source)
        self.assertFalse(node.localization_epoch_pending)
        self.assertTrue(node.localization_state()['healthy'])
        self.assertFalse(node.active,'Opening the barrier cannot resume an old action')


if __name__=='__main__':unittest.main(verbosity=2)
