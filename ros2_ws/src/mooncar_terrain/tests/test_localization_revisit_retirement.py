"""Planning-reference retirement counterexamples; no ROS execution or data replay."""
import ast
import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'scripts/autonomous_survey.py').is_file())
sys.path.insert(0,str(ROOT/'scripts'))
import localization_revisit as policy


def original_function(relative,name):
    path=ROOT/relative
    node,=[n for n in ast.parse(path.read_text(encoding='utf-8-sig')).body if isinstance(n,ast.FunctionDef)and n.name==name]
    namespace=dict(math=math,np=np)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),namespace)
    return namespace[name]


clearance=original_function('scripts/autonomous_survey.py','known_chassis_clearance')
within_region=original_function('scripts/experiment_motion.py','within_region')


def sample(stamp,ref=114,std=(.03,.02,.025),counter=0,session='producer'):
    payload=dict(localization_backend='metric_rgbd_3d',healthy=True,session_id=session,sequence=round(stamp*1000),
        accepted_stamp=stamp,last_fit=dict(accepted=True,exposure_stamp=stamp,reference_id=ref,position_std_m=list(std)))
    value=policy.sample_payload(payload,dict(healthy=True,session_id=session,sequence=payload['sequence'],fault_counter=counter))
    return value


def observation(stamp,x):return dict(stamp=stamp,x=x,y=1.05,yaw=0.,pan=0.,tilt=-.262)


def started():
    tracker=policy.RGBDRevisitPolicy()
    tracker.observe(sample(1.,79,(.015,.010,.012)),observation(1.,2.05))
    tracker.observe(sample(2.,76,(.012,.012,.014)),observation(2.,1.05))
    trigger=sample(4.)
    tracker.observe(trigger,observation(4.,3.05))
    checkpoint=next(row for row in tracker.observations if row['reference_id']==79)
    tracker.begin(dict(checkpoint=checkpoint,goals=[checkpoint],estimated_seconds=30.))
    return tracker,trigger


def retirement(tracker,stamp=5.,ref=79,**kwargs):
    value=sample(stamp,**kwargs);value['retired_reference_id']=ref
    try:tracker.observe(value,None)
    except policy.RetiredCheckpoint as event:return event
    raise AssertionError('An actual active checkpoint retirement did not raise its typed event')


def replan(tracker):
    grid=NS(info=NS(width=60,height=60,resolution=.1,origin=NS(position=NS(x=0.,y=0.))))
    costs=np.zeros((60,60),dtype=np.int16)
    return tracker.plan(dict(x=3.05,y=1.05,yaw=0.),grid,costs,clearance,within_region,
                        [-10.,-10.,10.,10.],[1.05,1.05],10.,1000.,[0.,-.262])


class RetiredCheckpointPolicyTests(unittest.TestCase):
    def test_observed_retirement_is_typed_and_other_observation_survives(self):
        tracker,_=started();other=next(row for row in tracker.observations if row['reference_id']==76)
        event=retirement(tracker)
        self.assertIsInstance(event,ValueError)
        self.assertEqual((event.reference_id,event.stamp),(79,5.))
        self.assertTrue(tracker.running)
        self.assertFalse(any(row['reference_id']==79 for row in tracker.observations))
        self.assertIn(other,tracker.observations)

    def test_invalidation_preserves_frozen_trigger_attempt_session_and_clock(self):
        tracker,trigger=started();frozen=copy.deepcopy(trigger)
        tracker.proof_stamps=[4.1,4.2]
        event=retirement(tracker)
        tracker.invalidate_retired_checkpoint(event)
        self.assertIs(tracker.pending['trigger'],trigger)
        self.assertEqual(trigger,frozen)
        self.assertEqual(tracker.attempts,1)
        self.assertEqual(tracker.session,'producer')
        self.assertEqual(tracker.last_stamp,5.)
        self.assertFalse(tracker.running)
        self.assertEqual(tracker.proof_stamps,[])
        self.assertNotIn('checkpoint',tracker.pending)
        self.assertNotIn('goals',tracker.pending)
        self.assertEqual(tracker.pending['retirement_event'],dict(reference_id=79,stamp=5.))

    def test_generic_wrong_reference_and_inactive_events_cannot_invalidate(self):
        for event in (ValueError('quality failed'),policy.RetiredCheckpoint(76,5.)):
            with self.subTest(event=type(event).__name__):
                tracker,_=started();before=copy.deepcopy(tracker.pending)
                with self.assertRaises(ValueError):tracker.invalidate_retired_checkpoint(event)
                self.assertEqual(tracker.pending,before)
                self.assertTrue(tracker.running)
        tracker,_=started();event=retirement(tracker);tracker.invalidate_retired_checkpoint(event)
        with self.assertRaises(ValueError):tracker.invalidate_retired_checkpoint(event)

    def test_quality_fault_has_priority_and_still_latches_during_pending_retry(self):
        tracker,_=started();value=sample(5.,counter=1);value['retired_reference_id']=79
        with self.assertRaises(ValueError)as caught:tracker.observe(value,None)
        self.assertNotIsInstance(caught.exception,policy.RetiredCheckpoint)
        self.assertTrue(tracker.running)
        self.assertEqual(tracker.attempts,1)
        tracker,_=started();event=retirement(tracker);tracker.invalidate_retired_checkpoint(event)
        pending_fault=sample(6.,counter=1);pending_fault['retired_reference_id']=76
        with self.assertRaises(ValueError)as caught:tracker.observe(pending_fault,None)
        self.assertNotIsInstance(caught.exception,policy.RetiredCheckpoint)
        self.assertFalse(tracker.running)
        self.assertEqual(tracker.pending['trigger']['fault_counter'],0)

    def test_clock_or_producer_change_cannot_be_reclassified_as_retirement(self):
        for value in (sample(3.),sample(5.,session='different')):
            with self.subTest(stamp=value['stamp'],session=value['session_id']):
                tracker,_=started();value['retired_reference_id']=79
                with self.assertRaises(ValueError)as caught:tracker.observe(value,None)
                self.assertNotIsInstance(caught.exception,policy.RetiredCheckpoint)
                self.assertTrue(tracker.running)
                self.assertEqual(tracker.pending['checkpoint']['reference_id'],79)

    def test_replan_uses_remaining_candidate_without_restoring_retired_goal(self):
        tracker,trigger=started();event=retirement(tracker);tracker.invalidate_retired_checkpoint(event)
        selected=replan(tracker)
        self.assertIsNotNone(selected)
        self.assertEqual(selected['checkpoint']['reference_id'],76)
        self.assertFalse(any(row.get('reference_id')==79 for row in selected['goals']))
        tracker.begin(selected)
        self.assertEqual(tracker.attempts,2)
        self.assertIs(tracker.pending['trigger'],trigger)

    def test_retirement_never_refunds_the_original_four_attempt_budget(self):
        tracker=policy.RGBDRevisitPolicy()
        for stamp,ref,x in ((1.,79,2.05),(2.,76,1.85),(3.,70,1.65),(4.,66,1.45)):
            tracker.observe(sample(stamp,ref,(.015,.010,.012)),observation(stamp,x))
        trigger=sample(5.);tracker.observe(trigger,observation(5.,3.05))
        for index in range(policy.MAX_REVISITS):
            selected=replan(tracker);self.assertIsNotNone(selected)
            tracker.begin(selected)
            event=retirement(tracker,6.+index,selected['checkpoint']['reference_id'])
            tracker.invalidate_retired_checkpoint(event)
            self.assertEqual(tracker.attempts,index+1)
            self.assertIs(tracker.pending['trigger'],trigger)
        with self.assertRaises(ValueError):replan(tracker)
        self.assertEqual(tracker.attempts,policy.MAX_REVISITS)

    def test_old_partial_proof_cannot_carry_into_new_checkpoint(self):
        tracker,_=started()
        self.assertFalse(tracker.prove(sample(4.1,79,(.015,.010,.012)),4.))
        self.assertFalse(tracker.prove(sample(4.2,79,(.015,.010,.012)),4.))
        event=retirement(tracker);tracker.invalidate_retired_checkpoint(event)
        self.assertEqual(tracker.proof_stamps,[])
        tracker.begin(replan(tracker))
        self.assertFalse(tracker.prove(sample(6.1,79,(.015,.010,.012)),6.))
        self.assertFalse(tracker.prove(sample(6.2,76,(.015,.010,.012)),6.))
        self.assertFalse(tracker.prove(sample(6.3,76,(.015,.010,.012)),6.))
        self.assertTrue(tracker.prove(sample(6.4,76,(.015,.010,.012)),6.))

    def test_retirement_of_a_different_reference_does_not_cancel_current_plan(self):
        tracker,_=started();before=tracker.pending
        value=sample(5.);value['retired_reference_id']=76
        tracker.observe(value,None)
        self.assertTrue(tracker.running)
        self.assertIs(tracker.pending,before)
        self.assertEqual(tracker.pending['checkpoint']['reference_id'],79)
        self.assertEqual([r['reference_id']for r in tracker.observations],[79])


if __name__=='__main__':unittest.main()
