"""Bounded post-view repositioning; no ROS/control or covariance proof changes.

The existing pure tests own the actual three-exposure proof. These actor tests
mock proof only to verify it cannot begin before a successful bounded return,
new view restoration and fresh same-request feedback.
"""
import copy
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock, patch

TEST_DIRECTORY=Path(__file__).resolve().parent
ROOT=next(parent for parent in TEST_DIRECTORY.parents if (parent/'scripts/autonomous_survey.py').is_file())
sys.path[:0]=[str(TEST_DIRECTORY),str(ROOT/'scripts'),str(ROOT/'ros2_ws/src/mooncar_terrain/tests')]
import autonomous_survey
import test_localization_revisit_integration as original


class RevisitDisplacementTests(unittest.TestCase):
    def setUp(self):
        self.original=original.RevisitIntegrationTests()
        self.original.setUp();self.addCleanup(self.original.doCleanups)

    def setup_actor(self,first_drift=.31,arrival=0.,second_drift=0.):
        survey,motion,state=self.original.fixture()
        survey.update_localization_revisit()
        motion.state.return_value=dict(state,active=False,phase='cancelled')
        motion.pose.return_value=dict(x=0.,y=0.,yaw=3.)
        low=original.sample((.012,.012,.014),stamp=5.)
        checkpoint=dict(low,x=0.,y=0.,yaw=0.,pan=0.,tilt=-.26)
        survey.revisit_policy.plan=Mock(return_value=dict(checkpoint=checkpoint,goals=[],estimated_seconds=30.))
        survey.revisit_last_sample=low;survey.revisit_policy.prove=Mock(return_value=True)
        calls=[];gimbals=[0]
        def execute(decision,turn=False):
            kind='gimbal'if decision.get('use_pan_tilt')else 'spin'if turn else'navigate_to_pose'
            calls.append(dict(kind=kind,decision=copy.deepcopy(decision)))
            identity='new-'+str(len(calls));token='token-'+str(len(calls))
            motion.token=token;survey.survey_action_identity=(token,identity)
            motion.state.return_value=dict(active=False,phase='succeeded',diagnostics=dict(goal_request_id=identity,goal_request_kind=kind))
            if kind=='spin':motion.pose.return_value=dict(x=first_drift,y=0.,yaw=0.)
            elif kind=='navigate_to_pose':motion.pose.return_value=dict(x=arrival,y=0.,yaw=0.)
            else:
                gimbals[0]+=1
                if gimbals[0]>1:motion.pose.return_value=dict(x=second_drift,y=0.,yaw=0.)
            motion.gimbal_joint_stamp=10.;motion.gimbal_joint_received=time.monotonic()
            motion.gimbal_position=[0.,-.26]
            return True
        survey.execute=Mock(side_effect=execute)
        return survey,motion,checkpoint,calls

    def direct_actor(self):
        survey,motion,checkpoint,calls=self.setup_actor()
        motion.token='view-token';survey.survey_action_identity=('view-token','view-id')
        motion.state.return_value=dict(active=False,phase='succeeded',diagnostics=dict(goal_request_id='view-id',goal_request_kind='gimbal'))
        motion.pose.return_value=dict(x=.31,y=0.,yaw=0.)
        survey.revisit_policy.running=True;survey.revisit_policy.attempts=1
        survey.revisit_policy.pending['checkpoint']=checkpoint
        record=dict(trigger=copy.deepcopy(survey.revisit_policy.pending['trigger']))
        survey.data['localization_revisits']=[record]
        return survey,motion,checkpoint,record,calls

    def test_correction_new_request_new_view_and_feedback_precede_proof(self):
        survey,motion,checkpoint,calls=self.setup_actor()
        original_proof=survey.revisit_policy.prove
        def prove(*args):
            self.assertEqual([r['kind']for r in calls],['spin','gimbal','navigate_to_pose','gimbal'])
            record=survey.data['localization_revisits'][0]
            self.assertEqual(len(record['view_restoration_checks']),2)
            self.assertEqual(len(record['view_displacement_terminals']),3)
            self.assertIn('joint_stamp',record['view_displacement_terminals'][-1])
            self.assertLessEqual(record['view_restoration_checks'][-1]['distance_to_saved_m'],.05)
            return True
        original_proof.side_effect=prove
        with patch.object(survey,'healthy',return_value=True):
            self.assertTrue(survey.perform_localization_revisit())
        self.assertEqual(survey.revisit_policy.attempts,2)
        self.assertEqual(sum(r['kind']=='navigate_to_pose'for r in calls),1)
        correction=survey.data['localization_revisits'][0]['view_restoration_displacement']
        self.assertEqual(correction['prior_terminal_request_id'],'new-2')
        self.assertEqual(correction['saved_reference_id'],checkpoint['reference_id'])
        self.assertEqual(correction['decision']['reason'],'view_restoration_displacement')
        original_proof.assert_called_once()

    def test_five_centimetre_boundary_does_not_add_action(self):
        survey,motion,checkpoint,calls=self.setup_actor(first_drift=.05)
        with patch.object(survey,'healthy',return_value=True),patch.object(survey,'correct_revisit_view_displacement',side_effect=AssertionError('boundary already within precise revisit tolerance')):
            self.assertTrue(survey.perform_localization_revisit())
        self.assertEqual([r['kind']for r in calls],['spin','gimbal'])
        self.assertEqual(survey.revisit_policy.attempts,1)

    def test_current_grid_target_clearance_strict_path_region_radius_are_mandatory(self):
        for cause in('stale','target66','unknown','hazard_disk','strict_path','region','radius'):
            with self.subTest(cause=cause):
                survey,motion,checkpoint,record,calls=self.direct_actor()
                if cause=='stale':motion.grid_fresh.return_value=False
                if cause=='target66':motion.grid.data[10*20+10]=66
                if cause=='unknown':motion.grid.data[10*20+10]=-1
                if cause=='hazard_disk':motion.grid.data[10*20+11]=100
                if cause=='region':survey.task['region']=[-.1,-.1,.1,.1]
                if cause=='radius':survey.data['radius']=.7
                geometry=patch('localization_revisit.safe_segment',return_value=False)if cause=='strict_path'else patch('autonomous_survey.time.sleep')
                with geometry,patch.object(survey,'healthy',return_value=True),patch.object(survey,'stop')as stop:
                    self.assertFalse(survey.correct_revisit_view_displacement(checkpoint,record))
                survey.execute.assert_not_called();survey.revisit_policy.prove.assert_not_called()
                self.assertEqual(survey.revisit_policy.attempts,1)
                stop.assert_called_once()

    def test_budget_reserves_full_new_view_and_proof_before_spending_attempt(self):
        survey,motion,checkpoint,record,calls=self.direct_actor()
        survey.data['budget_seconds']=10.1
        with patch.object(survey,'healthy',return_value=True),patch.object(survey,'stop')as stop, \
                patch.object(survey,'has_action_budget',wraps=survey.has_action_budget)as budget:
            self.assertFalse(survey.correct_revisit_view_displacement(checkpoint,record))
        self.assertGreater(budget.call_args.args[0]['estimated_seconds'],40.)
        self.assertEqual(stop.call_args.args[1],'budget_reached')
        self.assertEqual(survey.revisit_policy.attempts,1)
        survey.execute.assert_not_called()

    def test_original_four_attempt_budget_includes_correction(self):
        for attempts,expected in((3,True),(4,False)):
            with self.subTest(attempts=attempts):
                survey,motion,checkpoint,record,calls=self.direct_actor();survey.revisit_policy.attempts=attempts
                with patch.object(survey,'healthy',return_value=True),patch.object(survey,'stop'):
                    self.assertEqual(survey.correct_revisit_view_displacement(checkpoint,record),expected)
                self.assertEqual(survey.revisit_policy.attempts,4)
                self.assertEqual(survey.execute.call_count,int(expected))

    def test_same_frozen_trigger_cannot_gain_a_second_correction_after_reselection(self):
        survey,motion,checkpoint,record,calls=self.direct_actor()
        previous=dict(trigger=copy.deepcopy(record['trigger']),view_restoration_displacement=dict(attempt=1))
        survey.data['localization_revisits'].insert(0,previous)
        with patch.object(survey,'healthy',return_value=True),patch.object(survey,'stop')as stop:
            self.assertFalse(survey.correct_revisit_view_displacement(checkpoint,record))
        survey.execute.assert_not_called();survey.revisit_policy.prove.assert_not_called()
        self.assertEqual(survey.revisit_policy.attempts,1);stop.assert_called_once()

    def test_second_view_displacement_stops_instead_of_looping(self):
        survey,motion,checkpoint,calls=self.setup_actor(second_drift=.31)
        with patch.object(survey,'healthy',return_value=True),patch.object(survey,'stop')as stop:
            self.assertFalse(survey.perform_localization_revisit())
        self.assertEqual([r['kind']for r in calls],['spin','gimbal','navigate_to_pose','gimbal'])
        self.assertEqual(sum(r['kind']=='navigate_to_pose'for r in calls),1)
        survey.revisit_policy.prove.assert_not_called()
        self.assertEqual(stop.call_args.args[1],'blocked')

    def test_correction_not_arrived_cannot_restore_again_or_prove(self):
        survey,motion,checkpoint,calls=self.setup_actor(arrival=.27)
        with patch.object(survey,'healthy',return_value=True),patch.object(survey,'stop')as stop:
            self.assertFalse(survey.perform_localization_revisit())
        self.assertEqual([r['kind']for r in calls],['spin','gimbal','navigate_to_pose'])
        survey.revisit_policy.prove.assert_not_called()
        self.assertEqual(stop.call_args.args[1],'blocked')

    def test_quality_fault_precedes_correction_geometry_and_dispatch(self):
        survey,motion,checkpoint,record,calls=self.direct_actor()
        with patch.object(survey,'healthy',return_value=False):
            self.assertFalse(survey.correct_revisit_view_displacement(checkpoint,record))
        motion.grid_fresh.assert_not_called();survey.execute.assert_not_called()
        self.assertEqual(survey.revisit_policy.attempts,1)

    def test_empty_mismatched_active_or_failed_terminal_cannot_authorize_new_goal(self):
        for cause in('empty_token','empty_request','mismatch','active','failed'):
            with self.subTest(cause=cause):
                survey,motion,checkpoint,record,calls=self.direct_actor()
                state=copy.deepcopy(motion.state.return_value)
                if cause=='empty_token':motion.token='';survey.survey_action_identity=('','view-id')
                if cause=='empty_request':state['diagnostics']['goal_request_id']='';survey.survey_action_identity=('view-token','')
                if cause=='mismatch':survey.survey_action_identity=('other','view-id')
                if cause=='active':state['active']=True
                if cause=='failed':state['phase']='failed'
                motion.state.return_value=state
                with patch.object(survey,'healthy',return_value=True),patch.object(survey,'stop')as stop:
                    self.assertFalse(survey.correct_revisit_view_displacement(checkpoint,record))
                survey.execute.assert_not_called();self.assertEqual(survey.revisit_policy.attempts,1)
                stop.assert_called_once()

    def test_final_view_requires_fresh_actual_joint_feedback_before_proof(self):
        for cause in('old_sim','old_wall','invalid_position'):
            with self.subTest(cause=cause):
                survey,motion,checkpoint,record,calls=self.direct_actor()
                motion.gimbal_joint_stamp=10.;motion.gimbal_joint_received=time.monotonic();motion.gimbal_position=[0.,-.26]
                if cause=='old_sim':motion.gimbal_joint_stamp=9.8
                if cause=='old_wall':motion.gimbal_joint_received=time.monotonic()-2.1
                if cause=='invalid_position':motion.gimbal_position=[0.,float('nan')]
                with patch.object(survey,'healthy',return_value=True),patch.object(survey,'stop')as stop:
                    self.assertFalse(survey.revisit_correction_terminal(record,final_view=True))
                survey.execute.assert_not_called();survey.revisit_policy.prove.assert_not_called();stop.assert_called_once()


if __name__=='__main__':
    unittest.main()
