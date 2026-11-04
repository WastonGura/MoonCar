"""Reference retirement is a bounded planning restart, never a quality reset."""
import hashlib
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
ROOT=next(parent for parent in Path(__file__).resolve().parents if (parent/'scripts/experiment_motion.py').exists())
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'ros2_ws/src/mooncar_terrain/tests')]
import test_localization_revisit_integration as original_actor
from localization_revisit import MAX_REVISITS


class RetiredRevisitIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.original=original_actor.RevisitIntegrationTests()
        self.original.setUp();self.addCleanup(self.original.doCleanups)

    def fixture(self,kind='navigate_to_pose'):
        survey,motion,state=self.original.fixture(kind)
        trigger=original_actor.sample((.02467277166644797,.025339454167739642,.029194473810490857),59.2)
        trigger['reference_id']=114
        saved=dict(original_actor.sample((.014576427533610993,.018336087611623125,.01865108499882264),49.9),
                   reference_id=79,x=1.3956560662015438,y=.10172432719138767,yaw=.853523330172841,pan=0.,tilt=-.26)
        other=dict(original_actor.sample((.014,.016,.018),49.),reference_id=76,x=0.,y=0.,yaw=0.,pan=0.,tilt=-.26)
        policy=survey.revisit_policy;policy.session='session';policy.last_stamp=61.1
        policy.pending=dict(trigger=trigger,checkpoint=saved,goals=[saved],cancel_request_id='old-explore-id')
        policy.running=True;policy.attempts=1;policy.observations=[saved,other];policy.proof_stamps=[60.9]
        survey.data['localization_revisits']=[dict(trigger=dict(trigger),status='returning',steps=[],plan=dict(checkpoint=saved))]
        value=original_actor.sample((.027,.027,.031),61.2);value.update(reference_id=133,retired_reference_id=79)
        survey.revisit_sample.return_value=value;survey.revisit_observation.return_value=None
        motion.token='return-token';survey.survey_action_identity=('return-token','return-id')
        state['diagnostics']['goal_request_id']='return-id'
        return survey,motion,state,trigger,saved,other,value

    def test_retired_target_cancels_current_owned_return_and_freezes_warning(self):
        survey,motion,state,trigger,saved,other,value=self.fixture()
        survey.update_localization_revisit()
        self.assertTrue(survey.data['active']);self.assertFalse(survey.revisit_policy.running)
        self.assertIs(survey.revisit_policy.pending['trigger'],trigger)
        self.assertEqual(survey.data['localization_revisit_pending']['trigger'],trigger)
        self.assertEqual(survey.revisit_policy.attempts,1);self.assertEqual(survey.revisit_policy.proof_stamps,[])
        self.assertEqual(survey.revisit_policy.observations,[other])
        self.assertNotIn('goals',survey.revisit_policy.pending)
        self.assertEqual(survey.revisit_policy.pending['cancel_request_id'],'return-id')
        self.assertEqual(survey.revisit_policy.pending['old_token_fingerprint'],hashlib.sha256(b'return-token').hexdigest()[:16])
        motion.cancel.assert_called_once_with('保存的回访参考已退休，取消当前回访并重新规划')
        motion.start.assert_not_called();motion.turn.assert_not_called();motion.pan_tilt.assert_not_called()
        self.assertTrue(state['active']);self.assertEqual(state['phase'],'executing')

    def test_replacement_waits_actual_terminal_and_does_not_resume_old_goals(self):
        survey,motion,state,trigger,saved,other,value=self.fixture();survey.update_localization_revisit()
        terminal=[False];count=[0]
        def states():
            count[0]+=1
            if count[0]<3:return dict(state,phase='cancelling')
            terminal[0]=True;return dict(state,active=False,phase='cancelled')
        motion.state.side_effect=states
        survey.revisit_policy.plan=Mock(return_value=dict(checkpoint=other,goals=[],estimated_seconds=40.))
        survey.revisit_policy.prove=Mock(return_value=True)
        def dispatch(decision,turn=False):
            self.assertTrue(terminal[0]);self.assertTrue(turn)
            self.assertEqual((decision['x'],decision['y']),(other['x'],other['y']))
            self.assertNotEqual((decision['x'],decision['y']),(saved['x'],saved['y']))
            return True
        survey.execute=Mock(side_effect=dispatch)
        with patch.object(survey,'healthy',return_value=True),patch('autonomous_survey.time.sleep'):
            self.assertTrue(survey.perform_localization_revisit())
        survey.execute.assert_called_once();self.assertEqual(survey.revisit_policy.attempts,2)
        self.assertEqual(survey.data['localization_revisits'][0]['status'],'checkpoint_retired')
        record=survey.data['localization_revisits'][1]
        self.assertEqual(record['trigger'],trigger)
        self.assertEqual(record['old_action_terminal']['request_id'],'return-id')
        self.assertEqual(record['old_action_terminal']['phase'],'cancelled')
        self.assertEqual(record['cancellation']['token_fingerprint'],hashlib.sha256(b'return-token').hexdigest()[:16])

    def test_generic_value_error_is_not_a_retirement_restart(self):
        survey,motion,state,trigger,*_=self.fixture()
        survey.revisit_policy.observe=Mock(side_effect=ValueError('unexpected quality or geometry failure'))
        with self.assertRaisesRegex(ValueError,'unexpected quality'):survey.update_localization_revisit()
        self.assertTrue(survey.revisit_policy.running);self.assertIs(survey.revisit_policy.pending['trigger'],trigger)
        motion.cancel.assert_not_called()

    def test_real_outage_takes_priority_over_simultaneous_retirement(self):
        survey,motion,state,trigger,saved,other,value=self.fixture();value['fault_counter']=1
        with self.assertRaisesRegex(ValueError,'outage'):survey.update_localization_revisit()
        self.assertTrue(survey.revisit_policy.running)
        self.assertEqual(survey.data['localization_revisits'][0]['status'],'returning')
        motion.cancel.assert_not_called()

    def test_cumulative_attempt_bound_is_not_reset_by_retirement(self):
        survey,motion,state,trigger,*_=self.fixture();survey.update_localization_revisit()
        survey.revisit_policy.attempts=MAX_REVISITS
        with patch.object(survey,'stop') as stop,patch.object(survey,'_perform_localization_revisit_attempt') as attempt:
            self.assertFalse(survey.perform_localization_revisit())
        attempt.assert_not_called();stop.assert_called_once_with('定位回访的有界尝试次数已用尽，已停止','blocked')
        self.assertIs(survey.revisit_policy.pending['trigger'],trigger)

    def test_aborted_return_terminal_faults_without_replacement_dispatch(self):
        survey,motion,state,trigger,*_=self.fixture();survey.update_localization_revisit()
        motion.state.return_value=dict(state,active=False,phase='failed',error='Nav2 aborted')
        survey.revisit_policy.plan=Mock()
        with patch.object(survey,'healthy',return_value=True),patch.object(survey,'stop') as stop:
            self.assertFalse(survey.perform_localization_revisit())
        stop.assert_called_once_with('Nav2 aborted','fault')
        survey.revisit_policy.plan.assert_not_called()
        motion.start.assert_not_called();motion.turn.assert_not_called();motion.pan_tilt.assert_not_called()
        self.assertIs(survey.revisit_policy.pending['trigger'],trigger)

    def test_retired_during_gimbal_waits_same_bounded_command_not_fake_terminal(self):
        survey,motion,state,trigger,*_=self.fixture('gimbal');survey.update_localization_revisit()
        motion.cancel.assert_not_called();self.assertFalse(survey.revisit_interrupt_pending())
        self.assertTrue(survey.revisit_policy.pending['wait_for_gimbal'])
        motion.state.return_value=dict(state,active=False,phase='succeeded')
        survey.update_localization_revisit()
        self.assertTrue(survey.revisit_interrupt_pending())
        motion.pan_tilt.assert_not_called();self.assertIs(survey.revisit_policy.pending['trigger'],trigger)


if __name__=='__main__':unittest.main()
