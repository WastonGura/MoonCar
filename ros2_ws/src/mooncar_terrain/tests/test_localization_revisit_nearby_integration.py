"""New parking goals never replace saved real localization observations."""
import copy
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock,patch

ROOT=next(parent for parent in Path(__file__).resolve().parents if (parent/'scripts/experiment_motion.py').exists())
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'ros2_ws/src/mooncar_terrain/tests')]
import test_localization_revisit_integration as original_actor


class NearbyRevisitIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.original=original_actor.RevisitIntegrationTests()
        self.original.setUp();self.addCleanup(self.original.doCleanups)

    def fixture(self):
        survey,motion,state=self.original.fixture();survey.update_localization_revisit()
        motion.state.return_value=dict(state,active=False,phase='cancelled')
        low=original_actor.sample((.012,.012,.014),5.)
        saved=dict(low,x=.4,y=0.,yaw=.1,pan=.2,tilt=-.26)
        standpoint=dict(x=.44,y=0.,kind='near_saved_observation')
        plan=dict(checkpoint=saved,planned_standpoint=standpoint,goals=[standpoint],estimated_seconds=45.)
        survey.revisit_policy.plan=Mock(return_value=plan)
        survey.revisit_policy.prove=Mock(return_value=True)
        survey.revisit_last_sample=low
        return survey,motion,saved,standpoint,plan

    def test_dispatch_uses_new_standpoint_and_retains_saved_actual_view(self):
        survey,motion,saved,standpoint,plan=self.fixture();before=copy.deepcopy(saved)
        def execute(decision,turn=False):
            if not turn:
                self.assertEqual((decision['x'],decision['y']),(standpoint['x'],standpoint['y']))
                self.assertNotEqual(decision['x'],saved['x'])
                motion.pose.return_value=dict(x=standpoint['x'],y=0.,yaw=0.)
            else:
                self.assertTrue(decision['use_pan_tilt']);self.assertAlmostEqual(decision['pan'],saved['yaw']+saved['pan'])
                self.assertEqual(decision['tilt'],saved['tilt'])
            return True
        survey.execute=Mock(side_effect=execute)
        with patch.object(survey,'healthy',return_value=True):
            self.assertTrue(survey.perform_localization_revisit())
        self.assertEqual(saved,before);self.assertEqual(survey.execute.call_count,2)
        self.assertEqual(set(standpoint),{'x','y','kind'})
        self.assertIs(survey.data['localization_revisits'][0]['plan']['checkpoint'],saved)
        survey.revisit_policy.prove.assert_called_once()

    def test_actual_pose_still_must_be_within_precise_saved_pose_radius(self):
        survey,motion,saved,standpoint,plan=self.fixture()
        standpoint['x']=.5
        def execute(decision,turn=False):
            self.assertFalse(turn);motion.pose.return_value=dict(x=.5,y=0.,yaw=0.);return True
        survey.execute=Mock(side_effect=execute)
        with patch.object(survey,'healthy',return_value=True),patch.object(survey,'stop') as stop:
            self.assertFalse(survey.perform_localization_revisit())
        survey.execute.assert_called_once();survey.revisit_policy.prove.assert_not_called()
        stop.assert_called_once_with('回访未到达保存的真实观察位置，已停止','blocked')

    def test_current_target_disk_rechecked_even_when_segment_would_pass(self):
        survey,motion,saved,standpoint,plan=self.fixture();survey.execute=Mock()
        with patch.object(survey,'healthy',return_value=True),patch.object(survey,'stop') as stop,\
                patch('autonomous_survey.known_chassis_clearance',return_value=False) as disk,\
                patch('localization_revisit.safe_segment',return_value=True) as segment:
            self.assertFalse(survey.perform_localization_revisit())
        disk.assert_called_once();segment.assert_not_called();survey.execute.assert_not_called()
        survey.revisit_policy.prove.assert_not_called()
        stop.assert_called_once_with('回访路线的当前已观测安全条件已改变，已停止','blocked')


if __name__=='__main__':unittest.main()
