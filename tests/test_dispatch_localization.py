"""Real dispatch guards: refused localization never opens a motion lease."""
from pathlib import Path
import sys,threading,unittest
from types import SimpleNamespace
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from experiment_motion import ExperimentMotion
from localization_quality import LocalizationDispatchRejected
class DispatchTests(unittest.TestCase):
    def stub(self):
        health=dict(required=True,healthy=False,reason='observation_stale',session_id='sensor-session')
        state=dict(ready=False,pose=None,diagnostics=dict(laser_localization=health,pose_error='激光定位质量失效：observation_stale'))
        motion=SimpleNamespace(lock=threading.RLock(),localization_lock=threading.RLock(),active=False,phase='idle',
            state=lambda:state,localization_state=lambda:health,_localization_state_locked=lambda:health,
            spin_client=Mock(),activate_localization_action=Mock(),get_clock=Mock())
        motion.spin_client.server_is_ready.return_value=True
        return motion,state
    def test_turn_move_and_gimbal_dispatch_identify_stale_pose_before_lease(self):
        for action in ('turn','move','gimbal'):
            with self.subTest(action=action):
                m,_=self.stub()
                with self.assertRaises(LocalizationDispatchRejected) as error:
                    if action=='turn':ExperimentMotion.turn(m,.2,dict(id='test'))
                    elif action=='move':ExperimentMotion.start(m,dict(experiment='test',goal=[1.,0.]),dict(id='test'))
                    else:ExperimentMotion.pan_tilt(m,0.,-.2)
                self.assertEqual(error.exception.session_id,'sensor-session');self.assertFalse(m.active)
                m.activate_localization_action.assert_not_called()
    def test_final_activation_and_pose_checks_preserve_typed_rejection(self):
        m,_=self.stub()
        for method in (ExperimentMotion.pose,ExperimentMotion.activate_localization_action):
            with self.assertRaises(LocalizationDispatchRejected):method(m)
            self.assertFalse(m.active)
    def test_busy_or_unavailable_spin_is_not_relabelled_as_localization(self):
        m,state=self.stub();m.active=True
        with self.assertRaises(ValueError) as error:ExperimentMotion.turn(m,.2,{})
        self.assertNotIsInstance(error.exception,LocalizationDispatchRejected)
        m.active=False;state['ready']=True;m.spin_client.server_is_ready.return_value=False
        with self.assertRaises(ValueError) as error:ExperimentMotion.turn(m,.2,{})
        self.assertNotIsInstance(error.exception,LocalizationDispatchRejected)
