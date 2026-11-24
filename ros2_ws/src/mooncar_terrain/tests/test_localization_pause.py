"""Mission recovery keeps the real motion gate closed until fresh readiness."""
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from autonomous_survey import AutonomousSurvey,LOCALIZATION_STOP_REASON
from localization_quality import LocalizationDispatchRejected,require_dispatch_ready


class Motion:
    def __init__(self):
        self.lock=threading.RLock();self.sim=0.;self.step=0;self.dispatches=[];self.cancelled=[];self.beats=[]
        self.quality={};self.quality_received=time.monotonic();self.active=False;self.inject=False
        self.changed_session=False;self.persistent=False;self.bad_reason=None
    def get_clock(self):return SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=round(self.sim*1e9)))
    def grid_fresh(self):return True
    def state(self):
        bad=self.inject and (self.step==0 or self.persistent)
        return dict(active=self.active,ready=not bad,phase='executing' if self.active else 'succeeded',
            error=LOCALIZATION_STOP_REASON if self.cancelled else '',pose=dict(x=0.,y=0.,yaw=0.),
            diagnostics=dict(goal_request_id='request-'+str(len(self.dispatches)),
                laser_localization=dict(required=True,healthy=not bad,reason=(self.bad_reason or 'recovering') if bad else 'healthy',
                    session_id='changed' if self.changed_session and self.step else 'same',fault_counter=1 if self.inject else 0,
                    ready_since_stamp=self.sim if not bad else None)))
    def cancel(self,reason):self.cancelled.append(reason);self.active=False
    def start(self,body,task):
        self.dispatches.append('token-'+str(len(self.dispatches)+1));self.inject=True
        self.active=len(self.dispatches)==1
        return dict(token=self.dispatches[-1])
    def heartbeat(self,body):self.beats.append(body['token'])


class PauseTests(unittest.TestCase):
    def test_dispatch_time_localization_rejection_waits_outside_locks_then_retries(self):
        survey,motion=self.fixture();calls=[]
        def turn(yaw,task):
            calls.append(yaw)
            if len(calls)==1:
                motion.inject=True
                raise LocalizationDispatchRejected(dict(reason='observation_stale',session_id='same'))
            motion.dispatches.append('accepted-turn');return dict(token='accepted-turn')
        motion.turn=turn
        def advance(_):
            self.assertFalse(motion.lock._is_owned())
            self.assertFalse(survey.lock._is_owned())
            motion.step+=1;motion.sim+=.1
        with patch('autonomous_survey.time.sleep',side_effect=advance):
            self.assertTrue(survey.execute(dict(yaw=.4,estimated_seconds=1.),turn=True))
        self.assertEqual(len(calls),2);self.assertEqual(motion.dispatches,['accepted-turn'])
        self.assertEqual(survey.data['localization_pauses'][0]['outcome'],'recovered')
        self.assertTrue(motion.cancelled)

    def test_hard_or_other_session_dispatch_refusal_never_retries(self):
        for reason,session in [('clock_moved_backwards','same'),('observation_stale','old')]:
            with self.subTest(reason=reason,session=session):
                survey,motion=self.fixture()
                motion.turn=lambda *args:(_ for _ in ()).throw(LocalizationDispatchRejected(dict(reason=reason,session_id=session)))
                self.assertFalse(survey.execute(dict(yaw=.2,estimated_seconds=1.),turn=True))
                self.assertEqual(survey.data['phase'],'fault');self.assertFalse(motion.dispatches)

    def test_nonlocalization_dispatch_errors_remain_errors(self):
        survey,motion=self.fixture()
        motion.turn=lambda *args:(_ for _ in ()).throw(ValueError('等待导航结束及转向服务就绪'))
        with self.assertRaises(ValueError):survey.execute(dict(yaw=.2,estimated_seconds=1.),turn=True)
        self.assertFalse(survey.data.get('localization_pauses'))

    def test_readiness_helper_distinguishes_localization_from_sensor_failure(self):
        state=dict(ready=False,pose=None,diagnostics=dict(laser_localization=dict(required=True,healthy=False,reason='recovering',session_id='same')))
        with self.assertRaises(LocalizationDispatchRejected):require_dispatch_ready(state,'not ready')
        state['diagnostics']['laser_localization']['healthy']=True
        with self.assertRaises(ValueError) as error:require_dispatch_ready(state,'sensor not ready')
        self.assertNotIsInstance(error.exception,LocalizationDispatchRejected)
    def fixture(self):
        motion=Motion();survey=AutonomousSurvey(motion)
        survey.origin=[0.,0.];survey.task=dict(id='mission');survey.wall_started=time.monotonic()
        survey.data=dict(active=True,phase='moving',started=0.,budget_seconds=120.,radius=8.,history=[])
        survey.update_localization_revisit=lambda:None
        return survey,motion
    def advance(self,motion):
        def sleep(_):motion.step+=1;motion.sim+=.1
        return sleep
    def test_interrupted_action_gets_new_lease_only_after_recovery(self):
        survey,motion=self.fixture()
        with patch('autonomous_survey.time.sleep',side_effect=self.advance(motion)):
            self.assertTrue(survey.execute(dict(x=1.,y=0.,estimated_seconds=1.)))
        self.assertEqual(motion.dispatches,['token-1','token-2'])
        self.assertNotIn('token-1',motion.beats)
        self.assertEqual(motion.beats,['token-2'])
        self.assertEqual(survey.data['localization_pauses'][0]['outcome'],'recovered')
        self.assertTrue(survey.data['active'])
    def test_wait_does_not_dispatch_motion(self):
        survey,motion=self.fixture();motion.inject=True
        with patch('autonomous_survey.time.sleep',side_effect=self.advance(motion)):
            self.assertTrue(survey.wait_for_localization(motion.state()))
        self.assertFalse(motion.dispatches);self.assertFalse(motion.beats)
    def test_manual_stop_during_wait_does_not_resume(self):
        survey,motion=self.fixture();motion.inject=True
        with patch('autonomous_survey.time.sleep',side_effect=lambda _:survey.stop('user stop')):
            self.assertFalse(survey.wait_for_localization(motion.state()))
        self.assertFalse(survey.data['active']);self.assertFalse(motion.dispatches)
        self.assertEqual(survey.data['localization_pauses'][0]['outcome'],'stopped')
    def test_producer_session_change_never_resumes_old_goal(self):
        survey,motion=self.fixture();motion.inject=True;motion.changed_session=True
        with patch('autonomous_survey.time.sleep',side_effect=self.advance(motion)):
            self.assertFalse(survey.wait_for_localization(motion.state()))
        self.assertEqual(survey.data['phase'],'fault');self.assertFalse(motion.dispatches)
    def test_hard_fault_is_not_treated_as_transient(self):
        survey,motion=self.fixture();motion.inject=True;motion.bad_reason='clock_moved_backwards'
        self.assertFalse(survey.wait_for_localization(motion.state()))
        self.assertEqual(survey.data['phase'],'fault');self.assertNotIn('localization_pauses',survey.data)
    def test_wait_uses_mission_budget(self):
        survey,motion=self.fixture();motion.inject=True;motion.persistent=True
        with patch('autonomous_survey.time.sleep',side_effect=lambda _:setattr(motion,'sim',120.)):
            self.assertFalse(survey.wait_for_localization(motion.state()))
        self.assertEqual(survey.data['phase'],'budget_reached')
    def test_dead_producer_cannot_wait_forever(self):
        survey,motion=self.fixture();motion.inject=True
        with patch('autonomous_survey.LOCALIZATION_RECOVERY_WALL_SECONDS',0.):
            self.assertFalse(survey.wait_for_localization(motion.state()))
        self.assertEqual(survey.data['localization_pauses'][0]['outcome'],'timeout')

    def test_health_loss_between_initial_and_locked_snapshot_recovers(self):
        survey,motion=self.fixture();original=motion.state;calls=0
        def changing_state():
            nonlocal calls
            calls+=1
            if calls==2:motion.inject=True
            value=original()
            if not value['diagnostics']['laser_localization']['healthy']:
                value['pose']=None
                value['diagnostics']['pose_error']='激光定位质量失效：recovering'
            return value
        motion.state=changing_state
        with patch('autonomous_survey.time.sleep',side_effect=self.advance(motion)):
            self.assertTrue(survey.healthy())
        self.assertTrue(survey.data['active'])
        self.assertEqual(survey.data['localization_pauses'][0]['outcome'],'recovered')
        self.assertTrue(motion.cancelled);self.assertFalse(motion.dispatches)

    def test_pose_rejected_before_health_snapshot_cannot_bypass_wait(self):
        survey,motion=self.fixture();original=motion.state;calls=0
        def changing_state():
            nonlocal calls
            calls+=1;value=original()
            if calls==2:
                value['pose']=None;value['ready']=False
                value['diagnostics']['pose_error']='激光定位质量失效：observation_stale'
                self.assertTrue(value['diagnostics']['laser_localization']['healthy'])
            return value
        motion.state=changing_state
        self.assertTrue(survey.healthy())
        self.assertTrue(survey.data['active'])
        self.assertTrue(motion.cancelled);self.assertFalse(motion.dispatches)
        self.assertEqual(survey.data['localization_pauses'][0]['outcome'],'recovered')

    def test_hard_fault_on_locked_snapshot_still_stops(self):
        survey,motion=self.fixture();original=motion.state;calls=0
        def changing_state():
            nonlocal calls
            calls+=1
            if calls==2:motion.inject=True;motion.bad_reason='clock_moved_backwards'
            return original()
        motion.state=changing_state
        self.assertFalse(survey.healthy());self.assertEqual(survey.data['phase'],'fault')
        self.assertFalse(motion.dispatches)


if __name__=='__main__':unittest.main()
