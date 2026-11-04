import ast
import math
from pathlib import Path
import sys
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
ROOT=next(parent for parent in Path(__file__).resolve().parents if (parent/'scripts/experiment_motion.py').exists())
sys.path.insert(0,str(ROOT/'scripts'))
from autonomous_survey import AutonomousSurvey
from localization_revisit import sample_payload


class FakeTime:
    def __init__(self,*,seconds):self.nanoseconds=round(seconds*1e9)


def original_within_region():
    # Execute the unchanged original pure helper without importing ROS nodes.
    tree=ast.parse((ROOT/'scripts/experiment_motion.py').read_text(encoding='utf-8-sig'))
    function=next(node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name=='within_region')
    namespace=dict(math=math)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),'original_within_region','exec'),namespace)
    return namespace['within_region']


def sample(std=(.026,.026,.03),stamp=10.):
    payload=dict(localization_backend='metric_rgbd_3d',session_id='session',sequence=100,healthy=True,
                 accepted_stamp=stamp,last_fit=dict(accepted=True,exposure_stamp=stamp,reference_id=66,position_std_m=list(std)))
    return sample_payload(payload,dict(healthy=True,session_id='session',sequence=100,fault_counter=0))


class RevisitIntegrationTests(unittest.TestCase):
    def setUp(self):
        # These contracts need only the Time value and existing region helper;
        # no ROS process, callback thread or runtime installation is required.
        self.modules=patch.dict(sys.modules,{'rclpy.time':NS(Time=FakeTime),
            'experiment_motion':NS(within_region=original_within_region())})
        self.modules.start();self.addCleanup(self.modules.stop)

    def fixture(self,kind='navigate_to_pose'):
        motion=Mock();motion.lock=threading.RLock();motion.localization_lock=threading.RLock()
        motion.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
        motion.token='old-token';motion.sim_started=9.;motion.sim_budget=5.;motion.gimbal_deadline=time.monotonic()+30.
        pose=dict(x=0.,y=0.,yaw=0.)
        state=dict(active=True,phase='executing',pose=pose,
                   diagnostics=dict(goal_request_id='old-id',goal_request_kind=kind))
        motion.state.return_value=state;motion.pose.return_value=pose
        motion.grid=NS(info=NS(width=20,height=20,resolution=.25,origin=NS(position=NS(x=-2.5,y=-2.5))),data=[0]*400)
        motion.grid_fresh.return_value=True
        survey=AutonomousSurvey(motion);survey.origin=[0.,0.];survey.task=dict(id='test',region=[-15,-15,15,15])
        survey.data=dict(active=True,started=0.,budget_seconds=300.,radius=22.,history=[])
        survey.survey_action_identity=('old-token','old-id')
        survey.revisit_sample=Mock(return_value=sample());survey.revisit_observation=Mock(return_value=None)
        return survey,motion,state

    def test_nav_warn_requests_cancel_but_does_not_treat_ack_as_terminal(self):
        survey,motion,state=self.fixture()
        survey.update_localization_revisit()
        motion.cancel.assert_called_once()
        self.assertTrue(survey.revisit_interrupt_pending())
        self.assertEqual(survey.revisit_policy.pending['cancel_request_id'],'old-id')
        motion.start.assert_not_called();motion.turn.assert_not_called();motion.pan_tilt.assert_not_called()
        self.assertTrue(state['active'])
        self.assertEqual(state['phase'],'executing')

    def test_new_revisit_dispatch_waits_same_old_id_terminal_including_natural_success_race(self):
        for terminal in ('cancelled','succeeded'):
            with self.subTest(terminal=terminal):
                survey,motion,state=self.fixture();survey.update_localization_revisit()
                terminal_seen=[False];calls=[0]
                def states():
                    calls[0]+=1
                    if calls[0]<3:return dict(state,phase='cancelling')
                    terminal_seen[0]=True;return dict(state,active=False,phase=terminal)
                motion.state.side_effect=states
                low=sample((.012,.012,.014));checkpoint=dict(low,x=0.,y=0.,yaw=0.,pan=0.,tilt=-.26)
                survey.revisit_policy.plan=Mock(return_value=dict(checkpoint=checkpoint,goals=[],estimated_seconds=30.))
                survey.revisit_policy.prove=Mock(return_value=True)
                survey.revisit_last_sample=low
                def new_action(*args,**kwargs):
                    self.assertTrue(terminal_seen[0]);return True
                survey.execute=Mock(side_effect=new_action)
                with patch.object(survey,'healthy',return_value=True),patch('autonomous_survey.time.sleep'):
                    self.assertTrue(survey.perform_localization_revisit())
                survey.execute.assert_called_once()
                self.assertEqual(survey.data['localization_revisits'][0]['old_action_terminal']['phase'],terminal)
                self.assertIsNone(survey.revisit_policy.pending)

    def test_same_cancel_ack_without_terminal_times_out_without_new_goal(self):
        survey,motion,state=self.fixture();survey.update_localization_revisit()
        motion.state.return_value=dict(state,phase='cancelling')
        with patch.object(survey,'healthy',return_value=True),patch.object(survey,'stop') as stop,patch('autonomous_survey.time.monotonic',side_effect=[0.,11.]):
            self.assertFalse(survey.perform_localization_revisit())
        motion.start.assert_not_called();motion.turn.assert_not_called();motion.pan_tilt.assert_not_called()
        stop.assert_called_once_with('原动作尚未真实结束，未派发回访目标','blocked')

    def test_unrelated_request_cannot_be_cancelled_or_reused_by_policy(self):
        survey,motion,state=self.fixture();survey.survey_action_identity=('other-token','other-id')
        with self.assertRaisesRegex(ValueError,'another request'):survey.update_localization_revisit()
        motion.cancel.assert_not_called();motion.start.assert_not_called()

    def test_pending_gimbal_keeps_original_bounded_command_until_natural_success(self):
        survey,motion,state=self.fixture('gimbal');survey.update_localization_revisit()
        motion.cancel.assert_not_called();motion.pan_tilt.assert_not_called()
        self.assertFalse(survey.revisit_interrupt_pending())
        self.assertTrue(survey.revisit_policy.pending['wait_for_gimbal'])
        self.assertEqual(survey.data['localization_revisit_pending']['remaining_bounded_gimbal_sim_s'],4.)
        motion.state.return_value=dict(state,active=False,phase='succeeded')
        survey.update_localization_revisit()
        self.assertTrue(survey.revisit_interrupt_pending())
        motion.cancel.assert_not_called();motion.pan_tilt.assert_not_called()

    def test_pending_gimbal_cannot_wait_beyond_original_bound(self):
        survey,motion,state=self.fixture('gimbal');survey.update_localization_revisit()
        motion.get_clock.return_value.now.return_value.nanoseconds=15_000_000_001
        with self.assertRaisesRegex(ValueError,'bounded gimbal'):survey.update_localization_revisit()
        motion.cancel.assert_not_called()

    def test_payload_snapshot_holds_original_consumer_lock(self):
        survey,motion,state=self.fixture()
        del survey.revisit_sample
        consumer=NS(payload=dict(localization_backend='direct_metric_laser'))
        motion.localization_quality=consumer
        seen=[]
        def health():
            seen.append(motion.localization_lock._is_owned());return dict(healthy=True)
        motion._localization_state_locked.side_effect=health
        self.assertIsNone(survey.revisit_sample());self.assertEqual(seen,[True])

    def test_actual_same_exposure_tf_and_joint_axis_sign_not_latest_values(self):
        survey,motion,state=self.fixture();del survey.revisit_observation
        motion.gimbal_joint_stamp=10.;motion.gimbal_joint_received=time.monotonic()
        queried=[]
        def transform(parent,child,query):
            stamp=query.nanoseconds/1e9;queried.append((parent,child,stamp))
            q=NS(x=0.,y=0.,z=0.,w=1.)
            if child=='base_footprint':q.z=math.sin(.3/2);q.w=math.cos(.3/2)
            elif child=='camera_pan_link':q.z=math.sin(.4/2);q.w=math.cos(.4/2)
            elif child=='camera_tilt_link':q.y=math.sin(.25/2);q.w=math.cos(.25/2)
            return NS(header=NS(stamp=NS(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))),
                      transform=NS(rotation=q,translation=NS(x=.55,y=.46,z=.01)))
        motion.buffer.lookup_transform.side_effect=transform
        row=survey.revisit_observation(9.9)
        self.assertEqual([q[2] for q in queried],[9.9]*3)
        self.assertAlmostEqual(row['yaw'],.3);self.assertAlmostEqual(row['pan'],.4);self.assertAlmostEqual(row['tilt'],-.25)
        motion.gimbal_joint_stamp=9.8
        self.assertIsNone(survey.revisit_observation(9.9))
        motion.gimbal_joint_stamp=10.;motion.gimbal_joint_received=time.monotonic()-2.1
        self.assertIsNone(survey.revisit_observation(9.9))

    def test_tf_latest_timestamp_cannot_be_attached_to_past_covariance(self):
        survey,motion,state=self.fixture();del survey.revisit_observation
        motion.gimbal_joint_stamp=10.;motion.gimbal_joint_received=time.monotonic()
        motion.buffer.lookup_transform.return_value=NS(header=NS(stamp=NS(sec=10,nanosec=0)))
        self.assertIsNone(survey.revisit_observation(9.9))


class UnknownClearanceIntegrationTests(unittest.TestCase):
    setUp=RevisitIntegrationTests.setUp
    fixture=RevisitIntegrationTests.fixture

    def scan_fixture(self, freshness='new'):
        survey,motion,state=self.fixture()
        survey.save=Mock();survey.stop=Mock();survey.update_localization_revisit()
        state.update(active=False,phase='cancelled')
        motion.gimbal_position=[0.,-.26];motion.gimbal_command_rate_rad_s=.35
        motion.gimbal_joint_stamp=10.;motion.gimbal_joint_received=time.monotonic()
        motion.grid.header=NS(stamp=NS(sec=9,nanosec=0))
        motion.grid_received=0.;motion.quality=dict(stamp=9.,frames=10)
        clock=[10.];survey.now=Mock(side_effect=lambda:clock[0])
        scan=dict(checkpoint=dict(reference_id=66,stamp=9.),pan=.1,tilt=-.26,
                  unknown_cells=[dict(x=1.,y=0.,cell=[14,10])],estimated_seconds=7.,
                  return_reserve_seconds=30.,fresh_wait_sim_s=8.,fresh_wait_wall_s=15.)
        survey.revisit_policy.unknown_clearance_scan=Mock(return_value=scan)
        record=dict(trigger=dict(survey.revisit_policy.pending['trigger']),
                    old_action_terminal=dict(request_id='old-id',phase='cancelled'))
        survey.data['localization_revisits']=[record]
        def pan_tilt(*args):
            decision=survey.revisit_preplan_guard['decision']
            self.assertFalse(survey.revisit_policy.running)
            self.assertTrue(survey.revisit_interrupt_pending())
            self.assertTrue(survey.preplan_observation_allowed(decision))
            self.assertFalse(survey.preplan_observation_allowed(dict(decision)))
            motion.token='scan-token'
            state.update(active=False,phase='succeeded',diagnostics=dict(
                goal_request_id='scan-id',goal_request_kind='gimbal'))
            return dict(token='scan-token')
        motion.pan_tilt.side_effect=pan_tilt
        def healthy():
            evidence=record.get('preplan_unknown_observation',{})
            if evidence.get('status')=='waiting_new_observations':
                if freshness=='fault':return False
                survey.revisit_last_sample=sample(stamp=11.)
                motion.quality=dict(stamp=11.,frames=11 if freshness!='same_frames' else 10)
                motion.grid.header.stamp=NS(sec=11,nanosec=0)
                motion.grid_received=time.monotonic()+1.
                if freshness=='old_map':motion.grid.header.stamp=NS(sec=10,nanosec=0)
                if freshness=='old_received':motion.grid_received=0.
                if freshness!='new':clock[0]=19.
            return True
        survey.healthy=Mock(side_effect=healthy)
        return survey,motion,record

    def test_exact_owned_pending_gimbal_then_real_new_frames_map_and_acceptance(self):
        survey,motion,record=self.scan_fixture()
        with patch('autonomous_survey.time.sleep'):
            self.assertTrue(survey.observe_unknown_revisit_clearance(record))
        evidence=record['preplan_unknown_observation']
        self.assertEqual(evidence['status'],'new_observations_received')
        self.assertEqual((evidence['before_quality_frames'],evidence['new_quality_frames']),(10,11))
        self.assertGreater(evidence['new_accepted_stamp'],evidence['completed_sim'])
        self.assertGreater(evidence['new_quality_stamp'],evidence['completed_sim'])
        self.assertGreater(evidence['new_map_header_stamp'],evidence['completed_sim'])
        self.assertEqual(evidence['request_id'],'scan-id')
        motion.pan_tilt.assert_called_once();motion.turn.assert_not_called();motion.start.assert_not_called()
        survey.stop.assert_not_called();self.assertIsNone(survey.revisit_preplan_guard)

    def test_restamped_statistics_or_old_map_do_not_satisfy_freshness(self):
        for freshness in ('same_frames','old_map','old_received'):
            with self.subTest(freshness=freshness):
                survey,motion,record=self.scan_fixture(freshness)
                with patch('autonomous_survey.time.sleep'):
                    self.assertFalse(survey.observe_unknown_revisit_clearance(record))
                self.assertEqual(record['preplan_unknown_observation']['status'],'fresh_observation_timeout')
                self.assertEqual(survey.stop.call_args.args[1],'blocked')
                motion.pan_tilt.assert_called_once();self.assertIsNone(survey.revisit_preplan_guard)

    def test_once_per_frozen_trigger_survives_record_replacement_and_no_pan_tilt_fallback(self):
        survey,motion,record=self.scan_fixture()
        with patch('autonomous_survey.time.sleep'):
            self.assertTrue(survey.observe_unknown_revisit_clearance(record))
            replacement=dict(trigger=dict(record['trigger']),old_action_terminal=record['old_action_terminal'])
            self.assertFalse(survey.observe_unknown_revisit_clearance(replacement))
        motion.pan_tilt.assert_called_once()
        other,motion,record=self.scan_fixture();motion.pan_tilt=None
        self.assertFalse(other.observe_unknown_revisit_clearance(record))
        motion.turn.assert_not_called();motion.start.assert_not_called()
        self.assertNotIn('preplan_unknown_observation',record)

    def test_quality_fault_prevents_dispatch_and_cannot_be_rescued_by_fresh_map(self):
        survey,motion,record=self.scan_fixture();survey.healthy=Mock(return_value=False)
        self.assertFalse(survey.observe_unknown_revisit_clearance(record))
        motion.pan_tilt.assert_not_called();motion.turn.assert_not_called();motion.start.assert_not_called()
        self.assertEqual(survey.stop.call_args.args[1],'blocked')
        self.assertIsNone(survey.revisit_preplan_guard)


if __name__=='__main__':unittest.main()
