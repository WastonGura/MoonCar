"""Bounded initial observations use normal leased gimbal actions, never grid edits."""
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from autonomous_survey import AutonomousSurvey,bounded_gimbal_targets,initial_depth_connectivity


def grid(connected=False):
    values=np.full((41,41),-1,dtype=int);values[18:23,18:23]=0
    if connected:values[:]=0
    return NS(info=NS(width=41,height=41,resolution=.1,origin=NS(position=NS(x=-2.05,y=-2.05))),
              data=values.ravel().tolist(),header=NS(stamp=NS(sec=1,nanosec=0)))


class InitialDepthScanTests(unittest.TestCase):
    def survey(self,connected=False,observations=True,connect_after=None):
        motion=Mock();motion.lock=threading.RLock();motion.gimbal_position=[0.,0.]
        motion.grid=grid(connected);motion.grid_fresh.return_value=True
        motion.quality=dict(frames=3,stamp=1.,cells=[],resolution=.25);motion.quality_received=time.monotonic()
        motion.pose.return_value=dict(x=0.,y=0.,yaw=0.)
        motion.state.return_value=dict(active=False,ready=True,pose=motion.pose.return_value,phase='succeeded',error='')
        clock=[1.]
        motion.get_clock.return_value.now.side_effect=lambda:NS(nanoseconds=round(clock[0]*1e9))
        motion.pan_tilt.return_value=dict(token='gimbal')
        survey=AutonomousSurvey(motion,initial_depth_scan=True);survey.origin=[0.,0.];survey.task=dict(id='test')
        survey.data=dict(active=True,phase='planning',started=0.,budget_seconds=180.,radius=6.,history=[])
        survey.wall_started=time.monotonic();survey.save=Mock()
        count=[0]
        def scan(pan,tilt,task):
            count[0]+=1;clock[0]+=.1;motion.gimbal_position=[pan,tilt]
            if observations:
                motion.quality=dict(motion.quality,frames=motion.quality['frames']+1,stamp=clock[0])
                motion.grid.header.stamp=NS(sec=10+count[0],nanosec=0)
            if connect_after is not None and count[0]>=connect_after:
                motion.grid.data=grid(True).data
            return dict(token='gimbal')
        motion.pan_tilt.side_effect=scan
        return survey,motion,clock

    def test_connected_actual_map_does_not_issue_any_scan(self):
        survey,motion,_=self.survey(connected=True)
        self.assertTrue(survey.initial_depth_observation())
        motion.pan_tilt.assert_not_called();motion.start.assert_not_called();motion.turn.assert_not_called()
        self.assertEqual(survey.data['initial_observation']['status'],'not_needed')

    def test_disconnected_map_scans_with_lease_and_replans_only_after_real_updates(self):
        survey,motion,_=self.survey(connect_after=1)
        self.assertTrue(survey.initial_depth_observation())
        motion.heartbeat.assert_called_once_with(dict(token='gimbal'))
        motion.start.assert_not_called();motion.turn.assert_not_called()
        self.assertEqual(survey.data['initial_observation']['status'],'connected')
        self.assertEqual(len(survey.data['initial_observation']['history']),1)
        self.assertEqual(survey.data['history'],[])  # Candidate repeat rejection stays untouched.
        self.assertEqual(survey.data['initial_observation']['history'][0]['outcome'],'succeeded')

    def test_quality_or_readiness_loss_does_not_start_gimbal(self):
        survey,motion,_=self.survey();motion.state.return_value['ready']=False
        self.assertFalse(survey.initial_depth_observation())
        motion.pan_tilt.assert_not_called();self.assertEqual(survey.data['phase'],'fault')
        motion.cancel.assert_called_once()

    def test_quality_loss_during_action_preserves_fault_and_never_advances_to_next_view(self):
        survey,motion,_=self.survey();original=motion.pan_tilt.side_effect
        def lost(*args):
            result=original(*args);motion.state.return_value['ready']=False
            return result
        motion.pan_tilt.side_effect=lost
        self.assertFalse(survey.initial_depth_observation())
        self.assertEqual(motion.pan_tilt.call_count,1);self.assertEqual(survey.data['phase'],'fault')
        self.assertEqual(survey.data['initial_observation']['status'],'fault')
        motion.start.assert_not_called();motion.turn.assert_not_called()

    def test_connected_after_downward_views_restores_normal_forward_view_before_replanning(self):
        survey,motion,_=self.survey(connect_after=3)
        self.assertTrue(survey.initial_depth_observation())
        self.assertEqual(motion.pan_tilt.call_count,4)
        self.assertAlmostEqual(motion.pan_tilt.call_args.args[0],0.)
        self.assertAlmostEqual(motion.pan_tilt.call_args.args[1],-math.radians(15))
        self.assertEqual(survey.data['phase'],'planning')
        motion.start.assert_not_called();motion.turn.assert_not_called()

    def test_new_quality_heartbeat_without_new_frames_and_grid_is_not_success(self):
        survey,motion,_=self.survey(observations=False)
        with patch('autonomous_survey.INITIAL_DEPTH_OBSERVATION_WALL_SECONDS',.001),patch('autonomous_survey.time.sleep'):
            self.assertFalse(survey.initial_depth_observation())
        self.assertEqual(motion.pan_tilt.call_count,1)
        self.assertEqual(survey.data['phase'],'blocked')
        self.assertEqual(survey.data['initial_observation']['history'][0]['outcome'],'no_new_observation')

    def test_budget_guard_prevents_dispatch(self):
        survey,motion,_=self.survey();survey.data['budget_seconds']=3.
        self.assertFalse(survey.initial_depth_observation())
        motion.pan_tilt.assert_not_called();self.assertEqual(survey.data['phase'],'budget_reached')

    def test_finite_scan_cannot_fill_unknown_or_loop_forever(self):
        survey,motion,_=self.survey();original=list(motion.grid.data)
        self.assertFalse(survey.initial_depth_observation())
        self.assertLess(motion.pan_tilt.call_count,40);self.assertGreater(motion.pan_tilt.call_count,3)
        self.assertEqual(motion.grid.data,original);self.assertEqual(survey.data['phase'],'blocked')
        motion.start.assert_not_called();motion.turn.assert_not_called()

    def test_joint_targets_are_bounded_including_return_from_opposite_pan(self):
        previous=(0.,0.)
        for target in bounded_gimbal_targets(previous,[(2.356,-math.radians(35)),(-2.356,-math.radians(35)),(0.,-math.radians(15))]):
            self.assertLessEqual(max(abs(target[i]-previous[i]) for i in (0,1)),math.pi/6+1e-12)
            previous=target

    def test_unknown_band_rejects_connectivity_instead_of_being_cleared(self):
        source=grid();source.data=np.full(41*41,-1,dtype=int).tolist()
        self.assertFalse(initial_depth_connectivity(source,dict(x=0.,y=0.))['connected'])
        self.assertTrue(initial_depth_connectivity(grid(True),dict(x=0.,y=0.))['connected'])

    def test_narrow_known_corridor_does_not_finish_bootstrap_without_chassis_clearance(self):
        source=grid();values=np.full((41,41),-1,dtype=int)
        values[19:22,20:35]=0  # Connected beyond .8m, narrower than the chassis margin.
        source.data=values.ravel().tolist();before=list(source.data)
        result=initial_depth_connectivity(source,dict(x=0.,y=0.))
        self.assertFalse(result['connected']);self.assertGreater(result['reachable_distance_m'],.8)
        self.assertEqual(source.data,before)

    def test_high_cost_targets_do_not_finish_bootstrap_even_with_known_clearance(self):
        source=grid(True);values=np.full((41,41),66,dtype=int);values[18:23,18:23]=0
        source.data=values.ravel().tolist()
        self.assertFalse(initial_depth_connectivity(source,dict(x=0.,y=0.))['connected'])


if __name__=='__main__':unittest.main(verbosity=2)
