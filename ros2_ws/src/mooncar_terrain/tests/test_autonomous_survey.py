"""Run with the existing Python/NumPy environment; no ROS needed."""
from pathlib import Path
import sys
import math
import tempfile
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from autonomous_survey import choose_view,AutonomousSurvey,resampling_value,STRATEGIES
from terrain_map import TerrainMap


class SurveyTests(unittest.TestCase):
    def test_unbounded_view_can_explore_beyond_old_start_radius(self):
        grid=NS(info=NS(width=120,height=120,resolution=.25,
                       origin=NS(position=NS(x=-15.,y=-15.))),data=[0]*14400)
        quality=dict(resolution=.25,cells=[])
        pose=dict(x=9.,y=0.,yaw=0.)
        self.assertIsNone(choose_view(grid,quality,pose,[0,0],8,[],100,'surface_view_v2'))
        choice=choose_view(grid,quality,pose,[0,0],None,[],100,'surface_view_v2')
        self.assertIsNotNone(choice)
        self.assertGreater(math.hypot(choice['x'],choice['y']),8)

    def test_start_without_radius_records_null_and_keeps_deadline(self):
        motion=Mock();motion.lock=threading.RLock()
        motion.get_clock.return_value.now.return_value.nanoseconds=0
        motion.quality={};motion.quality_received=__import__('time').monotonic()
        motion.state.return_value=dict(active=False,ready=True,pose=dict(x=0,y=0,yaw=0))
        motion.spin_client.server_is_ready.return_value=True
        with tempfile.TemporaryDirectory() as directory,patch('autonomous_survey.threading.Thread'):
            (Path(directory)/'test').mkdir()
            survey=AutonomousSurvey(motion)
            survey.start(dict(experiment='test',budget_seconds=180),dict(id='test',start=[0,0,0]),directory)
            self.assertIsNone(survey.data['radius'])
            self.assertIsNone(motion.mission_limits['radius'])
            self.assertEqual(motion.mission_limits['deadline'],180)
            import json
            saved=json.loads(survey.path.read_text(),parse_constant=lambda v: self.fail('Nonfinite JSON: '+v))
            self.assertIsNone(saved['radius'])

    def test_candidate_clearance_rejects_inscribed_cost(self):
        resolution=.25;width=80
        data=[0]*(width*width)
        grid=NS(info=NS(width=width,height=width,resolution=resolution,
                        origin=NS(position=NS(x=-10.,y=-10.))),data=data)
        quality=dict(resolution=resolution,cells=[dict(ix=x,iy=y,quality=.8)
                     for x in range(-32,32) for y in range(-32,32)
                     if not (24<=x<=28 and -2<=y<=2)])
        pose=dict(x=0.,y=0.,yaw=0.)
        clear=choose_view(grid,quality,pose,[0,0],8,[],100,'nearest_view_v2')
        self.assertEqual((clear['x'],clear['y']),(1.5,0.))
        data[42*width+46]=99  # Inscribed obstacle 0.5 m from that candidate.
        blocked=choose_view(grid,quality,pose,[0,0],8,[],100,'nearest_view_v2')
        self.assertNotEqual((blocked['x'],blocked['y']),(1.5,0.))

    def test_boundary_fallback_uses_known_free_goal_to_extend_online_map(self):
        grid=NS(info=NS(width=40,height=40,resolution=.25,origin=NS(position=NS(x=-5.,y=-5.))),data=[0]*1600)
        quality=dict(resolution=.25,cells=[dict(ix=x,iy=y,quality=.8) for x in range(-20,20) for y in range(-20,20)])
        choice,diagnostics=choose_view(grid,quality,dict(x=0.,y=0.,yaw=0.),[0,0],8,[],100,
                                       'coverage_view_v2',return_diagnostics=True)
        self.assertIsNotNone(choice)
        self.assertTrue(choice['map_boundary_fallback'])
        self.assertGreater(choice['expected_new_m2'],0)
        self.assertLess(abs(choice['x']),5.)
        self.assertLess(abs(choice['y']),5.)
        self.assertGreater(diagnostics['map_boundary_fallback']['feasible_moves'],0)

    def test_strategy_objectives_and_surface_recovery_gate(self):
        cell=dict(fit_state='too_few_voxels',bearing_mask=1,quality=.1)
        self.assertEqual(resampling_value(cell,(0,0),.25,(1,.125)),.5)
        cell['bearing_mask']=255
        self.assertEqual(resampling_value(cell,(0,0),.25,(1,.125)),0)
        for state in ('valid','steep_fit','nonplanar'):
            self.assertEqual(resampling_value(dict(cell,fit_state=state,bearing_mask=0),(0,0),.25,(1,0)),0)
        grid=NS(info=NS(width=80,height=80,resolution=.25,origin=NS(position=NS(x=-10.,y=-10.))),data=[0]*6400)
        quality=dict(resolution=.25,cells=[dict(ix=x,iy=y,quality=.1,fit_state='too_few_voxels',bearing_mask=0) for x in range(-40,40) for y in range(-40,40)])
        for strategy in STRATEGIES[:2]:
            self.assertIsNone(choose_view(grid,quality,dict(x=0,y=0,yaw=0),[0,0],8,[],60,strategy))
        self.assertGreater(choose_view(grid,quality,dict(x=0,y=0,yaw=0),[0,0],8,[],60,STRATEGIES[2])['expected_improvement_m2'],0)
        with self.assertRaises(ValueError):
            choose_view(grid,quality,dict(x=0,y=0,yaw=0),[0,0],8,[],60,'unknown')
        terrain=TerrainMap();terrain.add_frame(np.array([[.01,.01,0],[.08,.01,0],[.15,.01,0]]),1.)
        self.assertEqual(terrain.quality(with_surface=True)['cells'][0]['fit_state'],'too_few_voxels')

    def test_slow_recording_initialization_does_not_block_start_or_issue_motion(self):
        motion=Mock();motion.lock=threading.RLock()
        motion.get_clock.return_value.now.return_value.nanoseconds=0
        motion.quality={};motion.quality_received=__import__('time').monotonic()
        motion.state.return_value=dict(active=False,ready=True,pose=dict(x=0,y=0,yaw=0))
        motion.spin_client.server_is_ready.return_value=True
        survey=AutonomousSurvey(motion);release=threading.Event()
        def slow(_):
            release.wait(2)
            raise ValueError('intentional recording failure')
        with tempfile.TemporaryDirectory() as directory,patch('autonomous_survey.EvidenceRecorder',side_effect=slow):
            (Path(directory)/'test').mkdir()
            started=__import__('time').monotonic()
            response=survey.start(dict(experiment='test',budget_seconds=15,radius=6),dict(id='test',start=[0,0,0]),directory)
            self.assertTrue(response['accepted'])
            motion.check_start.assert_called_once_with(dict(id='test',start=[0,0,0]),dict(x=0,y=0,yaw=0))
            self.assertLess(__import__('time').monotonic()-started,1.)
            motion.start.assert_not_called();motion.turn.assert_not_called()
            release.set();survey.worker.join(3)
            self.assertFalse(survey.data['active']);self.assertEqual(survey.data['phase'],'fault')
            motion.start.assert_not_called();motion.turn.assert_not_called()

    def test_mismatched_start_cannot_create_autonomous_mission(self):
        motion=Mock();motion.lock=threading.RLock();motion.quality={}
        motion.quality_received=__import__('time').monotonic()
        motion.state.return_value=dict(active=False,ready=True,pose=dict(x=-1.4,y=1.55,yaw=0))
        motion.spin_client.server_is_ready.return_value=True
        motion.check_start.side_effect=ValueError('定位起点与本轮计划起点不一致')
        survey=AutonomousSurvey(motion)
        with self.assertRaisesRegex(ValueError,'定位起点与本轮计划起点不一致'):
            survey.start(dict(experiment='test',budget_seconds=90,radius=8),
                         dict(id='test',start=[6.5,-2.,2.4]),'.')
        self.assertFalse(survey.data['active'])
        self.assertIsNone(survey.worker)

    def test_quality_saturates_and_archive_replays_it(self):
        t=TerrainMap();xy=np.array([[x,y,0.] for x in np.arange(.01,.25,.0625) for y in np.arange(.01,.25,.0625)])
        t.add_frame(xy,1.,translation=(0,0,0));first=t.quality()['cells'][0]
        t.add_frame(xy,2.,translation=(0,0,0))
        self.assertEqual(t.quality()['cells'][0],first)
        t.add_frame(xy-[1,0,0],3.,translation=(1,0,0))
        self.assertGreater(t.quality()['cells'][0]['quality'],first['quality'])
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'test.npz';t.save(path)
            self.assertEqual(TerrainMap.load(path).quality(),t.quality())
        self.assertLessEqual(t.quality()['cells'][0]['quality'],1.)

    def test_candidate_visibility_budget_and_repeat_avoidance(self):
        grid=NS(info=NS(width=80,height=80,resolution=.25,origin=NS(position=NS(x=-10.,y=-10.))),data=[0]*6400)
        pose=dict(x=0.,y=0.,yaw=0.)
        quality=dict(resolution=.25,cells=[])
        d=choose_view(grid,quality,pose,[0,0],8,[],180)
        self.assertEqual(d['kind'],'turn')
        self.assertGreater(d['expected_new_m2'],0)
        self.assertEqual(d['expected_quality_gain_m2'],d['expected_utility_m2'])
        self.assertIsNone(choose_view(grid,quality,pose,[0,0],8,[],1))
        legacy=choose_view(grid,quality,pose,[0,0],8,[],17,'nearest_view_v1')
        upgraded,diagnostics=choose_view(grid,quality,pose,[0,0],8,[],17,'nearest_view_v2',return_diagnostics=True)
        self.assertEqual(legacy['kind'],'turn')
        self.assertEqual(upgraded['kind'],'advance')
        blocked=dict(upgraded,blocked=True)
        alternate=choose_view(grid,quality,pose,[0,0],8,[blocked],180,'coverage_view_v2')
        self.assertGreaterEqual(math.hypot(alternate['x']-blocked['x'],alternate['y']-blocked['y']),.7)
        self.assertTrue(diagnostics['movement_reservation_applied'])
        self.assertGreater(upgraded['enroute_visible_cells'],0)
        rejected,diagnostics=choose_view(grid,quality,pose,[0,0],8,[],1,'coverage_view_v2',return_diagnostics=True)
        self.assertIsNone(rejected);self.assertTrue(diagnostics['budget_limited'])
        history=[]
        for _ in range(9):
            d=choose_view(grid,quality,pose,[0,0],8,history,180)
            self.assertLess(math.hypot(d['x'],d['y']),7.25)
            history.append(d)
        self.assertTrue(any(h['kind']=='advance' for h in history))
        quality['cells']=[dict(ix=x,iy=y,quality=.2) for x in range(-40,40) for y in range(-40,40)]
        d=choose_view(grid,quality,pose,[0,0],8,[dict(x=0,y=0,yaw=a) for a in np.arange(-math.pi,math.pi,math.pi/4)],180)
        self.assertEqual(d['kind'],'resample')
        grid.data=[100]*6400
        self.assertIsNone(choose_view(grid,quality,pose,[0,0],8,[],180))

    def test_budget_stop_and_sensor_fault_revoke_motion(self):
        motion=Mock();motion.lock=threading.RLock();clock=motion.get_clock.return_value.now.return_value
        clock.nanoseconds=31_000_000_000
        survey=AutonomousSurvey(motion)
        survey.data=dict(active=True,started=0.,budget_seconds=30,history=[])
        self.assertFalse(survey.healthy());self.assertEqual(survey.state()['phase'],'budget_reached')
        motion.cancel.assert_called_once();self.assertIsNone(motion.mission_limits)
        clock.nanoseconds=0;survey.wall_started=__import__('time').monotonic()
        motion.quality=None;motion.quality_received=0.
        survey.data.update(active=True,started=0.);motion.state.return_value=dict(ready=False)
        self.assertFalse(survey.healthy());self.assertEqual(survey.state()['phase'],'fault')
        self.assertIn('车辆定位不可用',survey.state()['reason'])
        self.assertEqual(survey.state()['stop_diagnostics']['reason'],survey.state()['reason'])

    def test_transient_stale_pose_only_waits_when_motion_is_inactive(self):
        motion=Mock();motion.lock=threading.RLock();motion.grid_fresh.return_value=True
        motion.get_clock.return_value.now.return_value.nanoseconds=0
        motion.quality={};motion.quality_received=__import__('time').monotonic()
        stale=dict(active=False,ready=False,pose=None,depth_active=True,
                   diagnostics=dict(pose_error='车辆定位时间失效：TF 与仿真时钟相差 2.197 秒'))
        ready=dict(active=False,ready=True,pose=dict(x=0.,y=0.,yaw=0.),depth_active=True,diagnostics={})
        motion.state.side_effect=[stale,ready,ready]
        survey=AutonomousSurvey(motion);survey.origin=[0.,0.]
        survey.data=dict(active=True,started=0.,budget_seconds=30.,radius=8.,history=[])
        survey.wall_started=__import__('time').monotonic()
        with patch('autonomous_survey.time.sleep') as sleep:
            self.assertTrue(survey.healthy())
            sleep.assert_called_once()
        motion.cancel.assert_not_called()
        motion.state.side_effect=None;motion.state.return_value=dict(stale,active=True)
        with patch('autonomous_survey.time.sleep') as sleep:
            self.assertFalse(survey.healthy())
            sleep.assert_not_called()
        self.assertEqual(survey.data['phase'],'fault')
        motion.cancel.assert_called_once()

    def test_stalled_navigation_can_replan_but_other_cancellation_faults(self):
        motion=Mock();motion.lock=threading.RLock();motion.start.return_value=dict(token='goal')
        survey=AutonomousSurvey(motion);survey.data=dict(active=True,phase='planning');survey.task=dict(id='test')
        decision=dict(x=1.,y=0.,yaw=0.)
        with patch.object(survey,'healthy',return_value=True),patch.object(survey,'stop') as stop:
            motion.state.return_value=dict(active=False,phase='cancelled',error='stalled',replan_allowed=True)
            self.assertFalse(survey.execute(decision))
            stop.assert_not_called()
            motion.state.return_value['replan_allowed']=False
            self.assertFalse(survey.execute(decision))
            stop.assert_called_once_with('stalled','fault')
            stop.reset_mock()
            motion.start.side_effect=ValueError('目标位于障碍或车体安全余量内，请远离障碍重新选点')
            self.assertFalse(survey.execute(decision))
            self.assertIn('目标位于障碍',decision['dispatch_error'])
            stop.assert_not_called()

    def test_time_filtered_candidates_are_reported_as_budget_reached(self):
        motion=Mock();motion.lock=threading.RLock()
        motion.get_clock.return_value.now.return_value.nanoseconds=0
        motion.quality={};motion.quality_received=__import__('time').monotonic()
        motion.grid=Mock();motion.grid_fresh.return_value=True;motion.pose.return_value=dict(x=0,y=0,yaw=0)
        motion.state.return_value=dict(active=False,ready=True,pose=dict(x=0,y=0,yaw=0),error='')
        survey=AutonomousSurvey(motion);survey.origin=[0,0]
        survey.task = dict(region=[-15., -15., 15., 15.])
        survey.data=dict(active=True,started=0.,budget_seconds=15,radius=6,history=[])
        with patch('autonomous_survey.choose_view',return_value=(None,dict(budget_limited=True))) as planner:survey.run()
        self.assertEqual(planner.call_args.kwargs['region'], survey.task['region'])
        self.assertEqual(survey.data['phase'],'budget_reached')
        self.assertIn('剩余时间不足',survey.data['reason'])

    def test_candidate_is_rechecked_against_budget_after_planning(self):
        motion=Mock();motion.lock=threading.RLock()
        motion.get_clock.return_value.now.return_value.nanoseconds=0
        motion.quality={};motion.quality_received=__import__('time').monotonic()
        motion.grid=Mock();motion.grid_fresh.return_value=True;motion.pose.return_value=dict(x=0,y=0,yaw=0)
        motion.state.return_value=dict(active=False,ready=True,pose=dict(x=0,y=0,yaw=0),error='')
        survey=AutonomousSurvey(motion);survey.origin=[0,0]
        survey.task = dict(region=[-15., -15., 15., 15.])
        survey.data=dict(active=True,started=0.,budget_seconds=15,radius=6,history=[])
        decision=dict(x=1.,y=0.,yaw=0.,kind='advance',estimated_seconds=14.6)
        with patch('autonomous_survey.choose_view',return_value=(decision,dict(budget_limited=False))),patch.object(survey,'execute') as execute:
            survey.run()
        execute.assert_not_called();self.assertEqual(survey.data['phase'],'budget_reached')
        self.assertTrue(survey.data['candidate_diagnostics']['execution_budget_recheck_rejected'])

    def test_candidate_is_rechecked_after_record_write(self):
        motion=Mock();motion.lock=threading.RLock()
        motion.get_clock.return_value.now.return_value.nanoseconds=0
        motion.quality={};motion.quality_received=__import__('time').monotonic()
        motion.grid=Mock();motion.grid_fresh.return_value=True;motion.pose.return_value=dict(x=0,y=0,yaw=0)
        motion.state.return_value=dict(active=False,ready=True,pose=dict(x=0,y=0,yaw=0),error='')
        survey=AutonomousSurvey(motion);survey.origin=[0,0]
        survey.task = dict(region=[-15., -15., 15., 15.])
        survey.data=dict(active=True,started=0.,budget_seconds=20,radius=6,history=[])
        decision=dict(x=1.,y=0.,yaw=0.,kind='advance',estimated_seconds=10.,expected_new_m2=1.,
                      expected_improvement_m2=0.,visible_cells=[])
        with patch('autonomous_survey.choose_view',return_value=(decision,dict(budget_limited=False))),patch.object(survey,'has_action_budget',side_effect=(True,False)),patch.object(survey,'execute') as execute:
            survey.run()
        execute.assert_not_called();self.assertEqual(survey.data['phase'],'budget_reached')
        self.assertEqual(survey.data['history'],[])

    def test_interrupted_action_is_retained_and_motion_revoked(self):
        motion=Mock();motion.lock=threading.RLock()
        motion.get_clock.return_value.now.return_value.nanoseconds=0
        motion.quality=dict(resolution=.25,cells=[],stamp=0,frames=0)
        motion.quality_received=motion.grid_received=__import__('time').monotonic()
        pose=dict(x=0.,y=0.,yaw=0.)
        motion.pose.return_value=pose;motion.state.return_value=dict(ready=True,pose=pose,error='')
        survey=AutonomousSurvey(motion);survey.origin=[0,0]
        survey.task = dict(region=[-15., -15., 15., 15.])
        survey.data=dict(active=True,started=0.,budget_seconds=15,radius=6,history=[])
        decision=dict(x=1.,y=0.,yaw=0.,kind='advance',expected_new_m2=1.,expected_improvement_m2=0.,
                      expected_quality_gain_m2=1.,estimated_seconds=6.,visible_cells=[[0,0]])
        with patch('autonomous_survey.choose_view',return_value=(decision,dict(budget_limited=False))),patch.object(survey,'execute',side_effect=lambda _:survey.stop('test interruption')):
            survey.run()
        self.assertEqual(len(survey.data['history']),1)
        record=survey.data['history'][0]
        self.assertFalse(record['success']);self.assertEqual(record['outcome'],'interrupted')
        self.assertEqual(record['roi_gain']['quality_gain_m2'],0)
        motion.cancel.assert_called_once()


if __name__=='__main__':unittest.main()
