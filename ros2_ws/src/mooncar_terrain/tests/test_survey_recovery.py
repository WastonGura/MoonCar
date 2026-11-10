"""Offline mission regressions for bounded recovery after failed navigation."""
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts'))
from autonomous_survey import AutonomousSurvey, visible_unknown_frontiers


class SurveyRecoveryTests(unittest.TestCase):
    def make_survey(self, strategy='surface_view_v3', budget=120):
        motion = Mock()
        motion.lock = threading.RLock()
        motion.get_clock.return_value.now.return_value.nanoseconds = 0
        motion.quality = dict(resolution=.25, cells=[], stamp=0., frames=1)
        motion.quality_received = time.monotonic()
        motion.grid_fresh.return_value = True
        pose = dict(x=0., y=0., yaw=0.)
        motion.pose.return_value = pose
        motion.state.return_value = dict(active=False, ready=True, pose=pose,
                                        error='没有找到可行路径（208）')
        survey = AutonomousSurvey(motion)
        survey.origin = [0., 0.]
        survey.task = dict(id='test', start=[0., 0.], region=[-25, -25, 25, 25])
        survey.data = dict(active=True, started=0., budget_seconds=budget,
                           radius=8., history=[], strategy=strategy)
        return survey

    def decision(self, index):
        return dict(x=float(index + 1), y=0., yaw=0., kind='advance',
                    expected_new_m2=1., expected_improvement_m2=0.,
                    expected_utility_m2=1., estimated_seconds=6., visible_cells=[])

    def run_mission(self, survey, outcomes, fresh=True, recovery_success=True):
        plans = iter([self.decision(i) for i in range(len(outcomes))] + [None])
        outcome_iter = iter(outcomes)
        def plan(*args, **kwargs):
            decision = next(plans)
            return decision, dict(budget_limited=False)
        def advance(seconds):
            clock = survey.motion.get_clock.return_value.now.return_value
            clock.nanoseconds += int(seconds * 1e9)
        def execute(decision, turn=False):
            if decision.get('recovery'):
                if fresh:
                    survey.motion.quality = dict(survey.motion.quality, frames=2, stamp=1.)
                return recovery_success
            return next(outcome_iter)
        with patch('autonomous_survey.choose_view', side_effect=plan) as planner, \
                patch('autonomous_survey.time.sleep', side_effect=advance), \
                patch.object(survey, 'execute', side_effect=execute) as executor:
            survey.run()
        return planner, executor

    def test_repeated_unreachable_goals_get_one_scan_then_fault(self):
        survey = self.make_survey()
        planner, executor = self.run_mission(survey, [False] * 6)
        self.assertEqual(survey.data['phase'], 'fault')
        self.assertEqual(planner.call_count, 6)
        self.assertEqual(executor.call_count, 7)
        self.assertEqual(survey.data['recovery_attempts'], 1)
        records = survey.data['history']
        self.assertEqual(len(records), 7)
        scan = records[3]
        self.assertTrue(scan['recovery'])
        self.assertTrue(scan['success'])
        self.assertTrue(scan['use_pan_tilt'])
        self.assertEqual((scan['x'], scan['y']), (0., 0.))
        self.assertTrue(all(r['retry_blocked'] for r in records if not r.get('recovery')))
        survey.motion.cancel.assert_called_once()

    def run_frontier_mission(self, fresh_grid):
        survey=self.make_survey(strategy='surface_view_v2')
        survey.initial_depth_scan=True
        survey.motion.gimbal_position=[0.,0.]
        survey.motion.gimbal_command_rate_rad_s=.35
        info=NS(width=40,height=40,resolution=.2,origin=NS(position=NS(x=-4.,y=-4.)))
        cells=[0 if 10<=i<30 and 10<=j<30 else -1 for j in range(40) for i in range(40)]
        def grid(stamp,data):
            return NS(info=info,header=NS(stamp=NS(sec=stamp,nanosec=0)),data=data)
        survey.motion.grid=grid(0,cells)
        survey.initial_depth_observation=Mock(return_value=True)
        clock=survey.motion.get_clock.return_value.now.return_value
        def advance(seconds):
            clock.nanoseconds+=int(seconds*1e9)
        def execute(decision,turn=False):
            if decision.get('frontier_observation'):
                self.assertTrue(turn and decision['use_pan_tilt'])
                self.assertEqual((decision['x'],decision['y']),(0.,0.))
                advance(1)
                survey.motion.quality=dict(survey.motion.quality,frames=2,stamp=1.)
                if fresh_grid:survey.motion.grid=grid(2,[0]*len(cells))
            return True
        plans=[None,self.decision(0),None] if fresh_grid else [None,None,None]
        with patch('autonomous_survey.choose_view',side_effect=[(p,dict(budget_limited=False)) for p in plans]), \
                patch('autonomous_survey.time.sleep',side_effect=advance), \
                patch.object(survey,'execute',side_effect=execute) as executor:
            survey.run()
        return survey,executor

    def test_new_navigation_evidence_allows_exploration_to_resume(self):
        survey,executor=self.run_frontier_mission(fresh_grid=True)
        scan=survey.data['frontier_observations'][0]
        self.assertEqual(scan['outcome'],'observed')
        self.assertGreater(scan['newly_known_cells'],0)
        self.assertEqual(executor.call_count,2)
        self.assertTrue(survey.data['history'][0]['success'])
        self.assertEqual(survey.data['phase'],'no_candidates')

    def test_stale_navigation_evidence_cannot_authorize_a_move(self):
        survey,executor=self.run_frontier_mission(fresh_grid=False)
        self.assertEqual(executor.call_count,2)
        self.assertEqual(survey.data['history'],[])
        self.assertEqual([r['outcome'] for r in survey.data['frontier_observations']],
                         ['no_new_evidence','no_new_evidence'])
        self.assertEqual(survey.data['phase'],'no_candidates')
        self.assertIn('待探测',survey.data['reason'])

    def test_frontier_rays_do_not_look_through_obstacles(self):
        info=NS(width=40,height=40,resolution=.2,origin=NS(position=NS(x=-4.,y=-4.)))
        cells=[0 if 10<i<29 and 10<j<29 else 100 if 10<=i<=29 and 10<=j<=29 else -1
               for j in range(40) for i in range(40)]
        grid=NS(info=info,data=cells)
        self.assertEqual(visible_unknown_frontiers(grid,dict(x=0.,y=0.,yaw=0.),[0.,0.],8.),{})

    def test_successful_navigation_after_scan_resumes_planning(self):
        survey = self.make_survey()
        self.run_mission(survey, [False, False, False, True])
        self.assertEqual(survey.data['phase'], 'no_candidates')
        self.assertTrue(survey.data['history'][-1]['success'])
        self.assertEqual(survey.data['recovery_attempts'], 1)

    def test_recovery_requires_new_observation_frames(self):
        survey = self.make_survey()
        planner, _ = self.run_mission(survey, [False] * 4, fresh=False)
        self.assertEqual(survey.data['phase'], 'fault')
        self.assertEqual(planner.call_count, 3)
        record = survey.data['history'][-1]
        self.assertTrue(record['recovery'])
        self.assertFalse(record['success'])
        self.assertIn('新观测', record['motion_error'])

    def test_failed_scan_stops_without_another_navigation_attempt(self):
        survey = self.make_survey()
        planner, _ = self.run_mission(survey, [False] * 4, recovery_success=False)
        self.assertEqual(survey.data['phase'], 'fault')
        self.assertEqual(planner.call_count, 3)
        self.assertFalse(survey.data['history'][-1]['success'])

    def test_small_remaining_budget_does_not_dispatch_scan(self):
        survey = self.make_survey(budget=11)
        _, executor = self.run_mission(survey, [False] * 4)
        self.assertEqual(survey.data['phase'], 'budget_reached')
        self.assertEqual(executor.call_count, 3)
        self.assertEqual(len(survey.data['history']), 3)

    def test_v2_stops_after_three_failures_without_gimbal_recovery(self):
        survey = self.make_survey(strategy='surface_view_v2')
        _, executor = self.run_mission(survey, [False] * 4)
        self.assertEqual(survey.data['phase'], 'fault')
        self.assertEqual(executor.call_count, 3)
        self.assertEqual(len(survey.data['history']), 3)

    def test_missing_gimbal_stops_without_chassis_spin(self):
        survey = self.make_survey()
        survey.motion.pan_tilt = None
        _, executor = self.run_mission(survey, [False] * 4)
        self.assertEqual(survey.data['phase'], 'fault')
        self.assertEqual(executor.call_count, 3)
        survey.motion.turn.assert_not_called()

    def test_successful_turns_do_not_erase_navigation_failures(self):
        survey = self.make_survey()
        original = self.decision
        def plan(index):
            if index % 2:
                return dict(original(index), kind='turn', x=0., use_pan_tilt=True, pan=0.)
            return original(index)
        with patch.object(self, 'decision', side_effect=plan):
            self.run_mission(survey, [False, True, False, True, False])
        self.assertEqual(survey.data['recovery_attempts'], 1)
        self.assertTrue(survey.data['history'][-1]['recovery'])

    def test_actual_scan_dispatch_uses_gimbal_and_internal_heartbeat(self):
        survey = self.make_survey()
        survey.wall_started = time.monotonic()
        survey.motion.pan_tilt.return_value = dict(token='scan')
        survey.motion.state.return_value.update(phase='succeeded')
        decision = dict(yaw=0., pan=0., tilt=-.2618, use_pan_tilt=True)
        self.assertTrue(survey.execute(decision, turn=True))
        survey.motion.pan_tilt.assert_called_once_with(0., -.2618, survey.task)
        survey.motion.heartbeat.assert_called_once_with(dict(token='scan'))
        survey.motion.start.assert_not_called()
        survey.motion.turn.assert_not_called()

    def test_sensor_fault_before_scan_prevents_dispatch(self):
        survey = self.make_survey()
        survey.wall_started = time.monotonic()
        survey.motion.quality = None
        self.assertFalse(survey.execute(dict(yaw=0., pan=0., use_pan_tilt=True), turn=True))
        self.assertEqual(survey.data['phase'], 'fault')
        survey.motion.pan_tilt.assert_not_called()
        survey.motion.start.assert_not_called()
        survey.motion.cancel.assert_called_once()


    def test_only_depth_missions_enable_supported_unknown_visibility(self):
        for depth in (False,True):
            with self.subTest(depth=depth):
                survey=self.make_survey();survey.initial_depth_scan=depth
                survey.motion.grid=NS(info=NS(width=1,height=1,resolution=.2,
                                             origin=NS(position=NS(x=0.,y=0.))),data=[0])
                with patch.object(survey,'initial_depth_observation',return_value=True), \
                        patch('autonomous_survey.choose_view',return_value=(None,dict(budget_limited=False))) as planner:
                    survey.run()
                self.assertEqual(survey.data['phase'],'no_candidates')
                self.assertIs(planner.call_args.kwargs['depth_observed_visibility'],depth)


if __name__ == '__main__':
    unittest.main()
