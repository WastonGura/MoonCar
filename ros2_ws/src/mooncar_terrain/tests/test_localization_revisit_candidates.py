"""Cheap current target checks precede the unchanged route-work budget."""
from pathlib import Path
import math
import sys
import unittest
from unittest.mock import patch

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=next(parent for parent in HERE.parents if (parent/'scripts/autonomous_survey.py').is_file())
sys.path[:0]=[str(HERE),str(ROOT/'scripts'),str(ROOT/'ros2_ws/src/mooncar_terrain/tests')]
import localization_revisit as policy
import test_localization_revisit as original


class RevisitCandidatePrefilterTests(unittest.TestCase):
    def fixture(self,count=8):
        tracker=policy.RGBDRevisitPolicy();current=original.observation(20.,5.05,5.05)
        tracker.observations=[dict(original.sample(float(index+1),ref=index+1),
            **original.observation(float(index+1),5.05-.2*(index+1),5.05))for index in range(count)]
        tracker.pending=dict(trigger=original.sample(20.,ref=100,std=(.03,.025,.03)))
        costs=np.zeros((100,100),dtype=np.int16)
        return tracker,current,costs

    def call(self,tracker,current,costs,**overrides):
        return original.plan(tracker,current,costs,grid_=original.grid(.1,100,100),**overrides)

    def test_four_nearest_bad_targets_do_not_hide_fifth_actual_safe_saved_pose(self):
        tracker,current,costs=self.fixture(5)
        saved=tracker.observations[4].copy()
        for row in tracker.observations[:4]:
            x,y=original.cell(original.grid(.1,100,100),row);costs[y,x]=66
        result=self.call(tracker,current,costs)
        self.assertIsNotNone(result)
        self.assertEqual(result['checkpoint'],saved)
        self.assertEqual(result['checkpoint']['reference_id'],5)
        self.assertEqual(result['goals'][-1]['x'],saved['x'])
        self.assertEqual(tracker.retirement_generation,0)
        self.assertNotIn('planned_standpoint',result)

    def test_four_expensive_route_candidates_remain_a_hard_cap(self):
        tracker,current,costs=self.fixture(8)
        with patch.object(policy,'safe_segment',return_value=False)as segment:
            self.assertIsNone(self.call(tracker,current,costs))
        self.assertEqual(segment.call_count,policy.MAX_CHECKPOINT_VALIDATIONS)

    def test_unknown_hazard_disk_region_radius_reject_before_route_work(self):
        for kind in ('unknown','hazard','disk','region','radius'):
            with self.subTest(kind=kind):
                tracker,current,costs=self.fixture(1);overrides={}
                x,y=original.cell(original.grid(.1,100,100),tracker.observations[0])
                if kind=='unknown':costs[y,x]=-1
                elif kind=='hazard':costs[y,x]=100
                elif kind=='disk':costs[y+5,x]=-1
                elif kind=='region':overrides['region']=[0.,0.,1.,1.]
                else:overrides['radius']=2.
                with patch.object(policy,'safe_segment',side_effect=AssertionError('cheap unsafe target reached route work')):
                    self.assertIsNone(self.call(tracker,current,costs,**overrides))

    def test_empty_cheap_survivors_cannot_enable_nearby_or_change_saved_rows(self):
        tracker,current,costs=self.fixture(5)
        before=[row.copy()for row in tracker.observations]
        for row in before:
            x,y=original.cell(original.grid(.1,100,100),row);costs[y,x]=66
        with patch.object(tracker,'_plan_retired_nearby',side_effect=AssertionError('initial warning must not use retirement nearby')):
            self.assertIsNone(self.call(tracker,current,costs))
        self.assertEqual(tracker.observations,before)
        self.assertEqual(tracker.attempts,0)


class UnknownClearanceScanTests(unittest.TestCase):
    def fixture(self):
        tracker,current=original.ready()
        current['yaw']=math.pi
        costs=np.zeros((60,60),dtype=np.int16)
        costs[15,10]=-1
        return tracker,current,costs

    def scan(self,tracker,current,costs,**overrides):
        arguments=dict(current=current,grid=original.grid(),costs=costs,clearance=original.clearance,
            within_region=original.within_region,region=[-10.,-10.,10.,10.],origin=[1.05,1.05],
            radius=10.,remaining=1000.,gimbal=[0.,-.2618])
        arguments.update(overrides)
        return tracker.unknown_clearance_scan(**arguments)

    def test_unknown_only_disk_can_propose_one_bounded_view_without_filling_map(self):
        tracker,current,costs=self.fixture();before=costs.copy()
        saved=[row.copy()for row in tracker.observations]
        self.assertIsNone(original.plan(tracker,current,costs))
        self.assertEqual(tracker.plan_diagnostics['cheap_eligible'],0)
        proposal=self.scan(tracker,current,costs)
        self.assertIsNotNone(proposal)
        self.assertEqual(proposal['checkpoint'],saved[0])
        self.assertTrue(proposal['unknown_cells'])
        self.assertLessEqual(abs(proposal['pan']),2.356)
        self.assertGreaterEqual(proposal['tilt'],-.611)
        self.assertLessEqual(proposal['tilt'],.262)
        self.assertGreater(proposal['estimated_seconds'],0.)
        self.assertGreater(proposal['return_reserve_seconds'],0.)
        self.assertLessEqual(proposal['estimated_seconds']+proposal['return_reserve_seconds'],1000.)
        np.testing.assert_array_equal(costs,before)
        self.assertEqual(tracker.observations,saved)
        self.assertFalse(tracker.running)

    def test_hazard_target_cost_or_outside_disk_cannot_be_reclassified_as_unknown(self):
        for kind in ('hazard99','hazard100','target66','target_unknown','edge'):
            with self.subTest(kind=kind):
                tracker,current,costs=self.fixture()
                if kind=='hazard99':costs[10,11]=99
                elif kind=='hazard100':costs[10,11]=100
                elif kind=='target66':costs[10,10]=66
                elif kind=='target_unknown':costs[10,10]=-1
                else:
                    tracker.observations[0]['x']=.05
                    costs[:]=0;costs[15,0]=-1
                original.plan(tracker,current,costs)
                self.assertIsNone(self.scan(tracker,current,costs))

    def test_current_strict_stationary_footprint_and_original_limits_still_required(self):
        for kind in ('current_unknown','current_hazard','region','radius','budget','attempt_limit'):
            with self.subTest(kind=kind):
                tracker,current,costs=self.fixture();options={}
                if kind=='current_unknown':costs[10,30]=-1
                elif kind=='current_hazard':costs[10,30]=100
                elif kind=='region':options['region']=[2.,0.,5.,5.]
                elif kind=='radius':options['radius']=1.
                elif kind=='attempt_limit':tracker.attempts=3
                else:options['remaining']=1.
                original.plan(tracker,current,costs)
                self.assertIsNone(self.scan(tracker,current,costs,**options))

    def test_route_unknown_with_cheap_survivor_and_retired_mode_are_not_new_scan_cases(self):
        tracker,current,costs=self.fixture();costs[:]=0;costs[10,20]=-1
        self.assertIsNone(original.plan(tracker,current,costs))
        self.assertEqual(tracker.plan_diagnostics['cheap_eligible'],1)
        self.assertIsNone(self.scan(tracker,current,costs))
        tracker,current,costs=self.fixture()
        tracker.retirement_generation=1
        tracker.pending['retirement_event']=dict(reference_id=7,stamp=2.)
        self.assertIsNone(self.scan(tracker,current,costs))


if __name__=='__main__':unittest.main()
