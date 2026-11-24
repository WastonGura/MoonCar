"""Planning-only RGBD revisit counterexamples, independent of ROS execution."""
import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np

REPO=next(path for path in Path(__file__).resolve().parents if (path/'scripts/autonomous_survey.py').is_file())
sys.path.insert(0,str(REPO/'scripts'))
import localization_revisit as policy


def original_function(path, name):
    tree = ast.parse(path.read_text(encoding='utf-8-sig'))
    node, = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == name]
    scope = dict(math=math, np=np)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),scope)
    return scope[name]


clearance = original_function(REPO/'scripts/autonomous_survey.py','known_chassis_clearance')
within_region = original_function(REPO/'scripts/experiment_motion.py','within_region')


def payload(stamp, ref=7, std=(.015,.010,.012), session='producer-a', sequence=None):
    sequence = round(stamp*1000) if sequence is None else sequence
    return dict(localization_backend='metric_rgbd_3d',healthy=True,session_id=session,sequence=sequence,
        accepted_stamp=stamp,last_fit=dict(accepted=True,exposure_stamp=stamp,reference_id=ref,position_std_m=list(std)))


def health(value, counter=0):
    return dict(healthy=True,session_id=value['session_id'],sequence=value['sequence'],
        accepted_stamp=value['accepted_stamp'],fault_counter=counter)


def sample(stamp, ref=7, std=(.015,.010,.012), session='producer-a', counter=0):
    value=payload(stamp,ref,std,session)
    return policy.sample_payload(value,health(value,counter))


def observation(stamp,x=1.05,y=1.05,yaw=0.,pan=0.,tilt=-.2618):
    return dict(stamp=stamp,x=x,y=y,yaw=yaw,pan=pan,tilt=tilt)


def grid(resolution=.1,width=60,height=60):
    return NS(info=NS(resolution=resolution,width=width,height=height,
                      origin=NS(position=NS(x=0.,y=0.))))


def cell(grid_,row):
    return (math.floor((row['x']-grid_.info.origin.position.x)/grid_.info.resolution),
            math.floor((row['y']-grid_.info.origin.position.y)/grid_.info.resolution))


def ready(saved_std=(.015,.010,.012), trigger_std=(.030,.020,.025)):
    value=policy.RGBDRevisitPolicy()
    value.observe(sample(1.,std=saved_std),observation(1.))
    current=observation(2.,x=3.05)
    value.observe(sample(2.,ref=20,std=trigger_std),current)
    return value,current


def plan(value,current,costs=None,**overrides):
    grid_=overrides.pop('grid_',grid())
    costs=np.zeros((grid_.info.height,grid_.info.width),dtype=np.int16) if costs is None else costs
    args=dict(current=current,grid=grid_,costs=costs,clearance=clearance,within_region=within_region,
              region=[-10.,-10.,10.,10.],origin=[1.05,1.05],radius=10.,remaining=1000.,gimbal=[0.,-.2618])
    args.update(overrides)
    return value.plan(**args)


class RevisitInputContractTests(unittest.TestCase):
    def test_non_rgbd_has_no_sample_and_no_policy_effect(self):
        value=payload(1.)
        value['localization_backend']='metric_laser_2d'
        tracker=policy.RGBDRevisitPolicy()
        self.assertIsNone(policy.sample_payload(value,health(value)))
        self.assertIsNone(tracker.pending)
        self.assertEqual(tracker.attempts,0)

    def test_rejects_mismatched_exposure_reference_sequence_or_std(self):
        mutations=[lambda p:p['last_fit'].update(exposure_stamp=.9),
                   lambda p:p['last_fit'].update(reference_id=True),
                   lambda p:p['last_fit'].update(position_std_m=[.01,-.01,.01]),
                   lambda p:p['last_fit'].update(position_std_m=[.01,math.nan,.01]),
                   lambda p:p['last_fit'].update(position_std_m=[.01,True,.01]),
                   lambda p:p['last_fit'].update(position_std_m=[.01,math.inf,.01])]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                value=payload(1.);mutate(value)
                with self.assertRaises(ValueError):policy.sample_payload(value,health(value))
        value=payload(1.); checked=health(value);checked['sequence']+=1
        with self.assertRaises(ValueError):policy.sample_payload(value,checked)

    def test_trace_is_an_upper_bound_not_maximum_eigenvalue(self):
        value=sample(1.,std=(.025,.025,.02))
        self.assertAlmostEqual(value['trace_xy_sigma_upper_m'],.025*math.sqrt(2))
        self.assertAlmostEqual(value['xy_variance_lower_m2'],.025**2)
        self.assertGreater(value['trace_xy_sigma_upper_m'],.025)

    def test_pose_exposure_and_producer_clock_are_fenced(self):
        tracker=policy.RGBDRevisitPolicy()
        with self.assertRaises(ValueError):tracker.observe(sample(1.),observation(.9))
        tracker=policy.RGBDRevisitPolicy();tracker.observe(sample(1.),observation(1.))
        with self.assertRaises(ValueError):tracker.observe(sample(.9),None)
        tracker=policy.RGBDRevisitPolicy();tracker.observe(sample(1.),observation(1.))
        with self.assertRaises(ValueError):tracker.observe(sample(1.1,session='producer-b'),None)


class RevisitRouteSafetyTests(unittest.TestCase):
    def test_saved_candidate_is_executable_only_with_current_safe_grid(self):
        tracker,current=ready()
        costs=np.zeros((60,60),dtype=np.int16)
        x,y=cell(grid(),tracker.observations[0]);costs[y,x]=-1
        self.assertIsNone(plan(tracker,current,costs))
        costs[y,x]=0
        selected=plan(tracker,current,costs)
        self.assertIsNotNone(selected)
        self.assertEqual(selected['checkpoint']['reference_id'],7)

    def test_unknown_hazard_and_target_cost_reject(self):
        for bad_cost in (-1,100):
            with self.subTest(cost=bad_cost):
                tracker,current=ready();costs=np.zeros((60,60),dtype=np.int16)
                costs[10,20]=bad_cost
                self.assertIsNone(plan(tracker,current,costs))
        tracker,current=ready();costs=np.zeros((60,60),dtype=np.int16);costs[10,10]=66
        self.assertIsNone(plan(tracker,current,costs))

    def test_original_chassis_disk_unknown_off_center_blocks(self):
        tracker,current=ready();costs=np.zeros((60,60),dtype=np.int16)
        costs[15,10]=-1
        self.assertFalse(clearance(costs,10,10,.1))
        self.assertIsNone(plan(tracker,current,costs))

    def test_region_radius_and_whole_return_budget_are_required(self):
        tracker,current=ready()
        self.assertIsNone(plan(tracker,current,region=[0.,0.,2.,2.]))
        self.assertIsNone(plan(tracker,current,radius=2.))
        self.assertIsNone(plan(tracker,current,remaining=1.))
        selected=plan(tracker,current)
        self.assertGreater(selected['estimated_seconds'],selected['route_length_m']/.06)
        self.assertGreater(selected['budget']['view_seconds'],0.)

    def test_corner_supercover_keeps_both_touched_neighbors(self):
        cells=set(policy.line_cells(dict(x=.5,y=.5),dict(x=2.5,y=2.5),grid(1.,10,10)))
        self.assertTrue({(1,0),(0,1),(2,1),(1,2)}<=cells)

    def test_segment_along_grid_boundary_covers_both_sides(self):
        cells=set(policy.line_cells(dict(x=3.,y=2.5),dict(x=3.,y=4.5),grid(1.,10,10)))
        self.assertTrue({(2,y) for y in (2,3,4)}<=cells)
        self.assertTrue({(3,y) for y in (2,3,4)}<=cells)
        cells=set(policy.line_cells(dict(x=2.5,y=3.),dict(x=4.5,y=3.),grid(1.,10,10)))
        self.assertTrue({(x,2) for x in (2,3,4)}<=cells)

    def test_actual_near_corner_endpoint_roundoff_forward_and_reverse(self):
        value=grid(.25,120,120)
        value.info.origin.position.x=value.info.origin.position.y=-15.
        a=dict(x=1.9993952776209143,y=-.04917848947647099)
        b=dict(x=2.,y=-9.797174393178826e-16)
        expected={(67,59),(68,59),(67,60),(68,60)}
        forward=set(policy.line_cells(a,b,value))
        reverse=set(policy.line_cells(b,a,value))
        self.assertTrue(expected<=forward)
        self.assertEqual(forward,reverse)
        self.assertFalse(any(y<59 for x,y in forward))
        self.assertLessEqual(len(forward),6)

    def test_every_actual_intermediate_goal_obeys_original_target_cost(self):
        tracker=policy.RGBDRevisitPolicy()
        tracker.observe(sample(1.),observation(1.,1.05,1.05))
        tracker.observe(sample(2.,std=(.02,.02,.02)),observation(2.,3.05,1.05))
        tracker.observe(sample(3.,std=(.02,.02,.02)),observation(3.,3.05,3.05))
        current=observation(4.,1.05,3.05)
        tracker.observe(sample(4.,ref=20,std=(.03,.02,.02)),current)
        costs=np.zeros((60,60),dtype=np.int16)
        costs[20,10]=100;costs[20,20]=100;costs[30,30]=80
        selected=plan(tracker,current,costs)
        self.assertTrue(selected is None or all(costs[y,x]<=65 for x,y in (cell(grid(),g) for g in selected['goals'])))

    def test_attempts_and_retained_observations_are_bounded(self):
        tracker,current=ready();selected=plan(tracker,current)
        for index in range(policy.MAX_REVISITS):
            if index:tracker.observe(sample(3.+index,ref=20,std=(.03,.02,.02)),None)
            tracker.begin(selected);tracker.complete()
        tracker.observe(sample(10.,ref=20,std=(.03,.02,.02)),None)
        with self.assertRaises(ValueError):plan(tracker,current)
        tracker=policy.RGBDRevisitPolicy()
        for index in range(policy.MAX_OBSERVATIONS+10):
            tracker.observe(sample(float(index+1),ref=index+1),observation(float(index+1),x=index*.2))
        self.assertLessEqual(len(tracker.observations),policy.MAX_OBSERVATIONS)


class RevisitProofTests(unittest.TestCase):
    def started(self,**kwargs):
        tracker,current=ready(**kwargs);selected=plan(tracker,current);self.assertIsNotNone(selected)
        tracker.begin(selected)
        return tracker

    def test_three_distinct_after_arrival_same_saved_reference_required(self):
        tracker=self.started()
        self.assertFalse(tracker.prove(sample(3.),3.))
        self.assertFalse(tracker.prove(sample(3.1),3.))
        self.assertFalse(tracker.prove(sample(3.1),3.))
        self.assertFalse(tracker.prove(sample(3.2),3.))
        self.assertFalse(tracker.prove(sample(3.3,ref=8),3.))
        self.assertFalse(tracker.prove(sample(3.4),3.))
        self.assertFalse(tracker.prove(sample(3.5),3.))
        self.assertTrue(tracker.prove(sample(3.6),3.))
        tracker.complete();self.assertFalse(tracker.running);self.assertIsNone(tracker.pending)

    def test_smaller_trace_alone_does_not_prove_variance_drop(self):
        tracker=self.started(saved_std=(.017,.017,.02),trigger_std=(.025,.025,.02))
        for stamp in (3.1,3.2,3.3):
            value=sample(stamp,std=(.019,.019,.02))
            self.assertLess(value['trace_xy_sigma_upper_m'],tracker.pending['trigger']['trace_xy_sigma_upper_m'])
            self.assertLessEqual(value['trace_xy_sigma_upper_m'],tracker.pending['checkpoint']['trace_xy_sigma_upper_m']+.003)
            self.assertFalse(tracker.prove(value,3.))

    def test_changed_session_counter_or_insufficient_drop_cannot_prove(self):
        for change in [dict(session='producer-b'),dict(counter=1),dict(std=(.025,.025,.02))]:
            with self.subTest(change=change):
                tracker=self.started()
                for stamp in (3.1,3.2,3.3):self.assertFalse(tracker.prove(sample(stamp,**change),3.))

    def test_active_revisit_rejects_outage_or_saved_reference_retirement(self):
        tracker=self.started()
        with self.assertRaises(ValueError):tracker.observe(sample(3.,counter=1),None)
        tracker=self.started();value=sample(3.);value['retired_reference_id']=7
        with self.assertRaises(ValueError):tracker.observe(value,None)


if __name__ == '__main__':
    unittest.main()
