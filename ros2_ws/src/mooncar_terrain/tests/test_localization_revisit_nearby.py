"""Nearby parking is a new plan coordinate, never a rewritten observation."""
import ast
import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'scripts/autonomous_survey.py').is_file())
sys.path.insert(0,str(ROOT/'scripts'))
import localization_revisit as policy


def original(relative,name):
    path=ROOT/relative
    function,=[node for node in ast.parse(path.read_text(encoding='utf-8-sig')).body if isinstance(node,ast.FunctionDef)and node.name==name]
    namespace=dict(np=np,math=math)
    exec(compile(ast.Module(body=[function],type_ignores=[]),str(path),'exec'),namespace)
    return namespace[name]


clearance=original('scripts/autonomous_survey.py','known_chassis_clearance')
within_region=original('scripts/experiment_motion.py','within_region')


def sample(stamp,ref,std=(.012,.012,.014)):
    payload=dict(localization_backend='metric_rgbd_3d',session_id='producer',sequence=round(stamp*1000),healthy=True,
        accepted_stamp=stamp,last_fit=dict(accepted=True,exposure_stamp=stamp,reference_id=ref,position_std_m=list(std)))
    return policy.sample_payload(payload,dict(healthy=True,session_id='producer',sequence=payload['sequence'],fault_counter=0))


def pose(stamp,x=1.05,y=1.05):return dict(stamp=stamp,x=x,y=y,yaw=.1,pan=.2,tilt=-.262)


def fixture(saved_x=1.05,retire=True):
    tracker=policy.RGBDRevisitPolicy()
    tracker.observe(sample(1.,76),pose(1.,saved_x))
    tracker.observe(sample(2.,79),pose(2.,2.05))
    tracker.observe(sample(4.,114,(.03,.02,.025)),pose(4.,3.05))
    saved=next(row for row in tracker.observations if row['reference_id']==76)
    if retire:
        old=next(row for row in tracker.observations if row['reference_id']==79)
        tracker.begin(dict(checkpoint=old,goals=[old],estimated_seconds=30.))
        event_sample=sample(5.,133,(.032,.025,.03));event_sample['retired_reference_id']=79
        try:tracker.observe(event_sample,None)
        except policy.RetiredCheckpoint as event:tracker.invalidate_retired_checkpoint(event)
        else:raise AssertionError('No typed retirement event')
    grid=NS(info=NS(resolution=.1,width=60,height=60,origin=NS(position=NS(x=0.,y=0.))))
    return tracker,saved,grid,np.zeros((60,60),dtype=np.int16)


def plan(tracker,grid,costs,**kw):
    args=dict(current=dict(x=3.05,y=1.05,yaw=0.),grid=grid,costs=costs,clearance=clearance,within_region=within_region,
        region=[-10.,-10.,10.,10.],origin=[1.05,1.05],radius=10.,remaining=1000.,gimbal=[0.,-.262])
    args.update(kw)
    return tracker.plan(**args)


class NearbyRevisitStandpointTests(unittest.TestCase):
    def test_initial_warn_cannot_use_nearby_parking(self):
        tracker,saved,grid,costs=fixture(retire=False)
        tracker.observations=[saved]
        costs[10,3]=-1
        self.assertFalse(clearance(costs,10,10,.1))
        self.assertTrue(clearance(costs,11,10,.1))
        self.assertIsNone(plan(tracker,grid,costs))

    def test_retirement_only_standpoint_does_not_mutate_saved_observation(self):
        tracker,saved,grid,costs=fixture();before=copy.deepcopy(saved)
        costs[10,3]=-1
        chosen=plan(tracker,grid,costs)
        self.assertIsNotNone(chosen)
        self.assertEqual(saved,before)
        self.assertEqual(chosen['checkpoint'],before)
        parking=chosen['planned_standpoint']
        self.assertIsNot(parking,chosen['checkpoint'])
        self.assertEqual(parking['kind'],'near_saved_observation')
        self.assertFalse(any(key in parking for key in ('stamp','reference_id','position_std_m')))
        distance=math.hypot(parking['x']-saved['x'],parking['y']-saved['y'])
        self.assertGreater(distance,0.)
        self.assertLessEqual(distance,.25)
        self.assertEqual((chosen['goals'][-1]['x'],chosen['goals'][-1]['y']),(parking['x'],parking['y']))
        x,y=math.floor(parking['x']/.1),math.floor(parking['y']/.1)
        self.assertTrue(clearance(costs,x,y,.1))
        self.assertLessEqual(costs[y,x],65)

    def test_existing_quarter_meter_boundary_is_kept(self):
        for saved_x,eligible in ((1.,True),(.99,False)):
            with self.subTest(saved_x=saved_x):
                tracker,saved,grid,costs=fixture(saved_x=saved_x)
                costs[:]=66;costs[10,12]=0
                chosen=plan(tracker,grid,costs)
                if eligible:
                    self.assertIsNotNone(chosen)
                    self.assertAlmostEqual(chosen['planned_standpoint']['x'],1.25)
                    self.assertAlmostEqual(math.hypot(chosen['planned_standpoint']['x']-saved_x,
                        chosen['planned_standpoint']['y']-saved['y']),.25)
                else:self.assertIsNone(chosen)

    def test_no_unknown_or_hazard_on_strict_direct_route(self):
        for blocked in (-1,100):
            with self.subTest(blocked=blocked):
                tracker,saved,grid,costs=fixture();costs[10,3]=-1;costs[10,20]=blocked
                self.assertIsNone(plan(tracker,grid,costs))

    def test_nearby_candidate_still_requires_disk_region_radius_and_budget(self):
        tracker,saved,grid,costs=fixture();costs[10,7]=-1
        self.assertIsNone(plan(tracker,grid,costs))
        for restriction in (dict(region=[0.,0.,1.5,1.5]),dict(radius=2.),dict(remaining=1.)):
            with self.subTest(restriction=restriction):
                tracker,saved,grid,costs=fixture();costs[10,3]=-1
                self.assertIsNone(plan(tracker,grid,costs,**restriction))

    def test_parking_does_not_replace_original_saved_reference_in_proof(self):
        tracker,saved,grid,costs=fixture();costs[10,3]=-1
        chosen=plan(tracker,grid,costs);self.assertIsNotNone(chosen)
        tracker.begin(chosen)
        for stamp in (6.1,6.2,6.3):self.assertFalse(tracker.prove(sample(stamp,79),6.))
        self.assertFalse(tracker.prove(sample(6.4,76),6.))
        self.assertFalse(tracker.prove(sample(6.5,76),6.))
        self.assertTrue(tracker.prove(sample(6.6,76),6.))
        self.assertEqual(tracker.pending['checkpoint']['stamp'],1.)
        self.assertEqual(tracker.pending['checkpoint']['reference_id'],76)

    def test_nearby_strict_route_probes_have_a_fixed_small_bound(self):
        tracker,saved,grid,costs=fixture(saved_x=1.)
        costs[:]=66
        costs[9:12,10:13]=0
        with patch.object(policy,'safe_segment',wraps=policy.safe_segment)as calls:
            chosen=plan(tracker,grid,costs)
        self.assertIsNotNone(chosen)
        self.assertLessEqual(calls.call_count,4)

    def test_fine_grid_and_full_saved_history_have_bounded_enumeration(self):
        tracker=policy.RGBDRevisitPolicy()
        old=pose(1.,80.5e-6,80.5e-6)
        tracker.observe(sample(1.,79),old)
        checkpoint=tracker.observations[0]
        tracker.observe(sample(2.,114,(.03,.02,.025)),pose(2.,90.5e-6,80.5e-6))
        tracker.begin(dict(checkpoint=checkpoint,goals=[checkpoint],estimated_seconds=30.))
        for index in range(64):
            row=pose(float(index+3),80.5e-6,80.5e-6);row['pan']=.2*(index%2)
            tracker.observe(sample(float(index+3),76),row)
        self.assertEqual(len(tracker.observations),64)
        value=sample(67.,133,(.032,.025,.03));value['retired_reference_id']=79
        try:tracker.observe(value,None)
        except policy.RetiredCheckpoint as event:tracker.invalidate_retired_checkpoint(event)
        else:self.fail('No retirement transaction')
        tiny=NS(info=NS(resolution=1e-6,width=200,height=200,origin=NS(position=NS(x=0.,y=0.))))
        costs=np.zeros((200,200),dtype=np.int16)
        # The actual unchanged clearance helper fails before creating a huge
        # disk because this tiny physical map cannot hold the .65m footprint.
        with patch.object(policy.heapq,'heappop',wraps=policy.heapq.heappop)as popped:
            with patch.object(policy,'safe_segment',wraps=policy.safe_segment)as segments:
                checked=Mock(wraps=clearance)
                chosen=plan(tracker,tiny,costs,current=dict(x=90.5e-6,y=80.5e-6,yaw=0.),clearance=checked)
        self.assertIsNone(chosen)
        self.assertGreater(checked.call_count,64)
        self.assertLessEqual(checked.call_count,64*(64+2))
        self.assertLessEqual(popped.call_count,64*64)
        self.assertEqual(segments.call_count,0)


if __name__=='__main__':unittest.main()
