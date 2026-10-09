"""Frame, motion-window, causality and stop regressions for origin diagnostics."""
import math
from pathlib import Path
import sys
import unittest
import json
import tempfile
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from r5_origin_analysis import rotation,trajectory,unwrap_yaw,contact_summary,WHEELS
from run_r5_origin_campaign import BASELINE,eligible_candidate,pair_inputs

class OriginDiagnosticTests(unittest.TestCase):
    def test_world_transform_rotates_body_offset(self):
        q=[math.cos(math.pi/4),0,0,math.sin(math.pi/4)]
        np.testing.assert_allclose(rotation(q)@np.array([-.157,0,.076]),[0,-.157,.076],atol=1e-12)

    def test_rotating_offset_is_not_center_translation(self):
        t=np.linspace(0,20,2001);yaw=.2*t
        xyz=np.column_stack((.157*np.cos(yaw),.157*np.sin(yaw),np.zeros(len(t))))
        result,*_=trajectory(t,xyz,yaw)
        self.assertAlmostEqual(result['circle_fit']['radius_m'],.157,places=8)
        np.testing.assert_allclose(result['circle_fit']['center_world_m'],[0,0],atol=1e-12)

    def test_zero_turn_has_no_fabricated_center(self):
        result,*_=trajectory(np.arange(10.),np.zeros((10,3)),np.zeros(10))
        self.assertIsNone(result['circle_fit']);self.assertIsNone(result['instantaneous_center_median_world_m'])

    def test_unwrap_pi_crossing(self):
        yaw=np.linspace(2.9,6.5,40)
        data=np.zeros(40,dtype=[(k,float) for k in ('qw','qx','qy','qz')])
        data['qw']=np.cos(yaw/2);data['qz']=np.sin(yaw/2)
        np.testing.assert_allclose(unwrap_yaw(data)-unwrap_yaw(data)[0],yaw-yaw[0],atol=1e-12)

    def test_invalid_quaternion_fails(self):
        with self.assertRaises(ValueError):rotation([0,0,0,0])

    def test_duplicate_time_fails(self):
        with self.assertRaises(ValueError):trajectory([0,1,1],np.zeros((3,3)),np.zeros(3))

    def test_inverse_frame_returns_body_coordinates(self):
        q=np.array([[1,0,0,0],[math.cos(.5),0,0,math.sin(.5)]])
        r=rotation(q);local=np.array([.157,.16555,.076]);world=r@local
        np.testing.assert_allclose(np.einsum('nji,nj->ni',r,world),np.tile(local,(2,1)),atol=1e-12)

    def test_baseline_is_fixed_eight_attempts(self):
        self.assertEqual(len(BASELINE),8)
        self.assertEqual(sum(c[2]==.2 and c[3]==0 for c in BASELINE),2)
        self.assertEqual(sum(c[2]==-.2 and c[3]==0 for c in BASELINE),2)
        self.assertEqual([c[3] for c in BASELINE[-2:]],[math.pi/2]*2)

    def test_contact_asymmetry_alone_does_not_authorize_patch(self):
        self.assertFalse(eligible_candidate({'prediction':'less drift','single_change':'friction'}))

    def test_specific_frozen_prediction_required(self):
        d=dict(specific_error='wrong frame conversion',single_change='transform',prediction='consistent with worldPose',
               unchanged_parameters_verified=True,frozen_before_execution=True)
        self.assertTrue(eligible_candidate(d));d['frozen_before_execution']=False;self.assertFalse(eligible_candidate(d))

    def test_input_budget_mismatch_fails(self):
        def result(turn):return dict(passed=True,formal=dict(command_window=dict(duration_s=20.,
            wheel_budget=dict(linear_integral_m=0.,angular_integral_rad=turn))))
        self.assertTrue(pair_inputs(result(4),result(4))['passed'])
        self.assertFalse(pair_inputs(result(4),result(4.003))['passed'])
        self.assertFalse(pair_inputs(result(4),result(-4))['passed'])

    def test_failed_attempt_cannot_qualify_pair(self):
        a=dict(passed=False,formal=dict(command_window=dict(duration_s=20.,wheel_budget=dict(linear_integral_m=0.,angular_integral_rad=4.))))
        self.assertFalse(pair_inputs(a,a)['passed'])

    def test_missing_force_is_not_zero_force(self):
        with tempfile.TemporaryDirectory() as d:
            for w in WHEELS:
                rows=[dict(header=dict(stamp=dict(nsec=i*10_000_000)),contact=[dict(wrench=[{}])]) for i in range(3)]
                (Path(d)/(w+'-contact.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in rows))
            values=contact_summary(d,0,.02)
            self.assertTrue(all(v['force_samples']==0 and v['force_mean_raw'] is None for v in values.values()))

    def test_explicit_zero_force_preserves_source_meaning(self):
        with tempfile.TemporaryDirectory() as d:
            for w in WHEELS:
                rows=[dict(header=dict(stamp=dict(nsec=i*10_000_000)),contact=[dict(wrench=[dict(body1Wrench=dict(force={}),body2Wrench=dict(force={}))])]) for i in range(3)]
                (Path(d)/(w+'-contact.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in rows))
            values=contact_summary(d,0,.02)
            self.assertTrue(all(v['force_samples']==3 and v['force_mean_raw']==[0,0,0] for v in values.values()))

    def test_contact_source_gap_fails_coverage(self):
        with tempfile.TemporaryDirectory() as d:
            for w in WHEELS:
                rows=[dict(header=dict(stamp=dict(nsec=i)),contact=[]) for i in (0,200_000_000)]
                (Path(d)/(w+'-contact.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in rows))
            self.assertTrue(all(not v['complete'] for v in contact_summary(d,0,.2).values()))

    def test_native_integer_nanoseconds_avoid_float_search_overshoot(self):
        import importlib.util
        p=Path(__file__).resolve().parents[4]/'reports/evidence/terrain-v2/r5-origin-20261009/derive_origin_evidence.py'
        spec=importlib.util.spec_from_file_location('origin_offline',p);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        row={'header':{'stamp':{'sec':'8','nsec':243_000_000}}}
        ns=module.stamp_ns(row)
        t=np.array([8.243,8.244]);grid=np.rint(t*1e9).astype(np.int64)
        self.assertEqual(int(np.searchsorted(grid,ns)),0)
        self.assertEqual(int(grid[0]),ns)

if __name__=='__main__':unittest.main()
