"""No-ROS regression for the observed straight/turn admission mistake."""
import copy
import importlib.util
from pathlib import Path
import unittest

def load(name):
    path=Path(__file__).resolve().parents[2]/'scripts'/(name+'.py')
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module

motion=load('check_r5_coplanar_round').motion_qualification
campaign=load('run_r5_coplanar_validation').qualified_attempt

class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.diagnostic=dict(eligible=False,checks={k:True for k in (
            'data_complete','ground_contact_gate','stop_zero','observer_flush_confirmed',
            'receiver_verified','raw_fixture_passed','capture_drained','image_content',
            'origin_qualification','gate','allocation','contact_source','no_critical_errors','finite_geometry')})
        self.diagnostic['checks']['valid_exit']=False
        self.result=dict(passed=True,raw_fixture_passed=True,
            turn_slip_probe=dict(endpoint_xy_m=[1.989,-.000158],max_lateral_m=.000158))

    def test_straight_is_not_a_turn_and_does_not_rewrite_history(self):
        original=copy.deepcopy(self.diagnostic)
        self.assertTrue(motion(self.diagnostic,self.result,.1,0)['passed'])
        self.assertEqual(original,self.diagnostic)
        self.assertFalse(motion(self.diagnostic,self.result,0,.2)['passed'])

    def test_missing_and_failed_data_still_fail(self):
        for key in self.diagnostic['checks']:
            if key=='valid_exit':continue
            with self.subTest(key=key):
                bad=copy.deepcopy(self.diagnostic);bad['checks'][key]=False
                self.assertFalse(motion(bad,self.result,.1,0)['passed'])

    def test_straight_limits_unchanged(self):
        for x,lateral in ((.9,0),(2.6,0),(1.989,.011)):
            with self.subTest(x=x,lateral=lateral):
                bad=copy.deepcopy(self.result);bad['turn_slip_probe']=dict(endpoint_xy_m=[x,0],max_lateral_m=lateral)
                self.assertFalse(motion(self.diagnostic,bad,.1,0)['passed'])

    def test_failed_underlying_fixture_is_not_waived(self):
        for key in ('error','origin_error','turn_slip_error'):
            with self.subTest(key=key):
                bad=copy.deepcopy(self.result);bad[key]='failure'
                self.assertFalse(motion(self.diagnostic,bad,.1,0)['passed'])

    def test_mixed_commands_and_missing_results_fail(self):
        self.assertFalse(motion(self.diagnostic,self.result,.1,.2)['passed'])
        self.assertFalse(motion(self.diagnostic,{},.1,0)['passed'])

    def test_typed_campaign_proof_required(self):
        attempt=dict(exit_code=0,qualification=self.diagnostic,
            loader=dict(passed=True,motion_qualification=dict(passed=True)))
        self.assertTrue(campaign(attempt))
        for key in ('loader','qualification'):
            bad=copy.deepcopy(attempt);bad[key]=None;self.assertFalse(campaign(bad))
        bad=copy.deepcopy(attempt);bad['exit_code']=1;self.assertFalse(campaign(bad))
        bad=copy.deepcopy(attempt);bad['loader']['motion_qualification']={};self.assertFalse(campaign(bad))

if __name__=='__main__':unittest.main()
