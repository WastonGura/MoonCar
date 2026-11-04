"""Publication invariants for the window navigation authority."""
from pathlib import Path
import sys
import unittest
import copy
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from offline_rgbd_window.navigation import WindowOutputGate
from autonomous_survey import AutonomousSurvey

class WindowNavigationTests(unittest.TestCase):
    def row(self):
        return dict(output=True,stamp_ns=1_000_000_000,gauge_stamp_ns=500_000_000,
            received_wall_ns=2_000_000_000,cache_age_s=.5,position=[1.,2.,.1],
            covariance_conditional=[[.001,.0001,0],[.0001,.002,0],[0,0,.003]])
    def check(self,row,gate=None,stamp=1_000_000_000,now=1_100_000_000,wall=2_100_000_000,fault=None):
        return (gate or WindowOutputGate()).accept(row,stamp,now,wall,fault)
    def test_position_and_complete_covariance_are_preserved_and_owned(self):
        r=self.row();answer,reason=self.check(r)
        self.assertEqual(reason,'accepted');np.testing.assert_array_equal(answer['xyz'],r['position'])
        np.testing.assert_array_equal(answer['covariance'],r['covariance_conditional'])
        r['position'][0]=100.;self.assertEqual(answer['xyz'][0],1.)
    def test_no_output_and_source_fault_never_publish(self):
        r=self.row();r['output']=False;self.assertIsNone(self.check(r)[0])
        self.assertIsNone(self.check(self.row(),fault='clock_regressed')[0])
    def test_original_exposure_is_required_not_planned_or_current_clock(self):
        self.assertEqual(self.check(self.row(),stamp=1_100_000_000)[1],'exposure_mismatch')
    def test_stale_future_and_wall_old_results_do_not_renew_pose(self):
        for kw in (dict(now=1_400_000_000),dict(now=800_000_000),dict(wall=2_600_000_000)):
            self.assertIsNone(self.check(self.row(),**kw)[0])
    def test_duplicate_pose_and_gauge_changes_rejected(self):
        gate=WindowOutputGate();self.assertIsNotNone(self.check(self.row(),gate)[0])
        self.assertEqual(self.check(self.row(),gate)[1],'nonincreasing_pose')
        r=self.row();r.update(stamp_ns=1_100_000_000,gauge_stamp_ns=600_000_000)
        self.assertEqual(self.check(r,gate,stamp=r['stamp_ns'])[1],'gauge_changed')
    def test_invalid_covariances_and_expired_cache_rejected(self):
        for C in (np.eye(3)*np.nan,np.diag([1.,-1.,1.]),np.ones((2,2))):
            r=self.row();r['covariance_conditional']=C;self.assertIsNone(self.check(r)[0])
        r=self.row();r['cache_age_s']=1.1;self.assertEqual(self.check(r)[1],'cache_age')
    def test_window_task_does_not_consume_or_trigger_legacy_revisit(self):
        from unittest.mock import Mock
        mission=AutonomousSurvey(Mock(),localization_revisit_enabled=False)
        mission.revisit_sample=Mock(side_effect=AssertionError('legacy observation should not be consumed'))
        mission.update_localization_revisit();mission.revisit_sample.assert_not_called()
        self.assertIsNone(mission.revisit_policy.pending)
    def test_legacy_task_keeps_explicit_revisit_mode(self):
        from unittest.mock import Mock
        mission=AutonomousSurvey(Mock());mission.revisit_sample=Mock(return_value=None)
        mission.update_localization_revisit();mission.revisit_sample.assert_called_once()

if __name__=='__main__':unittest.main()
