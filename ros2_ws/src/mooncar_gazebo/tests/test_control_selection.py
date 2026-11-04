"""Diagnostics must not silently substitute D-only or relax the approved gates."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from check_r5_control import select_candidate

class SelectionTests(unittest.TestCase):
    def result(self,root,profile,passed,seconds=1.):
        folder=Path(root)/('case-'+profile+'-torque');folder.mkdir()
        data=dict(passed=passed,sweep=[dict(started=0.,finished=seconds)],step_observation=dict(
            holds=[dict(axes=[dict(velocity_rms=.01,peak_to_peak_rad=.001)])]))
        (folder/'result.json').write_text(json.dumps(data))

    def test_failed_approved_profiles_never_select_d0(self):
        with tempfile.TemporaryDirectory() as root:
            self.result(root,'d0',True)
            for profile in ('A','B','C'):self.result(root,profile,False)
            self.assertIsNone(select_candidate(root)['selected'])

    def test_fixed_ranking_and_tie_order(self):
        with tempfile.TemporaryDirectory() as root:
            for profile in ('A','B','C'):self.result(root,profile,True)
            self.assertEqual(select_candidate(root)['selected'],'A')
        with tempfile.TemporaryDirectory() as root:
            self.result(root,'A',True,2.);self.result(root,'B',True,1.)
            self.result(root,'C',False)
            self.assertEqual(select_candidate(root)['selected'],'B')

    def test_missing_comparison_cannot_select_or_claim_all_failed(self):
        with tempfile.TemporaryDirectory() as root:
            self.result(root,'A',True)
            result=select_candidate(root)
            self.assertIsNone(result['selected'])
            self.assertEqual(result['status'],'incomplete_candidate_comparison')

if __name__=='__main__':unittest.main()
