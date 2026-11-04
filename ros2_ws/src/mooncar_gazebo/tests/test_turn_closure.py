"""Regression for typed release gates and isolated-round filenames."""
import importlib.util
from pathlib import Path
import unittest

repo=Path(__file__).resolve().parents[4]
spec=importlib.util.spec_from_file_location('turn_closure_tests',repo/'scripts/run_r5_turn_closure.py')
campaign=importlib.util.module_from_spec(spec);spec.loader.exec_module(campaign)

class TestClosure(unittest.TestCase):
    def test_geometry_limits(self):
        good=dict(endpoint_xy_m=[0,.0002],trajectory=dict(maximum_displacement_m=.0002,unwrapped_turn_rad=3.6))
        self.assertTrue(campaign.geometry(good,0,.2))
        self.assertFalse(campaign.geometry(good,0,-.2))
        bad=dict(good,trajectory=dict(maximum_displacement_m=.017,unwrapped_turn_rad=3.6))
        self.assertFalse(campaign.geometry(bad,0,.2))
        self.assertFalse(campaign.geometry({},0,.2))
    def test_straight_is_not_turn(self):
        straight=dict(endpoint_xy_m=[1.99,0],max_lateral_m=.001,trajectory=dict(maximum_displacement_m=1.99))
        self.assertTrue(campaign.geometry(straight,.1,0))
        self.assertFalse(campaign.geometry(straight,0,.2))
    def test_generated_fixture_step_scope(self):
        code=(repo/'scripts/prepare_r5_turn_closure.py').read_text()
        self.assertIn("ET.parse(world_src).findtext('world/physics/max_step_size')",code)
        self.assertNotIn("'--step', src_tree.findtext",code)
    def test_campaign_names_and_no_resume(self):
        code=(repo/'scripts/run_r5_turn_closure.py').read_text()
        self.assertIn("run_name=out.name+'-'+name",code)
        self.assertIn("raise FileExistsError('No resume or overwrite')",code)
        self.assertEqual(len(campaign.ROUNDS),12)

if __name__=='__main__':unittest.main()
