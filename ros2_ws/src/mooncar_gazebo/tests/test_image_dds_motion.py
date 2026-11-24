"""Actual physical command durations must survive PreUpdate observation offsets."""
import copy, json, sys, unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[4];sys.path.insert(0,str(ROOT/'scripts'))
from image_dds_motion import physical_command_window
from r5_d0_analysis import command_window,compare_pair
import test_paused_steps as step_tests

class PhysicalInputTests(unittest.TestCase):
    def fixture(self):return step_tests.PausedTests().fixture()
    def check(self,data,result):return physical_command_window(data,result,.1,0.,command_window)
    def test_heartbeat_observation_offsets_do_not_change_physical_duration(self):
        data,result=self.fixture();data['input_seen_sim'][25000:]+=.001
        window=self.check(data,result);self.assertTrue(window['passed']);self.assertEqual(window['duration_s'],20.)
        self.assertNotEqual(window['duration_s'],window['observed_message_times']['duration_s'])
    def test_actual_extra_steps_are_not_ignored(self):
        data,result=self.fixture();data['input_lx'][25000:25003]=.1
        self.assertFalse(self.check(data,result)['passed'])
    def test_missing_physical_step_rejects(self):
        data,result=self.fixture();self.assertFalse(self.check(np.delete(data,10000),result)['passed'])
    def test_duplicate_physical_step_rejects(self):
        data,result=self.fixture();self.assertFalse(self.check(np.insert(data,10000,data[10000]),result)['passed'])
    def test_wrong_native_command_rejects(self):
        data,result=self.fixture();data['input_lx'][10000]=.11;self.assertFalse(self.check(data,result)['passed'])
    def test_zero_dropout_rejects(self):
        data,result=self.fixture();data['input_lx'][10000]=0;self.assertFalse(self.check(data,result)['passed'])
    def test_forged_future_observation_still_rejects(self):
        data,result=self.fixture();data['input_seen_sim'][10000]+=1;self.assertFalse(self.check(data,result)['passed'])
    def test_changed_wheel_profile_still_rejects(self):
        data,result=self.fixture();data['front_left_wheel_joint_velocity_cmd'][10000]*=-1;self.assertFalse(self.check(data,result)['passed'])
    def test_motion_phase_duration_still_required(self):
        data,result=self.fixture();result['schedule']['motion']['after']['sim']+=.01;self.assertFalse(self.check(data,result)['passed'])
    def test_failed_raw_pair_reproduces_offline_without_rewriting_results(self):
        root=ROOT/'reports/evidence/terrain-v2/r5-dds-release-20261009/campaign'
        paths=sorted(root.glob('*calibration-flat*/result.json'))
        if len(paths)!=4:self.skipTest('Raw failed pairing campaign not on host')
        analyzed=[]
        for p in paths:
            raw=p.read_bytes();result=json.loads(raw);data=np.genfromtxt(p.parent/'physics-steps.csv',delimiter=',',names=True)
            window=physical_command_window(data,result,0.,.2,command_window)
            self.assertTrue(window['passed']);self.assertEqual(window['duration_s'],20.)
            candidate=copy.deepcopy(result);candidate['formal']['command_window']=window;analyzed.append(candidate)
            self.assertEqual(p.read_bytes(),raw)
        self.assertFalse(compare_pair(json.loads(paths[2].read_text()),json.loads(paths[3].read_text()))['valid'])
        self.assertTrue(compare_pair(analyzed[2],analyzed[3])['valid'])

if __name__=='__main__':unittest.main()
