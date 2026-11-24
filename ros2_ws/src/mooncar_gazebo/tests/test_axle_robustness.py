import copy,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4];sys.path.insert(0,str(ROOT/'scripts'))
from r5_axle_robustness import MATRIX,validate,qualify_grid

class RobustnessTests(unittest.TestCase):
    def test_campaign_entry_import_and_matrix(self):
        import run_r5_axle_robustness as entry
        self.assertTrue(callable(entry.signature));self.assertEqual(entry.MATRIX,MATRIX)
    def data(self,step):
        p=dict(passed=step==.001,world_pose_complete=True,per_step_grid=True,frames_valid=True,
            odometry={k:dict(passed=True) for k in ('ros','native')},contacts={str(i):dict(complete=True) for i in range(4)})
        s=dict(mode='paused_steps',step=step,motion=dict(before=dict(iteration=5000,sim=5.),
            after=dict(iteration=5000+round(20/step),sim=25.)))
        return p,s
    def test_matrix_budget_and_order(self):
        self.assertEqual(len(MATRIX),25);self.assertEqual(MATRIX[6],('flat',.0005,'zero',0.,0.))
        self.assertEqual([x[0] for x in MATRIX[13:]],['ramp']*4+['curve']*4+['survey']*4)
    def test_actual_half_ms_qualified_without_rewriting_observations(self):
        p,s=self.data(.0005);old=copy.deepcopy((p,s));r=qualify_grid(p,s)
        self.assertTrue(r['passed']);self.assertEqual(r['expected_motion_steps'],40000)
        self.assertFalse(r['legacy_fixed_1ms_gate_passed']);self.assertEqual((p,s),old)
    def test_one_ms_old_behavior_preserved(self):
        p,s=self.data(.001);self.assertTrue(qualify_grid(p,s)['passed'])
    def test_missing_frames_contacts_odom_or_steps_still_fail(self):
        for key in ('world_pose_complete','per_step_grid','frames_valid'):
            p,s=self.data(.0005);p[key]=False;self.assertFalse(qualify_grid(p,s)['passed'])
        for key,field in (('odometry','passed'),('contacts','complete')):
            p,s=self.data(.0005);next(iter(p[key].values()))[field]=False;self.assertFalse(qualify_grid(p,s)['passed'])
    def test_wrong_step_count_and_elapsed_or_schedule_rejected(self):
        for change in ('count','elapsed','mode'):
            p,s=self.data(.0005)
            if change=='count':s['motion']['after']['iteration']-=1
            elif change=='elapsed':s['motion']['after']['sim']+=.0001
            else:s['mode']='legacy'
            self.assertFalse(qualify_grid(p,s)['passed'])
    def test_unsupported_scene_step_rejected(self):
        validate('flat',.00025)
        for scene,step in (('unknown',.001),('flat',.002),('flat',float('nan'))):
            with self.assertRaises(ValueError):validate(scene,step)

