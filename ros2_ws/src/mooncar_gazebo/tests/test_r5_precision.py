import copy
import tempfile
from pathlib import Path
import unittest
import numpy as np
from r5_physics_profiles import main_step,profile_record
from r5_suspension_protocol import declared_matrix
from r5_csv import numeric_blocks,read_numeric_csv,aligned_observations
from qualify_r5_static_gate import qualify

class PrecisionTests(unittest.TestCase):
    def test_default_and_precision_are_separate_from_support_model(self):
        self.assertEqual(main_step('legacy_1ms'),.001)
        self.assertEqual(main_step('r5_precision_v1'),.00025)
        with self.assertRaises(ValueError):main_step('unknown')
        old=declared_matrix();new=declared_matrix('r5_precision_v1')
        self.assertEqual(len(new),17)
        self.assertEqual([x[:3]+x[4:] for x in old],[x[:3]+x[4:] for x in new])
        self.assertEqual([x[3] for x in new],[.00025]*10+[.0005,.00025,.0005,.00025]+[.00025]*3)

    def test_chunk_boundary_missing_row_and_bad_width(self):
        with tempfile.TemporaryDirectory() as folder:
            a=Path(folder)/'a.csv';b=Path(folder)/'b.csv'
            a.write_text('iteration,t,spring_q,force_cmd\n'+''.join(f'{i},{i*.00025},0,nan\n' for i in range(1,8200)))
            b.write_bytes(a.read_bytes())
            self.assertEqual([len(x) for x in numeric_blocks(a)],[4096,4096,7])
            self.assertEqual(len(read_numeric_csv(a,usecols=('t',))),8199)
            self.assertTrue(aligned_observations(a,b))
            b.write_text(b.read_text().rsplit('\n',2)[0]+'\n')
            self.assertFalse(aligned_observations(a,b))
            b.write_text('iteration,t\n1,.00025,0\n')
            with self.assertRaises(ValueError):read_numeric_csv(b,usecols=('t',))

    def test_precision_gate_uses_maxima_and_repeatability(self):
        joint=dict(mechanics_passed=True,measured=dict(q_mean_m=.00025,transmitted_fz_mean_N=-3),
            formula=dict(pre_constraint_implicit_force_mean_N=3,axial_compression_error_fraction=.025,
                axial_compression_error_max_m=.00025,axial_force_balance_error_max_fraction=.025))
        vehicles=[]
        for step,drive in [(.001,True),(.00025,True),(.00025,True),(.00025,False)]:
            j=copy.deepcopy(joint)
            if step==.001:j['formula']['axial_compression_error_fraction']=.1
            vehicles.append(dict(specification=dict(kind='vehicle',step=step,drive_enabled=drive),
                body_tail_speed_max_m_s=0,contact={'wheel':dict(empty_fraction=0)},joints={'joint':j},drive_commands_absent=not drive))
        analysis=dict(physics_profile=profile_record('r5_precision_v1'),results=[dict(specification=dict(kind='joint'))]*8+vehicles,
            ledger=dict(status='completed',freeze_unchanged=True,attempts=[dict(cleanup_passed=True,failure=None)]*12),native_bench_passed=True,fine_axial_balance_passed=True)
        self.assertTrue(qualify(analysis)['passed'])
        invalid=copy.deepcopy(analysis);invalid['results'][9]['joints']['joint']['formula']['axial_compression_error_max_m']=.000301
        self.assertFalse(qualify(invalid)['passed'])
        invalid=copy.deepcopy(analysis);invalid['results'][10]['joints']['joint']['measured']['q_mean_m']+=.000011
        self.assertFalse(qualify(invalid)['passed'])

if __name__=='__main__':unittest.main()
