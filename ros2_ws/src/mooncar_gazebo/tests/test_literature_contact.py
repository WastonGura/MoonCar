"""Dimensional, support and data-isolation checks for the research projection."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
import r5_literature_contact as m
import numpy as np

class LiteratureContactTests(unittest.TestCase):
    def test_static_support_and_refinement(self):
        a,fa=m.reference(3.6,points=4097);b,fb=m.reference(3.6,points=8193)
        self.assertAlmostEqual(a['normal_integral_N'],3.6,places=8)
        self.assertLess(abs(a['sinkage_m']-b['sinkage_m']),1e-8)
        np.testing.assert_allclose(fa([.5,3,10]),fb([.5,3,10]),rtol=1e-6)
    def test_odd_force_and_zero(self):
        _,f=m.reference(3.6)
        np.testing.assert_allclose(f([-3,0,3]),[-f(3),0,f(3)])
    def test_positive_mobilization_and_cap(self):
        a,f=m.reference(3.6)
        y=f(np.linspace(0,80,100))
        self.assertTrue(np.all(np.diff(y)>=0))
        self.assertLessEqual(y[-1],a['effective_lateral_cap'])
    def test_fit_and_holdout_disjoint(self):
        self.assertFalse(set(m.FIT_BETA_DEG)&set(m.HOLDOUT_BETA_DEG))
        a=m.fit(3.6)
        self.assertTrue(a['training_passed'])
        self.assertFalse(a['measured_force_data_used'])
        self.assertFalse(a['hardware_calibrated'])
    def test_reference_not_equal_to_mu(self):
        a,_=m.reference(3.6)
        self.assertGreater(a['effective_lateral_cap'],m.SOIL['external_mu'])
    def test_invalid_inputs_rejected(self):
        for load in [0,-1,float('nan')]:
            with self.assertRaises(ValueError):m.reference(load)
        with self.assertRaises(ValueError):m.reference(3.6,points=128)
        with self.assertRaises(ValueError):m.gazebo_ratio([1],0,.2)

if __name__=='__main__':unittest.main()
