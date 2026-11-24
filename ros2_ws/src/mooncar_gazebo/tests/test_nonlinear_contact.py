"""Constitutive properties and zero-spin handling, not simulated rover outcomes."""
import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
import numpy as np
import r5_nonlinear_contact as m

class NonlinearContactTests(unittest.TestCase):
    law=(.397,.58,.72,.21)
    def test_odd_monotone_and_saturation(self):
        x=np.r_[0,np.geomspace(1e-8,1e4,100)]
        y=m.ratio_from_lambda(x,*self.law)
        self.assertTrue(np.all(np.diff(y)>=0));self.assertLessEqual(y.max(),self.law[0])
        np.testing.assert_array_equal(m.ratio_from_lambda(-x,*self.law),-y)
        self.assertAlmostEqual(y[-1],self.law[0],places=12)
    def test_small_slip_tangent(self):
        cap,w,a,b=self.law;slope=cap*(w*a+(1-w)*b)
        self.assertAlmostEqual(float(m.ratio_from_lambda(1e-8,*self.law))/1e-8,slope,places=8)
        self.assertAlmostEqual(m.native_compliance(0,.03,3.6,*self.law),.03/(3.6*slope),places=14)
    def test_zero_spin_continuous_limit(self):
        self.assertEqual(m.native_compliance(0,0,3.6,*self.law),0)
        expected=.02/(3.6*self.law[0])
        self.assertEqual(m.native_compliance(.02,0,3.6,*self.law),expected)
        self.assertAlmostEqual(m.native_compliance(.02,1e-12,3.6,*self.law),expected,places=12)
    def test_native_secant_and_dissipation(self):
        for beta in (-85,-45,-1,1,45,85):
            v=.03*math.tan(math.radians(beta));c=m.native_compliance(v,.03,3.6,*self.law)
            force=-v/c;expected=-3.6*float(m.ratio_from_lambda(v/.03,*self.law))
            self.assertAlmostEqual(force,expected,places=12);self.assertLessEqual(force*v,0)
    def test_homogeneity(self):
        a=m.native_compliance(.02,.03,3.6,*self.law)
        self.assertAlmostEqual(m.native_compliance(.04,.06,3.6,*self.law),2*a)
        self.assertAlmostEqual(m.native_compliance(.02,.03,7.2,*self.law),a/2)
    def test_invalid_and_separate_domains(self):
        self.assertFalse(set(m.TRAIN_DEG)&set(m.HOLDOUT_DEG))
        for v in (float('nan'),float('inf')):
            with self.assertRaises(ValueError):m.native_compliance(v,.03,3.6,*self.law)
        with self.assertRaises(ValueError):m.native_compliance(.02,-.03,3.6,*self.law)
        with self.assertRaises(ValueError):m.ratio_from_lambda(1,.3,1,.5,.2)

if __name__=='__main__':unittest.main()
