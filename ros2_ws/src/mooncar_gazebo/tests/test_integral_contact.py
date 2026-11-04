"""Physical/numerical invariants of the continuous contact integral."""
import sys,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
from r5_integral_contact import build,ratio,compliance,initial_slope,kernel
from r5_literature_contact import SOIL

class IntegralContactTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.law=build(dict(load_N=3.572514673566879,soil=SOIL,effective_lateral_cap=.39670920433114515),32)
    def test_positive_monotone_bounded_odd_force(self):
        x=np.r_[0,np.logspace(-12,12,721)];q=ratio(x,self.law)
        self.assertTrue(np.all(np.diff(q)>=-1e-14));self.assertTrue(np.all(q>=0));self.assertTrue(np.all(q<=self.law['cap']+1e-14))
        np.testing.assert_allclose(ratio(-x,self.law),-q,rtol=0,atol=0)
        self.assertTrue(np.all((-q*x)<=0))
    def test_initial_tangent_and_cohesive_tail(self):
        self.assertAlmostEqual(float(ratio(1e-10,self.law))/1e-10/initial_slope(self.law),1,places=8)
        deficit=self.law['cap']-float(ratio(1e7,self.law))
        self.assertLess(abs(deficit*1e7/self.law['leading_deficit_coefficient']-1),1e-4)
    def test_native_secant_and_continuous_zero_spin(self):
        for v in [-.03,-.001,0,.001,.03]:
            c=float(compliance(v,.03,self.law))
            self.assertGreater(c,0)
            if v:self.assertAlmostEqual(abs(v)/c,self.law['load_N']*abs(float(ratio(v/.03,self.law))),places=12)
            self.assertEqual(float(compliance(v,0,self.law)),abs(v)/(self.law['load_N']*self.law['cap']))
        with self.assertRaises(ValueError):compliance(.01,-1,self.law)
    def test_kernel_small_argument_and_segment_convergence(self):
        l,r=kernel(np.array([0,1e-12,.249999999,.25]))
        self.assertEqual(l[0]+r[0],0);self.assertAlmostEqual((l[1]+r[1])/1e-12,.5,places=12)
        self.assertLess(abs((l[2]+r[2])-(l[3]+r[3])),1e-8)
        coarse=self.law;fine=build(dict(load_N=coarse['load_N'],soil=SOIL,effective_lateral_cap=coarse['cap']),64)
        x=np.logspace(-8,6,200)
        self.assertLess(float(np.max(abs(ratio(x,coarse)/ratio(x,fine)-1))),1e-4)
    def test_pressure_distribution_and_cap_preserved(self):
        p=np.asarray(self.law['pressure_density']);self.assertEqual(p[0],0);self.assertEqual(p[-1],0)
        np.testing.assert_equal(p,p[::-1]);self.assertAlmostEqual(float(np.trapezoid(p,np.linspace(0,1,len(p)))),1,places=14)
        self.assertEqual(self.law['source_soil'],SOIL)
        self.assertEqual(float(ratio(float('inf'),self.law)),self.law['cap'])

if __name__=='__main__':unittest.main()
