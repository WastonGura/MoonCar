import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'scripts'))
from r5_adaptive_turn import (AdaptiveAllocation, rotation, fit_scan_motion,
                              body_delta, scan_points, wheel_allocation)


class AdaptiveTurnTests(unittest.TestCase):
    def room(self):
        u = np.linspace(-2, 2, 180)
        return np.concatenate((np.column_stack((u, np.full_like(u, 3))),
            np.column_stack((np.full_like(u, -3), u)),
            np.column_stack((u, np.full_like(u, -2.5)))))

    def test_independent_sensor_translation_rotation(self):
        a = self.room(); yaw=.02; shift=np.array([.004, -.003])
        b=(a-shift) @ rotation(yaw)
        r=fit_scan_motion(a,b,seed_yaw=yaw)
        self.assertTrue(r['valid'],r)
        np.testing.assert_allclose(r['translation'],shift,atol=1e-5)
        self.assertAlmostEqual(r['yaw'],yaw,places=5)

    def test_single_wall_is_not_observable_even_with_zero_residual(self):
        a=np.column_stack((np.linspace(-3,3,200),np.ones(200)))
        r=fit_scan_motion(a,a)
        self.assertFalse(r['valid'])

    def test_sparse_and_nonfinite_fail(self):
        self.assertFalse(fit_scan_motion(np.zeros((10,2)),np.zeros((10,2)))['valid'])
        a=self.room();a[0,0]=float('nan')
        self.assertFalse(fit_scan_motion(a,self.room())['valid'])

    def test_lidar_offset_is_removed(self):
        yaw=.04;offset=np.array([.11,0.])
        r={'valid':True,'translation':(rotation(yaw)@offset-offset).tolist(),'yaw':yaw}
        np.testing.assert_allclose(body_delta(r),[0,0],atol=1e-12)

    def test_scan_preserves_angles_and_excludes_invalid(self):
        p=scan_points([None,1.,float('inf'),2.],0.,np.pi/2,.05,15.)
        np.testing.assert_allclose(p,[[0,1],[0,-2]],atol=1e-12)

    def test_both_directions_use_same_pivot_feedback_sign(self):
        a,b=AdaptiveAllocation(),AdaptiveAllocation()
        x=a.update(100000000,.1,.02,.2,0.,.2)
        y=b.update(100000000,.1,-.02,-.2,0.,-.2)
        self.assertLess(x['front'],a.initial_front)
        self.assertAlmostEqual(x['front'],y['front'])

    def test_opposite_pivot_adapts_opposite_way(self):
        c=AdaptiveAllocation();r=c.update(100000000,.1,-.02,.2,0.,.2)
        self.assertGreater(r['front'],c.initial_front)

    def test_nonfinite_wrong_sign_stale_and_duplicate_feedback(self):
        c=AdaptiveAllocation()
        self.assertFalse(c.update(1,.1,float('nan'),.2,0.,.2)['valid'])
        self.assertFalse(c.update(1,.1,.02,-.2,0.,.2)['valid'])
        self.assertFalse(c.update(1,.3,.02,.2,0.,.2)['valid'])
        self.assertTrue(c.update(100000000,.1,.02,.2,0.,.2)['valid'])
        self.assertFalse(c.update(100000000,.1,.02,.2,0.,.2)['valid'])

    def test_rate_bounds_saturation_and_zero_reset(self):
        c=AdaptiveAllocation()
        old=c.front
        for i in range(1,101):
            r=c.update(i*100000000,.1,.14,.2,0.,.2)
            self.assertLessEqual(abs(c.front-old),.002+1e-12);old=c.front
            self.assertGreaterEqual(c.front,.95)
        self.assertTrue(r['saturated'])
        r=c.update(20000000000,.1,0.,0.,0.,0.)
        self.assertEqual(r['reason'],'inactive');self.assertEqual(c.front,c.initial_front)

    def test_straight_and_stop_unchanged_bad_feedback_inhibits_turn(self):
        for native in ([1,1,1,1],[0,0,0,0]):
            result,mode=wheel_allocation(native,.96,False)
            np.testing.assert_array_equal(result,native);self.assertEqual(mode,'inactive')
        result,mode=wheel_allocation([-1,1,-1,1],.96,False)
        np.testing.assert_array_equal(result,[0,0,0,0]);self.assertEqual(mode,'inhibited')

    def test_average_side_input_preserved(self):
        result,mode=wheel_allocation([-1,1,-1,1],.96,True)
        self.assertEqual(mode,'adaptive')
        self.assertAlmostEqual((result[0]+result[2])/2,-1)
        self.assertAlmostEqual((result[1]+result[3])/2,1)

    def test_reject_unexpected_ordering_and_out_of_scope(self):
        with self.assertRaises(ValueError):wheel_allocation([-1,1,-2,1],.96,True)
        result,mode=wheel_allocation([-1,1,-1,1],.8,True)
        self.assertEqual(mode,'inhibited')
        np.testing.assert_array_equal(result,[0,0,0,0])


if __name__=='__main__':unittest.main()
