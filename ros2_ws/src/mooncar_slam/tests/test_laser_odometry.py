import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from laser_odometry_core import LaserOdometry, rotation
from laser_odometry_node import interpolate


ANGLES=np.linspace(-math.pi,math.pi,720,endpoint=False)
def scan(xy,yaw,lever=(.14,0.)):
    """Analytic 8 x 6 m room; test truth only generates sensor fixtures."""
    origin=np.asarray(xy)+rotation(yaw)@lever
    directions=np.c_[np.cos(ANGLES+yaw),np.sin(ANGLES+yaw)]
    candidates=[]
    for axis,limits in [(0,(-4,4)),(1,(-3,3))]:
        for value in limits:
            with np.errstate(divide='ignore',invalid='ignore'):
                distance=(value-origin[axis])/directions[:,axis]
            candidates.append(np.where(distance>0,distance,np.inf))
    return np.min(candidates,axis=0)


def initialized():
    core=LaserOdometry()
    for i in range(10):core.update(i*.1,scan((0,0),0),ANGLES,.08,12.,0,0,0)
    return core


class LaserOdometryTests(unittest.TestCase):
    def test_sideways_motion_is_observed_with_zero_wheel_translation(self):
        core=initialized()
        r=core.update(1.,scan((0,.04),0),ANGLES,.08,12.,0,0,.5)
        self.assertTrue(r['accepted'],r)
        np.testing.assert_allclose(r['xy'],[0,.04],atol=.003)
        self.assertGreater(r['body_velocity'][1],0)

    def test_rotation_with_laser_lever_arm_does_not_invent_body_translation(self):
        core=initialized()
        for i in range(1,8):
            yaw=i*.1
            r=core.update(.9+i*.1,scan((0,0),yaw),ANGLES,.08,12.,yaw,0,.5)
            self.assertTrue(r['accepted'],r)
            self.assertLess(np.linalg.norm(r['xy']),.006)

    def test_degenerate_single_wall_is_rejected(self):
        core=initialized();target=np.c_[np.zeros(60),np.linspace(-1,1,60)]
        core.frames.clear();core.reference(target)
        result,reason=core.fit(target,np.array([.03,0.]))
        self.assertIsNone(result);self.assertEqual(reason,'unobservable_translation')

    def test_missing_data_never_becomes_a_valid_laser_measurement(self):
        core=initialized()
        r=core.update(1.,np.full(720,np.inf),ANGLES,.08,12.,0,.1,0.)
        self.assertFalse(r['accepted']);self.assertEqual(r['reason'],'too_few_returns')

    def test_stop_refinement_has_a_time_bound(self):
        core=initialized()
        core.update(1.,scan((0,0),0),ANGLES,.08,12.,0,0,.5)
        for t in (1.1,1.4,1.8,2.0):
            self.assertTrue(core.update(t,scan((0,0),0),ANGLES,.08,12.,0,0,0)['accepted'])
        self.assertEqual(core.update(2.1,scan((0,0),0),ANGLES,.08,12.,0,0,0)['reason'],'stationary_hold')

    def test_bad_scan_cannot_poison_next_motion_prediction(self):
        core=initialized()
        # Force observable correspondence rejection without replacing prediction.
        original=core.fit;core.fit=lambda *a:(None,'too_few_line_correspondences')
        r=core.update(1.,scan((.02,0),0),ANGLES,.08,12.,0,.2,0)
        self.assertFalse(r['accepted']);np.testing.assert_allclose(core.xy,[.02,0])
        core.fit=original
        r=core.update(1.1,scan((.04,0),0),ANGLES,.08,12.,0,.2,0)
        self.assertTrue(r['accepted']);np.testing.assert_allclose(r['xy'],[.04,0],atol=.003)

    def test_moving_initialization_fails_instead_of_silently_changing_origin(self):
        core=LaserOdometry()
        r=core.update(0.,scan((0,0),0),ANGLES,.08,12.,0,.1,0)
        self.assertFalse(r['accepted'])
        self.assertEqual(core.update(.1,scan((0,0),0),ANGLES,.08,12.,0,0,0)['reason'],'initialization_motion_requires_new_session')

    def test_covariance_is_finite_positive_and_not_zero(self):
        core=initialized();r=core.update(1.,scan((.02,.01),0),ANGLES,.08,12.,0,.2,.1)
        self.assertTrue(r['accepted']);self.assertGreaterEqual(np.linalg.eigvalsh(r['covariance']).min(),.02**2)

    def test_interpolation_requires_brackets_and_no_large_hidden_gap(self):
        rows=[(1.,2.),(1.02,4.)]
        np.testing.assert_allclose(interpolate(rows,1.01,.03),[3.])
        self.assertIsNone(interpolate(rows,1.1,.03))
        self.assertIsNone(interpolate([(1.,2.),(2.,4.)],1.001,.03))

    def test_duplicate_and_long_gap_scans_are_rejected(self):
        core=initialized()
        self.assertEqual(core.update(.9,scan((0,0),0),ANGLES,.08,12.,0,0,0)['reason'],'nonincreasing_scan_stamp')
        self.assertEqual(core.update(3.,scan((0,0),0),ANGLES,.08,12.,0,0,0)['reason'],'scan_gap_requires_reset')

    def test_stationary_maintenance_respects_estimator_timeout_without_growing_submap(self):
        core=initialized();frames=len(core.frames);accepted=[]
        for i in range(10,51):
            t=i*.1;r=core.update(t,scan((0,0),0),ANGLES,.08,12.,0,0,0)
            if r['accepted']:accepted.append(r['stamp'])
        self.assertEqual(len(core.frames),frames)
        self.assertGreaterEqual(len(accepted),3)
        self.assertTrue(all(.2-1e-9<=b-a<=.2+1e-9 for a,b in zip(accepted,accepted[1:])))


if __name__=='__main__':unittest.main()
