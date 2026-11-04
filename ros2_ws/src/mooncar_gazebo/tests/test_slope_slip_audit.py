"""Mechanical checks for read-only slope/slip diagnosis, not controller tests."""
import unittest
import numpy as np
from r5_slope_slip_audit import tangent,slip_residual,contact_multiplicity


class SlopeSlipAuditTest(unittest.TestCase):
    def test_contact_multiplicity_preserves_whole_wheel_damping(self):
        # Same whole-wheel force and slip, distributed over 1/2/4 contacts.
        count=np.array([1,2,4]);force=-.03/(.09*count)
        np.testing.assert_allclose(slip_residual(.03,force,.09,count),0,atol=1e-17)
        self.assertGreater(abs(slip_residual(.03,force[-1],.09,1)),.02)

    def test_tangent_ignores_normal_part_and_handles_slope(self):
        normal=np.array([-.03,0,1.]);normal/=np.linalg.norm(normal)
        direction=np.array([1,1,0.]);a=tangent(direction,normal)
        np.testing.assert_allclose(a,tangent(direction+5*normal,normal),atol=1e-15)
        self.assertAlmostEqual(float(a@normal),0,places=14)
        self.assertAlmostEqual(float(a@a),1,places=14)

    def test_action_reaction_and_axis_reversal_leave_law_invariant(self):
        self.assertAlmostEqual(float(slip_residual(.02,-.1,.1,2)),0)
        self.assertAlmostEqual(float(slip_residual(-.02,.1,.1,2)),0)

    def test_invalid_contact_evidence_is_rejected(self):
        for count in (0,-1,1.5,float('nan')):
            with self.assertRaises(ValueError):slip_residual(.1,-1,.1,count)
        for compliance in (-1,float('nan')):
            with self.assertRaises(ValueError):slip_residual(.1,-1,compliance,1)
        with self.assertRaises(ValueError):tangent([0,0,1],[0,0,1])
        with self.assertRaises(ValueError):tangent([1,0,0],[0,0,2])

    def test_material_seam_counts_each_pair_separately_and_ignores_pair_order(self):
        actual=contact_multiplicity([1,1,1,1,1,2],[10,20,10,10,30,10],[20,10,30,30,10,20])
        np.testing.assert_array_equal(actual,[2,2,3,3,3,1])

    def test_incomplete_pair_identity_is_rejected(self):
        for args in (([1],[10],[]),([1.5],[10],[20]),([1],[10],[10]),([],[],[])):
            with self.assertRaises(ValueError):contact_multiplicity(*args)


if __name__=='__main__':unittest.main()
