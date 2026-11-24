from copy import deepcopy
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
from audit_r5_drive_constraints import static_loads, anchored_radius, normalized_slip, propose, model_audit


class DriveConstraintAuditTests(unittest.TestCase):
    def test_mass_shift_loads_and_weight_conservation(self):
        for x in (-.006237889,.006237889):
            a=static_loads(9.186,x,.157,-.157,1.62)
            self.assertAlmostEqual(a['front']+a['rear'],9.186*1.62)
            self.assertEqual(a['front']>a['rear'],x>0)

    def test_invalid_force_and_geometry_fail(self):
        for x in (float('nan'), .157, -.157):
            with self.assertRaises(ValueError):static_loads(9.,x,.157,-.157,1.62)
        with self.assertRaises(ValueError):normalized_slip(.1,0,10)

    def test_constant_longitudinal_control_does_not_remove_anchor_radius(self):
        for vx in (-.2,-.01,0,.01,.2):
            self.assertGreaterEqual(anchored_radius(.157,vx,.2),.157)
        with self.assertRaises(ValueError):anchored_radius(.157,0,0)

    def test_speed_dependent_proposal_restores_static_compliance(self):
        self.assertEqual(normalized_slip(0,3.57,10),0)
        self.assertAlmostEqual(normalized_slip(.03,3.57,10),normalized_slip(-.03,3.57,10))

    def test_proposal_does_not_mutate_model_or_drop_existing_plugins(self):
        model=ET.fromstring('<model name="mooncar"><link name="base"/><plugin name="existing" filename="keep"><gain>20</gain></plugin></model>')
        before=ET.tostring(model)
        proposal,patch=propose(model,dict(front=7.,rear=8.),.076)
        self.assertEqual(before,ET.tostring(model))
        self.assertTrue(proposal['physics_change_requires_approval'])
        self.assertEqual(proposal['changed_vehicle_fields'],0)
        self.assertIn('WheelSlip',patch)
        self.assertIn('3.5',patch)
        self.assertIn('<gain>20</gain>',patch)
        self.assertEqual(model.find("plugin[@name='existing']/gain").text,'20')

    def test_existing_slip_plugin_not_silently_overwritten(self):
        model=ET.fromstring('<model><plugin name="gz::sim::systems::WheelSlip"/></model>')
        with self.assertRaises(ValueError):propose(model,dict(front=7.,rear=8.),.076)

    def test_actual_frozen_model_is_read_only(self):
        model=Path(__file__).resolve().parent/'fixtures/r5-d0.sdf'
        if not model.is_file():self.skipTest('Archived frozen R5 model needed')
        before=model.read_bytes();audit,_=model_audit(model)
        self.assertEqual(before,model.read_bytes())
        self.assertAlmostEqual(audit['combined']['total_mass_kg'],9.186)
        self.assertAlmostEqual(audit['axle_half_spacing_m'],.157)
        self.assertEqual(audit['constant_zero_vx_rear_anchored_half_turn_displacement_m'],.314)
        self.assertLess(audit['proposal_prediction']['nominal_force_ratio'],audit['proposal_prediction']['saturation_limit'])
        self.assertGreater(audit['proposal_prediction']['maximum_center_offset_for_half_turn_m'],.01)


if __name__=='__main__':unittest.main()
