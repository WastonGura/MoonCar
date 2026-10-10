from pathlib import Path
import json
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
from r5_slip_probe import generate, slip_commands, compare
from r5_load_diagnostic import semantic
from audit_r5_drive_constraints import WHEELS


class SlipProbeTests(unittest.TestCase):
    def test_exact_single_intervention_and_restore(self):
        source=ROOT/'reports/evidence/terrain-v2/r5-slip-probe-20261010/r5-d0.sdf'
        before=source.read_bytes()
        with tempfile.TemporaryDirectory() as tmp:
            auth=Path(tmp)/'auth.json';auth.write_text(json.dumps(dict(human_reply='允许测试侧向滑移模型，仅限测试副本',production_change_allowed=False)))
            output=Path(tmp)/'models';info=generate(source,output,auth)
            self.assertEqual((output/'baseline.sdf').read_bytes(),before)
            self.assertEqual((output/'restore.sdf').read_bytes(),before)
            candidate=ET.parse(output/'candidate.sdf')
            plugin=candidate.find("model/plugin[@name='gz::sim::systems::WheelSlip']")
            self.assertEqual(len(plugin.findall('wheel')),4)
            for wheel in plugin.findall('wheel'):
                self.assertEqual(float(wheel.findtext('slip_compliance_lateral')),10)
                self.assertEqual(float(wheel.findtext('slip_compliance_longitudinal')),0)
            candidate.find('model').remove(plugin)
            self.assertEqual(semantic(candidate.getroot()),semantic(ET.parse(source).getroot()))
            self.assertFalse(info['production_changed'])
        self.assertEqual(source.read_bytes(),before)

    def test_authorization_required(self):
        with tempfile.TemporaryDirectory() as tmp:
            auth=Path(tmp)/'auth.json';auth.write_text('{}')
            with self.assertRaises(ValueError):generate('not-read',Path(tmp)/'model',auth)

    def fixture(self):
        dtype=[('t',float)]+[(w+f,float) for w in WHEELS for f in ('_slip_present','_slip1_cmd','_slip2_cmd','_spin_before')]
        data=np.zeros(3,dtype=dtype);data['t']=[1,2,3]
        params={w:dict(wheel_radius=.076,wheel_normal_force=4.,slip_compliance_lateral=10.) for w in WHEELS}
        for w in WHEELS:
            data[w+'_slip_present']=1;data[w+'_spin_before']=[0,-1,1];data[w+'_slip1_cmd']=[0,.19,.19]
        return data,dict(sim_started=1,sim_ended=3),params

    def test_actual_speed_formula_and_zero_speed(self):
        d,r,p=self.fixture();self.assertTrue(slip_commands(d,r,'candidate',p)['passed'])

    def test_missing_component_and_nonfinite_do_not_pass(self):
        for field,value in (('_slip_present',0),('_slip1_cmd',float('nan')),('_spin_before',float('nan'))):
            d,r,p=self.fixture();d['front_left'+field][1]=value
            self.assertFalse(slip_commands(d,r,'candidate',p)['passed'])

    def test_wrong_formula_or_longitudinal_change_fails(self):
        for field in ('_slip1_cmd','_slip2_cmd'):
            d,r,p=self.fixture();d['rear_right'+field][1]=.123
            self.assertFalse(slip_commands(d,r,'candidate',p)['passed'])

    def test_baseline_absence_is_explicit_not_fabricated_zero(self):
        d,r,p=self.fixture()
        for w in WHEELS:
            d[w+'_slip_present']=0;d[w+'_slip1_cmd']=float('nan');d[w+'_slip2_cmd']=float('nan')
        self.assertTrue(slip_commands(d,r,'baseline',p)['passed'])
        d['front_left_slip_present'][1]=1
        self.assertFalse(slip_commands(d,r,'restore',p)['passed'])

    def test_reduced_angle_and_missing_backend_cannot_fake_success(self):
        old={'slip_probe':{'trajectory':dict(maximum_displacement_m=.31,unwrapped_turn_rad=3.45)}}
        new={'slip_probe':{'trajectory':dict(maximum_displacement_m=.0125,unwrapped_turn_rad=3.5),
                           'contact':{'backend_effect_verified':True}}}
        result=compare(old,new)
        self.assertTrue(result['mechanism_probe_passed']);self.assertFalse(result['previous_one_cm_screen_passed'])
        self.assertFalse(result['whole_vehicle_accepted'])
        new['slip_probe']['trajectory']['unwrapped_turn_rad']=2
        self.assertFalse(compare(old,new)['mechanism_probe_passed'])
        new['slip_probe']['trajectory']['unwrapped_turn_rad']=3.5
        new['slip_probe']['contact']['backend_effect_verified']=False
        self.assertFalse(compare(old,new)['mechanism_probe_passed'])


if __name__=='__main__':unittest.main()
