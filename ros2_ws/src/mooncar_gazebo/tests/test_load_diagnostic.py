"""Mass intervention, point association and frozen causal decision regressions."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'scripts'))
from r5_load_diagnostic import (causal_decision, generate_models, mass_properties,
                                points_for_wheel, sha, stamp_ns, verify_single_change)
from run_r5_load_campaign import CASES

MODEL = '''<sdf version="1.9"><model name="test">
<link name="base_footprint"><inertial><mass>2</mass><pose>-0.1 0 0 0 0 0</pose><inertia>
<ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial></link>
<joint name="wheel_joint" type="revolute"><pose relative_to="base_footprint">0.2 0 0 0 0 0</pose><parent>base_footprint</parent><child>wheel</child></joint>
<link name="wheel"><pose relative_to="wheel_joint">0 0 0 0 0 0</pose><inertial><mass>1</mass><pose>0 0 0 0 0 0</pose><inertia>
<ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial></link>
</model></sdf>'''


def tree():
    t=ET.ElementTree(ET.fromstring(MODEL))
    t.find("./model/link[@name='base_footprint']/inertial/pose").text='-0.12 0 0 0 0 0'
    return t


def contact():
    return dict(contact=[dict(collision1=dict(name='mooncar::front_left_wheel_link::collision'), collision2=dict(name='ground'),
        position=[{}], normal=[dict(z=1)], depth=[0.], wrench=[dict(body1Wrench=dict(force=dict(z=2)),body2Wrench=dict(force=dict(z=-2)))])])


class LoadDiagnosticTests(unittest.TestCase):
    def test_whole_com_resolves_joint_frames(self):
        value=mass_properties(tree())
        self.assertAlmostEqual(value['total_mass_kg'],3.)
        self.assertAlmostEqual(value['center_of_mass_m'][0],(-.24+.2)/3)

    def test_generation_mirrors_whole_com_not_base_alone(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'source.sdf';tree().write(p)
            m=generate_models(p,Path(d)/'models')
            self.assertAlmostEqual(m['mirror_x']['center_of_mass_m'][0],-m['original']['center_of_mass_m'][0])
            self.assertAlmostEqual(m['base_inertial_x_delta_m'],.04)
            self.assertEqual(m['models']['original'],m['models']['restore'])
            self.assertEqual(m['models']['original'],sha(p))

    def test_combined_inertia_change_is_recorded(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'source.sdf';tree().write(p)
            m=generate_models(p,Path(d)/'models')
            self.assertNotEqual(m['original']['inertia_about_total_com_kg_m2'],m['mirror_x']['inertia_about_total_com_kg_m2'])

    def test_companion_urdf_preserves_kinematic_source(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'source.sdf';tree().write(p)
            p.with_suffix('.urdf').write_text('<robot name="unchanged"/>')
            m=generate_models(p,Path(d)/'models')
            self.assertEqual(set(m['tf_urdf']),{'original','mirror_x','restore'})
            self.assertEqual(len(set(m['tf_urdf'].values())),1)
            self.assertEqual(m['tf_urdf']['mirror_x'],sha(p.with_suffix('.urdf')))

    def test_unrelated_model_change_rejected(self):
        a=tree();b=copy.deepcopy(a);b.find('./model/link/inertial/pose').text='0.01 0 0 0 0 0'
        b.find('./model/link/inertial/mass').text='3'
        with self.assertRaises(ValueError):verify_single_change(a,b)

    def test_inertial_rotation_change_rejected(self):
        a=tree();b=copy.deepcopy(a);b.find('./model/link/inertial/pose').text='0.01 0 0 0 0 0.1'
        with self.assertRaises(ValueError):verify_single_change(a,b)

    def test_nonphysical_inertia_rejected(self):
        a=tree();a.find('./model/link/inertial/inertia/izz').text='4'
        with self.assertRaises(ValueError):mass_properties(a)

    def test_frame_cycle_rejected(self):
        a=tree();a.find('./model/joint/pose').set('relative_to','wheel')
        with self.assertRaises(ValueError):mass_properties(a)

    def test_missing_force_rejected(self):
        c=contact();del c['contact'][0]['wrench'][0]['body1Wrench']['force']
        with self.assertRaises(ValueError):points_for_wheel(c,'front_left')

    def test_missing_depth_rejected(self):
        c=contact();c['contact'][0]['depth']=[]
        with self.assertRaises(ValueError):points_for_wheel(c,'front_left')

    def test_absent_contact_rejected(self):
        with self.assertRaises(ValueError):points_for_wheel({'contact':[]},'front_left')

    def test_body_two_ownership_selects_opposite_force(self):
        c=contact();co=c['contact'][0];co['collision1'],co['collision2']=co['collision2'],co['collision1']
        self.assertEqual(points_for_wheel(c,'front_left')[0][3][2],-2)

    def test_unknown_collision_rejected(self):
        c=contact();c['contact'][0]['collision1']['name']='other'
        with self.assertRaises(ValueError):points_for_wheel(c,'front_left')

    def test_explicit_zero_force_is_valid(self):
        c=contact();w=c['contact'][0]['wrench'][0];w['body1Wrench']['force']={};w['body2Wrench']['force']={}
        np.testing.assert_array_equal(points_for_wheel(c,'front_left')[0][3],[0,0,0])

    def test_force_direction_mismatch_rejected(self):
        c=contact();c['contact'][0]['wrench'][0]['body2Wrench']['force']['z']=2
        with self.assertRaises(ValueError):points_for_wheel(c,'front_left')

    def test_timestamp_uses_integer_nanoseconds(self):
        self.assertEqual(stamp_ns(dict(header=dict(stamp=dict(sec='8',nsec=243000000)))),8243000000)

    def test_fixed_twelve_attempt_matrix(self):
        self.assertEqual(len(CASES),12)
        self.assertEqual([x[0] for x in CASES].count('baseline'),5)
        self.assertEqual([x[0] for x in CASES].count('mirror'),5)
        self.assertEqual([x[0] for x in CASES].count('restore'),2)

    def test_complete_exchange_does_not_claim_repair(self):
        entries=[dict(passed=True,angular=c[3],audit=dict(static_load_sign_matches_prediction=True,support_exchange_prediction=True)) for c in CASES]
        v=causal_decision(entries);self.assertTrue(v['cause_confirmed']);self.assertFalse(v['repair_verified'])
        entries[6]['audit']['support_exchange_prediction']=False
        self.assertFalse(causal_decision(entries)['cause_confirmed'])

    def test_failure_or_missing_restore_cannot_confirm(self):
        entries=[dict(passed=True,angular=c[3],audit=dict(static_load_sign_matches_prediction=True,support_exchange_prediction=True)) for c in CASES]
        self.assertFalse(causal_decision(entries[:-1])['cause_confirmed'])
        entries[-1]['passed']=False
        self.assertFalse(causal_decision(entries)['cause_confirmed'])


if __name__=='__main__':unittest.main()
