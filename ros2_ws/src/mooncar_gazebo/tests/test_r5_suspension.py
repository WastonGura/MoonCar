import copy
import subprocess
import unittest
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from r5_suspension import WHEELS,parameters,suspended
from prepare_r5_suspension import mass,compare
from r5_flat_scene import flat_world,validate_flat


class FlatSceneQualificationTests(unittest.TestCase):
    def fixture(self,folder,z):
        mesh=Path(folder)/'terrain.obj'
        mesh.write_text(f'v -4 -4 0\nv 4 -4 {z}\nv 4 4 0\nv -4 4 0\nf 1 2 3\nf 1 3 4\n')
        return ET.ElementTree(ET.fromstring(f'<sdf><world><model name="regolith_passable_ground"><static>true</static><link name="surface"><collision name="terrain"><geometry><mesh><uri>{mesh}</uri></mesh></geometry></collision></link></model></world></sdf>'))

    def test_reject_nonflat_straight_scene(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError,'non-planar'):
                validate_flat(self.fixture(folder,.04),2)

    def test_planar_route_requires_full_distance_coverage(self):
        with tempfile.TemporaryDirectory() as folder:
            world=self.fixture(folder,0)
            self.assertTrue(validate_flat(world,2)['passed'])
            with self.assertRaisesRegex(ValueError,'cover'):
                validate_flat(world,4)

    def test_flat_conversion_retains_collision_triangles(self):
        with tempfile.TemporaryDirectory() as folder:
            world=self.fixture(folder,.04)
            planar=flat_world(world,Path(folder)/'flat')
            self.assertTrue(validate_flat(planar,2)['passed'])
            path=planar.findtext('.//uri')
            self.assertEqual([s for s in Path(path).read_text().splitlines() if s.startswith('f ')],['f 1 2 3','f 1 3 4'])
            with self.assertRaises(ValueError):validate_flat(world,2)

    def test_bounding_vertices_do_not_hide_missing_triangles(self):
        with tempfile.TemporaryDirectory() as folder:
            world=self.fixture(folder,0);mesh=Path(world.findtext('.//uri'))
            mesh.write_text(mesh.read_text().replace('f 1 3 4\n',''))
            with self.assertRaisesRegex(ValueError,'hole'):validate_flat(world,2)

    def test_overlapping_triangles_cannot_fill_a_hole_by_area_sum(self):
        with tempfile.TemporaryDirectory() as folder:
            world=self.fixture(folder,0);mesh=Path(world.findtext('.//uri'))
            mesh.write_text(mesh.read_text().replace('f 1 3 4','f 1 2 3'))
            with self.assertRaisesRegex(ValueError,'hole'):validate_flat(world,2)

    def test_obstacle_outside_origin_but_envelope_intersects_route(self):
        with tempfile.TemporaryDirectory() as folder:
            world=self.fixture(folder,0)
            world.find('world').append(ET.fromstring('<model name="obstacle"><static>true</static><pose>3 0 0 0 0 0</pose><link name="box"><collision name="box"><geometry><box><size>2 1 1</size></box></geometry></collision></link></model>'))
            with self.assertRaisesRegex(ValueError,'envelope'):validate_flat(world,2)

@unittest.skipUnless(Path('/oldclosure/prepared-d0/curve-0.001/world.sdf').is_file(),
                     'Requires isolated archived-R5 integration fixture')
class SuspensionTests(unittest.TestCase):
    def setUp(self):
        self.model=ET.parse('/oldclosure/prepared-d0/curve-0.001/world.sdf').find("world/model[@name='mooncar']")
        self.candidate,self.proof=suspended(self.model,'OBSERVER','OUTPUT')

    def test_neutral_mass_com_and_inertia(self):
        compare(mass(self.model),mass(self.candidate))
        for w in WHEELS:
            self.assertAlmostEqual(float(self.candidate.findtext(f"link[@name='{w}_wheel_link']/inertial/mass")),.35)
            self.assertAlmostEqual(float(self.candidate.findtext(f"link[@name='{w}_wheel_carrier_link']/inertial/mass")),.05)

    def test_drive_and_collision_preserved(self):
        for w in WHEELS:
            path=f"link[@name='{w}_wheel_link']/collision"
            self.assertEqual(ET.tostring(self.model.find(path)),ET.tostring(self.candidate.find(path)))
            path=f"joint[@name='{w}_wheel_joint']/axis"
            self.assertEqual(ET.tostring(self.model.find(path)),ET.tostring(self.candidate.find(path)))
        for plugin,other in zip(self.model.findall('plugin'),self.candidate.findall('plugin')):
            if plugin.get('name')=='gz::sim::systems::JointStatePublisher':
                other=copy.deepcopy(other)
                added=[n for n in other.findall('joint_name') if n.text.endswith('_suspension_joint')]
                self.assertEqual(len(added),4)
                for n in added:other.remove(n)
            self.assertEqual(ET.tostring(plugin),ET.tostring(other))

    def test_static_load_and_moment(self):
        proof=self.proof;p=proof['parameters'];total=sum(x['force_N'] for x in p.values())
        self.assertAlmostEqual(total,proof['sprung_mass_kg']*1.62)
        moment=sum(x['force_N']*(.157 if w.startswith('front') else -.157) for w,x in p.items())
        self.assertAlmostEqual(moment,proof['sprung_first_moment_kg_m']*1.62)
        for x in p.values():self.assertAlmostEqual(x['k_N_m']*.01,x['force_N'])

    def test_passive_topology_and_travel(self):
        for w in WHEELS:
            j=self.candidate.find(f"joint[@name='{w}_wheel_suspension_joint']")
            self.assertEqual(j.get('type'),'prismatic');self.assertEqual(j.findtext('axis/xyz'),'0 0 1')
            self.assertEqual(float(j.findtext('axis/limit/lower')),-.02)
            self.assertEqual(float(j.findtext('axis/limit/upper')),.02)
        self.assertFalse(self.proof['hardware_calibrated'])

    def test_default_models_byte_identical(self):
        for name,before in [('rigid','r5'),('legacy','legacy')]:
            self.assertEqual(Path('/diagnosis/after/'+name+'.urdf').read_bytes(),Path('/diagnosis/before/'+before+'.urdf').read_bytes())

    def test_unknown_profile_rejected(self):
        result=subprocess.run(['xacro','/source/ros2_ws/src/mooncar_description/urdf/mooncar_r5.urdf.xacro','suspension_profile:=typo'],capture_output=True)
        self.assertNotEqual(result.returncode,0)

if __name__=='__main__':unittest.main()
