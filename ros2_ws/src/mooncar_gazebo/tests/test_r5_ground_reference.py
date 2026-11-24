from argparse import Namespace
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
from r5_ground_reference import FLAT_PROFILE,prepare_reference

class GroundReferenceTests(unittest.TestCase):
    def world(self,folder,nonflat=False,hole=False):
        folder=Path(folder);mesh=folder/'ground.obj'
        mesh.write_text('v -4 -4 0\nv 4 -4 '+('.04' if nonflat else '0')+'\nv 4 4 0\nv -4 4 0\nf 1 2 3\n'+('' if hole else 'f 1 3 4\n'))
        tree=ET.ElementTree(ET.fromstring(f'<sdf><world><model name="regolith_ground"><static>true</static><link name="surface"><collision name="mesh"><geometry><mesh><uri>{mesh}</uri></mesh></geometry></collision></link></model></world></sdf>'))
        path=folder/'world.sdf';tree.write(path);return path

    def test_loaded_flat_mesh_overrides_legacy_ramp_label_without_relaxing_limits(self):
        with tempfile.TemporaryDirectory() as folder:
            height,proof=prepare_reference(Namespace(scene='ramp',linear=.1,ground_reference_profile=FLAT_PROFILE),self.world(folder))
            self.assertTrue(proof['passed']);self.assertEqual(height(1.99,0),0.)
            self.assertTrue(-.01<=-.000256-height(1.99,0)<=.03)
            self.assertFalse(-.01<=-.02-height(1.99,0)<=.03)  # True penetration still fails.
            self.assertFalse(-.01<=.04-height(1.99,0)<=.03)  # True separation still fails.

    def test_mislabeled_nonflat_world_and_hole_rejected_before_run(self):
        for kind in ('nonflat','hole'):
            with self.subTest(kind=kind),tempfile.TemporaryDirectory() as folder:
                with self.assertRaises(ValueError):
                    prepare_reference(Namespace(linear=.1,ground_reference_profile=FLAT_PROFILE),self.world(folder,**{kind:True}))

    def test_queries_require_finite_points_inside_verified_route(self):
        with tempfile.TemporaryDirectory() as folder:
            height,_=prepare_reference(Namespace(linear=.1,ground_reference_profile=FLAT_PROFILE),self.world(folder))
            for x,y in [(2.6,0),(0,.6),(float('nan'),0)]:
                with self.assertRaises(ValueError):height(x,y)

    def test_legacy_path_remains_explicit_and_unknown_profile_rejected(self):
        self.assertEqual(prepare_reference(Namespace(linear=.1),'unused'),(None,None))
        with self.assertRaises(ValueError):prepare_reference(Namespace(linear=.1,ground_reference_profile='unknown'),'unused')

if __name__=='__main__':unittest.main()
