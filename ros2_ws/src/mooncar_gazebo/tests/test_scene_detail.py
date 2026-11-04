import sys
from pathlib import Path
import tempfile
import unittest

import numpy as np
from PIL import Image
from scipy.spatial import ConvexHull

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from terrain_scene_detail import excluded, rock_variants, sample_ground
from generate_terrain_materials import bake_scene


class SceneDetailTests(unittest.TestCase):
    def test_height_matches_triangle_diagonal_instead_of_bilinear(self):
        xs=ys=np.array([0.,1.]);z=np.array([[0.,0.],[0.,1.]])
        self.assertAlmostEqual(sample_ground(xs,ys,z,.75,.25),.25)
        self.assertAlmostEqual(sample_ground(xs,ys,z,.25,.75),.25)
        self.assertAlmostEqual(sample_ground(xs,ys,z,1.,1.),1.)

    def test_footprint_clearance_and_measured_exclusion(self):
        self.assertTrue(excluded('ramp',1.54,0,.05))
        self.assertTrue(excluded('ramp',2.55,.64,.05))
        self.assertFalse(excluded('ramp',3.,1.))
        self.assertTrue(excluded('survey',9.54,-5.1,.05))
        self.assertTrue(excluded('survey',4.8,6.54,.05))
        self.assertFalse(excluded('survey',-7.,0,.05))

    def test_rock_bank_is_reproducible_convex_and_varied(self):
        first,second=rock_variants(),rock_variants()
        for (vertices,faces),(again,again_faces) in zip(first,second):
            np.testing.assert_array_equal(vertices,again)
            np.testing.assert_array_equal(faces,again_faces)
            tri=vertices[faces]
            volume=np.einsum('ij,ij->i',tri[:,0],np.cross(tri[:,1],tri[:,2])).sum()/6
            self.assertAlmostEqual(volume,ConvexHull(vertices).volume,places=10)
        self.assertEqual(len({v.tobytes() for v,f in first}),6)

    def test_atlas_bakes_neutral_channels_and_repeatable_maps(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            labels=np.array([[0,0],[2,2]],dtype=np.uint8)
            bake_scene('ramp',labels,4,root,pixels_per_m=16)
            before={p.name:p.read_bytes() for p in root.glob('*.png')}
            bake_scene('ramp',labels,4,root,pixels_per_m=16)
            self.assertEqual(before,{p.name:p.read_bytes() for p in root.glob('*.png')})
            rgb=np.asarray(Image.open(root/'ramp_surface_albedo.png')).astype(float)
            self.assertEqual(rgb.shape,(64,64,3))
            self.assertLess(float(np.std(rgb[:,:,0]-rgb[:,:,1])),.8)


if __name__=='__main__': unittest.main()
