"""Geometry, source constraints and scene selection regressions for Terrain V3."""
import json
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(ROOT/'ros2_ws/src/mooncar_gazebo/scripts'))
sys.path.insert(0,str(ROOT/'ros2_ws/src/mooncar_gazebo/launch'))
from build_terrain_v3 import laplacian,surface,background
from check_terrain_v3 import check
from simulation_experiment import survey_world,validate_task
from experiment_scenes import scene_metadata
from physics_guard import resolve_control_profile

PACKAGE=ROOT/'ros2_ws/src/mooncar_gazebo'


class TerrainV3Tests(unittest.TestCase):
    def test_actual_mesh_is_complete_and_known_fake_cliff_is_gone(self):
        result=check(PACKAGE)
        self.assertTrue(result['passed'])
        self.assertAlmostEqual(result['projected_area_m2'],900,places=8)
        self.assertEqual(result['triangles'],180000)
        self.assertLess(result['former_49deg_seam_now_deg'],18)
        # A terrain repair must not flatten every genuine crater face.
        self.assertGreater(result['whole_surface_max_slope_deg'],25)

    def test_global_solver_preserves_measured_values_and_pad(self):
        axis,z,cores,report=surface(.3)
        x,y=np.meshgrid(axis,axis)
        base=background(x,y,.3)
        for code,filename,cx,cy in ((1,'ce4_fresh_crater_5cm.npz',4.8,3.6),
                                   (2,'ce4_background_5cm.npz',6.,-5.1)):
            selected=cores==code
            with np.load(PACKAGE/'data/terrain_sources'/filename) as source:
                c=np.rint((x[selected]-cx-source['x'][0])/.05).astype(int)
                r=np.rint((y[selected]-cy-source['y'][0])/.05).astype(int)
                expected=source['z'][r,c]+(base[selected] if code==1 else 0)
            np.testing.assert_allclose(z[selected],expected,atol=5.01e-7,rtol=0)
        self.assertLess(report['relative_linear_residual'],1e-8)
        self.assertTrue(np.isfinite(z).all())
        self.assertEqual(float(z[len(axis)//2,len(axis)//2]),0)
        n=8;xx,yy=np.meshgrid(np.arange(n),np.arange(n))
        result=(laplacian(n)@(xx+2*yy).ravel()).reshape(n,n)
        np.testing.assert_allclose(result[1:-1,1:-1],0,atol=1e-12)

    def test_v3_world_metadata_boundaries_and_r5_profile(self):
        tree=survey_world('survey_v3_native')
        self.assertEqual(tree.find('world').get('name'),'mooncar_survey_v3')
        self.assertFalse(any(m.get('name').startswith('wb_localization_marker') for m in tree.findall('./world/model')))
        metadata=scene_metadata('survey_v3_native','rgbd_window')
        self.assertEqual(metadata['bounds'],[-15,-15,15,15])
        self.assertEqual(metadata['localization_fixture_profile'],'none')
        self.assertEqual(resolve_control_profile(PACKAGE/'worlds/mooncar_survey_v3.sdf','mooncar_r5.urdf.xacro'),'r5_dart_v2')
        task=dict(version=1,scene='survey_v3_native',start=[.5,0,0],region=[-15,-15,15,15])
        self.assertEqual(validate_task(task),task)
        with self.assertRaises(ValueError):validate_task(dict(task,region=[-25,-25,25,25]))
        manifest=json.loads((PACKAGE/'models/terrain_v3/survey/manifest.json').read_text())
        for rock in manifest['boulders']:
            with self.assertRaises(ValueError):validate_task(dict(task,start=rock['position_m'][:2]+[0]))


if __name__=='__main__':unittest.main()
