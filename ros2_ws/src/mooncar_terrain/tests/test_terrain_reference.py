"""Independent known surfaces, explicit unsupported geometry, and fixed missing counts."""
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from terrain_reference import read_shapes, surface, reference_grid, statistics
from simulation_experiment import transform


class ReferenceTests(unittest.TestCase):
    def test_geometry_and_denominators(self):
        identity = np.eye(4)
        ground = ('ground','plane',np.array([50.,50.]),identity)
        tilt = transform(ET.fromstring('<model><pose>0 0 1 0 0.2 0</pose></model>'))
        box = ('ramp','box',np.array([2.,2.,.1]),tilt)
        xy = np.array([[0.,0.],[.2,.1],[5.,0.]])
        heights, owners = surface([ground,box],xy)
        np.testing.assert_allclose(heights[:2],1+.1/math.cos(.2)-xy[:2,0]*math.tan(.2))
        self.assertEqual(heights[2],0); self.assertEqual(list(owners),[1,1,0])
        cells = reference_grid([ground,box],[0.,0.],.5,.25)
        self.assertTrue(all(c['category']=='terrain' for c in cells))
        np.testing.assert_allclose([c['slope'] for c in cells],math.degrees(.2),atol=1e-9)
        sphere = ('sphere','ellipsoid',np.array([1.,1.,1.]),tilt)
        z,_ = surface([sphere],np.array([[0.,0.],[.6,0.]]))
        np.testing.assert_allclose(z,[2.,1.8],atol=1e-9)
        # A tilted non-spherical ellipsoid: solve independently with world quadratic form.
        ellipsoid = ('ellipsoid','ellipsoid',np.array([2.,1.,.5]),tilt)
        z,_ = surface([ellipsoid],np.array([[0.,0.]]))
        matrix = tilt[:3,:3] @ np.diag([.25,1.,4.]) @ tilt[:3,:3].T
        self.assertAlmostEqual(z[0],1+1/math.sqrt(matrix[2,2]))
        cylinder_pose = identity.copy(); cylinder_pose[2,3]=.012
        cylinder=('floor','cylinder',np.array([1.45,.009]),cylinder_pose)
        z,_=surface([ground,cylinder],np.array([[0.,0.],[2.,0.]]))
        np.testing.assert_allclose(z,[.021,0])
        edge=reference_grid([ground,('small','box',np.array([.12,.12,.5]),identity)],[0.,0.],.5,.25)
        self.assertTrue(any(c['category']=='edge_or_complex' for c in edge))
        rows=[dict(observed=False,valid_observed=False,height_error_m=None,slope_error_deg=None),
              dict(observed=True,valid_observed=False,height_error_m=None,slope_error_deg=None),
              dict(observed=True,valid_observed=True,height_error_m=.1,slope_error_deg=-2)]
        result=statistics(rows)
        self.assertEqual([result[k] for k in ('reference_cells','missing_cells','invalid_observed_cells','matched_cells')],[3,1,1,1])
        self.assertAlmostEqual(result['height_rmse_m'],.1); self.assertEqual(result['slope_mae_deg'],2)
        self.assertIsNone(statistics(rows[:2])['height_rmse_m'])
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'world.sdf'
            path.write_text('<sdf><world><model name="x"><static>true</static><link><visual><geometry><mesh/></geometry></visual></link></model></world></sdf>')
            with self.assertRaisesRegex(ValueError,'Unsupported'): read_shapes(path)
            # Visual-only cylinder must be retained even without a collision entry.
            path.write_text('<sdf><world><model name="x"><static>true</static><link><visual><geometry><cylinder><radius>1</radius><length>0.1</length></cylinder></geometry></visual></link></model></world></sdf>')
            self.assertEqual(len(read_shapes(path)),1)


if __name__=='__main__': unittest.main()
