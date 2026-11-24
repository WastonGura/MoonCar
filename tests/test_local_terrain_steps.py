"""Measured height discontinuities and elevation-invariant traversability."""
from pathlib import Path
import sys,unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2_ws/src/mooncar_terrain/scripts'))
from terrain_geometry import classify_cells,compute_traversability_cost

def patch(ix,iy,height,res=.2):
    x,y=np.meshgrid(np.linspace(.02,res-.02,6)+ix*res,np.linspace(.02,res-.02,6)+iy*res)
    z=height(x,y) if callable(height) else np.full(x.shape,height)
    return np.column_stack((x.ravel(),y.ravel(),z.ravel()))

class LocalHeightTests(unittest.TestCase):
    def classify(self,p):return classify_cells(p,resolution=.2)
    def test_long_elevated_and_depressed_inclines_do_not_become_steps(self):
        for sign in (1.,-1.):
            p=np.vstack([patch(i,j,lambda x,y:sign*.155*x) for i in range(30) for j in range(-1,2)])
            cells=self.classify(p)
            self.assertEqual(len(cells),90)
            self.assertTrue(all(c[4]==0 and compute_traversability_cost(*c[2:5])<100 for c in cells))
            self.assertGreater(max(abs(c[5]) for c in classify_cells(p,resolution=.2,return_elevation=True)),.8)

    def test_local_height_judgment_is_invariant_to_map_z_origin(self):
        base=np.vstack([patch(i,j,0.25 if i==1 and j==0 else 0.) for i in range(3) for j in range(-1,2)])
        reference=self.classify(base)
        for z in (-2.,.5,4.):
            changed=self.classify(base+[0,0,z])
            self.assertEqual([c[4] for c in reference],[c[4] for c in changed])
            np.testing.assert_allclose(np.asarray(reference)[:,:4],np.asarray(changed)[:,:4],atol=1e-10)

    def test_actual_step_top_and_visible_down_edge_stay_lethal(self):
        cells=self.classify(np.vstack([patch(i,0,.25 if i else 0.) for i in range(3)]))
        self.assertTrue(cells[1][4]&4)
        self.assertTrue(cells[0][4]&8)
        self.assertEqual(compute_traversability_cost(*cells[1][2:5]),100)
        self.assertEqual(compute_traversability_cost(*cells[0][2:5]),100)

    def test_stone_and_pit_borders_stay_lethal_at_any_baseline_height(self):
        for depth,bit in ((.25,4),(-.25,8)):
            p=np.vstack([patch(i,j,1.+(depth if i==0 and j==0 else 0.)) for i in range(-1,2) for j in range(-1,2)])
            center=next(c for c in self.classify(p) if 0<c[0]<.2 and 0<c[1]<.2)
            self.assertTrue(center[4]&bit)
            self.assertEqual(compute_traversability_cost(*center[2:5]),100)

    def test_smooth_curved_ground_has_no_artificial_height_edge(self):
        p=np.vstack([patch(i,j,lambda x,y:1.+.02*x*x+.01*y*y) for i in range(6) for j in range(3)])
        self.assertTrue(all(c[4]==0 for c in self.classify(p)))

    def test_a_step_inside_one_cell_still_fails_geometry_checks(self):
        p=patch(0,0,lambda x,y:np.where(x<.1,0.,.25))
        c=self.classify(p)[0]
        self.assertTrue(c[4]&3)
        self.assertEqual(compute_traversability_cost(*c[2:5]),100)

    def test_unknown_and_scanline_cells_do_not_supply_ground_references(self):
        flat=patch(0,0,.5)
        sparse=np.array([[.31,.11,0.],[.32,.11,0.]])
        cells=self.classify(np.vstack((flat,sparse)))
        self.assertEqual(len(cells),1)
        self.assertEqual(cells[0][4],0)
        self.assertEqual(self.classify(sparse),[])

    def test_steep_or_rough_surfaces_remain_blocked(self):
        for f in (lambda x,y:1.+.6*x,lambda x,y:.1*np.sin(x*180)):
            c=self.classify(patch(0,0,f))[0]
            self.assertTrue(c[4]&3)
            self.assertEqual(compute_traversability_cost(*c[2:5]),100)

if __name__=='__main__':unittest.main()
