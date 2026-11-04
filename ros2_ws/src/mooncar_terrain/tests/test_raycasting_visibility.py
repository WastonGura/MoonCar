"""Unit tests for 3D Line-of-Sight (LOS) Raycasting FOV model in autonomous survey."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from autonomous_survey import choose_view,known_chassis_clearance


class RaycastingVisibilityTests(unittest.TestCase):
    def setUp(self):
        self.resolution = 0.25
        self.width = 80
        self.origin = [-10.0, -10.0]
        self.grid_data = [0] * (self.width * self.width)
        self.grid = NS(info=NS(width=self.width, height=self.width,
                              resolution=self.resolution,
                              origin=NS(position=NS(x=self.origin[0], y=self.origin[1]))),
                       data=self.grid_data)

    def _setup_target_cells(self, elevation_fn=None):
        """Setup cells where only the +X corridor is unobserved, focusing planner on yaw=0."""
        qr = self.resolution
        cells = []
        for x in range(-32, 32):
            for y in range(-32, 32):
                wx, wy = (x + 0.5) * qr, (y + 0.5) * qr
                # Leave +X corridor unobserved so utility > 0 only along yaw=0
                if 1.0 <= wx <= 5.5 and abs(wy) <= 1.0:
                    continue
                z = elevation_fn(wx, wy) if elevation_fn else 0.0
                cells.append(dict(ix=x, iy=y, quality=0.8, z=z))
        return cells

    def test_flat_ground_matches_planar_visibility(self):
        """On flat ground with uniform z=0, 3D LOS visible cells should match planar visibility."""
        cells = self._setup_target_cells()
        quality = dict(resolution=self.resolution, cells=cells)
        pose = dict(x=0.0, y=0.0, yaw=0.0)

        # Use budget remaining=4.0 so both evaluate in-place turn at (0, 0)
        choice_v2 = choose_view(self.grid, quality, pose, [0, 0], 8.0, [], 6.0, 'coverage_view_v2')
        choice_v3 = choose_view(self.grid, quality, pose, [0, 0], 8.0, [], 6.0, 'coverage_view_v3')

        self.assertIsNotNone(choice_v2)
        self.assertIsNotNone(choice_v3)
        self.assertEqual((choice_v3['x'], choice_v3['y']), (0.0, 0.0))
        self.assertEqual(choice_v3['yaw'], 0.0)

        v2_cells = set(tuple(c) for c in choice_v2['visible_cells'])
        v3_cells = set(tuple(c) for c in choice_v3['visible_cells'])
        overlap = len(v2_cells & v3_cells) / max(len(v2_cells), len(v3_cells))
        self.assertGreater(overlap, 0.90, "Flat ground 3D LOS should be highly congruent with planar visibility")

    def test_hill_crest_occludes_backside_cells(self):
        """A hill crest at x=2.0 should occlude flat ground behind it at x>=2.75 in 3D LOS."""
        cells = self._setup_target_cells()
        qr = self.resolution
        # Add observed hill at x=2.0 (height 0.60m)
        for x in range(-32, 32):
            for y in range(-32, 32):
                wx, wy = (x + 0.5) * qr, (y + 0.5) * qr
                if 1.75 <= wx <= 2.25 and abs(wy) <= 0.75:
                    cells.append(dict(ix=x, iy=y, quality=0.8, z=0.60))

        quality = dict(resolution=qr, cells=cells)
        pose = dict(x=0.0, y=0.0, yaw=0.0)

        choice_v2 = choose_view(self.grid, quality, pose, [0, 0], 8.0, [], 6.0, 'coverage_view_v2')
        choice_v3 = choose_view(self.grid, quality, pose, [0, 0], 8.0, [], 6.0, 'coverage_view_v3')

        v2_cells = set(tuple(c) for c in choice_v2['visible_cells'])
        v3_cells = set(tuple(c) for c in choice_v3['visible_cells'])

        # Cells directly behind the hill along optical axis (x=3.0 to 4.5, y=0.0)
        backside_keys = {(math.floor(bx / qr), math.floor(0.0 / qr))
                         for bx in np.arange(3.0, 4.51, 0.25)}

        v2_saw_backside = len(v2_cells & backside_keys)
        v3_saw_backside = len(v3_cells & backside_keys)

        self.assertGreater(v2_saw_backside, 0, "Planar v2 should suffer from false visibility penetration")
        self.assertEqual(v3_saw_backside, 0, "3D LOS v3 must occlude cells behind the crest")

    def test_higher_peak_peeks_over_lower_ridge(self):
        """A distant tall mountain should be visible over an intervening lower ridge."""
        cells = self._setup_target_cells()
        qr = self.resolution
        # Lower ridge at x=2.0 (z=0.45m), taller mountain at x=4.25 (z=1.10m)
        for x in range(-32, 32):
            for y in range(-32, 32):
                wx, wy = (x + 0.5) * qr, (y + 0.5) * qr
                if 1.75 <= wx <= 2.25 and abs(wy) <= 0.75:
                    cells.append(dict(ix=x, iy=y, quality=0.8, z=0.45))
                elif 4.0 <= wx <= 4.5 and abs(wy) <= 0.75:
                    cells.append(dict(ix=x, iy=y, quality=0.8, z=1.10))

        quality = dict(resolution=qr, cells=cells)
        pose = dict(x=0.0, y=0.0, yaw=0.0)

        choice_v3 = choose_view(self.grid, quality, pose, [0, 0], 8.0, [], 6.0, 'coverage_view_v3')
        v3_cells = set(tuple(c) for c in choice_v3['visible_cells'])

        ridge_front_key = (math.floor(1.9 / qr), 0)
        valley_key = (math.floor(3.0 / qr), 0)
        peak_key = (math.floor(4.1 / qr), 0)

        # Ridge front must be visible
        self.assertIn(ridge_front_key, v3_cells, "Ridge front must be visible")
        # Valley behind ridge must be occluded
        self.assertNotIn(valley_key, v3_cells, "Valley behind ridge should be occluded")
        # Mountain peak must be visible over ridge
        self.assertIn(peak_key, v3_cells, "Taller peak must peek over lower ridge")

    def test_convex_brow_occludes_steep_descent(self):
        """Robot on elevated plateau cannot see bottom of steep drop right behind the brow."""
        qr = self.resolution
        cells = []
        for x in range(-32, 32):
            for y in range(-32, 32):
                wx, wy = (x + 0.5) * qr, (y + 0.5) * qr
                if 1.0 <= wx <= 5.5 and abs(wy) <= 1.0:
                    continue
                z = 1.50 if wx < 1.0 else 0.0
                cells.append(dict(ix=x, iy=y, quality=0.8, z=z))

        # Brow at x=1.0 (z=1.45m), steep drop at x>=2.0 (z=0.0m)
        cells.append(dict(ix=math.floor(1.0 / qr), iy=0, quality=0.8, z=1.45))
        for x in range(math.floor(2.0 / qr), math.floor(3.5 / qr) + 1):
            cells.append(dict(ix=x, iy=0, quality=0.8, z=0.0))

        quality = dict(resolution=qr, cells=cells)
        pose = dict(x=0.0, y=0.0, yaw=0.0, z=1.50)

        choice_v3 = choose_view(self.grid, quality, pose, [0, 0], 8.0, [], 6.0, 'coverage_view_v3')
        v3_cells = set(tuple(c) for c in choice_v3['visible_cells'])

        brow_key = (math.floor(1.0 / qr), 0)
        drop_key = (math.floor(2.25 / qr), 0)

        self.assertIn(brow_key, v3_cells, "Plateau brow must be visible")
        self.assertNotIn(drop_key, v3_cells, "Steep descent base must be occluded by brow")

    def test_steep_obstacle_above_vfov_terminates_ray(self):
        """A tall vertical barrier exceeding camera upper FOV terminates the ray."""
        cells = self._setup_target_cells()
        qr = self.resolution
        # Wall at x=1.25 to 1.75, height 2.5m
        for x in range(-32, 32):
            for y in range(-32, 32):
                wx, wy = (x + 0.5) * qr, (y + 0.5) * qr
                if 1.25 <= wx <= 1.75 and abs(wy) <= 0.75:
                    cells.append(dict(ix=x, iy=y, quality=0.8, z=2.50))

        quality = dict(resolution=qr, cells=cells)
        pose = dict(x=0.0, y=0.0, yaw=0.0)

        choice_v3 = choose_view(self.grid, quality, pose, [0, 0], 8.0, [], 6.0, 'coverage_view_v3')
        v3_cells = set(tuple(c) for c in choice_v3['visible_cells'])

        wall_front_key = (math.floor(1.25 / qr), 0)
        beyond_key = (math.floor(3.5 / qr), 0)

        self.assertIn(wall_front_key, v3_cells, "Wall front must be observed")
        self.assertNotIn(beyond_key, v3_cells, "Ray must terminate at tall wall")

    def test_graceful_fallback_without_elevations(self):
        """When quality['cells'] lacks z field, choose_view runs smoothly without exception."""
        cells = [dict(ix=x, iy=y, quality=0.8)
                 for x in range(-20, 20) for y in range(-20, 20)]
        quality = dict(resolution=self.resolution, cells=cells)
        pose = dict(x=0.0, y=0.0, yaw=0.0)

        choice_v3 = choose_view(self.grid, quality, pose, [0, 0], 6.0, [], 100, 'coverage_view_v3')
        self.assertIsNotNone(choice_v3)
        self.assertGreater(len(choice_v3['visible_cells']), 0)

    def test_legacy_strategy_preserves_planar_behavior(self):
        """v1 and v2 strategies must remain 100% planar even when elevation data exists."""
        cells = self._setup_target_cells()
        qr = self.resolution
        # Add high obstacle
        for x in range(-32, 32):
            for y in range(-32, 32):
                wx, wy = (x + 0.5) * qr, (y + 0.5) * qr
                if 1.75 <= wx <= 2.25 and abs(wy) <= 0.75:
                    cells.append(dict(ix=x, iy=y, quality=0.8, z=0.60))

        quality_with_z = dict(resolution=qr, cells=cells)
        cells_no_z = [dict(c) for c in cells]
        for c in cells_no_z:
            c.pop('z', None)
        quality_no_z = dict(resolution=qr, cells=cells_no_z)

        pose = dict(x=0.0, y=0.0, yaw=0.0)

        choice_v2_with_z = choose_view(self.grid, quality_with_z, pose, [0, 0], 8.0, [], 6.0, 'coverage_view_v2')
        choice_v2_no_z = choose_view(self.grid, quality_no_z, pose, [0, 0], 8.0, [], 6.0, 'coverage_view_v2')

        self.assertEqual(choice_v2_with_z['visible_cells'], choice_v2_no_z['visible_cells'],
                         "v2 strategy must be unaffected by elevation data")




class SupportedUnknownVisibilityTests(unittest.TestCase):
    def fixture(self,points=4):
        resolution=.25;width=80;origin=-10.
        y,x=np.mgrid[:width,:width]
        known=np.hypot(origin+(x+.5)*resolution,origin+(y+.5)*resolution)<=1.
        costs=np.full((width,width),-1,dtype=int);costs[known]=0
        grid=NS(info=NS(width=width,height=width,resolution=resolution,
                       origin=NS(position=NS(x=origin,y=origin))),data=costs.ravel().tolist())
        cells=[dict(ix=x,iy=y,quality=1.,points=points) for x in range(-12,12) for y in range(-12,12)]
        quality=dict(resolution=resolution,cells=cells)
        pose=dict(x=0.,y=0.,yaw=0.)
        history=[dict(x=0.,y=0.,yaw=float(yaw)) for yaw in np.arange(-math.pi,math.pi,math.pi/4) if abs(yaw)>1e-9]
        return grid,quality,pose,history

    def test_supported_unknown_transmits_prediction_but_unobserved_frontier_stops(self):
        grid,quality,pose,history=self.fixture()
        arguments=(grid,quality,pose,[0.,0.],8.,history,100.,'coverage_view_v2')
        baseline,old=choose_view(*arguments,return_diagnostics=True)
        explicit_false=choose_view(*arguments,depth_observed_visibility=False)
        self.assertIsNone(baseline);self.assertEqual(baseline,explicit_false)
        chosen,diag=choose_view(*arguments,return_diagnostics=True,depth_observed_visibility=True)
        self.assertIsNotNone(chosen)
        self.assertEqual(chosen['kind'],'turn');self.assertEqual(chosen['yaw'],0.)
        self.assertGreater(chosen['expected_new_m2'],.15)
        visible={tuple(key) for key in chosen['visible_cells']}
        self.assertTrue(any(key[0]==12 for key in visible))
        self.assertFalse(any(key[0]>12 for key in visible))
        # Optical support never converts an unknown cell into a route/target.
        self.assertEqual(diag['reachable_move_positions'],0)
        self.assertEqual(diag['feasible_moves'],0)
        self.assertEqual(old['reachable_move_positions'],0)

    def test_absent_zero_nonfinite_and_boolean_points_do_not_prove_depth_support(self):
        for points in (None,0,-1,float('nan'),float('inf'),True):
            with self.subTest(points=points):
                grid,quality,pose,history=self.fixture(points)
                self.assertIsNone(choose_view(grid,quality,pose,[0.,0.],8.,history,100.,
                                             'coverage_view_v2',depth_observed_visibility=True))
        grid,quality,pose,history=self.fixture()
        for cell in quality['cells']:cell.pop('points')
        self.assertIsNone(choose_view(grid,quality,pose,[0.,0.],8.,history,100.,
                                     'coverage_view_v2',depth_observed_visibility=True))

    def test_known_lethal_obstacle_still_blocks_supported_unknown_visibility(self):
        grid,quality,pose,history=self.fixture()
        costs=np.array(grid.data).reshape(80,80);costs[:,45]=100;grid.data=costs.ravel().tolist()
        self.assertIsNone(choose_view(grid,quality,pose,[0.,0.],8.,history,100.,
                                     'coverage_view_v2',depth_observed_visibility=True))

    def test_recursive_boundary_fallback_preserves_explicit_visibility_policy(self):
        width=20;resolution=.25
        grid=NS(info=NS(width=width,height=width,resolution=resolution,
                       origin=NS(position=NS(x=-2.5,y=-2.5))),data=[0]*(width*width))
        quality=dict(resolution=resolution,cells=[dict(ix=x,iy=y,quality=1.,points=1)
                                                for x in range(-10,10) for y in range(-10,10)])
        with patch('autonomous_survey.choose_view',wraps=choose_view) as recursive:
            chosen=choose_view(grid,quality,dict(x=0.,y=0.,yaw=0.),[0.,0.],8.,[],100.,
                               'coverage_view_v2',depth_observed_visibility=True)
        self.assertIsNotNone(chosen);self.assertTrue(chosen['map_boundary_fallback'])
        self.assertTrue(recursive.called)
        self.assertIs(recursive.call_args.kwargs['depth_observed_visibility'],True)


class DepthFallbackTests(unittest.TestCase):
    def patch(self):
        r=.1;v,u=np.mgrid[:200,:200]
        x,y=-10+(u+.5)*r,-10+(v+.5)*r
        # A real known corridor leads to a circumscribed safe patch offset from
        # the original eight radial candidate headings. Outside remains unknown.
        projection=np.clip((1.5*x+.6*y)/(1.5**2+.6**2),0.,1.)
        corridor=(x-1.5*projection)**2+(y-.6*projection)**2<=.22**2
        bulb=(x-1.5)**2+(y-.6)**2<=.85**2
        costs=np.full((200,200),-1,dtype=np.int16);costs[corridor|bulb]=0
        grid=NS(data=costs.ravel(),info=NS(width=200,height=200,resolution=r,
                origin=NS(position=NS(x=-10.,y=-10.))))
        return grid,dict(resolution=.25,cells=[]),dict(x=0.,y=0.,yaw=0.),costs

    def select(self,grid,quality,pose,*,depth=True,radius=22.):
        return choose_view(grid,quality,pose,[0.,0.],radius,[],120.,'surface_view_v2',
            return_diagnostics=True,_boundary_fallback=True,depth_observed_visibility=depth)

    def test_observed_diagonal_safe_patch_can_be_used_inside_allocated_depth_grid(self):
        grid,q,pose,costs=self.patch()
        decision,diagnostics=self.select(grid,q,pose)
        self.assertIsNotNone(decision)
        self.assertGreater(diagnostics['feasible_moves'],0)
        ix=int(math.floor((decision['x']+10.)/.1));iy=int(math.floor((decision['y']+10.)/.1))
        self.assertGreater(min(ix,iy,199-ix,199-iy)*.1,3.)
        self.assertLessEqual(costs[iy,ix],65)
        self.assertTrue(known_chassis_clearance(costs,ix,iy,.1))
        self.assertGreaterEqual(decision['expected_utility_m2'],.15)

    def test_legacy_2d_keeps_allocated_edge_filter(self):
        grid,q,pose,_=self.patch()
        decision,diagnostics=self.select(grid,q,pose,depth=False)
        self.assertIsNone(decision)
        self.assertEqual(diagnostics['reachable_move_positions'],0)

    def test_unknown_clearance_path_and_radius_still_block(self):
        grid,q,pose,costs=self.patch()
        costs[:]=-1;costs[100,100]=0
        self.assertIsNone(self.select(grid,q,pose)[0])
        grid,q,pose,costs=self.patch()
        # Unknown slices break the only observed path; they are never traversed
        # merely because measured depth can be optically transparent elsewhere.
        costs[:,107:109]=-1
        self.assertIsNone(self.select(grid,q,pose)[0])
        grid,q,pose,costs=self.patch()
        # The known bulb is outside radius-.75; allocated size grants no permit.
        self.assertIsNone(self.select(grid,q,pose,radius=2.)[0])



if __name__ == '__main__':
    unittest.main()
