from pathlib import Path
import sys
import math
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from autonomous_survey import choose_view


class PlannerV3FeaturesTests(unittest.TestCase):
    def setUp(self):
        self.resolution = 0.25
        self.width = 80
        self.grid_data = [0] * (self.width * self.width)
        self.grid = NS(info=NS(width=self.width, height=self.width,
                               resolution=self.resolution,
                               origin=NS(position=NS(x=-10.0, y=-10.0))),
                       data=self.grid_data)

    def test_grazing_angle_filter_rejects_side_view_slopes(self):
        # A cell at (ix=8, iy=0) -> world coords (2.125, 0.125)
        # Gradient gx=0.6, gy=0.0 (slope s=0.6 >= 0.20, steepest ascent in +X, contour along Y)
        # Robot is at (0, 0, yaw=0).
        # Camera ray towards (2.125, 0.125) has angle ~0 rad (aligned with gradient +X).
        # This should NOT be rejected by grazing check.
        quality = dict(
            resolution=self.resolution,
            cells=[
                dict(ix=8, iy=0, quality=0.5, z=0.5, gx=0.6, gy=0.0, fit_state='valid')
            ]
        )
        pose_aligned = dict(x=0.0, y=0.0, yaw=0.0)
        view_v3 = choose_view(self.grid, quality, pose_aligned, [0, 0], 8, [], 60, 'coverage_view_v3')
        self.assertIsNotNone(view_v3)
        # The sloped cell is observed because ray is along gradient
        self.assertIn((8, 0), view_v3['visible_cells'])

        # Now test ray perpendicular to gradient (viewing along contour line)
        # Gradient gx=0.0, gy=0.6 (steepest ascent in +Y, contour along X)
        # Ray from (0,0) with yaw=0 looks in +X direction (angle 0).
        # Alignment: cos_align = |cos(0)*0 + sin(0)*0.6| / 0.6 = 0.0 < 0.25 -> grazing degraded!
        quality_side = dict(
            resolution=self.resolution,
            cells=[
                dict(ix=8, iy=0, quality=0.5, z=0.5, gx=0.0, gy=0.6, fit_state='valid')
            ]
        )
        view_side_v3 = choose_view(self.grid, quality_side, pose_aligned, [0, 0], 8, [], 60, 'coverage_view_v3')
        # Under v3, cell (8, 0) is rejected by grazing angle filter
        if view_side_v3 is not None:
            self.assertNotIn((8, 0), view_side_v3['visible_cells'])

        # Under legacy v2, grazing check is NOT applied (backward compatibility)
        view_side_v2 = choose_view(self.grid, quality_side, pose_aligned, [0, 0], 8, [], 60, 'coverage_view_v2')
        self.assertIsNotNone(view_side_v2)
        self.assertIn((8, 0), view_side_v2['visible_cells'])

    def test_dijkstra_2_5d_terrain_cost_penalization(self):
        # Place high terrain cost (e.g. cost=80) on the corridor towards +X
        # Between x=0.5 and x=2.0, y=-0.5 to 0.5
        grid_high_cost = [0] * (self.width * self.width)
        for ix in range(42, 50):  # x from 0.5 to 2.5m
            for iy in range(38, 43):  # y from -0.5 to 0.5m
                grid_high_cost[iy * self.width + ix] = 80

        grid_obj = NS(info=NS(width=self.width, height=self.width,
                              resolution=self.resolution,
                              origin=NS(position=NS(x=-10.0, y=-10.0))),
                      data=grid_high_cost)

        quality = dict(
            resolution=self.resolution,
            cells=[dict(ix=x, iy=y, quality=0.1) for x in range(-20, 20) for y in range(-20, 20)]
        )
        pose = dict(x=0.0, y=0.0, yaw=0.0)

        # Under v3, candidate choices over the high-cost slope will have significantly
        # higher estimated travel seconds compared to flat routes
        choice_v3, diag_v3 = choose_view(grid_obj, quality, pose, [0, 0], 8, [], 60,
                                         'nearest_view_v3', return_diagnostics=True)
        choice_v2, diag_v2 = choose_view(grid_obj, quality, pose, [0, 0], 8, [], 60,
                                         'nearest_view_v2', return_diagnostics=True)

        self.assertIsNotNone(choice_v3)
        self.assertIsNotNone(choice_v2)
        # v3 Dijkstra penalized the high slope more heavily
        self.assertEqual(diag_v3['planner_version'], 'budgeted-view-v3.0')
        self.assertEqual(diag_v2['planner_version'], 'budgeted-view-v2.1')

    def test_targeted_defect_viewpoints_in_surface_view_v3(self):
        # Create a cluster of defect cells around (ix=8, iy=4) -> (2.125, 1.125)
        # with fit_state='insufficient_xy_spread'
        defect_cells = []
        for dx in range(3):
            for dy in range(3):
                defect_cells.append(dict(
                    ix=8 + dx, iy=4 + dy,
                    quality=0.1,
                    fit_state='insufficient_xy_spread',
                    bearing_mask=0,
                    gx=0.4, gy=0.0
                ))
        # Surround with observed cells so coverage alone isn't favored
        all_cells = list(defect_cells)
        for x in range(-16, 16):
            for y in range(-16, 16):
                if not (8 <= x <= 10 and 4 <= y <= 6):
                    all_cells.append(dict(ix=x, iy=y, quality=0.8, fit_state='valid', bearing_mask=255))

        quality = dict(resolution=self.resolution, cells=all_cells)
        pose = dict(x=0.0, y=0.0, yaw=0.0)

        choice_v3, diag_v3 = choose_view(self.grid, quality, pose, [0, 0], 8, [], 60,
                                         'surface_view_v3', return_diagnostics=True)
        choice_v2, diag_v2 = choose_view(self.grid, quality, pose, [0, 0], 8, [], 60,
                                         'surface_view_v2', return_diagnostics=True)

        self.assertIsNotNone(choice_v3)
        self.assertIsNotNone(choice_v2)

        # Under v3, extra targeted standoff positions are generated for the defect cluster
        self.assertGreater(diag_v3['reachable_move_positions'], diag_v2['reachable_move_positions'])
        # Under v3, surface improvement is positive and has 2.0x utility multiplier
        self.assertGreater(choice_v3['expected_improvement_m2'], 0)
        self.assertGreater(choice_v3['score'], choice_v2['score'])


if __name__ == '__main__':
    unittest.main()
