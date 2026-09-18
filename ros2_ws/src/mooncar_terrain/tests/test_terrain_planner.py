from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from terrain_planner import TerrainPlanner, Vehicle


def cells(size=12):
    return [dict(ix=x, iy=y, z=0., gx=0., gy=0., roughness=0., support=1., valid=True)
            for x in range(size) for y in range(size)]


class PlannerTests(unittest.TestCase):
    def test_reachable_goal_near_observed_boundary(self):
        planner = TerrainPlanner(cells(20), .25)
        goal = (4.72, 2.375)
        self.assertIsNotNone(planner.assess(*goal, 0.))
        result = planner.plan((.77, 2.375, 0.), goal, goal_yaw=0.)
        self.assertEqual(result['status'], 'ok')
        self.assertEqual((result['path'][-1]['x'], result['path'][-1]['y']), goal)
        for a,b in zip(result['path'], result['path'][1:]):
            self.assertIsNotNone(planner.motion_cost((a['x'],a['y'],a['yaw']), (b['x'],b['y'],b['yaw'])))

    def test_exact_endpoints_and_explicit_checked_turns(self):
        planner = TerrainPlanner(cells(20), .25)
        start, goal = (.91, 1.02, .1), (3.77, 2.01)
        result = planner.plan(start, goal, goal_yaw=.3)
        self.assertEqual(result['status'], 'ok')
        self.assertEqual((result['path'][0]['x'], result['path'][0]['y'], result['path'][0]['yaw']), start)
        last = result['path'][-1]
        self.assertEqual((last['x'], last['y'], last['yaw']), (*goal, .3))
        for a, b in zip(result['path'], result['path'][1:]):
            self.assertIsNotNone(planner.motion_cost((a['x'], a['y'], a['yaw']), (b['x'], b['y'], b['yaw'])))
        self.assertIsNone(planner.motion_cost((1., 1., 0.), (1., 2., 0.)))

    def test_directional_slope_and_unknown_footprint(self):
        surface = cells()
        for cell in surface:
            cell.update(gx=math.tan(math.radians(15)), z=(cell['ix'] + .5) * .25 * math.tan(math.radians(15)))
        planner = TerrainPlanner(surface, .25)
        self.assertIsNotNone(planner.assess(1., 1., 0.))
        self.assertIsNone(planner.assess(1., 1., math.pi / 2))
        self.assertIsNone(planner.assess(0., 0., 0.))
        self.assertEqual(planner.plan((1., 1., math.pi / 2), (2., 1.))['status'], 'invalid_start')

    def test_routes_and_support_ablation(self):
        surface = cells(20)
        for cell in surface:
            if 7 <= cell['ix'] <= 12 and 7 <= cell['iy'] <= 11:
                cell['support'] = .2
        planner = TerrainPlanner(surface, .25, Vehicle(uncertainty_weight=12.))
        start, goal = (.875, 2.375, 0.), (4.125, 2.375)
        fixed = planner.plan(start, goal, 'fixed')
        full = planner.plan(start, goal, 'full')
        self.assertEqual(fixed['status'], 'ok')
        self.assertEqual(full['status'], 'ok')
        self.assertGreater(full['length'], fixed['length'])
        for p in full['path']:
            self.assertIsNotNone(planner.assess(p['x'], p['y'], p['yaw']))

    def test_wall_step_failure_and_budget(self):
        surface = cells()
        for cell in surface:
            if cell['ix'] >= 6:
                cell['z'] = .3
        planner = TerrainPlanner(surface, .25)
        self.assertEqual(planner.plan((.875, 1.375, 0.), (2.375, 1.375), cancelled=lambda: True)['status'], 'cancelled')
        self.assertEqual(planner.plan((.875, 1.375, 0.), (2.375, 1.375))['status'], 'no_path')
        self.assertEqual(planner.plan((.875, 1.375, 0.), (2.375, 1.375), max_expansions=1)['status'], 'budget_exceeded')
        with self.assertRaises(ValueError):
            planner.plan((math.nan, 0., 0.), (1., 1.))
        with self.assertRaises(ValueError):
            TerrainPlanner(surface + [surface[0]], .25)


if __name__ == '__main__':
    unittest.main()
