import math
import pathlib
import sys
import time
import unittest
from unittest.mock import Mock

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
from terrain_geometry import classify_cells, compute_traversability_cost
from terrain_node import TerrainNode


class TraversabilityCostmapTest(unittest.TestCase):
    """Verifies continuous 2.5D traversability cost calculation and OccupancyGrid generation."""

    def test_compute_traversability_cost_flat_and_mild(self):
        """Flat ground and micro-roughness below mild thresholds produce zero cost."""
        self.assertEqual(compute_traversability_cost(0.0, 0.0, 0), 0)
        self.assertEqual(compute_traversability_cost(3.5, 0.010, 0), 0)
        self.assertEqual(compute_traversability_cost(5.0, 0.015, 0), 0)

    def test_compute_traversability_cost_monotonic_slopes(self):
        """Cost increases monotonically with slope angle, staying strictly < 99 for passable terrain."""
        slopes = [6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 17.5]
        costs = [compute_traversability_cost(s, 0.010, 0) for s in slopes]

        # Monotonicity check
        for c1, c2 in zip(costs, costs[1:]):
            self.assertLess(c1, c2)

        # Expected cost ranges
        self.assertGreaterEqual(costs[0], 1)
        self.assertLess(costs[0], 10)
        self.assertGreaterEqual(costs[3], 20)  # 12 deg
        self.assertLess(costs[3], 35)
        self.assertGreaterEqual(costs[-1], 60) # 17.5 deg
        self.assertLessEqual(costs[-1], 95)    # Must be <= 95 (< 99) so Nav2 can traverse when necessary

    def test_compute_traversability_cost_lethal_thresholds(self):
        """Any safety violation (slope, roughness, step, drop) must trigger lethal cost (100)."""
        # Slope >= 18 deg
        self.assertEqual(compute_traversability_cost(18.0, 0.010, 0), 100)
        self.assertEqual(compute_traversability_cost(25.0, 0.010, 0), 100)

        # Roughness >= 0.04m
        self.assertEqual(compute_traversability_cost(5.0, 0.040, 0), 100)
        self.assertEqual(compute_traversability_cost(0.0, 0.055, 0), 100)

        # Reason bitmask flags (1: slope, 2: roughness, 4: step, 8: drop)
        for reason in (1, 2, 4, 8, 15):
            self.assertEqual(compute_traversability_cost(0.0, 0.0, reason), 100)

    def test_roughness_penalty(self):
        """Rough surfaces add proportional traversal cost on top of slope."""
        smooth_cost = compute_traversability_cost(10.0, 0.015, 0)
        rough_cost = compute_traversability_cost(10.0, 0.035, 0)
        self.assertGreater(rough_cost, smooth_cost)

    def test_classify_cells_elevation_option(self):
        """classify_cells supports return_elevation while maintaining default 5-element compatibility."""
        x, y = np.meshgrid(np.linspace(0.01, 0.28, 12), np.linspace(0.01, 0.28, 12))
        points = np.column_stack((x.ravel(), y.ravel(), (0.5 * x).ravel()))

        default_cells = classify_cells(points, return_elevation=False)
        self.assertEqual(len(default_cells[0]), 5)

        elev_cells = classify_cells(points, return_elevation=True)
        self.assertEqual(len(elev_cells[0]), 6)
        # 6th element is fitted plane z elevation
        self.assertTrue(math.isfinite(elev_cells[0][5]))

    def test_evidence_keeps_the_observation_that_set_the_retained_cost(self):
        node = Mock(cell_evidence={})
        key = (2, 3)
        TerrainNode.record_cell_evidence(node, key, 10., 100, 100, 20., .01, 1, .4)
        TerrainNode.record_cell_evidence(node, key, 11., 100, 0, 2., .01, 0, .5)
        self.assertEqual(node.cell_evidence[key]['effective']['reason'], 1)
        self.assertEqual(node.cell_evidence[key]['effective']['stamp'], 10.)
        self.assertEqual(node.cell_evidence[key]['last_observed'], 11.)
        self.assertEqual(node.cell_evidence[key]['observations'], 2)
        TerrainNode.record_cell_evidence(node, key, 20., 0, 0, 2., .01, 0, .5)
        self.assertEqual(node.cell_evidence[key]['effective']['reason'], 0)

    def test_occupancy_grid_generation_and_eviction(self):
        """TerrainNode generates snapped OccupancyGrid, sets unknown to -1, and evicts stale cells."""
        node = Mock()
        node.traversability_pub = Mock()
        node.traversability_cells = {}
        node.get_parameter.side_effect = lambda name: {
            'traversability_resolution': Mock(value=0.20),
            'costmap_width': Mock(value=4.0),
            'costmap_height': Mock(value=4.0),
            'cell_persistence': Mock(value=10.0),
        }[name]
        node.frame = 'map'
        node.get_clock.return_value.now.return_value.to_msg.return_value = Mock()

        # Add 3 cells: one fresh at (1.0, 1.0), one fresh at (1.4, 1.0), one stale at (0.0, 0.0)
        now = time.monotonic()
        r = 0.20
        # (1.0, 1.0) -> cell index (5, 5), cost 30
        node.traversability_cells[(5, 5)] = (30, now)
        # (1.4, 1.0) -> cell index (7, 5), cost 80
        node.traversability_cells[(7, 5)] = (80, now)
        # stale cell older than 10 seconds -> evicted
        node.traversability_cells[(0, 0)] = (50, now - 15.0)

        TerrainNode.publish_traversability_map(node, robot_x=1.0, robot_y=1.0)

        # Stale cell should be evicted
        self.assertNotIn((0, 0), node.traversability_cells)
        self.assertIn((5, 5), node.traversability_cells)
        self.assertIn((7, 5), node.traversability_cells)

        # Check published OccupancyGrid
        node.traversability_pub.publish.assert_called_once()
        msg = node.traversability_pub.publish.call_args.args[0]
        self.assertEqual(msg.info.width, 20)  # 4.0m / 0.2m = 20
        self.assertEqual(msg.info.height, 20)
        self.assertAlmostEqual(msg.info.resolution, 0.20)

        # Origin snapped to multiple of 0.20
        self.assertAlmostEqual(msg.info.origin.position.x % 0.20, 0.0, places=5)
        self.assertAlmostEqual(msg.info.origin.position.y % 0.20, 0.0, places=5)

        data = np.array(msg.data, dtype=np.int8).reshape(20, 20)
        # Most cells should be unknown (-1)
        self.assertEqual(data[0, 0], -1)

        # Cell (5, 5) and (7, 5) must be present with their assigned costs
        origin_ix = int(round(msg.info.origin.position.x / 0.20))
        origin_iy = int(round(msg.info.origin.position.y / 0.20))
        self.assertEqual(data[5 - origin_iy, 5 - origin_ix], 30)
        self.assertEqual(data[5 - origin_iy, 7 - origin_ix], 80)


if __name__ == '__main__':
    unittest.main()
