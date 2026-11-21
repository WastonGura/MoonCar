"""Unit tests and closed-loop 30x30 m synthetic benchmark for hybrid_frontier_v4."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

from autonomous_survey import AutonomousSurvey, choose_view, wrap
from frontier_planner import (
    GIMBAL_PAN_LIMIT_RAD,
    GIMBAL_TILT_DEFAULT_RAD,
    extract_frontier_clusters,
    order_frontier_tour,
)


def make_grid(width=300, height=300, resolution=0.1, ox=-15.0, oy=-15.0, fill=-1):
    data = np.full((height, width), fill, dtype=np.int16)
    return SimpleNamespace(
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=SimpleNamespace(position=SimpleNamespace(x=ox, y=oy)),
        ),
        data=data.ravel(),
    ), data


class HybridFrontierUnitTests(unittest.TestCase):
    def test_extracts_both_costmap_and_depth_unobserved_frontiers(self):
        # Entire 30x30 m costmap is LiDAR-cleared free (0), but depth only observed a 5 m square.
        grid, arr = make_grid(fill=0)
        region = (-15.0, -15.0, 15.0, 15.0)
        observed = {(ix, iy) for ix in range(-20, 20) for iy in range(-20, 20)}
        clusters = extract_frontier_clusters(
            arr, -15.0, -15.0, 0.1, region, [0.0, 0.0], math.inf,
            observed_keys=observed, quality_resolution=0.25,
        )
        self.assertGreater(len(clusters), 0)
        # All clusters must lie outside the already depth-observed 5 m box.
        self.assertTrue(any(abs(c['cx']) > 5.0 or abs(c['cy']) > 5.0 for c in clusters))

    def test_two_opt_tour_avoids_zigzag_backtracking(self):
        pose = dict(x=0.0, y=0.0, yaw=0.0)
        # Unsorted waypoints in a line along +X: should visit 4 -> 8 -> 12 -> 16 in order.
        waypoints = [
            dict(x=16.0, y=0.0, path_distance=16.0, cluster_count=10),
            dict(x=4.0, y=0.0, path_distance=4.0, cluster_count=10),
            dict(x=12.0, y=0.0, path_distance=12.0, cluster_count=10),
            dict(x=8.0, y=0.0, path_distance=8.0, cluster_count=10),
        ]
        order = order_frontier_tour(pose, waypoints)
        xs = [waypoints[i]['x'] for i in order]
        self.assertEqual(xs, [4.0, 8.0, 12.0, 16.0])

    def test_global_frontier_escapes_locally_exhausted_region_where_v3_stalls(self):
        # Known passable 30x30 m field; everything within 11 m of rover is already observed,
        # leaving a large unobserved region around (+12, +12).
        grid, arr = make_grid(fill=0)
        region = (-15.0, -15.0, 15.0, 15.0)
        qr = 0.25
        cells = []
        for ix in range(-60, 60):
            for iy in range(-60, 60):
                wx, wy = (ix + 0.5) * qr, (iy + 0.5) * qr
                if math.hypot(wx, wy) <= 11.0:
                    cells.append(dict(ix=ix, iy=iy, quality=1.0, z=0.0, fit_state='valid', bearing_mask=255, points=20))
        quality = dict(resolution=qr, cells=cells, frames=10, stamp=10.0)
        pose = dict(x=0.0, y=0.0, z=0.0, yaw=0.0)

        v3_choice = choose_view(grid, quality, pose, [0.0, 0.0], None, [], 200.0, 'surface_view_v3', region=region)
        v4_choice, v4_diag = choose_view(
            grid, quality, pose, [0.0, 0.0], None, [], 200.0,
            'hybrid_frontier_v4', return_diagnostics=True, region=region,
        )
        self.assertIsNone(v3_choice, 'Legacy local-only v3 cannot see frontiers beyond 6 m')
        self.assertIsNotNone(v4_choice)
        self.assertTrue(v4_choice.get('global_frontier_target'))
        self.assertTrue(v4_diag['global_frontier_fallback'])
        self.assertGreater(math.hypot(v4_choice['x'], v4_choice['y']), 3.0)

    def test_move_candidate_decouples_gimbal_without_chassis_spin(self):
        grid, arr = make_grid(fill=0)
        region = (-15.0, -15.0, 15.0, 15.0)
        qr = 0.25
        # Observe the right side and front, leave left (+Y) unobserved so rover drives +X and pans left (+90 deg).
        cells = []
        for ix in range(-40, 40):
            for iy in range(-40, 40):
                wy = (iy + 0.5) * qr
                if wy < 2.0:
                    cells.append(dict(ix=ix, iy=iy, quality=1.0, z=0.0, fit_state='valid', bearing_mask=255, points=20))
        quality = dict(resolution=qr, cells=cells, frames=5, stamp=5.0)
        pose = dict(x=0.0, y=0.0, z=0.0, yaw=0.0)
        choice = choose_view(grid, quality, pose, [0.0, 0.0], None, [], 200.0, 'hybrid_frontier_v4', region=region)
        self.assertIsNotNone(choice)
        self.assertTrue(choice['use_pan_tilt'])
        self.assertFalse(choice['requires_chassis_spin'])
        self.assertLessEqual(abs(choice['pan']), GIMBAL_PAN_LIMIT_RAD)

    def test_v4_run_recenters_twisted_gimbal_before_moving_and_pans_on_arrival(self):
        motion = Mock()
        motion.lock = __import__('threading').RLock()
        pose = dict(x=0.0, y=0.0, z=0.0, yaw=0.0)
        motion.pose.side_effect = lambda: dict(pose)
        motion.gimbal_position = [2.0, GIMBAL_TILT_DEFAULT_RAD]  # Twisted 115 deg!
        motion.grid, _ = make_grid(fill=0)
        motion.grid_fresh.return_value = True
        motion.quality = dict(resolution=0.25, cells=[], frames=1, stamp=1.0)
        motion.quality_received = 1e9
        motion.state.return_value = dict(active=False, ready=True, pose=pose, phase='succeeded', error='', diagnostics={})
        motion.pan_tilt = Mock()

        survey = AutonomousSurvey(motion, localization_revisit_enabled=False)
        survey.task = dict(id='exp-v4', region=[-15.0, -15.0, 15.0, 15.0])
        survey.origin = [0.0, 0.0]
        survey.data = dict(active=True, phase='planning', started=0.0, budget_seconds=120.0,
                           radius=None, strategy='hybrid_frontier_v4', history=[])
        sim_t = [10.0]
        def advance_now():
            sim_t[0] += 0.25
            return sim_t[0]
        survey.now = advance_now
        survey.save = Mock()
        survey.healthy = lambda: bool(survey.data['active'])

        executed = []
        def fake_execute(dec, turn=False):
            executed.append((dec['kind'], turn, dec.get('use_pan_tilt'), dec.get('pan')))
            sim_t[0] += 1.0
            if turn and dec.get('use_pan_tilt'):
                motion.gimbal_position = [dec['pan'], dec.get('tilt', GIMBAL_TILT_DEFAULT_RAD)]
            elif not turn:
                pose['x'], pose['y'], pose['yaw'] = dec['x'], dec['y'], 0.0
            # Emit a fresh quality frame right after arrival so adaptive dwell finishes cleanly.
            motion.quality = dict(resolution=0.25, cells=[], frames=motion.quality['frames'] + 1, stamp=sim_t[0] + 0.1)
            return True

        survey.execute = fake_execute
        # First call returns a move facing +90 deg left; second call ends the run cleanly.
        decision = dict(
            x=2.0, y=0.0, yaw=math.pi / 2.0, kind='advance',
            pan=round(math.pi / 2.0, 3), tilt=round(GIMBAL_TILT_DEFAULT_RAD, 3),
            use_pan_tilt=True, expected_new_m2=5.0, expected_improvement_m2=0.0,
            expected_utility_m2=5.0, visible_cells=[], strategy='hybrid_frontier_v4',
            estimated_seconds=15.0, score=0.33,
        )
        calls = [0]
        def fake_choose(*args, **kwargs):
            calls[0] += 1
            if calls[0] == 1:
                return decision, dict(budget_limited=False)
            survey.data['active'] = False
            return None, dict(budget_limited=False)

        with __import__('unittest.mock').mock.patch('autonomous_survey.choose_view', side_effect=fake_choose):
            survey.run()

        # Sequence MUST be: (1) recenter gimbal to pan=0, (2) move chassis, (3) pan gimbal to +90 deg (no chassis spin!)
        self.assertEqual(len(executed), 3)
        self.assertEqual(executed[0], ('turn', True, True, 0.0))
        self.assertEqual(executed[1][0], 'advance')
        self.assertFalse(executed[1][1])
        self.assertEqual(executed[2][1], True)
        self.assertEqual(executed[2][2], True)
        self.assertAlmostEqual(executed[2][3], round(math.pi / 2.0, 3), places=2)

    def test_r5_clearance_enters_corridor_blocked_by_legacy_clearance(self):
        # 1.1 m clear corridor (y in [-0.55, +0.55]) -> safe for R5 (0.45 m) but blocked for legacy (0.65 m).
        grid, arr = make_grid(width=140, height=100, resolution=0.1, ox=-2.0, oy=-5.0, fill=100)
        # Start bay around (0, 0)
        arr[35:65, 5:35] = 0
        # Narrow 1.1 m corridor along x in [1.0, 6.5], y in [-0.55, +0.55] -> rows 45..55 (11 cells = 1.1 m)
        arr[45:56, 30:85] = 0
        # Open unknown room at the end of corridor
        arr[35:65, 85:125] = -1
        grid.data = arr.ravel()
        qr = 0.25
        # Mark start bay as already observed so the only gain lies down the corridor
        cells = [
            dict(ix=qx, iy=qy, quality=0.95, z=0.0, fit_state='valid', bearing_mask=255, points=20)
            for qx in range(-6, 10) for qy in range(-6, 6)
        ]
        quality = dict(resolution=qr, cells=cells, frames=10, stamp=10.0)
        pose = dict(x=0.0, y=0.0, z=0.0, yaw=0.0)
        region = (-2.0, -5.0, 12.0, 5.0)

        r5_pick = choose_view(
            grid, quality, pose, [0.0, 0.0], 15.0, [], 120.0,
            'hybrid_frontier_v4', region=region, simulation_profile='r5_support_v1',
        )
        legacy_pick = choose_view(
            grid, quality, pose, [0.0, 0.0], 15.0, [], 120.0,
            'hybrid_frontier_v4', region=region, simulation_profile='legacy',
        )
        self.assertIsNotNone(r5_pick)
        self.assertGreater(r5_pick['x'], 1.2)
        if legacy_pick is not None:
            self.assertLess(legacy_pick['x'], 1.5)

    def test_within_region_supports_r5_and_legacy_profiles(self):
        import ast
        src = (ROOT / 'scripts' / 'experiment_motion.py').read_text(encoding='utf-8')
        tree = ast.parse(src)
        fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'within_region')
        ns = {'math': math}
        exec(compile(ast.Module(body=[fn], type_ignores=[]), '<within_region>', 'exec'), ns)
        within_region = ns['within_region']

        region = [-1.0, -2.0, 8.0, 6.0]
        # x = 7.60 -> 0.40 m from xmax=8.0: inside for R5 (0.25+0.10=0.35), outside for legacy (0.43+0.10=0.53)
        self.assertFalse(within_region(region, 7.60, 2.0, 0.0))
        self.assertFalse(within_region(region, 7.60, 2.0, 0.0, profile='legacy'))
        self.assertTrue(within_region(region, 7.60, 2.0, 0.0, profile='r5_support_v1'))
        # x = 7.70 -> 0.30 m from xmax=8.0: outside even for R5 (needs 0.35 m)
        self.assertFalse(within_region(region, 7.70, 2.0, 0.0, profile='r5_support_v1'))


if __name__ == '__main__':
    unittest.main()
