"""Unit tests for R1 slope-safe heading adjustment (scripts/slope_heading.py & integrations)."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

from autonomous_survey import AutonomousSurvey, choose_view
from slope_heading import (
    GENTLE_SLOPE_MAX_DEG,
    MAX_DEPARTURE_HEADING_ERR_RAD,
    assess_local_slope,
    plan_heading_adjustment,
    quaternion_to_rpy,
)


def make_grid(width=160, height=160, resolution=0.1, ox=-8.0, oy=-8.0, fill=0):
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


def make_ramp_quality(qr=0.25, xmin=-2.0, xmax=7.5, ymin=-4.0, ymax=4.0,
                      slope_fn=None, roughness=0.005):
    """Synthesize a measured terrain quality map with configurable slope_fn(x, y) in degrees."""
    cells = []
    ix0, ix1 = int(math.floor(xmin / qr)), int(math.ceil(xmax / qr))
    iy0, iy1 = int(math.floor(ymin / qr)), int(math.ceil(ymax / qr))
    for ix in range(ix0, ix1):
        wx = (ix + 0.5) * qr
        for iy in range(iy0, iy1):
            wy = (iy + 0.5) * qr
            s_deg = float(slope_fn(wx, wy)) if slope_fn else 0.0
            gx = math.tan(math.radians(s_deg))
            cells.append(dict(
                ix=ix, iy=iy, quality=0.9, z=0.0,
                gx=gx, gy=0.0, slope=s_deg, roughness=roughness,
                fit_state='valid', valid=True, bearing_mask=255, points=20,
            ))
    return dict(resolution=qr, cells=cells, frames=20, stamp=20.0)


class SlopeHeadingUnitTests(unittest.TestCase):
    def test_quaternion_to_rpy_roundtrip(self):
        pitch = math.radians(8.8)
        qx, qy, qz, qw = 0.0, math.sin(pitch / 2.0), 0.0, math.cos(pitch / 2.0)
        r, p, y = quaternion_to_rpy(qx, qy, qz, qw)
        self.assertAlmostEqual(r, 0.0, places=5)
        self.assertAlmostEqual(p, pitch, places=5)
        self.assertAlmostEqual(y, 0.0, places=5)

    def test_assess_local_slope_distinguishes_flat_steep_rough_and_unverified(self):
        flat_q = make_ramp_quality(slope_fn=lambda x, y: 1.5)
        res_flat = assess_local_slope(dict(x=0.0, y=0.0, yaw=0.0), flat_q)
        self.assertTrue(res_flat['in_place_turn_safe'])
        self.assertFalse(res_flat['slope_exceeded'])
        self.assertLess(res_flat['slope_deg'], GENTLE_SLOPE_MAX_DEG)

        # Steep map slope (8.8 deg ramp)
        steep_q = make_ramp_quality(slope_fn=lambda x, y: 8.8)
        res_steep = assess_local_slope(dict(x=4.0, y=0.0, yaw=0.0), steep_q)
        self.assertFalse(res_steep['in_place_turn_safe'])
        self.assertTrue(res_steep['slope_exceeded'])
        self.assertGreater(res_steep['slope_deg'], 8.0)

        # Flat map but body pitch says 9 deg -> conservative max wins!
        res_tilt = assess_local_slope(dict(x=0.0, y=0.0, yaw=0.0, roll=0.0, pitch=math.radians(9.0)), flat_q)
        self.assertFalse(res_tilt['in_place_turn_safe'])
        self.assertTrue(res_tilt['slope_exceeded'])

        # Excessive roughness rejects in-place turn even on flat slope
        rough_q = make_ramp_quality(slope_fn=lambda x, y: 1.0, roughness=0.04)
        res_rough = assess_local_slope(dict(x=0.0, y=0.0, yaw=0.0), rough_q)
        self.assertFalse(res_rough['in_place_turn_safe'])
        self.assertTrue(res_rough['roughness_exceeded'])

        # Missing slope evidence is unverified (not safe for blind Spin)
        res_empty = assess_local_slope(dict(x=0.0, y=0.0, yaw=0.0), dict(resolution=0.25, cells=[]))
        self.assertFalse(res_empty['in_place_turn_safe'])
        self.assertEqual(res_empty['reason'], 'unverified_local_slope')

    def test_observe_heading_on_steep_slope_uses_gimbal_with_3d_tilt_comp(self):
        grid, _ = make_grid(fill=0)
        steep_q = make_ramp_quality(slope_fn=lambda x, y: 8.8)
        pose = dict(x=4.0, y=0.0, yaw=0.0, roll=0.0, pitch=math.radians(-8.8))
        plan = plan_heading_adjustment(
            'observe_heading', math.pi / 2.0, pose, grid, steep_q,
            (-8.0, -8.0, 8.0, 8.0), [0.0, 0.0], 8.0, 120.0,
            gimbal_available=True,
        )
        self.assertTrue(plan['feasible'])
        self.assertEqual(plan['strategy'], 'gimbal_observe')
        self.assertFalse(plan['chassis_motion'])
        self.assertAlmostEqual(plan['pan'], math.pi / 2.0, places=3)
        self.assertEqual(len(plan['stages']), 1)
        self.assertEqual(plan['stages'][0]['kind'], 'gimbal')

    def test_travel_heading_on_flat_ground_preserves_in_place_turn(self):
        grid, _ = make_grid(fill=0)
        flat_q = make_ramp_quality(slope_fn=lambda x, y: 1.0)
        pose = dict(x=0.0, y=0.0, yaw=0.0, roll=0.0, pitch=0.0)
        plan = plan_heading_adjustment(
            'travel_heading', math.pi, pose, grid, flat_q,
            (-8.0, -8.0, 8.0, 8.0), [0.0, 0.0], 8.0, 120.0,
        )
        self.assertTrue(plan['feasible'])
        self.assertEqual(plan['strategy'], 'in_place_turn')
        self.assertFalse(plan['slope_safe_transit'])
        self.assertEqual(plan['stages'][0]['kind'], 'turn')

    def test_travel_heading_on_ramp_stages_uphill_to_gentle_crest(self):
        # Mirrors shipped mooncar_ramp_v2: 8.8 deg ramp from x=2..5.8, gentle crest x>=6.2
        grid, _ = make_grid(fill=0)
        def ramp_slope(x, y):
            if x <= 1.2 or x >= 6.2:
                return 1.5
            return 8.8
        quality = make_ramp_quality(slope_fn=ramp_slope)
        pose = dict(x=4.0, y=0.0, yaw=0.0, roll=0.0, pitch=math.radians(-8.8))
        plan = plan_heading_adjustment(
            'travel_heading', math.pi, pose, grid, quality,
            (-8.0, -8.0, 8.0, 8.0), [0.0, 0.0], 8.0, 180.0,
        )
        self.assertTrue(plan['feasible'])
        self.assertEqual(plan['strategy'], 'gentle_zone_staging')
        self.assertTrue(plan['slope_safe_transit'])
        self.assertGreaterEqual(plan['staging']['x'], 6.0)
        self.assertLessEqual(plan['staging']['departure_heading_err_deg'], math.degrees(MAX_DEPARTURE_HEADING_ERR_RAD))
        self.assertEqual([s['stage'] for s in plan['stages']], [
            'stage_transit', 'verify_gentle_settled', 'reorient_in_gentle_zone'
        ])
        self.assertTrue(plan['stages'][0]['slope_safe'])

    def test_rear_gentle_zone_rejected_falls_back_to_smooth_forward_arc(self):
        # Flat zone exists ONLY behind the rover (x <= 1.0); ahead (x > 1.5) is uniform 8.8 deg slope.
        # Reaching x <= 1.0 would require a 180-deg spin on the steep slope -> must be rejected!
        grid, _ = make_grid(fill=0)
        quality = make_ramp_quality(xmin=-2.0, xmax=7.5, ymin=-6.0, ymax=6.0,
                                    slope_fn=lambda x, y: 1.0 if x <= 1.0 else 8.8)
        pose = dict(x=3.5, y=0.0, yaw=0.0, roll=0.0, pitch=math.radians(-8.8))
        plan = plan_heading_adjustment(
            'travel_heading', math.pi / 2.0, pose, grid, quality,
            (-8.0, -8.0, 8.0, 8.0), [0.0, 0.0], 8.0, 180.0,
        )
        self.assertTrue(plan['feasible'])
        self.assertEqual(plan['strategy'], 'smooth_arc_maneuver')
        self.assertEqual(plan['maneuver_kind'], 'curvature_bounded_arc')
        self.assertTrue(plan['slope_safe_transit'])
        self.assertTrue(plan['arc']['swept_checked'])
        self.assertGreaterEqual(plan['arc']['radius_m'], 1.0)
        self.assertGreaterEqual(len(plan['stages']), 3)
        self.assertTrue(all(st['slope_safe'] for st in plan['stages']))

    def test_refuses_safely_when_slope_corridor_is_unknown_or_blocked(self):
        # Narrow 1.4 m observed corridor on an 8.8 deg slope flanked by unknown (-1) cells:
        # neither a gentle zone nor a swept-clear arc fits -> explicit safe refusal!
        grid, arr = make_grid(fill=-1)
        # Mark only a narrow strip y in [-0.7, 0.7], x in [2.5, 5.5] as observed free
        arr[73:87, 105:135] = 0
        quality = make_ramp_quality(xmin=2.5, xmax=5.5, ymin=-0.7, ymax=0.7, slope_fn=lambda x, y: 8.8)
        pose = dict(x=4.0, y=0.0, yaw=0.0, roll=0.0, pitch=math.radians(-8.8))
        plan = plan_heading_adjustment(
            'travel_heading', math.pi, pose, grid, quality,
            (-8.0, -8.0, 8.0, 8.0), [0.0, 0.0], 8.0, 180.0,
        )
        self.assertFalse(plan['feasible'])
        self.assertEqual(plan['strategy'], 'refused_unsafe_slope_turn')
        self.assertFalse(plan['chassis_motion'])
        self.assertEqual(plan['stages'], [])

    def test_hybrid_frontier_v4_enables_slope_safe_on_steep_slope(self):
        # Rover on an 8.8 deg slope facing +X; unobserved frontier exists ahead and behind.
        grid, arr = make_grid(width=200, height=200, resolution=0.1, ox=-10.0, oy=-10.0, fill=0)
        region = (-10.0, -10.0, 10.0, 10.0)
        quality = make_ramp_quality(xmin=-2.0, xmax=3.0, ymin=-2.0, ymax=2.0, slope_fn=lambda x, y: 8.8)
        pose = dict(x=0.0, y=0.0, z=0.0, yaw=0.0, roll=0.0, pitch=math.radians(-8.8))
        choice, diag = choose_view(
            grid, quality, pose, [0.0, 0.0], None, [], 180.0,
            'hybrid_frontier_v4', return_diagnostics=True,
            region=region, gimbal_available=False, simulation_profile='r5_support_v1',
        )
        self.assertTrue(diag['slope_safe_active'])
        self.assertIsNotNone(choice)
        self.assertNotEqual(choice['kind'], 'turn')
        self.assertTrue(choice['slope_safe'])
        bearing = math.atan2(choice['y'] - pose['y'], choice['x'] - pose['x'])
        self.assertLessEqual(abs(math.atan2(math.sin(bearing), math.cos(bearing))), MAX_DEPARTURE_HEADING_ERR_RAD + 1e-6)

    def test_autonomous_survey_executes_gentle_staging_transaction_with_slope_safe(self):
        grid, _ = make_grid(fill=0)
        quality = make_ramp_quality(slope_fn=lambda x, y: 1.5 if x >= 6.2 else 8.8)

        state = dict(pose=dict(x=4.0, y=0.0, z=0.28, yaw=0.0),
                     att=dict(roll=0.0, pitch=math.radians(-8.8), yaw=0.0, tilt_deg=8.8),
                     sim=10.0)
        dispatched = []

        motion = Mock()
        motion.lock = __import__('threading').RLock()
        motion.grid = grid
        motion.quality = quality
        motion.gimbal_position = [0.0, -0.2618]
        motion.grid_fresh.return_value = True
        motion.pose.side_effect = lambda: dict(state['pose'])
        motion.pose_attitude.side_effect = lambda: dict(state['att'])
        def tick_now():
            state['sim'] += 0.5
            return SimpleNamespace(nanoseconds=int(state['sim'] * 1e9))

        motion.get_clock.return_value = SimpleNamespace(now=tick_now)
        motion.state.return_value = dict(
            active=False, phase='succeeded', ready=True, error='',
            diagnostics=dict(goal_dispatch=dict(slope_safe=True, controller_id='SlopeFollowPath')),
        )

        survey = AutonomousSurvey(motion)
        survey.task = dict(id='exp-slope', region=[-8.0, -8.0, 8.0, 8.0], simulation_profile='r5_support_v1')
        survey.origin = [0.0, 0.0]
        survey.data.update(active=True, started=0.0, budget_seconds=240.0, radius=8.0)
        survey.healthy = lambda: True

        def fake_execute(decision, turn=False):
            dispatched.append((turn, dict(decision)))
            state['sim'] += 5.0
            if not turn:
                state['pose']['x'] = decision['x']
                state['pose']['y'] = decision['y']
                # Arrived on flat crest -> tilt drops to 1.5 deg!
                state['att'] = dict(roll=0.0, pitch=math.radians(-1.5), yaw=0.0, tilt_deg=1.5)
            else:
                state['pose']['yaw'] = decision['yaw']
            return True

        survey.execute = fake_execute
        ok, rec = survey.execute_slope_safe_heading(math.pi, intent='travel_heading')
        self.assertTrue(ok)
        self.assertEqual(rec['status'], 'succeeded')
        self.assertEqual(rec['plan']['strategy'], 'gentle_zone_staging')
        self.assertEqual(rec['completed_stages'], [
            'stage_transit', 'verify_gentle_settled', 'reorient_in_gentle_zone'
        ])
        self.assertEqual(len(dispatched), 2)
        self.assertFalse(dispatched[0][0])  # first action is advance transit
        self.assertTrue(dispatched[0][1]['slope_safe'])  # must use SlopeFollowPath!
        self.assertTrue(dispatched[1][0])   # second action is flat-crest turn
        self.assertAlmostEqual(dispatched[1][1]['yaw'], math.pi, places=4)


if __name__ == '__main__':
    unittest.main()
