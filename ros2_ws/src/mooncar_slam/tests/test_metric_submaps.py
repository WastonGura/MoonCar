"""Independent bounded-range geometry; no world mesh or captured truth input."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from laser_odometry_core import rotation
from metric_laser_odometry import MetricLaserOdometry


def geometry():
    # A long set of distinct surveyed surfaces. Visibility is explicitly range
    # limited; later sections have no overlap with the initial measured target.
    x = np.arange(-4., 19., .08)
    patches = [np.c_[x, np.full(len(x), y)] for y in (-2.1, 2.3)]
    for index, x in enumerate(np.arange(-2., 19., 1.7)):
        y = np.arange(1.1 if index % 2 else -2.7, 2.65 if index % 2 else -1.15, .06)
        patches.append(np.c_[np.full(len(y), x), y])
    return np.concatenate(patches)


class MetricSubmapTests(unittest.TestCase):
    def measured(self, matcher, scene, position, stamp, rng, yaw=0., vx=0., wz=0., radius=3.7):
        body = (scene-np.asarray(position)) @ rotation(yaw)-np.array([.14, 0.])
        ranges = np.linalg.norm(body, axis=1)
        visible = (ranges > .08) & (ranges < radius-.01)
        body, ranges = body[visible], ranges[visible]
        angles = np.arctan2(body[:, 1], body[:, 0])
        ranges += rng.normal(0., .01, len(ranges))
        return matcher.update(stamp, ranges, angles, .08, radius, yaw, vx, wz)

    def initialized(self, seed=831):
        matcher, rng, scene = MetricLaserOdometry(), np.random.default_rng(seed), geometry()
        for index in range(10):
            result = self.measured(matcher, scene, [0., 0.], 1.+index*.1, rng)
        self.assertTrue(result['accepted'], result)
        return matcher, rng, scene

    def outward(self, matcher, rng, scene, distance=10., step=.04):
        values = []
        for index, x in enumerate(np.arange(step, distance+step/2, step), 1):
            result = self.measured(matcher, scene, [x, 0.], 1.9+index*.1, rng, vx=step/.1*.98)
            self.assertTrue(result['accepted'], (x, result))
            values.append((x, result))
        return values

    def test_leaves_initial_range_and_returns_to_immutable_anchor(self):
        matcher, rng, scene = self.initialized()
        target, normals, means = matcher.target.copy(), matcher.normals.copy(), matcher.normal_means.copy()
        values = self.outward(matcher, rng, scene)
        self.assertGreater(values[-1][1]['reference_id'], 0)
        self.assertLess(np.linalg.norm(values[-1][1]['xy']-[10., 0.]), .03)
        self.assertGreater(values[-1][1]['anchor_covariance'].trace(), 0.)
        self.assertLessEqual(len(matcher.local_references), matcher.MAX_LOCAL_REFERENCES)
        # Turn with measured heading before returning. Wheel input alone carries
        # 2% scale error throughout both legs; it cannot pass the endpoint test.
        stamp = matcher.stamp
        for yaw in np.linspace(0., np.pi, 21)[1:]:
            stamp += .1
            result = self.measured(matcher, scene, [10., 0.], stamp, rng, yaw=yaw, wz=1.)
            self.assertTrue(result['accepted'], result)
        for x in np.arange(9.96, -.02, -.04):
            stamp += .1
            result = self.measured(matcher, scene, [max(x, 0.), 0.], stamp, rng,
                                   yaw=np.pi, vx=.4*.98)
            self.assertTrue(result['accepted'], (x, result))
        self.assertEqual(result['reference_id'], 0)
        self.assertLess(np.linalg.norm(result['xy']), .015)
        np.testing.assert_array_equal(matcher.target, target)
        np.testing.assert_array_equal(matcher.normals, normals)
        np.testing.assert_array_equal(matcher.normal_means, means)
        self.assertLessEqual(len(matcher.measured_frames), 20)
        self.assertLessEqual(len(matcher.local_references), matcher.MAX_LOCAL_REFERENCES)

    def test_no_overlap_does_not_create_a_new_reference_or_pose(self):
        matcher, rng, scene = self.initialized()
        self.outward(matcher, rng, scene, distance=2.)
        reference_ids = [row['id'] for row in matcher.local_references]
        target = matcher.target.copy()
        # Different, distant local geometry has no anchored overlap. It cannot
        # bootstrap a fresh coordinate origin just because normals look valid.
        alien = np.r_[np.c_[np.linspace(6., 8., 40), np.full(40, 7.)],
                      np.c_[np.full(40, 7.), np.linspace(6., 8., 40)]]
        result = self.measured(matcher, alien, [0., 0.], matcher.stamp+.1, rng, vx=.4, radius=20.)
        self.assertFalse(result['accepted'], result)
        self.assertNotIn('xy', result)
        self.assertEqual(reference_ids, [row['id'] for row in matcher.local_references])
        self.assertEqual(len(matcher.measured_frames), 0)
        self.assertEqual(len(matcher.fresh_window.rows), 0)
        np.testing.assert_array_equal(matcher.target, target)

    def test_cross_region_lateral_motion_is_measured_with_biased_forward_wheels(self):
        for seed in (391, 1411):
            with self.subTest(seed=seed):
                matcher, rng, scene = self.initialized(seed)
                errors = []
                for index, x in enumerate(np.arange(.04, 8.02, .04), 1):
                    position = [x, .15*np.sin(x*.4)]
                    result = self.measured(matcher, scene, position, 1.9+index*.1, rng, vx=.392)
                    self.assertTrue(result['accepted'], (seed, position, result))
                    errors.append(np.linalg.norm(result['xy']-position))
                self.assertGreater(result['reference_id'], 0)
                self.assertLess(np.percentile(errors, 95), .03)
                self.assertLess(abs(result['xy'][1]-position[1]), .02)
                self.assertTrue(all(np.linalg.eigvalsh(row['anchor_covariance']).min() > 0
                                    for row in matcher.local_references))
                self.assertTrue(all(all(parent < row['id'] for parent in row['parent_ids'])
                                    for row in matcher.local_references))

    def test_single_wall_handoff_remains_unobservable(self):
        matcher, rng, scene = self.initialized()
        self.outward(matcher, rng, scene, distance=6.)
        before = matcher.reference_sequence
        # A nearby horizontal surface supplies many samples but only one normal.
        wall = np.c_[np.linspace(3., 9., 160), np.full(160, 2.3)]
        result = self.measured(matcher, wall, [6.04, 0.], matcher.stamp+.1, rng, vx=.4)
        self.assertFalse(result['accepted'], result)
        self.assertNotIn('xy', result)
        self.assertEqual(matcher.reference_sequence, before)
        self.assertFalse(matcher.measured_frames)

    def test_batched_local_normals_retain_original_metric_gates(self):
        matcher, _, _ = self.initialized()
        # Validate the accelerated rebuild independently against the original
        # scalar metric neighborhoods, rather than only checking a moving path.
        from metric_laser_odometry import _MetricNormals
        original = _MetricNormals()
        original.reference(matcher.target.copy())
        batched = matcher._local_model(matcher.target.copy())
        np.testing.assert_array_equal(batched.target, original.target)
        np.testing.assert_array_equal(batched.normal_valid, original.normal_valid)
        np.testing.assert_array_equal(batched.normal_support_m, original.normal_support_m)
        np.testing.assert_allclose(batched.normal_means, original.normal_means, atol=1e-13)
        # Normal signs are arbitrary; their outer products encode the same line.
        np.testing.assert_allclose(batched.normals[:, :, None]*batched.normals[:, None, :],
            original.normals[:, :, None]*original.normals[:, None, :], atol=1e-12)

    def test_stationary_scans_cannot_replace_or_accumulate_reference_geometry(self):
        matcher, rng, scene = self.initialized()
        self.outward(matcher, rng, scene, distance=1.)
        sequence, frames = matcher.reference_sequence, len(matcher.measured_frames)
        for index in range(1, 31):
            self.measured(matcher, scene, [1., 0.], matcher.stamp+.1, rng)
        self.assertEqual(matcher.reference_sequence, sequence)
        self.assertEqual(len(matcher.measured_frames), frames)


if __name__ == '__main__':
    unittest.main()
