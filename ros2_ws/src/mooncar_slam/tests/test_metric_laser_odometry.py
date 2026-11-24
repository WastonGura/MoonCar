"""Independent synthetic geometry checks; no captured world or truth data."""
import unittest

import numpy as np
import json
from scipy.spatial import cKDTree

from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from metric_laser_odometry import MetricLaserOdometry as LaserOdometry, FreshWindowEstimate


def curve_scene(centers, radii, count=9):
    patches = []
    for center, radius in zip(centers, radii):
        center = np.asarray(center, dtype=float)
        facing = np.arctan2(-center[1], -center[0])
        angle = facing + np.linspace(-1., 1., count)
        patches.append(center + radius * np.c_[np.cos(angle), np.sin(angle)])
    return np.concatenate(patches)


def scan(points, position, rng, noise=.01):
    body = points - np.asarray(position) - np.array([.14, 0.])
    ranges = np.linalg.norm(body, axis=1)
    angles = np.arctan2(body[:, 1], body[:, 0])
    return ranges + rng.normal(0., noise, len(ranges)), angles


class AdaptiveNormalsTest(unittest.TestCase):
    def initialized(self, points, seed=711):
        matcher = LaserOdometry()
        rng = np.random.default_rng(seed)
        for index in range(10):
            ranges, angles = scan(points, [0., 0.], rng)
            result = matcher.update(1. + index * .1, ranges, angles, .05, 20., 0., 0., 0.)
        self.assertTrue(result['accepted'])
        return matcher, rng

    def update_at(self, matcher, points, position, rng, stamp):
        ranges, angles = scan(points, position, rng)
        return matcher.update(stamp, ranges, angles, .05, 20., 0., 0., 0.)

    def test_varied_curved_scenes_with_centimeter_noise(self):
        scenes = [
            (([-3., -2.], [4., -2.], [4., 3.], [-3., 3.]), (.12, .21, .36, .17)),
            (([-2., -4.], [3., -3.], [2., 4.], [-4., 2.]), (.27, .11, .19, .31)),
            (([-5., -1.], [2., -5.], [5., 2.], [-2., 5.]), (.15, .29, .13, .22)),
        ]
        for case, (centers, radii) in enumerate(scenes):
            with self.subTest(case=case):
                points = curve_scene(centers, radii)
                matcher, rng = self.initialized(points, 711 + case)
                accepted = []
                for index in range(40):
                    result = self.update_at(matcher, points, [0., 0.], rng, 2. + index * .1)
                    if result['accepted']:
                        accepted.append(result)
                self.assertGreaterEqual(len(accepted), 18)
                self.assertLess(np.max(np.linalg.norm([r['xy'] for r in accepted], axis=1)), .03)
                self.assertTrue(all(np.linalg.eigvalsh(r['covariance'])[0] > 0. for r in accepted))

    def test_straight_perpendicular_walls(self):
        points = np.concatenate([np.c_[np.linspace(-2., 2., 41), np.full(41, 3.)],
                                 np.c_[np.full(41, 3.), np.linspace(-2., 2., 41)]])
        matcher, rng = self.initialized(points)
        result = self.update_at(matcher, points, [.03, -.02], rng, 2.1)
        self.assertTrue(result['accepted'], result)
        self.assertLess(np.linalg.norm(result['xy'] - [.03, -.02]), .015)

    def test_single_wall_is_unobservable(self):
        points = np.c_[np.linspace(-2., 2., 81), np.full(81, 3.)]
        matcher, rng = self.initialized(points)
        result = self.update_at(matcher, points, [0., .02], rng, 2.1)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'], 'unobservable_translation')

    def test_lateral_creep_with_zero_wheel_motion_and_fixed_yaw(self):
        points = curve_scene(([-3., -2.], [4., -2.], [4., 3.], [-3., 3.]), (.12, .21, .36, .17))
        matcher, rng = self.initialized(points)
        last = None
        for index in range(1, 21):
            result = self.update_at(matcher, points, [0., index * .002], rng, 1.9 + index * .1)
            if result['accepted']:
                last = result
        self.assertIsNotNone(last)
        self.assertGreater(last['xy'][1], .025)
        self.assertLess(np.linalg.norm(last['xy'] - [0., .04]), .015)

    def test_reference_and_normals_stay_fixed_after_repeated_keyframes(self):
        points = curve_scene(([-3., -2.], [4., -2.], [4., 3.], [-3., 3.]), (.12, .21, .36, .17))
        matcher, rng = self.initialized(points)
        target, normals, means = matcher.target.copy(), matcher.normals.copy(), matcher.normal_means.copy()
        for _ in range(30):
            matcher.reference(points + rng.normal(0., .01, points.shape))
        np.testing.assert_array_equal(matcher.target, target)
        np.testing.assert_array_equal(matcher.normals, normals)
        np.testing.assert_array_equal(matcher.normal_means, means)

    def test_fixed_angle_rays_observe_noisy_curved_landmarks_and_real_creep(self):
        angles = np.linspace(-np.pi, np.pi, 720, endpoint=False)
        directions = np.c_[np.cos(angles), np.sin(angles)]
        centers = np.array([[-3., -2.], [4., -2.], [4., 3.], [-3., 3.]])
        radii = (.12, .21, .36, .17)
        rng = np.random.default_rng(881)

        def ranges_at(position):
            origin = np.asarray(position) + np.array([.14, 0.])
            ranges = np.full(len(angles), np.inf)
            for center, radius in zip(centers, radii):
                delta = center - origin
                along = directions @ delta
                discriminant = radius ** 2 - delta @ delta + along ** 2
                valid = (discriminant >= 0) & (along > 0)
                value = along[valid] - np.sqrt(discriminant[valid])
                ranges[valid] = np.minimum(ranges[valid], value)
            finite = np.isfinite(ranges)
            ranges[finite] += rng.normal(0., .01, finite.sum())
            return ranges

        matcher = LaserOdometry()
        for index in range(10):
            matcher.update(1. + index * .1, ranges_at([0., 0.]), angles, .05, 20., 0., 0., 0.)
        accepted = []
        for index in range(1, 21):
            result = matcher.update(1.9 + index * .1, ranges_at([0., index * .002]),
                                    angles, .05, 20., 0., 0., 0.)
            if result['accepted']:
                accepted.append(result)
        self.assertGreaterEqual(len(accepted), 9)
        self.assertGreater(accepted[-1]['xy'][1], .025)
        self.assertLess(np.linalg.norm(accepted[-1]['xy'] - [0., .04]), .015)

    def test_rejected_scan_clears_window_without_emitting_a_fresh_pose(self):
        points = curve_scene(([-3., -2.], [4., -2.], [4., 3.], [-3., 3.]), (.12, .21, .36, .17))
        matcher, rng = self.initialized(points)
        self.assertTrue(self.update_at(matcher, points, [0., 0.], rng, 2.1)['accepted'])
        valid = matcher.normal_valid.copy()
        matcher.normal_valid[:] = False
        rejected = self.update_at(matcher, points, [0., 0.], rng, 2.3)
        self.assertFalse(rejected['accepted'])
        self.assertNotIn('xy', rejected)
        self.assertEqual(len(matcher.fresh_window.rows), 0)
        matcher.normal_valid[:] = valid
        restored = self.update_at(matcher, points, [0., 0.], rng, 2.4)
        self.assertTrue(restored['accepted'])
        self.assertEqual(restored['window_samples'], 1)

    def test_early_missing_returns_guard_also_clears_pre_failure_window(self):
        points = curve_scene(([-3., -2.], [4., -2.], [4., 3.], [-3., 3.]), (.12, .21, .36, .17))
        matcher, rng = self.initialized(points)
        self.assertTrue(self.update_at(matcher, points, [0., 0.], rng, 2.1)['accepted'])
        rejected = matcher.update(2.2, np.full(len(points), np.inf), np.zeros(len(points)),
                                  .05, 20., 0., 0., 0.)
        self.assertFalse(rejected['accepted'])
        self.assertEqual(rejected['reason'], 'too_few_returns')
        self.assertEqual(len(matcher.fresh_window.rows), 0)
        for index, stamp in enumerate((2.3, 2.5, 2.7), 1):
            restored = self.update_at(matcher, points, [0., 0.], rng, stamp)
            self.assertTrue(restored['accepted'])
            self.assertEqual(restored['window_samples'], index)
            self.assertTrue(all(row[0] >= 2.3 for row in matcher.fresh_window.rows))



class FreshWindowTests(unittest.TestCase):
    def test_noise_reduction_uses_new_measurements(self):
        rng = np.random.default_rng(42)
        estimator = FreshWindowEstimate()
        raw, filtered = [], []
        for index in range(100):
            measurement = rng.normal(0, .004, 2)
            value = estimator.update(index * .1, measurement, np.eye(2) * .004 ** 2)
            self.assertEqual(value['newest_measurement_stamp'], index * .1)
            self.assertLessEqual(value['window_span_s'], .4 + 1e-9)
            raw.append(measurement)
            filtered.append(value['xy'])
        self.assertLess(np.var(filtered[10:]), .65 * np.var(raw[10:]))

    def test_real_lateral_creep_with_no_wheel_input_is_not_frozen(self):
        rng = np.random.default_rng(27)
        estimator = FreshWindowEstimate()
        outputs = []
        for index in range(21):
            position = np.array([0., index * .002])
            measurement = position + rng.normal(0, .001, 2)
            outputs.append(estimator.update(index * .1, measurement, np.eye(2) * .001 ** 2))
        self.assertGreater(outputs[-1]['xy'][1] - outputs[0]['xy'][1], .03)
        np.testing.assert_allclose(outputs[-1]['xy'], [0., .04], atol=.005)
        self.assertEqual(outputs[-1]['motion_model'], 'constant_velocity')

    def test_sudden_sensor_observed_translation_is_not_held_at_old_position(self):
        estimator = FreshWindowEstimate()
        for index in range(5):
            estimator.update(index * .1, [0., 0.], np.eye(2) * .001 ** 2)
        value = estimator.update(.5, [.04, .02], np.eye(2) * .001 ** 2)
        np.testing.assert_allclose(value['xy'], [.04, .02])
        self.assertEqual(value['motion_model'], 'current_fit_motion_boundary')

    def test_repeat_invalid_and_missing_new_measurement_cannot_emit_fresh_pose(self):
        estimator = FreshWindowEstimate()
        estimator.update(0., [0., 0.], np.eye(2) * .001 ** 2)
        for stamp, position in ((0., [0., 0.]), (.1, [float('nan'), 0.])):
            with self.assertRaises(ValueError):
                estimator.update(stamp, position, np.eye(2) * .001 ** 2)

    def test_clear_and_long_gap_forget_pre_failure_samples(self):
        estimator = FreshWindowEstimate()
        estimator.update(0., [0., 0.], np.eye(2) * .001 ** 2)
        self.assertEqual(estimator.update(1., [.03, .02], np.eye(2) * .001 ** 2)['window_samples'], 1)
        estimator.clear()
        self.assertEqual(estimator.update(2., [.04, .02], np.eye(2) * .001 ** 2)['window_samples'], 1)



class NoiseInlierFallbackTests(unittest.TestCase):

    def plane_fixture(self,x_count=10,y_count=10):
        matcher=LaserOdometry()
        xwall=np.c_[np.full(x_count,4.),np.linspace(-.2,.2,x_count)]
        ywall=np.c_[np.linspace(-.2,.2,y_count),np.full(y_count,3.)]
        matcher.target=np.r_[xwall,ywall];matcher.tree=cKDTree(matcher.target)
        matcher.normal_means=matcher.target.copy()
        matcher.normals=np.r_[np.tile([1.,0.],(x_count,1)),np.tile([0.,1.],(y_count,1))]
        matcher.normal_valid=np.ones(x_count+y_count,dtype=bool);matcher.stamp=1.
        return matcher

    def test_minority_wrong_correspondences_rescue_requires_actual_refit(self):
        matcher=self.plane_fixture();world=matcher.target.copy()
        world[6:10,0]+=.1;world[16:20,1]+=[.1,-.1,.1,-.1]
        result,reason=matcher.fit(world,np.zeros(2))
        self.assertIsNotNone(result,reason);self.assertTrue(result['rescued_registration'])
        self.assertGreater(2*result['inliers'],len(world))
        self.assertLess(np.linalg.norm(result['xy']),.002)
        self.assertGreater(result['all_correspondence_rmse_m'],.03)

    def test_rescue_rejects_a_minority_of_reliable_points(self):
        matcher=self.plane_fixture();world=matcher.target.copy()
        world[3:10,0]+=[.1,-.1,.1,-.1,.1,-.1,.1]
        world[13:20,1]+=[.1,-.1,.1,-.1,.1,-.1,.1]
        result,reason=matcher.fit(world,np.zeros(2))
        self.assertIsNone(result);self.assertEqual(reason,'insufficient_noise_inliers')

    def test_rescue_inliers_must_remain_two_dimensionally_observable(self):
        matcher=self.plane_fixture(12,8);world=matcher.target.copy()
        world[12:,1]+=[.1,-.1]*4
        result,reason=matcher.fit(world,np.zeros(2))
        self.assertIsNone(result);self.assertEqual(reason,'unobservable_translation')

    def test_rescue_never_exceeds_original_correction_jump_bound(self):
        matcher=self.plane_fixture();world=matcher.target.copy()
        world[:6,0]-=.105;world[6:10,0]+=.1;world[16:20,1]+=[.1,-.1,.1,-.1]
        result,reason=matcher.fit(world,np.zeros(2))
        self.assertIsNone(result);self.assertEqual(reason,'correction_jump')

    def test_ordinary_good_updates_match_frozen_pre_rescue_outputs(self):
        fixture=json.loads((Path(__file__).resolve().parent/'data/metric-good-updates-v1.json').read_text())
        points=curve_scene(([-3.,-2.],[4.,-2.],[4.,3.],[-3.,3.]),(.12,.21,.36,.17))
        matcher=LaserOdometry();rng=np.random.default_rng(fixture['rng_seed'])
        for index,expected in enumerate(fixture['rows']):
            ranges,angles=scan(points,[0.,max(0,index-10)*.0005],rng)
            result=matcher.update(expected['stamp'],ranges,angles,.05,20.,0.,0.,0.)
            self.assertEqual((result['accepted'],result['reason']),(expected['accepted'],expected['reason']))
            if result['accepted']:
                self.assertNotIn('rescued_registration',result)
                np.testing.assert_array_equal(result['xy'],expected['xy'])
                np.testing.assert_array_equal(result['covariance'],expected['covariance'])
                self.assertEqual(result['inliers'],expected['inliers'])


if __name__ == '__main__':
    unittest.main()
