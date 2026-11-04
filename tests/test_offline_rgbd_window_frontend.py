"""Identity and planned-output guards specific to the offline prototype."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from offline_rgbd_window.frontend import TrackFrontend


def original_spaced(frontend, pixels, order, initial=()):
    """Unchanged f455 greedy oracle; it intentionally rebuilds occupied rows."""
    kept, occupied = [], [np.asarray(p, dtype=float) for p in initial]
    if len(occupied) >= frontend.measurements._orb.getMaxFeatures():
        return np.empty(0, dtype=int)
    for index in order:
        if occupied and np.min(np.sum((np.asarray(occupied)-pixels[index])**2, axis=1)) < frontend.MIN_TRACK_SPACING_PX**2:
            continue
        kept.append(index)
        occupied.append(pixels[index])
        if len(occupied) >= frontend.measurements._orb.getMaxFeatures():
            break
    return np.asarray(kept, dtype=int)


def table(first_id=0):
    u, v = np.meshgrid(np.linspace(20, 300, 12), np.linspace(20, 220, 8))
    pixels = np.column_stack((u.ravel(), v.ravel()))
    return dict(pixels=pixels, points=np.column_stack((pixels/100., np.ones(len(pixels)))),
                covariance=np.repeat((.003**2*np.eye(3))[None], len(pixels), axis=0),
                measured_roi=np.array([0, 0, 320, 240]), image_shape=(240, 320),
                track_ids=np.arange(first_id, first_id+len(pixels)))


class FrontendContracts(unittest.TestCase):
    def test_repeated_descriptors_do_not_invent_identity(self):
        frontend = TrackFrontend()
        repeated = dict(descriptors=np.zeros((30, 32), dtype=np.uint8))
        self.assertEqual(len(frontend._pairs(repeated, repeated)), 0)

    def test_disconnected_new_view_cannot_pass_as_continuing_map(self):
        frontend = TrackFrontend()
        first = table()
        chosen, diagnostics = frontend.select(first)
        self.assertTrue(diagnostics['adequate'])
        frontend.commit(first, chosen)
        _, diagnostics = frontend.select(table(1000))
        self.assertFalse(diagnostics['adequate'])
        self.assertEqual(diagnostics['connected'], 0)

    def test_full_identity_table_and_submitted_landmarks_remain_distinct(self):
        frontend = TrackFrontend(max_observations=48)
        first = table()
        chosen, _ = frontend.select(first)
        self.assertEqual(len(chosen), 48)
        frontend.commit(first, chosen)
        _, ids = frontend.keyframes[-1]
        self.assertEqual(len(ids), len(first['points']))
        _, diagnostics = frontend.select(first)
        self.assertTrue(diagnostics['adequate'])
        self.assertGreaterEqual(diagnostics['connected'], 24)
        self.assertLessEqual(diagnostics['selected'], 48)


class SpacingEquivalenceContracts(unittest.TestCase):
    def test_invalid_order_indices_keep_numpy_errors_instead_of_wrapping(self):
        pixels = np.array([[0., 0.], [4., 0.], [8., 0.]])
        frontend = TrackFrontend()
        for order in ([3], [-4], [1.0], [np.float32(0.)]):
            with self.subTest(order=order):
                with self.assertRaises(IndexError):
                    original_spaced(frontend, pixels, order)
                with self.assertRaises(IndexError):
                    frontend._spaced(pixels, order)

    def test_batched_neighbors_preserve_large_budget_and_initial_dtype_promotion(self):
        rng = np.random.default_rng(117)
        for dtype in (np.float32, np.float64):
            pixels = rng.uniform(-10., 110., (1500, 2)).astype(dtype)
            for initial in ((), rng.uniform(-10., 110., (27, 2)).astype(np.float32)):
                frontend = TrackFrontend()
                order = rng.permutation(len(pixels))
                np.testing.assert_array_equal(frontend._spaced(pixels, order, initial),
                    original_spaced(frontend, pixels, order, initial))

    def test_axis_box_is_only_broad_phase_and_strict_euclidean_edges_decide(self):
        for dtype in (np.float32, np.float64):
            four = dtype(4.)
            pixels = np.array([[0., 0.], [np.nextafter(four, dtype(0.)), 0.],
                [4., 0.], [np.nextafter(four, dtype(10.)), 0.], [-4., 0.], [0., 4.],
                [3.99, 3.99], [-.01, -.01], [-4.01, -4.01], [8., 0.]], dtype=dtype)
            for order in (list(range(len(pixels))), list(reversed(range(len(pixels)))),
                          [0, -1, 0, 2, -1, 4, 5, 6]):
                frontend = TrackFrontend()
                np.testing.assert_array_equal(frontend._spaced(pixels, order),
                    original_spaced(frontend, pixels, order))

    def test_nonfinite_unusual_dtype_and_integer_overflow_keep_original_scan(self):
        cases = [np.array([[0, 0], [4, 0], [127, 127]], np.int8),
                 np.array([[0., 0.], [np.nan, 1.], [np.inf, 1.], [4., 0.]]),
                 np.array([[0., 0.], [4., 0.]], np.float16), np.zeros((3, 3))]
        with np.errstate(all='ignore'):
            for pixels in cases:
                frontend = TrackFrontend()
                np.testing.assert_array_equal(frontend._spaced(pixels, range(len(pixels))),
                    original_spaced(frontend, pixels, range(len(pixels))))
            pixels = np.array([[0., 0.], [3., 0.], [4., 0.]])
            initial = [[np.nan, 1.]]
            frontend = TrackFrontend()
            np.testing.assert_array_equal(frontend._spaced(pixels, range(len(pixels)), initial),
                original_spaced(frontend, pixels, range(len(pixels)), initial))

    def test_random_order_initial_occupancy_and_dtypes_match_original_exactly(self):
        rng = np.random.default_rng(219)
        for dtype in (np.float32, np.float64, np.int32):
            for initial_count in (0, 1, 17, 71):
                with self.subTest(dtype=dtype, initial_count=initial_count):
                    frontend = TrackFrontend();frontend.measurements._orb.setMaxFeatures(71)
                    pixels = (rng.random((220, 2))*100).astype(dtype)
                    initial = (rng.random((initial_count, 2))*100).astype(np.float32)
                    before = pixels.copy();occupied = initial.copy()
                    order = rng.permutation(len(pixels))
                    np.testing.assert_array_equal(frontend._spaced(pixels, order, initial),
                        original_spaced(frontend, pixels, order, initial))
                    np.testing.assert_array_equal(pixels, before)
                    np.testing.assert_array_equal(initial, occupied)

    def test_strict_four_pixel_boundary_duplicates_and_reverse_order_are_unchanged(self):
        for dtype in (np.float32, np.float64):
            for reverse in (False, True):
                with self.subTest(dtype=dtype, reverse=reverse):
                    frontend = TrackFrontend()
                    below = np.nextafter(dtype(4.), dtype(0.))
                    above = np.nextafter(dtype(4.), dtype(np.inf))
                    pixels = np.array([[0., 0.], [below, 0.], [4., 0.], [above, 0.],
                                       [8., 0.], [8., 0.], [12., 0.]], dtype=dtype)
                    order = np.arange(len(pixels))[::-1] if reverse else np.arange(len(pixels))
                    for initial in ((), np.array([[0., 0.]], dtype=dtype)):
                        np.testing.assert_array_equal(frontend._spaced(pixels, order, initial),
                            original_spaced(frontend, pixels, order, initial))
        result = TrackFrontend()._spaced(np.array([[0., 0.], [4., 0.], [8., 0.]]), range(3))
        np.testing.assert_array_equal(result, [0, 1, 2])

    def test_capacity_and_empty_inputs_preserve_original_stop_and_index_dtype(self):
        frontend = TrackFrontend();frontend.measurements._orb.setMaxFeatures(3)
        pixels = np.array([[0., 0.], [4., 0.], [8., 0.], [12., 0.]])
        for initial in ((), [[-4., 0.]], [[-4., 0.], [-8., 0.], [-12., 0.]]):
            got = frontend._spaced(pixels, range(4), initial)
            np.testing.assert_array_equal(got, original_spaced(frontend, pixels, range(4), initial))
            self.assertEqual(got.dtype, np.dtype(int))
        got = frontend._spaced(np.empty((0, 2)), ())
        self.assertEqual(got.shape, (0,))


if __name__ == '__main__':
    unittest.main()
