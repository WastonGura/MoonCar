"""Analytic checks for offline geometry association, never physics acceptance."""
import sys
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts'))
from audit_r5_ramp_contact import surface_at, internal_edges, edge_distance, normal_angle, identity_ground, plate_top, classified_edges
from prepare_r5_contact_geometry import make_mesh, replace_collision


class RampContactAuditTests(unittest.TestCase):
    def setUp(self):
        self.triangles = np.array([[[0, 0, 0], [1, 0, 0], [1, 1, 0]],
                                   [[0, 0, 0], [1, 1, 0], [0, 1, 0]]], float)

    def test_both_triangles_and_shared_edge_project_to_plane(self):
        p = np.array([[.7, .2, -.00001], [.2, .7, .00003], [.5, .5, 0]])
        chosen, heights, normals = surface_at(p, self.triangles)
        np.testing.assert_array_equal(chosen, [0, 1, 0])
        np.testing.assert_array_equal(heights, [0, 0, 0])
        np.testing.assert_array_equal(normals, np.tile([0, 0, 1], (3, 1)))

    def test_sloped_face_does_not_become_horizontal(self):
        t = self.triangles.copy(); t[:, :, 2] = t[:, :, 0] * .1
        _, h, n = surface_at([[.7, .2, 0]], t)
        self.assertAlmostEqual(h[0], .07)
        np.testing.assert_allclose(n[0], np.array([-.1, 0, 1]) / np.sqrt(1.01))

    def test_outside_and_nonfinite_points_fail(self):
        for p in ([[2, 2, 0]], [[float('nan'), 0, 0]]):
            with self.assertRaises(ValueError): surface_at(p, self.triangles)

    def test_degenerate_or_reversed_face_fails(self):
        for t in (np.zeros((1, 3, 3)), self.triangles[:, ::-1]):
            with self.assertRaises(ValueError): surface_at([[.5, .5, 0]], t)

    def test_overlapping_surfaces_are_not_silently_selected(self):
        shifted = self.triangles.copy(); shifted[:, :, 2] += 1
        with self.assertRaises(ValueError): surface_at([[.5, .5, 0]], np.concatenate([self.triangles, shifted]))

    def test_coordinate_adjacency_ignores_obj_vertex_duplication(self):
        edges = internal_edges(self.triangles)
        self.assertEqual(edges.shape, (1, 2, 3))
        np.testing.assert_allclose(edge_distance([[.5, .5, 0], [0, 1, 0]], edges), [0, 1 / np.sqrt(2)])

    def test_non_coplanar_edge_is_not_reported_as_internal_flat_seam(self):
        t = self.triangles.copy(); t[1, 2, 2] = .1
        self.assertEqual(len(internal_edges(t)), 0)

    def test_shared_and_coincident_disconnected_edges_are_distinguished(self):
        v = self.triangles.reshape(-1, 3)
        a = classified_edges(v, [[0, 1, 2], [3, 4, 5]])
        self.assertEqual(len(a['duplicated_indices']), 1)
        self.assertEqual(len(a['shared_indices']), 0)
        b = classified_edges([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], [[0, 1, 2], [0, 2, 3]])
        self.assertEqual(len(b['shared_indices']), 1)
        self.assertEqual(len(b['duplicated_indices']), 0)

    def test_normal_angle_known_and_unknown(self):
        self.assertAlmostEqual(normal_angle([[0, 0, 1]], [[0, 0, 1]])[0], 0)
        self.assertAlmostEqual(normal_angle([[1, 0, 0]], [[0, 0, 1]])[0], np.pi / 2)
        for a in ([[0, 0, 2]], [[float('nan'), 0, 1]]):
            with self.assertRaises(ValueError): normal_angle(a, [[0, 0, 1]])

    def test_world_transform_is_not_ignored(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'world.sdf'
            p.write_text('<sdf><world><model name="ground"><pose>0 0 1 0 0 0</pose><link><collision/></link></model></world></sdf>')
            with self.assertRaises(ValueError): identity_ground(p, 'ground')

    def test_plate_link_and_collision_offsets_are_composed(self):
        p = ET.fromstring('<model><pose>0 0 1 0 0 0</pose><link><pose>0 0 -1.1 0 0 0</pose><collision><pose>0 0 .05 0 0 0</pose><geometry><box><size>40 40 .1</size></box></geometry></collision></link></model>')
        self.assertAlmostEqual(plate_top(p), 0)

    def test_tilted_or_named_frame_plate_fails(self):
        for pose in ('<pose>0 0 0 .1 0 0</pose>', '<pose relative_to="unknown">0 0 0 0 0 0</pose>'):
            p = ET.fromstring('<model>'+pose+'<link><collision><geometry><box><size>40 40 .1</size></box></geometry></collision></link></model>')
            with self.assertRaises(ValueError): plate_top(p)

    def test_both_mesh_diagonals_have_identical_horizontal_support(self):
        from audit_r5_ramp_contact import read_obj
        with tempfile.TemporaryDirectory() as d:
            for flip in (False, True):
                p = Path(d)/str(flip)
                make_mesh(p, flip, extent=1, cell=.25)
                v, f = read_obj(p)
                self.assertEqual(v.shape, (64, 3)); self.assertEqual(f.shape, (32, 3))
                _, h, n = surface_at([[.1, .2, .05]], v[f])
                self.assertAlmostEqual(h[0], .05)
                np.testing.assert_array_equal(n, [[0, 0, 1]])

    def test_fixture_replacement_preserves_vehicle_and_friction(self):
        a = ET.fromstring('<sdf><world><model name="plate"><link><collision><geometry><box/></geometry><surface><friction>same</friction></surface></collision><visual>same</visual></link></model><model name="mooncar"><joint/><plugin/><sensor/></model></world></sdf>')
        b = replace_collision(a, '/probe/mesh.obj')
        for path in ("world/model[@name='mooncar']", "world/model[@name='plate']/link/collision/surface", "world/model[@name='plate']/link/visual"):
            self.assertEqual(ET.tostring(a.find(path)), ET.tostring(b.find(path)))


if __name__ == '__main__': unittest.main()
