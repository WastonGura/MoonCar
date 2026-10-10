"""Offline, read-only comparison of archived contacts with ground geometry.

This never starts ROS/Gazebo, changes a model, or feeds truth to a controller.
Evidence associations are diagnostic; normal differences do not prove a cause.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from r5_origin_analysis import rotation

CASES = ('03-pure_turn-flat-1000us-left-1', '14-pure_turn-ramp-1000us-left')


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read_obj(path):
    vertices, faces = [], []
    for line in Path(path).open():
        row = line.split()
        if row and row[0] == 'v':
            vertices.append([float(x) for x in row[1:4]])
        elif row and row[0] == 'f':
            if len(row) != 4:
                raise ValueError('Only archived triangular faces are supported')
            faces.append([int(x.split('/')[0]) - 1 for x in row[1:]])
    v, f = np.asarray(vertices, float), np.asarray(faces, int)
    if v.ndim != 2 or v.shape[1] != 3 or not np.isfinite(v).all() or f.ndim != 2 or f.shape[1] != 3 or f.min() < 0 or f.max() >= len(v):
        raise ValueError('Invalid mesh')
    return v, f


def surface_at(points, triangles, tolerance=1e-9):
    """Exact XY barycentric association; fail rather than invent a surface."""
    p, t = np.asarray(points, float), np.asarray(triangles, float)
    if p.ndim != 2 or p.shape[1] != 3 or not np.isfinite(p).all() or t.ndim != 3 or t.shape[1:] != (3, 3) or not np.isfinite(t).all():
        raise ValueError('Missing/nonfinite geometry')
    normals = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    if np.any(lengths <= 1e-12):
        raise ValueError('Degenerate face')
    normals /= lengths[:, None]
    if np.any(normals[:, 2] <= 1e-12):
        raise ValueError('Vertical/reversed ground face unsupported')
    chosen = np.full(len(p), -1, int)
    heights = np.full(len(p), np.nan)
    for i, face in enumerate(t):
        a = np.column_stack((face[1, :2] - face[0, :2], face[2, :2] - face[0, :2]))
        uv = (p[:, :2] - face[0, :2]) @ np.linalg.inv(a).T
        valid = (uv >= -tolerance).all(axis=1) & (uv.sum(axis=1) <= 1 + tolerance)
        height = face[0, 2] + uv @ (face[1:, 2] - face[0, 2])
        overlap = valid & (chosen >= 0)
        if np.any(abs(heights[overlap] - height[overlap]) > tolerance):
            raise ValueError('Ambiguous overlapping ground surfaces')
        use = valid & (chosen < 0)
        chosen[use] = i
        heights[use] = height[use]
    if np.any(chosen < 0):
        raise ValueError('Contact projection outside archived ground')
    return chosen, heights, normals[chosen]


def internal_edges(triangles):
    """Coordinate adjacency works with duplicated OBJ vertices; no mesh edit."""
    triangles = np.asarray(triangles, float)
    edges = {}
    for face in triangles:
        normal = np.cross(face[1] - face[0], face[2] - face[0])
        length = np.linalg.norm(normal)
        if length <= 1e-12:
            raise ValueError('Degenerate face')
        normal /= length
        for i, j in ((0, 1), (1, 2), (2, 0)):
            # Quantization is only an adjacency key, never changed geometry.
            key = tuple(sorted((tuple(np.round(face[i], 12)), tuple(np.round(face[j], 12)))))
            edges.setdefault(key, []).append(normal)
    return np.asarray([key for key, normals in edges.items() if len(normals) == 2 and
                       np.linalg.norm(normals[0] - normals[1]) <= 1e-12], float).reshape(-1, 2, 3)


def edge_distance(points, edges):
    p, e = np.asarray(points, float)[:, :2], np.asarray(edges, float)[:, :, :2]
    if not len(e):
        raise ValueError('No internal edges available')
    best = np.full(len(p), np.inf)
    for a, b in e:
        delta = b - a
        if np.dot(delta, delta) <= 1e-24:
            raise ValueError('Zero length edge')
        u = np.clip((p - a) @ delta / np.dot(delta, delta), 0, 1)
        best = np.minimum(best, np.linalg.norm(p - a - u[:, None] * delta, axis=1))
    return best


def classified_edges(vertices, faces):
    """Separate true index adjacency from coincident, disconnected OBJ edges."""
    v, f = np.asarray(vertices, float), np.asarray(faces, int)
    grouped = {}
    for face in f:
        triangle = v[face]
        normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
        normal /= np.linalg.norm(normal)
        for i, j in ((0, 1), (1, 2), (2, 0)):
            coordinates = tuple(sorted((tuple(np.round(triangle[i], 12)), tuple(np.round(triangle[j], 12)))))
            indices = tuple(sorted((int(face[i]), int(face[j]))))
            grouped.setdefault(coordinates, []).append((indices, normal))
    result = dict(shared_indices=[], duplicated_indices=[])
    for edge, values in grouped.items():
        if len(values) == 2 and np.linalg.norm(values[0][1] - values[1][1]) <= 1e-12:
            key = 'shared_indices' if values[0][0] == values[1][0] else 'duplicated_indices'
            result[key].append(edge)
    return {key: np.asarray(value, float).reshape(-1, 2, 3) for key, value in result.items()}


def normal_angle(actual, geometric):
    a, g = np.asarray(actual, float), np.asarray(geometric, float)
    if a.shape != g.shape or a.ndim != 2 or a.shape[1] != 3 or not np.isfinite(a).all() or not np.isfinite(g).all():
        raise ValueError('Missing/nonfinite normal')
    lengths = np.linalg.norm(a, axis=1)
    if np.any(abs(lengths - 1) > 1e-6) or np.any(abs(np.linalg.norm(g, axis=1) - 1) > 1e-6):
        raise ValueError('Non-unit normal')
    return np.arctan2(np.linalg.norm(np.cross(a, g), axis=1), np.sum(a * g, axis=1))


def identity_ground(world, name):
    model = ET.parse(world).find(f"world/model[@name='{name}']")
    if model is None:
        raise ValueError('Ground missing')
    for item in (model, model.find('link'), model.find('link/collision')):
        pose = item.find('pose')
        if pose is not None and (pose.get('relative_to') or any(float(x) for x in pose.text.split())):
            raise ValueError('Non-identity archived ground transform unsupported')
    collision = model.find('link/collision')
    scale = collision.findtext('geometry/mesh/scale', '1 1 1')
    if [float(x) for x in scale.split()] != [1., 1., 1.]:
        raise ValueError('Unexpected archived mesh scale')
    return collision


def plate_top(plate):
    """Compose model/link/collision poses; never discard a link offset."""
    translation, rotation_matrix = np.zeros(3), np.eye(3)
    for item in (plate, plate.find('link'), plate.find('link/collision')):
        element = item.find('pose')
        if element is not None and element.get('relative_to'):
            raise ValueError('Named pose frame requires an explicit frame graph')
        pose = np.asarray([float(x) for x in item.findtext('pose', '0 0 0 0 0 0').split()])
        if pose.shape != (6,) or not np.isfinite(pose).all():
            raise ValueError('Invalid plate pose')
        roll, pitch, yaw = pose[3:]
        cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
        local = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                          [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                          [-sp, cp*sr, cp*cr]])
        translation += rotation_matrix @ pose[:3]
        rotation_matrix = rotation_matrix @ local
    if not np.allclose(rotation_matrix[:, 2], [0, 0, 1], rtol=0, atol=1e-12):
        raise ValueError('Tilted plate unsupported by this archived comparison')
    size = np.asarray([float(x) for x in plate.findtext('link/collision/geometry/box/size').split()])
    if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
        raise ValueError('Invalid plate size')
    return float(translation[2] + size[2] / 2)


def run(root, evidence, output):
    root, evidence, output = Path(root), Path(evidence), Path(output)
    output.mkdir(exist_ok=False)
    archive = {n.replace('\\', '/'): v for n, v in json.loads((evidence / 'archive-manifest.json').read_text())['files'].items()}
    frozen = json.loads((evidence / 'campaign/freeze.json').read_text())
    hashes, results = {}, []
    def verified(path, expected):
        actual = digest(path)
        if actual != expected:
            raise ValueError('Archived input changed: ' + str(path))
        hashes[str(path)] = actual
    resource = 'models/terrain_v2/ramp/regolith_passable.obj'
    mesh = root / 'ros2_ws/src/mooncar_gazebo' / resource
    verified(mesh, frozen['resources'][resource])
    verified(root / 'scripts/r5_origin_analysis.py', frozen['source']['scripts/r5_origin_analysis.py'])
    for name in CASES:
        folder = evidence / 'campaign' / name
        for filename in ('result.json', 'world.sdf', 'physics-steps.csv', name + '-contact-steps.csv'):
            rel = 'campaign/' + name + '/' + filename
            verified(folder / filename, archive[rel]['sha256'])
        r = json.loads((folder / 'result.json').read_text())
        motion = r['schedule']['motion']; first, last = motion['before']['iteration'], motion['after']['iteration']
        physics = np.genfromtxt(folder / 'physics-steps.csv', delimiter=',', names=True)
        rows = np.genfromtxt(folder / (name + '-contact-steps.csv'), delimiter=',', names=True)
        active = physics[(physics['iteration'] > first) & (physics['iteration'] <= last)]
        rows = rows[(rows['iteration'] > first) & (rows['iteration'] <= last)]
        if not np.array_equal(active['iteration'], np.arange(first + 1, last + 1)) or not np.isin(rows['state'], [0, 1]).all():
            raise ValueError('Missing exact-step/source qualification')
        index = (rows['iteration'] - first - 1).astype(int)
        if not np.array_equal(rows['t_ns'], np.rint(active['t'][index] * 1e9)):
            raise ValueError('Time/frame association mismatch')
        keys = rows['iteration'] * 4 + rows['wheel']
        if not np.array_equal(np.unique(keys), (np.arange(first + 1, last + 1)[:, None] * 4 + np.arange(4)).ravel()):
            raise ValueError('Missing wheel/step')
        positive = rows[rows['state'] == 1]
        points = np.column_stack([positive[k] for k in ('x', 'y', 'z')])
        normals = np.column_stack([positive[k] for k in ('nx', 'ny', 'nz')])
        if np.any(positive['extra_complete'] != 1) or not np.isfinite(points).all():
            raise ValueError('Missing contact data')
        triangles = None
        if 'ramp' in name:
            collision = identity_ground(folder / 'world.sdf', 'regolith_passable_ground')
            if collision.findtext('geometry/mesh/uri') != '/source/ros2_ws/src/mooncar_gazebo/' + resource:
                raise ValueError('Actual archived mesh URI differs')
            v, f = read_obj(mesh); triangles = v[f]
            lo, hi = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
            select = ((triangles[:, :, :2].min(axis=1) <= hi).all(axis=1) &
                      (triangles[:, :, :2].max(axis=1) >= lo).all(axis=1))
            triangles = triangles[select]
            faces, heights, expected = surface_at(points, triangles)
            edges = internal_edges(triangles)
            distance = edge_distance(points, edges)
            categories = classified_edges(v, f[select])
        else:
            plate = ET.parse(folder / 'world.sdf').find("world/model[@name='plate']")
            height = plate_top(plate)
            if abs(height) > 1e-12:
                raise ValueError('Unexpected plate top height')
            collision = plate.find('link/collision')
            expected = np.tile([0., 0., 1.], (len(points), 1))
            heights = np.full(len(points), height); distance = None
        angles = normal_angle(normals, expected)
        point_index = (positive['iteration'] - first - 1).astype(int)
        step_angles = np.full(len(active), -np.inf)
        np.maximum.at(step_angles, point_index, angles)
        step_angles[~np.isfinite(step_angles)] = np.nan
        body = np.column_stack([active[k] for k in ('x', 'y')])
        # Relative origin is the actual last parked step, not the first moving pose.
        start_row = physics[physics['iteration'] == first][0]
        relative = body - [start_row['x'], start_row['y']]
        counts = np.bincount(point_index, minlength=len(active))
        with (output / (name + '-steps.csv')).open('w') as stream:
            stream.write('iteration,t,dx_m,dy_m,contact_points,max_normal_difference_rad\n')
            for i, row in enumerate(active):
                stream.write(f"{int(row['iteration'])},{row['t']:.17g},{relative[i,0]:.17g},{relative[i,1]:.17g},{counts[i]},{step_angles[i]:.17g}\n")
        tilted = angles > 1e-6 # Diagnostic reporting resolution, not a performance gate.
        report = dict(name=name, physical_round_passed=r['passed'], source_point_rows=len(positive),
            motion_steps=len(active), all_contact_normals_unit=True,
            maximum_normal_difference_rad=float(np.max(angles)),
            maximum_normal_difference_deg=float(np.degrees(np.max(angles))),
            tilted_point_count=int(tilted.sum()), tilted_step_count=int((step_angles > 1e-6).sum()),
            point_count_histogram={str(int(k)): int(v) for k, v in zip(*np.unique(counts, return_counts=True))},
            maximum_ground_projection_distance_m=float(np.max(abs(points[:, 2] - heights))),
            maximum_center_displacement_m=float(np.max(np.linalg.norm(relative, axis=1))),
            actual_other_collision_ids=np.unique(positive['other_collision']).astype(int).tolist(),
            other_collision_scoped_name='Not in archived ECS identity registry; association is geometric projection',
            mu=float(collision.findtext('surface/friction/ode/mu')),
            mu2=float(collision.findtext('surface/friction/ode/mu2')),
            source_empty_forces_not_imputed=True)
        if triangles is not None:
            report.update(intersecting_faces=len(triangles), all_local_vertex_heights_zero=bool(np.all(triangles[:, :, 2] == 0)),
                local_triangle_normal_range={str(i): [float(expected[:, i].min()), float(expected[:, i].max())] for i in range(3)},
                coplanar_internal_edges=len(edges),
                median_seam_distance_all_points_m=float(np.median(distance)),
                median_seam_distance_tilted_points_m=float(np.median(distance[tilted])) if tilted.any() else None,
                tilted_points_within_1mm_seam_fraction=float(np.mean(distance[tilted] <= .001)) if tilted.any() else None)
            report['seam_categories'] = {}
            distances = {}
            for category, segments in categories.items():
                if not len(segments):
                    report['seam_categories'][category] = dict(edges=0)
                    continue
                d = edge_distance(points, segments); distances[category] = d
                report['seam_categories'][category] = dict(edges=len(segments),
                    median_tilted_distance_m=float(np.median(d[tilted])),
                    tilted_within_1mm_fraction=float(np.mean(d[tilted] <= .001)))
            if len(distances) == 2:
                a, b = distances['shared_indices'][tilted], distances['duplicated_indices'][tilted]
                report['tilted_nearest_shared_fraction'] = float(np.mean(a < b - 1e-12))
                report['tilted_nearest_duplicated_fraction'] = float(np.mean(b < a - 1e-12))
                report['tilted_nearest_tie_fraction'] = float(np.mean(abs(a - b) <= 1e-12))
        results.append(report)
    proof = dict(results=results, input_hashes=hashes, new_simulations=0, old_records_changed=False,
                 mechanism_confirmed=False, fix_verified=False, production_changed=False)
    (output / 'analysis.json').write_text(json.dumps(proof, indent=2) + '\n')
    print(json.dumps(results, indent=2))
    return proof


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', required=True); p.add_argument('--evidence', required=True); p.add_argument('--output', required=True)
    a = p.parse_args(); run(a.root, a.evidence, a.output)
