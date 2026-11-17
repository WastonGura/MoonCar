"""Offline reference from the V3 triangles; never imported by robot control.

The 5 cm samples are pixel centres. A 25 cm evaluation cell contains 5x5
samples, including its exact centre. Material partitions are one ground
surface; exposed rock/ground mixtures are marked separately.
"""
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
from experiment_scenes import PACKAGE, V3_WORLD, _obj, _transform, _resource

RESOLUTION = .25
FINE = .05
LOW = -15.
N = 120


def ground_at(axis, heights, x, y):
    """Exact SW-SE-NE / SW-NE-NW triangular interpolation, not bilinear."""
    step = float(axis[1]-axis[0])
    u, v = (np.asarray(x)-axis[0])/step, (np.asarray(y)-axis[0])/step
    ix = np.clip(np.floor(u).astype(int), 0, len(axis)-2)
    iy = np.clip(np.floor(v).astype(int), 0, len(axis)-2)
    a, b = u-ix, v-iy
    sw, se = heights[iy, ix], heights[iy, ix+1]
    nw, ne = heights[iy+1, ix], heights[iy+1, ix+1]
    return np.where(a >= b, sw+(se-sw)*a+(ne-se)*b,
                    sw+(ne-nw)*a+(nw-sw)*b)


def overlay_triangles(z, owner, axis, triangles, identity):
    """Vertical top envelope of actual transformed collision triangles."""
    for triangle in triangles:
        a, b, c = triangle
        ab, ac = b-a, c-a
        determinant = ab[0]*ac[1]-ab[1]*ac[0]
        if abs(determinant) < 1e-12:
            continue
        low, high = triangle[:, :2].min(axis=0), triangle[:, :2].max(axis=0)
        ix0, iy0 = np.searchsorted(axis, low, side='left')
        ix1, iy1 = np.searchsorted(axis, high, side='right')
        if ix0 == ix1 or iy0 == iy1:
            continue
        xx, yy = np.meshgrid(axis[ix0:ix1]-a[0], axis[iy0:iy1]-a[1])
        u = (xx*ac[1]-yy*ac[0])/determinant
        v = (ab[0]*yy-ab[1]*xx)/determinant
        zz = a[2]+u*ab[2]+v*ac[2]
        view = z[iy0:iy1, ix0:ix1]
        mask = (u >= -1e-9) & (v >= -1e-9) & (u+v <= 1+1e-9) & (zz > view+1e-9)
        view[mask] = zz[mask]
        owner[iy0:iy1, ix0:ix1][mask] = identity


def reduce_reference(fine_z, owner):
    blocks = fine_z.reshape(N, 5, N, 5).transpose(0, 2, 1, 3)
    labels = owner.reshape(N, 5, N, 5).transpose(0, 2, 1, 3)
    offsets = (np.arange(5)-2)*FINE
    gx = np.sum(blocks*offsets[None, None, None, :], axis=(2, 3))/(5*np.sum(offsets**2))
    gy = np.sum(blocks*offsets[None, None, :, None], axis=(2, 3))/(5*np.sum(offsets**2))
    intercept = blocks.mean(axis=(2, 3))
    predicted = intercept[:, :, None, None]+gx[:, :, None, None]*offsets[None, None, None, :]+gy[:, :, None, None]*offsets[None, None, :, None]
    return dict(z=blocks[:, :, 2, 2], plane_z=intercept, gx=gx, gy=gy,
                slope=np.degrees(np.arctan(np.hypot(gx, gy))),
                roughness=np.sqrt(np.mean((blocks-predicted)**2, axis=(2, 3))),
                mixed=labels.max(axis=(2, 3)) != labels.min(axis=(2, 3)))


def build(output):
    folder = PACKAGE/'models/terrain_v3/survey'
    grid = np.load(folder/'height_grid.npz')
    axis = LOW+(np.arange(N*5)+.5)*FINE
    xx, yy = np.meshgrid(axis, axis)
    fine_z = ground_at(grid['x'], grid['z'], xx, yy)
    owner = np.zeros(fine_z.shape, dtype=np.int16)
    manifest = json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
    ground_files = set(manifest['mesh_sha256'])-{'surface_pebbles.obj'}
    inputs = {V3_WORLD, folder/'height_grid.npz', folder/'manifest.json'}
    identity = 0
    names = {}
    ground_max_vertex_error = 0.
    for model in ET.parse(V3_WORLD).findall('./world/model'):
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                mesh = collision.find('geometry/mesh')
                if mesh is None:
                    raise ValueError('Unexpected collision geometry')
                path = _resource(mesh.findtext('uri'))
                inputs.add(path)
                vertices, faces = _obj(path)
                scale = np.asarray(list(map(float, mesh.findtext('scale', '1 1 1').split())))
                matrix = _transform(model) @ _transform(link) @ _transform(collision)
                vertices = (vertices*scale) @ matrix[:3, :3].T+matrix[:3, 3]
                if path.parent == folder and path.name in ground_files:
                    expected = ground_at(grid['x'], grid['z'], vertices[:, 0], vertices[:, 1])
                    maximum = float(np.max(np.abs(vertices[:, 2]-expected)))
                    if maximum > 1e-6:
                        raise ValueError('Height grid does not reproduce actual collision vertices')
                    ground_max_vertex_error = max(ground_max_vertex_error, maximum)
                    continue
                identity += 1
                names[identity] = model.get('name')
                overlay_triangles(fine_z, owner, axis, vertices[faces], identity)
    output.mkdir(parents=True, exist_ok=True)
    reference = reduce_reference(fine_z, owner)
    np.savez_compressed(output/'reference.npz', **reference, fine_z=fine_z, fine_owner=owner)
    metadata = dict(bounds=[-15, -15, 15, 15], frame='map/world (declared initial placement)',
                    resolution=RESOLUTION, fine_sample_m=FINE,
                    reference='Actual V3 triangular ground and highest rock/pebble collision surface at XY',
                    mixed_policy='5x5 samples with different exposed surface owners; material changes are not mixed',
                    sample_policy='Unregistered cell-centre estimated plane height minus exact reference centre height',
                    alignment='No ICP, no truth pose substitution, no fitted vertical offset, no filling',
                    ground_grid_vs_collision_max_vertex_error_m=ground_max_vertex_error,
                    exposed_rock_cells=int(np.count_nonzero(owner)), rock_models=names,
                    limitations=['2.5D comparison cannot represent vertical or multiple surfaces',
                                 'Mixed detection samples at 5 cm and can miss sub-sample features',
                                 'Single-surface slope reference is a 5x5 least-squares fit, not a sensor sample fit'],
                    inputs={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(inputs)})
    (output/'reference-metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n', encoding='utf-8', newline='\n')
    return reference


def compare(payload, reference):
    """No cropping of observed totals; out-of-reference cells remain unscored."""
    if payload['resolution'] != RESOLUTION:
        raise ValueError('Mismatched reconstruction resolution')
    cells = {(c['ix'], c['iy']):c for c in payload['cells']}
    if len(cells) != len(payload['cells']):
        raise ValueError('Duplicate map cells')
    observed = np.zeros((N, N), bool)
    valid = np.zeros((N, N), bool)
    reconstructed = np.full((N, N), np.nan)
    slope = np.full((N, N), np.nan)
    outside = outside_valid = 0
    for (ix, iy), c in cells.items():
        x, y = ix+60, iy+60
        fitted = c.get('fit_state') == 'valid' and c.get('valid') is True
        if not (0 <= x < N and 0 <= y < N):
            outside += 1
            outside_valid += int(fitted)
            continue
        observed[y, x] = True
        if fitted:
            if c.get('z') is None or not np.isfinite(c['z']):
                raise ValueError('Valid cell has no finite height')
            valid[y, x] = True
            reconstructed[y, x] = c['z']
            slope[y, x] = c['slope']
    difference = reconstructed-reference['z']
    single = valid & ~reference['mixed']
    def errors(values):
        return dict(count=int(values.size), mae_m=float(np.mean(np.abs(values))),
                    rmse_m=float(np.sqrt(np.mean(values**2))),
                    p95_abs_m=float(np.percentile(np.abs(values), 95)),
                    signed_mean_m=float(np.mean(values))) if values.size else dict(count=0, mae_m=None, rmse_m=None, p95_abs_m=None, signed_mean_m=None)
    stats = dict(frames=payload['frames'], stamp=payload['stamp'], total_observed_m2=len(cells)*RESOLUTION**2,
                 valid_surface_m2=(int(valid.sum())+outside_valid)*RESOLUTION**2,
                 observed_inside_reference_m2=int(observed.sum())*RESOLUTION**2,
                 invalid_inside_reference_m2=int((observed & ~valid).sum())*RESOLUTION**2,
                 outside_reference_cells=outside, outside_valid_cells=outside_valid,
                 mixed_valid_cells=int((valid & reference['mixed']).sum()),
                 single_surface=errors(difference[single]),
                 all_valid_including_mixed=errors(difference[valid]),
                 slope_mae_degrees=float(np.mean(np.abs((slope-reference['slope'])[single]))) if single.any() else None)
    return dict(observed=observed, valid=valid, z=reconstructed, error=difference, single=single), stats


if __name__ == '__main__':
    build(Path(__file__).resolve().parent)
