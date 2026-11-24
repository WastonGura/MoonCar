"""Read-only seam audit of written V2 collision triangles, not display shading."""
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
from experiment_scenes import _v2_geometry, PACKAGE, V2_WORLD


def query(triangles, points):
    """Highest actual triangle intersection, returning height and local slope."""
    minimum = triangles[:, :, :2].min(axis=1)
    maximum = triangles[:, :, :2].max(axis=1)
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    slopes = np.degrees(np.arctan2(np.linalg.norm(normals[:, :2], axis=1), np.abs(normals[:, 2])))
    heights, grades = [], []
    for x, y in points:
        candidates = np.flatnonzero((minimum[:, 0] <= x+1e-7) & (maximum[:, 0] >= x-1e-7)
                                   & (minimum[:, 1] <= y+1e-7) & (maximum[:, 1] >= y-1e-7))
        t = triangles[candidates]
        a, b, c = t[:, 0], t[:, 1], t[:, 2]
        det = (b[:, 1]-c[:, 1])*(a[:, 0]-c[:, 0])+(c[:, 0]-b[:, 0])*(a[:, 1]-c[:, 1])
        valid = np.abs(det) > 1e-12
        u = np.divide((b[:, 1]-c[:, 1])*(x-c[:, 0])+(c[:, 0]-b[:, 0])*(y-c[:, 1]),
                      det, out=np.zeros(len(det)), where=valid)
        v = np.divide((c[:, 1]-a[:, 1])*(x-c[:, 0])+(a[:, 0]-c[:, 0])*(y-c[:, 1]),
                      det, out=np.zeros(len(det)), where=valid)
        valid &= (u >= -1e-6) & (v >= -1e-6) & (u+v <= 1+1e-6)
        selected = np.flatnonzero(valid)
        if not len(selected):
            heights.append(math.nan); grades.append(math.nan); continue
        z = u*a[:, 2]+v*b[:, 2]+(1-u-v)*c[:, 2]
        chosen = selected[np.argmax(z[selected])]
        heights.append(float(z[chosen])); grades.append(float(slopes[candidates[chosen]]))
    return np.asarray(heights), np.asarray(grades)


def stats(values):
    return dict(median=float(np.median(values)), p95=float(np.percentile(values, 95)), maximum=float(np.max(values)))


def main():
    # The sampler must reproduce an analytic plane before touching the scene.
    triangle = np.array([[[0, 0, 0], [1, 0, 1], [0, 1, 2]]], dtype=float)
    z, _ = query(triangle, [(0.2, 0.3), (0.1, 0.1)])
    np.testing.assert_allclose(z, [.8, .3])
    triangles, classes, _, minimum, maximum = _v2_geometry()
    manifest_path = PACKAGE/'models/terrain_v2/survey/manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    reports = []
    profiles = {}
    for key, prefix in (('measured_patch', 'ce4_crater_patch'), ('measured_background_patch', 'ce4_background_')):
        info = manifest[key]
        cx, cy = info['center_m']; half = info['size_m']/2
        x0, x1, y0, y1 = cx-half, cx+half, cy-half, cy+half
        patch = np.char.startswith(classes.astype(str), prefix)
        nearby = (np.all(maximum >= [x0-.4, y0-.4], axis=1)
                  & np.all(minimum <= [x1+.4, y1+.4], axis=1))
        inner, outer = triangles[patch], triangles[~patch & nearby]
        points, inside, outside, side_names = [], [], [], []
        offsets = np.arange(.01, info['size_m'], .02)
        for side, base, tangent, normal in (
            ('west', (x0,y0), (0,1), (1,0)), ('east', (x1,y0), (0,1), (-1,0)),
            ('south', (x0,y0), (1,0), (0,1)), ('north', (x0,y1), (1,0), (0,-1))):
            for distance in offsets:
                xy = np.array(base)+distance*np.array(tangent)
                points.append(xy); inside.append(xy+.001*np.array(normal)); outside.append(xy-.001*np.array(normal))
                side_names.append(side)
        edge_in, _ = query(inner, points)
        edge_out, _ = query(outer, points)
        zi, si = query(inner, inside)
        zo, so = query(outer, outside)
        if not all(np.isfinite(a).all() for a in (edge_in, edge_out, zi, zo, si, so)):
            raise ValueError('Boundary sampler found unsupported geometry')
        jumps = np.abs(edge_in-edge_out)
        delta = np.abs(si-so)
        centers = inner.mean(axis=1)
        distance = np.minimum.reduce([centers[:,0]-x0, x1-centers[:,0], centers[:,1]-y0, y1-centers[:,1]])
        normals = np.cross(inner[:,1]-inner[:,0], inner[:,2]-inner[:,0])
        slope = np.degrees(np.arctan2(np.linalg.norm(normals[:,:2],axis=1),np.abs(normals[:,2])))
        area = np.abs(normals[:,2])/2
        band = (distance >= 0) & (distance <= info['edge_feather_m'])
        worst = int(np.argmax(np.where(band, slope, -1)))
        report = dict(patch=key, center_m=info['center_m'], size_m=info['size_m'],
                      sampled_edge_points=len(points), edge_height_gap_m=stats(jumps),
                      inside_edge_slope_deg=stats(si), outside_edge_slope_deg=stats(so),
                      edge_slope_magnitude_difference_deg=stats(delta),
                      blend_band_area_m2=float(area[band].sum()),
                      blend_band_over_18deg_area_m2=float(area[band & (slope >= 18)].sum()),
                      blend_band_slope_deg=stats(slope[band]),
                      steepest_band_xy_m=centers[worst,:2].tolist())
        if key == 'measured_background_patch':
            source_path = PACKAGE/'data/terrain_sources/ce4_background_5cm.npz'
            with np.load(source_path, allow_pickle=False) as source:
                px, py = centers[worst,:2]-[cx,cy]
                ix = int(np.searchsorted(source['x'],px)-1)
                iy = int(np.searchsorted(source['y'],py)-1)
                heights = source['z'][iy:iy+2,ix:ix+2]
                dx, dy = source['x'][ix+1]-source['x'][ix], source['y'][iy+1]-source['y'][iy]
                # Same v00-v11 diagonal as the actual OBJ writer.
                gradients = [[(heights[0,1]-heights[0,0])/dx,(heights[1,1]-heights[0,1])/dy],
                             [(heights[1,1]-heights[1,0])/dx,(heights[1,0]-heights[0,0])/dy]]
                report['unblended_source_same_cell_slopes_deg'] = np.degrees(np.arctan(np.linalg.norm(gradients,axis=1))).tolist()
            report['unblended_source_sha256'] = hashlib.sha256(source_path.read_bytes()).hexdigest()
        reports.append(report)
        profiles[key] = dict(xy=np.asarray(points).tolist(), sides=side_names,
                             height_gap_m=jumps.tolist(), inside_slope_deg=si.tolist(), outside_slope_deg=so.tolist())
    result = dict(world_sha256=hashlib.sha256(V2_WORLD.read_bytes()).hexdigest(),
                  manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
                  method='Actual collision triangles; shared-edge height and one-sided slope; 2 cm edge sampling',
                  scope='Offline geometry audit, not contact/traction validation; 18 degrees is current terrain classifier threshold',
                  patches=reports)
    output = Path(__file__).resolve().parent
    (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8', newline='\n')
    (output/'edge-samples.json').write_text(json.dumps(profiles)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
