"""Analytic checks for the offline evaluator, with no simulation processes."""
import argparse
import json
from pathlib import Path

import numpy as np

from map_reference import ground_at, overlay_triangles, reduce_reference, compare, N


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('reference-checks.json'))
    args = parser.parse_args()
    axis = np.array([0., 1.])
    heights = np.array([[0., 0.], [0., 1.]])
    assert np.allclose(ground_at(axis, heights, [.8, .2], [.2, .8]), [.2, .2])
    assert not np.allclose(ground_at(axis, heights, .8, .2), .8*.2)
    fine = -15+(np.arange(N*5)+.5)*.05
    xx, yy = np.meshgrid(fine, fine)
    ground = .1*xx-.2*yy+.3
    owner = np.zeros(ground.shape, dtype=np.int16)
    reference = reduce_reference(ground, owner)
    assert np.allclose(reference['gx'], .1)
    assert np.allclose(reference['gy'], -.2)
    assert np.max(reference['roughness']) < 1e-12
    assert not reference['mixed'].any()
    cells = [dict(ix=0, iy=0, z=float(reference['z'][60, 60])+.02, slope=float(reference['slope'][60, 60]), valid=True, fit_state='valid'),
             dict(ix=1, iy=0, z=None, valid=False, fit_state='too_few_voxels'),
             dict(ix=100, iy=100, z=None, valid=False, fit_state='too_few_voxels')]
    payload = dict(resolution=.25, frames=3, stamp=4., cells=cells)
    maps, stats = compare(payload, reference)
    assert np.isclose(stats['total_observed_m2'], 3*.25**2)
    assert stats['outside_reference_cells'] == 1
    assert stats['single_surface']['count'] == 1
    assert np.isclose(stats['single_surface']['rmse_m'], .02)
    assert np.isnan(maps['z'][59, 59]) and not maps['observed'][59, 59]
    assert maps['observed'][60, 61] and not maps['valid'][60, 61]
    reference['mixed'][60, 60] = True
    _, stats = compare(payload, reference)
    assert stats['single_surface']['count'] == 0 and stats['mixed_valid_cells'] == 1
    assert stats['all_valid_including_mixed']['count'] == 1
    small_axis = np.array([.1, .4, .8])
    surface = np.zeros((3, 3))
    labels = np.zeros((3, 3), dtype=int)
    top = np.array([[[0, 0, 1], [1, 0, 1], [0, 1, 1]],
                    [[0, 0, 2], [1, 0, 2], [0, 1, 2]],
                    [[0, 0, -1], [1, 0, -1], [0, 1, -1]]], float)
    overlay_triangles(surface, labels, small_axis, top, 7)
    assert surface[0, 0] == 2 and labels[0, 0] == 7
    assert surface[2, 2] == 0 and labels[2, 2] == 0
    changed = owner.copy()
    changed[0, 0] = 4
    assert reduce_reference(ground, changed)['mixed'][0, 0]
    checks = dict(analytic_checks=['fixed_triangle_diagonal_not_bilinear', 'plane_height_and_slope',
                                  'no_unknown_fill', 'invalid_cells_not_scored', 'outside_area_counted',
                                  'known_2cm_bias_recovered', 'mixed_surfaces_separate', 'highest_exposed_triangle'],
                  passed=True)
    args.output.write_text(json.dumps(checks, indent=2)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps(checks))


if __name__ == '__main__':
    main()
