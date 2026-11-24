"""Read-only recheck of Gemini's contact/impulse hypothesis; no simulation."""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
CAMPAIGN = ROOT.parent / 'r5-turn-gate-validation-20261011/campaign'
WHEELS = ['front_left', 'front_right', 'rear_left', 'rear_right']
INPUTS = {}


def load(path, cols):
    with path.open() as stream:
        header = stream.readline().strip().split(',')
    INPUTS[str(path.relative_to(ROOT.parent))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return np.genfromtxt(path, delimiter=',', names=True,
                         usecols=[header.index(c) for c in cols])


def runs(mask, iterations):
    indices = np.flatnonzero(mask)
    if not len(indices):
        return []
    groups = np.split(indices, np.flatnonzero(np.diff(indices) != 1) + 1)
    return [[int(iterations[g[0]]), int(iterations[g[-1]]), len(g)] for g in groups]


def analyze(name):
    folder = CAMPAIGN / name
    pcols = ['iteration', 't', 'x', 'y', 'z', 'com_world_x', 'com_world_y',
             'total_mass', 'input_lx', 'input_az']
    pcols += [w + '_world_' + axis for w in WHEELS for axis in 'xyz']
    p = load(folder / 'physics-steps.csv', pcols)
    p = p[(p['iteration'] >= 5200) & (p['iteration'] <= 25201)]
    ccols = ['iteration', 'wheel', 'state', 'extra_complete', 'nx', 'ny', 'nz',
             'x', 'y', 'z', 'depth', 'f1x', 'f1y', 'f1z', 'f2x', 'f2y', 'f2z']
    c = load(next(folder.glob('*-contact-steps.csv')), ccols)
    c = c[(c['iteration'] >= 5200) & (c['iteration'] <= 25201)]
    iterations = p['iteration'].astype(int)
    assert np.all(np.diff(iterations) == 1)
    n = len(p)
    forces = np.zeros((n, 4, 3))
    counts = np.zeros((n, 4), dtype=int)
    maxdepth = np.zeros((n, 4))
    coverage = np.zeros((n, 4), dtype=int)
    si = (c['iteration'] - iterations[0]).astype(int)
    wi = c['wheel'].astype(int)
    assert np.all((wi >= 0) & (wi <= 3))
    np.add.at(coverage, (si, wi), 1)
    assert np.all(coverage > 0), 'Missing contact source rows'
    assert np.all(np.isin(c['state'], [0, 1])), 'Unknown contact state'
    active = c['state'] == 1
    assert np.all(c['extra_complete'][active] == 1)
    for field in ['depth', 'nx', 'ny', 'nz', 'f1x', 'f1y', 'f1z']:
        assert np.all(np.isfinite(c[field][active])), field
    np.add.at(counts, (si[active], wi[active]), 1)
    np.maximum.at(maxdepth, (si[active], wi[active]), c['depth'][active])
    for j, field in enumerate(['f1x', 'f1y', 'f1z']):
        np.add.at(forces[:, :, j], (si[active], wi[active]), c[field][active])
    net = forces.sum(axis=1)
    body = np.column_stack([p['x'], p['y']])
    com = np.column_stack([p['com_world_x'], p['com_world_y']])
    vb = np.gradient(body, .001, axis=0)
    vc = np.gradient(com, .001, axis=0)
    a = int(np.flatnonzero(iterations == 17020)[0])
    b = int(np.flatnonzero(iterations == 17520)[0])
    mass = float(p['total_mass'][a])
    motion = (iterations > 5200) & (iterations <= 25200)
    disp = (body - body[0]) * 1000
    speed = np.linalg.norm(vb, axis=1) * 1000
    closure = {}
    for label, sl in [('gemini_17020_to_17519', slice(a, b)),
                      ('next_steps_17021_to_17520', slice(a+1, b+1))]:
        impulse = net[sl, :2].sum(axis=0) * .001
        pred = impulse / mass * 1000
        actual_body = (vb[b] - vb[a]) * 1000
        actual_com = (vc[b] - vc[a]) * 1000
        closure[label] = {
            'impulse_Ns': impulse.tolist(), 'predicted_dv_mm_s': pred.tolist(),
            'actual_body_dv_mm_s': actual_body.tolist(),
            'actual_com_dv_mm_s': actual_com.tolist(),
            'impulse_direction_deg': float(np.degrees(np.arctan2(pred[1], pred[0]))),
            'body_dv_direction_deg': float(np.degrees(np.arctan2(actual_body[1], actual_body[0]))),
            'com_dv_direction_deg': float(np.degrees(np.arctan2(actual_com[1], actual_com[0]))),
            'body_vector_relative_error_percent': float(np.linalg.norm(pred-actual_body)/np.linalg.norm(actual_body)*100) if np.linalg.norm(actual_body)>1e-6 else None,
            'com_vector_relative_error_percent': float(np.linalg.norm(pred-actual_com)/np.linalg.norm(actual_com)*100),
            'body_speed_relative_error_percent': float(abs(np.linalg.norm(pred)-np.linalg.norm(actual_body))/np.linalg.norm(actual_body)*100) if np.linalg.norm(actual_body)>1e-6 else None,
            'wheel_impulses_Ns': (forces[sl].sum(axis=0)[:, :2] * .001).tolist(),
        }
    timeline = []
    for iteration in range(17016, 17025):
        k = int(np.flatnonzero(iterations == iteration)[0])
        timeline.append({'iteration': iteration, 'sim_time_s': float(p['t'][k]),
                         'motion_elapsed_s': float(p['t'][k]-5.2),
                         'contact_point_counts': counts[k].tolist(),
                         'wheel_fz_N': forces[k, :, 2].tolist(),
                         'max_depth_um': [float(maxdepth[k,w]*1e6) if counts[k,w] else None for w in range(4)],
                         'net_fxy_N': net[k, :2].tolist(),
                         'body_vxy_mm_s': (vb[k]*1000).tolist(),
                         'wheel_center_z_m': [float(p[w+'_world_z'][k]) for w in WHEELS],
                         'FL_center_y_minus_x_m': float(p['front_left_world_y'][k]-p['front_left_world_x'][k])})
    fr_runs = runs(counts[:, 1] == 0, iterations)
    active_motion = active & (c['iteration'] > 5200) & (c['iteration'] <= 25200)
    normal_angle = np.degrees(np.arctan2(np.hypot(c['nx'][active_motion], c['ny'][active_motion]), np.abs(c['nz'][active_motion])))
    summary = {
        'mass_kg': mass, 'motion_steps': int(motion.sum()),
        'body_final_mm': disp[iterations == 25200][0].tolist(),
        'body_max_displacement_mm': float(np.linalg.norm(disp[motion], axis=1).max()),
        'pre_17020_max_speed_mm_s': float(speed[motion & (iterations < 17020)].max()),
        'pre_17020_max_displacement_mm': float(np.linalg.norm(disp[motion & (iterations < 17020)], axis=1).max()),
        'pre_17020_peak_displacement_iteration': int(iterations[motion & (iterations < 17020)][np.argmax(np.linalg.norm(disp[motion & (iterations < 17020)], axis=1))]),
        'mean_fz_17020_to_17519_N': forces[a:b, :, 2].mean(axis=0).tolist(),
        'contact_point_count_ranges_motion': [[int(counts[motion,w].min()), int(counts[motion,w].max())] for w in range(4)],
        'zero_contact_steps_motion_per_wheel': (counts[motion] == 0).sum(axis=0).tolist(),
        'FR_zero_contact_runs_intersecting_17020_to_17519': [r for r in fr_runs if r[1] >= 17020 and r[0] < 17520],
        'net_horizontal_force_window_max_N': float(np.linalg.norm(net[a:b,:2], axis=1).max()),
        'net_horizontal_force_window_rms_N': float(np.sqrt(np.mean(np.sum(net[a:b,:2]**2,axis=1)))),
        'net_horizontal_force_motion_max_N': float(np.linalg.norm(net[motion,:2], axis=1).max()),
        'maximum_normal_tilt_deg_motion': float(normal_angle.max()),
        'closure': closure, 'onset_timeline': timeline,
        'FL_source_points_17019_and_17021': [{f:float(row[f]) for f in ccols} for row in c[(c['wheel']==0)&active&np.isin(c['iteration'],[17019,17021])]],
        'force_pair_sum_max_N': float(max(np.max(np.abs(c[f'f1{ax}'][active]+c[f'f2{ax}'][active])) for ax in 'xyz')),
    }
    return summary


def main():
    result = {'scope': 'offline only; no Gazebo, containers or original evidence modified',
              'wheels_order': WHEELS,
              'velocity_method': 'central difference of step poses; force interval alignment shown explicitly',
              'r14': analyze('14-pure_turn-ramp-1000us-left'),
              'flat03': analyze('03-pure_turn-flat-1000us-left-1'),
              'spawn_shift_check': {'seam': 'y-x=0.25', 'proposed_shift_m': [.05,.05],
                                    'change_in_y_minus_x_m': 0,
                                    'conclusion': 'Shift is parallel to this seam; it does not change perpendicular distance to this line.'},
              'inputs_sha256': INPUTS}
    source = ROOT.parent / 'r5-v2-offset-cause-analysis-20261011'
    manifest = json.loads((source/'manifest.json').read_text(encoding='utf-8-sig'))
    result['gemini_manifest_verified'] = all(hashlib.sha256((source/name).read_bytes()).hexdigest() == entry['sha256'] for name,entry in manifest.items())
    (ROOT/'review-results.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    with (ROOT/'onset-timeline.csv').open('w', newline='', encoding='utf-8') as stream:
        fields = ['iteration', 'sim_time_s', 'motion_elapsed_s', 'contact_point_counts', 'wheel_fz_N', 'max_depth_um', 'net_fxy_N', 'body_vxy_mm_s', 'wheel_center_z_m', 'FL_center_y_minus_x_m']
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(result['r14']['onset_timeline'])
    for key in ['r14','flat03']:
        small = {k:v for k,v in result[key].items() if k != 'onset_timeline'}
        print(key, json.dumps(small, indent=2))
    print('onset', json.dumps(result['r14']['onset_timeline'], indent=2))
    print('Gemini manifest verified:', result['gemini_manifest_verified'])


if __name__ == '__main__':
    main()
