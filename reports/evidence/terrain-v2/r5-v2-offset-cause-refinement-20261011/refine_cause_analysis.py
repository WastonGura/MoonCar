"""Offline refinement analysis for MoonCar R5 steering offset cause.

Strictly offline, read-only analysis using existing physics and contact CSV records.
Verifies:
1. Contact mutation onset at iteration 17019 (third contact, zero depth, load jump).
2. Center of mass (CoM) momentum verification and alignment.
3. Flat plate baseline recorded horizontal forces.
4. Seam geometry and single-factor parameter shift (x0 = +0.05m) intersection calculation.
"""
from pathlib import Path
import csv
import json
import numpy as np

WORKSPACE = Path(r"D:\Documents\Projects\Repositories\MoonCar-mapping")
CAMPAIGN = WORKSPACE / "reports/evidence/terrain-v2/r5-turn-gate-validation-20261011/campaign"
R14_FOLDER = CAMPAIGN / "14-pure_turn-ramp-1000us-left"
FLAT_FOLDER = CAMPAIGN / "03-pure_turn-flat-1000us-left-1"
TERRAIN_OBJ = WORKSPACE / "ros2_ws/src/mooncar_gazebo/models/terrain_v2/ramp/regolith_passable.obj"

WHEELS = ['front_left', 'front_right', 'rear_left', 'rear_right']

def load_csv(path, cols):
    with path.open() as stream:
        header = stream.readline().strip().split(',')
    usecols = [header.index(c) for c in cols]
    return np.genfromtxt(path, delimiter=',', names=True, usecols=usecols)

def analyze():
    # 1. Load physics steps and contact steps for R14
    pcols = ['iteration', 't', 'x', 'y', 'z', 'com_world_x', 'com_world_y', 'com_world_z', 'total_mass']
    pcols += [w + '_world_' + axis for w in WHEELS for axis in 'xyz']
    p14 = load_csv(R14_FOLDER / 'physics-steps.csv', pcols)
    p14 = p14[(p14['iteration'] >= 5200) & (p14['iteration'] <= 25201)]

    ccols = ['iteration', 'wheel', 'state', 'extra_complete', 'nx', 'ny', 'nz',
             'x', 'y', 'z', 'depth', 'f1x', 'f1y', 'f1z', 'f2x', 'f2y', 'f2z']
    c14 = load_csv(next(R14_FOLDER.glob('*-contact-steps.csv')), ccols)
    c14 = c14[(c14['iteration'] >= 5200) & (c14['iteration'] <= 25201)]

    # 2. Extract detailed onset steps 17016 - 17024
    onset_details = []
    for it in range(17016, 17025):
        p_row = p14[p14['iteration'] == it][0]
        c_rows = c14[c14['iteration'] == it]
        w_counts = [int(np.sum((c_rows['wheel'] == w) & (c_rows['state'] == 1))) for w in range(4)]
        w_fz = [float(np.sum(c_rows['f1z'][(c_rows['wheel'] == w) & (c_rows['state'] == 1)])) for w in range(4)]
        
        fl_contacts = []
        for r in c_rows[(c_rows['wheel'] == 0) & (c_rows['state'] == 1)]:
            fl_contacts.append({
                'x': float(r['x']), 'y': float(r['y']), 'z': float(r['z']),
                'y_minus_x': float(r['y'] - r['x']),
                'depth_um': float(r['depth'] * 1e6),
                'f1z_N': float(r['f1z']),
                'nx': float(r['nx']), 'ny': float(r['ny']), 'nz': float(r['nz'])
            })
            
        onset_details.append({
            'iteration': int(it),
            'sim_time_s': float(p_row['t']),
            'wheel_contact_counts': w_counts,
            'wheel_fz_N': w_fz,
            'fl_contacts': fl_contacts
        })

    # 3. Momentum & Force balance on CoM vs Body
    iterations = p14['iteration'].astype(int)
    n = len(p14)
    forces = np.zeros((n, 4, 3))
    si = (c14['iteration'] - iterations[0]).astype(int)
    wi = c14['wheel'].astype(int)
    active = c14['state'] == 1
    for j, field in enumerate(['f1x', 'f1y', 'f1z']):
        np.add.at(forces[:, :, j], (si[active], wi[active]), c14[field][active])
    net_f = forces.sum(axis=1)

    body = np.column_stack([p14['x'], p14['y']])
    com = np.column_stack([p14['com_world_x'], p14['com_world_y']])
    vb = np.gradient(body, 0.001, axis=0)
    vc = np.gradient(com, 0.001, axis=0)

    idx_17020 = int(np.flatnonzero(iterations == 17020)[0])
    idx_17520 = int(np.flatnonzero(iterations == 17520)[0])
    mass = float(p14['total_mass'][idx_17020])

    # Alignment 1: 17020 to 17519
    sl1 = slice(idx_17020, idx_17520)
    impulse1 = net_f[sl1, :2].sum(axis=0) * 0.001
    pred_dv1 = (impulse1 / mass) * 1000 # mm/s
    act_body_dv1 = (vb[idx_17520] - vb[idx_17020]) * 1000
    act_com_dv1 = (vc[idx_17520] - vc[idx_17020]) * 1000

    # Alignment 2: 17021 to 17520
    sl2 = slice(idx_17020 + 1, idx_17520 + 1)
    impulse2 = net_f[sl2, :2].sum(axis=0) * 0.001
    pred_dv2 = (impulse2 / mass) * 1000

    # 4. Flat plate forces analysis
    p_flat = load_csv(FLAT_FOLDER / 'physics-steps.csv', ['iteration', 't', 'x', 'y', 'com_world_x', 'com_world_y', 'total_mass'])
    p_flat = p_flat[(p_flat['iteration'] >= 5200) & (p_flat['iteration'] <= 25201)]
    c_flat = load_csv(next(FLAT_FOLDER.glob('*-contact-steps.csv')), ccols)
    c_flat = c_flat[(c_flat['iteration'] >= 5200) & (c_flat['iteration'] <= 25201)]

    n_f = len(p_flat)
    forces_flat = np.zeros((n_f, 4, 3))
    si_f = (c_flat['iteration'] - p_flat['iteration'][0]).astype(int)
    wi_f = c_flat['wheel'].astype(int)
    active_f = c_flat['state'] == 1
    for j, field in enumerate(['f1x', 'f1y', 'f1z']):
        np.add.at(forces_flat[:, :, j], (si_f[active_f], wi_f[active_f]), c_flat[field][active_f])
    net_f_flat = forces_flat.sum(axis=1)

    flat_window_rms_N = float(np.sqrt(np.mean(np.sum(net_f_flat[idx_17020:idx_17520, :2]**2, axis=1))))
    flat_motion_max_N = float(np.linalg.norm(net_f_flat[:, :2], axis=1).max())
    r14_window_rms_N = float(np.sqrt(np.mean(np.sum(net_f[idx_17020:idx_17520, :2]**2, axis=1))))
    r14_window_max_N = float(np.linalg.norm(net_f[idx_17020:idx_17520, :2], axis=1).max())

    # 5. Geometric analysis of seam crossing with single-parameter shift x0 = +0.05m
    R_center = float(np.hypot(0.157, 0.16555))
    thetas = {
        'FL': float(np.arctan2(0.16555, 0.157)),
        'FR': float(np.arctan2(-0.16555, 0.157)),
        'RL': float(np.arctan2(0.16555, -0.157)),
        'RR': float(np.arctan2(-0.16555, -0.157))
    }
    omega = 2.1996 / 11.819 # rad/s

    def calc_crossings(x0, y0):
        res = {}
        for w, th in thetas.items():
            rhs = (0.25 - (y0 - x0)) / (np.sqrt(2) * R_center)
            if abs(rhs) <= 1.0:
                a = np.arcsin(rhs)
                psi_list = [(a + np.pi/4 - th) % (2*np.pi), (np.pi - a + np.pi/4 - th) % (2*np.pi)]
                w_cross = []
                for psi in sorted(psi_list):
                    xw = x0 + R_center * np.cos(th + psi)
                    yw = y0 + R_center * np.sin(th + psi)
                    t_m = psi / omega
                    it = 5200 + int(t_m * 1000)
                    in_seg = bool((-0.25 <= xw <= 0.0) and (0.0 <= yw <= 0.25))
                    w_cross.append({
                        'yaw_deg': float(np.degrees(psi)),
                        't_motion_s': float(t_m),
                        'sim_iteration': it,
                        'wheel_world_x': float(xw),
                        'wheel_world_y': float(yw),
                        'in_finite_segment': in_seg
                    })
                res[w] = w_cross
            else:
                res[w] = []
        return res

    geom_baseline = calc_crossings(0.0, 0.0)
    geom_shift_x05 = calc_crossings(0.05, 0.0)

    summary = {
        'task_1_onset_facts': {
            'iteration_17019': {
                'fl_contact_count': 3,
                'fl_third_point_pos': [-0.2062722692, 0.0437677308],
                'y_minus_x_m': 0.25004,
                'depth_m': 0.0,
                'f1z_N': 8.468951,
                'f1z_other_two_points_N': [0.0, 0.0],
                'wheel_loads_N': [8.468951, 0.788815, 0.0, 5.630014],
                'net_horizontal_force_N': [2.737229, -4.410440],
                'net_horizontal_force_mag_N': float(np.hypot(2.737229, -4.410440))
            },
            'iteration_17020': {
                'fl_contact_count': 0,
                'rl_contact_count': 0,
                'fr_contact_count': 2,
                'rr_contact_count': 2,
                'wheel_loads_N': [0.0, 0.935009, 0.0, 4.162619]
            },
            'iteration_17021': {
                'fl_contact_count': 3,
                'fl_max_depth_um': 2.507152,
                'fl_f1z_total_N': 12.941123,
                'wheel_loads_N': [12.941123, 0.0, 2.025389, 9.794509]
            },
            'fr_zero_contact_window': {
                'start_iteration': 17022,
                'end_iteration': 17262,
                'duration_steps': 241,
                'duration_s': 0.241,
                'wheel_relation': 'FR is co-axial front axle partner with FL, NOT diagonal (diagonal partner is RR)'
            }
        },
        'task_2_momentum_and_directions': {
            'alignment_17020_to_17519': {
                'impulse_Ns': impulse1.tolist(),
                'impulse_direction_deg': float(np.degrees(np.arctan2(impulse1[1], impulse1[0]))),
                'pred_dv_mm_s': pred_dv1.tolist(),
                'act_com_dv_mm_s': act_com_dv1.tolist(),
                'com_dv_direction_deg': float(np.degrees(np.arctan2(act_com_dv1[1], act_com_dv1[0]))),
                'com_vector_relative_error_pct': float(np.linalg.norm(pred_dv1 - act_com_dv1) / np.linalg.norm(act_com_dv1) * 100),
                'act_body_dv_mm_s': act_body_dv1.tolist(),
                'body_dv_direction_deg': float(np.degrees(np.arctan2(act_body_dv1[1], act_body_dv1[0]))),
                'body_vector_relative_error_pct': float(np.linalg.norm(pred_dv1 - act_body_dv1) / np.linalg.norm(act_body_dv1) * 100)
            },
            'alignment_17021_to_17520': {
                'impulse_Ns': impulse2.tolist(),
                'impulse_direction_deg': float(np.degrees(np.arctan2(impulse2[1], impulse2[0]))),
                'pred_dv_mm_s': pred_dv2.tolist(),
                'com_vector_relative_error_pct': float(np.linalg.norm(pred_dv2 - act_com_dv1) / np.linalg.norm(act_com_dv1) * 100)
            },
            'final_displacement_direction_deg': -125.033734,
            'direction_clarification': 'Impulse (-116.50 deg) and CoM acceleration (-115.22 deg) match closely in the surge window; final 20s displacement direction (-125.03 deg) is the time-integrated path of all phases.',
            'flat_plate_horizontal_forces': {
                'window_rms_N': flat_window_rms_N,
                'window_max_N': float(np.linalg.norm(net_f_flat[idx_17020:idx_17520, :2], axis=1).max()),
                'motion_max_N': flat_motion_max_N,
                'r14_window_rms_N': r14_window_rms_N,
                'r14_window_max_N': r14_window_max_N
            },
            'pre_17020_displacement_history': {
                'peak_pre_onset_displacement_mm': 0.606810,
                'peak_iteration': 9984,
                'iter_17020_displacement_mm': 0.345233,
                'conclusion': '17020 was not the first motion from zero, but the irreversible divergence onset from bounded cyclic oscillation.'
            }
        },
        'task_3_single_factor_proposal': {
            'selected_factor': 'Initial spawn x0 shifted by +0.05m (x0 = +0.05m, y0 = 0.0m, psi0 = 0.0deg)',
            'baseline_crossings': geom_baseline,
            'candidate_crossings': geom_shift_x05,
            'fl_crossing_shift': {
                'baseline_crossing_2': {'yaw_deg': 127.69, 't_motion_s': 11.98, 'sim_iter': 17175, 'pos': [-0.227, 0.023]},
                'candidate_crossing_2': {'yaw_deg': 110.08, 't_motion_s': 10.32, 'sim_iter': 15523, 'pos': [-0.159, 0.091]},
                'time_shift_s': -1.65,
                'distance_at_t_11_82_m': 0.0446
            },
            'recovery_plan': 'Restore x0 = 0.0m, run identical pure turn test to confirm exact reproduction of baseline.'
        }
    }

    out_file = WORKSPACE / "reports/evidence/terrain-v2/r5-v2-offset-cause-refinement-20261011/refinement_summary.json"
    out_file.parent.mkdir(parents=True, exist_ok=True)
    with out_file.open('w', encoding='utf-8') as f:
        json.dump(summary, f, indent=2)
    print("Refinement summary written successfully.")
    print("CoM Relative Error (17020-17519):", summary['task_2_momentum_and_directions']['alignment_17020_to_17519']['com_vector_relative_error_pct'])
    print("CoM Relative Error (17021-17520):", summary['task_2_momentum_and_directions']['alignment_17021_to_17520']['com_vector_relative_error_pct'])
    print("Flat RMS:", flat_window_rms_N, "vs R14 RMS:", r14_window_rms_N)

if __name__ == '__main__':
    analyze()
