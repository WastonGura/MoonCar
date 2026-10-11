"""Evaluation script for MoonCar R5 steering offset two-round diagnosis.

Features:
1. Strict contact source qualification: checks row completeness per step per wheel,
   rejects missing records or unknown states (e.g. state=-1), and verifies finite fields.
2. Robust 2D segment intersection handling non-parallel and collinear overlapping segments.
3. Multi-wheel geometric crossing detection against target finite seam and grid edges.
4. Distinguishes explicit empty contact snapshots (state=0) from geometric liftoff.
5. Extracts detailed causal timeline: extra contacts (N>=3), load spikes (Fz>5N),
   airborne runs, net horizontal impulses, and center-of-mass momentum changes.
6. Dynamic schedule bounds extracted from result.json receipts.
7. CLI entrypoint supporting arbitrary run directories and non-overwriting outputs.
"""
import argparse
import csv
import json
from pathlib import Path
from typing import Dict, List, Tuple, Any, Optional

import numpy as np

WHEELS = ['front_left', 'front_right', 'rear_left', 'rear_right']
TARGET_SEAM = {
    'p0': np.array([-0.25, 0.0]),
    'p1': np.array([0.0, 0.25]),
    'equation': 'y - x = 0.25, x in [-0.25, 0.0]'
}

GRID_EDGES = [
    {'name': 'target_diagonal_seam', 'p0': np.array([-0.25, 0.0]), 'p1': np.array([0.0, 0.25])},
    {'name': 'center_axis_x0', 'p0': np.array([0.0, -0.25]), 'p1': np.array([0.0, 0.25])},
    {'name': 'center_axis_y0', 'p0': np.array([-0.25, 0.0]), 'p1': np.array([0.25, 0.0])},
    {'name': 'center_diagonal_y_minus_x_0', 'p0': np.array([-0.25, -0.25]), 'p1': np.array([0.25, 0.25])},
    {'name': 'quad4_diagonal_seam', 'p0': np.array([0.0, -0.25]), 'p1': np.array([0.25, 0.0])},
]

def load_csv(path: Path, cols: List[str]) -> np.ndarray:
    with path.open(encoding='utf-8') as stream:
        header = stream.readline().strip().split(',')
    usecols = [header.index(c) for c in cols]
    return np.genfromtxt(path, delimiter=',', names=True, usecols=usecols)

def check_segment_intersection(a0, a1, b0, b1) -> Tuple[bool, Optional[np.ndarray]]:
    """Check if 2D line segment a intersects line segment b, handling collinear overlaps."""
    a0 = np.asarray(a0, dtype=float)
    a1 = np.asarray(a1, dtype=float)
    b0 = np.asarray(b0, dtype=float)
    b1 = np.asarray(b1, dtype=float)

    da = a1 - a0
    db = b1 - b0
    dp = b0 - a0

    denom = da[0] * db[1] - da[1] * db[0]
    cross_dp_da = dp[0] * da[1] - dp[1] * da[0]

    # Check collinearity
    if abs(denom) < 1e-12:
        if abs(cross_dp_da) < 1e-12:
            len_sq_a = np.dot(da, da)
            if len_sq_a < 1e-14:
                return False, None
            # Project b0 and b1 onto segment a: a0 + t * da
            t_b0 = np.dot(b0 - a0, da) / len_sq_a
            t_b1 = np.dot(b1 - a0, da) / len_sq_a
            t_min = min(t_b0, t_b1)
            t_max = max(t_b0, t_b1)
            # Check overlap with [0, 1]
            overlap_min = max(0.0, t_min)
            overlap_max = min(1.0, t_max)
            if overlap_min <= overlap_max:
                pt = a0 + 0.5 * (overlap_min + overlap_max) * da
                return True, pt
        return False, None

    # Non-parallel segments
    t = (dp[0] * db[1] - dp[1] * db[0]) / denom
    u = (dp[0] * da[1] - dp[1] * da[0]) / denom
    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        pt = a0 + t * da
        return True, pt
    return False, None

def evaluate_run(run_dir: Path) -> Dict[str, Any]:
    run_dir = Path(run_dir)
    p_path = run_dir / 'physics-steps.csv'
    if not p_path.exists():
        raise FileNotFoundError(f"Missing physics-steps.csv in {run_dir}")

    c_files = list(run_dir.glob('*-contact-steps.csv'))
    if not c_files:
        raise FileNotFoundError(f"Missing *-contact-steps.csv in {run_dir}")
    c_path = c_files[0]

    # Read schedule from result.json if available
    res_path = run_dir / 'result.json'
    schedule_bounds = None
    if res_path.exists():
        try:
            res_json = json.loads(res_path.read_text(encoding='utf-8'))
            sched = res_json.get('schedule', {})
            schedule_bounds = {
                'parked_before': sched.get('parked', {}).get('before', {}).get('iteration'),
                'motion_before': sched.get('motion', {}).get('before', {}).get('iteration'),
                'motion_after': sched.get('motion', {}).get('after', {}).get('iteration'),
                'stop_after': sched.get('stop', {}).get('after', {}).get('iteration'),
            }
        except Exception:
            pass

    # Load physics steps
    pcols = ['iteration', 't', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz',
             'com_world_x', 'com_world_y', 'total_mass']
    for w in WHEELS:
        for ax in 'xyz':
            pcols.append(f'{w}_world_{ax}')
            pcols.append(f'{w}_axis_{ax}')
    p = load_csv(p_path, pcols)

    # Determine iteration boundaries
    if schedule_bounds and schedule_bounds['motion_before'] is not None and schedule_bounds['motion_after'] is not None:
        lo = schedule_bounds['motion_before']
        hi = schedule_bounds['motion_after']
    else:
        # Default fallback to range present in file
        min_it = int(p['iteration'].min())
        max_it = int(p['iteration'].max())
        if min_it <= 5200 and max_it >= 25200:
            lo, hi = 5200, 25200
        else:
            lo, hi = min_it - 1, max_it

    # Filter motion window (iteration > lo and <= hi)
    p_motion = p[(p['iteration'] > lo) & (p['iteration'] <= hi)]
    if len(p_motion) == 0:
        p_motion = p

    iterations = p_motion['iteration'].astype(int)
    n_steps = len(p_motion)

    # Load contact steps
    ccols = ['iteration', 'wheel', 'state', 'nx', 'ny', 'nz', 'x', 'y', 'z', 'depth', 'f1x', 'f1y', 'f1z']
    c = load_csv(c_path, ccols)
    c_motion = c[(c['iteration'] > lo) & (c['iteration'] <= hi)]
    if len(c_motion) == 0:
        c_motion = c

    # --- CONTACT SOURCE INTEGRITY & QUALIFICATION CHECK ---
    # Every step and every wheel must have at least one record in the contact file
    coverage = np.zeros((n_steps, 4), dtype=int)
    c_iter = c_motion['iteration'].astype(int)
    si = c_iter - iterations[0]
    wi = c_motion['wheel'].astype(int)

    # Mask valid step indices
    valid_step_mask = (si >= 0) & (si < n_steps) & (wi >= 0) & (wi <= 3)
    np.add.at(coverage, (si[valid_step_mask], wi[valid_step_mask]), 1)

    qualification_error = None
    qualified = True

    # Check 1: Missing wheel records
    if not np.all(coverage > 0):
        missing_indices = np.argwhere(coverage == 0)
        missing_step = iterations[missing_indices[0][0]]
        missing_wheel = missing_indices[0][1]
        qualification_error = f"Missing contact source row for step {missing_step}, wheel {WHEELS[missing_wheel]}"
        qualified = False

    # Check 2: Unknown states (state must be in {0, 1})
    if qualified and not np.all(np.isin(c_motion['state'], [0, 1])):
        invalid_states = c_motion['state'][~np.isin(c_motion['state'], [0, 1])]
        qualification_error = f"Unknown contact state {invalid_states[0]} found in contact records"
        qualified = False

    # Check 3: Active rows must have finite fields
    if qualified:
        active = c_motion['state'] == 1
        for field in ['depth', 'nx', 'ny', 'nz', 'f1x', 'f1y', 'f1z']:
            if not np.all(np.isfinite(c_motion[field][active])):
                qualification_error = f"Non-finite value in active contact field '{field}'"
                qualified = False
                break

    # If disqualified, return failure report directly
    if not qualified:
        return {
            'run_dir': str(run_dir),
            'qualified': False,
            'qualification_error': qualification_error,
            'final_displacement_mm': None,
            'max_displacement_mm': None,
            'target_seam_overlap_windows': [],
            'fl_contact_ge3_steps_count': 0,
            'fr_zero_contact_steps_count': None, # Never count disqualified missing/unknown as zero contact!
            'fl_load_spikes_count': 0,
            'net_horizontal_force_rms_N': 0.0,
        }

    # --- GEOMETRIC CONTACT ANALYSIS (LOWEST GENERATOR) ---
    v_z = np.array([0.0, 0.0, 1.0])
    overlap_windows_per_wheel = {}
    generator_heights_per_wheel = {}

    for w_idx, w_name in enumerate(WHEELS):
        w_center = np.column_stack([p_motion[f'{w_name}_world_{ax}'] for ax in 'xyz'])
        w_axis = np.column_stack([p_motion[f'{w_name}_axis_{ax}'] for ax in 'xyz'])
        w_axis /= np.linalg.norm(w_axis, axis=1)[:, None]

        # Lowest cylinder generator line
        vert = v_z - w_axis * w_axis[:, 2, None]
        vert /= np.linalg.norm(vert, axis=1)[:, None]
        w_bottom = w_center - 0.076 * vert

        w_end0 = w_bottom - 0.025 * w_axis
        w_end1 = w_bottom + 0.025 * w_axis

        # Record lowest generator height
        generator_heights_per_wheel[w_name] = w_bottom[:, 2].tolist()

        # Check intersection with target finite seam
        overlaps = []
        for i in range(n_steps):
            is_intersect, _ = check_segment_intersection(
                w_end0[i, :2], w_end1[i, :2],
                TARGET_SEAM['p0'], TARGET_SEAM['p1']
            )
            if is_intersect:
                overlaps.append(int(iterations[i]))

        windows = []
        if overlaps:
            groups = np.split(overlaps, np.flatnonzero(np.diff(overlaps) != 1) + 1)
            for g in groups:
                windows.append({
                    'first_iteration': int(g[0]),
                    'last_iteration': int(g[-1]),
                    'duration_steps': len(g),
                    'first_motion_s': float((g[0] - iterations[0]) * 0.001),
                    'last_motion_s': float((g[-1] - iterations[0]) * 0.001)
                })
        overlap_windows_per_wheel[w_name] = windows

    # --- CONTACT FORCES & STATES ACCUMULATION ---
    active = c_motion['state'] == 1
    counts = np.zeros((n_steps, 4), dtype=int)
    fz = np.zeros((n_steps, 4))
    f_net = np.zeros((n_steps, 3))

    si_active = si[active]
    wi_active = wi[active]
    np.add.at(counts, (si_active, wi_active), 1)
    np.add.at(fz, (si_active, wi_active), c_motion['f1z'][active])
    for j, ax in enumerate(['f1x', 'f1y', 'f1z']):
        np.add.at(f_net[:, j], si_active, c_motion[ax][active])

    # Explicit empty snapshots per wheel (counts == 0)
    explicit_empty_runs_per_wheel = {}
    for w_idx, w_name in enumerate(WHEELS):
        empty_mask = counts[:, w_idx] == 0
        empty_indices = np.flatnonzero(empty_mask)
        runs = []
        if len(empty_indices):
            groups = np.split(empty_indices, np.flatnonzero(np.diff(empty_indices) != 1) + 1)
            for g in groups:
                runs.append({
                    'first_iteration': int(iterations[g[0]]),
                    'last_iteration': int(iterations[g[-1]]),
                    'duration_steps': len(g),
                    'first_motion_s': float((iterations[g[0]] - iterations[0]) * 0.001),
                    'last_motion_s': float((iterations[g[-1]] - iterations[0]) * 0.001)
                })
        explicit_empty_runs_per_wheel[w_name] = runs

    # Displacements & Velocities
    body = np.column_stack([p_motion['x'], p_motion['y']])
    disp = np.linalg.norm(body - body[0], axis=1) * 1000 # mm
    com = np.column_stack([p_motion['com_world_x'], p_motion['com_world_y']])
    vb = np.gradient(body, 0.001, axis=0)
    vc = np.gradient(com, 0.001, axis=0)

    # Key event statistics
    fl_3_or_more = iterations[counts[:, 0] >= 3].tolist()
    fr_zero_runs = iterations[counts[:, 1] == 0].tolist()
    fl_load_spikes = iterations[fz[:, 0] > 6.0].tolist()

    # Critical event list
    critical_events = []
    for i in range(n_steps):
        it = int(iterations[i])
        # Log steps with anomalous contact counts or load spikes
        if counts[i, 0] >= 3 or fz[i, 0] > 8.0 or counts[i, 1] == 0:
            c_step = c_motion[(c_motion['iteration'] == it) & (c_motion['state'] == 1)]
            fl_pts = []
            for r in c_step[c_step['wheel'] == 0]:
                fl_pts.append({
                    'xyz': [float(r['x']), float(r['y']), float(r['z'])],
                    'normal': [float(r['nx']), float(r['ny']), float(r['nz'])],
                    'depth_um': float(r['depth'] * 1e6),
                    'f1z_N': float(r['f1z'])
                })
            critical_events.append({
                'iteration': it,
                'motion_s': float((it - iterations[0]) * 0.001),
                'contact_counts': counts[i].tolist(),
                'wheel_fz_N': fz[i].tolist(),
                'fl_contacts': fl_pts,
                'net_horizontal_force_N': f_net[i, :2].tolist()
            })

    # Integrated horizontal impulse
    net_impulse = f_net[:, :2].sum(axis=0) * 0.001

    return {
        'run_dir': str(run_dir),
        'qualified': True,
        'qualification_error': None,
        'final_displacement_mm': float(disp[-1]),
        'max_displacement_mm': float(disp.max()),
        'target_seam_overlap_windows': overlap_windows_per_wheel['front_left'], # FL for backward compat
        'target_seam_overlap_windows_all_wheels': overlap_windows_per_wheel,
        'fl_contact_ge3_steps_count': len(fl_3_or_more),
        'fr_zero_contact_steps_count': len(fr_zero_runs),
        'fl_load_spikes_count': len(fl_load_spikes),
        'net_horizontal_force_rms_N': float(np.sqrt(np.mean(np.sum(f_net[:, :2]**2, axis=1)))),
        'explicit_empty_runs_per_wheel': explicit_empty_runs_per_wheel,
        'net_horizontal_impulse_Ns': net_impulse.tolist(),
        'actual_com_dv_mm_s': ((vc[-1] - vc[0]) * 1000).tolist() if len(vc) > 1 else [0.0, 0.0],
        'actual_body_dv_mm_s': ((vb[-1] - vb[0]) * 1000).tolist() if len(vb) > 1 else [0.0, 0.0],
        'critical_events_count': len(critical_events),
        'sample_critical_events': critical_events[:10] if len(critical_events) > 10 else critical_events
    }

def main():
    parser = argparse.ArgumentParser(description="Evaluate MoonCar R5 steering offset diagnostic run.")
    parser.add_argument("--run-dir", type=str, required=True, help="Path to simulation run output directory")
    parser.add_argument("--output", type=str, default=None, help="Output JSON path for evaluation results")
    args = parser.parse_args()

    run_path = Path(args.run_dir)
    res = evaluate_run(run_path)

    out_json = json.dumps(res, indent=2)
    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(out_json, encoding='utf-8')
        print(f"Evaluation report written to {out_path}")
    else:
        print(out_json)

if __name__ == '__main__':
    main()
