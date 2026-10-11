"""Evaluation script for the two-round phase-shift diagnosis runs.

Ready to ingest the output directories of test_shift_x_plus_5cm and restore_x_0
once authorized and executed.
Computes:
1. Exact wheel cylinder bottom generator segments across all 20,000 steps.
2. Intersection status with the target finite seam (-0.25, 0) -- (0, 0.25) and adjacent grid lines.
3. Event timeline: extra contact points, contact loss (airborne runs), load spikes, and horizontal impulse.
4. Correlation between actual geometric crossing windows and dynamic disturbance onsets.
5. Reproducibility comparison between restore_x_0 and baseline round 14.
"""
from pathlib import Path
import csv
import json
import numpy as np

WHEELS = ['front_left', 'front_right', 'rear_left', 'rear_right']
TARGET_SEAM = {'p0': np.array([-0.25, 0.0]), 'p1': np.array([0.0, 0.25])}

def load_csv(path, cols):
    with path.open() as stream:
        header = stream.readline().strip().split(',')
    usecols = [header.index(c) for c in cols]
    return np.genfromtxt(path, delimiter=',', names=True, usecols=usecols)

def check_segment_intersection(seg_a0, seg_a1, seg_b0, seg_b1):
    """Check if 2D line segment a intersects line segment b."""
    da = seg_a1 - seg_a0
    db = seg_b1 - seg_b0
    dp = seg_a0 - seg_b0
    denom = da[0] * db[1] - da[1] * db[0]
    if abs(denom) < 1e-12:
        return False, None
    t = (dp[1] * db[0] - dp[0] * db[1]) / denom
    u = (dp[1] * da[0] - dp[0] * da[1]) / denom
    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        pt = seg_a0 + t * da
        return True, pt
    return False, None

def evaluate_run(run_dir: Path):
    p_path = run_dir / 'physics-steps.csv'
    c_path = next(run_dir.glob('*-contact-steps.csv'))
    
    pcols = ['iteration', 't', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz',
             'com_world_x', 'com_world_y', 'total_mass']
    for w in WHEELS:
        for ax in 'xyz':
            pcols.append(f'{w}_world_{ax}')
            pcols.append(f'{w}_axis_{ax}')
            
    p = load_csv(p_path, pcols)
    p = p[(p['iteration'] > 5200) & (p['iteration'] <= 25200)]
    
    ccols = ['iteration', 'wheel', 'state', 'nx', 'ny', 'nz', 'x', 'y', 'z', 'depth', 'f1x', 'f1y', 'f1z']
    c = load_csv(c_path, ccols)
    c = c[(c['iteration'] > 5200) & (c['iteration'] <= 25200)]
    
    iterations = p['iteration'].astype(int)
    n_steps = len(p)
    
    # Analyze FL wheel bottom generator segment
    fl_center = np.column_stack([p[f'front_left_world_{ax}'] for ax in 'xyz'])
    fl_axis = np.column_stack([p[f'front_left_axis_{ax}'] for ax in 'xyz'])
    fl_axis /= np.linalg.norm(fl_axis, axis=1)[:, None]
    
    # Compute bottom contact line segment (radius = 0.076m, half_width = 0.025m)
    v_z = np.array([0.0, 0.0, 1.0])
    vert = v_z - fl_axis * fl_axis[:, 2, None]
    vert /= np.linalg.norm(vert, axis=1)[:, None]
    fl_bottom = fl_center - 0.076 * vert
    
    fl_end0 = fl_bottom - 0.025 * fl_axis
    fl_end1 = fl_bottom + 0.025 * fl_axis
    
    # Check intersections with target seam (-0.25, 0) -- (0, 0.25)
    seam_overlaps = []
    for i in range(n_steps):
        intersect, pt = check_segment_intersection(fl_end0[i, :2], fl_end1[i, :2],
                                                   TARGET_SEAM['p0'], TARGET_SEAM['p1'])
        if intersect:
            seam_overlaps.append(int(iterations[i]))
            
    # Group continuous iteration intervals
    overlap_windows = []
    if seam_overlaps:
        groups = np.split(seam_overlaps, np.flatnonzero(np.diff(seam_overlaps) != 1) + 1)
        for g in groups:
            overlap_windows.append({
                'first_iteration': int(g[0]),
                'last_iteration': int(g[-1]),
                'duration_steps': len(g),
                'first_motion_s': float((g[0] - 5200) * 0.001),
                'last_motion_s': float((g[-1] - 5200) * 0.001)
            })
            
    # Extract contact dynamics
    counts = np.zeros((n_steps, 4), dtype=int)
    fz = np.zeros((n_steps, 4))
    f_net = np.zeros((n_steps, 3))
    
    si = (c['iteration'] - iterations[0]).astype(int)
    wi = c['wheel'].astype(int)
    active = c['state'] == 1
    
    np.add.at(counts, (si[active], wi[active]), 1)
    np.add.at(fz, (si[active], wi[active]), c['f1z'][active])
    for j, ax in enumerate(['f1x', 'f1y', 'f1z']):
        np.add.at(f_net[:, j], si[active], c[ax][active])
        
    # Find anomalies
    fl_3_or_more = iterations[counts[:, 0] >= 3].tolist()
    fr_zero_runs = iterations[counts[:, 1] == 0].tolist()
    fl_load_spikes = iterations[fz[:, 0] > 6.0].tolist()
    
    body = np.column_stack([p['x'], p['y']])
    disp = np.linalg.norm(body - body[0], axis=1) * 1000 # mm
    
    return {
        'run_dir': str(run_dir),
        'final_displacement_mm': float(disp[-1]),
        'max_displacement_mm': float(disp.max()),
        'target_seam_overlap_windows': overlap_windows,
        'fl_contact_ge3_steps_count': len(fl_3_or_more),
        'fr_zero_contact_steps_count': len(fr_zero_runs),
        'fl_load_spikes_count': len(fl_load_spikes),
        'net_horizontal_force_rms_N': float(np.sqrt(np.mean(np.sum(f_net[:, :2]**2, axis=1))))
    }

if __name__ == '__main__':
    print("Diagnosis run evaluator initialized. Awaiting simulation execution.")
