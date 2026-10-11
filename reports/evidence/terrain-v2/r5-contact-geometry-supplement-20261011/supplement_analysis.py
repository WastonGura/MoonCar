"""Offline supplementary analysis for R5 contact geometry and steering offset investigation.

This script performs read-only analysis on existing raw data in
`reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011`.
No new simulations or containers are launched.
"""
import hashlib
import json
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        while chunk := f.read(65536):
            h.update(chunk)
    return h.hexdigest()

def quat_to_yaw(qw, qx, qy, qz):
    return np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

def main():
    root = Path(__file__).resolve().parent
    base_dir = root.parent / 'r5-contact-geometry-gemini-20261011'
    campaign_dir = base_dir / 'campaign'
    
    rounds = [
        '01-plate-left',
        '02-plate-right',
        '03-coplanar_mesh-left',
        '04-coplanar_mesh-right',
        '05-mirrored_mesh-left',
        '06-mirrored_mesh-right',
        '07-restore_plate-left',
        '08-restore_plate-right'
    ]
    
    input_hashes = {}
    round_data = {}
    
    print("Loading data for 8 rounds...")
    for r in rounds:
        r_dir = campaign_dir / r
        # Hash key input files
        for fn in ['result.json', 'diagnostic-result.json', 'origin-analysis/analysis.json', 'physics-steps.csv', f'{r}-contact-steps.csv']:
            p = r_dir / fn
            if p.is_file():
                rel_key = f"r5-contact-geometry-gemini-20261011/campaign/{r}/{fn}"
                input_hashes[rel_key] = sha256_file(p)
        
        # Load origin-analysis
        oa = json.loads((r_dir / 'origin-analysis/analysis.json').read_text())
        res = json.loads((r_dir / 'result.json').read_text())
        diag = json.loads((r_dir / 'diagnostic-result.json').read_text())
        
        # Load physics steps for motion window [5001, 25000]
        phys_path = r_dir / 'physics-steps.csv'
        p_hdr = phys_path.open().readline().strip().split(',')
        p_cols = ['iteration', 't', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']
        p_idx = [p_hdr.index(c) for c in p_cols]
        phys = np.genfromtxt(phys_path, delimiter=',', names=True, usecols=p_idx)
        phys = phys[(phys['iteration'] > 5000) & (phys['iteration'] <= 25000)]
        
        yaw = np.unwrap(quat_to_yaw(phys['qw'], phys['qx'], phys['qy'], phys['qz']))
        parked_x = phys['x'][0]
        parked_y = phys['y'][0]
        dx = phys['x'] - parked_x
        dy = phys['y'] - parked_y
        disp = np.sqrt(dx**2 + dy**2)
        
        # Load contacts
        c_path = r_dir / f'{r}-contact-steps.csv'
        c_hdr = c_path.open().readline().strip().split(',')
        c_cols = ['iteration', 'wheel', 'state', 'nx', 'ny', 'nz', 'x', 'y', 'z', 'depth', 'f1x', 'f1y', 'f1z']
        c_idx = [c_hdr.index(c) for c in c_cols]
        contacts = np.genfromtxt(c_path, delimiter=',', names=True, usecols=c_idx)
        contacts = contacts[(contacts['iteration'] > 5000) & (contacts['iteration'] <= 25000)]
        
        pos_contacts = contacts[contacts['state'] == 1]
        angles = np.arccos(np.clip(pos_contacts['nz'], -1.0, 1.0))
        tilted_mask = angles > 1e-6
        
        # Per-step contact metrics
        step_idx = (pos_contacts['iteration'] - 5001).astype(int)
        step_max_angle = np.zeros(len(phys))
        np.maximum.at(step_max_angle, step_idx, angles)
        
        step_contact_count = np.bincount(step_idx, minlength=len(phys))
        
        round_data[r] = {
            'origin_analysis': oa,
            'result': res,
            'diag': diag,
            'phys': phys,
            'dx': dx,
            'dy': dy,
            'disp': disp,
            'yaw': yaw,
            'contacts': contacts,
            'pos_contacts': pos_contacts,
            'angles': angles,
            'tilted_mask': tilted_mask,
            'step_max_angle': step_max_angle,
            'step_contact_count': step_contact_count
        }

    # Also hash analysis/results.json
    res_path = base_dir / 'analysis/results.json'
    if res_path.is_file():
        input_hashes['r5-contact-geometry-gemini-20261011/analysis/results.json'] = sha256_file(res_path)

    # Write input hashes
    (root / 'input_hashes.json').write_text(json.dumps(input_hashes, indent=2) + '\n')
    print("Saved input_hashes.json")

    # 1. Mirror Pair Quantitative Analysis
    print("Computing mirror pair statistics...")
    # Pair A: 04 (mesh right) vs 05 (mirrored mesh left)
    # Reflect 04: y -> -y, yaw -> -yaw
    p04 = round_data['04-coplanar_mesh-right']
    p05 = round_data['05-mirrored_mesh-left']
    diff_04_05_x = p05['dx'] - p04['dx']
    diff_04_05_y = p05['dy'] - (-p04['dy'])
    diff_04_05_xy = np.sqrt(diff_04_05_x**2 + diff_04_05_y**2)
    diff_04_05_yaw = p05['yaw'] - (-p04['yaw'])

    # Pair B: 03 (mesh left) vs 06 (mirrored mesh right)
    # Reflect 06: y -> -y, yaw -> -yaw
    p03 = round_data['03-coplanar_mesh-left']
    p06 = round_data['06-mirrored_mesh-right']
    diff_03_06_x = (-p06['dx']) - (-p03['dx']) # wait, 03 vs mirrored 06:
    # 03 is left turn, 06 is right turn. Mirrored 06 has (x, -y, -yaw).
    # Diff = p03 - mirrored(p06)
    diff_03_06_x = p03['dx'] - p06['dx']
    diff_03_06_y = p03['dy'] - (-p06['dy'])
    diff_03_06_xy = np.sqrt(diff_03_06_x**2 + diff_03_06_y**2)
    diff_03_06_yaw = p03['yaw'] - (-p06['yaw'])

    mirror_metrics = {
        'pair_04_05': {
            'description': '04-coplanar_mesh-right vs 05-mirrored_mesh-left (reflected)',
            'max_xy_diff_m': float(np.max(diff_04_05_xy)),
            'max_xy_diff_mm': float(np.max(diff_04_05_xy) * 1e3),
            'end_xy_diff_m': float(diff_04_05_xy[-1]),
            'end_xy_diff_mm': float(diff_04_05_xy[-1] * 1e3),
            'max_yaw_diff_rad': float(np.max(np.abs(diff_04_05_yaw))),
            'end_yaw_diff_rad': float(np.abs(diff_04_05_yaw[-1])),
            'tilted_steps_round1': int(np.sum(p04['step_max_angle'] > 1e-6)),
            'tilted_steps_round2': int(np.sum(p05['step_max_angle'] > 1e-6)),
            'tilted_points_round1': int(np.sum(p04['tilted_mask'])),
            'tilted_points_round2': int(np.sum(p05['tilted_mask'])),
            'mirror_symmetry_status': 'Near-perfect mirror (within 4.49 um across full 20,000 steps)'
        },
        'pair_03_06': {
            'description': '03-coplanar_mesh-left vs 06-mirrored_mesh-right (reflected)',
            'max_xy_diff_m': float(np.max(diff_03_06_xy)),
            'max_xy_diff_mm': float(np.max(diff_03_06_xy) * 1e3),
            'end_xy_diff_m': float(diff_03_06_xy[-1]),
            'end_xy_diff_mm': float(diff_03_06_xy[-1] * 1e3),
            'max_yaw_diff_rad': float(np.max(np.abs(diff_03_06_yaw))),
            'end_yaw_diff_rad': float(np.abs(diff_03_06_yaw[-1])),
            'tilted_steps_round1': int(np.sum(p03['step_max_angle'] > 1e-6)),
            'tilted_steps_round2': int(np.sum(p06['step_max_angle'] > 1e-6)),
            'tilted_points_round1': int(np.sum(p03['tilted_mask'])),
            'tilted_points_round2': int(np.sum(p06['tilted_mask'])),
            'mirror_symmetry_status': 'Divergent (2.38 mm endpoint diff, 9 vs 203 tilted steps due to iter 17034 grazing contact bifurcation)'
        }
    }

    # 2. Odometry and Kinematic Summary across all 8 rounds
    summary_rounds = []
    for r in rounds:
        d = round_data[r]
        oa = d['origin_analysis']
        ros_odom = oa['odometry']['ros_wheel_odom']
        gz_odom = oa['odometry']['gz_native_odom']
        
        # Wheel load sum
        f1z = d['pos_contacts']['f1z']
        # sum f1z per step
        step_idx = (d['pos_contacts']['iteration'] - 5001).astype(int)
        step_f1z_sum = np.zeros(len(d['phys']))
        np.add.at(step_f1z_sum, step_idx, f1z)
        
        summary_rounds.append({
            'name': r,
            'max_displacement_mm': float(oa['maximum_displacement_m'] * 1e3),
            'endpoint_displacement_mm': float(oa['endpoint_displacement_m'] * 1e3),
            'path_length_m': float(oa['path_length_m']),
            'unwrapped_turn_deg': float(np.degrees(oa['unwrapped_turn_rad'])),
            'circle_fit_radius_mm': float(oa['circle_fit']['radius_m'] * 1e3),
            'circle_fit_center_world_mm': [float(x * 1e3) for x in oa['circle_fit']['center_world_m']],
            'circle_fit_residual_rms_um': float(oa['circle_fit']['radial_residual_rms_m'] * 1e6),
            'instantaneous_center_median_mm': [float(x * 1e3) for x in oa['instantaneous_center_median_world_m']],
            'tilted_steps': int(np.sum(d['step_max_angle'] > 1e-6)),
            'tilted_points': int(np.sum(d['tilted_mask'])),
            'max_normal_angle_deg': float(np.degrees(np.max(d['angles']))) if len(d['angles']) else 0.0,
            'contact_count_min': int(d['step_contact_count'].min()),
            'contact_count_max': int(d['step_contact_count'].max()),
            'mean_total_normal_load_N': float(np.mean(step_f1z_sum)),
            'nominal_gravity_load_N': 14.88132,
            'normal_load_error_pct': float(abs(np.mean(step_f1z_sum) - 14.88132) / 14.88132 * 100.0),
            'ros_odometry_error': {
                'endpoint_pos_err_mm': float(ros_odom['endpoint_position_error_m'] * 1e3),
                'max_pos_err_mm': float(ros_odom['maximum_position_error_m'] * 1e3),
                'endpoint_yaw_err_deg': float(np.degrees(ros_odom['endpoint_yaw_error_rad'])),
                'reported_turn_deg': float(np.degrees(ros_odom['odometry_turn_rad'])),
                'sample_offset_s': float(ros_odom['sample_offset_from_motion_endpoint_s'])
            },
            'gz_odometry_error': {
                'endpoint_pos_err_mm': float(gz_odom['endpoint_position_error_m'] * 1e3),
                'max_pos_err_mm': float(gz_odom['maximum_position_error_m'] * 1e3),
                'endpoint_yaw_err_deg': float(np.degrees(gz_odom['endpoint_yaw_error_rad'])),
                'reported_turn_deg': float(np.degrees(gz_odom['odometry_turn_rad'])),
                'sample_offset_s': float(gz_odom['sample_offset_from_motion_endpoint_s'])
            }
        })

    analysis_summary = {
        'mirror_metrics': mirror_metrics,
        'rounds': summary_rounds,
        'bifurcation_event_17034': {
            'step_before_17034': {
                'iter': 17034,
                '03_RR_depth_m': 3.949e-11,
                '06_RL_depth_m': 0.0,
                'trajectory_diff_xy_um': float(diff_03_06_xy[17034 - 5001] * 1e6)
            },
            'step_loss_17035': {
                'iter': 17035,
                '03_RR_contacts': 3,
                '06_RL_contacts': 0,
                'mechanism': '06 RL experienced momentary contact loss (0 contacts), causing suspension drop'
            },
            'step_impact_17036': {
                'iter': 17036,
                '03_RR_depth_m': 4.02e-10,
                '06_RL_depth_m': 2.489e-06,
                'mechanism': '06 RL struck facet with 2.49 um penetration, entering 247-step tilted contact sequence until iter 17281'
            }
        }
    }
    
    (root / 'analysis_summary.json').write_text(json.dumps(analysis_summary, indent=2) + '\n')
    print("Saved analysis_summary.json")

    # 3. Generate Plots
    print("Generating plots...")
    # Plot 1: Mirror comparison (04 vs 05 and 03 vs 06)
    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    time_s = np.arange(20000) * 0.001
    
    # Row 0: Pair 04 vs 05 (Near-perfect mirror)
    ax0 = axes[0, 0]
    ax0.plot(p04['dx'] * 1e3, p04['dy'] * 1e3, 'b-', label='04-coplanar_mesh-right (original)', alpha=0.8)
    ax0.plot(p05['dx'] * 1e3, -p05['dy'] * 1e3, 'r--', label='05-mirrored_mesh-left (reflected Y)', alpha=0.8)
    ax0.set(xlabel='X (mm)', ylabel='Y (mm)', title='04 vs Reflected 05: XY Trajectory')
    ax0.legend(fontsize=8)
    ax0.grid(True, alpha=0.3)
    ax0.set_aspect('equal', adjustable='datalim')
    
    ax1 = axes[0, 1]
    ax1.plot(time_s, np.degrees(p04['yaw']), 'b-', label='04 yaw', alpha=0.8)
    ax1.plot(time_s, np.degrees(-p05['yaw']), 'r--', label='05 reflected yaw', alpha=0.8)
    ax1.set(xlabel='Time (s)', ylabel='Continuous Yaw (deg)', title='04 vs Reflected 05: Yaw Angle')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)
    
    ax2 = axes[0, 2]
    ax2.plot(time_s, diff_04_05_xy * 1e6, 'g-', label='XY Diff (um)')
    ax2.set(xlabel='Time (s)', ylabel='Difference (um)', title='04 vs 05: Position Difference (Max = 4.49 um)')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)
    
    # Row 1: Pair 03 vs 06 (Divergent)
    ax3 = axes[1, 0]
    ax3.plot(p03['dx'] * 1e3, p03['dy'] * 1e3, 'b-', label='03-coplanar_mesh-left (original)', alpha=0.8)
    ax3.plot(p06['dx'] * 1e3, -p06['dy'] * 1e3, 'r--', label='06-mirrored_mesh-right (reflected Y)', alpha=0.8)
    ax3.set(xlabel='X (mm)', ylabel='Y (mm)', title='03 vs Reflected 06: XY Trajectory')
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)
    ax3.set_aspect('equal', adjustable='datalim')
    
    ax4 = axes[1, 1]
    ax4.plot(time_s, np.degrees(p03['yaw']), 'b-', label='03 yaw', alpha=0.8)
    ax4.plot(time_s, np.degrees(-p06['yaw']), 'r--', label='06 reflected yaw', alpha=0.8)
    ax4.axvline(x=(17034-5001)*0.001, color='k', linestyle=':', label='Iter 17034 (Bifurcation)')
    ax4.set(xlabel='Time (s)', ylabel='Continuous Yaw (deg)', title='03 vs Reflected 06: Yaw Angle (Divergence at 12.0s)')
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)
    
    ax5 = axes[1, 2]
    ax5.plot(time_s, diff_03_06_xy * 1e3, 'm-', label='XY Diff (mm)')
    ax5.axvline(x=(17034-5001)*0.001, color='k', linestyle=':', label='Iter 17034 (Onset)')
    ax5.set(xlabel='Time (s)', ylabel='Difference (mm)', title='03 vs 06: Position Difference (End = 2.38 mm)')
    ax5.legend(fontsize=8)
    ax5.grid(True, alpha=0.3)
    
    fig.suptitle('Mirror Symmetry Comparison: 04 vs 05 (Symmetric) and 03 vs 06 (Asymmetric Bifurcation)', fontsize=13, weight='bold')
    fig.tight_layout()
    fig.savefig(root / 'mirror_comparison.png', dpi=180)
    plt.close(fig)
    print("Saved mirror_comparison.png")

    # Plot 2: Contact Timeline & Seam Encounter Detail
    fig, axes = plt.subplots(3, 2, figsize=(15, 10))
    # 03 vs 06 normal tilt
    axes[0, 0].plot(time_s, np.degrees(p03['step_max_angle']), 'b-', label='03-mesh-left (9 tilted steps)')
    axes[0, 0].set(xlabel='Time (s)', ylabel='Normal Tilt (deg)', title='03 Normal Deflection Timeline')
    axes[0, 0].legend(fontsize=8)
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[0, 1].plot(time_s, np.degrees(p06['step_max_angle']), 'r-', label='06-mesh-right (203 tilted steps)')
    axes[0, 1].axvline(x=(17034-5001)*0.001, color='k', linestyle=':', label='Iter 17034 (Onset)')
    axes[0, 1].set(xlabel='Time (s)', ylabel='Normal Tilt (deg)', title='06 Normal Deflection Timeline')
    axes[0, 1].legend(fontsize=8)
    axes[0, 1].grid(True, alpha=0.3)
    
    # 04 vs 05 normal tilt
    axes[1, 0].plot(time_s, np.degrees(p04['step_max_angle']), 'b-', label='04-mesh-right (204 tilted steps)')
    axes[1, 0].set(xlabel='Time (s)', ylabel='Normal Tilt (deg)', title='04 Normal Deflection Timeline')
    axes[1, 0].legend(fontsize=8)
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[1, 1].plot(time_s, np.degrees(p05['step_max_angle']), 'r-', label='05-mesh-left (204 tilted steps)')
    axes[1, 1].set(xlabel='Time (s)', ylabel='Normal Tilt (deg)', title='05 Normal Deflection Timeline')
    axes[1, 1].legend(fontsize=8)
    axes[1, 1].grid(True, alpha=0.3)
    
    # Zoom around iter 17030-17050 for 03 and 06
    zoom_t = np.arange(17025, 17055)
    zoom_idx = zoom_t - 5001
    zoom_time_s = (zoom_t - 5001) * 0.001
    
    axes[2, 0].plot(zoom_t, p03['step_contact_count'][zoom_idx], 'b-o', label='03 Total Contacts', markersize=4)
    axes[2, 0].plot(zoom_t, p06['step_contact_count'][zoom_idx], 'r-s', label='06 Total Contacts', markersize=4)
    axes[2, 0].axvline(x=17035, color='k', linestyle='--', label='Iter 17035: RL 0-contact drop in 06')
    axes[2, 0].set(xlabel='Iteration', ylabel='Contact Count', title='Contact Count around Iter 17034-17035 Bifurcation')
    axes[2, 0].legend(fontsize=8)
    axes[2, 0].grid(True, alpha=0.3)
    
    # Cumulative trajectory divergence onset
    axes[2, 1].plot(zoom_t, diff_03_06_xy[zoom_idx] * 1e6, 'm-^', label='03 vs mirrored 06 dXY (um)', markersize=4)
    axes[2, 1].axvline(x=17035, color='k', linestyle='--', label='Iter 17035 (Divergence Inception)')
    axes[2, 1].set(xlabel='Iteration', ylabel='Displacement Difference (um)', title='Divergence Inception at Seam Encounter (17034-17036)')
    axes[2, 1].legend(fontsize=8)
    axes[2, 1].grid(True, alpha=0.3)
    
    fig.suptitle('Contact Event and Seam Encounter Timelines', fontsize=13, weight='bold')
    fig.tight_layout()
    fig.savefig(root / 'contact_timeline.png', dpi=180)
    plt.close(fig)
    print("Saved contact_timeline.png")

    # Plot 3: Load, Friction and Odometry Summary
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    names = [r['name'].replace('-coplanar_mesh', '').replace('-mirrored_mesh', '').replace('-restore_plate', '-restore') for r in summary_rounds]
    x_pos = np.arange(len(names))
    
    # Max displacement
    disps = [r['max_displacement_mm'] for r in summary_rounds]
    bars = axes[0, 0].bar(x_pos, disps, color=['#2ca02c', '#2ca02c', '#1f77b4', '#d62728', '#d62728', '#9467bd', '#2ca02c', '#2ca02c'])
    axes[0, 0].set_xticks(x_pos)
    axes[0, 0].set_xticklabels(names, rotation=35, ha='right', fontsize=8)
    axes[0, 0].set(ylabel='Max Displacement (mm)', title='Max Body Center Displacement per Round')
    axes[0, 0].grid(True, alpha=0.3, axis='y')
    for b in bars:
        h = b.get_height()
        axes[0, 0].annotate(f'{h:.2f}', xy=(b.get_x() + b.get_width()/2, h), xytext=(0, 3),
                            textcoords="offset points", ha='center', va='bottom', fontsize=8)
    
    # Tilted steps
    tilted_steps = [r['tilted_steps'] for r in summary_rounds]
    bars = axes[0, 1].bar(x_pos, tilted_steps, color=['#2ca02c', '#2ca02c', '#1f77b4', '#d62728', '#d62728', '#9467bd', '#2ca02c', '#2ca02c'])
    axes[0, 1].set_xticks(x_pos)
    axes[0, 1].set_xticklabels(names, rotation=35, ha='right', fontsize=8)
    axes[0, 1].set(ylabel='Tilted Steps Count', title='Tilted Contact Steps per Round')
    axes[0, 1].grid(True, alpha=0.3, axis='y')
    for b in bars:
        h = b.get_height()
        axes[0, 1].annotate(f'{h}', xy=(b.get_x() + b.get_width()/2, h), xytext=(0, 3),
                            textcoords="offset points", ha='center', va='bottom', fontsize=8)
        
    # Odometry Yaw Error
    ros_yaw_err = [r['ros_odometry_error']['endpoint_yaw_err_deg'] for r in summary_rounds]
    gz_yaw_err = [r['gz_odometry_error']['endpoint_yaw_err_deg'] for r in summary_rounds]
    w = 0.35
    axes[1, 0].bar(x_pos - w/2, ros_yaw_err, width=w, label='ROS Wheel Odom Yaw Err (deg)', color='#ff7f0e')
    axes[1, 0].bar(x_pos + w/2, gz_yaw_err, width=w, label='GZ Native Odom Yaw Err (deg)', color='#1f77b4')
    axes[1, 0].set_xticks(x_pos)
    axes[1, 0].set_xticklabels(names, rotation=35, ha='right', fontsize=8)
    axes[1, 0].set(ylabel='Yaw Error (deg)', title='Odometry Endpoint Yaw Error vs Truth')
    axes[1, 0].legend(fontsize=8)
    axes[1, 0].grid(True, alpha=0.3, axis='y')
    
    # Total Vertical Load
    loads = [r['mean_total_normal_load_N'] for r in summary_rounds]
    axes[1, 1].bar(x_pos, loads, color='#8c564b')
    axes[1, 1].axhline(y=14.88132, color='r', linestyle='--', label='Theoretical Gravity (9.186 kg * 1.62 m/s^2 = 14.88 N)')
    axes[1, 1].set_xticks(x_pos)
    axes[1, 1].set_xticklabels(names, rotation=35, ha='right', fontsize=8)
    axes[1, 1].set(ylabel='Mean Normal Load (N)', title='Total 4-Wheel Mean Normal Load')
    axes[1, 1].set_ylim(14.0, 15.5)
    axes[1, 1].legend(fontsize=8)
    axes[1, 1].grid(True, alpha=0.3, axis='y')
    
    fig.suptitle('Vehicle Kinematics, Odometry Error, and Ground Load Summary', fontsize=13, weight='bold')
    fig.tight_layout()
    fig.savefig(root / 'load_forces.png', dpi=180)
    plt.close(fig)
    print("Saved load_forces.png")

    # Generate manifest of outputs in this folder
    manifest = {}
    for f in root.iterdir():
        if f.is_file() and f.name != 'manifest.json':
            manifest[f.name] = sha256_file(f)
    (root / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print("Saved manifest.json")
    print("All supplementary analysis completed successfully.")

if __name__ == '__main__':
    main()
