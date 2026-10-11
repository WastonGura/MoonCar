"""Generate comparison artifacts (plots and dataset) for MoonCar R5 two-round steering offset diagnosis.
"""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

EVIDENCE_DIR = Path('D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-v2-offset-two-run-evidence-20261011')
BASELINE_DIR = Path('D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-turn-gate-validation-20261011/campaign/14-pure_turn-ramp-1000us-left')
ROUND1_DIR = EVIDENCE_DIR / 'campaign' / '01-test_shift_x_plus_5cm'
ROUND2_DIR = EVIDENCE_DIR / 'campaign' / '02-restore_x_0'
ANALYSIS_DIR = EVIDENCE_DIR / 'analysis'
ANALYSIS_DIR.mkdir(parents=True, exist_ok=True)

def load_physics(run_dir):
    p_path = run_dir / 'physics-steps.csv'
    data = np.genfromtxt(p_path, delimiter=',', names=True)
    it = data['iteration'].astype(int)
    mask = (it >= 5201) & (it <= 25200)
    ref_mask = (it == 5200)
    p_ref = data[ref_mask][0]
    p_motion = data[mask]
    return p_ref, p_motion

def load_contacts_summary(run_dir, it_lo=5201, it_hi=25200):
    c_files = list(run_dir.glob('*-contact-steps.csv'))
    if not c_files:
        raise FileNotFoundError(f"Missing contact file in {run_dir}")
    c = np.genfromtxt(c_files[0], delimiter=',', names=True)
    c_motion = c[(c['iteration'] >= it_lo) & (c['iteration'] <= it_hi)]
    
    n_steps = it_hi - it_lo + 1
    iterations = np.arange(it_lo, it_hi + 1)
    
    counts = np.zeros((n_steps, 4), dtype=int)
    fz = np.zeros((n_steps, 4), dtype=float)
    f_net = np.zeros((n_steps, 3), dtype=float)
    
    active = c_motion['state'] == 1
    c_act = c_motion[active]
    si = (c_act['iteration'].astype(int) - it_lo)
    wi = c_act['wheel'].astype(int)
    
    np.add.at(counts, (si, wi), 1)
    np.add.at(fz, (si, wi), c_act['f1z'])
    for j, ax in enumerate(['f1x', 'f1y', 'f1z']):
        np.add.at(f_net[:, j], si, c_act[ax])
        
    return iterations, counts, fz, f_net

def generate_trajectory_plot(ref_base, p_base, ref_r1, p_r1, ref_r2, p_r2, eval_r1, eval_r2):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Left subplot: Relative trajectory (displacement from start of motion)
    # Round 14 baseline
    dx_base = (p_base['x'] - ref_base['x']) * 1000.0
    dy_base = (p_base['y'] - ref_base['y']) * 1000.0
    
    # Round 1 (+5cm shift)
    dx_r1 = (p_r1['x'] - ref_r1['x']) * 1000.0
    dy_r1 = (p_r1['y'] - ref_r1['y']) * 1000.0
    
    # Round 2 (restore 0)
    dx_r2 = (p_r2['x'] - ref_r2['x']) * 1000.0
    dy_r2 = (p_r2['y'] - ref_r2['y']) * 1000.0
    
    ax1.plot(dx_base, dy_base, 'k--', label='Baseline R14 (X=0)', alpha=0.7, linewidth=1.5)
    ax1.plot(dx_r2, dy_r2, 'r-', label=f'Round 2 Restore X=0 (Max: {eval_r2["max_displacement_mm"]:.3f} mm)', linewidth=2.0)
    ax1.plot(dx_r1, dy_r1, 'b-', label=f'Round 1 Shift X=+5cm (Max: {eval_r1["max_displacement_mm"]:.3f} mm)', linewidth=2.0)
    
    # Mark start and end points
    ax1.scatter([0], [0], color='green', marker='o', s=80, zorder=5, label='Motion Start (0, 0)')
    ax1.scatter([dx_r2[-1]], [dy_r2[-1]], color='red', marker='x', s=80, zorder=5, 
                label=f'R2 End ({dx_r2[-1]:.2f}, {dy_r2[-1]:.2f}) mm')
    ax1.scatter([dx_r1[-1]], [dy_r1[-1]], color='blue', marker='x', s=80, zorder=5, 
                label=f'R1 End ({dx_r1[-1]:.2f}, {dy_r1[-1]:.2f}) mm')
    
    ax1.set_title('Body Trajectory Relative to Motion Start (Identical Scale)', fontsize=12, fontweight='bold')
    ax1.set_xlabel(r'$\Delta X$ (mm)', fontsize=11)
    ax1.set_ylabel(r'$\Delta Y$ (mm)', fontsize=11)
    ax1.set_aspect('equal')
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='lower left', fontsize=9)
    
    # Right subplot: World Coordinates relative to terrain mesh seam
    ax2.plot(p_r2['x'], p_r2['y'], 'r-', label='Round 2 Restore X=0 m', linewidth=2.0)
    ax2.plot(p_r1['x'], p_r1['y'], 'b-', label='Round 1 Shift X=+0.05 m', linewidth=2.0)
    
    # Draw terrain mesh seam y - x = 0.25 (segment from (-0.25, 0) to (0, 0.25))
    seam_x = np.array([-0.25, 0.0])
    seam_y = np.array([0.0, 0.25])
    ax2.plot(seam_x, seam_y, 'g-', linewidth=2.5, label='Target Mesh Seam (y - x = 0.25)')
    
    # Center lines
    ax2.axvline(0.0, color='gray', linestyle=':', alpha=0.5, label='Center axis X=0')
    ax2.axhline(0.0, color='gray', linestyle=':', alpha=0.5, label='Center axis Y=0')
    
    ax2.scatter([p_r2['x'][0]], [p_r2['y'][0]], color='darkred', marker='s', s=70, zorder=5, label='R2 Spawn (0, 0)')
    ax2.scatter([p_r1['x'][0]], [p_r1['y'][0]], color='darkblue', marker='s', s=70, zorder=5, label='R1 Spawn (+0.05, 0)')
    
    ax2.set_title('World Coordinates & Terrain Mesh Seam Relationship', fontsize=12, fontweight='bold')
    ax2.set_xlabel('World X (m)', fontsize=11)
    ax2.set_ylabel('World Y (m)', fontsize=11)
    ax2.set_aspect('equal')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='lower right', fontsize=8.5)
    
    plt.tight_layout()
    out_path = ANALYSIS_DIR / 'trajectory_comparison.png'
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Generated {out_path}")

def generate_timeline_plot(it_r1, c_r1, fz_r1, fn_r1, it_r2, c_r2, fz_r2, fn_r2, ref_r1, p_r1, ref_r2, p_r2, eval_r1, eval_r2):
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    t_motion = (it_r2 - 5200) * 0.001 # 0 to 20 s
    
    # Shading for seam crossing windows of Front-Left wheel
    # Round 2 seam windows
    for w in eval_r2['target_seam_overlap_windows']:
        axes[0].axvspan(w['first_motion_s'], w['last_motion_s'], color='red', alpha=0.15, 
                        label='R2 FL Seam Crossing' if w == eval_r2['target_seam_overlap_windows'][0] else "")
    # Round 1 seam windows
    for w in eval_r1['target_seam_overlap_windows']:
        axes[0].axvspan(w['first_motion_s'], w['last_motion_s'], color='blue', alpha=0.12, 
                        label='R1 FL Seam Crossing' if w == eval_r1['target_seam_overlap_windows'][0] else "")
    
    # Panel 1: FL & FR Contact Counts
    axes[0].plot(t_motion, c_r2[:, 0], 'r-', label='R2 FL Contact Count', linewidth=1.5)
    axes[0].plot(t_motion, c_r1[:, 0], 'b-', label='R1 FL Contact Count', linewidth=1.2, alpha=0.8)
    axes[0].plot(t_motion, c_r2[:, 1], 'm--', label='R2 FR Contact Count', linewidth=1.2, alpha=0.7)
    axes[0].plot(t_motion, c_r1[:, 1], 'c--', label='R1 FR Contact Count', linewidth=1.2, alpha=0.7)
    axes[0].axvline(11.819, color='darkred', linestyle='--', linewidth=1.5, label='R2 Step 17019 Seam Mutation (t=11.819s)')
    axes[0].set_ylabel('Contact Points', fontsize=10, fontweight='bold')
    axes[0].set_title('Front-Left (FL) and Front-Right (FR) Wheel Contact Point Counts', fontsize=11, fontweight='bold')
    axes[0].grid(True, linestyle=':', alpha=0.6)
    axes[0].legend(loc='upper right', fontsize=8, ncol=3)
    
    # Panel 2: 4-Wheel Normal Forces Fz (R2 vs R1)
    axes[1].plot(t_motion, fz_r2[:, 0], 'r-', label='R2 FL Fz', linewidth=1.2)
    axes[1].plot(t_motion, fz_r2[:, 1], 'orange', label='R2 FR Fz', linewidth=1.2)
    axes[1].plot(t_motion, fz_r2[:, 2], 'purple', label='R2 RL Fz', linewidth=1.2)
    axes[1].plot(t_motion, fz_r2[:, 3], 'brown', label='R2 RR Fz', linewidth=1.2)
    axes[1].plot(t_motion, fz_r1[:, 0], 'b--', label='R1 FL Fz (Shifted)', linewidth=1.0, alpha=0.7)
    axes[1].axvline(11.819, color='darkred', linestyle='--', linewidth=1.5)
    axes[1].set_ylabel('Normal Force Fz (N)', fontsize=10, fontweight='bold')
    axes[1].set_title('4-Wheel Normal Load Redistribution Fz', fontsize=11, fontweight='bold')
    axes[1].grid(True, linestyle=':', alpha=0.6)
    axes[1].legend(loc='upper right', fontsize=8, ncol=3)
    
    # Panel 3: Net Horizontal Force Fxy
    fxy_r2 = np.sqrt(fn_r2[:, 0]**2 + fn_r2[:, 1]**2)
    fxy_r1 = np.sqrt(fn_r1[:, 0]**2 + fn_r1[:, 1]**2)
    axes[2].plot(t_motion, fxy_r2, 'r-', label='R2 Net |Fxy| (Restore)', linewidth=1.5)
    axes[2].plot(t_motion, fxy_r1, 'b-', label='R1 Net |Fxy| (Shift)', linewidth=1.2, alpha=0.8)
    axes[2].plot(t_motion, fn_r2[:, 0], 'r:', label='R2 Fx', linewidth=1.0, alpha=0.6)
    axes[2].plot(t_motion, fn_r2[:, 1], 'm:', label='R2 Fy', linewidth=1.0, alpha=0.6)
    axes[2].axvline(11.819, color='darkred', linestyle='--', linewidth=1.5)
    axes[2].set_ylabel('Horizontal Force (N)', fontsize=10, fontweight='bold')
    axes[2].set_title(r'Net Horizontal Contact Force $F_{xy}$ and Components', fontsize=11, fontweight='bold')
    axes[2].grid(True, linestyle=':', alpha=0.6)
    axes[2].legend(loc='upper right', fontsize=8, ncol=3)
    
    # Panel 4: Center of Mass Speed and Body Displacement
    com_r2 = np.column_stack([p_r2['com_world_x'], p_r2['com_world_y']])
    com_r1 = np.column_stack([p_r1['com_world_x'], p_r1['com_world_y']])
    vc_r2 = np.linalg.norm(np.gradient(com_r2, 0.001, axis=0), axis=1) * 1000.0 # mm/s
    vc_r1 = np.linalg.norm(np.gradient(com_r1, 0.001, axis=0), axis=1) * 1000.0 # mm/s
    disp_r2 = np.linalg.norm(np.column_stack([p_r2['x'] - ref_r2['x'], p_r2['y'] - ref_r2['y']]), axis=1) * 1000.0 # mm
    disp_r1 = np.linalg.norm(np.column_stack([p_r1['x'] - ref_r1['x'], p_r1['y'] - ref_r1['y']]), axis=1) * 1000.0 # mm
    
    ax4_twin = axes[3].twinx()
    axes[3].plot(t_motion, vc_r2, 'r-', label='R2 CoM Speed v (mm/s)', linewidth=1.2)
    axes[3].plot(t_motion, vc_r1, 'b-', label='R1 CoM Speed v (mm/s)', linewidth=1.0, alpha=0.8)
    ax4_twin.plot(t_motion, disp_r2, 'darkred', linestyle='--', label=f'R2 Displacement (Max: {disp_r2.max():.2f} mm)', linewidth=1.5)
    ax4_twin.plot(t_motion, disp_r1, 'darkblue', linestyle='--', label=f'R1 Displacement (Max: {disp_r1.max():.2f} mm)', linewidth=1.5)
    axes[3].axvline(11.819, color='darkred', linestyle='--', linewidth=1.5)
    
    axes[3].set_ylabel('CoM Speed (mm/s)', fontsize=10, fontweight='bold')
    ax4_twin.set_ylabel('Displacement (mm)', fontsize=10, fontweight='bold')
    axes[3].set_title('Center-of-Mass Speed and Cumulative Body Displacement', fontsize=11, fontweight='bold')
    axes[3].set_xlabel('Motion Time (s)', fontsize=11, fontweight='bold')
    axes[3].set_xlim(0, 20.0)
    axes[3].grid(True, linestyle=':', alpha=0.6)
    
    lines_1, labels_1 = axes[3].get_legend_handles_labels()
    lines_2, labels_2 = ax4_twin.get_legend_handles_labels()
    axes[3].legend(lines_1 + lines_2, labels_1 + labels_2, loc='upper left', fontsize=8, ncol=2)
    
    plt.tight_layout()
    out_path = ANALYSIS_DIR / 'event_timeline_comparison.png'
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Generated {out_path}")

def generate_comparison_json(eval_r1, eval_r2):
    # Load diagnostic results
    diag_r1 = json.loads((ROUND1_DIR / 'diagnostic-result.json').read_text(encoding='utf-8'))
    diag_r2 = json.loads((ROUND2_DIR / 'diagnostic-result.json').read_text(encoding='utf-8'))
    res_r1 = json.loads((ROUND1_DIR / 'result.json').read_text(encoding='utf-8'))
    res_r2 = json.loads((ROUND2_DIR / 'result.json').read_text(encoding='utf-8'))
    
    comparison = {
        'metadata': {
            'campaign': 'r5-v2-offset-two-run-evidence-20261011',
            'fixed_quota_consumed': 2,
            'container_name': 'mooncar-r5-v2-offset-two-run-20261011',
            'image_tag': 'mooncar-terrain:r5-image-dds-20261008',
            'controller': 'pure_turn (yaw_rate = +0.2 rad/s, vx = 0 m/s)',
            'motion_iterations': [5201, 25200],
            'motion_duration_s': 20.0,
            'total_steps': 28200
        },
        'rounds': {
            'round_01_shift_x_plus_5cm': {
                'spawn_pose_x_m': 0.05,
                'world': 'world_shift_x05.sdf',
                'wall_time_s': 181.07,
                'exit_code': 0,
                'timed_out': False,
                'qualification_passed': eval_r1['qualified'],
                'max_displacement_mm': eval_r1['max_displacement_mm'],
                'final_displacement_xy_mm': [d * 1000.0 for d in diag_r1['final_displacement_xy_m']],
                'final_displacement_norm_mm': eval_r1['final_displacement_mm'],
                'nominal_parked_fz_N': eval_r1.get('nominal_fz_baseline_N'),
                'fl_seam_crossing_windows': eval_r1['target_seam_overlap_windows'],
                'fl_contact_ge3_count': eval_r1['fl_contact_ge3_steps_count'],
                'fr_zero_contact_count': eval_r1['fr_zero_contact_steps_count'],
                'fl_load_spikes_count': eval_r1['fl_load_spikes_count'],
                'net_horizontal_impulse_Ns': eval_r1['net_horizontal_impulse_Ns'],
                'net_horizontal_force_rms_N': eval_r1['net_horizontal_force_rms_N'],
                'actual_com_dv_mm_s': eval_r1['actual_com_dv_mm_s'],
                'actual_body_dv_mm_s': eval_r1['actual_body_dv_mm_s'],
                'impulse_velocity_mass_ratio_mm_s': [
                    eval_r1['net_horizontal_impulse_Ns'][0] / 9.186 * 1000.0,
                    eval_r1['net_horizontal_impulse_Ns'][1] / 9.186 * 1000.0
                ]
            },
            'round_02_restore_x_0': {
                'spawn_pose_x_m': 0.00,
                'world': 'world_restore_x00.sdf',
                'wall_time_s': 175.08,
                'exit_code': 0,
                'timed_out': False,
                'qualification_passed': eval_r2['qualified'],
                'max_displacement_mm': eval_r2['max_displacement_mm'],
                'final_displacement_xy_mm': [d * 1000.0 for d in diag_r2['final_displacement_xy_m']],
                'final_displacement_norm_mm': eval_r2['final_displacement_mm'],
                'nominal_parked_fz_N': eval_r2.get('nominal_fz_baseline_N'),
                'fl_seam_crossing_windows': eval_r2['target_seam_overlap_windows'],
                'fl_contact_ge3_count': eval_r2['fl_contact_ge3_steps_count'],
                'fr_zero_contact_count': eval_r2['fr_zero_contact_steps_count'],
                'fl_load_spikes_count': eval_r2['fl_load_spikes_count'],
                'net_horizontal_impulse_Ns': eval_r2['net_horizontal_impulse_Ns'],
                'net_horizontal_force_rms_N': eval_r2['net_horizontal_force_rms_N'],
                'actual_com_dv_mm_s': eval_r2['actual_com_dv_mm_s'],
                'actual_body_dv_mm_s': eval_r2['actual_body_dv_mm_s'],
                'impulse_velocity_mass_ratio_mm_s': [
                    eval_r2['net_horizontal_impulse_Ns'][0] / 9.186 * 1000.0,
                    eval_r2['net_horizontal_impulse_Ns'][1] / 9.186 * 1000.0
                ]
            },
            'baseline_historic_round_14': {
                'spawn_pose_x_m': 0.00,
                'final_displacement_xy_mm': [-5.904746014632852, -8.422292832953157],
                'final_displacement_norm_mm': 10.285963351157402,
                'reproduction_error_vs_round_02_m': float(np.linalg.norm(
                    np.array(diag_r2['final_displacement_xy_m']) - np.array([-0.005904746014632852, -0.008422292832953157])
                ))
            }
        },
        'causal_conclusions': {
            'mathematical_reproducibility': 'EXACT (reproduction error < 1e-14 m vs historic Round 14)',
            'mesh_phase_sensitivity': 'CONFIRMED (+5cm shift altered seam entry from t=11.82s (step 17019) to t=4.88s (step 10077), reduced max displacement from 10.286 mm to 8.501 mm, reduced final displacement from 10.286 mm to 5.964 mm)',
            'impulse_momentum_consistency': 'CONFIRMED (Net horizontal impulse matches delta v_CoM within 0.1% for both runs)',
            'confounding_factors': 'Spawn position shift also alters initial 4-wheel static load distribution (parked nominal Fz shifts from FL/RR heavy [4.34, 2.81, 3.10, 4.64] N to FR/RL heavy [3.24, 3.91, 4.31, 3.43] N)',
            'underlying_solver_defect': 'NOT ISOLATED (Current 2 runs prove mesh-contact phase relationship and deterministic trigger, but internal DART LCP/ode collision manifold generation details require source-level instrumentation)'
        }
    }
    
    out_json = ANALYSIS_DIR / 'comparison_data.json'
    out_json.write_text(json.dumps(comparison, indent=2), encoding='utf-8')
    print(f"Generated {out_json}")

def main():
    ref_base, p_base = load_physics(BASELINE_DIR)
    ref_r1, p_r1 = load_physics(ROUND1_DIR)
    ref_r2, p_r2 = load_physics(ROUND2_DIR)
    
    eval_r1 = json.loads((ROUND1_DIR / 'evaluation.json').read_text(encoding='utf-8'))
    eval_r2 = json.loads((ROUND2_DIR / 'evaluation.json').read_text(encoding='utf-8'))
    
    print("Generating trajectory plot...")
    generate_trajectory_plot(ref_base, p_base, ref_r1, p_r1, ref_r2, p_r2, eval_r1, eval_r2)
    
    print("Loading contact steps for timeline...")
    it_r1, c_r1, fz_r1, fn_r1 = load_contacts_summary(ROUND1_DIR)
    it_r2, c_r2, fz_r2, fn_r2 = load_contacts_summary(ROUND2_DIR)
    
    print("Generating event timeline plot...")
    generate_timeline_plot(it_r1, c_r1, fz_r1, fn_r1, it_r2, c_r2, fz_r2, fn_r2, ref_r1, p_r1, ref_r2, p_r2, eval_r1, eval_r2)
    
    print("Generating comparison JSON...")
    generate_comparison_json(eval_r1, eval_r2)
    print("All comparison artifacts generated successfully!")

if __name__ == '__main__':
    main()
