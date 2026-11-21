#!/usr/bin/env python3
"""Four-way V3 8-minute Gazebo autonomy comparison analyzer.

Evaluates:
  1. coverage_v3_gimbal : coverage_view_v3 (gimbal enabled)
  2. surface_v3_gimbal  : surface_view_v3  (gimbal enabled)
  3. hybrid_v4_fixed    : hybrid_frontier_v4 (gimbal locked / fixed)
  4. hybrid_v4_gimbal   : hybrid_frontier_v4 (gimbal enabled)

Reuses the audited V3 ground-truth elevation reference (map_reference.py),
per-minute causal scoring (analyze_duration.py), and multi-segment archive
counting (v3_camera_comparison_20261118/analyze.py).
"""
import argparse
from collections import Counter
import io
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np

HERE = Path(__file__).resolve().parent
REPORTS = HERE.parent
sys.path.insert(0, str(REPORTS / 'v3_mapping_comparison_20261117'))
sys.path.insert(0, str(REPORTS / 'v3_eight_minute_comparison_20261117'))

from map_reference import build as build_reference, compare as compare_map  # noqa: E402
from analyze_duration import analyze as analyze_duration_run, read, rows, write  # noqa: E402


GROUPS = (
    dict(key='coverage_v3_gimbal', label='coverage_v3 (云台)', short='coverage_v3\n(云台)',
         strategy='coverage_view_v3', camera_mode='gimbal', color='#5aa9ff'),
    dict(key='surface_v3_gimbal', label='surface_v3 (云台)', short='surface_v3\n(云台)',
         strategy='surface_view_v3', camera_mode='gimbal', color='#ff9f43'),
    dict(key='hybrid_v4_fixed', label='hybrid_v4 (无云台)', short='hybrid_v4\n(无云台)',
         strategy='hybrid_frontier_v4', camera_mode='fixed', color='#c58af9'),
    dict(key='hybrid_v4_gimbal', label='hybrid_v4 (有云台)', short='hybrid_v4\n(有云台)',
         strategy='hybrid_frontier_v4', camera_mode='gimbal', color='#2ed573'),
)


def archive_counts(path, mission_start, mission_end):
    frames = points = mission_frames = mission_points = 0
    first_part_end = last_stamp = None
    with zipfile.ZipFile(path) as bundle:
        members = [name for name in sorted(bundle.namelist()) if name.startswith('segments/')]
        for name in members or [None]:
            source = io.BytesIO(bundle.read(name)) if name is not None else path
            with np.load(source, allow_pickle=False) as data:
                stamps, offsets = data['stamps'], data['offsets']
                assert len(stamps) + 1 == len(offsets)
                frames += len(stamps)
                points += int(offsets[-1])
                if len(stamps):
                    last_stamp = float(stamps[-1])
                    if first_part_end is None:
                        first_part_end = last_stamp
                begin = np.searchsorted(stamps, mission_start)
                end = np.searchsorted(stamps, mission_end, side='right')
                mission_frames += int(end - begin)
                mission_points += int(offsets[end] - offsets[begin])
    return dict(
        frames=frames,
        raw_points=points,
        segments=len(members) or 1,
        mission_frames=mission_frames,
        mission_raw_points=mission_points,
        first_part_end_sim=first_part_end,
        last_stamp_sim=last_stamp,
    )


def summarize_trial(raw, group, reference):
    name = group['key']
    folder = raw / name
    result = read(folder / 'case/result.json')
    mission = result['autonomy']
    protocol = read(folder / 'protocol.json')
    end = mission['started'] + mission['elapsed_seconds']

    terminal_wall = math.inf
    with (folder / 'case/states.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            if not line.strip():
                continue
            row = json.loads(line)
            survey = row['state'].get('autonomy', {})
            if (survey.get('started') == mission['started']
                    and not survey.get('active')
                    and survey.get('phase') not in (None, 'idle')):
                terminal_wall = min(terminal_wall, row['wall'])
    if not math.isfinite(terminal_wall):
        raise ValueError(f'{name}: no terminal mission state receipt')

    snapshots = []
    map_path = folder / 'case/observer/survey-map.jsonl'
    with map_path.open('rb') as stream:
        while True:
            offset = stream.tell()
            line = stream.readline()
            if not line:
                break
            row = json.loads(line)
            if row['wall'] <= terminal_wall and row['payload']['stamp'] <= end + 1e-6:
                snapshots.append(dict(
                    wall=row['wall'], sim=row['sim'], offset=offset,
                    stamp=row['payload']['stamp'],
                    observed_m2=len(row['payload']['cells']) * (row['payload']['resolution'] ** 2),
                ))
    if not snapshots:
        raise ValueError(f'{name}: no survey map snapshots found')

    with map_path.open('rb') as stream:
        stream.seek(snapshots[-1]['offset'])
        final_payload = json.loads(stream.readline())['payload']
    final_maps, stats = compare_map(final_payload, reference)

    initial = next((r for r in reversed(snapshots) if r['sim'] is not None and r['sim'] <= mission['started']), None)
    baseline_area = initial['observed_m2'] if initial else None

    alignment = rows(folder / 'case/observer/alignment.jsonl')
    errors = [r['xy_error_m'] for r in alignment if mission['started'] <= r['stamp'] <= end]
    truth = [r for r in rows(folder / 'case/observer/truth.jsonl') if mission['started'] <= r['stamp'] <= end]
    trail, prev = [], -math.inf
    for row in truth:
        if row['stamp'] - prev >= 0.1:
            trail.append(row['xyz'][:2])
            prev = row['stamp']

    history = mission.get('history', [])
    chassis_spins = 0
    gimbal_scans = 0
    move_actions = 0
    global_switches = 0
    for act in history:
        kind = act.get('kind', '')
        used_gimbal = bool(act.get('use_pan_tilt'))
        if act.get('global_frontier_target') or act.get('candidate_diagnostics', {}).get('global_frontier_fallback'):
            global_switches += 1
        if kind == 'turn':
            if used_gimbal:
                gimbal_scans += 1
            else:
                chassis_spins += 1
        else:
            move_actions += 1
            if used_gimbal:
                gimbal_scans += 1
            elif act.get('requires_chassis_spin', True) and act.get('strategy') != 'hybrid_frontier_v4':
                # v2/v3 moves execute a chassis turn after translation when heading differs
                if abs(float(act.get('pan', 0.0))) > 0.25 or True:
                    pass
            elif act.get('requires_chassis_spin'):
                chassis_spins += 1

    # Count actual Nav2 Spin dispatches from states/actions or history turn_executed markers
    actual_spin_actions = 0
    prev_phase = None
    with (folder / 'case/states.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            if not line.strip():
                continue
            row = json.loads(line)
            sim = row['state']['motion'].get('diagnostics', {}).get('sim_stamp')
            if sim is None or not (mission['started'] <= sim <= end):
                continue
            phase = row['state'].get('autonomy', {}).get('phase')
            if phase == 'turning' and prev_phase != 'turning':
                actual_spin_actions += 1
            prev_phase = phase

    diag_samples = []
    for row in rows(folder / 'case/states.jsonl'):
        motion = row['state']['motion'].get('diagnostics', {})
        stamp = motion.get('gimbal_joint_stamp_sim')
        pos = motion.get('gimbal_position')
        if pos is not None and stamp is not None and mission['started'] <= stamp <= end:
            diag_samples.append((stamp, math.degrees(pos[0]), math.degrees(pos[1])))

    session = raw / f'{name}-sessions' / result['experiment']
    archives = sorted(session.glob('survey-*.npz'))
    archive = archive_counts(archives[-1], mission['started'], stats['stamp']) if archives else None

    retries = Counter()
    plan_status = Counter()
    longest_gap = gap = 0
    win_log = session / 'window-localization.jsonl'
    if win_log.is_file():
        for row in rows(win_log):
            if row.get('type') != 'plan' or not (mission['started'] <= row['planned_stamp_ns'] / 1e9 <= end):
                continue
            if row.get('output'):
                plan_status['solved'] += 1
                gap = 0
            else:
                plan_status[row.get('reason', 'unknown')] += 1
                gap += 1
                longest_gap = max(longest_gap, gap)
            retry = (row.get('frontend') or {}).get('metric_spread_retry')
            if retry is not None:
                retries['attempted'] += 1
                retries['geometry_accepted'] += int(retry['accepted'])
                retries['solved'] += int(bool(row.get('output')))

    capture_path = folder / 'video/recording.json'
    capture = read(capture_path) if capture_path.is_file() else None
    exits = {
        k: read(folder / f'{k}-exit.json')['exit_code']
        for k in ('driver', 'video', 'backend')
        if (folder / f'{k}-exit.json').is_file()
    }

    wall_s = float(result['elapsed_wall_s'])
    elapsed_s = float(mission['elapsed_seconds'])

    stats.update(
        key=name,
        label=group['label'],
        strategy=name,
        terrain_strategy=protocol['terrain_strategy'],
        camera_mode=protocol.get('camera_mode'),
        autonomy_strategy=mission.get('strategy'),
        experiment=result['experiment'],
        tested_commit=result['software']['git_commit'],
        outcome=result['outcome'],
        reason=mission.get('reason'),
        budget_seconds=float(mission['budget_seconds']),
        elapsed_seconds=elapsed_s,
        wall_seconds=wall_s,
        realtime_factor=elapsed_s / wall_s if wall_s > 0 else None,
        radius=mission.get('radius'),
        terminal_receipt_wall=terminal_wall,
        snapshot_receipt_wall=snapshots[-1]['wall'],
        initial_received_observed_m2=baseline_area,
        new_observed_since_start_m2=(stats['total_observed_m2'] - baseline_area) if baseline_area is not None else None,
        actions=len(history),
        successful_actions=sum(bool(a.get('success')) for a in history),
        move_actions=move_actions,
        chassis_turn_phases=actual_spin_actions,
        gimbal_selected_actions=gimbal_scans,
        global_frontier_switches=global_switches,
        localization_pauses=len(mission.get('localization_pauses', [])),
        premature_ready_samples=result.get('premature_ready_samples', 0),
        path_length_m=sum(math.dist(a, b) for a, b in zip(trail, trail[1:])),
        max_distance_from_start_m=max((math.dist(p, mission['origin']) for p in trail), default=0.0),
        localization_xy_p95_m=float(np.percentile(errors, 95)) if errors else None,
        localization_xy_max_m=float(max(errors)) if errors else None,
        gimbal=dict(
            samples=len(set(s for s, _, _ in diag_samples)),
            pan_range_deg=[round(min(p for _, p, _ in diag_samples), 2), round(max(p for _, p, _ in diag_samples), 2)] if diag_samples else None,
            tilt_range_deg=[round(min(t for _, _, t in diag_samples), 2), round(max(t for _, _, t in diag_samples), 2)] if diag_samples else None,
            startup_actions=len(mission.get('initial_observation', {}).get('history', [])),
            frontier_actions=len(mission.get('frontier_observations', [])),
        ),
        localization=dict(
            plan_status=dict(plan_status),
            longest_no_solution_frames=longest_gap,
            spread_retry=dict(retries),
        ),
        archive=archive,
        capture=capture,
        process_exits=exits,
    )
    return stats, final_maps


def plot_charts(details, output_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    for fp in (
        'C:/Windows/Fonts/msyh.ttc',
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
    ):
        if Path(fp).is_file():
            font_manager.fontManager.addfont(fp)
            prop = font_manager.FontProperties(fname=fp)
            plt.rcParams['font.family'] = prop.get_name()
            break
    plt.rcParams['axes.unicode_minus'] = False

    # 1. Coverage progression curves (Observed + Valid Elevation)
    fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.2), dpi=180)
    for g, det in zip(GROUPS, details):
        pts = det['minute_metrics']
        ts = [p['elapsed_seconds'] / 60.0 for p in pts]
        obs = [p['total_observed_m2'] for p in pts]
        val = [p['valid_surface_m2'] for p in pts]
        lw = 2.8 if g['key'] == 'hybrid_v4_gimbal' else 2.0
        ls = '--' if g['camera_mode'] == 'fixed' else '-'
        axes[0].plot(ts, obs, marker='o', markersize=4.5, linewidth=lw, linestyle=ls, color=g['color'], label=g['label'])
        axes[1].plot(ts, val, marker='s', markersize=4.5, linewidth=lw, linestyle=ls, color=g['color'], label=g['label'])

    axes[0].set_title('总观测面积随任务时间变化（30 × 30 m V3 地形）', fontsize=12, fontweight='bold')
    axes[1].set_title('有效拟合高程面积随任务时间变化', fontsize=12, fontweight='bold')
    for ax in axes:
        ax.set_xlabel('任务时间（仿真分钟）', fontsize=10.5)
        ax.set_ylabel('面积（m²）', fontsize=10.5)
        ax.set_xlim(0, 8.2)
        ax.grid(True, linestyle=':', alpha=0.55)
        ax.legend(frameon=True, fontsize=9.5, loc='upper left')
    fig.tight_layout()
    fig.savefig(output_dir / 'coverage-curves.png', bbox_inches='tight')
    plt.close(fig)

    # 2. Four-panel key metrics comparison bar chart
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 8.2), dpi=180)
    labels = [g['short'] for g in GROUPS]
    colors = [g['color'] for g in GROUPS]
    x = np.arange(len(GROUPS))

    panels = [
        (axes[0, 0], '有效高程面积（m²，越高越好）', [d['summary']['valid_surface_m2'] for d in details], '{:.1f}'),
        (axes[0, 1], '后半程（4–8 min）新增观测面积（m²）',
         [d['half_windows'][1]['new_observed_m2'] if len(d['half_windows']) > 1 and d['half_windows'][1].get('reached') else 0.0 for d in details], '{:.1f}'),
        (axes[1, 0], 'XY 定位误差 P95（cm，越低越好）',
         [(d['summary']['localization_xy_p95_m'] or 0.0) * 100.0 for d in details], '{:.2f}'),
        (axes[1, 1], '底盘原地转向阶段次数（次，越少越好）',
         [d['summary']['chassis_turn_phases'] for d in details], '{:d}'),
    ]
    for ax, title, vals, fmt in panels:
        bars = ax.bar(x, vals, color=colors, width=0.56, edgecolor='#1f2937', linewidth=0.8)
        ax.set_title(title, fontsize=11.5, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(labels, fontsize=9.5)
        ax.grid(True, axis='y', linestyle=':', alpha=0.55)
        ymax = max(vals) if max(vals) > 0 else 1.0
        ax.set_ylim(0, ymax * 1.22)
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width() / 2.0, bar.get_height() + ymax * 0.03,
                    fmt.format(v), ha='center', va='bottom', fontsize=10, fontweight='bold')
    fig.suptitle('MoonCar R5 自主探索四组 8 分钟 Gazebo 物理仿真对比（2026-11-22）', fontsize=13.5, fontweight='bold', y=1.01)
    fig.tight_layout()
    fig.savefig(output_dir / 'metrics-summary.png', bbox_inches='tight')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    reference = build_reference(args.output)
    summaries = []
    final_maps = []
    for group in GROUPS:
        stats, maps = summarize_trial(args.raw, group, reference)
        summaries.append(stats)
        final_maps.append(maps)

    common_elapsed = min(s['elapsed_seconds'] for s in summaries)

    # Four-way common valid single-surface intersection
    common4 = np.ones_like(final_maps[0]['single'], dtype=bool)
    for m in final_maps:
        common4 &= m['single']

    # Also compute pairwise intersection: surface_v3_gimbal vs hybrid_v4_gimbal
    common_v3_v4 = final_maps[1]['single'] & final_maps[3]['single']

    def err_stats(mapped, mask):
        vals = mapped['error'][mask]
        return dict(
            rmse_m=float(np.sqrt(np.mean(vals ** 2))) if vals.size else None,
            mae_m=float(np.mean(np.abs(vals))) if vals.size else None,
            p95_abs_m=float(np.percentile(np.abs(vals), 95)) if vals.size else None,
        )

    common_errors = dict(
        common_4way_cells=int(common4.sum()),
        common_4way_m2=float(common4.sum()) * (0.25 ** 2),
        common_4way=[dict(key=g['key'], label=g['label'], **err_stats(m, common4)) for g, m in zip(GROUPS, final_maps)],
        common_surface_v3_vs_hybrid_v4_cells=int(common_v3_v4.sum()),
        common_surface_v3_vs_hybrid_v4_m2=float(common_v3_v4.sum()) * (0.25 ** 2),
        common_surface_v3_vs_hybrid_v4=[
            dict(key=GROUPS[idx]['key'], label=GROUPS[idx]['label'], **err_stats(final_maps[idx], common_v3_v4))
            for idx in (1, 3)
        ],
    )
    write(args.output / 'common-area-errors.json', common_errors)
    write(args.output / 'summary.json', summaries)

    details = []
    for group, summary in zip(GROUPS, summaries):
        dur = analyze_duration_run(args.raw, group['key'], summary, reference, common_elapsed)
        dur['summary'] = summary
        dur['label'] = group['label']
        dur['autonomy_strategy'] = group['strategy']
        dur['camera_mode'] = group['camera_mode']
        details.append(dur)
    write(args.output / 'four-way-summary.json', details)

    plot_charts(details, args.output)
    print(json.dumps(dict(
        common_elapsed_s=common_elapsed,
        common_4way_m2=common_errors['common_4way_m2'],
        groups=[
            dict(
                key=s['key'],
                outcome=s['outcome'],
                elapsed_s=s['elapsed_seconds'],
                wall_s=round(s['wall_seconds'], 1),
                observed_m2=round(s['total_observed_m2'], 2),
                valid_m2=round(s['valid_surface_m2'], 2),
                xy_p95_cm=round(s['localization_xy_p95_m'] * 100.0, 2) if s['localization_xy_p95_m'] is not None else None,
                height_rmse_cm=round(s['single_surface']['rmse_m'] * 100.0, 2) if s['single_surface']['rmse_m'] is not None else None,
                spins=s['chassis_turn_phases'],
                gimbal_actions=s['gimbal_selected_actions'],
                global_switches=s['global_frontier_switches'],
                path_m=round(s['path_length_m'], 2),
            )
            for s in summaries
        ],
    ), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
