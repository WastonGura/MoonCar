"""Extract and plot continuous time-series metrics S(t), L(t), Q(t) from survey strategy comparison trials."""
import argparse
import csv
import json
import math
import os
from pathlib import Path

STRATEGY_DISPLAY = {
    'nearest_view_v2': ('就近观察 v2', '#1f77b4', '--'),
    'coverage_view_v2': ('覆盖优先 v2', '#ff7f0e', '--'),
    'surface_view_v2': ('地表补采 v2', '#2ca02c', '--'),
    'nearest_view_v3': ('就近观察 v3', '#1f77b4', '-'),
    'coverage_view_v3': ('覆盖优先 v3', '#ff7f0e', '-'),
    'surface_view_v3': ('地表补采 v3', '#2ca02c', '-'),
}

DEFAULT_MILESTONES = [30, 60, 120, 180, 240, 300]


def setup_matplotlib():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    font_path = os.environ.get('MOONCAR_CJK_FONT', '')
    if font_path and os.path.exists(font_path):
        from matplotlib import font_manager
        font_manager.fontManager.addfont(font_path)
        prop = font_manager.FontProperties(fname=font_path)
        plt.rcParams['font.sans-serif'] = [prop.get_name(), 'SimHei', 'DejaVu Sans']
    else:
        plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    return plt


def read_telemetry(csv_path):
    rows = []
    with open(csv_path, 'r', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                t = float(r.get('elapsed_seconds', 0.0))
                s = float(r.get('observed_area_m2', 0.0))
                l = float(r.get('path_m', 0.0))
                q = float(r.get('quality_area_m2', 0.0))
                v = float(r.get('valid_surface_area_m2', 0.0))
                q_mean = float(r.get('mean_observed_quality', 0.0))
                rows.append({
                    't': t, 's': s, 'l': l, 'q': q, 'v': v, 'q_mean': q_mean
                })
            except (ValueError, TypeError):
                continue
    rows.sort(key=lambda x: x['t'])
    return rows


def sample_at_time(series, target_t):
    if not series:
        return {'t': target_t, 's': 0.0, 'l': 0.0, 'q': 0.0, 'v': 0.0, 'q_mean': 0.0}
    candidate = series[0]
    for pt in series:
        if pt['t'] <= target_t:
            candidate = pt
        else:
            break
    if target_t > series[-1]['t']:
        candidate = series[-1]
    return candidate


def analyze_benchmark(benchmark_dir, milestones=None):
    benchmark_dir = Path(benchmark_dir)
    milestones = milestones or DEFAULT_MILESTONES
    trials_path = benchmark_dir / 'trials.json'
    if not trials_path.exists():
        raise FileNotFoundError(f"Missing {trials_path}")
    trials = json.loads(trials_path.read_text(encoding='utf-8'))

    results = {}
    milestone_rows = []

    for trial in trials:
        strategy = trial.get('strategy')
        index = trial.get('index')
        folder_str = trial.get('folder')
        if not folder_str:
            continue
        folder = Path(folder_str)
        if not folder.exists():
            rel = Path(*Path(folder_str).parts[-3:])
            candidate = benchmark_dir.parent / rel
            if candidate.exists():
                folder = candidate

        telemetry_path = folder / 'telemetry.csv'
        if not telemetry_path.exists():
            continue

        series = read_telemetry(telemetry_path)
        results[strategy] = {
            'index': index,
            'strategy': strategy,
            'status': trial.get('status'),
            'reason': trial.get('reason'),
            'series': series,
            'summary': trial.get('summary', {}),
            'acceptance': trial.get('acceptance', {})
        }

        for m in milestones:
            pt = sample_at_time(series, m)
            milestone_rows.append({
                'trial_index': index,
                'strategy': strategy,
                'time_s': m,
                'observed_area_m2': round(pt['s'], 2),
                'path_m': round(pt['l'], 2),
                'quality_area_m2': round(pt['q'], 2),
                'valid_surface_m2': round(pt['v'], 2),
                'efficiency_s_per_m': round(pt['s'] / max(pt['l'], 0.1), 2)
            })

    out_csv = benchmark_dir / 'time_milestones.csv'
    if milestone_rows:
        with open(out_csv, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.DictWriter(f, fieldnames=list(milestone_rows[0].keys()))
            writer.writeheader()
            writer.writerows(milestone_rows)
        print(f"Wrote milestone metrics to {out_csv}")

    plt = setup_matplotlib()
    fig, axes = plt.subplots(2, 2, figsize=(12, 9), layout='constrained')
    ax_s, ax_l = axes[0, 0], axes[0, 1]
    ax_v, ax_q = axes[1, 0], axes[1, 1]

    ax_s.set_title('观测投影面积 S(t) (m²)', fontsize=12, fontweight='bold')
    ax_s.set_xlabel('仿真时间 (s)', fontsize=10)
    ax_s.set_ylabel('观测面积 (m²)', fontsize=10)
    ax_s.grid(True, linestyle=':', alpha=0.6)

    ax_l.set_title('底盘累计行驶路程 L(t) (m)', fontsize=12, fontweight='bold')
    ax_l.set_xlabel('仿真时间 (s)', fontsize=10)
    ax_l.set_ylabel('路径里程 (m)', fontsize=10)
    ax_l.grid(True, linestyle=':', alpha=0.6)

    ax_v.set_title('有效拟合表面面积 S_valid(t) (m²)', fontsize=12, fontweight='bold')
    ax_v.set_xlabel('仿真时间 (s)', fontsize=10)
    ax_v.set_ylabel('有效表面 (m²)', fontsize=10)
    ax_v.grid(True, linestyle=':', alpha=0.6)

    ax_q.set_title('采样质量积分面积 Q(t) (m²)', fontsize=12, fontweight='bold')
    ax_q.set_xlabel('仿真时间 (s)', fontsize=10)
    ax_q.set_ylabel('质量积分 (m²)', fontsize=10)
    ax_q.grid(True, linestyle=':', alpha=0.6)

    for strategy, data in sorted(results.items()):
        series = data['series']
        if not series:
            continue
        label, color, style = STRATEGY_DISPLAY.get(strategy, (strategy, 'black', '-'))
        t = [pt['t'] for pt in series]
        s = [pt['s'] for pt in series]
        l = [pt['l'] for pt in series]
        v = [pt['v'] for pt in series]
        q = [pt['q'] for pt in series]

        if t and t[-1] < 300:
            t.append(300)
            s.append(s[-1])
            l.append(l[-1])
            v.append(v[-1])
            q.append(q[-1])

        lw = 2.2 if '_v3' in strategy else 1.6
        ax_s.plot(t, s, label=label, color=color, linestyle=style, linewidth=lw)
        ax_l.plot(t, l, label=label, color=color, linestyle=style, linewidth=lw)
        ax_v.plot(t, v, label=label, color=color, linestyle=style, linewidth=lw)
        ax_q.plot(t, q, label=label, color=color, linestyle=style, linewidth=lw)

    for ax in (ax_s, ax_l, ax_v, ax_q):
        ax.legend(fontsize=9, loc='best')
        ax.set_xlim(0, 305)

    plot_path = benchmark_dir / 'time_series_curves.png'
    fig.savefig(plot_path, dpi=200)
    fig.savefig(benchmark_dir / 'time_series_curves.pdf')
    plt.close(fig)
    print(f"Saved time-series curves to {plot_path}")

    return milestone_rows


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('benchmark_dir', type=Path, help='Path to benchmark folder containing trials.json')
    parser.add_argument('--milestones', type=int, nargs='+', default=DEFAULT_MILESTONES)
    args = parser.parse_args()
    analyze_benchmark(args.benchmark_dir, args.milestones)
