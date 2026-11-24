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
    from matplotlib import font_manager
    font_candidates = [
        '/usr/share/fonts/truetype/msyh.ttc',
        '/usr/share/fonts/truetype/simhei.ttf',
        'C:/Windows/Fonts/msyh.ttc',
        'C:/Windows/Fonts/simhei.ttf',
        os.environ.get('MOONCAR_CJK_FONT', ''),
    ]
    for fp in font_candidates:
        if fp and os.path.exists(fp):
            font_manager.fontManager.addfont(fp)
            prop = font_manager.FontProperties(fname=fp)
            plt.rcParams['font.sans-serif'] = [prop.get_name(), 'Microsoft YaHei', 'SimHei', 'DejaVu Sans']
            break
    else:
        plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
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
                'efficiency_s_per_m': round(pt['s'] / max(pt['l'], 0.1), 2),
                'efficiency_valid_per_m': round(pt['v'] / max(pt['l'], 0.1), 2)
            })

    out_csv = benchmark_dir / 'time_milestones.csv'
    if milestone_rows:
        with open(out_csv, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.DictWriter(f, fieldnames=list(milestone_rows[0].keys()))
            writer.writeheader()
            writer.writerows(milestone_rows)
        print(f"Wrote milestone metrics to {out_csv}")

    plt = setup_matplotlib()
    fig, axes = plt.subplots(3, 2, figsize=(13, 12), layout='constrained')
    ax_s, ax_l = axes[0, 0], axes[0, 1]
    ax_eta, ax_eta_v = axes[1, 0], axes[1, 1]
    ax_v, ax_q = axes[2, 0], axes[2, 1]

    ax_s.set_title('累计观测投影面积 S(t) (m²)', fontsize=12, fontweight='bold')
    ax_s.set_xlabel('仿真时间 (s)', fontsize=10)
    ax_s.set_ylabel('观测面积 (m²)', fontsize=10)
    ax_s.grid(True, linestyle=':', alpha=0.6)

    ax_l.set_title('底盘累计行驶里程 L(t) (m)', fontsize=12, fontweight='bold')
    ax_l.set_xlabel('仿真时间 (s)', fontsize=10)
    ax_l.set_ylabel('路径里程 (m)', fontsize=10)
    ax_l.grid(True, linestyle=':', alpha=0.6)

    ax_eta.set_title('单位位移观测收益 η(t) = S(t)/L(t) (m²/m)', fontsize=12, fontweight='bold')
    ax_eta.set_xlabel('仿真时间 (s)', fontsize=10)
    ax_eta.set_ylabel('观测位移效率 (m²/m)', fontsize=10)
    ax_eta.set_ylim(0, 95)
    ax_eta.grid(True, linestyle=':', alpha=0.6)

    ax_eta_v.set_title('单位位移有效拟合收益 η_valid(t) = S_valid(t)/L(t) (m²/m)', fontsize=12, fontweight='bold')
    ax_eta_v.set_xlabel('仿真时间 (s)', fontsize=10)
    ax_eta_v.set_ylabel('有效表面位移效率 (m²/m)', fontsize=10)
    ax_eta_v.set_ylim(0, 60)
    ax_eta_v.grid(True, linestyle=':', alpha=0.6)

    ax_v.set_title('有效拟合表面面积 S_valid(t) (m²)', fontsize=12, fontweight='bold')
    ax_v.set_xlabel('仿真时间 (s)', fontsize=10)
    ax_v.set_ylabel('有效表面 (m²)', fontsize=10)
    ax_v.grid(True, linestyle=':', alpha=0.6)

    ax_q.set_title('采样质量积分面积 Q(t) (m²)', fontsize=12, fontweight='bold')
    ax_q.set_xlabel('仿真时间 (s)', fontsize=10)
    ax_q.set_ylabel('质量积分 (m²)', fontsize=10)
    ax_q.grid(True, linestyle=':', alpha=0.6)

    # Standalone efficiency figure
    fig_eta, (ax_e1, ax_e2) = plt.subplots(1, 2, figsize=(12.5, 5.2), layout='constrained')
    ax_e1.set_title('单位位移观测收益 η(t) = S(t)/L(t) (m²/m)', fontsize=12.5, fontweight='bold')
    ax_e1.set_xlabel('仿真时间 (s)', fontsize=10.5)
    ax_e1.set_ylabel('单位位移观测面积 (m²/m)', fontsize=10.5)
    ax_e1.set_ylim(0, 95)
    ax_e1.grid(True, linestyle=':', alpha=0.6)

    ax_e2.set_title('单位位移有效表面收益 η_valid(t) = S_valid(t)/L(t) (m²/m)', fontsize=12.5, fontweight='bold')
    ax_e2.set_xlabel('仿真时间 (s)', fontsize=10.5)
    ax_e2.set_ylabel('单位位移有效拟合表面 (m²/m)', fontsize=10.5)
    ax_e2.set_ylim(0, 60)
    ax_e2.grid(True, linestyle=':', alpha=0.6)

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

        t_eta = [pt['t'] for pt in series if pt['l'] >= 0.5]
        eta_vals = [pt['s'] / pt['l'] for pt in series if pt['l'] >= 0.5]
        eta_v_vals = [pt['v'] / pt['l'] for pt in series if pt['l'] >= 0.5]
        if t_eta:
            if t_eta[-1] < 300:
                t_eta.append(300)
                eta_vals.append(eta_vals[-1])
                eta_v_vals.append(eta_v_vals[-1])
            ax_eta.plot(t_eta, eta_vals, label=label, color=color, linestyle=style, linewidth=lw)
            ax_eta_v.plot(t_eta, eta_v_vals, label=label, color=color, linestyle=style, linewidth=lw)
            ax_e1.plot(t_eta, eta_vals, label=label, color=color, linestyle=style, linewidth=lw)
            ax_e2.plot(t_eta, eta_v_vals, label=label, color=color, linestyle=style, linewidth=lw)
        else:
            # 底盘未行驶或移动极小 (<0.5m)，在图例中予以规范说明避免误认为丢失
            dummy_label = f"{label} (底盘未行驶 L=0m)"
            ax_eta.plot([], [], label=dummy_label, color=color, linestyle=style, linewidth=lw)
            ax_eta_v.plot([], [], label=dummy_label, color=color, linestyle=style, linewidth=lw)
            ax_e1.plot([], [], label=dummy_label, color=color, linestyle=style, linewidth=lw)
            ax_e2.plot([], [], label=dummy_label, color=color, linestyle=style, linewidth=lw)

    for ax in (ax_s, ax_l, ax_eta, ax_eta_v, ax_v, ax_q):
        ax.legend(fontsize=9, loc='best')
        ax.set_xlim(0, 305)

    ax_e1.legend(fontsize=9.5, loc='best')
    ax_e1.set_xlim(0, 305)
    ax_e2.legend(fontsize=9.5, loc='best')
    ax_e2.set_xlim(0, 305)

    plot_path = benchmark_dir / 'time_series_curves.png'
    fig.savefig(plot_path, dpi=200)
    fig.savefig(benchmark_dir / 'time_series_curves.pdf')
    plt.close(fig)
    print(f"Saved 6-panel time-series curves to {plot_path}")

    eta_path = benchmark_dir / 'displacement_efficiency_curves.png'
    fig_eta.savefig(eta_path, dpi=200)
    fig_eta.savefig(benchmark_dir / 'displacement_efficiency_curves.pdf')
    plt.close(fig_eta)
    print(f"Saved displacement efficiency curves to {eta_path}")

    return milestone_rows


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('benchmark_dir', type=Path, help='Path to benchmark folder containing trials.json')
    parser.add_argument('--milestones', type=int, nargs='+', default=DEFAULT_MILESTONES)
    args = parser.parse_args()
    analyze_benchmark(args.benchmark_dir, args.milestones)
