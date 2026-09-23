"""Generate publication-ready benchmark charts with full Chinese annotations and labels."""
import argparse
import csv
import json
import os
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# Configure Chinese font: Prefer Microsoft YaHei for complete Unicode/glyph support
FONT_CANDIDATES = [
    '/usr/share/fonts/truetype/msyh.ttc',
    '/usr/share/fonts/truetype/simhei.ttf',
    'C:/Windows/Fonts/msyh.ttc',
    'C:/Windows/Fonts/simhei.ttf',
    os.environ.get('MOONCAR_CJK_FONT', ''),
]

cjk_font_loaded = False
for fpath in FONT_CANDIDATES:
    if fpath and os.path.exists(fpath):
        fm.fontManager.addfont(fpath)
        prop = fm.FontProperties(fname=fpath)
        plt.rcParams['font.sans-serif'] = [prop.get_name(), 'Microsoft YaHei', 'SimHei', 'DejaVu Sans']
        cjk_font_loaded = True
        break

if not cjk_font_loaded:
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Noto Sans CJK SC', 'DejaVu Sans']

plt.rcParams['axes.unicode_minus'] = False

STRATEGY_CN = {
    'nearest_view_v1': '就近观察 v1',
    'coverage_view_v1': '覆盖优先 v1',
    'surface_view_v1': '地表补采 v1',
    'nearest_view_v2': '就近观察 v2',
    'coverage_view_v2': '覆盖优先 v2',
    'surface_view_v2': '地表补采 v2',
    'nearest_view_v3': '就近观察 v3',
    'coverage_view_v3': '覆盖优先 v3',
    'surface_view_v3': '地表补采 v3',
}

STRATEGY_ORDER = [
    'nearest_view_v3', 'coverage_view_v3', 'surface_view_v3',
    'nearest_view_v2', 'coverage_view_v2', 'surface_view_v2'
]

# Style mapping for consistency across all charts
STYLE_MAP = {
    'nearest_view_v3': {'color': '#bfdbfe', 'edge': '#1d4ed8', 'line_color': '#2563eb', 'hatch': '', 'marker': 'o', 'fill': True},
    'coverage_view_v3': {'color': '#fed7aa', 'edge': '#c2410c', 'line_color': '#ea580c', 'hatch': '', 'marker': 's', 'fill': True},
    'surface_view_v3': {'color': '#bbf7d0', 'edge': '#15803d', 'line_color': '#16a34a', 'hatch': '', 'marker': '^', 'fill': True},
    'nearest_view_v2': {'color': '#eff6ff', 'edge': '#3b82f6', 'line_color': '#3b82f6', 'hatch': '//', 'marker': 'o', 'fill': False},
    'coverage_view_v2': {'color': '#fff7ed', 'edge': '#f97316', 'line_color': '#f97316', 'hatch': '//', 'marker': 's', 'fill': False},
    'surface_view_v2': {'color': '#f0fdf4', 'edge': '#22c55e', 'line_color': '#22c55e', 'hatch': '//', 'marker': '^', 'fill': False},
}


def plot_comparison_bars(report_dir, out_path):
    comp_file = report_dir / 'comparison.csv'
    grouped_file = report_dir / 'grouped.csv'
    if not comp_file.exists() or not grouped_file.exists():
        print(f"Skipping bars for {report_dir}: files missing")
        return

    with open(comp_file, 'r', encoding='utf-8-sig') as f:
        rows = list(csv.DictReader(f))
    with open(grouped_file, 'r', encoding='utf-8-sig') as f:
        groups = list(csv.DictReader(f))

    for r in rows:
        for k in ['new_observed_area_m2', 'valid_surface_gain_m2', 'resampling_success_fraction', 'path_m']:
            if r.get(k) is not None and r[k] != '':
                try:
                    r[k] = float(r[k])
                except ValueError:
                    r[k] = None
            else:
                r[k] = None

    for g in groups:
        for k in ['new_observed_area_m2', 'valid_surface_gain_m2', 'resampling_success_fraction', 'path_m']:
            for suffix in ['_mean', '_min', '_max']:
                field = k + suffix
                if g.get(field) is not None and g[field] != '':
                    try:
                        g[field] = float(g[field])
                    except ValueError:
                        g[field] = None
                else:
                    g[field] = None

    groups = [g for g in groups if int(g.get('reported_runs', 0)) > 0]

    def sort_key(g):
        strat = g.get('strategy', '')
        if strat in STRATEGY_ORDER:
            return STRATEGY_ORDER.index(strat)
        return 99

    groups.sort(key=sort_key)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8.5), layout='constrained')
    keys = ('new_observed_area_m2', 'valid_surface_gain_m2', 'resampling_success_fraction', 'path_m')
    titles = (
        '新增观测投影面积 (m²)',
        '有效拟合表面净增面积 (m²)',
        '初始欠采样栅格恢复率',
        '底盘定位累计行驶里程 (m)'
    )
    ylabels = ('新增面积 (m²)', '净增面积 (m²)', '恢复比例', '累计里程 (m)')

    positions = list(range(len(groups)))
    labels = []
    for g in groups:
        strat_label = STRATEGY_CN.get(g['strategy'], g['strategy'])
        if g.get('start') and g['start'].startswith('['):
            try:
                coords = json.loads(g['start'])
                labels.append(f"{strat_label}\n({coords[0]:.1f}, {coords[1]:.2f})")
            except Exception:
                labels.append(strat_label)
        else:
            labels.append(strat_label)

    for ax, key, title, ylabel in zip(axes.flat, keys, titles, ylabels):
        for i, g in enumerate(groups):
            val = g[key + '_mean']
            if val is None:
                continue
            strat = g['strategy']
            style = STYLE_MAP.get(strat, {'color': '#f2f2f2', 'edge': 'black', 'hatch': ''})
            ax.bar(i, val, color=style['color'], edgecolor=style['edge'], hatch=style['hatch'], width=0.55, linewidth=1.3, zorder=2)
            if g[key + '_min'] is not None and g[key + '_max'] is not None:
                ax.plot([i, i], [g[key + '_min'], g[key + '_max']], 'k-', linewidth=1.6, zorder=3)
            selected = [r[key] for r in rows if r.get('strategy') == g['strategy'] and r.get(key) is not None]
            ax.plot([i] * len(selected), selected, 'ko', markersize=4.5, zorder=4)

        ax.set_xticks(positions)
        ax.set_xticklabels(labels, fontsize=9.5)
        ax.set_title(title, fontsize=12, fontweight='bold', pad=8)
        ax.set_ylabel(ylabel, fontsize=10.5)
        ax.grid(True, linestyle=':', alpha=0.5, axis='y', zorder=1)
        if key == 'resampling_success_fraction':
            ax.set_ylim(0.0, 1.05)

    fig.supxlabel('注：圆点代表各实验轮次；柱高代表描述性均值；误差竖线代表极值范围（最小值/最大值，非统计置信区间）。保留所有失败轮次。', fontsize=9.5)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=200)
    pdf_path = out_path.with_suffix('.pdf')
    fig.savefig(pdf_path)
    plt.close(fig)
    print(f"Saved Chinese comparison bar chart to {out_path}")


def plot_prediction_errors(report_dir, out_path):
    pred_file = report_dir / 'prediction-errors.csv'
    if not pred_file.exists():
        print(f"Skipping predictions for {report_dir}: file missing")
        return

    predictions = []
    with open(pred_file, 'r', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        for r in reader:
            for k in ['predicted_new_m2', 'actual_new_m2', 'predicted_resampling_utility_m2', 'actual_recovered_surface_m2']:
                if r.get(k) is not None and r[k] != '':
                    try:
                        r[k] = float(r[k])
                    except ValueError:
                        r[k] = None
                else:
                    r[k] = None
            predictions.append(r)

    fig, axes = plt.subplots(1, 2, figsize=(11.5, 5.2), layout='constrained')

    pairs = [
        ('predicted_new_m2', 'actual_new_m2', '新增观测面积：预期 vs 实际', '模型预期视场增益 (m²)', '实际新增观测面积 (m²)'),
        ('predicted_resampling_utility_m2', 'actual_recovered_surface_m2', '欠采样地表补采：预期效用 vs 实际恢复', '模型补采预期效用 (m²)', '实际恢复地表面积 (m²)')
    ]

    for ax, (xkey, ykey, title, xlabel, ylabel) in zip(axes, pairs):
        for strategy in STRATEGY_ORDER:
            selected = [p for p in predictions if p.get('strategy') == strategy and p.get(xkey) is not None and p.get(ykey) is not None]
            if not selected:
                continue
            style = STYLE_MAP.get(strategy, {'marker': 'o', 'fill': True, 'line_color': '#333333'})
            label = STRATEGY_CN.get(strategy, strategy)
            fc = style['color'] if style['fill'] else 'none'
            ax.scatter([p[xkey] for p in selected], [p[ykey] for p in selected],
                       marker=style['marker'], facecolors=fc, edgecolors=style['edge'],
                       s=40, linewidth=1.2, label=label, zorder=3)

        upper = max([p[k] for p in predictions for k in (xkey, ykey) if p.get(k) is not None] + [1])
        ax.plot([0, upper], [0, upper], 'k--', linewidth=1.0, label='理想预测线 (y=x)', zorder=2)
        ax.set_xlim(-upper * 0.03, upper * 1.05)
        ax.set_ylim(-upper * 0.03, upper * 1.05)
        ax.set_xlabel(xlabel, fontsize=11)
        ax.set_ylabel(ylabel, fontsize=11)
        ax.set_title(title, fontsize=12.5, fontweight='bold', pad=8)
        ax.grid(True, linestyle=':', alpha=0.5, zorder=1)
        ax.legend(fontsize=9, loc='best')

    fig.supxlabel('注：统计包含所有失败与被中断动作。固定补采权重为模型决策效用值，非物理标定绝对恢复概率。', fontsize=9.5)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=200)
    pdf_path = out_path.with_suffix('.pdf')
    fig.savefig(pdf_path)
    plt.close(fig)
    print(f"Saved Chinese prediction errors plot to {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-dir', type=Path, help='Path to report directory containing comparison.csv and prediction-errors.csv')
    parser.add_argument('--bar-out', type=Path, help='Output path for comparison bar chart')
    parser.add_argument('--pred-out', type=Path, help='Output path for prediction errors plot')
    parser.add_argument('--all-benchmarks', action='store_true', help='Process all three benchmarks in /evidence')
    args = parser.parse_args()

    if args.all_benchmarks:
        evidence = Path('/evidence')
        mapping = {
            'benchmark-300s-ramp-v3retest': '01_visible_ramp_v1',
            'benchmark-300s-curve-fixed': '02_visible_curve_v1',
            'benchmark-300s-survey-fixed': '03_survey'
        }
        for bench, folder in mapping.items():
            report = evidence / bench / 'report'
            if not report.exists():
                print(f"Report dir not found: {report}")
                continue
            bar_out = report / 'comparison_bar_chart_cn.png'
            pred_out = report / 'prediction_errors_cn.png'
            plot_comparison_bars(report, bar_out)
            plot_prediction_errors(report, pred_out)
    else:
        if args.report_dir and args.bar_out:
            plot_comparison_bars(args.report_dir, args.bar_out)
        if args.report_dir and args.pred_out:
            plot_prediction_errors(args.report_dir, args.pred_out)


if __name__ == '__main__':
    main()
