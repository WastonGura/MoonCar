"""Summarize sealed offline RGBD runs; never run an estimator or feed it truth.

Example: python -m offline_rgbd_window.summarize --run RUN23 --run RUN25
         --run RUN27 --output NEW_DIRECTORY --conditional

The input JSONL, rather than historical metrics denominators, is authoritative.
All planned failures remain in the denominator and break plotted trajectories.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys

import numpy as np

sys.dont_write_bytecode = True
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from offline_rgbd_window.data import interpolate_truth, load_truth


REPO = Path(__file__).resolve().parents[2]
LIMIT_M = .05


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def read_json(path):
    with Path(path).open(encoding='utf-8-sig') as stream:
        return json.load(stream)


def recorded_path(value, run_directory):
    """Resolve existing recorded /source or /evidence paths on either host."""
    text = str(value).replace('\\', '/')
    candidates = [Path(value), run_directory / value]
    if text.startswith('/evidence/'):
        candidates.append(REPO / 'tmp/ros_acceptance' / text[len('/evidence/'):])
    if text.startswith('/source/'):
        candidates.append(REPO / text[len('/source/'):])
    marker = '/tmp/ros_acceptance/'
    if marker in text:
        candidates.append(REPO / 'tmp/ros_acceptance' / text.split(marker, 1)[1])
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(f'Cannot resolve recorded input: {value}')


def read_rows(path):
    with Path(path).open(encoding='utf-8-sig') as stream:
        rows = [json.loads(line) for line in stream if line.strip()]
    if not rows:
        raise ValueError(f'Empty output history: {path}')
    previous = None
    for row in rows:
        stamp = row.get('stamp_ns')
        if type(stamp) is not int or (previous is not None and stamp <= previous):
            raise ValueError(f'Nonincreasing or invalid output stamp in {path}')
        if type(row.get('output')) is not bool:
            raise ValueError(f'Missing explicit output status in {path}')
        previous = stamp
    return rows


def quantiles(values):
    return None if not len(values) else dict(zip(('p50', 'p95', 'max'),
        (float(x) for x in np.percentile(values, [50, 95, 100]))))


def covariance_sigma(row, key):
    matrix = np.asarray(row[key], dtype=float)
    if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
            or not np.allclose(matrix, matrix.T, atol=1e-10, rtol=1e-8)
            or float(np.linalg.eigvalsh(matrix).min()) < -1e-10):
        raise ValueError('An output contains an invalid translation covariance')
    return math.sqrt(max(0., float(np.linalg.eigvalsh(matrix[:2, :2]).max())))


def event(row, start_ns, *, reason=None):
    if row is None:
        return None
    return dict(stamp_ns=row['stamp_ns'], elapsed_s=(row['stamp_ns'] - start_ns) / 1e9,
                reason=reason if reason is not None else row.get('reason'))


def model_stats(rows, truth_xyz, truth_valid, truth_origin, start_ns, *, position_origin,
                covariance_key='covariance'):
    sigmas = np.full(len(rows), np.nan)
    errors = np.full(len(rows), np.nan)
    first_failure = last_solved = first_above_limit = None
    reasons = {}
    for index, row in enumerate(rows):
        if not row['output']:
            if first_failure is None:
                first_failure = event(row, start_ns)
            reason = row.get('reason', 'unspecified_failure')
            reasons[reason] = reasons.get(reason, 0) + 1
            continue
        position = np.asarray(row.get('position'), dtype=float)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError('An output contains an invalid position')
        sigmas[index] = covariance_sigma(row, covariance_key)
        last_solved = event(row, start_ns)
        if sigmas[index] > LIMIT_M and first_above_limit is None:
            first_above_limit = event(row, start_ns, reason='model_sigma_above_50mm')
        if truth_valid[index]:
            delta = (position - position_origin) - (truth_xyz[index] - truth_origin)
            errors[index] = float(np.linalg.norm(delta[:2]))
    finite = np.isfinite(sigmas)
    within = int(np.count_nonzero(finite & (sigmas <= LIMIT_M)))
    above = int(np.count_nonzero(finite & (sigmas > LIMIT_M)))
    finite_indices = np.flatnonzero(finite)
    result = dict(planned=len(rows), outputs=int(finite.sum()), no_output=int((~finite).sum()),
        truth_covered_plans=int(np.count_nonzero(truth_valid)), scored_outputs=int(np.isfinite(errors).sum()),
        model_xy_sigma_m=quantiles(sigmas[finite]), error_xy_m=quantiles(errors[np.isfinite(errors)]),
        sigma_first_last_m=([float(sigmas[finite_indices[0]]), float(sigmas[finite_indices[-1]])]
                            if len(finite_indices) else None),
        within_50mm=within, above_50mm=above,
        within_50mm_fraction_of_all_plans=within / len(rows) if rows else None,
        first_failure=first_failure, first_above_50mm=first_above_limit, last_solved=last_solved,
        no_output_reasons=reasons)
    return result, sigmas, errors


def measured_timing(rows, aliases):
    """Use only explicitly recorded measurements; no wall-to-CPU inference."""
    values = []
    for row in rows:
        value = next((row[name] for name in aliases if name in row), None)
        if value is None:
            continue
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise ValueError(f'Invalid compute timing: {aliases[0]}')
        values.append(float(value))
    return dict(measured_rows=len(values), input_rows=len(rows),
                total_s=sum(values) if values else None, seconds=quantiles(values))


def analyze_run(directory):
    directory = Path(directory).resolve()
    paths = {name: directory / name for name in ('run.json', 'metrics.json', 'baseline.jsonl', 'window.jsonl')}
    run = read_json(paths['run.json'])
    old_metrics = read_json(paths['metrics.json'])
    if not run.get('source_before') or run['source_before'] != run.get('source_after'):
        raise ValueError(f'Run has no matching source seal: {directory}')
    for name, expected in run.get('output_sha256', {}).items():
        if name in paths and digest(paths[name]) != expected:
            raise ValueError(f'Sealed output hash differs: {directory / name}')
    baseline = read_rows(paths['baseline.jsonl'])
    planned = read_rows(paths['window.jsonl'])
    if run.get('sensor_frames') != len(baseline) or run.get('planned_frames') != len(planned):
        raise ValueError(f'Run counts differ from recorded JSONL: {directory}')
    start_ns = baseline[0]['stamp_ns']
    if planned[0]['stamp_ns'] != start_ns or not baseline[0]['output'] or not planned[0]['output']:
        raise ValueError('Both methods must initialize at the same fixed first exposure')
    lookup = {row['stamp_ns']: row for row in baseline}
    if any(row['stamp_ns'] not in lookup for row in planned):
        raise ValueError('A planned timestamp has no same-stamp baseline result')
    old_planned = [lookup[row['stamp_ns']] for row in planned]
    config = run['config']
    truth_path = recorded_path(config['truth'], directory)
    manifest_path = recorded_path(config['manifest'], directory)
    truth_sha, manifest_sha = digest(truth_path), digest(manifest_path)
    if run.get('truth_sha256') != truth_sha or run.get('manifest_sha256') != manifest_sha:
        raise ValueError('Truth or manifest differs from the sealed replay input')
    manifest = read_json(manifest_path)
    truth = load_truth(truth_path)
    truth_xyz, truth_valid = interpolate_truth(truth, [row['stamp_ns'] for row in planned])
    initial_truth, initial_valid = interpolate_truth(truth, [start_ns])
    if not initial_valid[0]:
        raise ValueError('No independently recorded truth at the fixed initial exposure')
    base_origin = np.asarray(baseline[0]['position'], dtype=float)
    models = {}
    curves = {}
    for key, rows, origin, covariance in (
            ('baseline_planned', old_planned, base_origin, 'covariance'),
            ('window_with_start_prior', planned, np.zeros(3), 'covariance'),
            ('window_conditional', planned, np.zeros(3), 'covariance_conditional')):
        stats, sigma, error = model_stats(rows, truth_xyz, truth_valid, initial_truth[0], start_ns,
                                         position_origin=origin, covariance_key=covariance)
        models[key] = stats
        curves[key] = dict(sigma=sigma, error=error)
    all_values, all_valid = interpolate_truth(truth, [row['stamp_ns'] for row in baseline])
    models['baseline_all_sensor_frames'] = model_stats(baseline, all_values, all_valid, initial_truth[0],
        start_ns, position_origin=base_origin)[0]
    common = (np.isfinite(curves['baseline_planned']['sigma'])
              & np.isfinite(curves['window_with_start_prior']['sigma']))
    common_truth = common & truth_valid
    common_outputs = dict(outputs=int(common.sum()), truth_scored_outputs=int(common_truth.sum()),
        stamp_range_ns=([int(planned[i]['stamp_ns']) for i in (np.flatnonzero(common)[0], np.flatnonzero(common)[-1])]
                        if common.any() else None),
        baseline_error_xy_m=quantiles(curves['baseline_planned']['error'][common_truth]),
        window_error_xy_m=quantiles(curves['window_with_start_prior']['error'][common_truth]),
        baseline_sigma_xy_m=quantiles(curves['baseline_planned']['sigma'][common]),
        window_sigma_xy_m=quantiles(curves['window_with_start_prior']['sigma'][common]))
    frontend = measured_timing(baseline, ('frontend_s',))
    backend_rows = [row for row in planned if 'backend_s' in row]
    backend = measured_timing(backend_rows, ('backend_s',))
    old = measured_timing(baseline, ('baseline_s',))
    front_cpu = measured_timing(baseline, ('frontend_cpu_s', 'frontend_thread_cpu_s'))
    back_cpu = measured_timing(backend_rows, ('backend_cpu_s', 'backend_thread_cpu_s'))
    old_cpu = measured_timing(baseline, ('baseline_cpu_s', 'baseline_thread_cpu_s'))
    timing = dict(frontend_all_sensor_frames_wall=frontend, backend_all_planned_attempts_wall=backend,
        baseline_all_sensor_frames_wall=old,
        candidate_total_compute_wall_s=(frontend['total_s'] + (backend['total_s'] or 0.)
            if frontend['measured_rows'] == len(baseline) else None),
        frontend_all_sensor_frames_cpu=front_cpu, backend_all_planned_attempts_cpu=back_cpu,
        baseline_all_sensor_frames_cpu=old_cpu,
        candidate_total_measured_cpu_s=(front_cpu['total_s'] + (back_cpu['total_s'] or 0.)
            if front_cpu['measured_rows'] == len(baseline) and back_cpu['measured_rows'] == len(backend_rows) else None),
        scope='Sum every 10Hz frontend call plus every attempted 2Hz backend solve, including failed solves. '
              'Wall compute excludes file I/O, ROS/decode/queues and is not an online deadline result. '
              'CPU is unavailable unless explicitly recorded; thread CPU need not include OpenCV worker CPU.')
    case_match = re.search(r'sequence(\d+)', directory.name)
    label = f'Sequence {case_match.group(1)}' if case_match else directory.name
    is_old_attempt = directory.name in ('sequence25-01', 'sequence27-01')
    result = dict(run_directory=str(directory), run_name=directory.name, label=label,
        case=manifest.get('case'), revision_scope=('initial_attempt_not_final' if is_old_attempt else 'explicit_requested_run'),
        first_stamp_ns=start_ns, last_planned_stamp_ns=planned[-1]['stamp_ns'],
        planned_duration_s=(planned[-1]['stamp_ns'] - start_ns) / 1e9,
        initialization=dict(stamp_ns=start_ns, baseline_position=base_origin.tolist(),
                            truth_position=initial_truth[0].tolist(), alignment='initial translation only; no fitted rotation or scale'),
        sensor_frames=len(baseline), planned_frames=len(planned), models=models, common_outputs=common_outputs,
        timing=timing, source_before=run['source_before'], source_after=run['source_after'],
        summarize_in_run_source_seal=any(str(path).replace('\\', '/').endswith('/offline_rgbd_window/summarize.py')
                                       for path in run['source_before']),
        inputs=dict(manifest_path=str(manifest_path), manifest_sha256=manifest_sha,
                    truth_path=str(truth_path), truth_sha256=truth_sha),
        sealed_output_sha256={name:digest(path) for name, path in paths.items()},
        original_metrics_used_only_for_model_limitations_and_covariance_description=True,
        covariance_comparison=old_metrics.get('covariance_comparison'),
        limitations=old_metrics.get('limitations', []))
    plot = dict(seconds=np.array([(row['stamp_ns'] - start_ns) / 1e9 for row in planned]), curves=curves)
    return result, plot


def write_comparison(path, results):
    columns = ['run', 'case', 'model', 'planned', 'outputs', 'no_output', 'truth_covered_plans',
        'scored_outputs', 'within_50mm', 'above_50mm', 'within_50mm_fraction_of_all_plans',
        'first_failure_s', 'first_failure_reason', 'last_solved_s', 'sigma_first_m', 'sigma_last_m',
        'sigma_p95_m', 'sigma_max_m', 'error_p95_m', 'error_max_m', 'same_stamp_common_outputs',
        'same_stamp_baseline_error_p95_m', 'same_stamp_window_error_p95_m',
        'candidate_all_frontend_wall_s', 'candidate_all_backend_wall_s', 'candidate_total_compute_wall_s',
        'solve_wall_p95_s', 'solve_wall_max_s', 'candidate_total_measured_cpu_s']
    with Path(path).open('x', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns);writer.writeheader()
        for result in results:
            timing = result['timing'];common = result['common_outputs']
            for model, stats in result['models'].items():
                first = stats['first_failure'] or {};last = stats['last_solved'] or {}
                sigma = stats['model_xy_sigma_m'] or {};error = stats['error_xy_m'] or {}
                endpoints = stats['sigma_first_last_m'] or [None, None]
                solve = timing['backend_all_planned_attempts_wall']['seconds'] or {}
                writer.writerow(dict(run=result['run_name'], case=result['case'], model=model,
                    **{key:stats[key] for key in ('planned', 'outputs', 'no_output', 'truth_covered_plans',
                      'scored_outputs', 'within_50mm', 'above_50mm', 'within_50mm_fraction_of_all_plans')},
                    first_failure_s=first.get('elapsed_s'), first_failure_reason=first.get('reason'),
                    last_solved_s=last.get('elapsed_s'), sigma_first_m=endpoints[0], sigma_last_m=endpoints[1],
                    sigma_p95_m=sigma.get('p95'), sigma_max_m=sigma.get('max'),
                    error_p95_m=error.get('p95'), error_max_m=error.get('max'),
                    same_stamp_common_outputs=common['outputs'],
                    same_stamp_baseline_error_p95_m=(common['baseline_error_xy_m'] or {}).get('p95'),
                    same_stamp_window_error_p95_m=(common['window_error_xy_m'] or {}).get('p95'),
                    candidate_all_frontend_wall_s=timing['frontend_all_sensor_frames_wall']['total_s'],
                    candidate_all_backend_wall_s=timing['backend_all_planned_attempts_wall']['total_s'],
                    candidate_total_compute_wall_s=timing['candidate_total_compute_wall_s'],
                    solve_wall_p95_s=solve.get('p95'), solve_wall_max_s=solve.get('max'),
                    candidate_total_measured_cpu_s=timing['candidate_total_measured_cpu_s']))


def write_plot(path, results, plots, conditional=False):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    columns = len(results)
    fig, axes = plt.subplots(2, columns, figsize=(max(7, 5.3*columns), 8), squeeze=False)
    for column, (result, plot) in enumerate(zip(results, plots)):
        x = plot['seconds'];curves = plot['curves']
        for key, label, color in (('baseline_planned', 'Baseline (same planned stamps)', '#1f77b4'),
                                  ('window_with_start_prior', 'Window C + C0', '#d95f02')):
            axes[0, column].plot(x, curves[key]['sigma'] * 1000., label=label, color=color, lw=1.4)
            axes[1, column].plot(x, curves[key]['error'] * 1000., label=label, color=color, lw=1.4)
        if conditional:
            axes[0, column].plot(x, curves['window_conditional']['sigma'] * 1000.,
                                label='Window conditional C', color='#00876c', ls='--', lw=1.)
        for row in range(2):
            axes[row, column].axhline(50., color='#666666', ls=':', lw=1., label='50 mm reference')
            axes[row, column].grid(alpha=.22)
            axes[row, column].set_xlim(0., max(float(x[-1]), .1))
            axes[row, column].legend(fontsize=8, loc='best')
            axes[row, column].set_ylabel('Model XY sigma (mm)' if row == 0 else 'Actual XY error (mm)')
        axes[0, column].set_title(f"{result['label']} — {result['run_name']}\nAll {result['planned_frames']} planned outputs")
        axes[1, column].set_xlabel('Seconds from common initial exposure')
    fig.suptitle('Offline replay only — failures remain gaps; truth never enters estimation', fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, .95));fig.savefig(path, dpi=180);plt.close(fig)


def number(value, scale=1.):
    return '未记录' if value is None else f'{value * scale:.3f}'


def write_report(path, results):
    lines = ['# 离线多关键帧回放事实汇总', '',
        '全部计划输出为主分母；失败不剔除，图中留 NaN 断线。50mm 模型门计数独立于真值覆盖。',
        '误差来自同曝光的独立真值时间插值，仅固定初始平移，不拟合旋转、尺度或轨迹。',
        'C+C0 是一次加入初始先验的敏感性对照，不代表已标定后验。低模型协方差本身不证明准确度提高。', '']
    for result in results:
        lines.extend([f"## {result['label']} / {result['run_name']}", '',
            f"传感器 {result['sensor_frames']} 帧，计划 {result['planned_frames']} 次，计划跨度 {result['planned_duration_s']:.3f}s。",
            '| 模型 | 输出/全部计划 | ≤50mm/全部计划 | 首次无输出(s) | 最后输出(s) | σ P95/max(mm) | 同计划误差 P95/max(mm) |',
            '|---|---:|---:|---:|---:|---:|---:|'])
        for key, label in (('baseline_planned', '基线'), ('window_with_start_prior', '滑窗 C+C0'),
                            ('window_conditional', '滑窗 conditional C')):
            stats = result['models'][key];s = stats['model_xy_sigma_m'] or {};e = stats['error_xy_m'] or {}
            first = stats['first_failure'];last = stats['last_solved']
            lines.append(f"| {label} | {stats['outputs']}/{stats['planned']} | {stats['within_50mm']}/{stats['planned']} | "
                f"{number(first['elapsed_s']) if first else '无'} | {number(last['elapsed_s']) if last else '无'} | "
                f"{number(s.get('p95'), 1000)}/{number(s.get('max'), 1000)} | "
                f"{number(e.get('p95'), 1000)}/{number(e.get('max'), 1000)} |")
        common = result['common_outputs'];timing = result['timing']
        solver = timing['backend_all_planned_attempts_wall']['seconds'] or {}
        failures = [f"{key}：{stats['first_failure']['reason']}" for key, stats in result['models'].items()
                    if key != 'baseline_all_sensor_frames' and stats['first_failure'] is not None]
        lines.extend(['', '首次无输出原因：' + ('；'.join(failures) if failures else '无') + '。',
            f"双方同曝光且都有输出 {common['outputs']} 次，其中真值覆盖 {common['truth_scored_outputs']} 次；"
            f"共同输出误差 P95：基线 {number((common['baseline_error_xy_m'] or {}).get('p95'), 1000)}mm，"
            f"滑窗 {number((common['window_error_xy_m'] or {}).get('p95'), 1000)}mm。",
            f"全部 10Hz 前端墙时合计 {number(timing['frontend_all_sensor_frames_wall']['total_s'])}s，"
            f"全部计划求解墙时合计 {number(timing['backend_all_planned_attempts_wall']['total_s'])}s；"
            f"求解 P95/max {number(solver.get('p95'), 1000)}/{number(solver.get('max'), 1000)}ms。",
            f"CPU 合计：{number(timing['candidate_total_measured_cpu_s'])}；墙时与 CPU 不混用。", ''])
        if result['revision_scope'] == 'initial_attempt_not_final':
            lines.extend(['该目录是已封第一尝试，不能当最终修订结果。', ''])
    lines.extend(['若一个方法提前失败，它最后成功时的低 σ 不能解释为后续不确定性累积已改善；共同输出表只作附表。',
        '离线墙时不包含 ROS、解码、调度及在线队列，不能据此声称满足在线时效。',
        '本汇总不作在线导航、长时间运行或实车通过结论。', ''])
    Path(path).write_text('\n'.join(lines), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='append', required=True, type=Path, help='Sealed run directory; repeat for up to three cases.')
    parser.add_argument('--output', required=True, type=Path, help='A new directory; existing files are never overwritten.')
    parser.add_argument('--conditional', action='store_true', help='Also plot gauge-conditional covariance separately from C+C0.')
    args = parser.parse_args()
    if not 1 <= len(args.run) <= 3 or len({p.resolve() for p in args.run}) != len(args.run):
        parser.error('provide one to three distinct, explicitly selected run directories')
    if args.output.exists():
        parser.error('output directory already exists')
    source_before = digest(__file__)
    results, plots = [], []
    for directory in args.run:
        result, plot = analyze_run(directory);results.append(result);plots.append(plot)
    args.output.mkdir(parents=True, exist_ok=False)
    write_comparison(args.output / 'comparison.csv', results)
    write_plot(args.output / 'comparison.png', results, plots, args.conditional)
    write_report(args.output / 'report.md', results)
    if digest(__file__) != source_before:
        raise RuntimeError('Summarizer source changed during execution')
    summary = dict(schema='offline-rgbd-window-summary-v1', created_utc=datetime.now(timezone.utc).isoformat(),
        summarizer_source_sha256=source_before, runs=results,
        denominator='Every recorded planned output, including frontend rejection, unsolved windows and outputs above 50mm.',
        comparison='Baseline uses the same planned timestamps; common solved outputs are an appendix, not the primary denominator.',
        truth_scope='Evaluation only; no estimator is imported or invoked by this tool.',
        artifacts={name:digest(args.output / name) for name in ('comparison.csv', 'comparison.png', 'report.md')})
    with (args.output / 'summary.json').open('x', encoding='utf-8') as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2, allow_nan=False);stream.write('\n')
    print(json.dumps(dict(output=str(args.output.resolve()), runs=len(results),
        summary_sha256=digest(args.output / 'summary.json')), ensure_ascii=False))


if __name__ == '__main__':
    main()
