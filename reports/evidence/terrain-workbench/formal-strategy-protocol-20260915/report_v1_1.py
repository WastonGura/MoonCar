"""Summarize the six frozen conditions and three repeats under evaluation v1.1."""
import csv
import hashlib
import itertools
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[4]
SOURCE = ROOT / 'tmp/ros_acceptance/formal-strategy-20260915-reevaluated-v1_1-20260916'
OUTPUT = ROOT / 'reports/evidence/terrain-workbench/formal-strategy-results-v1_1-20260916'
STRATEGIES = ('nearest_view_v1', 'coverage_view_v1', 'surface_view_v1')
METRICS = ('S', 'C_obs', 'C', 'P', 'height_rmse_m', 'slope_mae_deg')


def sign_flip(differences):
    observed = abs(statistics.mean(differences))
    values = [abs(statistics.mean(s * d for s, d in zip(signs, differences)))
              for signs in itertools.product((-1, 1), repeat=len(differences))]
    return sum(value >= observed - 1e-12 for value in values) / len(values)


def holm(raw):
    adjusted = {}; previous = 0
    for rank, (name, value) in enumerate(sorted(raw.items(), key=lambda item: item[1])):
        previous = max(previous, min(1, (len(raw) - rank) * value)); adjusted[name] = previous
    return adjusted


def write_csv(path, rows):
    with path.open('w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)


def run():
    if not (SOURCE / 'COMPLETE').is_file() or (SOURCE / 'FAILED').exists():
        raise ValueError('Reevaluation is not complete')
    if OUTPUT.exists():
        raise ValueError('Preserve previous formal report')
    frozen = json.loads((SOURCE / 'hashes.json').read_text(encoding='utf-8'))
    for name, expected in frozen.items():
        if hashlib.sha256((SOURCE / name).read_bytes()).hexdigest() != expected:
            raise ValueError('Reevaluation hash mismatch: ' + name)
    trials = json.loads((SOURCE / 'trials.json').read_text(encoding='utf-8'))
    if len(trials) != 54 or any(t['acceptance']['status'] == 'not_evaluable' for t in trials):
        raise ValueError('Expected 54 evaluable trials')
    OUTPUT.mkdir(parents=True)
    trial_rows = []
    for trial in trials:
        row = {k: trial[k] for k in ('scene', 'start_index', 'repeat', 'strategy')}
        row['terminal_status'] = trial['status']; row['acceptance_status'] = trial['acceptance']['status']
        row.update({k: v for k, v in trial['acceptance'].items() if k != 'status'})
        row.update({k: trial['summary'].get(k) for k in ('elapsed_seconds', 'path_m', 'actions', 'failed_actions')})
        trial_rows.append(row)
    groups = []
    conditions = sorted({(r['scene'], r['start_index']) for r in trial_rows})
    for scene, start in conditions:
        for strategy in STRATEGIES:
            selected = [r for r in trial_rows if (r['scene'], r['start_index'], r['strategy']) == (scene, start, strategy)]
            if len(selected) != 3: raise ValueError('Expected three repeats per condition and strategy')
            row = {'scene': scene, 'start_index': start, 'strategy': strategy, 'trials': 3}
            for metric in METRICS + ('elapsed_seconds', 'path_m'):
                values = [r[metric] for r in selected]
                row.update({metric + '_mean': statistics.mean(values), metric + '_median': statistics.median(values),
                            metric + '_stdev': statistics.stdev(values), metric + '_min': min(values), metric + '_max': max(values)})
            groups.append(row)
    condition_scores = {(g['scene'], g['start_index'], g['strategy']): g['S_mean'] for g in groups}
    overall = {strategy: statistics.mean(condition_scores[scene, start, strategy] for scene, start in conditions)
               for strategy in STRATEGIES}
    raw = {}; pairs = []
    for left, right in itertools.combinations(STRATEGIES, 2):
        differences = [condition_scores[scene, start, left] - condition_scores[scene, start, right]
                       for scene, start in conditions]
        name = left + '__vs__' + right; raw[name] = sign_flip(differences)
        pairs.append({'left': left, 'right': right, 'conditions': 6,
                      'mean_S_difference_left_minus_right': statistics.mean(differences),
                      'median_S_difference_left_minus_right': statistics.median(differences),
                      'raw_exact_p': raw[name]})
    adjusted = holm(raw)
    for row in pairs:
        row['holm_adjusted_p'] = adjusted[row['left'] + '__vs__' + row['right']]
    statuses = {}
    for row in trial_rows:
        key = row['terminal_status']; statuses[key] = statuses.get(key, 0) + 1
    acceptance_statuses = {}
    for row in trial_rows:
        key = row['acceptance_status']; acceptance_statuses[key] = acceptance_statuses.get(key, 0) + 1
    summary = {'version': 'mooncar-formal-strategy-results-v1', 'evaluation_version': 'mapping-acceptance-v1.1',
               'trial_count': 54, 'condition_count': 6, 'repeats_per_condition_strategy': 3,
               'overall_equal_condition_mean_S': overall, 'terminal_statuses': statuses,
               'acceptance_statuses': acceptance_statuses, 'pairwise_tests': pairs,
               'interpretation': 'Descriptive comparison of six predefined conditions; no universal superiority claim.'}
    write_csv(OUTPUT / 'trial_metrics.csv', trial_rows)
    write_csv(OUTPUT / 'condition_summary.csv', groups)
    write_csv(OUTPUT / 'pairwise_tests.csv', pairs)
    (OUTPUT / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in OUTPUT.iterdir() if p.is_file()}
    (OUTPUT / 'hashes.json').write_text(json.dumps(hashes, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    assert sign_flip([1] * 6) == .03125
    run()
