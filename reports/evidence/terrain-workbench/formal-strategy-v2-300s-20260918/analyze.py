"""Audit the registered 300-second strategy comparison and summarize its first results."""
import csv
import hashlib
import itertools
import json
import math
import os
import statistics
import sys
import zipfile
from collections import Counter, defaultdict
from pathlib import Path

SOURCE_REPORT = Path(__file__).resolve().parent
ROOT = SOURCE_REPORT.parents[3]
REPORT = Path(os.environ.get('MOONCAR_FORMAL_REPORT', SOURCE_REPORT))
RUN = Path(os.environ.get('MOONCAR_FORMAL_RUN',
                          ROOT / 'tmp/ros_acceptance/formal-strategy-v2-300s-20260918'))
EVIDENCE = ROOT / 'tmp/ros_acceptance'
SCENES = {'survey': 'survey', 'visible_ramp_v1': 'visible-ramp',
          'visible_curve_v1': 'visible-curve'}


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def save(name, value):
    (REPORT / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n',
                               encoding='utf-8')


def table(name, rows):
    with (REPORT / name).open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def mean(rows, key):
    values = [row[key] for row in rows if row[key] is not None]
    return statistics.mean(values) if values else None


def sign_flip_p(differences):
    observed = abs(statistics.mean(differences))
    means = (abs(statistics.mean(sign * difference for sign, difference in zip(signs, differences)))
             for signs in itertools.product((-1, 1), repeat=len(differences)))
    return sum(value >= observed - 1e-10 for value in means) / 2 ** len(differences)


def check_hashes(base, hashes, issues, label):
    for name, expected in hashes.items():
        path = base / name
        if not path.is_file() or sha(path) != expected:
            issues.append(f'{label}: hash mismatch: {path}')


def main(scene_filter=None):
    protocol = read(RUN / 'protocol.json')
    if scene_filter is not None and scene_filter not in SCENES:
        raise ValueError(f'Unknown scene: {scene_filter}')
    strategies = tuple(protocol['strategies'])
    issues = []
    rows = []
    references = {}
    if (protocol['total_planned_trials'], protocol['repeats_per_condition'],
            protocol['budget_sim_seconds'], protocol['roi_radius_m']) != (54, 3, 300, 8):
        issues.append('Unexpected registered design')
    check_hashes(ROOT, protocol['source_sha256'], issues, 'frozen source')
    for scene_spec in protocol['scenes']:
        scene = scene_spec['name']
        if scene_filter is not None and scene != scene_filter:
            continue
        folder = RUN / SCENES[scene]
        if not (folder / 'plan.json').is_file() or not (folder / 'trials.json').is_file():
            issues.append(f'{scene}: batch missing')
            continue
        plan = read(folder / 'plan.json')['trials']
        trials = read(folder / 'trials.json')
        if len(plan) != 18 or len(trials) != 18:
            issues.append(f'{scene}: expected 18 planned and 18 completed slots')
        counts = Counter((p['start_index'], p['strategy']) for p in plan)
        if any(counts[(start, strategy)] != 3 for start in (1, 2) for strategy in strategies):
            issues.append(f'{scene}: strategy matrix is unbalanced')
        for index, planned in enumerate(plan, 1):
            label = f'{scene}/trial-{index:02d}'
            if index > len(trials):
                issues.append(f'{label}: result missing')
                continue
            entry = trials[index - 1]
            if entry.get('index') != index or any(entry.get(key) != value for key, value in planned.items()):
                issues.append(f'{label}: actual condition differs from plan')
            if (entry.get('start') != scene_spec['starts'][entry['start_index'] - 1] or
                    entry.get('budget_seconds') != 300 or entry.get('radius') != 8 or
                    entry.get('capture_gui')):
                issues.append(f'{label}: condition differs from registered protocol')
            row = dict(scene=scene, start_index=entry['start_index'], repeat=entry['repeat'],
                       strategy=entry['strategy'], index=index, status=entry['status'],
                       experiment=entry.get('experiment'), error=entry.get('error'),
                       acceptance_status=None, S=None, C_obs=None, C=None, P=None,
                       ground_C=None, ground_P=None, terrain_C=None, terrain_P=None,
                       height_rmse_m=None, slope_mae_deg=None, elapsed_seconds=None,
                       path_m=None, actions=None, failed_actions=None)
            rows.append(row)
            if entry['status'] in ('starting', 'trial_failed'):
                issues.append(f'{label}: {entry["status"]}: {entry.get("error", "")}')
                continue
            capture = folder / 'captures' / f'trial-{index:02d}'
            evaluation = folder / 'evaluations' / f'trial-{index:02d}'
            research = EVIDENCE / entry['folder'].removeprefix('/evidence/')
            try:
                cap, ev = read(capture / 'manifest.json'), read(evaluation / 'summary.json')
                mission, summary = read(research / 'mission.json'), read(research / 'summary.json')
                manifest = read(research / 'manifest.json')
                with (capture / 'truth.jsonl').open(encoding='utf-8') as stream:
                    physical = json.loads(stream.readline())['pose'][:2]
                planned_xy = entry['start'][:2]
                if any(math.dist(a, b) > .25 for a, b in
                       ((mission['origin'], planned_xy), (physical, planned_xy),
                        (mission['origin'], physical))):
                    issues.append(f'{label}: physical/localized start mismatch')
                if cap['status'] != 'captured' or not (capture / 'raw-depth.npz').is_file():
                    issues.append(f'{label}: incomplete raw capture')
                if manifest.get('report_status') != 'ready' or manifest.get('capture_errors'):
                    issues.append(f'{label}: research report incomplete')
                if summary['strategy'] != entry['strategy'] or summary['status'] != entry['status']:
                    issues.append(f'{label}: research summary mismatch')
                if (ev['version'] != 'mapping-acceptance-v1.1' or not ev['reference_audit']['passed'] or
                        not ev['evaluable'] or ev['S'] is None):
                    issues.append(f'{label}: acceptance result not evaluable')
                elif (ev['P'] is None and (ev['C'], ev['S']) != (0, 0)) or (ev['P'] is not None and
                        abs(ev['S'] - 100 * ev['C'] * ev['P']) > 1e-8):
                    issues.append(f'{label}: S formula mismatch')
                for key in ('status', 'S', 'C_obs', 'C', 'P', 'height_rmse_m', 'slope_mae_deg'):
                    if entry['acceptance'][key] != ev[key]:
                        issues.append(f'{label}: runner/evaluator {key} mismatch')
                condition = (scene, entry['start_index'])
                reference = (ev['reference_sha256']['reference.json'], ev['evaluation_area_m2'])
                if condition in references and references[condition] != reference:
                    issues.append(f'{label}: fixed reference/denominator changed')
                references[condition] = reference
                check_hashes(capture, ev['input_sha256'], issues, label)
                check_hashes(folder / 'references' / f'start-{entry["start_index"]:02d}',
                             ev['reference_sha256'], issues, label)
                check_hashes(evaluation, read(evaluation / 'hashes.json'), issues, label)
                check_hashes(research, read(research / 'hashes.json'), issues, label)
                with zipfile.ZipFile(research / 'source.zip') as source:
                    for name, expected in protocol['source_sha256'].items():
                        if name not in source.namelist() or hashlib.sha256(source.read(name)).hexdigest() != expected:
                            issues.append(f'{label}: frozen source mismatch: {name}')
                with (research / 'actions.csv').open(newline='', encoding='utf-8') as stream:
                    actions = list(csv.DictReader(stream))
                outcomes = Counter(action['outcome'] for action in actions)
                if len(actions) != summary['actions']:
                    issues.append(f'{label}: action count mismatch')
                row.update(acceptance_status=ev['status'], S=ev['S'], C_obs=ev['C_obs'],
                           C=ev['C'], P=ev['P'], ground_C=ev['by_category']['ground']['C'],
                           ground_P=ev['by_category']['ground']['P'],
                           terrain_C=ev['by_category']['terrain']['C'],
                           terrain_P=ev['by_category']['terrain']['P'],
                           height_rmse_m=ev['height_rmse_m'], slope_mae_deg=ev['slope_mae_deg'],
                           elapsed_seconds=summary['elapsed_seconds'], path_m=summary['path_m'],
                           actions=len(actions), failed_actions=outcomes['failed'])
            except (OSError, KeyError, ValueError, zipfile.BadZipFile) as error:
                issues.append(f'{label}: evidence read failed: {error}')
    expected_rows = 18 if scene_filter else 54
    if len(rows) != expected_rows:
        issues.append(f'Expected {expected_rows} trial rows; found {len(rows)}')
    audit = dict(status='pass' if not issues else 'fail', trial_count=len(rows),
                 reference_conditions=len(references), issues=issues)
    audit_name = f'audit-{scene_filter}.json' if scene_filter else 'audit.json'
    save(audit_name, audit)
    if issues:
        raise RuntimeError(f'{len(issues)} audit issue(s); inspect {audit_name}')
    if scene_filter:
        print(json.dumps(audit, ensure_ascii=False))
        return
    table('trial_metrics.csv', rows)
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row['scene'], row['start_index'], row['strategy'])].append(row)
    measures = ('S', 'C_obs', 'C', 'P', 'ground_C', 'ground_P', 'terrain_C', 'terrain_P',
                'height_rmse_m', 'slope_mae_deg', 'elapsed_seconds', 'path_m')
    conditions = [dict(scene=scene, start_index=start, strategy=strategy,
                       trials=len(group), **{key: mean(group, key) for key in measures},
                       failed_actions=sum(row['failed_actions'] or 0 for row in group),
                       budget_reached=sum(row['status'] == 'budget_reached' for row in group))
                  for (scene, start, strategy), group in sorted(grouped.items())]
    table('condition_summary.csv', conditions)
    overall = {strategy: dict(trials=sum(row['strategy'] == strategy for row in rows),
                              **{key: mean([c for c in conditions if c['strategy'] == strategy], key)
                                 for key in measures},
                              failed_actions=sum(row['failed_actions'] or 0 for row in rows
                                                 if row['strategy'] == strategy))
               for strategy in strategies}
    pairs = []
    for first, second in itertools.combinations(strategies, 2):
        differences = [next(c['S'] for c in conditions if
                            (c['scene'], c['start_index'], c['strategy']) == (scene, start, first)) -
                       next(c['S'] for c in conditions if
                            (c['scene'], c['start_index'], c['strategy']) == (scene, start, second))
                       for scene in SCENES for start in (1, 2)]
        pairs.append(dict(first=first, second=second, conditions=6,
                          mean_S_difference=statistics.mean(differences),
                          exact_sign_flip_p=sign_flip_p(differences)))
    adjusted = 0
    for rank, index in enumerate(sorted(range(3), key=lambda i: pairs[i]['exact_sign_flip_p'])):
        adjusted = max(adjusted, min(1.0, (3 - rank) * pairs[index]['exact_sign_flip_p']))
        pairs[index]['holm_p'] = adjusted
    table('pairwise_tests.csv', pairs)
    save('summary.json', dict(version='formal-strategy-v2-300s-results-v1',
                              evaluation_version='mapping-acceptance-v1.1',
                              trial_count=54, conditions=conditions, overall=overall,
                              pairwise_tests=pairs, audit=audit))
    print(json.dumps(dict(audit=audit, overall=overall, pairwise_tests=pairs),
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) == 2 else None)
