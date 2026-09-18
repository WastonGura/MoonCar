"""Audit and summarize the frozen MoonCar v2 formal strategy campaign."""
import csv
import hashlib
import itertools
import json
import math
import statistics
import zipfile
from collections import Counter, defaultdict
from pathlib import Path

REPORT = Path(__file__).resolve().parent
ROOT = REPORT.parents[3]
EVIDENCE = ROOT / 'tmp/ros_acceptance'
RUN = EVIDENCE / 'formal-strategy-v2-20260916'
REPAIR = EVIDENCE / 'formal-strategy-v2-coordinate-repair-20260916'
FREEZE = ROOT / 'reports/evidence/terrain-workbench/planner-v2-validation-20260916/freeze.json'
SCENES = ('survey', 'visible-ramp', 'visible-curve')
STRATEGIES = ('nearest_view_v2', 'coverage_view_v2', 'surface_view_v2')


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def mean(rows, key):
    values = [row[key] for row in rows if row[key] is not None]
    return statistics.mean(values) if values else None


def write_csv(name, rows, fields):
    with (REPORT / name).open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def check_hashes(folder, issues):
    hashes = read(folder / 'hashes.json')
    for name, expected in hashes.items():
        path = folder / name
        if not path.is_file() or sha(path) != expected:
            issues.append(f'hash mismatch: {path}')
    return len(hashes)


def sign_flip_p(differences):
    observed = abs(statistics.mean(differences))
    values = [abs(statistics.mean(a * d for a, d in zip(signs, differences)))
              for signs in itertools.product((-1, 1), repeat=len(differences))]
    return sum(value >= observed - 1e-10 for value in values) / len(values)


def main():
    protocol = read(RUN / 'protocol.json')
    repair_protocol = read(REPAIR / 'protocol.json')
    repair_manifest = read(REPAIR / 'repair_manifest.json')
    repair_spec = {(item['scene'], item['original_index']): item
                   for item in repair_protocol['replacements']}
    replacements = {(item['scene'], item['original_index']): item
                    for item in repair_manifest['accepted']}
    freeze = read(FREEZE)['files']
    issues = []
    invalid_originals = []
    if not (RUN / 'COMPLETE').is_file() or (RUN / 'FAILED').exists():
        issues.append('campaign completion marker failed')
    if sha(RUN / 'protocol.json') != (RUN / 'protocol.sha256').read_text().strip():
        issues.append('protocol hash mismatch')
    if sha(REPAIR / 'protocol.json') != (REPAIR / 'protocol.sha256').read_text().strip():
        issues.append('coordinate-repair protocol hash mismatch')
    if set(replacements) != set(repair_spec) or len(repair_spec) != 6:
        issues.append('coordinate-repair replacement set differs from predeclared six')
    if protocol['expected_trials'] != 54 or protocol['evaluation'] != 'mapping-acceptance-v1.1':
        issues.append('unexpected protocol')

    rows = []
    hash_files = 0
    reference_by_condition = {}
    for scene in protocol['scene_order']:
        name = scene['directory']
        folder = RUN / name
        plan = read(folder / 'plan.json')['trials']
        trials = read(folder / 'trials.json')
        if len(plan) != 18 or len(trials) != 18:
            issues.append(f'{name}: expected 18 planned and completed trials')
        counts = Counter((r['start_index'], r['strategy']) for r in plan)
        if any(counts[(start, strategy)] != 3 for start in (1, 2) for strategy in STRATEGIES):
            issues.append(f'{name}: unbalanced strategy matrix')
        for entry, planned in zip(trials, plan):
            index = entry['index']
            label = f'{name}/trial-{index:02d}'
            if any(entry.get(key) != planned.get(key) for key in planned):
                issues.append(f'{label}: differs from predeclared plan')
            if entry['start'] != scene['starts'][entry['start_index'] - 1]:
                issues.append(f'{label}: start differs from protocol')
            if entry['budget_seconds'] != 90 or entry['radius'] != 8 or entry['capture_gui']:
                issues.append(f'{label}: common budget/radius/GUI mismatch')
            if entry['status'] in ('starting', 'trial_failed'):
                issues.append(f'{label}: unfinished or failed runner')

            capture = folder / 'captures' / f'trial-{index:02d}'
            evaluation = folder / 'evaluations' / f'trial-{index:02d}'
            research = EVIDENCE / entry['folder'].removeprefix('/evidence/')
            cap = read(capture / 'manifest.json')
            ev = read(evaluation / 'summary.json')
            manifest = read(research / 'manifest.json')
            summary = read(research / 'summary.json')
            mission = read(research / 'mission.json')
            localized = mission['origin']
            with (capture / 'truth.jsonl').open(encoding='utf-8') as stream:
                physical = json.loads(stream.readline())['pose'][:2]
            planned_xy = entry['start'][:2]
            slot = (name, index)
            expected_invalid = slot in repair_spec
            coordinate_invalid = math.dist(localized, planned_xy) > 0.25
            if coordinate_invalid and not expected_invalid:
                issues.append(f'{label}: localized start {localized} differs from planned {planned_xy}')
            if expected_invalid and not coordinate_invalid:
                issues.append(f'{label}: declared coordinate-invalid original is valid')
            if expected_invalid:
                invalid_originals.append(dict(scene=name, index=index, repeat=entry['repeat'],
                                              strategy=entry['strategy'], planned=planned_xy,
                                              localized=localized, physical=physical,
                                              original_S=ev['S']))
            if math.dist(physical, planned_xy) > 0.25:
                issues.append(f'{label}: physical start {physical} differs from planned {planned_xy}')
            if math.dist(localized, physical) > 0.25 and not expected_invalid:
                issues.append(f'{label}: localization/truth start mismatch')
            if cap['status'] != 'captured' or not (capture / 'raw-depth.npz').is_file():
                issues.append(f'{label}: incomplete raw capture')
            if manifest.get('report_status') != 'ready' or manifest.get('capture_errors'):
                issues.append(f'{label}: research report not ready')
            if summary['strategy'] != entry['strategy'] or summary['status'] != entry['status']:
                issues.append(f'{label}: research summary mismatch')
            if ev['version'] != 'mapping-acceptance-v1.1' or not ev['reference_audit']['passed']:
                issues.append(f'{label}: evaluation/reference audit mismatch')
            if not ev['evaluable'] or ev['S'] is None:
                issues.append(f'{label}: not evaluable; preserve as failure')
            elif abs(ev['S'] - 100 * ev['C'] * ev['P']) > 1e-8:
                issues.append(f'{label}: score formula mismatch')
            for key in ('status', 'S', 'C_obs', 'C', 'P', 'height_rmse_m', 'slope_mae_deg'):
                if entry['acceptance'][key] != ev[key]:
                    issues.append(f'{label}: runner/evaluation {key} mismatch')
            condition = (name, entry['start_index'])
            reference = (ev['reference_sha256']['reference.json'], ev['evaluation_area_m2'])
            if condition in reference_by_condition and reference_by_condition[condition] != reference:
                issues.append(f'{label}: reference/denominator changed within condition')
            reference_by_condition[condition] = reference
            for source_name, expected in ev['input_sha256'].items():
                if sha(capture / source_name) != expected:
                    issues.append(f'{label}: raw input hash mismatch: {source_name}')
                hash_files += 1
            ref_dir = folder / 'references' / f'start-{entry["start_index"]:02d}'
            for source_name, expected in ev['reference_sha256'].items():
                if sha(ref_dir / source_name) != expected:
                    issues.append(f'{label}: reference input hash mismatch: {source_name}')
                hash_files += 1
            hash_files += check_hashes(evaluation, issues)
            hash_files += check_hashes(research, issues)
            with zipfile.ZipFile(research / 'source.zip') as source:
                for source_name, expected in freeze.items():
                    if source_name.startswith('docs/'):
                        continue
                    if source_name not in source.namelist() or hashlib.sha256(source.read(source_name)).hexdigest() != expected:
                        issues.append(f'{label}: frozen source mismatch: {source_name}')
                    hash_files += 1
            with (research / 'actions.csv').open(newline='', encoding='utf-8') as stream:
                actions = list(csv.DictReader(stream))
            outcomes = Counter(action['outcome'] for action in actions)
            if len(actions) != summary['actions']:
                issues.append(f'{label}: action count mismatch')
            rows.append(dict(scene=name, start_index=entry['start_index'], start=entry['start'],
                             repeat=entry['repeat'], strategy=entry['strategy'], index=index,
                             status=entry['status'], acceptance_status=ev['status'],
                             S=ev['S'], C_obs=ev['C_obs'], C=ev['C'], P=ev['P'],
                             height_rmse_m=ev['height_rmse_m'], slope_mae_deg=ev['slope_mae_deg'],
                             elapsed_seconds=summary['elapsed_seconds'], path_m=summary['path_m'],
                             actions=len(actions), succeeded=outcomes['succeeded'],
                             failed=outcomes['failed'], interrupted=outcomes['interrupted'],
                             evaluation_area_m2=ev['evaluation_area_m2'],
                             experiment=entry['experiment'], archive=entry['archive'],
                             source_batch='original'))
            if expected_invalid:
                if slot not in replacements:
                    issues.append(f'{label}: missing coordinate-valid replacement')
                    continue
                replacement = replacements[slot]
                spec = repair_spec[slot]
                if any(replacement.get(field) != spec[field] for field in spec):
                    issues.append(f'{label}: replacement differs from repair protocol')
                if replacement.get('attempt', 0) < 1 or replacement.get('attempt', 4) > 3:
                    issues.append(f'{label}: replacement attempt outside cap')
                rfolder = REPAIR / replacement['folder']
                rrow = read(rfolder / 'trials.json')[0]
                rcap = rfolder / 'captures/trial-01'
                reval = rfolder / 'evaluations/trial-01'
                rresearch = EVIDENCE / rrow['folder'].removeprefix('/evidence/')
                rmission = read(rresearch / 'mission.json')
                with (rcap / 'truth.jsonl').open(encoding='utf-8') as stream:
                    rphysical = json.loads(stream.readline())['pose'][:2]
                if any(math.dist(a, b) > 0.25 for a, b in
                       ((rmission['origin'], planned_xy), (rphysical, planned_xy),
                        (rmission['origin'], rphysical))):
                    issues.append(f'{label}: replacement coordinate mismatch')
                if (rrow['start'] != entry['start'] or rrow['scene'] != entry['scene'] or
                        rrow['strategy'] != entry['strategy'] or rrow['budget_seconds'] != 90 or
                        rrow['radius'] != 8 or rrow['capture_gui']):
                    issues.append(f'{label}: replacement condition mismatch')
                rev = read(reval / 'summary.json')
                rsummary = read(rresearch / 'summary.json')
                rmanifest = read(rresearch / 'manifest.json')
                if (read(rcap / 'manifest.json')['status'] != 'captured' or
                        not (rcap / 'raw-depth.npz').is_file() or
                        rmanifest.get('report_status') != 'ready' or rmanifest.get('capture_errors')):
                    issues.append(f'{label}: replacement capture/research incomplete')
                if (rev['version'] != 'mapping-acceptance-v1.1' or not rev['reference_audit']['passed'] or
                        not rev['evaluable'] or abs(rev['S'] - 100 * rev['C'] * rev['P']) > 1e-8):
                    issues.append(f'{label}: replacement evaluation invalid')
                for field in ('status', 'S', 'C_obs', 'C', 'P', 'height_rmse_m', 'slope_mae_deg'):
                    if rrow['acceptance'][field] != rev[field]:
                        issues.append(f'{label}: replacement runner/evaluation {field} mismatch')
                original_ref = read(ref_dir / 'reference.json')
                replacement_ref = read(rfolder / 'references/start-01/reference.json')
                original_ref.pop('created_utc', None)
                replacement_ref.pop('created_utc', None)
                if original_ref != replacement_ref or ev['evaluation_area_m2'] != rev['evaluation_area_m2']:
                    issues.append(f'{label}: replacement reference/denominator changed')
                for source_name, expected in rev['input_sha256'].items():
                    if sha(rcap / source_name) != expected:
                        issues.append(f'{label}: replacement raw input hash mismatch: {source_name}')
                    hash_files += 1
                for source_name, expected in rev['reference_sha256'].items():
                    if sha(rfolder / 'references/start-01' / source_name) != expected:
                        issues.append(f'{label}: replacement reference hash mismatch: {source_name}')
                    hash_files += 1
                hash_files += check_hashes(reval, issues) + check_hashes(rresearch, issues)
                with zipfile.ZipFile(rresearch / 'source.zip') as source:
                    for source_name, expected in freeze.items():
                        if source_name.startswith('docs/'):
                            continue
                        if (source_name not in source.namelist() or
                                hashlib.sha256(source.read(source_name)).hexdigest() != expected):
                            issues.append(f'{label}: replacement frozen source mismatch: {source_name}')
                        hash_files += 1
                with (rresearch / 'actions.csv').open(newline='', encoding='utf-8') as stream:
                    ractions = list(csv.DictReader(stream))
                routcomes = Counter(action['outcome'] for action in ractions)
                if len(ractions) != rsummary['actions'] or rsummary['status'] != rrow['status']:
                    issues.append(f'{label}: replacement action/summary mismatch')
                rows[-1].update(status=rrow['status'], acceptance_status=rev['status'],
                                S=rev['S'], C_obs=rev['C_obs'], C=rev['C'], P=rev['P'],
                                height_rmse_m=rev['height_rmse_m'], slope_mae_deg=rev['slope_mae_deg'],
                                elapsed_seconds=rsummary['elapsed_seconds'], path_m=rsummary['path_m'],
                                actions=len(ractions), succeeded=routcomes['succeeded'],
                                failed=routcomes['failed'], interrupted=routcomes['interrupted'],
                                evaluation_area_m2=rev['evaluation_area_m2'],
                                experiment=rrow['experiment'], archive=rrow['archive'],
                                source_batch=replacement['folder'])
    if len(rows) != 54:
        issues.append(f'expected 54 rows; found {len(rows)}')
    audit = dict(status='pass' if not issues else 'fail', trial_count=len(rows),
                 invalid_originals=len(invalid_originals), accepted_replacements=len(replacements),
                 verified_file_hashes=hash_files, reference_conditions=len(reference_by_condition),
                 frozen_source_files_per_trial=sum(not name.startswith('docs/') for name in freeze),
                 issues=issues)
    (REPORT / 'audit.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    if issues:
        raise RuntimeError(f'{len(issues)} audit issue(s); inspect audit.json')

    write_csv('invalid_coordinate_trials.csv',
              [{**row, 'planned': json.dumps(row['planned']),
                'localized': json.dumps(row['localized']), 'physical': json.dumps(row['physical'])}
               for row in invalid_originals], list(invalid_originals[0]))

    fields = ('scene', 'start_index', 'start', 'repeat', 'strategy', 'index', 'status',
              'acceptance_status', 'S', 'C_obs', 'C', 'P', 'height_rmse_m',
              'slope_mae_deg', 'elapsed_seconds', 'path_m', 'actions', 'succeeded',
              'failed', 'interrupted', 'evaluation_area_m2', 'experiment', 'archive', 'source_batch')
    write_csv('trial_metrics.csv', [{**row, 'start': json.dumps(row['start'])} for row in rows], fields)
    groups = defaultdict(list)
    for row in rows:
        groups[(row['scene'], row['start_index'], row['strategy'])].append(row)
    condition_rows = []
    measures = ('S', 'C_obs', 'C', 'P', 'height_rmse_m', 'slope_mae_deg', 'elapsed_seconds', 'path_m')
    for (scene, start, strategy), trials in sorted(groups.items()):
        condition_rows.append(dict(scene=scene, start_index=start, strategy=strategy,
                                   trials=len(trials), evaluable=sum(row['S'] is not None for row in trials),
                                   **{key: mean(trials, key) for key in measures},
                                   failed_actions=sum(row['failed'] for row in trials),
                                   interrupted_actions=sum(row['interrupted'] for row in trials),
                                   zero_path_trials=sum(row['path_m'] < 0.01 for row in trials),
                                   budget_reached=sum(row['status'] == 'budget_reached' for row in trials),
                                   no_candidates=sum(row['status'] == 'no_candidates' for row in trials)))
    write_csv('condition_summary.csv', condition_rows, list(condition_rows[0]))
    overall = {}
    for strategy in STRATEGIES:
        conditions = [row for row in condition_rows if row['strategy'] == strategy]
        trials = [row for row in rows if row['strategy'] == strategy]
        overall[strategy] = dict(trials=len(trials), **{key: mean(conditions, key) for key in measures},
                                 failed_actions=sum(row['failed'] for row in trials),
                                 interrupted_actions=sum(row['interrupted'] for row in trials),
                                 zero_path_trials=sum(row['path_m'] < 0.01 for row in trials),
                                 budget_reached=sum(row['status'] == 'budget_reached' for row in trials),
                                 no_candidates=sum(row['status'] == 'no_candidates' for row in trials))
    pairs = []
    for first, second in itertools.combinations(STRATEGIES, 2):
        differences = []
        for scene in SCENES:
            for start in (1, 2):
                a = next(row['S'] for row in condition_rows if (row['scene'], row['start_index'], row['strategy']) == (scene, start, first))
                b = next(row['S'] for row in condition_rows if (row['scene'], row['start_index'], row['strategy']) == (scene, start, second))
                differences.append(a - b)
        pairs.append(dict(first=first, second=second, conditions=6,
                          mean_S_difference=statistics.mean(differences),
                          exact_sign_flip_p=sign_flip_p(differences)))
    largest = 0
    for rank, index in enumerate(sorted(range(len(pairs)), key=lambda i: pairs[i]['exact_sign_flip_p'])):
        largest = max(largest, min(1.0, (len(pairs) - rank) * pairs[index]['exact_sign_flip_p']))
        pairs[index]['holm_p'] = largest
    write_csv('pairwise_tests.csv', pairs, list(pairs[0]))
    summary = dict(version='formal-strategy-v2-results-v1', evaluation_version='mapping-acceptance-v1.1',
                   trial_count=len(rows), condition_count=6, all_trial_statuses=dict(Counter(row['status'] for row in rows)),
                   acceptance_statuses=dict(Counter(row['acceptance_status'] for row in rows)),
                   overall=overall, condition_summary=condition_rows, pairwise_tests=pairs,
                   audit=audit)
    (REPORT / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(dict(audit=audit, overall=overall, pairwise_tests=pairs), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
