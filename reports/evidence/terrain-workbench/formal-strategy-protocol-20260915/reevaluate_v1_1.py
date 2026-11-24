"""Re-evaluate the completed 54 captures under the frozen v1.1 reference rule."""
import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts'))
from report_mapping_acceptance import evaluate, prepare
from survey_evaluation import write_csv, write_json

SOURCE = Path('/evidence/formal-strategy-20260915')
OUTPUT = Path('/evidence/formal-strategy-20260915-reevaluated-v1_1-20260916')
SCENES = ('survey', 'visible-ramp', 'visible-curve')
ACCEPTANCE = ('status', 'S', 'C_obs', 'C', 'P', 'height_rmse_m',
              'slope_mae_deg', 'baseline_S', 'delta_S', 'evaluation_area_m2')


def csv_row(trial):
    return {**{k: v for k, v in trial.items() if k not in ('summary', 'acceptance')},
            **trial.get('summary', {}),
            **{'acceptance_' + k: v for k, v in trial['acceptance'].items()},
            'start': json.dumps(trial['start'])}


def run():
    if not (SOURCE / 'COMPLETE').is_file() or (SOURCE / 'FAILED').exists():
        raise ValueError('Source campaign is not complete')
    if OUTPUT.exists():
        raise ValueError('Preserve previous reevaluation: output already exists')
    OUTPUT.mkdir()
    all_trials = []
    try:
        for scene in SCENES:
            source = SOURCE / scene
            target = OUTPUT / scene
            target.mkdir(); (target / 'references').mkdir(); (target / 'evaluations').mkdir()
            plan = json.loads((source / 'plan.json').read_text(encoding='utf-8'))
            trials = json.loads((source / 'trials.json').read_text(encoding='utf-8'))
            if len(trials) != 18 or any(t['status'] == 'trial_failed' for t in trials):
                raise ValueError(f'Unexpected source trials: {scene}')
            shutil.copyfile(source / 'plan.json', target / 'plan.json')
            references = {}
            for trial in trials:
                index = trial['start_index']
                if index not in references:
                    folder = target / 'references' / f'start-{index:02d}'
                    references[index] = folder
                    prepare(source / 'references' / f'start-{index:02d}-world.sdf', folder,
                            trial['start'][:2], trial['radius'], trial['budget_seconds'])
            revised = []
            for trial in trials:
                row = copy.deepcopy(trial)
                result = evaluate(Path(trial['capture']), references[trial['start_index']],
                                  target / 'evaluations' / f"trial-{trial['index']:02d}")
                row['original_evaluation'] = row['evaluation']
                row['evaluation'] = str(target / 'evaluations' / f"trial-{trial['index']:02d}")
                row['acceptance'] = {key: result.get(key) for key in ACCEPTANCE}
                revised.append(row); all_trials.append(copy.deepcopy(row))
                write_json(target / 'trials.json', revised)
            write_csv(target / 'comparison.csv', [csv_row(t) for t in revised])
        write_json(OUTPUT / 'trials.json', all_trials)
        write_csv(OUTPUT / 'comparison.csv', [csv_row(t) for t in all_trials])
        statuses = {}
        for trial in all_trials:
            status = trial['acceptance']['status']; statuses[status] = statuses.get(status, 0) + 1
        write_json(OUTPUT / 'summary.json', {
            'version': 'mooncar-formal-reevaluation-v1',
            'evaluation_version': 'mapping-acceptance-v1.1',
            'created_utc': datetime.now(timezone.utc).isoformat(),
            'source_campaign': str(SOURCE), 'source_trials': len(all_trials),
            'acceptance_statuses': statuses,
            'simulation_rerun': False,
            'interpretation': 'Same 54 frozen captures reevaluated uniformly; no strategy or threshold changes.'})
        shutil.copyfile(ROOT / 'reports/evidence/terrain-workbench/research-freeze-20260916/freeze.json', OUTPUT / 'research-freeze-v4.json')
        hashes = {str(p.relative_to(OUTPUT)): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in OUTPUT.rglob('*') if p.is_file() and p.name != 'hashes.json'}
        write_json(OUTPUT / 'hashes.json', hashes)
        (OUTPUT / 'COMPLETE').write_text(datetime.now(timezone.utc).isoformat() + '\n')
    except Exception:
        (OUTPUT / 'FAILED').write_text(datetime.now(timezone.utc).isoformat() + '\n')
        raise


if __name__ == '__main__':
    run()
