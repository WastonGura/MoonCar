"""Merge updated strategy trials into an existing benchmark suite and regenerate reports."""
import argparse
import csv
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


def merge_trials(target_dir: Path, source_dir: Path, strategies_to_replace: list):
    target_trials_file = target_dir / 'trials.json'
    source_trials_file = source_dir / 'trials.json'
    
    if not target_trials_file.exists():
        raise FileNotFoundError(f"Target trials file not found: {target_trials_file}")
    if not source_trials_file.exists():
        raise FileNotFoundError(f"Source trials file not found: {source_trials_file}")
        
    target_trials = json.loads(target_trials_file.read_text(encoding='utf-8'))
    source_trials = json.loads(source_trials_file.read_text(encoding='utf-8'))
    
    source_by_strategy = {t['strategy']: t for t in source_trials}
    
    for trial in target_trials:
        strat = trial['strategy']
        if strat in strategies_to_replace and strat in source_by_strategy:
            new_trial = source_by_strategy[strat]
            print(f"Checking target trial {trial['index']} ({strat}) with new result: status={new_trial.get('status')}")
            if new_trial.get('status') == 'trial_failed' or 'capture' not in new_trial:
                print(f"WARNING: Skipping strategy {strat} because new trial failed (status='{new_trial.get('status')}')")
                continue
            
            # Copy capture directory
            new_capture_src = Path(new_trial['capture'])
            target_capture_dst = target_dir / 'captures' / f"trial-{trial['index']:02d}-{strat}"
            if new_capture_src.exists():
                if target_capture_dst.exists():
                    shutil.rmtree(target_capture_dst)
                shutil.copytree(new_capture_src, target_capture_dst)
            
            # Copy evaluation directory
            new_eval_src = Path(new_trial['evaluation'])
            target_eval_dst = target_dir / 'evaluations' / f"trial-{trial['index']:02d}-{strat}"
            if new_eval_src.exists():
                if target_eval_dst.exists():
                    shutil.rmtree(target_eval_dst)
                shutil.copytree(new_eval_src, target_eval_dst)
                
            # Update trial record fields while keeping trial index and start intact
            idx = trial['index']
            start = trial['start']
            start_index = trial['start_index']
            trial.update(new_trial)
            trial['index'] = idx
            trial['start'] = start
            trial['start_index'] = start_index
            trial['capture'] = str(target_capture_dst)
            trial['evaluation'] = str(target_eval_dst)
            
    # Save merged trials.json
    target_trials_file.write_text(json.dumps(target_trials, indent=2, ensure_ascii=False), encoding='utf-8')
    
    # Update comparison.csv in target_dir
    comp_csv = target_dir / 'comparison.csv'
    rows = []
    for r in target_trials:
        row = {k: v for k, v in r.items() if k not in ('summary', 'acceptance')}
        row.update(r.get('summary', {}))
        for k, v in r.get('acceptance', {}).items():
            row['acceptance_' + k] = v
        row['start'] = json.dumps(r['start'])
        rows.append(row)
        
    if rows:
        fieldnames = list(rows[0].keys())
        for r in rows:
            for k in r.keys():
                if k not in fieldnames:
                    fieldnames.append(k)
        with open(comp_csv, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
    print(f"Updated {target_trials_file} and {comp_csv}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('target_dir', type=Path, help='Existing benchmark folder to update')
    parser.add_argument('source_dir', type=Path, help='New benchmark folder containing fresh trials')
    parser.add_argument('--strategies', nargs='+', default=['nearest_view_v3', 'coverage_view_v3', 'surface_view_v3'])
    args = parser.parse_args()
    merge_trials(args.target_dir, args.source_dir, args.strategies)
