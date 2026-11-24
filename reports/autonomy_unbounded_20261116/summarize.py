"""Summarize uncropped observed area and trajectories from the two frozen runs."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
from survey_evaluation import metrics, gain


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def rows(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line.strip()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, required=True)
    args = parser.parse_args()
    summaries = []
    for name in ('baseline', 'candidate'):
        folder = args.raw/name
        result = read(folder/'case/result.json')
        mission = result['autonomy']
        if mission['radius'] is not None:
            raise ValueError('This comparison requires radius=null')
        research = args.raw/(name+'-sessions')/result['experiment']/'research'
        quality_path = next(research.glob('autonomy-*/quality-end.json'))
        final, initial = read(quality_path), read(quality_path.with_name('quality-start.json'))
        measured = metrics(final, mission['origin'], None)
        delta = gain(initial, final, mission['origin'], None)
        start, end = mission['started'], mission['started']+mission['elapsed_seconds']
        truth = [r for r in rows(folder/'case/observer/truth.jsonl') if start <= r['stamp'] <= end]
        track, previous = [], -math.inf
        for row in truth:
            if row['stamp']-previous >= .1:
                track.append(row['xyz'][:2]); previous = row['stamp']
        alignment = [r['xy_error_m'] for r in rows(folder/'case/observer/alignment.jsonl') if start <= r['stamp'] <= end]
        recording = read(folder/'video/recording.json')
        history = mission['history']
        summaries.append(dict(strategy=name, experiment=result['experiment'],
            terrain_strategy='baseline' if name == 'baseline' else 'elevation_neighborhood_v1',
            tested_commit=result['software']['git_commit'], radius=None,
            area_scope='All unique actually observed projected cells; no distance or coordinate clipping',
            budget_seconds=mission['budget_seconds'], elapsed_seconds=mission['elapsed_seconds'],
            wall_seconds=result['elapsed_wall_s'], outcome=result['outcome'], reason=mission['reason'],
            total_observed_m2=measured['observed_area_m2'], newly_observed_m2=delta['new_observed_m2'],
            sufficient_area_m2=measured['sufficient_area_m2'], valid_surface_area_m2=measured['valid_surface_area_m2'],
            max_distance_from_start_m=max(math.dist(p, mission['origin']) for p in track),
            truth_track_length_m=sum(math.dist(a, b) for a, b in zip(track, track[1:])),
            chosen_actions=len(history), successful_actions=sum(bool(a.get('success')) for a in history),
            movement_actions=sum(a['kind'] != 'turn' for a in history),
            localization_pauses=len(mission.get('localization_pauses', [])),
            xy_error_m=dict(median=float(np.median(alignment)), p95=float(np.percentile(alignment, 95)),
                            maximum=max(alignment)) if alignment else None,
            quality_stamp=final['stamp'], recorded_depth_keyframes=final['frames'],
            gui_captures=recording['frame_count'], capture_error=recording['error'],
            camera_follow_confirmed=read(folder/'case/camera/ready.json')['confirmed'],
            process_exits={key: read(folder/(key+'-exit.json'))['exit_code'] for key in ('driver','video','backend')}))
    target = Path(__file__).with_name('summary.json')
    target.write_text(json.dumps(summaries, ensure_ascii=False, indent=2)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps(summaries, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
