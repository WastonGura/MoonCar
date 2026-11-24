"""Score the recorded fixed-view and gimbal autonomy runs at a common elapsed time."""
import argparse
from collections import Counter
import io
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np

here = Path(__file__).resolve()
sys.path.insert(0, str(here.parents[1] / 'v3_eight_minute_comparison_20261117'))
from analyze_duration import analyze, read, rows, write


def archive_counts(path, mission_start, mission_end):
    frames = points = mission_frames = mission_points = 0
    first_part_end = last_stamp = None
    with zipfile.ZipFile(path) as bundle:
        members = [name for name in sorted(bundle.namelist()) if name.startswith('segments/')]
        for name in members or [None]:
            source = io.BytesIO(bundle.read(name)) if name is not None else path
            with np.load(source, allow_pickle=False) as data:
                stamps, offsets = data['stamps'], data['offsets']
                assert len(stamps) + 1 == len(offsets)
                frames += len(stamps)
                points += int(offsets[-1])
                if len(stamps):
                    last_stamp = float(stamps[-1])
                    if first_part_end is None:
                        first_part_end = last_stamp
                begin = np.searchsorted(stamps, mission_start)
                end = np.searchsorted(stamps, mission_end, side='right')
                mission_frames += int(end-begin)
                mission_points += int(offsets[end]-offsets[begin])
    return dict(frames=frames, raw_points=points, segments=len(members) or 1,
                mission_frames=mission_frames, mission_raw_points=mission_points,
                first_part_end_sim=first_part_end, last_stamp_sim=last_stamp)


def instrument(raw, name, summary):
    trial = raw/name
    result = read(trial/'case/result.json')
    session = raw/(name+'-sessions')/result['experiment']
    mission = result['autonomy']
    start, stop = mission['started'], mission['started']+mission['elapsed_seconds']
    protocol = read(trial/'protocol.json')
    assert protocol['camera_mode'] == ('fixed' if name == 'baseline' else 'gimbal')
    assert mission['strategy'] == protocol['autonomy_strategy']
    assert mission.get('active_localization_revisit') is False
    diag = []
    for row in rows(trial/'case/states.jsonl'):
        motion = row['state']['motion'].get('diagnostics', {})
        stamp = motion.get('gimbal_joint_stamp_sim')
        position = motion.get('gimbal_position')
        if position is not None and stamp is not None and start <= stamp <= stop:
            diag.append((stamp, math.degrees(position[0]), math.degrees(position[1])))
    gimbal_actions = [a for a in mission['history'] if a.get('use_pan_tilt')]
    if protocol['camera_mode'] == 'fixed':
        assert not gimbal_actions and not mission.get('frontier_observations')
        assert not mission.get('initial_observation', {}).get('history')
    gimbal = dict(samples=len(set(stamp for stamp, _, _ in diag)),
                  pan_range_deg=[min(p for _, p, _ in diag), max(p for _, p, _ in diag)] if diag else None,
                  tilt_range_deg=[min(t for _, _, t in diag), max(t for _, _, t in diag)] if diag else None,
                  selected_actions=len(gimbal_actions),
                  successful_actions=sum(bool(a.get('success')) for a in gimbal_actions),
                  startup_actions=len(mission.get('initial_observation', {}).get('history', [])),
                  frontier_actions=len(mission.get('frontier_observations', [])))

    retries = Counter()
    plan = Counter()
    longest = gap = 0
    for row in rows(session/'window-localization.jsonl'):
        if row['type'] != 'plan' or not start <= row['planned_stamp_ns']/1e9 <= stop:
            continue
        if row['output']:
            plan['solved'] += 1
            gap = 0
        else:
            plan[row['reason']] += 1
            gap += 1
            longest = max(longest, gap)
        retry = (row.get('frontend') or {}).get('metric_spread_retry')
        if retry is not None:
            retries['attempted'] += 1
            retries['geometry_accepted'] += int(retry['accepted'])
            retries['solved'] += int(row['output'])

    archives = sorted(session.glob('survey-*.npz'))
    archive = archive_counts(archives[-1], start, summary['stamp']) if archives else None
    return dict(gimbal=gimbal, localization=dict(plan_status=dict(plan), longest_no_solution_frames=longest,
        spread_retry=dict(retries)), archive=archive, cleanup=read(trial/'owned-cleanup.json'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    summaries = read(args.output/'summary.json')
    common = min(s['elapsed_seconds'] for s in summaries)
    with np.load(args.output/'reference.npz', allow_pickle=False) as reference:
        results = []
        for name, summary in zip(('baseline', 'candidate'), summaries):
            result = analyze(args.raw, name, summary, reference, common)
            result.update(instrument(args.raw, name, summary),
                          camera_mode=summary['camera_mode'], autonomy_strategy=summary['autonomy_strategy'])
            results.append(result)
    write(args.output/'camera-long-run-summary.json', results)


if __name__ == '__main__':
    main()
