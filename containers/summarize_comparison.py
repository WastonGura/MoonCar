"""Summarize matched Gazebo runs without treating predicted terrain as measured attitude."""
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, '/work/ros2_ws/src/mooncar_terrain/scripts')
from terrain_map import TerrainMap
from terrain_planner import TerrainPlanner, Vehicle

root = Path('/evidence')
archive = root / 'gazebo-final/survey-20260909T151316328964Z.npz'
terrain = TerrainMap.load(archive)
checker = TerrainPlanner(terrain.elevation(), terrain.resolution, Vehicle(length=.86, width=.72))
report = dict(source='Gazebo execution, timestamped sensor archive and Nav2; no physical hardware',
              archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
              initial_spawn=[3.5, 3., 0.], goal=[4.875, 3.125],
              same_world_reset_per_mode=True, controller='Nav2 FollowPath RPP and Spin',
              physical_footprint_m=[.86, .72], planning_margin_per_side_m=.15,
              trials_per_mode=1, results=[],
              limits='One short route per mode verifies execution only. Wall time is host-load dependent. '
                     'Pitch/roll below are terrain predictions, not measured IMU values. No efficacy claim.')

def length(points):
    return sum(math.hypot(b['x']-a['x'], b['y']-a['y']) for a,b in zip(points, points[1:]))

for mode in ('fixed', 'full'):
    run = json.loads((root / f'compare-{mode}.json').read_text())
    witness = json.loads((root / f'compare-{mode}-check.json').read_text())
    assert run['passed'] and witness['passed'] and run['mode'] == mode
    assert run['goal'] == report['goal'] and run['obstacle'] is None
    path, track = run['final']['path'], run['final']['track']
    predictions = [checker.assess(p['x'], p['y'], p['yaw']) for p in path]
    assert all(p is not None for p in predictions)
    samples = run['samples']
    report['results'].append(dict(mode=mode, passed=True, initial_pose=run['initial']['pose'],
        planned_length_m=length(path), localized_track_length_m=length(track),
        wall_seconds=run['elapsed_seconds'],
        planning_wall_seconds_sampled=sum(b['elapsed']-a['elapsed'] for a,b in zip(samples,samples[1:])
                                          if a['phase']=='planning'),
        predicted_max_pitch_rad=max(p['pitch'] for p in predictions),
        predicted_max_roll_rad=max(p['roll'] for p in predictions),
        minimum_footprint_support=min(1-p['uncertainty'] for p in predictions),
        ground_truth_error_m=witness['ground_truth_error_m'],
        safe_commands_zero=witness['safe_commands_zero']))
(root / 'runtime-comparison.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
