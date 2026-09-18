#!/usr/bin/env python3
"""Deterministic geometric search comparison. No simulated/physical driving claim."""
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/mooncar_terrain/scripts'))
from terrain_planner import TerrainPlanner


def scenario(name):
    cells = []
    for ix in range(20):
        for iy in range(16):
            x, y = (ix + .5) * .25, (iy + .5) * .25
            z, gx, gy, support = 0., 0., 0., 1.
            if name == 'uncertain_patch' and 7 <= ix <= 12 and 6 <= iy <= 9:
                support = .2
            if name == 'rounded_ridge':
                z = .15 * math.exp(-((x - 2.5) ** 2 / .7 + (y - 2.) ** 2 / .2))
                gx, gy = -2 * (x - 2.5) / .7 * z, -2 * (y - 2.) / .2 * z
            if name == 'unobserved_barrier' and ix == 10:
                continue
            if name == 'unclimbable_step' and ix >= 10:
                z = .4
            cells.append(dict(ix=ix, iy=iy, z=z, gx=gx, gy=gy, support=support,
                              roughness=0., valid=True))
    return cells


def run(output):
    report = dict(source='synthetic mathematical terrain; no ROS/Gazebo or vehicle execution',
                  created_at=datetime.now(timezone.utc).isoformat(), resolution=.25, results=[])
    for name in ('flat', 'uncertain_patch', 'rounded_ridge', 'unobserved_barrier', 'unclimbable_step'):
        planner = TerrainPlanner(scenario(name), .25)
        for mode in ('geometry', 'fixed', 'direction', 'uncertainty', 'full'):
            result = planner.plan((.875, 1.875, 0.), (4.125, 1.875), mode)
            assessments = [planner.assess(p['x'], p['y'], p['yaw']) for p in result['path']]
            result.update(scenario=name,
                          predicted_max_pitch=max((a['pitch'] for a in assessments), default=None),
                          predicted_max_roll=max((a['roll'] for a in assessments), default=None),
                          minimum_footprint_support=min((1-a['uncertainty'] for a in assessments), default=None))
            report['results'].append(result)
            print(name, mode, result['status'], round(result['length'], 3), round(result['compute_ms']), flush=True)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    for result in report['results']:
        expected = 'no_path' if result['scenario'] in ('unobserved_barrier', 'unclimbable_step') else 'ok'
        if result['status'] != expected:
            raise SystemExit(f'Unexpected benchmark result: {result["scenario"]}/{result["mode"]}: {result["status"]}')


if __name__ == '__main__':
    run(sys.argv[1] if len(sys.argv) > 1 else 'tmp/terrain-benchmark.json')
