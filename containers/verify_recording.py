"""Inspect only the newly recorded archive; no scene mesh or ground truth input."""
import json
from pathlib import Path
import sys
import numpy as np
from terrain_map import TerrainMap

directory = Path(sys.argv[1])
archives = list(directory.glob('survey-*.npz'))
assert len(archives) == 1, f'Expected one recording, got {archives}'
terrain = TerrainMap.load(archives[0])
cloud, cells = terrain.cloud(), terrain.elevation()
positions = np.array([f[2] for f in terrain.frames])
report = dict(source=terrain.source, frame=terrain.frame, archive=archives[0].name,
              frames=len(terrain.frames), voxel_points=len(cloud), elevation_cells=len(cells),
              valid_cells=sum(c['valid'] for c in cells),
              recorded_robot_poses=sum(p is not None for p in terrain.robot_poses),
              camera_displacement=float(np.linalg.norm(np.ptp(positions, axis=0))) if len(positions) else 0.,
              physical_hardware=False, ground_truth_used_for_mapping=False)
report['passed'] = bool('Gazebo' in terrain.source and terrain.frame == 'map' and len(terrain.frames) >= 10
                        and len(cloud) >= 1000 and report['valid_cells'] >= 20
                        and report['recorded_robot_poses'] == len(terrain.frames)
                        and report['camera_displacement'] >= .05)
terrain.export_ply(directory / 'observed-cloud.ply')
(directory / 'recording-check.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(report, ensure_ascii=False, indent=2))
if not report['passed']:
    raise SystemExit('Observed map acceptance failed; inspect recording logs')
