#!/usr/bin/env python3
"""Generate explicitly synthetic observations for software/UI tests, not evidence of SLAM."""
import argparse
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/mooncar_terrain/scripts'))
from terrain_map import TerrainMap


def generate(path):
    terrain = TerrainMap(source='合成算法测试数据 · 非 Gazebo 采集 / 非实测')
    x, y = np.meshgrid(np.arange(-.975, 5, .05), np.arange(-1.975, 2, .05))
    z = .10 * np.exp(-((x - 2.3) ** 2 + (y - .5) ** 2) / .35)
    z += .12 * np.clip(x - 3., 0, 1.)
    points = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
    # A missing patch is absent observation, never a zero-elevation platform.
    points = points[~((points[:, 0] > 1.4) & (points[:, 0] < 2.1) & (points[:, 1] < -.5) & (points[:, 1] > -1.2))]
    stamp = 1.
    for center in np.arange(0., 4.1, .5):
        sample = points[np.abs(points[:, 0] - center) < 1.]
        for _ in range(3):
            pose = np.array([center, 0., .4])
            terrain.add_frame(sample - pose, stamp, pose,
                              robot_translation=(center, 0., 0.), robot_quaternion=(0., 0., 0., 1.))
            stamp += 1.
    terrain.save(path)
    print(f'SYNTHETIC fixture: {path}, {len(terrain.cloud())} voxel points', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    generate(parser.parse_args().output)
