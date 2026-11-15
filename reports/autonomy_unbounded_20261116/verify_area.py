"""Independently reconstruct unique observed XY cells from archived depth frames."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/mooncar_terrain/scripts'))
from terrain_geometry import transform_points


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, required=True)
    args = parser.parse_args()
    results = [read(args.raw/name/'case/result.json') for name in ('baseline','candidate')]
    common = min(r['autonomy']['elapsed_seconds'] for r in results)
    checks = []
    for name, result in zip(('baseline','candidate'), results):
        mission = result['autonomy']
        research = args.raw/(name+'-sessions')/result['experiment']/'research'
        final_path = next(research.glob('autonomy-*/quality-end.json'))
        quality = read(final_path)
        archive_path = final_path.with_name('observations.npz')
        observed, matched = set(), set()
        with np.load(archive_path, allow_pickle=False) as archive:
            data = {key: archive[key] for key in ('points','stamps','offsets','translations','quaternions')}
        for i, stamp in enumerate(data['stamps']):
            if stamp > quality['stamp']+1e-8:
                continue
            points = data['points'][data['offsets'][i]:data['offsets'][i+1]]
            world = transform_points(points, data['translations'][i], data['quaternions'][i])
            keys = {tuple(k) for k in np.unique(np.floor(world[:, :2]/quality['resolution']).astype(int), axis=0)}
            observed.update(keys)
            if stamp <= mission['started']+common:
                matched.update(keys)
        assert observed == {(c['ix'],c['iy']) for c in quality['cells']}
        checks.append(dict(strategy=name, archive_reproduces_complete_cell_set=True,
                           total_observed_m2=len(observed)*quality['resolution']**2,
                           common_elapsed_seconds=common,
                           common_elapsed_total_observed_m2=len(matched)*quality['resolution']**2,
                           archive_sha256=hashlib.sha256(archive_path.read_bytes()).hexdigest(),
                           quality_sha256=hashlib.sha256(final_path.read_bytes()).hexdigest()))
    Path(__file__).with_name('area-verification.json').write_text(json.dumps(checks, indent=2)+'\n',
                                                               encoding='utf-8', newline='\n')
    print(json.dumps(checks), flush=True)


if __name__ == '__main__':
    main()
