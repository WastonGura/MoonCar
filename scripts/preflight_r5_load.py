#!/usr/bin/env python3
"""No-Gazebo readiness receipt; archived data only validates analysis tools."""
import json
from pathlib import Path

from r5_load_diagnostic import diagnose_round, save, sha
from run_r5_load_campaign import CASES


def main():
    e = Path('/evidence')
    old = Path('/source/reports/evidence/terrain-v2/r5-origin-20261009')
    files = ['observer/libmooncar-origin-observer.so', 'observer/mooncar-image-observer', 'r5-d0.sdf', 'r5-before.sdf']
    verified = {n:sha(e/n) == sha(old/n) for n in files}
    if not all(verified.values()):
        raise ValueError('Model/library changed versus qualified original candidate')
    prior = json.loads((old/'campaign/freeze.json').read_text())
    dart = '/opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so'
    if sha(dart) != prior['libraries'][dart]:
        raise ValueError('Qualified DART runtime changed')
    for n in ('containers/r5_origin_observer/CMakeLists.txt', 'containers/r5_origin_observer/OriginObserver.cc'):
        if sha(Path('/source')/n) != prior['source'][n]:
            raise ValueError('Read-only observer source changed')
    for n in ('r5_origin_capture.py', 'r5_origin_analysis.py', 'check_r5_origin_round.py'):
        if sha(Path('/source/scripts')/n) != sha(old/'campaign/source/scripts'/n):
            raise ValueError('Original observation/command path changed')
    model = json.loads((e/'mass-models/manifest.json').read_text())
    if len(model['tf_urdf']) != 3 or model['models']['original'] != model['models']['restore']:
        raise ValueError('Companion TF URDFs/restoration identity missing')
    for profile in ('original', 'mirror_x', 'restore'):
        if sha(e/'mass-models'/(profile+'.sdf')) != model['models'][profile]:
            raise ValueError('Mass model hash mismatch')
        if sha(e/'mass-models'/(profile+'.urdf')) != model['tf_urdf'][profile]:
            raise ValueError('Unchanged TF model hash mismatch')
    # These old observations are only software fixtures, never new round results.
    archive = {}
    for name in ('01-zero', '03-left-1'):
        target = e/('archive-readiness-'+name)
        archive[name] = diagnose_round(old/'campaign'/name, model['original'], 'original', target)
        if not archive[name]['qualified']:
            raise ValueError('Read-only archived analysis fixture failed')
    container = json.loads((e/'container-start.json').read_text(encoding='utf-8-sig'))[0]
    limits = container['HostConfig']
    if limits['NanoCpus'] != 4_000_000_000 or limits['Memory'] != 6*1024**3 or limits['ShmSize'] != 512*1024**2 or limits['PortBindings']:
        raise ValueError('Resource/port isolation differs from plan')
    environment = dict(v.split('=', 1) for v in container['Config']['Env'])
    if environment.get('ROS_DOMAIN_ID') != '95' or environment.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise ValueError('ROS isolation differs from plan')
    for check in ('derived-preflight-ready', 'boundary-preflight', 'lifecycle-preflight', 'teardown-preflight'):
        receipt = json.loads((e/check/'result.json').read_text())
        if not receipt['passed']:
            raise ValueError('Prior tool preflight failed: '+check)
    save(e/'load-preflight.json', dict(passed=True, prior_hashes=verified, model=model, cases=CASES,
        archive_analysis_tool_only=archive, resource_limits=limits,
        prediction='COM mirror reverses static fore/aft load sign, pivot axle and lateral slip; restore reverts',
        limits=dict(icr_axle_error_m=.02, sliding_axle_rms_min_m_s=.02, sticking_axle_rms_max_m_s=.005,
                    static_load_difference_min_N=.1, weight_error_fraction=.05, com_error_m=.0001),
        windows=dict(static='motion_start-3 to motion_start', steady='motion_start+3 to motion_end-1'),
        production_changed=False, repair_verified=False))


if __name__ == '__main__':
    main()
