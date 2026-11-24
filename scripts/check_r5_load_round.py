#!/usr/bin/env python3
"""One frozen mass intervention, reusing the original real command chain."""
import argparse
import json
from pathlib import Path
import sys

from r5_load_diagnostic import diagnose_round, save, sha


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True); parser.add_argument('--name', required=True)
    parser.add_argument('--mass-profile', choices=('original', 'mirror_x'), required=True)
    parser.add_argument('--restore', action='store_true')
    parser.add_argument('--angular', type=float, required=True)
    a = parser.parse_args()
    manifest = json.loads(Path('/evidence/mass-models/manifest.json').read_text())
    key = 'restore' if a.restore else a.mass_profile
    if a.restore and a.mass_profile != 'original':
        raise ValueError('Restore must select original mass profile')
    model = Path('/evidence/mass-models')/(key+'.sdf')
    if sha(model) != manifest['models'][key]:
        raise ValueError('Frozen model hash mismatch')
    if sha(model.with_suffix('.urdf')) != manifest['tf_urdf'][key]:
        raise ValueError('Unchanged ROS kinematic URDF missing/mismatched')
    import check_r5_origin_round
    sys.argv = [sys.argv[0], '--output', a.output, '--name', a.name, '--model', str(model),
                '--step', '.001', '--linear', '0', '--angular', str(a.angular), '--spawn-yaw', '0']
    code = check_r5_origin_round.main()
    folder = Path(a.output)/a.name
    result = json.loads((folder/'result.json').read_text())
    try:
        if code != 0 or not result['passed']:
            raise ValueError('Original input/capture/basic physical gate failed')
        audit = diagnose_round(folder, manifest[a.mass_profile], a.mass_profile)
        result['load_qualification'] = audit
        result['passed'] = bool(audit['qualified'] and sha(model) == manifest['models'][key])
    except Exception as error:
        result.update(passed=False, load_error=repr(error))
    result['mass_profile'] = a.mass_profile
    result['restore'] = a.restore
    save(folder/'result.json', result)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
