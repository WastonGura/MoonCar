"""Require all witness, isolation and precision regressions before stepping."""
import hashlib
import argparse
import json
import os
from pathlib import Path
import sys
import unittest

MODULES=('test_r5_monitoring','test_image_dds_lifecycle','test_probe_shutdown','test_capture_io','test_r5_precision','test_r5_robot_description','test_r5_ground_reference','test_r5_static_diagnosis',
         'test_r5_suspension','test_turn_slip_control','test_axle_allocation','test_turn_gate_validation','test_origin_diagnostic')

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='/diagnosis/nonphysics-gate.json')
    parser.add_argument('--prepared',default='/diagnosis/prepared');args=parser.parse_args()
    target=Path(args.output)
    if target.exists():raise FileExistsError('Preserve earlier nonphysics gate')
    os.environ['R5_DDS_PREFLIGHT_ROOT']='/diagnosis/dds-preflight'
    sys.path.insert(0,'/source/ros2_ws/src/mooncar_gazebo/tests')
    suite=unittest.defaultTestLoader.loadTestsFromNames(MODULES)
    path=target.with_suffix('.log')
    with path.open('x') as log:
        result=unittest.TextTestRunner(stream=log,verbosity=2).run(suite)
    from run_r5_suspension import frozen
    record=dict(passed=result.wasSuccessful() and not result.skipped,tests=result.testsRun,
        failures=len(result.failures),errors=len(result.errors),skipped=result.skipped,physical_trials=0,
        witness_deadline_seconds=5,parent_wait_seconds=8,
        dds_preflight_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in Path('/diagnosis/dds-preflight').glob('*/result.json')},
        freeze=frozen(args.prepared))
    target.write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps({k:v for k,v in record.items() if k!='freeze'}))
    return 0 if record['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
