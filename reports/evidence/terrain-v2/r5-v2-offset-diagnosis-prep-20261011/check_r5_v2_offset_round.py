#!/usr/bin/env python3
"""Single diagnostic round runner connecting independent worlds to the ROS control chain.

Reuses the proven paused_steps execution and safety relay from check_r5_turn_gate_validation,
injecting the verified independent world replicas and recording diagnostic screen outcomes.
"""
import argparse
import json
import shutil
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

SCREEN_ERROR = "ValueError('Existing 1 cm turn screen failed')"

def main():
    parser = argparse.ArgumentParser(description="MoonCar R5 single diagnostic round runner")
    parser.add_argument("--output", required=True, help="Output directory")
    parser.add_argument("--name", required=True, help="Round name")
    parser.add_argument("--world", required=True, help="Path to independent world SDF replica")
    parser.add_argument("--angular", type=float, default=0.2, help="Command angular velocity")
    args = parser.parse_args()

    folder = Path(args.output) / args.name
    world_src = Path(args.world)
    if not world_src.exists():
        raise FileNotFoundError(f"World file not found: {world_src}")

    import check_r5_runtime as runtime
    original_prepare = runtime.prepare

    def prepare(run_args, out):
        # Execute original prepare for validation and folder setup, then inject our replica
        path = original_prepare(run_args, out)
        shutil.copy2(world_src, path)
        return path

    runtime.prepare = prepare
    old_argv = sys.argv
    sys.argv = [
        'round',
        '--output', args.output,
        '--name', args.name,
        '--profile', 'pure_turn',
        '--scene', 'ramp',
        '--step', '0.001',
        '--linear', '0.0',
        '--angular', str(args.angular)
    ]

    try:
        import check_r5_turn_gate_validation
        code = check_r5_turn_gate_validation.main()
    finally:
        runtime.prepare = original_prepare
        sys.argv = old_argv

    # Read and qualify round results
    res_path = folder / 'result.json'
    if not res_path.exists():
        raise RuntimeError(f"Execution produced no result.json in {folder}")

    r = json.loads(res_path.read_text(encoding='utf-8'))
    probe = r.get('turn_slip_probe', {})
    trajectory = probe.get('trajectory', {})
    disp = trajectory.get('maximum_displacement_m', float('nan'))

    # Check whether the legacy 1 cm acceptance threshold was exceeded
    screen_passed = bool(disp <= 0.01)

    # Rigorous physical and sensor qualification checks
    checks = {
        'data_complete': bool(r.get('data_complete')),
        'ground_contact_gate': bool(r.get('ground_contact_gate')),
        'stop_zero': bool(r.get('stop_zero')),
        'observer_flush_confirmed': bool(r.get('observer_flush_confirmed')),
        'receiver_verified': bool(r.get('receiver_verified')),
        'raw_fixture_passed': bool(r.get('raw_fixture_passed')),
        'capture_drained': bool(r.get('capture', {}).get('drained')),
        'gate': bool(probe.get('gate', {}).get('passed')),
        'allocation': bool(probe.get('allocation', {}).get('passed')),
        'contact_source': bool(probe.get('source', {}).get('proof', {}).get('passed')),
        'no_critical_errors': not any(r.get(k) for k in ('error', 'production_error', 'origin_error', 'secondary_collection_error')),
        'finite_geometry': bool(disp == disp)
    }

    # An expected diagnostic screen exceeding 10 mm produces code=1 with SCREEN_ERROR
    is_screen_error = (code == 1 and r.get('turn_slip_error') == SCREEN_ERROR)
    checks['valid_exit'] = (code == 0) if screen_passed else is_screen_error

    eligible = all(checks.values())
    diag_result = {
        'round_id': args.name,
        'eligible': eligible,
        'checks': checks,
        'geometric_screen_passed': screen_passed,
        'diagnostic_screen_exceeded': not screen_passed,
        'maximum_displacement_m': disp,
        'final_displacement_xy_m': probe.get('endpoint_xy_m'),
        'vehicle_accepted': False,
        'world_consumed': str(world_src),
        'turn_slip_error': r.get('turn_slip_error')
    }

    (folder / 'diagnostic-result.json').write_text(json.dumps(diag_result, indent=2) + '\n', encoding='utf-8')
    return 0 if eligible else 1

if __name__ == '__main__':
    raise SystemExit(main())
