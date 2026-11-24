#!/usr/bin/env python3
"""Start an owned validation backend, optionally selecting the shipped ramp fixture."""
import argparse
from pathlib import Path
import runpy
import sys

p = argparse.ArgumentParser(); p.add_argument('--sessions', required=True)
p.add_argument('--fixture', choices=('native', 'ramp'), required=True)
p.add_argument('--lock-camera', action='store_true')
args = p.parse_args()
if args.lock_camera and args.fixture != 'native':
    p.error('--lock-camera requires the native validation fixture')
root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root/'scripts'))
if args.fixture == 'ramp':
    import ramp_fixture
if args.lock_camera:
    # Use the same robot and forward mount, but give this validation process
    # no gimbal actuator for startup, frontier or any other fallback.
    from experiment_motion import ExperimentMotion
    ExperimentMotion.pan_tilt = None
sys.argv = [str(root/'scripts/map_workbench.py'), '--sessions', args.sessions,
            '--simulation', '--host', '0.0.0.0', '--port', '8766']
runpy.run_path(sys.argv[0], run_name='__main__')
