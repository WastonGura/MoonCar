#!/usr/bin/env python3
"""Start an owned validation backend, optionally selecting the shipped ramp fixture."""
import argparse
from pathlib import Path
import runpy
import sys

p = argparse.ArgumentParser(); p.add_argument('--sessions', required=True)
p.add_argument('--fixture', choices=('native', 'ramp'), required=True)
args = p.parse_args()
root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root/'scripts'))
if args.fixture == 'ramp':
    import ramp_fixture
sys.argv = [str(root/'scripts/map_workbench.py'), '--sessions', args.sessions,
            '--simulation', '--host', '0.0.0.0', '--port', '8766']
runpy.run_path(sys.argv[0], run_name='__main__')
