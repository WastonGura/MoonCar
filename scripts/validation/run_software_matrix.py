#!/usr/bin/env python3
"""Run every software group, retaining failures instead of stopping at the first."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE='OFF', OPENBLAS_NUM_THREADS='1',
               OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
    for key in ('ROS_STATIC_PEERS', 'ROS_LOCALHOST_ONLY'):
        env.pop(key, None)
    results = []
    def run(name, command):
        begun = time.monotonic()
        with (args.output/(name+'.log')).open('w') as stream:
            try:
                code = subprocess.run(command, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                      timeout=900).returncode
            except subprocess.TimeoutExpired:
                code = 124
        text = (args.output/(name+'.log')).read_text(errors='replace')
        counts = re.findall(r'^Ran (\d+) tests? in ([\d.]+)s', text, re.M)
        row = dict(group=name, command=command, exit_code=code, elapsed_wall_s=time.monotonic()-begun,
                   tests_run=int(counts[-1][0]) if counts else None,
                   summary=[line for line in text.splitlines()
                            if line.startswith(('OK', 'FAILED', 'ERROR:', 'FAIL:'))][:100])
        results.append(row)
        (args.output/'results.json').write_text(json.dumps(results, indent=2)+'\n')
        print(json.dumps(row), flush=True)
        return code
    for package in ('control', 'navigation', 'terrain', 'gazebo', 'slam'):
        run(package, [sys.executable, '-B', '-m', 'unittest', 'discover', '-s',
                      str(ROOT/'ros2_ws/src'/('mooncar_'+package)/'tests'), '-v'])
    run('root', [sys.executable, '-B', '-m', 'unittest', 'discover', '-s', str(ROOT/'tests'), '-v'])
    binary = args.output/'firmware_core'
    compiled = run('firmware-build', ['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=undefined',
        '-I', str(ROOT/'firmware/stm32/core'), str(ROOT/'firmware/stm32/core/mooncar_core.c'),
        str(ROOT/'firmware/stm32/tests/test_core.c'), '-lm', '-o', str(binary)])
    if compiled == 0:
        run('firmware-run', [str(binary)])
    return int(any(row['exit_code'] for row in results))


if __name__ == '__main__':
    raise SystemExit(main())
