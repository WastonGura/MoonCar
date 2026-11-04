#!/usr/bin/env python3
"""One isolated slope heading trial; never changes the installed default config."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from urllib.request import build_opener, ProxyHandler
import yaml

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('half_spin', 'gimbal_observe'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sessions', type=Path, required=True)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    opener = build_opener(ProxyHandler({}))
    def state():
        with opener.open('http://127.0.0.1:8766/api/simulation', timeout=3) as response:
            return json.load(response)
    try: existing = state()
    except Exception: existing = None
    if existing is not None: raise RuntimeError('Preserve the existing workbench')
    env = dict(os.environ, ROS_DOMAIN_ID='89', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
        MOONCAR_EXPERIMENT_ROS_DOMAIN_ID='89', MOONCAR_RGBD_CPUSET='0,1',
        MOONCAR_RUNTIME_MANIFEST='/evidence/runtime-baseline.json',
        MOONCAR_VALIDATION_OUTPUT=str(args.output/'fixture'), MOONCAR_RAMP_HEADING_CASE=args.mode,
        OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
    for key in ('ROS_STATIC_PEERS', 'ROS_LOCALHOST_ONLY', 'MOONCAR_LASER_ODOMETRY_MODE'):
        env.pop(key, None)
    def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()
    def write(name, value):
        (args.output/name).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    protocol = dict(mode=args.mode, changes=[], production_defaults_changed=False,
        maximum_new_worlds=1, control_truth_inputs=False, physical_parameters_changed=False)
    if args.mode == 'half_spin':
        from ament_index_python.packages import get_package_share_directory
        source = Path(get_package_share_directory('mooncar_navigation'))
        overlay = args.output/'overlay'; target = overlay/'share/mooncar_navigation'
        shutil.copytree(source, target)
        cfg = target/'config/nav2_params.yaml'; before = yaml.safe_load(cfg.read_text())
        assert before['behavior_server']['ros__parameters']['max_rotational_vel'] == .3
        expected = copy.deepcopy(before)
        expected['behavior_server']['ros__parameters']['max_rotational_vel'] = .15
        cfg.write_text(yaml.safe_dump(expected, sort_keys=False))
        assert yaml.safe_load(cfg.read_text()) == expected
        index = overlay/'share/ament_index/resource_index/packages'
        index.mkdir(parents=True); (index/'mooncar_navigation').write_text('')
        env['AMENT_PREFIX_PATH'] = str(overlay)+':'+env.get('AMENT_PREFIX_PATH', '')
        protocol['changes'] = [dict(parameter='behavior_server.max_rotational_vel', before=.3, after=.15)]
        protocol['private_navigation_package'] = dict(path=str(target),
            original_yaml_sha256=digest(source/'config/nav2_params.yaml'), candidate_yaml_sha256=digest(cfg),
            actual_files={str(p.relative_to(target)): digest(p) for p in target.rglob('*')
                          if p.is_file() and '__pycache__' not in p.parts})
    write('protocol.json', protocol)
    backend = None
    try:
        with (args.output/'backend.log').open('x') as log:
            backend = subprocess.Popen([sys.executable, '-B', str(ROOT/'scripts/validation/launch_backend.py'),
                '--fixture', 'ramp', '--sessions', str(args.sessions)], env=env,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+45
            while True:
                try:
                    value = state()
                    if value.get('phase') == 'idle' and not value.get('task'): break
                except Exception: pass
                if backend.poll() is not None: raise RuntimeError('Validation backend exited')
                if time.monotonic() > deadline: raise TimeoutError('Backend startup timed out')
                time.sleep(.5)
            with (args.output/'driver.log').open('x') as driver_log:
                code = subprocess.run([sys.executable, '-B', str(ROOT/'scripts/validation/run_online_matrix_case.py'),
                    '--case', 'ramp', '--output', str(args.output/'case'), '--sessions', str(args.sessions)],
                    env=env, stdout=driver_log, stderr=subprocess.STDOUT, timeout=1350).returncode
            write('driver-exit.json', dict(exit_code=code))
    finally:
        if backend:
            if backend.poll() is None: backend.send_signal(signal.SIGINT)
            try: code_backend = backend.wait(timeout=30)
            except subprocess.TimeoutExpired:
                backend.terminate(); code_backend = backend.wait(timeout=10)
            write('backend-exit.json', dict(exit_code=code_backend))
    print(json.dumps(dict(mode=args.mode, driver_exit=code, backend_exit=code_backend)), flush=True)
    return code


if __name__ == '__main__': raise SystemExit(main())
