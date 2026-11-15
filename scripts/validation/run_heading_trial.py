#!/usr/bin/env python3
"""One isolated R5 validation run, with optional full GUI recording."""
import argparse
import copy
import hashlib
import json
import math
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
    parser.add_argument('--mode', choices=('half_spin', 'fixed_observe', 'gimbal_observe', 'feedback_observe','autonomy'), required=True)
    parser.add_argument('--record-video', action='store_true', help='Actual X11 Gazebo GUI frames with per-frame capture timestamps')
    parser.add_argument('--camera-pose', default='1.7 -3.2 2.6 0 0.55 1.016',
                        help='Six Gazebo GUI camera pose coordinates for this recording')
    parser.add_argument('--capture-fps', type=int, default=6)
    parser.add_argument('--terrain-strategy',choices=('baseline','elevation_neighborhood_v1'),default='baseline')
    parser.add_argument('--survey-budget',type=float,default=180.)
    parser.add_argument('--radius',type=float,default=None,help='Optional legacy start radius; omitted means full scene')
    parser.add_argument('--wall-budget',type=float,default=1800.)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sessions', type=Path, required=True)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    if args.record_video and not os.environ.get('DISPLAY'):
        raise RuntimeError('Video requires DISPLAY; start this runner through xvfb-run')
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
    env['MOONCAR_TERRAIN_STRATEGY']=args.terrain_strategy
    if args.mode=='autonomy':
        env.pop('MOONCAR_RAMP_HEADING_CASE',None)
        if args.record_video:env['MOONCAR_VALIDATION_FOLLOW_CAMERA']='1'
    if args.record_video:
        camera = args.camera_pose.split()
        if len(camera)!=6 or not all(math.isfinite(float(v)) for v in camera):
            raise ValueError('The recording camera needs six finite pose coordinates')
        env.update(MOONCAR_VALIDATION_GUI='1',MOONCAR_VALIDATION_CAMERA_POSE=args.camera_pose,
                   MOONCAR_VALIDATION_VIDEO_TAIL_SECONDS='2',
                   MOONCAR_VALIDATION_VIDEO_STOP_FILE=str(args.output/'screen-stop'),
                   MOONCAR_VALIDATION_VIDEO_RECEIPT=str(args.output/'video/recording.json'),LP_NUM_THREADS='1')
    for key in ('ROS_STATIC_PEERS', 'ROS_LOCALHOST_ONLY', 'MOONCAR_LASER_ODOMETRY_MODE'):
        env.pop(key, None)
    def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()
    def write(name, value):
        (args.output/name).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    protocol = dict(mode=args.mode, changes=[], production_defaults_changed=False,
        terrain_strategy=args.terrain_strategy,
        survey_budget=args.survey_budget,radius=args.radius if args.mode=='autonomy' else 8.,wall_budget=args.wall_budget,
        maximum_new_worlds=1, control_truth_inputs=False, physical_parameters_changed=False,
        actual_gui_video=args.record_video, video_timing_comparable_to_headless=False)
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
    backend = None; video = None; video_log = None
    try:
        with (args.output/'backend.log').open('x') as log:
            backend = subprocess.Popen([sys.executable, '-B', str(ROOT/'scripts/validation/launch_backend.py'),
                '--fixture', 'native' if args.mode=='autonomy' else 'ramp', '--sessions', str(args.sessions)], env=env,
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
            if args.record_video:
                video_log=(args.output/'screen-recorder.log').open('x')
                video=subprocess.Popen([sys.executable,'-B',str(ROOT/'scripts/validation/record_screen.py'),
                    '--output',str(args.output/'video'),'--follow-workbench','--fps',str(args.capture_fps),
                    '--stop-file',str(args.output/'screen-stop')],env=env,stdout=video_log,stderr=subprocess.STDOUT)
            with (args.output/'driver.log').open('x') as driver_log:
                code = subprocess.run([sys.executable, '-B', str(ROOT/'scripts/validation/run_online_matrix_case.py'),
                    '--case', 'autonomy' if args.mode=='autonomy' else 'ramp',
                    '--output', str(args.output/'case'), '--sessions', str(args.sessions),
                    *(['--survey-budget',str(args.survey_budget),
                       '--wall-budget',str(args.wall_budget)] if args.mode=='autonomy' else []),
                    *(['--radius',str(args.radius)] if args.mode=='autonomy' and args.radius is not None else [])],
                    env=env, stdout=driver_log, stderr=subprocess.STDOUT,
                    timeout=args.wall_budget+150 if args.mode=='autonomy' else 1350).returncode
            write('driver-exit.json', dict(exit_code=code))
    finally:
        if video:
            if video.poll() is None:video.send_signal(signal.SIGINT)
            try:video_code=video.wait(timeout=30)
            except subprocess.TimeoutExpired:
                video.terminate();video_code=video.wait(timeout=10)
            write('video-exit.json',dict(exit_code=video_code))
        if video_log:video_log.close()
        if backend:
            if backend.poll() is None: backend.send_signal(signal.SIGINT)
            try: code_backend = backend.wait(timeout=30)
            except subprocess.TimeoutExpired:
                backend.terminate(); code_backend = backend.wait(timeout=10)
            write('backend-exit.json', dict(exit_code=code_backend))
    print(json.dumps(dict(mode=args.mode, driver_exit=code, backend_exit=code_backend)), flush=True)
    return code


if __name__ == '__main__': raise SystemExit(main())
