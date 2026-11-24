#!/usr/bin/env python3
"""Reproduce the frozen six-case isolation/candidate roster without publishing."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
from check_r5_physics import sha
from r5_control_analysis import PROFILES

def main(args):
    root=Path(args.output);root.mkdir(parents=True)  # Never overwrite old rounds.
    source=root/'source';source.mkdir()
    for name in ('check_r5_control.py','check_r5_runtime.py','check_r5_physics.py',
                 'r5_runtime_witness.py','r5_control_analysis.py','experiment_motion.py'):
        shutil.copy2(Path(__file__).with_name(name),source/name)
    cases=[('01-legacy-torque','legacy'),('02-d0-torque','d0'),('03-restored-torque','legacy'),
           ('04-A-torque','A'),('05-B-torque','B'),('06-C-torque','C')]
    plan=dict(cases=cases,profiles=PROFILES,step_seconds=.001,diagnostic_limit=20,
              per_round_wall_limit_seconds=600,observer_sha256=sha(args.observer_plugin),
              plugin_sha256=sha(args.plugin),model_sha256=sha(args.model),
              source_sha256={p.name:sha(p) for p in source.glob('*.py')},publication=False)
    (root/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    for name,profile in cases:
        argv=[sys.executable,str(source/'check_r5_control.py'),'--output',str(root),
              '--name',name,'--profile',profile,'--model',args.model,'--plugin',args.plugin,
              '--observer-plugin',args.observer_plugin,'--package',args.package]
        with (root/(name+'.log')).open('w') as stream:
            child=subprocess.Popen(argv,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            try:child.wait(timeout=590)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                (root/'summary.json').write_text(json.dumps(dict(passed=False,status='round_wall_timeout',case=name)))
                raise RuntimeError('Round deadline; stop this exclusive test container before another run')
        result_path=root/name/'result.json'
        if child.returncode!=0 or not result_path.is_file():
            (root/'summary.json').write_text(json.dumps(dict(passed=False,status='capture_failure',case=name)))
            raise RuntimeError('Capture failed; retained all evidence')
        result=json.loads(result_path.read_text())
        print(json.dumps(dict(case=name,passed=result['passed'])),flush=True)
    # Only fixed A/B/C may be selected; D=0 is an isolation case, never fallback.
    sys.path.insert(0,str(source))
    from check_r5_control import select_candidate
    selected=select_candidate(root)
    (root/'summary.json').write_text(json.dumps(selected,indent=2)+'\n')
    print(json.dumps(selected),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True);p.add_argument('--model',required=True)
    p.add_argument('--plugin',required=True);p.add_argument('--observer-plugin',required=True)
    p.add_argument('--package',default='/source/ros2_ws/src/mooncar_gazebo')
    main(p.parse_args())
