#!/usr/bin/env python3
"""No-Gazebo actual bridge library/capacity verification before a campaign freeze."""
import argparse, importlib.util, json, os, signal, subprocess, sys, time
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument('--runtime',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    folder=Path(a.output);folder.mkdir(parents=True)
    path=Path('/source/ros2_ws/src/mooncar_gazebo/launch/image_dds_guard.py')
    spec=importlib.util.spec_from_file_location('image_dds_guard',path);guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)
    manifest=guard.verify(a.runtime)
    from r5_dds_shm import project_runtime
    project=project_runtime()
    from image_dds_validation import installed_files,model_preflight
    installed=installed_files()
    models=model_preflight(folder/'models')
    log=(folder/'bridge.log').open('x')
    child=subprocess.Popen([sys.executable,str(path),'--root',a.runtime,'--receipt-directory',str(folder/'bridge'),
        '--binary','/opt/ros/jazzy/lib/ros_gz_bridge/bridge_node','--ros-args','-p',
        'config_file:=/work/install/share/mooncar_gazebo/config/ros_gz_bridge_r5.yaml'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        until=time.monotonic()+12
        while not (folder/'bridge/ready.json').exists():
            if child.poll() is not None or time.monotonic()>until:raise RuntimeError('Actual bridge preflight failed')
            time.sleep(.05)
        proof=json.loads((folder/'bridge/ready.json').read_text())
        result=dict(passed=True,project_runtime=project,installed=installed,models=models,manifest_sha256=guard.sha(Path(a.runtime)/'manifest.json'),actual_bridge=proof)
        (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    finally:
        from image_dds_validation import stop_process
        stop_process(child)
        log.close()

if __name__=='__main__':main()
