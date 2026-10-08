#!/usr/bin/env python3
"""D=0 collection fixture using the production bridge supervisor and stock DDS receivers."""
import importlib.util, json, os, subprocess, sys
from pathlib import Path

def main():
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--name',required=True)
    p.add_argument('--model',required=True);p.add_argument('--step',type=float,required=True)
    a=p.parse_args()
    import check_r5_runtime as runtime
    from argparse import Namespace
    folder=Path(a.output)/a.name
    guard='/source/ros2_ws/src/mooncar_gazebo/launch/image_dds_guard.py'
    original=subprocess.Popen
    def start(argv,*pos,**kw):
        if argv[:4]==['ros2','run','ros_gz_bridge','bridge_node']:
            argv=[sys.executable,guard,'--root',os.environ['MOONCAR_IMAGE_DDS_RUNTIME'],
                '--receipt-directory',str(folder/'production-bridge'),'--binary',
                '/opt/ros/jazzy/lib/ros_gz_bridge/bridge_node']+argv[4:]
        return original(argv,*pos,**kw)
    subprocess.Popen=start
    args=Namespace(output=a.output,name=a.name,model=a.model,step=a.step,scene='flat',kind='drive',linear=0.,angular=.2,
        plugin='/opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so',package='/source/ros2_ws/src/mooncar_gazebo',
        observer_plugin='/evidence/observer/libmooncar-step-observer.so',formal_d0=True,motion_schedule='paused_steps',
        capture_profile='async',image_trace='headers',image_sampling='legacy',image_observer='/evidence/observer/mooncar-image-observer',
        cpp_image_observer=None,image_bridge='stock',image_delivery_profile='legacy',dds_runtime='stock',dds_trace='off',dds_prefix=None)
    error=None
    try:runtime.run(args)
    except Exception as e:error=repr(e)
    finally:subprocess.Popen=original
    result=json.loads((folder/'result.json').read_text()) if (folder/'result.json').exists() else {'passed':False}
    proof=folder/'production-bridge/ready.json'
    receiver_maps=Path('/proc/self/maps').read_text()
    receiver_shm={n:Path(n).stat().st_size for n in {line.split()[-1] for line in receiver_maps.splitlines() if '/dev/shm/' in line} if Path(n).is_file()}
    result['production_runtime']=json.loads(proof.read_text()) if proof.is_file() else None
    result['stock_receiver_libraries']=list(sorted({line.split()[-1] for line in receiver_maps.splitlines() if any(n in line for n in ('librmw_fastrtps','libfastrtps'))}))
    result['receiver_segments']=receiver_shm
    # rclpy shutdown unmaps its segment; the low-rate probe captures it while live in formal launch tests.
    result['passed']=bool(result.get('passed') and proof.is_file() and not error and 'image-dds' not in receiver_maps)
    if error:result['production_error']=error
    (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
