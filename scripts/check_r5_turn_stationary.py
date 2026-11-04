"""Stationary/gimbal release checks with scoped runtime and continuous ROS receive."""
from argparse import Namespace
import argparse
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--name',required=True)
    p.add_argument('--model',required=True);p.add_argument('--kind',choices=('static','gimbal-sweep'),required=True)
    a=p.parse_args();folder=Path(a.output)/a.name
    if folder.exists():raise FileExistsError(folder)
    sys.path.insert(0,'/source/ros2_ws/src/mooncar_gazebo/launch')
    from turn_guard import verify
    from turn_runtime import loaded_runtime
    from image_dds_guard import complete_receipt
    from image_dds_validation import image_files,participants
    root='/opt/mooncar/turn/r5-v2';manifest=verify(root)
    import check_r5_runtime as runtime
    import r5_runtime_witness
    old_start=subprocess.Popen;old_prepare=runtime.prepare;old_command=runtime.command
    old_executor=runtime.SingleThreadedExecutor;old_witness=runtime.Witness
    state={};pool=ThreadPoolExecutor(max_workers=1);stopped=threading.Event();proof={};monitor_errors=[]
    def executor(*args,**kw):state['executor']=old_executor(*args,**kw);return state['executor']
    def witness(*args,**kw):state['node']=old_witness(*args,**kw);return state['node']
    def command(*args,**kw):
        future=pool.submit(old_command,*args,**kw)
        while not future.done():
            if 'executor' in state:
                state['executor'].spin_once(timeout_sec=.02)
                from geometry_msgs.msg import Twist
                state['node'].publisher.publish(Twist())
            else:time.sleep(.02)
        return future.result()
    def prepare(args,out):
        world=old_prepare(args,out)
        import xml.etree.ElementTree as ET
        t=ET.parse(world);model=t.find("world/model[@name='mooncar']")
        for name,label in (('mooncar::AxleAllocation','axle'),('mooncar::TurnSlipControl','slip')):
            ET.SubElement(model.find(f"plugin[@name='{name}']"),'output').text=str(Path('/work')/(a.name+'-'+label+'.csv'))
        t.write(world);return world
    def start(argv,*args,**kw):
        if argv[:2]==['gz','sim']:
            env=dict(kw.get('env',os.environ));env['LD_LIBRARY_PATH']=root+':'+env.get('LD_LIBRARY_PATH','');kw['env']=env
            child=old_start(argv,*args,**kw);state['server']=child;return child
        if argv[:4]==['ros2','run','ros_gz_bridge','bridge_node']:
            argv=[sys.executable,'/source/ros2_ws/src/mooncar_gazebo/launch/image_dds_guard.py','--root',os.environ['MOONCAR_IMAGE_DDS_RUNTIME'],
                '--receipt-directory',str(folder/'production-bridge'),'--binary','/opt/ros/jazzy/lib/ros_gz_bridge/bridge_node']+argv[4:]
        return old_start(argv,*args,**kw)
    def monitor():
        try:
            while not stopped.wait(.2):
                child=state.get('server')
                if child and 'runtime' not in proof:
                    value=loaded_runtime(child.pid,root,manifest)
                    if value:proof['runtime']=value
                receipt=complete_receipt(folder/'production-bridge/ready.json')
                # ready.json is the guard's completed proof, not initialization.json.
                if receipt and 'participants' not in proof:
                    value=participants(receipt['pid'])
                    if value['passed']:proof['participants']=value
        except Exception as error:monitor_errors.append(repr(error))
    subprocess.Popen=start;runtime.prepare=prepare;runtime.command=command
    runtime.SingleThreadedExecutor=executor;runtime.Witness=witness
    worker=threading.Thread(target=monitor,daemon=True);worker.start()
    error=None
    try:
        args=Namespace(output=a.output,name=a.name,model=a.model,scene='flat',step=.001,kind=a.kind,linear=0.,angular=0.,
            plugin=manifest['dart_path'],package='/work/install/share/mooncar_gazebo',
            observer_plugin='/evidence/observer/libmooncar-origin-observer.so',formal_d0=True,motion_schedule='legacy',
            capture_profile='async',image_trace='headers',image_sampling='legacy',image_observer='/evidence/observer/mooncar-image-observer',
            cpp_image_observer=None,image_bridge='stock',image_delivery_profile='legacy',dds_runtime='stock',dds_trace='off',dds_prefix=None)
        runtime.run(args)
    except Exception as failure:error=repr(failure)
    finally:
        stopped.set();worker.join(3);pool.shutdown(wait=True,cancel_futures=True)
        subprocess.Popen=old_start;runtime.prepare=old_prepare;runtime.command=old_command
        runtime.SingleThreadedExecutor=old_executor;runtime.Witness=old_witness
    path=folder/'result.json';r=json.loads(path.read_text()) if path.exists() else {'passed':False}
    images=image_files(folder,[json.loads(x) for x in (folder/'observations.jsonl').read_text().splitlines()]) if (folder/'observations.jsonl').exists() else {'passed':False}
    passed=bool(r.get('passed') and 'runtime' in proof and 'participants' in proof and images['passed'] and not error and not monitor_errors)
    (Path(a.output)/(a.name+'-release.json')).write_text(json.dumps(dict(passed=passed,kind=a.kind,error=error,monitor_errors=monitor_errors,
        proof=proof,images=images,underlying_passed=r.get('passed')),indent=2)+'\n')
    print(json.dumps(dict(name=a.name,passed=passed,error=error,monitor_errors=monitor_errors)),flush=True)
    return 0 if passed else 1

if __name__=='__main__':raise SystemExit(main())
