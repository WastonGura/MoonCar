#!/usr/bin/env python3
"""D=0 collection fixture using the production bridge supervisor and stock DDS receivers."""
import importlib.util, json, os, subprocess, sys, time
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
    from image_dds_validation import receiver_snapshot,image_files,participants,reception_tail,pause_flush
    import r5_runtime_witness
    original_close=r5_runtime_witness.Witness.close
    def close(node):
        # Capture all actual receiver mappings while their ROS participants are still alive.
        snapshots={'main':receiver_snapshot()}
        processes=json.loads((folder/'dds-processes.json').read_text())
        # Child receivers may already have exited in the original runner's finally block;
        # their live snapshots are taken below at command scheduling boundaries.
        (folder/'receiver-before-close.json').write_text(json.dumps(snapshots,indent=2)+'\n')
        return original_close(node)
    r5_runtime_witness.Witness.close=close
    import r5_step_schedule
    original_health=r5_step_schedule.PausedSteps.healthy
    live_receivers={}
    def healthy(schedule):
        result=original_health(schedule)
        process_path=folder/'dds-processes.json'
        if process_path.is_file():
            for label,pid in json.loads(process_path.read_text()).items():
                if label=='bridge' or label in live_receivers:continue
                try:snapshot=receiver_snapshot(pid)
                except FileNotFoundError:continue
                if snapshot['verified']:live_receivers[label]=snapshot
            receipt=folder/'production-bridge/ready.json'
            if receipt.is_file() and 'all_participants' not in live_receivers:
                peers=participants(json.loads(receipt.read_text())['pid'])
                if peers['passed']:live_receivers['all_participants']=peers
        return result
    r5_step_schedule.PausedSteps.healthy=healthy
    original_run=r5_step_schedule.PausedSteps.run
    original_command=runtime.command
    completed=[False]
    def scheduled_run(schedule,linear,angular):
        result=original_run(schedule,linear,angular)
        boundary=result['stop']['after'];until=min(schedule.deadline,time.monotonic()+5)
        while time.monotonic()<until:
            schedule.healthy()
            with schedule.node.lock:tail=reception_tail(list(schedule.node.rows),boundary['sim'],schedule.node.now())
            flush=pause_flush(schedule.observer,boundary)
            if tail['passed'] and flush['passed']:break
            time.sleep(.02)
        else:raise RuntimeError('Final paused CSV/flush/reception tail incomplete')
        receipt=folder/'production-bridge/ready.json'
        roles={schedule.node.image_writer.process.pid:'archive-worker'} if schedule.node.image_writer else {}
        proof=participants(json.loads(receipt.read_text())['pid'],roles)
        if not proof['passed']:raise RuntimeError('Actual participant ownership failed')
        live_receivers['all_participants']=proof
        result.update(reception_tail=tail,csv_flush=flush,ownership=proof)
        completed[0]=True
        return result
    def command(*argv,**kwargs):
        if completed[0] and 'pause: true' in argv:
            # The scheduled final iteration is already actually paused and flushed.
            # Do not dispatch a duplicate cleanup pause request.
            return 'Already confirmed actual final pause; no request dispatched'
        return original_command(*argv,**kwargs)
    runtime.command=command;r5_step_schedule.PausedSteps.run=scheduled_run
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
    finally:
        subprocess.Popen=original;r5_runtime_witness.Witness.close=original_close
        r5_step_schedule.PausedSteps.healthy=original_health
        r5_step_schedule.PausedSteps.run=original_run;runtime.command=original_command
    result=json.loads((folder/'result.json').read_text()) if (folder/'result.json').exists() else {'passed':False}
    proof=folder/'production-bridge/ready.json'
    result['production_runtime']=json.loads(proof.read_text()) if proof.is_file() else None
    result['live_receivers']=live_receivers
    before=folder/'receiver-before-close.json'
    result['receiver_before_close']=json.loads(before.read_text()) if before.is_file() else {}
    result['receiver_verified']=bool({'main','capture-probe','all_participants'}<=live_receivers.keys() and result['receiver_before_close'].get('main',{}).get('verified'))
    observations=folder/'observations.jsonl'
    result['image_content']=image_files(folder,[json.loads(r) for r in observations.read_text().splitlines()]) if observations.is_file() else {'passed':False}
    result['passed']=bool(result.get('passed') and proof.is_file() and not error and result['receiver_verified'] and result['image_content']['passed'])
    if error:result['production_error']=error
    (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
