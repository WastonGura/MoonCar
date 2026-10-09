#!/usr/bin/env python3
"""Isolated origin diagnosis using the proven collection and physical-step fixture."""
import importlib.util, json, os, subprocess, sys, time
from pathlib import Path

def main():
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--name',required=True)
    p.add_argument('--model',required=True);p.add_argument('--step',type=float,required=True)
    p.add_argument('--linear',type=float,default=0.);p.add_argument('--angular',type=float,default=.2)
    p.add_argument('--spawn-yaw',type=float,default=0.)
    a=p.parse_args()
    import check_r5_runtime as runtime
    import r5_d0_analysis
    import r5_runtime_witness
    from r5_origin_capture import OriginWitness
    runtime.Witness=OriginWitness;r5_runtime_witness.Witness=OriginWitness
    from copy import deepcopy
    import xml.etree.ElementTree as ET
    original_prepare=runtime.prepare
    def prepare(args,folder):
        path=original_prepare(args,folder)
        tree=ET.parse(path);tree.find("world/model[@name='mooncar']/pose").text=f'0 0 .03 0 0 {a.spawn_yaw:.17g}'
        tree.write(path)
        return path
    runtime.prepare=prepare
    from image_dds_motion import physical_command_window
    original_window=r5_d0_analysis.command_window
    r5_d0_analysis.command_window=lambda data,result,linear=0.,angular=0.:physical_command_window(data,result,linear,angular,original_window)
    from argparse import Namespace
    folder=Path(a.output)/a.name
    guard='/source/ros2_ws/src/mooncar_gazebo/launch/image_dds_guard.py'
    original=subprocess.Popen
    from image_dds_ownership import Lifecycle
    # The underlying fixture creates its directory; keep lifecycle evidence in
    # its existing output parent until the fixture is ready.
    history=Lifecycle(Path(a.output)/(a.name+'-dds-lifecycle.jsonl'))
    from image_dds_validation import receiver_snapshot,image_files,participants,reception_tail,pause_flush,active_receiver_gate
    import r5_runtime_witness
    original_close=r5_runtime_witness.Witness.close
    def close(node):
        # The original runner has already stopped peer ROS processes here.
        # Preserve these maps as teardown diagnostics; do not require an exited
        # bridge to still hold a creator lock. Active proof is captured below.
        snapshots={'main':receiver_snapshot(history=history)}
        processes=json.loads((folder/'dds-processes.json').read_text())
        # Child receivers may already have exited in the original runner's finally block;
        # their live snapshots are taken below at command scheduling boundaries.
        (folder/'receiver-after-peer-shutdown.json').write_text(json.dumps(snapshots,indent=2)+'\n')
        return original_close(node)
    r5_runtime_witness.Witness.close=close
    import r5_step_schedule
    original_health=r5_step_schedule.PausedSteps.healthy
    live_receivers={}
    def healthy(schedule):
        result=original_health(schedule)
        if history.errors:raise RuntimeError('Lifecycle observation failed: '+repr(history.errors))
        process_path=folder/'dds-processes.json'
        if process_path.is_file():
            for label,pid in json.loads(process_path.read_text()).items():
                if label=='bridge' or label in live_receivers:continue
                try:snapshot=receiver_snapshot(pid,history)
                except FileNotFoundError:continue
                if snapshot['verified']:live_receivers[label]=snapshot
            receipt=folder/'production-bridge/ready.json'
            if receipt.is_file() and 'all_participants' not in live_receivers:
                peers=participants(json.loads(receipt.read_text())['pid'],history=history)
                if peers['passed']:
                    live_receivers['all_participants']=peers;history.quiesce()
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
            with schedule.node.lock:
                odom_tail=all(max((r['stamp'] for r in schedule.node.rows if r['kind']==kind),default=-1)>=boundary['sim']-.1 for kind in ('ros_wheel_odom','gz_native_odom'))
            if tail['passed'] and flush['passed'] and odom_tail:break
            time.sleep(.02)
        else:raise RuntimeError('Final paused CSV/flush/reception tail incomplete')
        receipt=folder/'production-bridge/ready.json'
        roles={schedule.node.image_writer.process.pid:'archive-worker'} if schedule.node.image_writer else {}
        proof=participants(json.loads(receipt.read_text())['pid'],roles,history)
        if not proof['passed']:raise RuntimeError('Actual participant ownership failed')
        (folder/'receiver-before-peer-shutdown.json').write_text(json.dumps(proof,indent=2)+'\n')
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
    args=Namespace(output=a.output,name=a.name,model=a.model,step=a.step,scene='flat',kind='drive',linear=a.linear,angular=a.angular,
        plugin='/opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so',package='/source/ros2_ws/src/mooncar_gazebo',
        observer_plugin='/evidence/observer/libmooncar-origin-observer.so',formal_d0=True,motion_schedule='paused_steps',
        capture_profile='async',image_trace='headers',image_sampling='legacy',image_observer='/evidence/observer/mooncar-image-observer',
        cpp_image_observer=None,image_bridge='stock',image_delivery_profile='legacy',dds_runtime='stock',dds_trace='off',dds_prefix=None)
    error=None
    try:runtime.run(args)
    except Exception as e:error=repr(e)
    finally:
        subprocess.Popen=original;r5_runtime_witness.Witness.close=original_close
        r5_step_schedule.PausedSteps.healthy=original_health
        r5_step_schedule.PausedSteps.run=original_run;runtime.command=original_command
        r5_d0_analysis.command_window=original_window
        lifecycle_capture=history.close()
    result=json.loads((folder/'result.json').read_text()) if (folder/'result.json').exists() else {'passed':False}
    proof=folder/'production-bridge/ready.json'
    result['production_runtime']=json.loads(proof.read_text()) if proof.is_file() else None
    result['live_receivers']=live_receivers
    result['lifecycle_capture']=lifecycle_capture
    before=folder/'receiver-before-peer-shutdown.json'
    result['receiver_before_close']=json.loads(before.read_text()) if before.is_file() else {}
    result['receiver_verified']=active_receiver_gate(live_receivers,result['receiver_before_close'])
    post=folder/'receiver-after-peer-shutdown.json'
    result['receiver_after_peer_shutdown']=json.loads(post.read_text()) if post.is_file() else {}
    observations=folder/'observations.jsonl'
    result['image_content']=image_files(folder,[json.loads(r) for r in observations.read_text().splitlines()]) if observations.is_file() else {'passed':False}
    result['raw_fixture_passed']=result.get('passed',False)
    if a.linear==0 and a.angular==0:
        # The general drive fixture requires 1 m forward displacement. Zero
        # input is instead qualified by its existing static/contact/stop gates.
        result['passed']=all((result.get('data_complete'),result.get('ground_contact_gate'),result.get('stop_zero'),
            result.get('parked',{}).get('passed'),result.get('max_tilt_rad',1)<=.1,
            result.get('stop_drift_last_second_m',1)<=.01,result.get('observer_flush_confirmed'),
            result.get('capture',{}).get('drained'),result.get('step_observation',{}).get('passed'),
            result.get('formal',{}).get('passed'),result.get('capture_analysis',{}).get('passed'),
            result.get('image_trace_analysis',{}).get('passed'),result.get('dds',{}).get('passed')))
    result['passed']=bool(result.get('passed') and proof.is_file() and not error and result['receiver_verified'] and result['image_content']['passed'] and lifecycle_capture['passed'])
    result['origin_parameters']=dict(linear=a.linear,angular=a.angular,spawn_yaw=a.spawn_yaw)
    if error:result['production_error']=error
    try:
        from r5_origin_analysis import qualify_round,analyze
        result['origin_qualification']=qualify_round(folder,result)
        result['passed']=result['passed'] and result['origin_qualification']['passed']
    except Exception as failure:
        result.update(passed=False,origin_error=repr(failure))
    (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    if result['passed']:
        analyze(folder,folder/'origin-analysis')
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
