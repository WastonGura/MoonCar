#!/usr/bin/env python3
"""Read-only endpoint, socket and RTPS-header witness in the test container."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from r5_capture_io import EventWriter

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);args=p.parse_args()
    folder=Path(args.output);errors=[];writer=EventWriter(folder,errors,filename='delivery-transport.jsonl',receipt_filename='delivery-transport-receipts.jsonl')
    rclpy.init();node=Node('r5_delivery_transport_observer');halt=threading.Event();event_id=0
    def stop(*_):halt.set()
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    err=(folder/'rtps-capture.stderr').open('x')
    # Header-only snaplen bounds retained payload; no rotation or overwrite.
    child=subprocess.Popen(['tcpdump','-i','any','-nn','-B','4096','-s','256','-U','-w',str(folder/'rtps-headers.pcap'),'udp'],stderr=err,stdout=subprocess.DEVNULL)
    def emit(**row):
        nonlocal event_id
        event_id+=1;writer.submit(dict(event_id=event_id,wall_ns=time.monotonic_ns(),**row))
    def command(argv):
        proc=subprocess.run(argv,capture_output=True,text=True,timeout=3)
        if proc.returncode:raise RuntimeError(str(argv)+': '+proc.stderr)
        return proc.stdout
    try:
        while not halt.is_set():
            if child.poll() is not None:raise RuntimeError('RTPS capture process exited')
            packet=folder/'rtps-headers.pcap'
            if packet.exists() and packet.stat().st_size>=64*1024**2:raise RuntimeError('RTPS evidence cap reached')
            topics={}
            for topic in ('/mooncar/rgb/image','/mooncar/depth/image'):
                def endpoint(e):
                    q=e.qos_profile
                    return dict(node=e.node_name,gid=list(e.endpoint_gid),type=e.topic_type,
                        reliability=str(q.reliability),durability=str(q.durability),
                        history=None if q.history.value==3 else str(q.history),
                        depth=None if q.history.value==3 else q.depth,
                        history_raw=str(q.history),depth_raw=q.depth)
                topics[topic]=dict(publishers=[endpoint(e) for e in node.get_publishers_info_by_topic(topic)],
                    subscribers=[endpoint(e) for e in node.get_subscriptions_info_by_topic(topic)])
            processes={}
            for path in Path('/proc').iterdir():
                if not path.name.isdigit():continue
                try:
                    cmd=(path/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
                    if not any(n in cmd for n in ('bridge_node','check_r5_runtime','r5_capture_probe','mooncar-ros-image-observer')):continue
                    env=dict(v.split('=',1) for v in (path/'environ').read_bytes().decode(errors='replace').split('\0') if '=' in v)
                    processes[path.name]=dict(command=cmd,environment={k:v for k,v in env.items() if k.startswith(('RMW','ROS','FAST','MOONCAR','LD_LIBRARY','GZ_PARTITION'))},
                        libraries=sorted(set(line.split()[-1] for line in (path/'maps').read_text().splitlines() if '/' in line and any(n in line for n in ('fastrtps','fastdds','rmw','ros_gz_bridge','mooncar-dds-trace')))))
                except (FileNotFoundError,ProcessLookupError,PermissionError):continue
            emit(kind='transport',topics=topics,processes=processes,udp_sockets=command(['ss','-u','-a','-n','-p','-m']),
                proc_udp=Path('/proc/net/udp').read_text(),proc_udp6=Path('/proc/net/udp6').read_text(),
                net_snmp=Path('/proc/net/snmp').read_text(),shm_usage=command(['df','-B1','/dev/shm']),
                shm_files=[dict(name=f.name,bytes=f.stat().st_size) for f in Path('/dev/shm').iterdir() if f.is_file()])
            halt.wait(.5)
    except Exception as e:errors.append(repr(e))
    finally:
        child.send_signal(signal.SIGINT)
        try:child.wait(3)
        except subprocess.TimeoutExpired:child.kill();child.wait();errors.append('RTPS capture forced shutdown')
        err.close()
        text=(folder/'rtps-capture.stderr').read_text()
        if '0 packets dropped by kernel' not in text:errors.append('RTPS capture loss count missing or nonzero')
        status=writer.finish(time.monotonic()+5)
        (folder/'delivery-transport-status.json').write_text(json.dumps(dict(errors=errors,drained=status['drained'],writer=status,
            transport_conclusion='Requires endpoint GUID/DATA or shared-memory payload correlation; discovery UDP is insufficient.'))+'\n')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 1 if errors else 0

if __name__=='__main__':raise SystemExit(main())
