#!/usr/bin/env python3
"""No Gazebo: bounded writer regressions and late-directory ROS binding probe."""
import argparse,json,os,subprocess,sys,hashlib
from pathlib import Path
from r5_dds_shm import verify_runtime,project_runtime

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True);p.add_argument('--dds-prefix',required=True)
    args=p.parse_args();args.dds_runtime='observed';args.dds_trace='images';args.image_delivery_profile='legacy'
    runtime=project_runtime()
    verify_runtime(args)
    root=Path(args.output);root.mkdir(parents=True,exist_ok=False)
    tests=Path(__file__).resolve().parents[1]/'ros2_ws/src/mooncar_gazebo/tests'
    with (root/'writer-regressions.log').open('x') as log:
        subprocess.run([sys.executable,'-m','unittest','discover','-s',str(tests),'-p','test_dds_shm.py','-v'],
            stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
    folder=root/'late-ros-directory'
    env=dict(os.environ,LD_LIBRARY_PATH=args.dds_prefix+'/lib:'+os.environ.get('LD_LIBRARY_PATH',''),
        AMENT_PREFIX_PATH=args.dds_prefix+':'+os.environ.get('AMENT_PREFIX_PATH',''),
        MOONCAR_DDS_TRACE=str(folder/'trace'))
    code='''import rclpy
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
import time,ctypes,json,os,sys
from pathlib import Path
folder=Path(sys.argv[1]);time.sleep(.2);folder.mkdir()
rclpy.init();node=rclpy.create_node('mooncar_dds_preflight')
counts={'rgb':0,'depth':0}
def received(kind):
    def callback(message):counts[kind]+=1
    return callback
subs=[node.create_subscription(Image,'/mooncar/'+k+'/image',received(k),qos_profile_sensor_data) for k in counts]
pubs=[node.create_publisher(Image,'/mooncar/'+k+'/image',qos_profile_sensor_data) for k in counts]
image=Image();image.header.stamp.sec=1;image.height=1;image.width=1;image.encoding='rgb8';image.step=3;image.data=[1,2,3]
until=time.monotonic()+4
while time.monotonic()<until:
    for publisher in pubs:publisher.publish(image)
    rclpy.spin_once(node,timeout_sec=.05)
(folder/'loaded-libraries.txt').write_text(Path('/proc/self/maps').read_text())
node.destroy_node();rclpy.shutdown()
ctypes.CDLL(sys.argv[2]+'/lib/libmooncar-dds-trace.so').mooncar_dds_flush()
path=folder/('trace.'+str(os.getpid())+'.jsonl')
rows=[json.loads(s) for s in path.read_text().splitlines()]
status=json.loads(Path(str(path)+'.status.json').read_text())
assert status['drained'] and not status['failed'] and status['written']==len(rows)==status['submitted']
assert all(counts.values()),counts
assert any(r['stage']=='segment_initialized' and r['a']==524288 for r in rows)
publish={(r['a'],r['b'],r['c'],r['d']) for r in rows if r['stage']=='rmw_publish'}
receive={(r['a'],r['b'],r['c'],r['d']) for r in rows if r['stage']=='reader_qos'}
assert publish and receive==publish,(publish,receive)
(folder/'result.json').write_text(json.dumps(dict(passed=True,physics_attempt=False,received=counts,
    qos_raw=[list(q) for q in sorted(receive)],rows=len(rows),status=status),indent=2)+'\\n')
'''
    with (root/'late-ros-directory.log').open('x') as log:
        subprocess.run([sys.executable,'-c',code,str(folder),args.dds_prefix],env=env,
            stdout=log,stderr=subprocess.STDOUT,check=True,timeout=30)
    digest=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
    (root/'result.json').write_text(json.dumps(dict(passed=True,physics_attempts=0,
        project_runtime=runtime,
        runtime_manifest_sha256=digest(Path(args.dds_prefix)/'manifest.json'),
        preflight_source_sha256=digest(__file__),test_source_sha256=digest(tests/'test_dds_shm.py')),indent=2)+'\n')

if __name__=='__main__':main()
