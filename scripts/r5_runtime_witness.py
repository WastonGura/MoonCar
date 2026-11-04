"""Isolated R5 5 s parked / 20 s straight drive / 2 s stop scene trials."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan, Imu

from check_r5_physics import timestamp
from r5_capture_io import EventWriter,ImageWriter,encode_image,capture_config,dds_info,image_receive_info
from r5_image_trace import decision

class Witness(Node):
    def __init__(self,scene,folder,env,sample_resources=False,capture_profile='legacy',deadline=None,image_trace='off',image_sampling='legacy',image_observer_sha256=None):
        super().__init__('v2_r5_trial_'+scene,parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.folder=folder;self.rows=[];self.counts={};self.truth=[];self.errors=[]
        self.lock=threading.Lock();self.capture_profile=capture_profile;self.deadline=deadline
        self.image_trace=image_trace;self.image_sampling=image_sampling;self.saved_ns={};self.image_observer_sha256=image_observer_sha256
        self.event_writer=EventWriter(folder,self.errors) if capture_profile=='async' else None
        self.log=(folder/'observations.jsonl').open('w') if self.event_writer is None else None
        self.image_writer=ImageWriter(folder,self.errors,self.append_row) if capture_profile=='async' else None
        self.saved={};self.latest={}
        self.publisher=self.create_publisher(Twist,'/cmd_vel',10)
        self.create_subscription(Twist,'/mooncar/cmd_vel_safe',self.safe,10)
        self.create_subscription(Image,'/mooncar/rgb/image',lambda m,i:self.image('rgb',m,i),qos_profile_sensor_data)
        self.create_subscription(Image,'/mooncar/depth/image',lambda m,i:self.image('depth',m,i),qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/mooncar/scan',self.scan,qos_profile_sensor_data)
        self.create_subscription(Imu,'/mooncar/imu',self.imu,qos_profile_sensor_data)
        self.raw_pose=(folder/'link-poses.jsonl').open('w')
        self.gzlog=(folder/'truth-stderr.log').open('w')
        self.gz=subprocess.Popen(['gz','topic','-e','-t','/model/mooncar/pose','--json-output'],
                                 stdout=subprocess.PIPE,stderr=self.gzlog,text=True,env=env,start_new_session=True)
        self.thread=threading.Thread(target=self.read_truth,daemon=True);self.thread.start()
        self.command_log=(folder/'gz-input-timed.jsonl').open('w')
        self.command_err=(folder/'gz-input-timed.stderr').open('w')
        self.command_proc=subprocess.Popen(['gz','topic','-e','-t','/cmd_vel','--json-output'],
            stdout=subprocess.PIPE,stderr=self.command_err,text=True,env=env,start_new_session=True)
        self.command_thread=threading.Thread(target=self.read_commands,daemon=True);self.command_thread.start()
        self.resource_stop=threading.Event();self.resource_thread=None
        if sample_resources:
            self.resource_thread=threading.Thread(target=self.read_resources,daemon=True);self.resource_thread.start()

    def read_resources(self):
        root=Path('/sys/fs/cgroup')
        with (self.folder/'resources.jsonl').open('w') as stream:
            while not self.resource_stop.is_set():
                try:
                    cpu=dict(line.split() for line in (root/'cpu.stat').read_text().splitlines())
                    stream.write(json.dumps(dict(wall=time.monotonic(),sim=self.now(),
                        cpu_usage_us=int(cpu['usage_usec']),memory_bytes=int((root/'memory.current').read_text()),
                        cpu_max=(root/'cpu.max').read_text().strip(),memory_max=(root/'memory.max').read_text().strip()))+'\n')
                    stream.flush()
                except Exception as failure:
                    self.errors.append('resource observer: '+str(failure));return
                self.resource_stop.wait(1.)

    def now(self): return self.get_clock().now().nanoseconds/1e9
    def record(self,kind,**data):
        row=dict(kind=kind,sim=self.now(),wall=time.monotonic(),**data)
        return self.append_row(row)

    def append_row(self,row):
        with self.lock:
            row=dict(row,event_id=len(self.rows)+1);self.rows.append(row)
            if self.log is not None:self.log.write(json.dumps(row,allow_nan=False)+'\n')
        if self.event_writer:
            try:self.event_writer.submit(row)
            except Exception as error:self.errors.append('event submit: '+repr(error))
        return row

    def safe(self,m,info):
        values=[m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z]
        self.record('safe_command',values=values,**dds_info(info),clock='callback ROS simulation clock; headerless message')

    def image(self,kind,m,info):
        received=time.monotonic();received_sim=self.now()
        self.counts[kind]=self.counts.get(kind,0)+1;self.latest[kind]=m
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        stamp_ns=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
        previous=self.saved_ns.get(kind)
        selected=decision(stamp_ns,previous,self.image_sampling)
        if self.image_trace=='headers':
            self.record('image_trace',image_kind=kind,stamp_ns=stamp_ns,stamp=stamp,received_wall=received,
                received_sim=received_sim,width=m.width,height=m.height,previous_selected_ns=previous,
                selected=selected,sampling_mode=self.image_sampling,**image_receive_info(info))
        if not selected:return
        self.saved_ns[kind]=stamp_ns
        self.saved[kind]=stamp
        job=dict(kind=kind,stamp=stamp,data=bytes(m.data),height=m.height,width=m.width,step=m.step,
            encoding=m.encoding,bigendian=m.is_bigendian)
        row=dict(kind=kind,stamp=stamp,wall=received,sim=received_sim,received_wall=received)
        try:
            if self.image_writer:self.image_writer.submit(job,row)
            else:self.record(kind,stamp=stamp,received_wall=received,**encode_image(self.folder,job))
        except Exception as error:self.errors.append('image submit/encode: '+repr(error))
        self.record('image_callback',image_kind=kind,stamp=stamp,callback_started_wall=received,
            callback_duration_s=time.monotonic()-received,
            pending_images=len(self.image_writer.pending) if self.image_writer else 0)

    def scan(self,m):
        self.counts['scan']=self.counts.get('scan',0)+1
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        if stamp-self.saved.get('scan',-10)<.5:return
        self.saved['scan']=stamp
        values=np.asarray(m.ranges)
        self.record('scan',stamp=stamp,finite=int(np.isfinite(values).sum()),
                    angle_min=m.angle_min,angle_increment=m.angle_increment,
                    range_min=m.range_min,range_max=m.range_max,
                    ranges=[float(v) if math.isfinite(v) else None for v in values])

    def imu(self,m,info):
        self.counts['imu']=self.counts.get('imu',0)+1
        self.record('imu',stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                    **dds_info(info),
                    angular=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],
                    linear=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])

    def read_truth(self):
        try:
            for line in self.gz.stdout:
                if not line.strip():continue
                data=json.loads(line); self.raw_pose.write(line)
                body=next((p for p in data.get('pose',[]) if p.get('name')=='mooncar'),None)
                if body is None:continue
                p,q=body.get('position',{}),body.get('orientation',{})
                x,y,z,w=[float(q.get(k,1 if k=='w' else 0)) for k in ('x','y','z','w')]
                roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
                pitch=math.asin(max(-1,min(1,2*(w*y-z*x))))
                row=self.record('truth',stamp=timestamp(body),x=float(p.get('x',0)),y=float(p.get('y',0)),z=float(p.get('z',0)),
                                quaternion=[x,y,z,w],roll=roll,pitch=pitch)
                self.truth.append(row)
        except Exception as error:self.errors.append(str(error))

    def read_commands(self):
        try:
            for line in self.command_proc.stdout:
                if not line.strip():continue
                msg=json.loads(line)
                values=[float(msg.get(axis,{}).get(k,0)) for axis in ('linear','angular') for k in ('x','y','z')]
                row=self.record('gz_command',values=values,clock='observer ROS simulation clock',raw=msg)
                self.command_log.write(json.dumps(row)+'\n')
        except Exception as error:self.errors.append('command observer: '+str(error))

    def close(self):
        self.resource_stop.set()
        if self.resource_thread:self.resource_thread.join(timeout=2)
        os.killpg(self.command_proc.pid,signal.SIGINT)
        try:self.command_proc.wait(timeout=4)
        except subprocess.TimeoutExpired:os.killpg(self.command_proc.pid,signal.SIGKILL);self.command_proc.wait()
        self.command_thread.join(timeout=2);self.command_log.close();self.command_err.close()
        os.killpg(self.gz.pid,signal.SIGINT)
        try:self.gz.wait(timeout=4)
        except subprocess.TimeoutExpired:os.killpg(self.gz.pid,signal.SIGKILL);self.gz.wait()
        self.thread.join(timeout=2);self.raw_pose.close();self.gzlog.close()
        deadline=self.deadline if self.deadline is not None else time.monotonic()+30
        status=dict(config=capture_config(self.capture_profile,self.image_trace,self.image_sampling,self.image_observer_sha256),errors=self.errors)
        if self.image_writer:status['images']=self.image_writer.finish(deadline)
        if self.event_writer:status['events']=self.event_writer.finish(deadline)
        if self.log:self.log.close()
        status['drained']=not self.errors and all(v.get('drained',True) for v in status.values() if isinstance(v,dict))
        (self.folder/'capture-status.json').write_text(json.dumps(status,indent=2)+'\n')
        self.destroy_node();return status
