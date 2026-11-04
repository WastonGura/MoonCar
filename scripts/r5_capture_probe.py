#!/usr/bin/env python3
"""Independent lightweight ROS/transport witness; never publishes commands."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu,Image
from geometry_msgs.msg import Twist
from check_r5_physics import timestamp
from r5_capture_io import dds_info,EventWriter,image_receive_info
from r5_probe_shutdown import shutdown_witness

class Probe(Node):
    def __init__(self,folder,image_trace='off'):
        super().__init__('r5_capture_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.folder=folder;self.errors=[];self.event_id=0
        self.writer=EventWriter(folder,self.errors,filename='ros-probe.jsonl',receipt_filename='probe-receipts.jsonl') if image_trace=='headers' else None
        self.stream=None if self.writer else (folder/'ros-probe.jsonl').open('x')
        self.create_subscription(Imu,'/mooncar/imu',self.imu,qos_profile_sensor_data)
        self.create_subscription(Twist,'/mooncar/cmd_vel_safe',self.safe,10)
        if image_trace=='headers':
            self.create_subscription(Image,'/mooncar/rgb/image',lambda m,i:self.image('rgb',m,i),qos_profile_sensor_data)
            self.create_subscription(Image,'/mooncar/depth/image',lambda m,i:self.image('depth',m,i),qos_profile_sensor_data)

    def emit(self,kind,info,**data):
        row=dict(kind=kind,wall=time.monotonic(),sim=self.get_clock().now().nanoseconds/1e9,
            **image_receive_info(info),
            dds_clock='RMW timestamp; not a simulation timestamp',**data)
        self.event_id+=1;row['event_id']=self.event_id
        if self.writer:
            try:self.writer.submit(row)
            except Exception as error:self.errors.append(repr(error))
        else:self.stream.write(json.dumps(row)+'\n')

    def image(self,kind,msg,info):
        self.emit(kind,info,stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
            width=msg.width,height=msg.height)

    def imu(self,msg,info):self.emit('imu',info,stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
    def safe(self,msg,info):self.emit('safe_command',info,values=[msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z],clock='callback ROS simulation clock; headerless message')

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    parser.add_argument('--image-trace',choices=('off','headers'),default='off');args=parser.parse_args()
    folder=Path(args.output);rclpy.init();node=Probe(folder,args.image_trace)
    err=(folder/'native-imu.stderr').open('x')
    child=subprocess.Popen(['gz','topic','-e','--json-output','-t','/mooncar/imu'],stdout=subprocess.PIPE,stderr=err,text=True)
    def native():
        try:
            with (folder/'native-imu.jsonl').open('x') as stream:
                for line in child.stdout:
                    raw=json.loads(line);stream.write(json.dumps(dict(wall=time.monotonic(),stamp=timestamp(raw),raw=raw))+'\n')
        except Exception as error:node.errors.append(repr(error))
    thread=threading.Thread(target=native,daemon=True);thread.start()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        # The parent has an eight-second wait. Repeated SIGINT must not
        # interrupt this five-second drain after callbacks have stopped.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        shutdown_witness(child,thread,node.writer,node.errors,folder)
        err.close()
        if node.stream:node.stream.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
