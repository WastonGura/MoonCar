#!/usr/bin/env python3
"""Window localization as the sole ROS odometry authority, with fresh-output gating."""
import os

def _startup_affinity():
    requested=os.environ.get('MOONCAR_RGBD_CPUSET')
    if requested is None:return
    parts=requested.split(',')
    if not parts or any(not x.isascii() or not x.isdecimal() for x in parts):raise ValueError('Invalid RGBD CPU set')
    cpus={int(x) for x in parts}
    if len(cpus)!=len(parts) or not cpus.issubset(os.sched_getaffinity(0)):raise ValueError('Unavailable RGBD CPU set')
    os.sched_setaffinity(0,cpus)
    if os.sched_getaffinity(0)!=cpus:raise RuntimeError('RGBD CPU affinity not applied')

if __name__=='__main__':_startup_affinity()

import argparse
import importlib.util
import json
import math
import signal
from pathlib import Path
import sys
import threading
import time
import uuid

sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
import rclpy
from rclpy.clock import Clock,ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

spec=importlib.util.spec_from_file_location('_window_primary_sensor_transport',Path(__file__).with_name('rgbd_window_shadow.py'))
transport=importlib.util.module_from_spec(spec);sys.modules[spec.name]=transport;spec.loader.exec_module(transport)
from offline_rgbd_window.transport_async import install_async_transport,AsyncTransportProcessor
from offline_rgbd_window.frontend import TrackFrontend
from offline_rgbd_window.navigation import WindowOutputGate
from localization_quality import LaserQualitySource
from terrain_odometry_node import TerrainOdometryNode,imu_at

install_async_transport(transport)
def _processor(**kwargs):
    return AsyncTransportProcessor(frontend=TrackFrontend(track_budget=576),prewarm_landmarks=96,**kwargs)
transport.ShadowProcessor=_processor


class WindowOdometryNode(transport.ShadowNode):
    def __init__(self,args):
        self.publication_lock=threading.RLock()
        super().__init__(args)
        self.output_gate=WindowOutputGate();self.quality=LaserQualitySource(self.epoch)
        self.last_accepted=None;self.last_fit=None;self.sensor_for_publication=None;self.sensor_wz=None
        self.publisher=self.create_publisher(Odometry,'/mooncar/laser_odometry',10)
        self.filtered_publisher=self.create_publisher(Odometry,'/odometry/filtered',10)
        self.tf_broadcaster=TransformBroadcaster(self)
        self.status=self.create_publisher(String,'/mooncar/laser_odometry/status',10)
        self.create_timer(.05,self.report,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.event(type='navigation_authority',backend='rgbd_window',legacy_fallback=False,
            pose_frame='odom',child_frame='base_footprint',covariance='model_conditional',
            prewarm_landmarks=96,track_budget=576,active_revisit=False)

    def prepare(self,pair):
        sensor=super().prepare(pair)
        with self.lock:attitude=imu_at(list(self.imu),pair.stamp_ns/1e9)
        if attitude is None:raise ValueError('missing_publication_imu_bracket')
        self.sensor_for_publication=sensor;self.sensor_wz=float(attitude[1])
        return sensor

    def write_plan(self,row):
        super().write_plan(row)
        if self.closing or not rclpy.ok():
            if row.get('output'):
                self.event(type='navigation_pose_rejected',stamp_ns=row.get('stamp_ns'),reason='closing')
            return
        with self.publication_lock:
            sensor=self.sensor_for_publication
            stamp=None if sensor is None else sensor.stamp_ns
            result,reason=self.output_gate.accept(row,stamp,self.sim_now(),time.monotonic_ns(),self.fault)
            if result is not None:
                heading=math.atan2(sensor.nominal_R_wb[1,0],sensor.nominal_R_wb[0,0])
                image=Image();image.header.stamp.sec,image.header.stamp.nanosec=divmod(stamp,1_000_000_000)
                # Same ROS coordinate, velocity and covariance layout as the
                # original publisher, fed exclusively by the window estimate.
                TerrainOdometryNode.publish(self,image,result,heading,self.sensor_wz)
                self.quality.observe(stamp/1e9,dict(accepted=True,reason='accepted'))
                self.last_fit=dict(accepted=True,reason='accepted',exposure_stamp=stamp/1e9,
                    position_std_m=np.sqrt(np.maximum(0,np.diag(result['covariance']))).tolist(),
                    cache_version=row.get('cache_version'),gauge_stamp_ns=row.get('gauge_stamp_ns'),
                    covariance_kind='model_conditional')
                self.event(type='navigation_pose_published',stamp_ns=stamp,published_wall_ns=time.monotonic_ns(),
                    published_sim_ns=self.sim_now(),cache_version=row.get('cache_version'))
            elif row.get('output'):
                self.event(type='navigation_pose_rejected',stamp_ns=row.get('stamp_ns'),reason=reason)
            self.report()

    def reject(self,reason,stamp=None):
        self.quality.fault=reason;self.quality.reject(reason)

    def report(self):
        if self.closing or not rclpy.ok():return
        with self.publication_lock:
            if self.fault:self.quality.fault=self.fault
            payload=self.quality.snapshot(self.sim_now()/1e9)
            payload.update(localization_backend='rgbd_window',active_revisit=False,
                counts=dict(self.counts),last_fit=self.last_fit,truth_inputs=False,
                tf_published=bool(self.counts['published']),imu_heading_reference='startup',
                imu_heading_offset=self.initial_imu_yaw_rad,legacy_fallback=False)
            self.status.publish(String(data=json.dumps(payload,allow_nan=False)))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,default=Path(os.environ.get('MOONCAR_WINDOW_OUTPUT',f'/tmp/mooncar-window-{uuid.uuid4().hex}.jsonl')))
    p.add_argument('--image-reliability',choices=('reliable','best_effort'),default='reliable')
    p.add_argument('--readiness-wait-wall-s',type=float,default=.5)
    args,ros_args=p.parse_known_args();args.cached_query=True
    if not 0<args.readiness_wait_wall_s<=2:p.error('Invalid readiness wait')
    # Join the foreground/background and seal JSON before destroying ROS.
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    def stop(signum,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    node=WindowOdometryNode(args)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        try:node.finish()
        finally:
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
