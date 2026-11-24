#!/usr/bin/env python3
"""Isolated experimental ROS controller: LaserScan + IMU + safe request only.

Private String wire protocol v1: version sequence sec nsec front valid.
No /odom, /tf, poses, contacts or Gazebo state are subscribed to. The actuator
independently expires feedback; this node never publishes cmd_vel or wheel speed.
"""
import argparse
from collections import deque
import math
from pathlib import Path
import signal
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from r5_adaptive_turn import AdaptiveAllocation, scan_points, fit_scan_motion, body_delta
from r5_capture_io import EventWriter


def ns(stamp):return stamp.sec*1_000_000_000+stamp.nanosec


def tilt(msg):
    q=msg.orientation
    norm=math.sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z)
    if msg.orientation_covariance[0] < 0 or not math.isfinite(norm) or abs(norm-1) > .01:
        raise ValueError('Invalid or unavailable IMU orientation')
    roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
    pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
    return roll,pitch


class AdaptiveNode(Node):
    def __init__(self, output):
        super().__init__('mooncar_adaptive_turn_candidate')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.errors=[];self.output=Path(output)
        self.writer=EventWriter(self.output,self.errors,filename='adaptive-feedback.jsonl',
                                receipt_filename='adaptive-writer-receipts.jsonl')
        self.imu=deque(maxlen=1024);self.last_imu_ns=-1
        self.previous=None;self.previous_ns=None;self.previous_tilt=None
        self.controller=AdaptiveAllocation();self.request=(0.,0.)
        self.request_wall=-math.inf;self.request_ns=-1;self.sequence=0;self.event_id=0
        self.publisher=self.create_publisher(String,'/mooncar/turn_allocation',10)
        self.sensor_subscriptions=[
            self.create_subscription(Imu,'/mooncar/imu',self.on_imu,qos_profile_sensor_data),
            self.create_subscription(LaserScan,'/mooncar/scan',self.on_scan,qos_profile_sensor_data),
            self.create_subscription(Twist,'/mooncar/cmd_vel_safe',self.on_request,10)]
        self.create_timer(.05,self.health)

    def health(self):
        if self.errors:raise RuntimeError('Adaptive evidence writer failed: '+repr(self.errors))

    def record(self,row):
        self.event_id+=1
        row['event_id']=self.event_id
        self.writer.submit(row)

    def on_request(self,msg):
        self.request=(msg.linear.x,msg.angular.z)
        self.request_wall=time.monotonic();self.request_ns=self.get_clock().now().nanoseconds

    def on_imu(self,msg):
        stamp=ns(msg.header.stamp)
        if stamp <= self.last_imu_ns:return
        self.last_imu_ns=stamp
        try:
            angles=tilt(msg)
            gyro=np.array([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
            if msg.header.frame_id != 'imu_link' or not np.isfinite(gyro).all():raise ValueError('IMU frame or gyro invalid')
            self.imu.append((stamp,*angles,*gyro))
        except ValueError as exc:
            self.record(dict(kind='imu_rejected',stamp_ns=stamp,reason=str(exc),wall=time.monotonic()))

    def on_scan(self,msg):
        stamp=ns(msg.header.stamp);now=self.get_clock().now().nanoseconds
        row=dict(kind='adaptive',stamp_ns=stamp,received_sim_ns=now,wall=time.monotonic(),request=list(self.request))
        decision=dict(valid=False,reason='unqualified_sensor_feedback',front=1.,rear=1.)
        try:
            if self.errors:raise ValueError('Evidence writer failed')
            if msg.header.frame_id != 'lidar_scan_frame':raise ValueError('Unexpected laser frame')
            if not -20_000_000 <= now-stamp <= 250_000_000:raise ValueError('Laser feedback stale/future')
            if now-self.request_ns > 150_000_000 or time.monotonic()-self.request_wall > .5:raise ValueError('Safe request stale')
            samples=[r for r in self.imu if abs(r[0]-stamp)<=50_000_000]
            if not samples:raise ValueError('Missing time-aligned IMU')
            latest=min(samples,key=lambda r:abs(r[0]-stamp));angles=latest[1:3]
            points=scan_points(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max)
            row.update(points=len(points),imu_stamp_ns=latest[0],roll=angles[0],pitch=angles[1])
            if self.previous_ns is None:
                self.previous,self.previous_ns,self.previous_tilt=points,stamp,angles
                raise ValueError('First scan has no independent motion feedback')
            dt=(stamp-self.previous_ns)*1e-9
            if not .04 <= dt <= .25:raise ValueError('Scan interval or ordering invalid')
            if max(abs(a-b) for a,b in zip(angles,self.previous_tilt)) > .01:
                raise ValueError('Changing scan plane; planar feedback unqualified')
            span=[r for r in self.imu if self.previous_ns <= r[0] <= stamp]
            if len(span)<3:raise ValueError('Insufficient gyro samples')
            omega=float(np.mean([r[5] for r in span]))
            registration=fit_scan_motion(self.previous,points,omega*dt)
            row.update(registration=registration,dt=dt,measured_omega=omega)
            if not registration['valid']:raise ValueError(registration['reason'])
            delta=body_delta(registration);velocity=delta/dt
            row.update(measured_body_delta_m=delta.tolist(),measured_body_velocity_m_s=velocity.tolist())
            decision=self.controller.update(stamp,dt,float(velocity[1]),omega,*self.request)
        except (ValueError,np.linalg.LinAlgError) as exc:
            decision['reason']=str(exc)
        finally:
            # Advance only monotonically; duplicate messages cannot re-use a lease.
            if self.previous_ns is None or stamp > self.previous_ns:
                if 'points' in locals() and 'angles' in locals():
                    self.previous,self.previous_ns,self.previous_tilt=points,stamp,angles
            self.sequence+=1
            message=String();message.data=f'1 {self.sequence} {stamp//1000000000} {stamp%1000000000} {decision["front"]:.17g} {int(decision["valid"])}'
            self.publisher.publish(message)
            row.update(decision=decision,wire=message.data)
            self.record(row)

    def close(self):
        drained=self.writer.finish(time.monotonic()+5)
        import json
        (self.output/'adaptive-node-status.json').write_text(json.dumps(dict(drained=drained,errors=self.errors,
            frames=self.sequence,inputs=['/mooncar/scan','/mooncar/imu','/mooncar/cmd_vel_safe'],
            truth_input=False,wheel_odometry_input=False))+'\n')
        return drained['drained'] and not self.errors


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);a,_=parser.parse_known_args()
    rclpy.init();node=AdaptiveNode(a.output);done=False
    def stop(*_):
        nonlocal done
        done=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    code=0
    try:
        while not done:rclpy.spin_once(node,timeout_sec=.05)
    except Exception:
        code=1;raise
    finally:
        if not node.close():code=1
        node.destroy_node();rclpy.shutdown()
    return code


if __name__=='__main__':raise SystemExit(main())
