#!/usr/bin/env python3
"""Opt-in sensor-only odometry; the selected backend alone owns odom TF."""
from collections import Counter, deque
import json
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException, TransformBroadcaster
from laser_odometry_core import LaserOdometry, rotation, wrap
try:
    from localization_quality import LaserQualitySource
except ModuleNotFoundError:
    # Source-tree execution remains supported; installed nodes use the
    # colocated module installed by mooncar_slam's CMakeLists.
    from pathlib import Path
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
    from localization_quality import LaserQualitySource


def seconds(stamp):return stamp.sec+stamp.nanosec*1e-9


def interpolate(rows,stamp,max_gap):
    """Require bracketing sensor timestamps; never extrapolate a stale sample."""
    if not rows or stamp<rows[0][0] or stamp>rows[-1][0]:return None
    for a,b in zip(rows, list(rows)[1:]):
        if a[0]<=stamp<=b[0]:
            if b[0]-a[0]>2*max_gap:return None
            if min(stamp-a[0],b[0]-stamp)>max_gap:return None
            f=(stamp-a[0])/max(b[0]-a[0],1e-12)
            return np.asarray(a[1:])+f*(np.asarray(b[1:])-a[1:])
    return np.asarray(rows[-1][1:]) if stamp==rows[-1][0] else None


class LaserOdometryNode(Node):
    def __init__(self):
        super().__init__('mooncar_laser_odometry')
        self.declare_parameter('direct_mode',False)
        self.declare_parameter('heading_origin',0.)
        # World-referenced IMUs subtract the declared initial world heading.
        # Startup-referenced IMUs instead measure their initial body heading;
        # Gazebo's default IMU already uses its creation attitude as reference.
        self.declare_parameter('imu_heading_reference','world')
        self.direct_mode=self.get_parameter('direct_mode').value
        self.heading_origin=float(self.get_parameter('heading_origin').value)
        if not math.isfinite(self.heading_origin):raise ValueError('heading_origin must be finite')
        self.imu_heading_reference=self.get_parameter('imu_heading_reference').value
        if self.imu_heading_reference not in ('world','startup'):
            raise ValueError('imu_heading_reference must be world or startup')
        self.imu_heading_offset=None
        self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self)
        self.imu=deque(maxlen=500);self.wheel=deque(maxlen=200);self.pending=deque(maxlen=20)
        self.core=None;self.last_yaw=None;self.counts=Counter();self.last_accepted=None;self.max_compute_s=0.
        self.quality=LaserQualitySource()
        self.publisher=self.create_publisher(Odometry,'/mooncar/laser_odometry',10)
        self.filtered_publisher=self.create_publisher(Odometry,'/odometry/filtered',10) if self.direct_mode else None
        self.localized_scan_publisher=self.create_publisher(LaserScan,'/mooncar/scan_localized',qos_profile_sensor_data) if self.direct_mode else None
        self.tf_broadcaster=TransformBroadcaster(self) if self.direct_mode else None
        self.status=self.create_publisher(String,'/mooncar/laser_odometry/status',10)
        self.create_subscription(Imu,'/mooncar/imu',self.on_imu,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_wheel,50)
        self.create_subscription(LaserScan,'/mooncar/scan_level',self.on_scan,qos_profile_sensor_data)
        # A wall-clock heartbeat keeps missing scans and paused ROS time visible.
        self.create_timer(.1,self.report,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def reject_input(self,reason):
        self.counts[reason]+=1
        self.quality.reject(reason)
        self.clear_estimate_window()
        self.report()

    def clear_estimate_window(self):
        if self.direct_mode and self.core is not None:
            self.core.fresh_window.clear()

    def on_imu(self,msg):
        stamp=seconds(msg.header.stamp)
        if not math.isfinite(stamp):
            self.reject_input('invalid_imu_stamp');return
        if self.imu and stamp<=self.imu[-1][0]:
            self.reject_input('nonincreasing_imu_stamp');return
        try:
            tf=self.buffer.lookup_transform('base_footprint',msg.header.frame_id,Time())
        except TransformException:
            self.reject_input('imu_transform_unavailable');return
        try:
            q=msg.orientation;m=tf.transform.rotation
            R=Rotation.from_quat([q.x,q.y,q.z,q.w])*Rotation.from_quat([m.x,m.y,m.z,m.w]).inv()
        except ValueError:
            self.reject_input('invalid_imu_orientation');return
        gyro=tf.transform.rotation
        body_rate=Rotation.from_quat([gyro.x,gyro.y,gyro.z,gyro.w]).apply(
            [msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
        if not np.isfinite(body_rate).all():
            self.reject_input('invalid_imu_angular_velocity');return
        matrix=R.as_matrix();angle=math.atan2(matrix[1,0],matrix[0,0])
        if self.direct_mode:
            if self.imu_heading_reference=='startup':
                if self.imu_heading_offset is None:self.imu_heading_offset=angle
                angle=wrap(angle-self.imu_heading_offset)
            else:angle=wrap(angle-self.heading_origin)
        yaw=angle if self.last_yaw is None else self.last_yaw+wrap(angle-self.last_yaw)
        self.last_yaw=yaw
        self.imu.append((stamp,yaw,float(body_rate[2])));self.process_pending()

    def on_wheel(self,msg):
        stamp=seconds(msg.header.stamp)
        if not np.isfinite([stamp,msg.twist.twist.linear.x,msg.twist.twist.angular.z]).all():
            self.reject_input('invalid_wheel_input');return
        if self.wheel and stamp<=self.wheel[-1][0]:
            self.reject_input('nonincreasing_wheel_stamp');return
        self.wheel.append((stamp,msg.twist.twist.linear.x,msg.twist.twist.angular.z));self.process_pending()

    def on_scan(self,msg):
        self.counts['received_scans']+=1
        if len(self.pending)==self.pending.maxlen:
            discarded=self.pending.popleft();self.counts['pending_overflow']+=1
            self.quality.observe(seconds(discarded.header.stamp),dict(accepted=False,reason='pending_overflow'))
            self.clear_estimate_window()
            self.report()
        self.pending.append(msg);self.process_pending()

    def process_pending(self):
        while self.pending and self.imu and self.wheel:
            msg=self.pending[0];stamp=seconds(msg.header.stamp)
            if min(self.imu[-1][0],self.wheel[-1][0])<stamp:return
            angle=interpolate(self.imu,stamp,.03);wheel=interpolate(self.wheel,stamp,.08)
            if angle is None or wheel is None:
                self.pending.popleft();self.counts['unsynchronized']+=1
                self.quality.observe(stamp,dict(accepted=False,reason='unsynchronized'))
                self.clear_estimate_window();self.report();continue
            if self.core is None:
                try:tf=self.buffer.lookup_transform('base_footprint',msg.header.frame_id,Time())
                except TransformException:
                    self.reject_input('laser_transform_unavailable');return
                p=tf.transform.translation;q=tf.transform.rotation
                R=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
                if abs(R[2,0])+abs(R[2,1])>1e-3:
                    self.pending.popleft();self.counts['nonplanar_mount']+=1
                    self.quality.observe(stamp,dict(accepted=False,reason='nonplanar_mount'));self.report();continue
                if self.direct_mode:
                    from metric_laser_odometry import MetricLaserOdometry
                    self.core=MetricLaserOdometry((p.x,p.y),math.atan2(R[1,0],R[0,0]))
                else:self.core=LaserOdometry((p.x,p.y),math.atan2(R[1,0],R[0,0]))
            self.pending.popleft();begin=time.perf_counter()
            try:
                result=self.core.update(stamp,np.asarray(msg.ranges),msg.angle_min+np.arange(len(msg.ranges))*msg.angle_increment,
                    msg.range_min,msg.range_max,float(angle[0]),float(wheel[0]),float(wheel[1]))
            except (ValueError,TypeError,FloatingPointError):
                result=dict(accepted=False,reason='invalid_scan_input')
            self.max_compute_s=max(self.max_compute_s,time.perf_counter()-begin)
            self.counts[result['reason']]+=1
            self.quality.observe(stamp,result)
            if not result['accepted'] and result['reason']!='stationary_hold':self.report()
            if result['accepted']:
                result['angular_velocity_z']=float(angle[1]);self.publish(msg,result)
                # Consumers must see the new accepted timestamp with its pose.
                # Waiting for the wall heartbeat adds phase delay to the 5 Hz
                # stationary fits and can falsely expire a current observation.
                self.report()

    def publish(self,scan,result):
        msg=Odometry();msg.header.stamp=scan.header.stamp;msg.header.frame_id='odom';msg.child_frame_id='base_footprint'
        xy=result['xy'];yaw=result['yaw'];cov=result['covariance']
        msg.pose.pose.position.x=float(xy[0]);msg.pose.pose.position.y=float(xy[1])
        msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
        for i in range(2):
            for j in range(2):msg.pose.covariance[i*6+j]=float(cov[i,j])
        for i in (2,3,4,5):msg.pose.covariance[i*6+i]=1e6  # Yaw is an IMU prior, not an independent observation.
        # Twist is descriptive and is not fused by the candidate EKF. Compute both
        # body axes from consecutive accepted body-origin poses, including lever arm.
        if self.last_accepted is not None:
            last_stamp,last_xy=self.last_accepted;dt=result['stamp']-last_stamp
            if dt>0 and (not self.direct_mode or dt<=.3+1e-9):
                velocity=rotation(yaw).T@(xy-last_xy)/dt
                msg.twist.twist.linear.x=float(velocity[0]);msg.twist.twist.linear.y=float(velocity[1])
        for i in range(6):msg.twist.covariance[i*6+i]=1e6
        msg.twist.twist.angular.z=float(result.get('angular_velocity_z',0.))
        self.last_accepted=(result['stamp'],xy.copy());self.publisher.publish(msg);self.counts['published']+=1
        if self.direct_mode:
            # Missing or rejected scans never acquire a fresh TF timestamp.
            self.filtered_publisher.publish(msg)
            transform=TransformStamped();transform.header=msg.header;transform.child_frame_id=msg.child_frame_id
            transform.transform.translation.x=msg.pose.pose.position.x
            transform.transform.translation.y=msg.pose.pose.position.y
            transform.transform.rotation=msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)
            if self.quality.ready_since_stamp is not None:
                self.localized_scan_publisher.publish(scan)
                self.counts['localized_scans']+=1

    def report(self):
        payload=self.quality.snapshot(self.get_clock().now().nanoseconds/1e9)
        if payload['reason']=='observation_stale':self.clear_estimate_window()
        payload.update(counts=dict(self.counts),initialized=bool(self.core and self.core.tree is not None),
            max_compute_s=self.max_compute_s,tf_published=self.direct_mode,truth_inputs=False,
            localization_backend='direct_metric_laser' if self.direct_mode else 'laser_pose_to_ekf',
            imu_heading_reference=self.imu_heading_reference,imu_heading_offset=self.imu_heading_offset,
            stop_refinement_s=1.,stationary_refresh_hz=5.)
        message=String();message.data=json.dumps(payload,allow_nan=False)
        self.status.publish(message)


def main():
    rclpy.init();node=LaserOdometryNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
