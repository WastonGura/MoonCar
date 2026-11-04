#!/usr/bin/env python3
"""Sensor-only depth odometry with timestamped camera pose and full IMU tilt.

The core receives measured 3D points in startup-aligned odom axes. It owns
reference maintenance and registration acceptance; missing frames never publish
a new pose. The published base TF contains XYZ and yaw only because terrain
projection separately applies the original synchronized IMU roll/pitch.
"""
from collections import Counter,deque
import json
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import rclpy
from rclpy.clock import Clock,ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo,Image,Imu
from std_msgs.msg import String
from tf2_ros import Buffer,TransformBroadcaster,TransformException,TransformListener

from laser_odometry_node import interpolate,seconds
from metric_depth_odometry import MetricDepthOdometry,project_depth
try:
    from localization_quality import LaserQualitySource,MAX_OBSERVATION_AGE,MAX_STATUS_WALL_AGE,FUTURE_TOLERANCE
except ModuleNotFoundError:
    from pathlib import Path
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
    from localization_quality import LaserQualitySource,MAX_OBSERVATION_AGE,MAX_STATUS_WALL_AGE,FUTURE_TOLERANCE


def outside_rover(points):
    """Nominal rover body/tyre/mast envelope in base_footprint, in metres.

    Vehicle geometry only: fixed on the robot, so it cannot supply an external
    position constraint. This contains no terrain/world geometry or pose input.
    """
    return ~((np.abs(points[:,0])<=.50)&(np.abs(points[:,1])<=.45)&
             (points[:,2]>=-.10)&(points[:,2]<=.60))


def imu_at(rows,stamp,max_gap=.03):
    """Interpolate orientation on SO(3), requiring genuine timestamp brackets."""
    if not rows or stamp<rows[0][0] or stamp>rows[-1][0]:return None
    for a,b in zip(rows,list(rows)[1:]):
        if a[0]<=stamp<=b[0]:
            if b[0]-a[0]>2*max_gap or min(stamp-a[0],b[0]-stamp)>max_gap:return None
            fraction=(stamp-a[0])/(b[0]-a[0])
            orientation=Slerp([a[0],b[0]],Rotation.from_quat([a[1],b[1]]))([stamp])[0]
            return orientation,a[2]+fraction*(b[2]-a[2])
    return (Rotation.from_quat(rows[-1][1]),rows[-1][2]) if stamp==rows[-1][0] else None


class TerrainOdometryNode(Node):
    node_name='mooncar_terrain_odometry'
    backend_name='metric_depth_3d'
    input_contract='Measured depth + CameraInfo + timestamped camera TF + bracketing full IMU + wheel initial guess; no map, world geometry or truth input'

    def create_core(self):
        return MetricDepthOdometry(asynchronous_references=True)

    def image_qos(self):
        return qos_profile_sensor_data

    def __init__(self):
        super().__init__(self.node_name)
        self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self)
        self.core=self.create_core();self.quality=LaserQualitySource()
        self.imu=deque(maxlen=500);self.wheel=deque(maxlen=200);self.pending=deque(maxlen=10)
        self.heading_offset=None;self.intrinsics=None;self.camera_frame=None;self.camera_size=None
        self.last_depth_stamp=None;self.last_accepted=None;self.counts=Counter();self.max_compute_s=0.
        self.last_fit=None
        self.publisher=self.create_publisher(Odometry,'/mooncar/laser_odometry',10)
        self.filtered_publisher=self.create_publisher(Odometry,'/odometry/filtered',10)
        self.tf_broadcaster=TransformBroadcaster(self)
        self.status=self.create_publisher(String,'/mooncar/laser_odometry/status',10)
        self.create_subscription(CameraInfo,'/mooncar/depth/camera_info',self.on_info,self.image_qos())
        self.create_subscription(Image,'/mooncar/depth/image',self.on_depth,self.image_qos())
        self.create_subscription(Imu,'/mooncar/imu',self.on_imu,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_wheel,50)
        self.create_timer(.1,self.report,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def reject(self,reason,stamp=None):
        self.counts[reason]+=1
        if stamp is None:self.quality.reject(reason)
        else:self.quality.observe(stamp,dict(accepted=False,reason=reason))
        self.pending.clear()
        self.report()

    def destroy_node(self):
        self.core.close()
        return super().destroy_node()

    def on_info(self,msg):
        values=(msg.k[0],msg.k[4],msg.k[2],msg.k[5])
        if (not msg.header.frame_id or msg.width<=0 or msg.height<=0 or
                not np.isfinite(values).all() or min(values[:2])<=0):
            self.reject('invalid_camera_info');return
        incoming=(tuple(values),msg.header.frame_id,(msg.width,msg.height))
        previous=(self.intrinsics,self.camera_frame,self.camera_size)
        if self.intrinsics is not None and incoming!=previous:
            # Intrinsics/frame changes require a new producer, never blend epochs.
            self.quality.fault='camera_calibration_changed'
            self.reject('camera_calibration_changed');return
        self.intrinsics,self.camera_frame,self.camera_size=incoming
        self.process_pending()

    def on_imu(self,msg):
        stamp=seconds(msg.header.stamp)
        if not math.isfinite(stamp) or (self.imu and stamp<=self.imu[-1][0]):
            self.reject('nonincreasing_imu_stamp');return
        try:tf=self.buffer.lookup_transform('base_footprint',msg.header.frame_id,Time())
        except TransformException:
            self.reject('imu_transform_unavailable');return
        q=msg.orientation;m=tf.transform.rotation
        try:
            mount=Rotation.from_quat([m.x,m.y,m.z,m.w])
            body=Rotation.from_quat([q.x,q.y,q.z,q.w])*mount.inv()
            rate=mount.apply([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
        except ValueError:
            self.reject('invalid_imu_orientation');return
        if not np.isfinite(rate).all():self.reject('invalid_imu_angular_velocity');return
        matrix=body.as_matrix();heading=math.atan2(matrix[1,0],matrix[0,0])
        if self.heading_offset is None:self.heading_offset=heading
        # Left multiplication changes only the yaw reference; measured tilt stays.
        body=Rotation.from_rotvec([0.,0.,-self.heading_offset])*body
        self.imu.append((stamp,body.as_quat(),float(rate[2])))
        self.process_pending()

    def on_wheel(self,msg):
        stamp=seconds(msg.header.stamp)
        values=(stamp,msg.twist.twist.linear.x,msg.twist.twist.angular.z)
        if not np.isfinite(values).all() or (self.wheel and stamp<=self.wheel[-1][0]):
            self.reject('invalid_wheel_input');return
        self.wheel.append(values);self.process_pending()

    def on_depth(self,msg):
        stamp=seconds(msg.header.stamp)
        if not math.isfinite(stamp) or (self.last_depth_stamp is not None and stamp<=self.last_depth_stamp):
            self.reject('nonincreasing_depth_stamp',stamp);return
        self.last_depth_stamp=stamp
        if len(self.pending)==self.pending.maxlen:self.reject('depth_queue_overflow')
        self.pending.append((msg,time.monotonic()))
        self.process_pending()

    def process_pending(self):
        if self.quality.fault:
            self.pending.clear();return
        while self.pending and self.intrinsics is not None and self.imu and self.wheel:
            msg,received=self.pending[0];stamp=seconds(msg.header.stamp)
            now=self.get_clock().now().nanoseconds/1e9
            if now-stamp>MAX_OBSERVATION_AGE+1e-9:
                self.reject('depth_observation_stale',stamp);return
            if stamp>now+FUTURE_TOLERANCE+1e-9:
                self.reject('depth_observation_from_future',stamp);return
            if min(self.imu[-1][0],self.wheel[-1][0])<stamp:return
            attitude=imu_at(self.imu,stamp);wheel=interpolate(self.wheel,stamp,.08)
            if attitude is None or wheel is None:
                self.reject('unsynchronized',stamp);return
            if msg.header.frame_id!=self.camera_frame or (msg.width,msg.height)!=self.camera_size:
                self.reject('depth_camera_info_mismatch',stamp);return
            try:
                # Gimbal orientation is time varying: latest TF would move points
                # into a different camera pose from the image being registered.
                tf=self.buffer.lookup_transform('base_footprint',msg.header.frame_id,
                    Time.from_msg(msg.header.stamp))
            except TransformException:
                if time.monotonic()-received>.5:self.reject('camera_transform_unavailable',stamp)
                return
            self.pending.popleft();begin=time.perf_counter()
            try:
                points=project_depth(msg.data,msg.width,msg.height,msg.step,msg.encoding,msg.is_bigendian,
                    self.intrinsics,stride=4,pixel_offset=.5)
                q=tf.transform.rotation;p=tf.transform.translation
                extrinsic=Rotation.from_quat([q.x,q.y,q.z,q.w])
                lever=np.asarray([p.x,p.y,p.z],dtype=float)
                if not np.isfinite(lever).all():raise ValueError('Invalid camera lever arm')
                body_points=extrinsic.apply(points)+lever
                external=outside_rover(body_points)
                self.counts['self_depth_returns_omitted']+=int((~external).sum())
                body_points=body_points[external]
                orientation,wz=attitude
                measured=orientation.apply(body_points)
                matrix=orientation.as_matrix();heading=math.atan2(matrix[1,0],matrix[0,0])
                result=self.core.update(stamp,measured,heading,float(wheel[0]),float(wheel[1]))
            except (ValueError,TypeError,FloatingPointError):
                self.reject('invalid_depth_input',stamp);return
            self.max_compute_s=max(self.max_compute_s,time.perf_counter()-begin)
            # Computation cannot renew a measurement's age. Initial surface
            # construction can be slow; discard its publication and recover
            # only from subsequent genuinely fresh images.
            after=self.get_clock().now().nanoseconds/1e9
            if (after-stamp>MAX_OBSERVATION_AGE+1e-9 or
                    time.monotonic()-received>MAX_STATUS_WALL_AGE):
                self.reject('depth_observation_stale_after_compute',stamp);return
            self.counts[result['reason']]+=1
            self.last_fit={name:result[name] for name in (
                'accepted','reason','stamp','rmse_m','inliers','overlap','reference_id','reference_frames',
                'reference_pending','reference_build_failures','reference_support') if name in result}
            if result.get('accepted') and 'covariance' in result:
                self.last_fit['position_std_m']=np.sqrt(np.diag(result['covariance'])).tolist()
            self.quality.observe(stamp,result)
            if result['accepted']:
                self.publish(msg,result,heading,wz)
            else:
                self.pending.clear()
            self.report()

    def publish(self,image,result,heading,wz):
        xyz=np.asarray(result['xyz'],dtype=float);cov=np.asarray(result['covariance'],dtype=float)
        if xyz.shape!=(3,) or cov.shape!=(3,3) or not np.isfinite(xyz).all() or not np.isfinite(cov).all():
            self.reject('invalid_registered_pose');return
        msg=Odometry();msg.header.stamp=image.header.stamp;msg.header.frame_id='odom';msg.child_frame_id='base_footprint'
        msg.pose.pose.position.x=float(xyz[0]);msg.pose.pose.position.y=float(xyz[1]);msg.pose.pose.position.z=float(xyz[2])
        msg.pose.pose.orientation.z=math.sin(heading/2);msg.pose.pose.orientation.w=math.cos(heading/2)
        for i in range(3):
            for j in range(3):msg.pose.covariance[i*6+j]=float(cov[i,j])
        for i in (3,4,5):msg.pose.covariance[i*6+i]=1e6
        if self.last_accepted is not None:
            previous_stamp,previous_xyz=self.last_accepted;dt=result['stamp']-previous_stamp
            if 0<dt<=MAX_OBSERVATION_AGE+1e-9:
                world_velocity=(xyz-previous_xyz)/dt
                body_velocity=Rotation.from_rotvec([0.,0.,-heading]).apply(world_velocity)
                msg.twist.twist.linear.x=float(body_velocity[0]);msg.twist.twist.linear.y=float(body_velocity[1]);msg.twist.twist.linear.z=float(body_velocity[2])
        for i in range(6):msg.twist.covariance[i*6+i]=1e6
        msg.twist.twist.angular.z=float(wz)
        self.last_accepted=(result['stamp'],xyz.copy())
        self.publisher.publish(msg);self.filtered_publisher.publish(msg)
        transform=TransformStamped();transform.header=msg.header;transform.child_frame_id=msg.child_frame_id
        transform.transform.translation.x=float(xyz[0]);transform.transform.translation.y=float(xyz[1]);transform.transform.translation.z=float(xyz[2])
        transform.transform.rotation=msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform);self.counts['published']+=1

    def report(self):
        payload=self.quality.snapshot(self.get_clock().now().nanoseconds/1e9)
        if payload['reason']=='observation_stale':self.pending.clear()
        payload.update(localization_backend=self.backend_name,counts=dict(self.counts),
            last_fit=self.last_fit,reference_frames=len(self.core.references),
            initialized=bool(self.core.references),max_compute_s=self.max_compute_s,
            tf_published=bool(self.counts['published']),truth_inputs=False,imu_heading_reference='startup',imu_heading_offset=self.heading_offset,
            input_contract=self.input_contract)
        payload.update(self.runtime_diagnostics())
        self.status.publish(String(data=json.dumps(payload,allow_nan=False)))

    def runtime_diagnostics(self):
        return {}


def main():
    rclpy.init();node=TerrainOdometryNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
