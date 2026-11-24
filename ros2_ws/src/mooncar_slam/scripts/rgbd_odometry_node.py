#!/usr/bin/env python3
"""Registered RGB plus metric depth, with timestamped gimbal and IMU attitude.

Input contract is rectified RGB/depth with identical calibration, optical frame
and exposure stamp. Raw cameras with different intrinsics/extrinsics must be
registered before entering this node. Invalid depth remains unknown.
Sensor callbacks capture immutable exposures without waiting for registration.
One worker keeps only the newest ready exposure; omitted raw frames never renew
the accepted observation or localization health.
"""

def _apply_startup_affinity():
    """Explicit experiment opt-in, before ROS/OpenCV can create threads."""
    import os
    requested = os.environ.get('MOONCAR_RGBD_CPUSET')
    if requested is None:
        return
    parts = requested.split(',')
    if not parts or any(not p or not p.isascii() or not p.isdecimal() for p in parts):
        raise ValueError('MOONCAR_RGBD_CPUSET must be distinct comma-separated CPU numbers')
    cpus = {int(p) for p in parts}
    if len(cpus) != len(parts) or not cpus.issubset(os.sched_getaffinity(0)):
        raise ValueError('MOONCAR_RGBD_CPUSET is duplicate or outside the inherited CPU set')
    os.sched_setaffinity(0, cpus)
    if os.sched_getaffinity(0) != cpus:
        raise RuntimeError('MOONCAR_RGBD_CPUSET startup affinity was not applied')


if __name__ == '__main__':
    _apply_startup_affinity()

from collections import OrderedDict
from dataclasses import dataclass
import threading
import time
from time import thread_time

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image,CameraInfo
from tf2_ros import TransformException

from terrain_odometry_node import TerrainOdometryNode,outside_rover,imu_at
from laser_odometry_node import interpolate,seconds
from localization_quality import MAX_OBSERVATION_AGE,MAX_STATUS_WALL_AGE,FUTURE_TOLERANCE


@dataclass(frozen=True)
class ExposureImage:
    width: int
    height: int
    step: int
    encoding: str
    is_bigendian: int
    data: bytes

    @classmethod
    def capture(cls,msg):
        return cls(msg.width,msg.height,msg.step,msg.encoding,msg.is_bigendian,bytes(msg.data))


@dataclass(frozen=True)
class Exposure:
    stamp: float
    sec: int
    nanosec: int
    frame_id: str
    depth: ExposureImage
    color: ExposureImage
    received_wall: float
    intrinsics: tuple
    camera_rotation: tuple
    camera_translation: tuple
    body_rotation: tuple
    heading: float
    wz: float
    wheel: tuple | None
    generation: int
    diagnostic_prepared_sim: float | None = None
    diagnostic_prepared_wall: float | None = None


def gray_image(msg):
    channels={'rgb8':3,'bgr8':3,'mono8':1}.get(msg.encoding)
    if (channels is None or min(msg.width,msg.height)<=0 or msg.step<msg.width*channels
            or len(msg.data)<msg.step*msg.height):
        raise ValueError('Invalid RGB image encoding, shape or buffer')
    shape=(msg.height,msg.width) if channels==1 else (msg.height,msg.width,3)
    strides=(msg.step,1) if channels==1 else (msg.step,3,1)
    pixels=np.ndarray(shape,dtype=np.uint8,buffer=msg.data,strides=strides)
    if channels==1:return np.ascontiguousarray(pixels)
    return cv2.cvtColor(pixels,cv2.COLOR_RGB2GRAY if msg.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)


def depth_grid(msg,intrinsics):
    if msg.encoding not in ('32FC1','16UC1') or min(msg.width,msg.height)<=0:
        raise ValueError('Invalid aligned depth image')
    dtype=np.dtype(('>' if msg.is_bigendian else '<')+('f4' if msg.encoding=='32FC1' else 'u2'))
    if msg.step<msg.width*dtype.itemsize or len(msg.data)<msg.step*msg.height:
        raise ValueError('Truncated aligned depth image')
    z=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                 strides=(msg.step,dtype.itemsize)).astype(float)
    if msg.encoding=='16UC1':z*=.001
    valid=np.isfinite(z)&(z>=.1)&(z<=6.)
    z[~valid]=np.nan
    fx,fy,cx,cy=intrinsics
    if not np.isfinite(intrinsics).all() or min(fx,fy)<=0:raise ValueError('Invalid calibration')
    v,u=np.mgrid[:msg.height,:msg.width]
    points=np.stack(((u+.5-cx)*z/fx,(v+.5-cy)*z/fy,z),axis=2)
    points[~valid]=np.nan
    return points


def calibration_signature(msg):
    """A rectified, uncropped pixel grid; both streams must describe it fully."""
    fields=[*msg.k,*msg.d,*msg.r,*msg.p]
    if (not np.isfinite(fields).all() or min(msg.width,msg.height)<=0 or
            min(msg.k[0],msg.k[4])<=0 or any(abs(value)>1e-9 for value in msg.d)
            or abs(msg.k[1])+abs(msg.k[3])+abs(msg.k[6])+abs(msg.k[7])>1e-9 or abs(msg.k[8]-1)>1e-9
            or msg.binning_x not in (0,1) or msg.binning_y not in (0,1)
            or any((msg.roi.x_offset,msg.roi.y_offset,msg.roi.width,msg.roi.height,msg.roi.do_rectify))):
        raise ValueError('RGBD input must use registered rectified pixel calibration')
    # Gazebo/ideal producers can omit R/P and use the declared rectified K.
    # Populated matrices must describe exactly that pixel grid and optical axis.
    expected_p=np.column_stack((np.asarray(msg.k).reshape(3,3),np.zeros(3))).ravel()
    if ((np.any(msg.r) and not np.allclose(msg.r,np.eye(3).ravel(),atol=1e-9,rtol=0)) or
            (np.any(msg.p) and not np.allclose(msg.p,expected_p,atol=1e-9,rtol=0))):
        raise ValueError('Rectified R/P must agree with the projected K and optical frame')
    return (msg.header.frame_id,msg.width,msg.height,tuple(msg.k),tuple(msg.d),tuple(msg.r),tuple(msg.p),
            msg.distortion_model,msg.binning_x,msg.binning_y,
            msg.roi.x_offset,msg.roi.y_offset,msg.roi.width,msg.roi.height,msg.roi.do_rectify)


class RGBDOdometryNode(TerrainOdometryNode):
    node_name='mooncar_rgbd_odometry'
    backend_name='metric_rgbd_3d'
    input_contract='Exact-stamp rectified RGB + aligned metric depth + identical CameraInfo + timestamped gimbal TF + full IMU; wheel search only, optional after initialization; no map/world/truth input'

    def create_core(self):
        from metric_rgbd_odometry import MetricRGBDOdometry
        return MetricRGBDOdometry()

    def image_qos(self):
        if not hasattr(self,'rgbd_image_qos'):
            policy=self.declare_parameter('rgbd_image_reliability','best_effort',
                ParameterDescriptor(read_only=True,description='Match the RGBD publisher transport; requires node restart to change.')).value
            if policy not in ('best_effort','reliable'):
                raise ValueError('rgbd_image_reliability must be best_effort or reliable')
            self.rgbd_image_reliability=policy
            self.rgbd_image_qos=QoSProfile(depth=2 if policy=='reliable' else qos_profile_sensor_data.depth,
                reliability=(ReliabilityPolicy.RELIABLE
                if policy=='reliable' else ReliabilityPolicy.BEST_EFFORT))
        return self.rgbd_image_qos

    def __init__(self,asynchronous_processing=True):
        cv2.setNumThreads(2)
        self.processing_lock=threading.RLock()
        self.processing_condition=threading.Condition(self.processing_lock)
        self.processing_async=asynchronous_processing
        self.processing_closing=False;self.processing_generation=0
        self.processing_latest=None;self.processing_inflight=False;self.processing_thread=None
        self.pending_wait_reason='no_pending_exposure'
        self.last_per_exposure_timing=None
        self.colors=OrderedDict();self.color_info=None;self.depth_info=None;self.last_color_stamp=None
        super().__init__()
        self.create_subscription(Image,'/mooncar/localization_rgb/image',self.on_color,self.image_qos())
        self.create_subscription(CameraInfo,'/mooncar/localization_rgb/camera_info',self.on_color_info,self.image_qos())
        if self.processing_async:
            self.processing_thread=threading.Thread(target=self._processing_loop,
                name='mooncar-rgbd-registration',daemon=True)
            self.processing_thread.start()

    def on_depth(self,msg):
        with self.processing_lock:super().on_depth(msg)

    def on_imu(self,msg):
        with self.processing_lock:super().on_imu(msg)

    def on_wheel(self,msg):
        with self.processing_lock:super().on_wheel(msg)

    def reject(self,reason,stamp=None):
        with self.processing_lock:
            self.processing_generation+=1;self.processing_latest=None
            super().reject(reason,stamp)

    def report(self):
        with self.processing_lock:
            before=self.quality.fault
            super().report()
            if self.quality.fault and self.quality.fault!=before:
                self.processing_generation+=1;self.processing_latest=None

    def runtime_diagnostics(self):
        return dict(last_per_exposure_timing=(dict(self.last_per_exposure_timing)
                    if self.last_per_exposure_timing is not None else None),
            rgbd_processing_async=self.processing_async,
            rgbd_image_reliability=self.rgbd_image_reliability,
            rgbd_image_queue_depth=self.rgbd_image_qos.depth,
            rgbd_processing_inflight=self.processing_inflight,
            rgbd_pending_exposure_stamp=self.processing_latest.stamp if self.processing_latest else None,
            rgbd_wait_reason=self.pending_wait_reason,
            rgbd_last_depth_stamp=self.last_depth_stamp,rgbd_last_color_stamp=self.last_color_stamp,
            rgbd_last_imu_stamp=self.imu[-1][0] if self.imu else None,
            rgbd_last_wheel_stamp=self.wheel[-1][0] if self.wheel else None,
            rgbd_oldest_pending_stamp=seconds(self.pending[0][0].header.stamp) if self.pending else None)

    def destroy_node(self):
        with self.processing_condition:
            self.processing_closing=True;self.processing_generation+=1
            self.processing_latest=None;self.processing_condition.notify_all()
        if self.processing_thread is not None:
            self.processing_thread.join(timeout=5.)
            if self.processing_thread.is_alive():
                raise RuntimeError('RGBD registration did not stop before node shutdown')
        return super().destroy_node()

    def on_info(self,msg):
        with self.processing_lock:self._on_info(msg)

    def _on_info(self,msg):
        try:signature=calibration_signature(msg)
        except ValueError:
            self.quality.fault='invalid_rectified_depth_calibration';self.reject(self.quality.fault);return
        if self.depth_info is not None and signature!=self.depth_info:
            self.quality.fault='camera_calibration_changed';self.reject(self.quality.fault);return
        self.depth_info=signature
        if self.color_info is not None and self.color_info!=signature:
            self.quality.fault='rgb_depth_calibration_mismatch';self.reject(self.quality.fault);return
        super().on_info(msg)

    def on_color_info(self,msg):
        with self.processing_lock:self._on_color_info(msg)

    def _on_color_info(self,msg):
        try:signature=calibration_signature(msg)
        except ValueError:
            self.quality.fault='invalid_rectified_rgb_calibration';self.reject(self.quality.fault);return
        if self.color_info is not None and signature!=self.color_info:
            self.quality.fault='camera_calibration_changed';self.reject('camera_calibration_changed');return
        self.color_info=signature
        if self.depth_info is not None and self.depth_info!=signature:
            self.quality.fault='rgb_depth_calibration_mismatch';self.reject(self.quality.fault);return
        self.process_pending()

    def on_color(self,msg):
        with self.processing_lock:self._on_color(msg)

    def _on_color(self,msg):
        stamp=seconds(msg.header.stamp)
        if not np.isfinite(stamp) or (self.last_color_stamp is not None and stamp<=self.last_color_stamp):
            self.reject('nonincreasing_rgb_stamp');return
        self.last_color_stamp=stamp
        self.colors[stamp]=(msg,time.monotonic())
        while len(self.colors)>10:self.colors.popitem(last=False)
        self.process_pending()

    def process_pending(self):
        with self.processing_lock:self._prepare_pending()

    def _prepare_pending(self):
        if self.processing_closing:return
        self.pending_wait_reason='no_pending_exposure' if not self.pending else 'waiting_calibration_or_inertial'
        if self.quality.fault:
            self.pending.clear();return
        while self.pending and self.intrinsics is not None and self.color_info is not None and self.imu:
            msg,received=self.pending[0];stamp=seconds(msg.header.stamp)
            pair=self.colors.get(stamp)
            if pair is None and self.last_color_stamp is not None and self.last_color_stamp>stamp:
                # An unmatched raw message never renews the accepted observation,
                # even when this obsolete raw message is already old.
                self.pending.popleft();self.counts['unpaired_depth_omitted']+=1;continue
            now=self.get_clock().now().nanoseconds/1e9
            if now-stamp>MAX_OBSERVATION_AGE+1e-9:
                self.reject('depth_observation_stale',stamp);return
            if stamp>now+FUTURE_TOLERANCE+1e-9:
                self.reject('depth_observation_from_future',stamp);return
            if self.color_info!=self.depth_info:
                self.quality.fault='rgb_depth_calibration_mismatch';self.reject('rgb_depth_calibration_mismatch');return
            if pair is None:
                self.pending_wait_reason='waiting_matching_rgb';return
            color,color_received=pair
            if self.imu[-1][0]<stamp:
                self.pending_wait_reason='waiting_imu_bracket';return
            attitude=imu_at(self.imu,stamp)
            wheel=interpolate(self.wheel,stamp,.08) if self.wheel else None
            if wheel is None and not self.core.references:
                self.pending_wait_reason='waiting_initial_wheel_bracket';return
            if attitude is None:
                self.reject('unsynchronized',stamp);return
            if (msg.header.frame_id!=self.camera_frame or color.header.frame_id!=self.camera_frame
                    or (msg.width,msg.height)!=self.camera_size or (color.width,color.height)!=self.camera_size):
                self.reject('rgb_depth_frame_mismatch',stamp);return
            try:
                tf=self.buffer.lookup_transform('base_footprint',self.camera_frame,Time.from_msg(msg.header.stamp))
            except TransformException:
                self.pending_wait_reason='waiting_exposure_camera_tf'
                if time.monotonic()-received>.5:self.reject('camera_transform_unavailable',stamp)
                return
            self.pending.popleft();self.colors.pop(stamp)
            orientation,wz=attitude
            matrix=orientation.as_matrix();heading=float(np.arctan2(matrix[1,0],matrix[0,0]))
            q=tf.transform.rotation;p=tf.transform.translation
            exposure=Exposure(stamp,msg.header.stamp.sec,msg.header.stamp.nanosec,msg.header.frame_id,
                ExposureImage.capture(msg),ExposureImage.capture(color),min(received,color_received),
                tuple(self.intrinsics),(q.x,q.y,q.z,q.w),(p.x,p.y,p.z),tuple(orientation.as_quat()),
                heading,float(wz),tuple(wheel) if wheel is not None else None,self.processing_generation,
                diagnostic_prepared_sim=now,diagnostic_prepared_wall=time.monotonic())
            if self.processing_async:
                if self.processing_latest is not None:self.counts['rgbd_queued_exposures_omitted']+=1
                self.processing_latest=exposure;self.processing_condition.notify()
            else:self._process_exposure(exposure)
        if not self.pending:self.pending_wait_reason='no_pending_exposure'

    def _processing_loop(self):
        while True:
            with self.processing_condition:
                self.processing_condition.wait_for(lambda:self.processing_closing or self.processing_latest is not None)
                if self.processing_closing:return
                exposure=self.processing_latest;self.processing_latest=None
                self.processing_inflight=True
            try:self._process_exposure(exposure)
            except Exception as error:
                with self.processing_lock:
                    self.quality.fault=self.quality.fault or 'rgbd_processing_failed'
                    self.reject(self.quality.fault)
                self.get_logger().error('RGBD registration worker failed: '+repr(error))
            finally:
                with self.processing_lock:self.processing_inflight=False

    def _process_exposure(self,exposure):
        # One bounded numeric record, never an input to quality or control.
        # Prepared sim reuses the original input-age clock read. Thread CPU
        # covers decode+core only; wall-minus-CPU is not a CPU-throttling proof.
        _timing=dict(exposure_stamp=exposure.stamp,
            prepared_sim_age_s=(None if exposure.diagnostic_prepared_sim is None else
                                exposure.diagnostic_prepared_sim-exposure.stamp),
            start_sim=None,start_sim_age_s=None,input_wall_age_start_s=None,
            ready_wait_wall_s=None,old_accepted_stamp_start=None,old_accepted_age_start_s=None,
            decode_wall_s=None,core_wall_s=None,worker_thread_cpu_s=None,
            end_lock_wait_wall_s=None,post_core_to_lock_wall_s=None,decode_core_wall_s=None,
            after_sim=None,old_accepted_stamp_before_snapshot=None,
            old_accepted_age_before_snapshot_s=None,new_exposure_age_after_s=None,
            stage='entered')
        with self.processing_lock:
            self.last_per_exposure_timing=_timing
            if (self.processing_closing or self.quality.fault or
                    exposure.generation!=self.processing_generation):return
            now=self.get_clock().now().nanoseconds/1e9
            _timing_wall_start=time.monotonic()
            _timing.update(start_sim=now,start_sim_age_s=now-exposure.stamp,
                input_wall_age_start_s=_timing_wall_start-exposure.received_wall,
                ready_wait_wall_s=(None if exposure.diagnostic_prepared_wall is None else
                                   _timing_wall_start-exposure.diagnostic_prepared_wall),
                old_accepted_stamp_start=self.quality.accepted_stamp,
                old_accepted_age_start_s=(None if self.quality.accepted_stamp is None else
                                         now-self.quality.accepted_stamp),stage='before_compute')
            self.quality.snapshot(now)
            if self.quality.fault:
                self.processing_generation+=1;self.processing_latest=None;return
            if exposure.stamp>now+FUTURE_TOLERANCE+1e-9:
                self.reject('depth_observation_from_future',exposure.stamp);return
            if (now-exposure.stamp>MAX_OBSERVATION_AGE+1e-9 or
                    time.monotonic()-exposure.received_wall>MAX_STATUS_WALL_AGE):
                self.reject('depth_observation_stale_before_compute',exposure.stamp);return
            self.counts['rgbd_processing_started']+=1
            if exposure.wheel is None:self.counts['rgbd_processing_without_wheel_prediction']+=1
        begin=time.perf_counter()
        _timing_cpu_begin=thread_time()
        try:
            gray=gray_image(exposure.color);optical=depth_grid(exposure.depth,exposure.intrinsics)
            valid=np.isfinite(optical).all(axis=2);indices=np.flatnonzero(valid.ravel())
            extrinsic=Rotation.from_quat(exposure.camera_rotation);lever=np.array(exposure.camera_translation)
            if not np.isfinite(lever).all():raise ValueError('Invalid camera pose')
            body=extrinsic.apply(optical.reshape(-1,3)[indices])+lever
            external=outside_rover(body)
            omitted=int((~external).sum())
            measured=np.full(optical.shape,np.nan,dtype=float)
            orientation=Rotation.from_quat(exposure.body_rotation)
            measured.reshape(-1,3)[indices[external]]=orientation.apply(body[external])
            fx,fy,cx,cy=exposure.intrinsics
            camera_model=dict(K=np.array([[fx,0.,cx],[0.,fy,cy],[0.,0.,1.]]),
                rotation=(orientation*extrinsic).as_matrix(),
                translation=orientation.apply(lever))
            hint=(None,None) if exposure.wheel is None else tuple(map(float,exposure.wheel[:2]))
            _timing_decode_end=time.perf_counter()
            result=self.core.update(exposure.stamp,gray,measured,exposure.heading,*hint,
                camera_model=camera_model)
            _timing_core_end=time.perf_counter()
            _timing_cpu_end=thread_time()
        except (ValueError,TypeError,FloatingPointError,cv2.error):
            with self.processing_lock:self.reject('invalid_rgbd_input',exposure.stamp)
            return
        _timing_lock_begin=time.perf_counter()
        with self.processing_lock:
            _timing_locked=time.perf_counter()
            _timing.update(decode_wall_s=_timing_decode_end-begin,
                core_wall_s=_timing_core_end-_timing_decode_end,
                worker_thread_cpu_s=_timing_cpu_end-_timing_cpu_begin,
                end_lock_wait_wall_s=_timing_locked-_timing_lock_begin,
                post_core_to_lock_wall_s=_timing_locked-_timing_core_end,
                decode_core_wall_s=_timing_core_end-begin,stage='computed')
            self.max_compute_s=max(self.max_compute_s,time.perf_counter()-begin)
            self.counts['rgbd_processing_completed']+=1
            if (self.processing_closing or self.quality.fault or
                    exposure.generation!=self.processing_generation):
                self.counts['rgbd_results_invalidated']+=1;return
            after=self.get_clock().now().nanoseconds/1e9
            _timing.update(after_sim=after,
                old_accepted_stamp_before_snapshot=self.quality.accepted_stamp,
                old_accepted_age_before_snapshot_s=(None if self.quality.accepted_stamp is None else
                                                   after-self.quality.accepted_stamp),
                new_exposure_age_after_s=after-exposure.stamp)
            self.quality.snapshot(after)
            if self.quality.fault:
                self.processing_generation+=1;self.processing_latest=None
                self.counts['rgbd_results_invalidated']+=1;return
            if exposure.stamp>after+FUTURE_TOLERANCE+1e-9:
                self.reject('depth_observation_from_future_after_compute',exposure.stamp);return
            if (after-exposure.stamp>MAX_OBSERVATION_AGE+1e-9 or
                    time.monotonic()-exposure.received_wall>MAX_STATUS_WALL_AGE):
                self.reject('depth_observation_stale_after_compute',exposure.stamp);return
            self.counts['self_depth_returns_omitted']+=omitted
            self.counts[result['reason']]+=1
            self.last_fit={name:result[name] for name in ('accepted','reason','stamp','inliers','rmse_m',
                'visual_matches','identity_matches','reference_id','reference_frames','overlap','reference_support',
                'wheel_prediction_used','reference_attempts','view_admission') if name in result}
            self.last_fit['exposure_stamp']=exposure.stamp
            if result.get('accepted') and 'covariance' in result:
                self.last_fit['position_std_m']=np.sqrt(np.diag(result['covariance'])).tolist()
            self.quality.observe(exposure.stamp,result)
            _timing['stage']='result_observed'
            if result['accepted']:
                image=Image();image.header.frame_id=exposure.frame_id
                image.header.stamp.sec=exposure.sec;image.header.stamp.nanosec=exposure.nanosec
                self.publish(image,result,exposure.heading,exposure.wz)
            else:self.pending.clear()
            self.report()


def main():
    rclpy.init();node=RGBDOdometryNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
