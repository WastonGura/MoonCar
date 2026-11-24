"""Sensor-only planar scan odometry with an IMU heading prior.

Robust point-to-line translation fitting, bounded submap and stop refinement.
No map pose, commands, Gazebo truth, or soil/contact parameters are inputs.
"""
from collections import deque
import math
import numpy as np
from scipy.spatial import cKDTree


def rotation(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s],[s,c]])


def wrap(a):return math.atan2(math.sin(a),math.cos(a))


class LaserOdometry:
    def __init__(self,laser_xy=(.14,0.),laser_yaw=0.):
        self.laser_xy=np.asarray(laser_xy,dtype=float);self.laser_yaw=float(laser_yaw)
        self.xy=np.zeros(2);self.yaw=None;self.stamp=None;self.last_motion=None
        self.bootstrap=[];self.frames=deque(maxlen=20);self.target=None;self.tree=None
        self.last_reference=None;self.last_fit=None
        self.bootstrap_moved=False
        self.last_observation_stamp=None

    def points(self,ranges,angles,range_min,range_max):
        valid=np.isfinite(ranges)&(ranges>range_min)&(ranges<range_max-.01)
        r=np.asarray(ranges)[valid];a=np.asarray(angles)[valid]+self.laser_yaw
        return np.c_[r*np.cos(a),r*np.sin(a)]+self.laser_xy

    def reference(self,world):
        self.frames.append(world)
        points=np.concatenate(self.frames)
        _,indices=np.unique(np.floor(points/.01).astype(np.int64),axis=0,return_index=True)
        self.target=points[indices];self.tree=cKDTree(self.target)
        self.last_reference=(self.xy.copy(),self.yaw)

    def fit(self,world,prediction):
        translation=prediction.copy();information=None;residual=None;used=0
        for _ in range(12):
            moved=world+translation
            distances,indices=self.tree.query(moved,k=min(8,len(self.target)))
            if distances.ndim!=2 or distances.shape[1]<5:return None,'too_few_reference_points'
            neighborhoods=self.target[indices]
            mean=neighborhoods.mean(axis=1);centered=neighborhoods-mean[:,None,:]
            cov=np.einsum('nki,nkj->nij',centered,centered)/neighborhoods.shape[1]
            eigenvalues,eigenvectors=np.linalg.eigh(cov);normals=eigenvectors[:,:,0]
            # Reject poor correspondences and neighborhoods without a local line.
            valid=(distances[:,0]<.15)&(distances[:,-1]<.25)&(eigenvalues[:,1]>1e-5)&(eigenvalues[:,0]<.2*eigenvalues[:,1])
            n=normals[valid];r=np.sum(n*(moved[valid]-mean[valid]),axis=1)
            used=len(r)
            # Two translation unknowns need independent normal directions. Six
            # constraints give 3x redundancy; rank/conditioning is checked below.
            if used<6:return None,'too_few_line_correspondences'
            weights=np.minimum(1.,.02/np.maximum(abs(r),1e-9))
            information=n.T@(weights[:,None]*n)
            ev=np.linalg.eigvalsh(information)
            if ev[0]<.02*ev[-1]:return None,'unobservable_translation'
            step=-np.linalg.solve(information,n.T@(weights*r))
            translation+=step;residual=r
            if np.linalg.norm(translation-prediction)>.1:return None,'correction_jump'
            if np.linalg.norm(step)<1e-5:break
        rmse=float(np.sqrt(np.mean(residual**2)))
        if rmse>.03:return None,'registration_residual'
        covariance=max(rmse**2,.01**2)*np.linalg.inv(information)
        covariance+=np.eye(2)*.02**2  # Conservative floor for correlated scan errors.
        return dict(xy=translation,covariance=covariance,rmse_m=rmse,inliers=used),'accepted'

    def update(self,stamp,ranges,angles,range_min,range_max,yaw,wheel_vx,wheel_wz):
        if not np.isfinite([stamp,yaw,wheel_vx,wheel_wz]).all():return dict(accepted=False,reason='nonfinite_input')
        if self.stamp is not None and stamp<=self.stamp:return dict(accepted=False,reason='nonincreasing_scan_stamp')
        points=self.points(ranges,angles,range_min,range_max)
        if len(points)<12:return dict(accepted=False,reason='too_few_returns')
        moving=abs(wheel_vx)>.005 or abs(wheel_wz)>.03
        if self.tree is None and moving:self.bootstrap_moved=True
        if self.bootstrap_moved:return dict(accepted=False,reason='initialization_motion_requires_new_session')
        if moving:self.last_motion=stamp
        if self.yaw is None:self.yaw=yaw
        dt=0 if self.stamp is None else stamp-self.stamp
        if dt>1.:return dict(accepted=False,reason='scan_gap_requires_reset')
        prediction=self.xy+rotation(self.yaw)@np.array([wheel_vx*dt,0.])
        self.stamp=stamp;self.yaw=yaw
        world=points@rotation(yaw).T
        if self.tree is None:
            self.bootstrap.append(world)
            if len(self.bootstrap)<10:return dict(accepted=False,reason='bootstrap')
            self.reference(np.concatenate(self.bootstrap));self.bootstrap=[]
            self.last_fit=dict(xy=self.xy.copy(),covariance=np.eye(2)*.02**2,rmse_m=0.,inliers=len(points))
            self.last_observation_stamp=stamp
            return dict(accepted=True,reason='initialized',stamp=stamp,yaw=yaw,**self.last_fit)
        # Fast correction continues for 1 s after motion. Afterwards, maintain a
        # genuine fresh position observation at <=5 Hz (the unchanged EKF's
        # 0.2 s sensor timeout): pose-only fusion must not
        # be left integrating an inferred lateral velocity without observations.
        # Stationary maintenance never inserts submap keyframes.
        if self.last_motion is None or stamp-self.last_motion>1.:
            if stamp-self.last_observation_stamp<.2-1e-9:
                return dict(accepted=False,reason='stationary_hold')
        fitted,reason=self.fit(world,prediction)
        if fitted is None:
            # Carry prediction to the next scan, but never publish wheel-only
            # prediction as a successful laser observation.
            self.xy=prediction
            return dict(accepted=False,reason=reason)
        old=self.xy.copy();self.xy=fitted['xy'];self.last_fit=fitted
        self.last_observation_stamp=stamp
        reference_xy,reference_yaw=self.last_reference
        if moving and (np.linalg.norm(self.xy-reference_xy)>=.2 or abs(wrap(yaw-reference_yaw))>=.2):
            self.reference(world+self.xy)
        vxvy=rotation(yaw).T@(self.xy-old)/max(dt,1e-9)
        return dict(accepted=True,reason=reason,stamp=stamp,yaw=yaw,body_velocity=vxvy,**fitted)
