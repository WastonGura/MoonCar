"""Bounded, validation-only pose feedback maneuver. No ROS or truth inputs.

This is an experiment, not a replacement for Nav2 Spin. At most one return-to-
anchor leg and one final turn are allowed; failure to retain position is explicit.
"""
import math


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def clip(value, limit):
    return max(-limit, min(limit, value))


class FeedbackTurn:
    def __init__(self, pose, yaw, sim):
        self.anchor = (pose['x'], pose['y'])
        self.target_yaw = yaw
        self.start_sim = self.previous_sim = sim
        self.phase = 'turn'
        self.previous_command = (0., 0.)
        self.settled_since = None
        self.correction_used = False

    def step(self, pose, sim):
        values = [pose[k] for k in ('x','y','yaw')]+[sim]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Nonfinite pose/clock')
        dt = sim-self.previous_sim
        if dt < -1e-9:
            raise ValueError('Clock moved backwards')
        if dt > .5:
            raise ValueError('Controller update gap exceeds 0.5 simulated seconds')
        if dt < .04:
            return None
        self.previous_sim = sim
        dx, dy = self.anchor[0]-pose['x'], self.anchor[1]-pose['y']
        distance = math.hypot(dx,dy)
        yaw_error = wrap(self.target_yaw-pose['yaw'])
        status = 'running'; reason = ''
        if sim-self.start_sim > 40:
            status,reason = 'failed','bounded_turn_timeout'
        elif distance > .6:
            status,reason = 'failed','maneuver_displacement_limit'
        forward = dx*math.cos(pose['yaw'])+dy*math.sin(pose['yaw'])
        v = w = 0.
        if status == 'running':
            if self.phase in ('turn','final_turn'):
                v = clip(.7*forward,.12)
                w = clip(1.0*yaw_error,.3)
                if abs(yaw_error) <= math.radians(3):
                    if distance <= .12:
                        self.phase = 'settle'
                    elif not self.correction_used:
                        self.phase = 'return_to_anchor'; self.correction_used = True
                    else:
                        status,reason = 'failed','position_not_retained_after_single_correction'
                    v=w=0.
            elif self.phase == 'return_to_anchor':
                if distance <= .08:
                    self.phase = 'final_turn'
                else:
                    bearing = math.atan2(dy,dx)
                    reverse = abs(wrap(bearing-pose['yaw'])) > math.pi/2
                    motion_yaw = wrap(pose['yaw']+(math.pi if reverse else 0.))
                    alpha = wrap(bearing-motion_yaw)
                    beta = wrap(self.target_yaw-motion_yaw-alpha)
                    v = (-1 if reverse else 1)*min(.12,.5*distance)*max(0.,math.cos(alpha))
                    w = clip(1.2*alpha-.25*beta,.3)
            else:
                if distance > .12 or abs(yaw_error) > math.radians(5):
                    status,reason = 'failed','position_or_heading_lost_while_settling'
                else:
                    if self.settled_since is None:self.settled_since=sim
                    if sim-self.settled_since >= .8:
                        status,reason = 'succeeded','pose_retained'
        if self.phase == 'settle' or status != 'running':
            v=w=0.
        else:
            v = self.previous_command[0]+clip(v-self.previous_command[0],.2*dt)
            w = self.previous_command[1]+clip(w-self.previous_command[1],.4*dt)
        self.previous_command = (v,w)
        return dict(sim=sim,phase=self.phase,status=status,reason=reason,
                    linear=v,angular=w,distance_m=distance,yaw_error_rad=yaw_error,
                    anchor=list(self.anchor),pose=dict(pose),correction_used=self.correction_used)


def swept_clear(grid, region, pose, linear, angular, horizon=2.):
    """Conservative padded chassis sweep in the observed costmap; unknown blocks.

    Includes each intersecting cell by expanding the rectangle by a cell diagonal.
    This is a local constant-command prediction, not a guarantee against slip.
    """
    import numpy as np
    info=grid.info; r=info.resolution; ox=info.origin.position.x; oy=info.origin.position.y
    costs=np.asarray(grid.data).reshape(info.height,info.width)
    x,y,yaw=pose['x'],pose['y'],pose['yaw']
    steps=max(1,math.ceil(horizon/.05));dt=horizon/steps
    # Existing boundary footprint plus 0.10m uncertainty buffer for this trial.
    hx,hy=.63,.56
    for step in range(steps+1):
        c,s=math.cos(yaw),math.sin(yaw)
        bx,by=hx*abs(c)+hy*abs(s),hx*abs(s)+hy*abs(c)
        if x-bx<region[0] or x+bx>region[2] or y-by<region[1] or y+by>region[3]:
            return False,'predicted_region_boundary'
        ix0=math.floor((x-bx- r)/r-ox/r);ix1=math.floor((x+bx+r-ox)/r)
        iy0=math.floor((y-by- r)/r-oy/r);iy1=math.floor((y+by+r-oy)/r)
        if ix0<0 or iy0<0 or ix1>=info.width or iy1>=info.height:
            return False,'predicted_outside_costmap'
        xx,yy=np.meshgrid(ox+(np.arange(ix0,ix1+1)+.5)*r,oy+(np.arange(iy0,iy1+1)+.5)*r)
        dx,dy=xx-x,yy-y
        mask=(abs(c*dx+s*dy)<=hx+r/math.sqrt(2)) & (abs(-s*dx+c*dy)<=hy+r/math.sqrt(2))
        values=costs[iy0:iy1+1,ix0:ix1+1][mask]
        if np.any(values<0):return False,'predicted_unknown_footprint'
        if np.any(values>=99):return False,'predicted_obstacle_footprint'
        if abs(angular)<1e-9:
            x+=linear*c*dt;y+=linear*s*dt
        else:
            nxt=yaw+angular*dt
            x+=linear/angular*(math.sin(nxt)-s)
            y+=linear/angular*(c-math.cos(nxt))
        yaw+=angular*dt
    return True,'observed_swept_footprint_clear'
