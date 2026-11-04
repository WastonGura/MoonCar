"""Opt-in fixture adapter retaining leases, localization, map and region guards."""
import json
import math
import secrets
import time
import uuid

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from experiment_motion import ExperimentMotion
from localization_quality import require_dispatch_ready
from feedback_turn import FeedbackTurn, swept_clear


def install(output):
    original_init=ExperimentMotion.__init__
    original_turn=ExperimentMotion.turn
    original_tick=ExperimentMotion.tick
    original_cancel=ExperimentMotion.cancel
    log=(output/'feedback-turn.jsonl').open('x',buffering=1)
    def emit(row):log.write(json.dumps(row,allow_nan=False)+'\n')
    def initialize(self,*args,**kwargs):
        self.feedback_turn_controller=None
        original_init(self,*args,**kwargs)
        self.feedback_turn_pub=self.create_publisher(Twist,'/cmd_vel',10)
    def terminate(self,status,reason):
        self.feedback_turn_pub.publish(Twist())
        self.active=False;self.phase=status;self.error='' if status=='succeeded' else reason
        self.permit.publish(Bool(data=False))
        if status!='succeeded':self.record_stop(reason)
        emit(dict(event='terminal',request_id=self.goal_request_id,status=status,reason=reason,
                  sim=self.get_clock().now().nanoseconds/1e9))
        self.feedback_turn_controller=None
    def turn(self,yaw,task):
        if not getattr(self,'feedback_turn_enabled',False):return original_turn(self,yaw,task)
        if type(yaw) not in (int,float) or not math.isfinite(yaw) or abs(yaw)>math.pi:
            raise ValueError('Invalid turn yaw')
        with self.lock:
            if self.active:raise ValueError('Previous motion is still active')
            require_dispatch_ready(self.state(),'等待导航、定位与传感器就绪')
            p=self.pose();self.check_start(task,p)
            if not self.grid_fresh():raise ValueError('Fresh observed map is required')
            clear,reason=swept_clear(self.grid,task['region'],p,0.,0.)
            if not clear:raise ValueError(reason)
            self.region=task['region'];self.token=secrets.token_urlsafe(24)
            self.activate_localization_action();self.phase='executing';self.error=''
            self.active=True;self.goal_handle=None;self.goal_request_future=None
            self.stop_diagnostics=None;self.started=self.last_heartbeat=time.monotonic()
            self.sim_started=self.get_clock().now().nanoseconds/1e9;self.sim_budget=40.
            self.goal_request_kind='feedback_turn';self.goal_request_id=str(uuid.uuid4())
            self.goal_dispatch=dict(request_id=self.goal_request_id,kind='feedback_turn',validation_only=True)
            self.goal=[p['x'],p['y']];self.path=[];self.track=[[p['x'],p['y'],p['z']]]
            self.feedback_turn_controller=FeedbackTurn(p,yaw,self.sim_started)
            emit(dict(event='start',request_id=self.goal_request_id,sim=self.sim_started,
                      pose=p,target_yaw=yaw,truth_inputs=False))
            return dict(token=self.token)
    def tick(self):
        original_tick(self)
        with self.lock:
            controller=getattr(self,'feedback_turn_controller',None)
            if controller is None:return
            if not self.active or self.phase!='executing':
                terminate(self,'cancelled',self.error or 'motion_authority_revoked');return
            try:
                row=controller.step(self.pose(),self.get_clock().now().nanoseconds/1e9)
                if row is None:return
                row.update(event='cycle',request_id=self.goal_request_id)
                if row['status']!='running':
                    emit(row);terminate(self,row['status'],row['reason']);return
                if not self.grid_fresh():raise ValueError('costmap_stale')
                clear,reason=swept_clear(self.grid,self.region,row['pose'],row['linear'],row['angular'])
                row['swept_check']=reason;emit(row)
                if not clear:raise ValueError(reason)
                command=Twist();command.linear.x=row['linear'];command.angular.z=row['angular']
                self.feedback=dict(validation_only=True,controller_phase=row['phase'],
                    distance_to_turn_anchor_m=row['distance_m'],yaw_error_rad=row['yaw_error_rad'])
                self.feedback_turn_pub.publish(command)
            except Exception as error:terminate(self,'failed',str(error))
    def cancel(self,reason=''):
        result=original_cancel(self,reason)
        with self.lock:
            if getattr(self,'feedback_turn_controller',None) is not None:
                terminate(self,'cancelled',reason or 'cancelled')
        return result
    ExperimentMotion.__init__=initialize
    ExperimentMotion.turn=turn
    ExperimentMotion.tick=tick
    ExperimentMotion.cancel=cancel
