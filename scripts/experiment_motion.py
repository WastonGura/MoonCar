"""Leased Nav2 goal execution for the isolated simulation, with boundary stopping."""
import math
import secrets
import threading
import time
import base64
import json
from collections import deque
from dataclasses import dataclass
from localization_quality import LocalizationQuality, MAX_STATUS_WALL_AGE, LocalizationDispatchRejected, require_dispatch_ready
from nav_msgs.msg import Path, OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, Spin
from std_msgs.msg import Bool, Float64, String
from tf2_ros import Buffer, TransformListener
from unique_identifier_msgs.msg import UUID
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data


GOAL_RESPONSE_TIMEOUT_SECONDS = 5.
GOAL_RESPONSE_TIMEOUT_REASON = 'Nav2 目标接收超过 5 秒，已停止本次动作'
MAX_FUTURE_TF_SECONDS = .3
NAVIGATION_STALL_REASON = '导航持续未接近目标，已取消候选动作'
GIMBAL_TOLERANCE = math.radians(5)
GIMBAL_FEEDBACK_FUTURE_TOLERANCE = .05
NAV2_LIFECYCLE_NODES = ('bt_navigator', 'controller_server', 'planner_server', 'behavior_server')
NAV2_LIFECYCLE_ACTIVE_ID = 3
NAV2_LIFECYCLE_POLL_SECONDS = .5
NAV2_LIFECYCLE_TIMEOUT_SECONDS = 2.
NAV2_LIFECYCLE_MAX_AGE_SECONDS = 5.


@dataclass(frozen=True)
class GimbalJointCapture:
    stamp: float
    name: tuple
    position: tuple
    velocity: tuple
    received_wall: float


def within_region(region, x, y, yaw):
    hx=.43*abs(math.cos(yaw))+.36*abs(math.sin(yaw))+.1
    hy=.43*abs(math.sin(yaw))+.36*abs(math.cos(yaw))+.1
    return region[0]+hx <= x <= region[2]-hx and region[1]+hy <= y <= region[3]-hy


class ExperimentMotion(Node):
    def __init__(self, require_laser_localization=False, gimbal_command_rate_rad_s=None):
        if (gimbal_command_rate_rad_s is not None and
                (type(gimbal_command_rate_rad_s) not in (int,float) or
                 not math.isfinite(gimbal_command_rate_rad_s) or gimbal_command_rate_rad_s<=0)):
            raise ValueError('Gimbal command rate must be a positive finite value or None')
        super().__init__('mooncar_experiment_motion')
        self.gimbal_command_rate_rad_s=(float(gimbal_command_rate_rad_s)
                                       if gimbal_command_rate_rad_s is not None else None)
        self.gimbal_command=None;self.gimbal_command_stamp=None
        self.gimbal_final_command_stamp=None;self.gimbal_final_command_wall=None;self.gimbal_command_fault=None
        self.localization_quality=LocalizationQuality(required=require_laser_localization)
        self.localization_lock=threading.RLock()
        self.localization_fault_counter=0
        self.localization_action_fault_counter=0
        self.localization_was_healthy=False
        self.localization_generation=0
        self.localization_subscription=None
        self.localization_epoch_pending=False
        self.localization_epoch_candidate=None
        self.localization_previous_session=None
        # A separate callback group still shares the busy motion executor. Keep
        # /clock consumption on its own executor so TF freshness uses live time.
        self.clock_node=Node('mooncar_experiment_clock',context=self.context,
            start_parameter_services=False,parameter_overrides=[
                Parameter('use_sim_time',value=self.get_parameter('use_sim_time').value)])
        # Jazzy Node has no public TimeSource accessor. Transfer the existing ROS
        # clock to the helper's standard TimeSource, with exactly one writer.
        self._time_source.detach_node()
        self.clock_node._time_source.attach_clock(self.get_clock())
        self.clock_executor=SingleThreadedExecutor(context=self.context)
        self.clock_executor.add_node(self.clock_node)
        self.clock_thread=threading.Thread(target=self.clock_executor.spin,
            name='mooncar-experiment-clock',daemon=True)
        self.clock_thread.start()
        self.data_group=MutuallyExclusiveCallbackGroup()
        self.control_group=MutuallyExclusiveCallbackGroup()
        self.declare_parameter('gimbal_position_tolerance_rad',GIMBAL_TOLERANCE)
        self.declare_parameter('gimbal_velocity_tolerance_rad_s',0.)
        self.declare_parameter('gimbal_settle_duration_s',0.)
        self.declare_parameter('gimbal_action_timeout_s',0.)
        self.gimbal_config=(GIMBAL_TOLERANCE,0.,0.)
        self.gimbal_settled_since=None
        self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self)
        self.client=ActionClient(self,NavigateToPose,'/navigate_to_pose',callback_group=self.control_group)
        self.spin_client=ActionClient(self,Spin,'/spin',callback_group=self.control_group)
        self.nav2_lifecycle_lock=threading.RLock()
        self.nav2_lifecycle_states={}
        self.nav2_lifecycle_pending={}
        self.nav2_lifecycle_clients=self._create_nav2_lifecycle_clients()
        self.permit=self.create_publisher(Bool,'/mooncar/workbench/permit',1)
        self.pub_camera_pan=self.create_publisher(Float64,'/mooncar/cmd_camera_pan',10)
        self.pub_camera_tilt=self.create_publisher(Float64,'/mooncar/cmd_camera_tilt',10)
        self.gimbal_deadline=0.
        self.gimbal_target=None;self.gimbal_position=None
        self.gimbal_joint_received=0.;self.gimbal_joint_stamp=-1.;self.gimbal_settled_samples=0
        self.gimbal_capture_lock=threading.Lock();self.gimbal_captures=deque(maxlen=32)
        self.gimbal_capture_count=0;self.gimbal_capture_omitted=0;self.gimbal_capture_last_stamp=None
        self.gimbal_capture_dropped_old=0;self.gimbal_capture_dropped_future=0
        self.lock=threading.RLock()
        self.active=False;self.goal_handle=None;self.token='';self.phase='idle';self.error=''
        self.last_heartbeat=0.;self.started=0.;self.last_depth=0.;self.depth_message='等待传感器';self.region=None
        self.path=[];self.track=[];self.goal=None;self.grid=None;self.grid_view=None;self.grid_received=0.;self.last_path=0.
        self.keepout_grid=None
        self.feedback={};self.sim_started=0.;self.sim_budget=180.
        self.best_distance=math.inf;self.best_goal_distance=math.inf;self.last_progress_sim=0.
        self.goal_request_future=None;self.goal_request_id='';self.goal_request_kind='';self.goal_sent_wall=0.
        self.goal_dispatch=None
        self.start_checked=False
        self.quality=None;self.quality_received=0.;self.mission_limits=None
        self.stop_diagnostics=None
        if require_laser_localization:
            self.subscribe_localization_quality()
        self.create_subscription(String,'/mooncar/map/quality',self.on_quality,1,callback_group=self.data_group)
        if self.gimbal_command_rate_rad_s is not None:
            self.gimbal_joint_subscription=self.clock_node.create_subscription(JointState,'/joint_states',
                self.capture_gimbal_joints,qos_profile_sensor_data)
        else:
            self.gimbal_joint_subscription=self.create_subscription(JointState,'/joint_states',
                self.on_gimbal_joints,qos_profile_sensor_data,callback_group=self.data_group)
        self.create_subscription(DiagnosticArray,'/mooncar/terrain/status',self.depth,10,callback_group=self.data_group)
        self.create_subscription(Path,'/plan',self.on_path,10,callback_group=self.data_group)
        self.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.on_grid,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL),callback_group=self.data_group)
        self.create_subscription(OccupancyGrid,'/mooncar/terrain/keepout',self.on_keepout,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL),callback_group=self.data_group)
        self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME),callback_group=self.control_group)
        self.create_timer(NAV2_LIFECYCLE_POLL_SECONDS,self.poll_nav2_lifecycle,
                          clock=Clock(clock_type=ClockType.STEADY_TIME),callback_group=self.data_group)

    def _create_nav2_lifecycle_clients(self):
        return {name:self.create_client(GetState,f'/{name}/get_state',callback_group=self.data_group)
                for name in NAV2_LIFECYCLE_NODES}

    def _collect_nav2_lifecycle_locked(self,now):
        for name,(future,sent) in list(self.nav2_lifecycle_pending.items()):
            if future.done():
                try:
                    current=future.result().current_state
                    self.nav2_lifecycle_states[name]=dict(id=int(current.id),label=str(current.label),wall=now)
                except Exception as error:
                    self.nav2_lifecycle_states[name]=dict(error=repr(error),wall=now)
                del self.nav2_lifecycle_pending[name]
            elif now-sent>=NAV2_LIFECYCLE_TIMEOUT_SECONDS:
                client=self.nav2_lifecycle_clients.get(name)
                if client is not None:
                    try:client.remove_pending_request(future)
                    except Exception:pass
                future.cancel()
                self.nav2_lifecycle_states[name]=dict(error='get_state_timeout',wall=now)
                del self.nav2_lifecycle_pending[name]

    def poll_nav2_lifecycle(self,now=None):
        now=time.monotonic() if now is None else float(now)
        with self.nav2_lifecycle_lock:
            self._collect_nav2_lifecycle_locked(now)
            for name,client in self.nav2_lifecycle_clients.items():
                if name not in self.nav2_lifecycle_pending and client.service_is_ready():
                    self.nav2_lifecycle_pending[name]=(client.call_async(GetState.Request()),now)

    def nav2_lifecycle_snapshot(self,now=None):
        now=time.monotonic() if now is None else float(now)
        if not hasattr(self,'nav2_lifecycle_lock'):
            return dict(all_active=True,behavior_active=True,nodes={})
        with self.nav2_lifecycle_lock:
            self._collect_nav2_lifecycle_locked(now)
            nodes={};active_flags={}
            for name in NAV2_LIFECYCLE_NODES:
                entry=self.nav2_lifecycle_states.get(name)
                fresh=entry is not None and now-entry.get('wall',-math.inf)<=NAV2_LIFECYCLE_MAX_AGE_SECONDS
                is_active=bool(fresh and entry.get('id')==NAV2_LIFECYCLE_ACTIVE_ID)
                active_flags[name]=is_active
                if entry is None:nodes[name]='unknown'
                elif not fresh:nodes[name]='stale'
                else:nodes[name]=entry.get('label') or entry.get('error') or 'unknown'
            return dict(all_active=all(active_flags.values()),
                        behavior_active=active_flags.get('behavior_server',False),
                        nodes=nodes)

    def destroy_node(self):
        self.clock_executor.shutdown()
        self.clock_thread.join(timeout=3)
        self.clock_node.destroy_node()
        return super().destroy_node()

    def depth(self,msg):
        for status in msg.status:
            if status.name=='terrain_geometry':
                self.depth_message=status.message
                if status.level in (DiagnosticStatus.OK, DiagnosticStatus.WARN) or 'Too few depth returns' in status.message or 'No recent valid depth' in status.message:
                    self.last_depth=time.monotonic()

    def on_quality(self,msg):
        try:
            value=json.loads(msg.data)
            with self.lock:self.quality=value;self.quality_received=time.monotonic()
        except (ValueError,TypeError):pass

    def subscribe_localization_quality(self):
        with self.localization_lock:
            generation=self.localization_generation
            self.localization_subscription=self.clock_node.create_subscription(String,'/mooncar/laser_odometry/status',
                lambda msg:self.on_localization_quality(msg,generation),1)

    def on_localization_quality(self,msg,generation=None):
        try:value=json.loads(msg.data)
        except (ValueError,TypeError):value={}
        # This callback shares the isolated clock executor. It must never wait
        # for the motion lock or perform cancellation/diagnostic work.
        with self.localization_lock:
            if generation is not None and generation!=self.localization_generation:return
            if self.localization_epoch_pending and not self._accept_new_epoch_locked(value,time.monotonic()):return
            # Preserve a lost-heartbeat interval before replacing its receipt.
            # Age the incoming measurement after observe(), not the superseded
            # sample: normal scan/heartbeat phase jitter can straddle 0.3 s.
            previous_receipt=self.localization_quality.received_wall
            if previous_receipt is not None and time.monotonic()-previous_receipt>MAX_STATUS_WALL_AGE:
                self._localization_state_locked()
            self.localization_quality.observe(value,time.monotonic())
            self._localization_state_locked()

    def _accept_new_epoch_locked(self,value,now_wall):
        # Armed only by explicit reset after the previous publishers have exited.
        # A temporary validator applies the unchanged health/freshness policy
        # without binding the persistent consumer to the previous clock value.
        if isinstance(value,dict) and value.get('session_id')==self.localization_previous_session:
            return False
        probe=LocalizationQuality(required=True)
        probe.observe(value,now_wall)
        now_sim=self.get_clock().now().nanoseconds/1e9
        if not probe.check(now_sim,now_wall)['healthy']:
            self.localization_epoch_candidate=None
            return False
        candidate=(value['session_id'],value['sequence'],now_sim,value['accepted_stamp'])
        previous=self.localization_epoch_candidate
        if previous is None or candidate[0]!=previous[0] or candidate[2]<=previous[2]:
            self.localization_epoch_candidate=candidate
            return False
        # Heartbeats, a paused clock or duplicate accepted scans cannot open it.
        if candidate[1]<=previous[1] or candidate[3]<=previous[3]:return False
        self.localization_quality.reset()
        self.localization_epoch_pending=False
        self.localization_epoch_candidate=None
        return True

    def _localization_state_locked(self):
        if self.localization_epoch_pending:
            state=dict(required=self.localization_quality.required,healthy=False,
                reason='awaiting_new_clock_epoch',session_id=None,sequence=None,
                accepted_stamp=None,ready_since_stamp=None)
        else:
            state=self.localization_quality.check(self.get_clock().now().nanoseconds/1e9,time.monotonic())
        if self.localization_was_healthy and not state['healthy']:
            self.localization_fault_counter+=1
        self.localization_was_healthy=state['healthy']
        return dict(state,fault_counter=self.localization_fault_counter,
                    action_fault_counter=self.localization_action_fault_counter,
                    subscription_generation=self.localization_generation)

    def localization_state(self):
        with self.localization_lock:
            return self._localization_state_locked()

    def activate_localization_action(self):
        # Call while holding the main motion lock. Readiness and the outage
        # snapshot are atomic with respect to the independent quality callback.
        with self.localization_lock:
            state=self._localization_state_locked()
            if not state['healthy']:
                raise LocalizationDispatchRejected(state)
            self.localization_action_fault_counter=self.localization_fault_counter
            self.active=True

    def capture_gimbal_joints(self,msg):
        # This runs on the clock executor and never acquires the motion lock.
        received_wall=time.monotonic()
        captured=GimbalJointCapture(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
            tuple(msg.name),tuple(msg.position),tuple(msg.velocity),received_wall)
        with self.gimbal_capture_lock:
            if len(self.gimbal_captures)==self.gimbal_captures.maxlen:self.gimbal_capture_omitted+=1
            self.gimbal_captures.append(captured);self.gimbal_capture_count+=1
            self.gimbal_capture_last_stamp=captured.stamp

    def drain_gimbal_captures(self):
        # Called under the motion lock. Release the tiny queue lock before any
        # feedback/settlement work, so independent captures stay responsive.
        with self.gimbal_capture_lock:
            captured=list(self.gimbal_captures);self.gimbal_captures.clear()
        sim=self.get_clock().now().nanoseconds/1e9
        for sample in captured:
            delta=sim-sample.stamp
            if delta>.1+1e-9:
                self.gimbal_capture_dropped_old+=1;continue
            if delta < -GIMBAL_FEEDBACK_FUTURE_TOLERANCE-1e-9:
                self.gimbal_capture_dropped_future+=1;continue
            self.on_gimbal_joints(sample,sample.received_wall,sample.stamp)

    def on_gimbal_joints(self,msg,received_wall=None,captured_stamp=None):
        positions=dict(zip(msg.name,msg.position))
        values=[positions.get(name) for name in ('camera_pan_joint','camera_tilt_joint')]
        if any(v is None or not math.isfinite(v) for v in values):
            if self.gimbal_config[1]>0:
                with self.lock:
                    self.gimbal_settled_since=None;self.gimbal_settled_samples=0
            return
        stamp=(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
               if captured_stamp is None else captured_stamp)
        with self.lock:
            if stamp<=self.gimbal_joint_stamp:return
            self.gimbal_joint_stamp=stamp
            self.gimbal_joint_received=time.monotonic() if received_wall is None else received_wall
            self.gimbal_position=values
            if self.active and self.goal_request_kind=='gimbal' and stamp>=self.sim_started:
                rate=getattr(self,'gimbal_command_rate_rad_s',None)
                if (type(rate) in (int,float) and rate>0 and self.gimbal_final_command_wall is not None
                        and self.gimbal_joint_received<self.gimbal_final_command_wall):
                    # The header can lead /clock slightly. Preserve real position
                    # and age, but a pre-command capture cannot settle that command.
                    return
                tolerance,velocity_tolerance,duration=getattr(self,'gimbal_config',(GIMBAL_TOLERANCE,0.,0.))
                at_target=all(abs(a-b)<=tolerance for a,b in zip(values,self.gimbal_target))
                if velocity_tolerance>0:
                    velocities=dict(zip(msg.name,msg.velocity))
                    at_target=at_target and all(name in velocities and math.isfinite(velocities[name]) and
                        abs(velocities[name])<=velocity_tolerance for name in ('camera_pan_joint','camera_tilt_joint'))
                gap=stamp-getattr(self,'gimbal_previous_feedback_stamp',stamp)
                if not at_target or (duration>0 and gap>.100001):self.gimbal_settled_since=None
                if at_target and getattr(self,'gimbal_settled_since',None) is None:self.gimbal_settled_since=stamp
                self.gimbal_previous_feedback_stamp=stamp
                self.gimbal_settled_samples=self.gimbal_settled_samples+1 if at_target else 0

    def turn(self,yaw,task):
        if type(yaw) not in (int,float) or not math.isfinite(yaw) or abs(yaw)>math.pi:
            raise ValueError('观察朝向须为 -pi 至 pi 的有限弧度值')
        with self.lock:
            if self.active:
                raise ValueError('等待导航结束及转向服务就绪')
            state=self.state()
            require_dispatch_ready(state,'等待导航结束及转向服务就绪')
            if not self.spin_client.server_is_ready() or not state.get('diagnostics',{}).get('spin_ready',True):
                raise ValueError('等待导航结束及转向服务就绪')
            p=self.pose();self.check_start(task,p)
            angle=math.atan2(math.sin(yaw-p['yaw']),math.cos(yaw-p['yaw']))
            if not within_region(task['region'],p['x'],p['y'],p['yaw']) or not within_region(task['region'],p['x'],p['y'],yaw):
                raise ValueError('转向前后车体须位于任务区域内')
            # Spin keeps Nav2 collision prediction and the same safety relay as translation.
            self.region=task['region'];self.token=secrets.token_urlsafe(24);token=self.token
            self.activate_localization_action();self.phase='planning';self.error='';self.goal_handle=None
            self.stop_diagnostics=None
            self.started=self.last_heartbeat=time.monotonic()
            self.sim_started=self.get_clock().now().nanoseconds/1e9;self.sim_budget=60.
            self.goal=[p['x'],p['y']];self.path=[];self.track=[[p['x'],p['y'],p['z']]];self.feedback={}
            request=Spin.Goal();request.target_yaw=float(angle);request.time_allowance.sec=60
            self.send_goal(self.spin_client,request,token,'spin')
            return dict(token=token)

    def pan_tilt(self,pan,tilt,task=None):
        if type(pan) not in (int,float) or not math.isfinite(pan) or abs(pan)>2.356:
            raise ValueError('云台水平偏航须为 -2.356 至 2.356 弧度（±135°）')
        if type(tilt) not in (int,float) or not math.isfinite(tilt) or not (-0.611<=tilt<=0.262):
            raise ValueError('云台俯仰须为 -0.611 至 0.262 弧度（-35°~+15°）')
        with self.lock:
            rate=getattr(self,'gimbal_command_rate_rad_s',None)
            slew=type(rate) in (int,float) and rate>0
            if slew:self.drain_gimbal_captures()
            if self.active:raise ValueError('等待前一动作结束')
            require_dispatch_ready(self.state(),'等待定位与传感器就绪')
            p=self.pose()
            if task:
                self.check_start(task,p)
                if not within_region(task['region'],p['x'],p['y'],p['yaw']):
                    raise ValueError('车辆须位于任务区域内')
            rate=getattr(self,'gimbal_command_rate_rad_s',None)
            slew=type(rate) in (int,float) and rate>0
            if slew:
                if self.gimbal_command_fault:
                    raise ValueError('云台命令时钟失效：'+self.gimbal_command_fault)
                self.drain_gimbal_captures()
                sim=self.get_clock().now().nanoseconds/1e9
                if (self.gimbal_position is None or len(self.gimbal_position)!=2 or
                        not all(math.isfinite(value) for value in self.gimbal_position) or
                        not -GIMBAL_FEEDBACK_FUTURE_TOLERANCE-1e-9<=sim-self.gimbal_joint_stamp<=.1+1e-9 or
                        not 0<=time.monotonic()-self.gimbal_joint_received<=2.):
                    raise ValueError('等待当前云台关节反馈后再开始扫描')
                measured_start=list(self.gimbal_position)
            self.token=secrets.token_urlsafe(24);token=self.token
            self.activate_localization_action();self.phase='executing';self.error='';self.goal_handle=None
            self.goal_request_kind='gimbal'
            self.goal_request_id=secrets.token_hex(16)
            self.goal_dispatch=None
            self.stop_diagnostics=None
            self.started=self.last_heartbeat=time.monotonic()
            self.sim_started=self.get_clock().now().nanoseconds/1e9
            duration=max(1.5,abs(pan)/1.5+1.5)
            self.sim_budget=duration+5.0
            settings=[float(self.get_parameter(n).value) for n in
                      ('gimbal_position_tolerance_rad','gimbal_velocity_tolerance_rad_s',
                       'gimbal_settle_duration_s','gimbal_action_timeout_s')]
            if not all(math.isfinite(v) and v>=0 for v in settings) or settings[0]<=0:
                self.active=False;self.phase='failed'
                raise ValueError('Invalid gimbal feedback configuration')
            self.gimbal_config=tuple(settings[:3]);self.gimbal_settled_since=None
            self.gimbal_previous_feedback_stamp=self.sim_started
            if slew:
                distance=math.hypot(pan-measured_start[0],tilt-measured_start[1])
                duration=max(1.5,distance/rate+settings[2]+1.5)
                self.sim_budget=duration+5.
                self.gimbal_command=measured_start
                self.gimbal_command_stamp=self.sim_started
                self.gimbal_final_command_stamp=None
                self.gimbal_final_command_wall=None
            if settings[3]>0:self.sim_budget=settings[3]
            self.gimbal_deadline=time.monotonic()+max(30.,duration*10.)
            self.gimbal_target=[float(pan),float(tilt)];self.gimbal_settled_samples=0
            self.goal=[p['x'],p['y']];self.path=[];self.track=[[p['x'],p['y'],p['z']]];self.feedback={}
            if not slew:
                self.pub_camera_pan.publish(Float64(data=float(pan)))
                self.pub_camera_tilt.publish(Float64(data=float(tilt)))
            return dict(token=token)

    def _advance_gimbal_command(self,sim,fresh):
        """A bounded command ramp, independent of measured completion feedback."""
        elapsed=sim-self.gimbal_command_stamp
        self.gimbal_command_stamp=sim
        # Never accumulate a catch-up jump while feedback or simulation time is
        # unavailable. The actual joint feedback alone confirms the final pose.
        if not fresh or elapsed<=0 or self.gimbal_final_command_stamp is not None:return
        delta=[target-command for target,command in zip(self.gimbal_target,self.gimbal_command)]
        distance=math.hypot(*delta)
        scale=min(1.,self.gimbal_command_rate_rad_s*min(elapsed,.1)/distance) if distance>0 else 1.
        self.gimbal_command=(list(self.gimbal_target) if scale==1. else
                             [command+change*scale for command,change in zip(self.gimbal_command,delta)])
        self.pub_camera_pan.publish(Float64(data=self.gimbal_command[0]))
        self.pub_camera_tilt.publish(Float64(data=self.gimbal_command[1]))
        if scale==1.:
            self.gimbal_command=list(self.gimbal_target)
            self.gimbal_final_command_stamp=sim
            self.gimbal_final_command_wall=time.monotonic()
            self.gimbal_settled_since=None;self.gimbal_settled_samples=0
            self.gimbal_previous_feedback_stamp=sim

    def send_goal(self,client,request,token,kind,feedback_callback=None):
        goal_uuid=UUID(uuid=list(secrets.token_bytes(16)))
        self.goal_request_id=bytes(goal_uuid.uuid).hex();self.goal_request_kind=kind
        self.goal_sent_wall=time.monotonic()
        self.goal_dispatch=dict(request_id=self.goal_request_id,kind=kind,sent_wall=self.goal_sent_wall)
        try:
            future=client.send_goal_async(request,feedback_callback=feedback_callback,goal_uuid=goal_uuid)
        except Exception as error:
            self.active=False;self.phase='failed';self.error=str(error)
            self.permit.publish(Bool(data=False))
            raise
        self.goal_request_future=future
        self.goal_dispatch['send_return_wall']=time.monotonic()
        request_id=self.goal_request_id
        # RcutilsLogger resolves caller source paths synchronously on first use.
        # Never let diagnostic I/O hold the motion lock before response handling.
        self.goal_dispatch['callback_attach_started_wall']=time.monotonic()
        future.add_done_callback(lambda f:self.accepted(f,token,request_id))

    def pose(self):
        health=self.localization_state()
        if not health['healthy']:
            raise LocalizationDispatchRejected(health)
        t=self.buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
        age=(self.get_clock().now().nanoseconds-rclpy.time.Time.from_msg(t.header.stamp).nanoseconds)/1e9
        if not -MAX_FUTURE_TF_SECONDS <= age <= 2:raise ValueError(f'车辆定位时间失效：TF 与仿真时钟相差 {age:.3f} 秒')
        p,q=t.transform.translation,t.transform.rotation
        return dict(x=p.x,y=p.y,z=p.z,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

    def refresh_tf_listener(self):
        with self.lock:
            self.listener.unregister()
            self.listener=TransformListener(self.buffer,self)

    def check_start(self,task,pose):
        if self.start_checked:return
        x,y=task['start'][:2]
        if math.hypot(pose['x']-x,pose['y']-y)>.25:
            raise ValueError(f'定位起点与本轮计划起点不一致：计划 ({x:.2f}, {y:.2f})，定位 ({pose["x"]:.2f}, {pose["y"]:.2f})')
        self.start_checked=True

    def state(self):
        with self.lock:
            rate=getattr(self,'gimbal_command_rate_rad_s',None)
            slew=type(rate) in (int,float) and rate>0
            if slew:self.drain_gimbal_captures()
            pose_error=''
            try:p=self.pose()
            except Exception as error:p=None;pose_error=str(error)
            if slew:self.drain_gimbal_captures()
            wall=time.monotonic();sim=self.get_clock().now().nanoseconds/1e9
            lifecycle=self.nav2_lifecycle_snapshot(wall) if hasattr(self,'nav2_lifecycle_lock') else dict(all_active=True,behavior_active=True,nodes={})
            navigation_ready=self.client.server_is_ready() and lifecycle['all_active'];map_fresh=self.grid_fresh()
            spin_ready=self.spin_client.server_is_ready() and lifecycle['behavior_active']
            depth_active=wall-self.last_depth<6
            age=lambda stamp:round(wall-stamp,3) if stamp else None
            diagnostics=dict(sim_stamp=sim,pose_error=pose_error,navigation_ready=navigation_ready,
                nav2_lifecycle_active=lifecycle['all_active'],nav2_lifecycle=lifecycle['nodes'],
                laser_localization=self.localization_state(),
                spin_ready=spin_ready,
                depth_age_wall_seconds=age(self.last_depth),depth_message=self.depth_message,
                quality_age_wall_seconds=age(self.quality_received),quality_present=self.quality is not None,
                costmap_age_wall_seconds=age(self.grid_received) if self.grid else None,costmap_fresh=map_fresh,
                costmap_age_sim_seconds=sim-rclpy.time.Time.from_msg(self.grid.header.stamp).nanoseconds/1e9 if self.grid else None,
                goal_request_id=self.goal_request_id,goal_request_kind=self.goal_request_kind,
                goal_dispatch=dict(self.goal_dispatch) if self.goal_dispatch else None,
                goal_response_future_done=self.goal_request_future.done() if self.goal_request_future else None,
                goal_response_age_wall_seconds=age(self.goal_sent_wall) if self.active and self.phase=='planning' else None,
                gimbal_target=self.gimbal_target,gimbal_position=self.gimbal_position,
                gimbal_feedback_age_wall_seconds=age(self.gimbal_joint_received),
                gimbal_joint_stamp_sim=self.gimbal_joint_stamp if self.gimbal_joint_stamp>=0 else None,
                gimbal_feedback_age_sim_seconds=sim-self.gimbal_joint_stamp if self.gimbal_joint_stamp>=0 else None,
                gimbal_settled_samples=self.gimbal_settled_samples)
            if slew:
                with self.gimbal_capture_lock:
                    diagnostics.update(gimbal_feedback_independent_capture=True,
                        gimbal_feedback_receipt_source='clock_executor_capture',
                        gimbal_queued_capture_count=len(self.gimbal_captures),
                        gimbal_capture_count=self.gimbal_capture_count,
                        gimbal_capture_last_stamp_sim=self.gimbal_capture_last_stamp,
                        gimbal_capture_omitted=self.gimbal_capture_omitted,
                        gimbal_capture_dropped_old=self.gimbal_capture_dropped_old,
                        gimbal_capture_dropped_future=self.gimbal_capture_dropped_future)
            bounds=None
            if self.grid:
                i=self.grid.info;o=i.origin.position
                bounds=[o.x,o.y,o.x+i.width*i.resolution,o.y+i.height*i.resolution]
            return dict(active=self.active,phase=self.phase,error=self.error,pose=p,
                        replan_allowed=self.phase=='cancelled' and self.error==NAVIGATION_STALL_REASON,
                        goal=self.goal,path=self.path,track=self.track,feedback=self.feedback,
                        map_bounds=bounds,navigation_grid=self.grid_view,path_age=time.monotonic()-self.last_path if self.path else None,
                        map_fresh=map_fresh,diagnostics=diagnostics,stop_diagnostics=self.stop_diagnostics,
                        ready=navigation_ready and p is not None and depth_active and map_fresh,
                        depth_active=depth_active,depth_message=self.depth_message)

    def record_stop(self,reason,**details):
        # Diagnostic failures must never prevent permit revocation or cancellation.
        try:self.stop_diagnostics=dict(reason=reason,**details,health=self.state()['diagnostics'])
        except Exception as error:self.stop_diagnostics=dict(reason=reason,diagnostic_error=str(error),**details)

    def on_grid(self,msg):
        if msg.header.frame_id=='map':
            with self.lock:
                self.grid=msg;self.grid_received=time.monotonic()
                i=msg.info
                self.grid_view=dict(version=self.grid_received,width=i.width,height=i.height,resolution=i.resolution,
                                    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                                    origin=[i.origin.position.x,i.origin.position.y],data=base64.b64encode(msg.data.tobytes()).decode())

    def on_keepout(self,msg):
        if msg.header.frame_id=='map':
            with self.lock:self.keepout_grid=msg

    def grid_fresh(self):
        if self.grid is None:return False
        stamp=rclpy.time.Time.from_msg(self.grid.header.stamp).nanoseconds/1e9
        age=self.get_clock().now().nanoseconds/1e9-stamp
        # The 1 Hz costmap publisher uses simulation time, not wall time.
        # A separate wall limit also rejects a disconnected/paused producer.
        return -.1<=age<=3. and time.monotonic()-self.grid_received<=30.

    def on_path(self,msg):
        with self.lock:
            if not self.active or self.phase=='cancelling' or msg.header.frame_id!='map':return
            if not msg.poses or not self.goal:return
            end=msg.poses[-1].pose.position
            if math.hypot(end.x-self.goal[0],end.y-self.goal[1])>.3:return
            # Preserve turns; no geometry simplification of the path actually planned by Nav2.
            self.path=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses]
            self.last_path=time.monotonic()

    def on_feedback(self,msg,token):
        with self.lock:
            if token!=self.token or not self.active:return
            f=msg.feedback
            distance=float(f.distance_remaining)
            now=self.get_clock().now().nanoseconds/1e9
            if math.isfinite(distance) and distance>.25 and distance<=self.best_distance-.15:
                self.best_distance=distance;self.last_progress_sim=now
            self.feedback=dict(distance_remaining=round(float(f.distance_remaining),2),
                               navigation_seconds=f.navigation_time.sec+f.navigation_time.nanosec/1e9,
                               recoveries=int(f.number_of_recoveries))

    def check_goal(self,goal):
        if not self.grid_fresh():
            raise ValueError('等待最新导航地图，请稍后重试')
        i=self.grid.info;o=i.origin.position
        x=math.floor((goal[0]-o.x)/i.resolution);y=math.floor((goal[1]-o.y)/i.resolution)
        if not (0<=x<i.width and 0<=y<i.height):
            raise ValueError('目标超出当前导航地图边界；请先导航到已观测的边缘，逐步扩展地图')
        cost=self.grid.data[y*i.width+x]
        if cost<0:raise ValueError('目标尚未观测；请先选择已观测的可通行位置')
        if cost>=99:raise ValueError('目标位于障碍或车体安全余量内，请远离障碍重新选点')

    def start(self,body,task,*,precise_revisit=False):
        if not isinstance(body,dict) or set(body)!={'experiment','goal'} or body['experiment']!=task['id']:
            raise ValueError('实验已改变，请刷新状态后重试')
        if type(precise_revisit) is not bool:raise ValueError('精确回访标志必须是内部布尔值')
        behavior_tree=None
        if precise_revisit:
            from pathlib import Path as FilePath
            from ament_index_python.packages import get_package_share_directory
            import xml.etree.ElementTree as XML
            installed=FilePath(get_package_share_directory('mooncar_navigation'))/'config'/'behavior_trees'/'navigate_to_pose_revisit.xml'
            if not installed.is_file():raise ValueError('精确回访行为树尚未安装，已停止')
            try:
                tree=XML.parse(installed);followers=tree.findall('.//FollowPath')
                valid=(tree.getroot().tag=='root' and len(followers)==1
                       and followers[0].get('goal_checker_id')=='revisit_goal_checker')
            except (OSError,XML.ParseError) as error:raise ValueError('精确回访行为树资源无效，已停止') from error
            if not valid:raise ValueError('精确回访行为树没有专用到位检查器，已停止')
            behavior_tree=str(installed)
        goal=body['goal']
        if not isinstance(goal,list) or len(goal)!=2 or not all(type(v) in (int,float) and math.isfinite(v) for v in goal):
            raise ValueError('行驶目标必须是有限的 X、Y 坐标')
        with self.lock:
            if self.active or self.phase=='cancelling':raise ValueError('请先等待当前导航结束')
            require_dispatch_ready(self.state(),'等待导航、定位与传感器就绪')
            self.check_goal(goal)
            p=self.pose();self.check_start(task,p)
            yaw=math.atan2(goal[1]-p['y'],goal[0]-p['x'])
            if not within_region(task['region'],p['x'],p['y'],p['yaw']) or not within_region(task['region'],*goal,yaw):
                raise ValueError('起点及目标的整个车体需位于探测区域内，并留出边界余量')
            self.region=task['region'];self.token=secrets.token_urlsafe(24);token=self.token
            self.activate_localization_action();self.phase='planning';self.error='';self.goal_handle=None
            self.stop_diagnostics=None
            self.started=self.last_heartbeat=time.monotonic()
            self.sim_started=self.get_clock().now().nanoseconds/1e9
            self.sim_budget=min(900.,max(120.,math.hypot(goal[0]-p['x'],goal[1]-p['y'])/.06+90))
            self.best_distance=math.inf;self.best_goal_distance=math.hypot(goal[0]-p['x'],goal[1]-p['y'])
            self.last_progress_sim=self.sim_started
            self.goal=list(goal);self.path=[];self.track=[[p['x'],p['y'],p['z']]];self.feedback={}
            request=NavigateToPose.Goal();request.pose.header.frame_id='map'
            if precise_revisit:request.behavior_tree=behavior_tree
            request.pose.pose.position.x,request.pose.pose.position.y=map(float,goal)
            request.pose.pose.orientation.z,request.pose.pose.orientation.w=math.sin(yaw/2),math.cos(yaw/2)
            self.send_goal(self.client,request,token,'navigate_to_pose',lambda msg:self.on_feedback(msg,token))
            if precise_revisit:self.goal_dispatch.update(precise_revisit=True,behavior_tree=behavior_tree,
                goal_checker_id='revisit_goal_checker',position_tolerance_m=.05)
            return dict(token=token)

    def accepted(self,future,token,request_id=''):
        with self.lock:
            try:
                handle=future.result()
                if token!=self.token:
                    if handle.accepted:handle.cancel_goal_async()
                    return
                if not self.active:
                    if handle.accepted:handle.cancel_goal_async()
                    return
                handled=time.monotonic()
                dispatch=getattr(self,'goal_dispatch',None)
                if isinstance(dispatch,dict) and dispatch['request_id']==request_id:
                    dispatch.update(response_handled_wall=handled,
                        response_latency_wall_seconds=handled-self.goal_sent_wall,accepted=bool(handle.accepted))
                # Enforce the same deadline even when a completed Future's callback
                # runs before the watchdog gets its next turn.
                if handled-self.goal_sent_wall>GOAL_RESPONSE_TIMEOUT_SECONDS:
                    self.record_stop(GOAL_RESPONSE_TIMEOUT_REASON,goal_request_id=request_id,
                        goal_request_kind=self.goal_request_kind,goal_response_future_done=True)
                    self.active=False;self.phase='failed';self.error=GOAL_RESPONSE_TIMEOUT_REASON
                    self.permit.publish(Bool(data=False))
                    if handle.accepted:handle.cancel_goal_async()
                    return
                if not handle.accepted:
                    self.active=False;self.phase='failed';self.error='Nav2 拒绝目标'
                    self.record_stop(self.error,action_status='rejected');return
                self.goal_handle=handle
                if self.phase=='cancelling':handle.cancel_goal_async()
                else:self.phase='executing'
                handle.get_result_async().add_done_callback(lambda f:self.finished(f,token))
            except Exception as error:
                if token!=self.token:return
                self.active=False;self.phase='failed';self.error=str(error)
                self.record_stop(self.error)

    def finished(self,future,token):
        with self.lock:
            if token!=self.token:return
            try:
                result=future.result()
                self.phase='succeeded' if result.status==4 and result.result.error_code==0 else ('cancelled' if self.phase=='cancelling' else 'failed')
                if self.phase=='failed' and not self.error:
                    errors={203:'车辆超出导航地图',204:'目标超出导航地图',205:'车辆位于障碍区域',206:'目标位于障碍区域',207:'路径规划超时',208:'没有找到可行路径',104:'控制器持续无法给出可行运动',105:'长时间未能前进',106:'没有可行控制指令',107:'控制器运行超时'}
                    code=result.result.error_code;detail=result.result.error_msg
                    self.error=errors.get(code,'Nav2 未完成目标')+(f'（{code}）' if code else '')+(f'：{detail}' if detail else '')
                if self.phase=='failed':self.record_stop(self.error,action_status=result.status,
                    nav2_error_code=result.result.error_code,nav2_error_msg=result.result.error_msg)
            except Exception as error:
                self.phase='failed';self.error=str(error);self.record_stop(self.error)
            self.active=False;self.goal_handle=None;self.permit.publish(Bool(data=False))

    def heartbeat(self,body):
        with self.lock:
            if not isinstance(body,dict) or not isinstance(body.get('token'),str) or not secrets.compare_digest(body['token'],self.token):
                raise ValueError('导航连接已失效')
            self.last_heartbeat=time.monotonic()
            return dict(active=self.active)

    def cancel(self,reason=''):
        with self.lock:
            self.permit.publish(Bool(data=False))
            if self.active and self.phase!='cancelling':
                self.record_stop(reason)
                self.phase='cancelling';self.error=reason
                if self.goal_handle:self.goal_handle.cancel_goal_async()
                if self.goal_request_kind=='gimbal':
                    self.active=False
            return dict(accepted=True)

    def tick(self):
        with self.lock:
            rate=getattr(self,'gimbal_command_rate_rad_s',None)
            slew=type(rate) in (int,float) and rate>0
            if slew:self.drain_gimbal_captures()
            if self.active:
                health=self.localization_state()
                if (not health['healthy'] or health.get('fault_counter',0)!=health.get('action_fault_counter',0)):
                    self.cancel('激光定位质量失效，已停止本次动作')
                    self.permit.publish(Bool(data=False))
                    return
            if self.active and self.goal_request_kind=='gimbal':
                if time.monotonic()-self.last_heartbeat>.9:
                    self.cancel('页面失联，已停止运动')
                    return
                sim=self.get_clock().now().nanoseconds/1e9
                rate=getattr(self,'gimbal_command_rate_rad_s',None)
                slew=type(rate) in (int,float) and rate>0
                if slew and sim<self.gimbal_command_stamp-1e-9:
                    self.gimbal_command_fault='clock_moved_backwards'
                    self.cancel('云台仿真时钟回退，已停止本次扫描')
                    self.permit.publish(Bool(data=False))
                    return
                feedback_age=.1 if slew or getattr(self,'gimbal_config',(GIMBAL_TOLERANCE,0.,0.))[1]>0 else .5
                earliest=-GIMBAL_FEEDBACK_FUTURE_TOLERANCE-1e-9 if slew else 0
                fresh=(time.monotonic()-self.gimbal_joint_received<=2. and
                       earliest<=sim-self.gimbal_joint_stamp<=feedback_age+(1e-9 if slew else 0) and self.gimbal_joint_stamp>=self.sim_started)
                duration=getattr(self,'gimbal_config',(GIMBAL_TOLERANCE,0.,0.))[2]
                settled=getattr(self,'gimbal_settled_since',None)
                duration_ready=duration==0 or (settled is not None and self.gimbal_joint_stamp-settled>=duration)
                commanded=(not slew or (self.gimbal_final_command_stamp is not None and
                            self.gimbal_joint_stamp>=self.gimbal_final_command_stamp))
                if fresh and commanded and self.gimbal_settled_samples>=2 and duration_ready:
                    self.active=False;self.phase='succeeded'
                elif sim-self.sim_started>self.sim_budget or time.monotonic()>=self.gimbal_deadline:
                    self.error='云台反馈未确认到位，已停止本次扫描'
                    self.record_stop(self.error)
                    self.active=False;self.phase='failed'
                elif slew:self._advance_gimbal_command(sim,fresh)
                self.permit.publish(Bool(data=False))
                return
            if (self.active and self.phase=='planning' and self.goal_request_future is not None
                    and time.monotonic()-self.goal_sent_wall>GOAL_RESPONSE_TIMEOUT_SECONDS):
                reason=GOAL_RESPONSE_TIMEOUT_REASON
                self.record_stop(reason,goal_request_id=self.goal_request_id,goal_request_kind=self.goal_request_kind,
                                 goal_response_future_done=self.goal_request_future.done(),
                                 goal_server_ready=(self.spin_client if self.goal_request_kind=='spin' else self.client).server_is_ready())
                self.active=False;self.phase='failed';self.error=reason
                self.permit.publish(Bool(data=False))
                return
            if self.active and self.phase!='cancelling':
                reason=''
                if time.monotonic()-self.last_heartbeat>.9:reason='页面失联，已停止运动'
                elif self.get_clock().now().nanoseconds/1e9-self.sim_started>self.sim_budget:reason='导航超过本段仿真时间预算，已停止运动'
                elif time.monotonic()-self.started>3600:reason='导航超过一小时运行上限，已停止运动'
                elif time.monotonic()-self.last_depth>6:reason='传感器未更新，已停止运动'
                elif not self.grid_fresh():reason='导航地图失效，已停止运动'
                else:
                    try:
                        p=self.pose()
                        if not self.track or math.hypot(p['x']-self.track[-1][0],p['y']-self.track[-1][1])>=.05:
                            self.track.append([p['x'],p['y'],p['z']])
                        if not within_region(self.region,p['x'],p['y'],p['yaw']):reason='接近探测区域边界，已停止运动'
                        limits=getattr(self,'mission_limits',None)
                        if limits:
                            if self.get_clock().now().nanoseconds/1e9>=limits['deadline']:reason='自主探测时间预算已用尽'
                            elif (limits.get('radius') is not None and
                                  math.hypot(p['x']-limits['origin'][0],p['y']-limits['origin'][1])>limits['radius']-.65):reason='接近自主探测活动距离限制'
                        if not reason and self.goal_request_kind=='navigate_to_pose' and self.phase=='executing':
                            direct=math.hypot(p['x']-self.goal[0],p['y']-self.goal[1])
                            if direct<=self.best_goal_distance-.15:
                                self.best_goal_distance=direct
                                self.last_progress_sim=self.get_clock().now().nanoseconds/1e9
                            if (direct>.25 and self.feedback.get('distance_remaining',math.inf)>.25 and
                                     self.get_clock().now().nanoseconds/1e9-self.last_progress_sim>=25.):
                                reason=NAVIGATION_STALL_REASON
                    except Exception:reason='车辆定位失效，已停止运动'
                if reason:self.cancel(reason)
            self.permit.publish(Bool(data=self.active and self.phase=='executing'))

    def reset(self):
        self.cancel()
        with self.lock:
            self.active=False;self.goal_handle=None;self.token='';self.phase='idle';self.error='';self.last_depth=0
            self.goal_dispatch=None
            self.gimbal_deadline=0.
            self.gimbal_target=None;self.gimbal_position=None
            self.gimbal_joint_received=0.;self.gimbal_joint_stamp=-1.;self.gimbal_settled_samples=0
            self.gimbal_command=None;self.gimbal_command_stamp=None
            self.gimbal_final_command_stamp=None;self.gimbal_final_command_wall=None;self.gimbal_command_fault=None
            rate=getattr(self,'gimbal_command_rate_rad_s',None)
            if type(rate) in (int,float) and rate>0:
                with self.gimbal_capture_lock:
                    self.gimbal_captures.clear();self.gimbal_capture_count=0;self.gimbal_capture_omitted=0
                    self.gimbal_capture_last_stamp=None
                    self.gimbal_capture_dropped_old=0;self.gimbal_capture_dropped_future=0
            self.path=[];self.track=[];self.goal=None;self.grid=None;self.grid_view=None;self.feedback={};self.last_path=0.
            self.keepout_grid=None
            self.quality=None;self.quality_received=0.;self.mission_limits=None
            self.stop_diagnostics=None
            with self.localization_lock:
                self.localization_generation+=1
                if self.localization_subscription is not None:
                    self.clock_node.destroy_subscription(self.localization_subscription)
                    self.localization_subscription=None
                self.localization_previous_session=(self.localization_quality.session_id
                    or self.localization_previous_session)
                self.localization_epoch_pending=self.localization_quality.required
                self.localization_epoch_candidate=None
                self.localization_quality.reset()
                self.localization_fault_counter=0;self.localization_action_fault_counter=0
                self.localization_was_healthy=False
                if self.localization_quality.required:self.subscribe_localization_quality()
            self.goal_request_future=None;self.goal_request_id='';self.goal_request_kind='';self.goal_sent_wall=0.
            self.grid_received=0.;self.depth_message='等待新实验传感器'
            self.start_checked=False
            self.listener.unregister()
            self.buffer.jump_handle.unregister();self.destroy_service(self.buffer.srv)
            self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self)
            self.client.destroy();self.spin_client.destroy()
            self.client=ActionClient(self,NavigateToPose,'/navigate_to_pose',callback_group=self.control_group)
            self.spin_client=ActionClient(self,Spin,'/spin',callback_group=self.control_group)
            if hasattr(self,'nav2_lifecycle_lock'):
                with self.nav2_lifecycle_lock:
                    for name,(future,_) in list(self.nav2_lifecycle_pending.items()):
                        client=self.nav2_lifecycle_clients.get(name)
                        if client is not None:
                            try:client.remove_pending_request(future)
                            except Exception:pass
                        future.cancel()
                    self.nav2_lifecycle_pending.clear()
                    self.nav2_lifecycle_states.clear()
                    for client in self.nav2_lifecycle_clients.values():
                        self.destroy_client(client)
                    self.nav2_lifecycle_clients=self._create_nav2_lifecycle_clients()
