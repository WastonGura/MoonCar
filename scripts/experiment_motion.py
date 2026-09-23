"""Leased Nav2 goal execution for the isolated simulation, with boundary stopping."""
import math
import secrets
import threading
import time
import base64
import json
from nav_msgs.msg import Path, OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav2_msgs.action import NavigateToPose, Spin
from std_msgs.msg import Bool, Float64, String
from tf2_ros import Buffer, TransformListener
from unique_identifier_msgs.msg import UUID


GOAL_RESPONSE_TIMEOUT_SECONDS = 5.
MAX_FUTURE_TF_SECONDS = .3
NAVIGATION_STALL_REASON = '导航持续未接近目标，已取消候选动作'


def within_region(region, x, y, yaw):
    hx=.43*abs(math.cos(yaw))+.36*abs(math.sin(yaw))+.1
    hy=.43*abs(math.sin(yaw))+.36*abs(math.cos(yaw))+.1
    return region[0]+hx <= x <= region[2]-hx and region[1]+hy <= y <= region[3]-hy


class ExperimentMotion(Node):
    def __init__(self):
        super().__init__('mooncar_experiment_motion')
        self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self)
        self.client=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.spin_client=ActionClient(self,Spin,'/spin')
        self.permit=self.create_publisher(Bool,'/mooncar/workbench/permit',1)
        self.pub_camera_pan=self.create_publisher(Float64,'/mooncar/cmd_camera_pan',10)
        self.pub_camera_tilt=self.create_publisher(Float64,'/mooncar/cmd_camera_tilt',10)
        self.gimbal_deadline=0.
        self.lock=threading.RLock()
        self.active=False;self.goal_handle=None;self.token='';self.phase='idle';self.error=''
        self.last_heartbeat=0.;self.started=0.;self.last_depth=0.;self.depth_message='等待传感器';self.region=None
        self.path=[];self.track=[];self.goal=None;self.grid=None;self.grid_view=None;self.grid_received=0.;self.last_path=0.
        self.feedback={};self.sim_started=0.;self.sim_budget=180.
        self.best_distance=math.inf;self.best_goal_distance=math.inf;self.last_progress_sim=0.
        self.goal_request_future=None;self.goal_request_id='';self.goal_request_kind='';self.goal_sent_wall=0.
        self.start_checked=False
        self.quality=None;self.quality_received=0.;self.mission_limits=None
        self.stop_diagnostics=None
        self.create_subscription(String,'/mooncar/map/quality',self.on_quality,1)
        self.create_subscription(DiagnosticArray,'/mooncar/terrain/status',self.depth,10)
        self.create_subscription(Path,'/plan',self.on_path,10)
        self.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.on_grid,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

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

    def turn(self,yaw,task):
        if type(yaw) not in (int,float) or not math.isfinite(yaw) or abs(yaw)>math.pi:
            raise ValueError('观察朝向须为 -pi 至 pi 的有限弧度值')
        with self.lock:
            if self.active or not self.state()['ready'] or not self.spin_client.server_is_ready():
                raise ValueError('等待导航结束及转向服务就绪')
            p=self.pose();self.check_start(task,p)
            angle=math.atan2(math.sin(yaw-p['yaw']),math.cos(yaw-p['yaw']))
            if not within_region(task['region'],p['x'],p['y'],p['yaw']) or not within_region(task['region'],p['x'],p['y'],yaw):
                raise ValueError('转向前后车体须位于任务区域内')
            # Spin keeps Nav2 collision prediction and the same safety relay as translation.
            self.region=task['region'];self.token=secrets.token_urlsafe(24);token=self.token
            self.active=True;self.phase='planning';self.error='';self.goal_handle=None
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
            if self.active:raise ValueError('等待前一动作结束')
            if not self.state()['ready']:raise ValueError('等待定位与传感器就绪')
            p=self.pose()
            if task:
                self.check_start(task,p)
                if not within_region(task['region'],p['x'],p['y'],p['yaw']):
                    raise ValueError('车辆须位于任务区域内')
            self.token=secrets.token_urlsafe(24);token=self.token
            self.active=True;self.phase='executing';self.error='';self.goal_handle=None
            self.goal_request_kind='gimbal'
            self.goal_request_id=secrets.token_hex(16)
            self.stop_diagnostics=None
            self.started=self.last_heartbeat=time.monotonic()
            self.sim_started=self.get_clock().now().nanoseconds/1e9
            duration=max(1.5,abs(pan)/1.5+1.5)
            self.sim_budget=duration+5.0
            self.gimbal_deadline=time.monotonic()+duration
            self.goal=[p['x'],p['y']];self.path=[];self.track=[[p['x'],p['y'],p['z']]];self.feedback={}
            self.pub_camera_pan.publish(Float64(data=float(pan)))
            self.pub_camera_tilt.publish(Float64(data=float(tilt)))
            return dict(token=token)

    def send_goal(self,client,request,token,kind,feedback_callback=None):
        goal_uuid=UUID(uuid=list(secrets.token_bytes(16)))
        self.goal_request_id=bytes(goal_uuid.uuid).hex();self.goal_request_kind=kind
        self.goal_sent_wall=time.monotonic()
        try:
            future=client.send_goal_async(request,feedback_callback=feedback_callback,goal_uuid=goal_uuid)
        except Exception as error:
            self.active=False;self.phase='failed';self.error=str(error)
            self.permit.publish(Bool(data=False))
            raise
        self.goal_request_future=future
        self.get_logger().info(f'Nav2 goal submitted: kind={kind} id={self.goal_request_id}')
        request_id=self.goal_request_id
        future.add_done_callback(lambda f:self.accepted(f,token,request_id))

    def pose(self):
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
            pose_error=''
            try:p=self.pose()
            except Exception as error:p=None;pose_error=str(error)
            wall=time.monotonic();sim=self.get_clock().now().nanoseconds/1e9
            navigation_ready=self.client.server_is_ready();map_fresh=self.grid_fresh()
            depth_active=wall-self.last_depth<6
            age=lambda stamp:round(wall-stamp,3) if stamp else None
            diagnostics=dict(sim_stamp=sim,pose_error=pose_error,navigation_ready=navigation_ready,
                spin_ready=self.spin_client.server_is_ready(),
                depth_age_wall_seconds=age(self.last_depth),depth_message=self.depth_message,
                quality_age_wall_seconds=age(self.quality_received),quality_present=self.quality is not None,
                costmap_age_wall_seconds=age(self.grid_received) if self.grid else None,costmap_fresh=map_fresh,
                costmap_age_sim_seconds=sim-rclpy.time.Time.from_msg(self.grid.header.stamp).nanoseconds/1e9 if self.grid else None,
                goal_request_id=self.goal_request_id,goal_request_kind=self.goal_request_kind,
                goal_response_future_done=self.goal_request_future.done() if self.goal_request_future else None,
                goal_response_age_wall_seconds=age(self.goal_sent_wall) if self.active and self.phase=='planning' else None)
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
                                    origin=[i.origin.position.x,i.origin.position.y],data=base64.b64encode(msg.data.tobytes()).decode())

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

    def start(self,body,task):
        if not isinstance(body,dict) or set(body)!={'experiment','goal'} or body['experiment']!=task['id']:
            raise ValueError('实验已改变，请刷新状态后重试')
        goal=body['goal']
        if not isinstance(goal,list) or len(goal)!=2 or not all(type(v) in (int,float) and math.isfinite(v) for v in goal):
            raise ValueError('行驶目标必须是有限的 X、Y 坐标')
        with self.lock:
            if self.active or self.phase=='cancelling':raise ValueError('请先等待当前导航结束')
            if not self.state()['ready']:raise ValueError('等待导航、定位与传感器就绪')
            self.check_goal(goal)
            p=self.pose();self.check_start(task,p)
            yaw=math.atan2(goal[1]-p['y'],goal[0]-p['x'])
            if not within_region(task['region'],p['x'],p['y'],p['yaw']) or not within_region(task['region'],*goal,yaw):
                raise ValueError('起点及目标的整个车体需位于探测区域内，并留出边界余量')
            self.region=task['region'];self.token=secrets.token_urlsafe(24);token=self.token
            self.active=True;self.phase='planning';self.error='';self.goal_handle=None
            self.stop_diagnostics=None
            self.started=self.last_heartbeat=time.monotonic()
            self.sim_started=self.get_clock().now().nanoseconds/1e9
            self.sim_budget=min(900.,max(120.,math.hypot(goal[0]-p['x'],goal[1]-p['y'])/.06+90))
            self.best_distance=math.inf;self.best_goal_distance=math.hypot(goal[0]-p['x'],goal[1]-p['y'])
            self.last_progress_sim=self.sim_started
            self.goal=list(goal);self.path=[];self.track=[[p['x'],p['y'],p['z']]];self.feedback={}
            request=NavigateToPose.Goal();request.pose.header.frame_id='map'
            request.pose.pose.position.x,request.pose.pose.position.y=map(float,goal)
            request.pose.pose.orientation.z,request.pose.pose.orientation.w=math.sin(yaw/2),math.cos(yaw/2)
            self.send_goal(self.client,request,token,'navigate_to_pose',lambda msg:self.on_feedback(msg,token))
            return dict(token=token)

    def accepted(self,future,token,request_id=''):
        with self.lock:
            try:
                handle=future.result()
                self.get_logger().info(f'Nav2 goal response: id={request_id} accepted={handle.accepted} current={token==self.token}')
                if token!=self.token:
                    if handle.accepted:handle.cancel_goal_async()
                    return
                if not self.active:
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
                self.get_logger().error(f'Nav2 goal response failed: id={request_id} error={error}')
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
            if self.active and self.goal_request_kind=='gimbal':
                if time.monotonic()-self.last_heartbeat>.9:
                    self.cancel('页面失联，已停止运动')
                    return
                if time.monotonic()>=self.gimbal_deadline:
                    self.active=False;self.phase='succeeded'
                self.permit.publish(Bool(data=False))
                return
            if (self.active and self.phase=='planning' and self.goal_request_future is not None
                    and time.monotonic()-self.goal_sent_wall>GOAL_RESPONSE_TIMEOUT_SECONDS):
                reason='Nav2 目标接收超过 5 秒，已停止本次动作'
                self.record_stop(reason,goal_request_id=self.goal_request_id,goal_request_kind=self.goal_request_kind,
                                 goal_response_future_done=self.goal_request_future.done(),
                                 goal_server_ready=(self.spin_client if self.goal_request_kind=='spin' else self.client).server_is_ready())
                self.active=False;self.phase='failed';self.error=reason
                self.permit.publish(Bool(data=False))
                self.get_logger().error(f'{reason}: kind={self.goal_request_kind} id={self.goal_request_id}')
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
                            elif math.hypot(p['x']-limits['origin'][0],p['y']-limits['origin'][1])>limits['radius']-.65:reason='接近自主探测活动距离限制'
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
            self.gimbal_deadline=0.
            self.path=[];self.track=[];self.goal=None;self.grid=None;self.grid_view=None;self.feedback={};self.last_path=0.
            self.quality=None;self.quality_received=0.;self.mission_limits=None
            self.stop_diagnostics=None
            self.goal_request_future=None;self.goal_request_id='';self.goal_request_kind='';self.goal_sent_wall=0.
            self.grid_received=0.;self.depth_message='等待新实验传感器'
            self.start_checked=False
            self.listener.unregister()
            self.buffer.jump_handle.unregister();self.destroy_service(self.buffer.srv)
            self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self)
            self.client.destroy();self.spin_client.destroy()
            self.client=ActionClient(self,NavigateToPose,'/navigate_to_pose')
            self.spin_client=ActionClient(self,Spin,'/spin')
