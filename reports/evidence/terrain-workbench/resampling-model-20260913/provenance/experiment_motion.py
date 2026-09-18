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
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


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
        self.lock=threading.RLock()
        self.active=False;self.goal_handle=None;self.token='';self.phase='idle';self.error=''
        self.last_heartbeat=0.;self.started=0.;self.last_depth=0.;self.depth_message='等待传感器';self.region=None
        self.path=[];self.track=[];self.goal=None;self.grid=None;self.grid_view=None;self.grid_received=0.;self.last_path=0.
        self.feedback={};self.sim_started=0.;self.sim_budget=180.
        self.quality=None;self.quality_received=0.;self.mission_limits=None
        self.stop_diagnostics=None
        self.create_subscription(String,'/mooncar/map/quality',self.on_quality,1)
        self.create_subscription(DiagnosticArray,'/mooncar/map/status',self.depth,10)
        self.create_subscription(Path,'/plan',self.on_path,10)
        self.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.on_grid,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def depth(self,msg):
        for status in msg.status:
            if status.name=='map_recorder':
                self.depth_message=status.message
                if status.level==DiagnosticStatus.OK:self.last_depth=time.monotonic()

    def on_quality(self,msg):
        try:
            value=json.loads(msg.data)
            with self.lock:self.quality=value;self.quality_received=time.monotonic()
        except (ValueError,TypeError):pass

    def turn(self,yaw,task):
        with self.lock:
            if self.active or not self.state()['ready'] or not self.spin_client.server_is_ready():
                raise ValueError('等待导航结束及转向服务就绪')
            p=self.pose();angle=math.atan2(math.sin(yaw-p['yaw']),math.cos(yaw-p['yaw']))
            # Spin keeps Nav2 collision prediction and the same safety relay as translation.
            self.region=task['region'];self.token=secrets.token_urlsafe(24);token=self.token
            self.active=True;self.phase='planning';self.error='';self.goal_handle=None
            self.stop_diagnostics=None
            self.started=self.last_heartbeat=time.monotonic()
            self.sim_started=self.get_clock().now().nanoseconds/1e9;self.sim_budget=60.
            self.goal=[p['x'],p['y']];self.path=[];self.track=[[p['x'],p['y'],p['z']]];self.feedback={}
            request=Spin.Goal();request.target_yaw=float(angle);request.time_allowance.sec=60
            future=self.spin_client.send_goal_async(request)
            future.add_done_callback(lambda f:self.accepted(f,token))
            return dict(token=token)

    def pose(self):
        t=self.buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
        age=(self.get_clock().now().nanoseconds-rclpy.time.Time.from_msg(t.header.stamp).nanoseconds)/1e9
        if not -.1 <= age <= 2:raise ValueError(f'车辆定位时间失效：TF 与仿真时钟相差 {age:.3f} 秒')
        p,q=t.transform.translation,t.transform.rotation
        return dict(x=p.x,y=p.y,z=p.z,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

    def state(self):
        with self.lock:
            pose_error=''
            try:p=self.pose()
            except Exception as error:p=None;pose_error=str(error)
            wall=time.monotonic();sim=self.get_clock().now().nanoseconds/1e9
            navigation_ready=self.client.server_is_ready();map_fresh=self.grid_fresh()
            depth_active=wall-self.last_depth<3
            age=lambda stamp:round(wall-stamp,3) if stamp else None
            diagnostics=dict(sim_stamp=sim,pose_error=pose_error,navigation_ready=navigation_ready,
                depth_age_wall_seconds=age(self.last_depth),depth_message=self.depth_message,
                quality_age_wall_seconds=age(self.quality_received),quality_present=self.quality is not None,
                costmap_age_wall_seconds=age(self.grid_received) if self.grid else None,costmap_fresh=map_fresh,
                costmap_age_sim_seconds=sim-rclpy.time.Time.from_msg(self.grid.header.stamp).nanoseconds/1e9 if self.grid else None)
            bounds=None
            if self.grid:
                i=self.grid.info;o=i.origin.position
                bounds=[o.x,o.y,o.x+i.width*i.resolution,o.y+i.height*i.resolution]
            return dict(active=self.active,phase=self.phase,error=self.error,pose=p,
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
            p=self.pose();yaw=math.atan2(goal[1]-p['y'],goal[0]-p['x'])
            if not within_region(task['region'],p['x'],p['y'],p['yaw']) or not within_region(task['region'],*goal,yaw):
                raise ValueError('起点及目标的整个车体需位于探测区域内，并留出边界余量')
            self.region=task['region'];self.token=secrets.token_urlsafe(24);token=self.token
            self.active=True;self.phase='planning';self.error='';self.goal_handle=None
            self.stop_diagnostics=None
            self.started=self.last_heartbeat=time.monotonic()
            self.sim_started=self.get_clock().now().nanoseconds/1e9
            self.sim_budget=min(900.,max(120.,math.hypot(goal[0]-p['x'],goal[1]-p['y'])/.06+90))
            self.goal=list(goal);self.path=[];self.track=[[p['x'],p['y'],p['z']]];self.feedback={}
            request=NavigateToPose.Goal();request.pose.header.frame_id='map'
            request.pose.pose.position.x,request.pose.pose.position.y=map(float,goal)
            request.pose.pose.orientation.z,request.pose.pose.orientation.w=math.sin(yaw/2),math.cos(yaw/2)
            future=self.client.send_goal_async(request,feedback_callback=lambda msg:self.on_feedback(msg,token))
            future.add_done_callback(lambda f:self.accepted(f,token))
            return dict(token=token)

    def accepted(self,future,token):
        with self.lock:
            try:
                handle=future.result()
                if token!=self.token:
                    if handle.accepted:handle.cancel_goal_async()
                    return
                if not handle.accepted:
                    self.active=False;self.phase='failed';self.error='Nav2 拒绝目标'
                    self.record_stop(self.error,action_status='rejected');return
                if not self.active:
                    handle.cancel_goal_async();return
                self.goal_handle=handle
                if self.phase=='cancelling':handle.cancel_goal_async()
                else:self.phase='executing'
                handle.get_result_async().add_done_callback(lambda f:self.finished(f,token))
            except Exception as error:
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
            return dict(accepted=True)

    def tick(self):
        with self.lock:
            if self.active and self.phase!='cancelling':
                reason=''
                if time.monotonic()-self.last_heartbeat>.9:reason='页面失联，已停止运动'
                elif self.get_clock().now().nanoseconds/1e9-self.sim_started>self.sim_budget:reason='导航超过本段仿真时间预算，已停止运动'
                elif time.monotonic()-self.started>3600:reason='导航超过一小时运行上限，已停止运动'
                elif time.monotonic()-self.last_depth>3:reason='传感器未更新，已停止运动'
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
                    except Exception:reason='车辆定位失效，已停止运动'
                if reason:self.cancel(reason)
            self.permit.publish(Bool(data=self.active and self.phase=='executing'))

    def reset(self):
        self.cancel()
        with self.lock:
            self.active=False;self.goal_handle=None;self.token='';self.phase='idle';self.error='';self.last_depth=0
            self.path=[];self.track=[];self.goal=None;self.grid=None;self.grid_view=None;self.feedback={};self.last_path=0.
            self.quality=None;self.quality_received=0.;self.mission_limits=None
            self.stop_diagnostics=None
            self.grid_received=0.;self.depth_message='等待新实验传感器'
            self.buffer.clear()
