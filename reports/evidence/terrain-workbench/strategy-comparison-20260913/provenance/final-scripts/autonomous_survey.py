"""Budgeted observation planning. Inputs are sensor maps; no Gazebo world access."""
import heapq
import json
import math
from pathlib import Path
import threading
import time
import uuid

import numpy as np
from survey_evaluation import VERSION, EvidenceRecorder, gain, scoped_cells

STRATEGIES = ('nearest_view_v1', 'coverage_view_v1', 'surface_view_v1')


def resampling_value(cell, key, resolution, camera):
    """Sensor-only heuristic, not a calibrated probability of recovering a surface."""
    if cell.get('fit_state') not in ('too_few_voxels', 'insufficient_xy_spread'):
        return 0.
    angle=math.atan2(camera[1]-(key[1]+.5)*resolution,camera[0]-(key[0]+.5)*resolution)
    bearing=int((angle+math.pi)/(math.pi/4))%8
    if cell.get('bearing_mask',255)&(1<<bearing):return 0.
    # ponytail: one new bearing receives a fixed half-cell recovery utility; calibrate
    # from held-out resampling trials before interpreting this as expected area.
    return .5


def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


def choose_view(grid,quality,pose,origin,radius,history,remaining,strategy='quality_view_heuristic_v1'):
    """Known-space reachability plus conservative planar camera visibility prediction."""
    if strategy not in (*STRATEGIES,'quality_view_heuristic_v1'):raise ValueError('Unknown survey strategy')
    info=grid.info;resolution=info.resolution;ox,oy=info.origin.position.x,info.origin.position.y
    costs=np.asarray(grid.data,dtype=np.int16).reshape(info.height,info.width)
    def index(x,y):return int(math.floor((x-ox)/resolution)),int(math.floor((y-oy)/resolution))
    def inside(x,y):return 0<=x<info.width and 0<=y<info.height
    sx,sy=index(pose['x'],pose['y'])
    if not inside(sx,sy):return None
    # A bounded Dijkstra estimates travel cost; Nav2 still validates the actual footprint/path.
    distances={(sx,sy):0.};queue=[(0.,sx,sy)]
    while queue:
        distance,x,y=heapq.heappop(queue)
        if distance!=distances[(x,y)]:continue
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx,ny=x+dx,y+dy
            if not inside(nx,ny) or not 0<=costs[ny,nx]<99:continue
            px,py=ox+(nx+.5)*resolution,oy+(ny+.5)*resolution
            if abs(px)>24.3 or abs(py)>24.3 or math.hypot(px-origin[0],py-origin[1])>radius-.75:continue
            nd=distance+resolution*(1+costs[ny,nx]/100)
            if nd<=6 and nd<distances.get((nx,ny),math.inf):
                distances[(nx,ny)]=nd;heapq.heappush(queue,(nd,nx,ny))
    positions=[(pose['x'],pose['y'],0.)]
    for distance in (1.5,3.):
        for angle in np.arange(0,2*math.pi,math.pi/4):
            x,y=pose['x']+distance*math.cos(angle),pose['y']+distance*math.sin(angle)
            ix,iy=index(x,y)
            if (ix,iy) not in distances or costs[iy,ix]>65:continue
            # Circumscribed chassis clearance allows subsequent stationary camera turns.
            n=math.ceil(.65/resolution)
            if ix<n or iy<n or ix+n>=info.width or iy+n>=info.height:continue
            patch=costs[iy-n:iy+n+1,ix-n:ix+n+1]
            yy,xx=np.ogrid[-n:n+1,-n:n+1];disk=xx*xx+yy*yy<=n*n
            if np.any((patch[disk]<0)|(patch[disk]>=100)):continue
            positions.append((x,y,distances[(ix,iy)]))
    qr=quality['resolution'];cells=scoped_cells(quality,origin,radius)
    observed={k:c['quality'] for k,c in cells.items()}
    best=None
    for x,y,distance in positions:
        for yaw in np.arange(-math.pi,math.pi,math.pi/4):
            if any(math.hypot(x-h['x'],y-h['y'])<.7 and abs(wrap(yaw-h['yaw']))<.6 for h in history[-80:]):continue
            bearing=math.atan2(y-pose['y'],x-pose['x']) if distance else pose['yaw']
            seconds=distance/.20+(abs(wrap(bearing-pose['yaw']))+abs(wrap(yaw-bearing)))/.5+5
            if seconds>remaining:continue
            visible=set()
            for angle in np.linspace(yaw-math.pi/6,yaw+math.pi/6,17):
                # ponytail: planar occlusion and nominal ground view; upgrade to 3D ray casting
                # if predicted-vs-observed gains show systematic errors around tall obstacles.
                cx,cy=x+.435*math.cos(yaw),y+.435*math.sin(yaw)
                for r in np.arange(.25,5.51,.25):
                    px,py=cx+r*math.cos(angle),cy+r*math.sin(angle)
                    if math.hypot(px-origin[0],py-origin[1])>radius or abs(px)>25 or abs(py)>25:break
                    ix,iy=index(px,py)
                    if not inside(ix,iy):break
                    cost=costs[iy,ix]
                    key=(math.floor(px/qr),math.floor(py/qr))
                    if r>=.6 and math.hypot((key[0]+.5)*qr-origin[0],(key[1]+.5)*qr-origin[1])<=radius:
                        visible.add(key)
                    if cost<0 or cost>=100:break
            new=sum(q not in observed for q in visible)*qr*qr
            improve=0.
            if strategy=='quality_view_heuristic_v1':
                improve=sum((1-observed[q])*.4 for q in visible if q in observed)*qr*qr
            elif strategy=='surface_view_v1':
                improve=sum(resampling_value(cells[q],q,qr,(cx,cy)) for q in visible if q in cells)*qr*qr
            gain=new+improve
            score=1/seconds if strategy=='nearest_view_v1' else gain/seconds
            if gain<.15:continue
            kind='turn' if distance<.1 else ('resample' if improve>new else 'advance')
            if best is None or score>best['score']:
                best=dict(x=round(x,3),y=round(y,3),yaw=float(yaw),kind=kind,
                          expected_new_m2=round(new,3),expected_improvement_m2=round(improve,3),
                          expected_utility_m2=round(gain,3),visible_cells=sorted(visible),strategy=strategy,
                          estimated_seconds=round(seconds,1),score=float(score))
    if best is not None and strategy=='quality_view_heuristic_v1':
        best['expected_quality_gain_m2']=best['expected_utility_m2']
    return best


class AutonomousSurvey:
    def __init__(self,motion):
        self.motion=motion;self.lock=threading.RLock();self.worker=None
        self.data=dict(active=False,phase='idle',reason='尚未开始自主探测',history=[])

    def now(self):return self.motion.get_clock().now().nanoseconds/1e9

    def state(self):
        with self.lock:
            result=dict(self.data)
            if 'started' in result:
                result['elapsed_seconds']=round(max(0,self.now()-result['started']),1) if result['active'] else result.get('elapsed_seconds',0)
                result['remaining_seconds']=round(max(0,result['budget_seconds']-result['elapsed_seconds']),1)
            return result

    def save(self):
        if hasattr(self,'path'):
            temporary=self.path.with_suffix('.tmp')
            temporary.write_text(json.dumps(self.state(),ensure_ascii=False,indent=2),encoding='utf-8')
            temporary.replace(self.path)

    def start(self,body,task,directory):
        if not isinstance(body,dict) or set(body) not in ({'experiment','budget_seconds','radius'}, {'experiment','budget_seconds','radius','strategy'}) or body['experiment']!=task['id']:
            raise ValueError('实验已改变，请刷新后启动自主探测')
        strategy=body.get('strategy','surface_view_v1')
        if strategy not in STRATEGIES:raise ValueError('请选择就近观察、覆盖优先或地表补采策略')
        budget,radius=body['budget_seconds'],body['radius']
        if not all(type(v) in (int,float) and math.isfinite(v) for v in (budget,radius)) or not 15<=budget<=1800 or not 2<=radius<=25:
            raise ValueError('时间预算为 15～1800 仿真秒，活动距离为 2～25 米')
        with self.lock:
            if self.data['active'] or self.worker and self.worker.is_alive():raise ValueError('自主探测尚未结束')
            with self.motion.lock:
                state=self.motion.state()
                if state['active'] or not state['ready'] or self.motion.quality is None or time.monotonic()-self.motion.quality_received>10:
                    raise ValueError('请等待车辆静止、定位与采样质量数据就绪')
                if not self.motion.spin_client.server_is_ready():raise ValueError('转向服务尚未就绪')
                pose=state['pose'];self.origin=[pose['x'],pose['y']]
                started=self.now()
                self.motion.mission_limits=dict(origin=self.origin,radius=radius,deadline=started+budget)
            self.task=dict(task,region=[-25,-25,25,25])
            self.path=Path(directory)/task['id']/f'autonomy-{uuid.uuid4().hex[:10]}.json'
            self.data=dict(active=True,phase='planning',reason='根据当前观测比较转向、前进与换位补采',
                           origin=self.origin,budget_seconds=budget,radius=radius,started=started,
                           elapsed_seconds=0,history=[],decision=None,archive=self.path.name,strategy=strategy)
            self.data['metrics_version']=VERSION
            self.recorder=None
            self.save();self.worker=threading.Thread(target=self.run_recorded,args=(directory,),daemon=True);self.worker.start()
        return dict(accepted=True)

    def run_recorded(self,directory):
        # Source snapshots can take seconds on the Windows bind mount. Acknowledge
        # the command first, then prepare recording before issuing any motion.
        try:
            self.recorder=EvidenceRecorder(self)
            with self.lock:
                self.data['evidence_folder']=str(self.recorder.folder.relative_to(Path(directory)))
                self.save()
            self.run()
        except Exception as error:
            self.data['evidence_error']=str(error)
            self.stop(f'实验记录初始化失败，未启动运动：{error}','fault')

    def stop(self,reason='已手动停止自主探测',phase='stopped'):
        with self.lock:
            if self.data['active']:
                self.data.update(active=False,phase=phase,reason=reason,elapsed_seconds=round(max(0,self.now()-self.data['started']),1))
                self.motion.cancel(reason);self.motion.mission_limits=None;self.save()
        return dict(accepted=True)

    def healthy(self):
        if not self.data['active']:return False
        if self.now()-self.data['started']>=self.data['budget_seconds']:
            self.stop('时间预算已用尽，已停止运动并保留采集档案','budget_reached');return False
        if time.monotonic()-self.wall_started>3600:
            self.stop('达到一小时运行上限','budget_reached');return False
        with self.motion.lock:
            state=self.motion.state()
            if not self.motion.grid_fresh():
                reason,phase='导航地图失效，已停止','fault'
            elif not state['ready'] or self.motion.quality is None or time.monotonic()-self.motion.quality_received>12:
                reason,phase='定位、传感器或质量数据失效，已停止','fault'
            elif math.hypot(state['pose']['x']-self.origin[0],state['pose']['y']-self.origin[1])>self.data['radius']-.65:
                reason,phase='已接近活动距离限制','distance_reached'
            else:reason=''
        if reason:self.stop(reason,phase);return False
        return True

    def execute(self,decision,turn=False):
        with self.lock:
            if not self.healthy():return False
            response=self.motion.turn(decision['yaw'],self.task) if turn else self.motion.start(
                dict(experiment=self.task['id'],goal=[decision['x'],decision['y']]),self.task)
            token=response['token'];self.data['phase']='turning' if turn else 'moving';self.save()
        while self.healthy():
            self.motion.heartbeat(dict(token=token))
            state=self.motion.state()
            if not state['active']:
                if state['phase']=='cancelled':self.stop(state['error'] or '运动已取消','fault')
                return state['phase']=='succeeded'
            time.sleep(.15)
        return False

    def run(self):
        self.wall_started=time.monotonic();history=[];failures=0
        try:
            while self.healthy():
                with self.motion.lock:
                    grid=self.motion.grid;quality=self.motion.quality;pose=self.motion.pose()
                    if not self.motion.grid_fresh():
                        raise ValueError('导航地图未更新')
                before=quality
                decision=choose_view(grid,quality,pose,self.origin,self.data['radius'],history,
                                     self.data['budget_seconds']-(self.now()-self.data['started']),
                                     self.data.get('strategy','quality_view_heuristic_v1'))
                if not decision:
                    self.stop('当前预算内没有新的安全候选观察位置；不代表整个环境已探测完毕','no_candidates');break
                names={'turn':'转向观察','advance':'前进探索','resample':'换位补采'}
                with self.lock:
                    if not self.healthy():break
                    self.data.update(decision=decision,phase='planning',reason=f"{names[decision['kind']]}：预计新增 {decision['expected_new_m2']:.2f} m²，补采收益 {decision['expected_improvement_m2']:.2f} m²；预计 {decision['estimated_seconds']:.0f} 仿真秒")
                    self.save()
                history.append(decision);success=True
                record=dict(decision,success=False,started_seconds=self.now()-self.data['started'],
                            outcome='running',metrics_version=VERSION)
                with self.lock:self.data['history'].append(record);self.save()
                if decision['kind']!='turn':success=self.execute(decision)
                if success and self.healthy():
                    angle=abs(wrap(decision['yaw']-self.motion.pose()['yaw']))
                    if angle>.12:success=self.execute(decision,turn=True)
                if success and self.healthy():
                    self.data['phase']='observing';until=self.now()+3
                    while self.healthy() and self.now()<until:time.sleep(.2)
                current=self.motion.quality
                roi_gain=gain(before,current,self.origin,self.data['radius'])
                view_gain=gain(before,current,self.origin,self.data['radius'],{tuple(k) for k in decision['visible_cells']})
                utility=view_gain['new_observed_m2']+view_gain['recovered_surface_m2'] if view_gain['recovered_surface_m2'] is not None else None
                with self.lock:
                    record.update(success=success and self.data['active'],outcome=('succeeded' if success else 'failed') if self.data['active'] else 'interrupted',
                                  actual_quality_gain_m2=round(roi_gain['quality_gain_m2'],3),roi_gain=roi_gain,
                                  predicted_view_gain=view_gain,
                                  actual_view_utility_m2=utility,
                                  prediction_residual_m2=(view_gain['quality_gain_m2']-decision['expected_quality_gain_m2'] if 'expected_quality_gain_m2' in decision else
                                                         utility-decision['expected_utility_m2'] if utility is not None else None),
                                  elapsed_seconds=round(self.now()-self.data['started'],1),motion_error=self.motion.state()['error'])
                    self.save()
                if not self.data['active']:break
                failures=0 if success else failures+1
                if failures>=3:self.stop('连续三个候选动作未能执行，已停止；请检查场地与导航','fault')
        except Exception as error:self.stop(f'自主探测已停止：{error}','fault')
        finally:
            if self.data['active']:self.stop('自主探测结束')
            with self.lock:
                if self.data['history'] and self.data['history'][-1].get('outcome')=='running':
                    self.data['history'][-1].update(outcome='interrupted',success=False,motion_error=self.data.get('reason',''))
                self.save()
            if getattr(self,'recorder',None):
                try:self.recorder.finish()
                except Exception as error:
                    self.data['evidence_error']=str(error);self.save()
