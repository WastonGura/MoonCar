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

LEGACY_STRATEGIES = ('nearest_view_v1', 'coverage_view_v1', 'surface_view_v1')
V2_STRATEGIES = ('nearest_view_v2', 'coverage_view_v2', 'surface_view_v2')
V3_STRATEGIES = ('nearest_view_v3', 'coverage_view_v3', 'surface_view_v3')
STRATEGIES = (*V2_STRATEGIES, *V3_STRATEGIES)
EXECUTION_MARGIN_SECONDS = 4.
INACTIVE_POSE_GRACE_SECONDS = 1.


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


def choose_view(grid,quality,pose,origin,radius,history,remaining,strategy='quality_view_heuristic_v1',return_diagnostics=False,
                _boundary_fallback=False):
    """Known-space reachability plus conservative planar camera visibility prediction."""
    if strategy not in (*LEGACY_STRATEGIES,*STRATEGIES,'quality_view_heuristic_v1'):raise ValueError('Unknown survey strategy')
    planner_v3=strategy in V3_STRATEGIES
    planner_v2=strategy in (*V2_STRATEGIES,*V3_STRATEGIES)
    objective=strategy.replace('_v3','_v1').replace('_v2','_v1')
    diagnostics=dict(planner_version='budgeted-view-v3.0' if planner_v3 else ('budgeted-view-v2.1' if planner_v2 else 'budgeted-view-v1'),
                     reachable_move_positions=0,candidate_orientations=0,history_rejected=0,
                     low_gain_rejected=0,time_rejected=0,feasible_turns=0,feasible_moves=0,
                     positive_gain_over_budget=0,movement_reservation_applied=False,budget_limited=False)
    def result(value):return (value,diagnostics) if return_diagnostics else value
    info=grid.info;resolution=info.resolution;ox,oy=info.origin.position.x,info.origin.position.y
    costs=np.asarray(grid.data,dtype=np.int16).reshape(info.height,info.width)
    def index(x,y):return int(math.floor((x-ox)/resolution)),int(math.floor((y-oy)/resolution))
    def inside(x,y):return 0<=x<info.width and 0<=y<info.height
    sx,sy=index(pose['x'],pose['y'])
    if not inside(sx,sy):return result(None)
    # A bounded Dijkstra estimates travel cost; Nav2 still validates the actual footprint/path.
    distances={(sx,sy):0.};parents={(sx,sy):None};queue=[(0.,sx,sy)]
    while queue:
        distance,x,y=heapq.heappop(queue)
        if distance!=distances[(x,y)]:continue
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx,ny=x+dx,y+dy
            if not inside(nx,ny):continue
            px,py=ox+(nx+.5)*resolution,oy+(ny+.5)*resolution
            dist_to_robot=math.hypot(px-pose['x'],py-pose['y'])
            max_cost=100 if dist_to_robot<=.65 else 99
            if not 0<=costs[ny,nx]<max_cost:continue
            if abs(px)>24.3 or abs(py)>24.3 or math.hypot(px-origin[0],py-origin[1])>radius-.75:continue
            if planner_v3:
                terrain_weight = 1.0 + 2.5 * ((max(0, int(costs[ny, nx])) / 100.0) ** 2)
                nd = distance + resolution * terrain_weight
            else:
                nd = distance + resolution * (1 + costs[ny, nx] / 100)
            if nd<=(12 if _boundary_fallback else 6) and nd<distances.get((nx,ny),math.inf):
                distances[(nx,ny)]=nd;parents[(nx,ny)]=(x,y);heapq.heappush(queue,(nd,nx,ny))
    qr=quality['resolution'];cells=scoped_cells(quality,origin,radius)
    observed={k:c['quality'] for k,c in cells.items()}
    elevations={k:c['z'] for k,c in cells.items() if c.get('z') is not None}
    gradients={k:(c['gx'],c['gy']) for k,c in cells.items() if c.get('gx') is not None and c.get('gy') is not None}
    positions=[(pose['x'],pose['y'],0.,[(pose['x'],pose['y'])])]
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
            if np.any((patch[disk]<0)|(patch[disk]>=99)):continue
            path=[];key=(ix,iy)
            while key is not None:
                path.append((ox+(key[0]+.5)*resolution,oy+(key[1]+.5)*resolution));key=parents[key]
            positions.append((x,y,distances[(ix,iy)],list(reversed(path))))
    defect_clusters=[]
    if strategy=='surface_view_v3':
        defect_cells=[
            ((k[0]+.5)*qr,(k[1]+.5)*qr,k)
            for k,c in cells.items()
            if c.get('fit_state') in ('too_few_voxels','insufficient_xy_spread')
        ]
        for wx,wy,k in defect_cells:
            merged=False
            for cl in defect_clusters:
                if math.hypot(wx-cl['cx'],wy-cl['cy'])<1.5:
                    cl['pts'].append((wx,wy,k))
                    cl['cx']=sum(p[0] for p in cl['pts'])/len(cl['pts'])
                    cl['cy']=sum(p[1] for p in cl['pts'])/len(cl['pts'])
                    merged=True
                    break
            if not merged:
                defect_clusters.append({'cx':wx,'cy':wy,'pts':[(wx,wy,k)]})
        defect_clusters.sort(key=lambda cl:len(cl['pts']),reverse=True)
        for cl in defect_clusters[:6]:
            cx_def,cy_def=cl['cx'],cl['cy']
            cluster_grads=[gradients[k] for _,_,k in cl['pts'] if k in gradients]
            standoff_dirs=[]
            if cluster_grads:
                avg_gx=sum(g[0] for g in cluster_grads)/len(cluster_grads)
                avg_gy=sum(g[1] for g in cluster_grads)/len(cluster_grads)
                if math.hypot(avg_gx,avg_gy)>=0.05:
                    standoff_dirs.append(math.atan2(-avg_gy,-avg_gx))
                    standoff_dirs.append(math.atan2(avg_gy,avg_gx))
            standoff_dirs.append(math.atan2(pose['y']-cy_def,pose['x']-cx_def))
            for sdir in standoff_dirs:
                for ds in (1.5,2.0):
                    sx_pos=cx_def+ds*math.cos(sdir)
                    sy_pos=cy_def+ds*math.sin(sdir)
                    six,siy=index(sx_pos,sy_pos)
                    if (six,siy) not in distances or costs[siy,six]>65:continue
                    n=math.ceil(.65/resolution)
                    if six<n or siy<n or six+n>=info.width or siy+n>=info.height:continue
                    patch=costs[siy-n:siy+n+1,six-n:six+n+1]
                    yy,xx=np.ogrid[-n:n+1,-n:n+1];disk=xx*xx+yy*yy<=n*n
                    if np.any((patch[disk]<0)|(patch[disk]>=99)):continue
                    if any(math.hypot(sx_pos-p[0],sy_pos-p[1])<0.3 for p in positions):continue
                    path=[];key=(six,siy)
                    while key is not None:
                        path.append((ox+(key[0]+.5)*resolution,oy+(key[1]+.5)*resolution));key=parents[key]
                    positions.append((sx_pos,sy_pos,distances[(six,siy)],list(reversed(path))))
    if _boundary_fallback:
        positions=[]
        stride=max(1,round(.3/resolution))
        for (ix,iy),distance in distances.items():
            if (distance<.75 or ix%stride or iy%stride or costs[iy,ix]>65 or
                    min(ix,iy,info.width-1-ix,info.height-1-iy)*resolution>3.):continue
            n=math.ceil(.65/resolution)
            if ix<n or iy<n or ix+n>=info.width or iy+n>=info.height:continue
            patch=costs[iy-n:iy+n+1,ix-n:ix+n+1]
            yy,xx=np.ogrid[-n:n+1,-n:n+1];disk=xx*xx+yy*yy<=n*n
            if np.any((patch[disk]<0)|(patch[disk]>=99)):continue
            path=[];key=(ix,iy)
            while key is not None:
                path.append((ox+(key[0]+.5)*resolution,oy+(key[1]+.5)*resolution));key=parents[key]
            positions.append((ox+(ix+.5)*resolution,oy+(iy+.5)*resolution,distance,list(reversed(path))))
    diagnostics['reachable_move_positions']=len(positions)-(0 if _boundary_fallback else 1)
    def is_grazing(key, ray_angle):
        if not planner_v3:
            return False
        grad = gradients.get(key)
        if grad is None:
            return False
        gx, gy = grad
        s = math.hypot(gx, gy)
        if s < 0.20:
            return False
        cos_align = abs(math.cos(ray_angle) * gx + math.sin(ray_angle) * gy) / s
        return cos_align < 0.25
    def visibility(x,y,yaw,tilt=-0.2618):
        visible=set();cx,cy=x+.435*math.cos(yaw),y+.435*math.sin(yaw)
        if planner_v3 and elevations:
            robot_key=(math.floor(x/qr),math.floor(y/qr))
            z_robot=elevations.get(robot_key,float(pose.get('z',0.0)) if isinstance(pose,dict) else 0.0)
            cz=z_robot+0.35
            half_vfov=0.5061
            s_min=math.tan(tilt-half_vfov)
            s_max=math.tan(tilt+half_vfov)
            for angle in np.linspace(yaw-math.pi/6,yaw+math.pi/6,17):
                max_slope=s_min
                for r in np.arange(.25,5.51,.25):
                    px,py=cx+r*math.cos(angle),cy+r*math.sin(angle)
                    if math.hypot(px-origin[0],py-origin[1])>radius or abs(px)>25 or abs(py)>25:break
                    ix,iy=index(px,py)
                    if not inside(ix,iy):
                        if _boundary_fallback and r>=.6:
                            key=(math.floor(px/qr),math.floor(py/qr))
                            if math.hypot((key[0]+.5)*qr-origin[0],(key[1]+.5)*qr-origin[1])<=radius:
                                visible.add(key)
                        break
                    cost=costs[iy,ix];key=(math.floor(px/qr),math.floor(py/qr))
                    in_roi=math.hypot((key[0]+.5)*qr-origin[0],(key[1]+.5)*qr-origin[1])<=radius
                    zt=elevations.get(key,z_robot)
                    slope=(zt-cz)/r
                    if slope<max_slope:
                        if cost<0 or cost>=100:break
                        continue
                    if slope<=s_max:
                        if r>=.6 and in_roi and not is_grazing(key, angle):
                            visible.add(key)
                        max_slope=slope
                    else:
                        if r>=.6 and in_roi and not is_grazing(key, angle):
                            visible.add(key)
                        break
                    if cost<0 or cost>=100:break
        else:
            for angle in np.linspace(yaw-math.pi/6,yaw+math.pi/6,17):
                for r in np.arange(.25,5.51,.25):
                    px,py=cx+r*math.cos(angle),cy+r*math.sin(angle)
                    if math.hypot(px-origin[0],py-origin[1])>radius or abs(px)>25 or abs(py)>25:break
                    ix,iy=index(px,py)
                    if not inside(ix,iy):
                        if _boundary_fallback and r>=.6:
                            key=(math.floor(px/qr),math.floor(py/qr))
                            if math.hypot((key[0]+.5)*qr-origin[0],(key[1]+.5)*qr-origin[1])<=radius:
                                visible.add(key)
                        break
                    cost=costs[iy,ix];key=(math.floor(px/qr),math.floor(py/qr))
                    if r>=.6 and math.hypot((key[0]+.5)*qr-origin[0],(key[1]+.5)*qr-origin[1])<=radius:
                        if not is_grazing(key, angle):
                            visible.add(key)
                    if cost<0 or cost>=100:break
        return visible,(cx,cy)
    candidates=[]
    for x,y,distance,path in positions:
        route_visible=set();route_cameras={}
        if planner_v2 and distance:
            step=max(1,round(.75/resolution));sampled=path[::step]
            if sampled[-1]!=path[-1]:sampled.append(path[-1])
            for i,(px,py) in enumerate(sampled[:-1]):
                nx,ny=sampled[i+1];seen,camera=visibility(px,py,math.atan2(ny-py,nx-px))
                route_visible.update(seen)
                for q in seen:route_cameras.setdefault(q,camera)
        target_yaws=list(np.arange(-math.pi,math.pi,math.pi/4))
        if planner_v3 and strategy=='surface_view_v3' and defect_clusters:
            for cl in defect_clusters:
                if 0.8<=math.hypot(cl['cx']-x,cl['cy']-y)<=3.0:
                    facing=round(wrap(math.atan2(cl['cy']-y,cl['cx']-x)),4)
                    if not any(abs(wrap(facing-y_))<0.15 for y_ in target_yaws):
                        target_yaws.append(facing)
        for yaw in target_yaws:
            diagnostics['candidate_orientations']+=1
            if any(math.hypot(x-h['x'],y-h['y'])<.7 and
                   (h.get('blocked') or abs(wrap(yaw-h['yaw']))<.6) for h in history[-80:]):
                diagnostics['history_rejected']+=1;continue
            bearing=math.atan2(y-pose['y'],x-pose['x']) if distance else pose['yaw']
            if planner_v3 and not distance:
                pan=float(wrap(yaw-pose['yaw']))
                tilt=-0.2618
                if abs(pan)<=2.35619:
                    seconds=abs(pan)/1.5+2.0
                    use_pan_tilt=True
                else:
                    seconds=abs(pan)/.5+5.0
                    use_pan_tilt=False
            else:
                seconds=distance/.20+(abs(wrap(bearing-pose['yaw']))+abs(wrap(yaw-bearing)))/.5+5
                pan=float(wrap(yaw-pose['yaw'])) if not distance else 0.
                tilt=-0.2618
                use_pan_tilt=False
            endpoint_visible,camera=visibility(x,y,yaw,tilt=tilt);visible=set(endpoint_visible);cameras={q:camera for q in visible}
            visible.update(route_visible)
            for q,route_camera in route_cameras.items():cameras.setdefault(q,route_camera)
            new=sum(q not in observed for q in visible)*qr*qr
            improve=0.
            if objective=='quality_view_heuristic_v1':
                improve=sum((1-observed[q])*.4 for q in visible if q in observed)*qr*qr
            elif objective=='surface_view_v1':
                improve=sum(resampling_value(cells[q],q,qr,cameras[q]) for q in visible if q in cells)*qr*qr
                if planner_v3:
                    improve*=2.0
            utility=new+improve
            if utility<.15:
                diagnostics['low_gain_rejected']+=1;continue
            kind='turn' if distance<.1 else ('resample' if improve>new else 'advance')
            score=1/seconds if objective=='nearest_view_v1' else utility/seconds
            candidate=dict(x=round(x,3),y=round(y,3),yaw=float(yaw),kind=kind,
                           pan=round(pan,3),tilt=round(tilt,3),use_pan_tilt=use_pan_tilt,
                           expected_new_m2=round(new,3),expected_improvement_m2=round(improve,3),
                           expected_utility_m2=round(utility,3),visible_cells=sorted(visible),strategy=strategy,
                           estimated_seconds=round(seconds,1),score=float(score))
            if planner_v2 and distance:
                candidate['enroute_visible_cells']=len(visible-endpoint_visible)
            if seconds>remaining:
                diagnostics['time_rejected']+=1;diagnostics['positive_gain_over_budget']+=1;continue
            diagnostics['feasible_turns' if kind=='turn' else 'feasible_moves']+=1
            candidates.append(candidate)
    diagnostics['budget_limited']=not candidates and diagnostics['positive_gain_over_budget']>0
    best=max(candidates,key=lambda c:c['score'],default=None)
    task_extends_map=(origin[0]-radius<ox or origin[0]+radius>ox+info.width*resolution or
                      origin[1]-radius<oy or origin[1]+radius>oy+info.height*resolution)
    if best is None and planner_v2 and not _boundary_fallback and not diagnostics['budget_limited'] and task_extends_map:
        fallback,details=choose_view(grid,quality,pose,origin,radius,history,remaining,strategy,
                                     return_diagnostics=True,_boundary_fallback=True)
        diagnostics['map_boundary_fallback']=details
        if fallback is not None:fallback['map_boundary_fallback']=True
        return result(fallback)
    moves=[c for c in candidates if c['kind']!='turn']
    if planner_v2 and best is not None and best['kind']=='turn' and moves:
        fastest=min(moves,key=lambda c:c['estimated_seconds'])
        if best['estimated_seconds']+fastest['estimated_seconds']>remaining:
            best=max(moves,key=lambda c:c['score']);diagnostics['movement_reservation_applied']=True
    if best is not None and objective=='quality_view_heuristic_v1':
        best['expected_quality_gain_m2']=best['expected_utility_m2']
    return result(best)


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
        strategy=body.get('strategy','surface_view_v2')
        if strategy not in (*LEGACY_STRATEGIES,*STRATEGIES):raise ValueError('请选择就近观察、覆盖优先或地表补采策略')
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
                pose=state['pose'];self.motion.check_start(task,pose)
                self.origin=[pose['x'],pose['y']]
                started=self.now()
                self.motion.mission_limits=dict(origin=self.origin,radius=radius,deadline=started+budget)
            self.task=dict(task,region=[-25,-25,25,25])
            self.path=Path(directory)/task['id']/f'autonomy-{strategy}-{uuid.uuid4().hex[:8]}.json'
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
                self.motion.cancel(reason);self.motion.mission_limits=None
                if phase=='fault' and 'stop_diagnostics' not in self.data:
                    try:
                        state=self.motion.state()
                        self.data['stop_diagnostics']=dict(reason=reason,health=state.get('diagnostics'),motion_stop=state.get('stop_diagnostics'))
                    except Exception as error:self.data['stop_diagnostics']=dict(reason=reason,diagnostic_error=str(error))
                self.save()
        return dict(accepted=True)

    def healthy(self):
        if not self.data['active']:return False
        if self.now()-self.data['started']>=self.data['budget_seconds']:
            self.stop('时间预算已用尽，已停止运动并保留采集档案','budget_reached');return False
        if time.monotonic()-self.wall_started>3600:
            self.stop('达到一小时运行上限','budget_reached');return False
        state=self.motion.state()
        if (not state.get('active') and state.get('pose') is None and
                '车辆定位时间失效' in state.get('diagnostics',{}).get('pose_error','')):
            # A completed/cancelled action has no motion permit; allow a brief TF update,
            # but never tolerate a stale pose while the rover is moving.
            deadline=time.monotonic()+INACTIVE_POSE_GRACE_SECONDS
            while self.data['active'] and time.monotonic()<deadline and self.now()-self.data['started']<self.data['budget_seconds']:
                time.sleep(.1)
                if self.motion.state().get('pose') is not None:break
            if self.now()-self.data['started']>=self.data['budget_seconds']:
                self.stop('时间预算已用尽，已停止运动并保留采集档案','budget_reached');return False
        with self.motion.lock:
            state=self.motion.state()
            if not self.motion.grid_fresh():
                reason,phase='导航地图失效，已停止','fault'
            elif not state['ready'] or self.motion.quality is None or time.monotonic()-self.motion.quality_received>12:
                failed=[]
                if state.get('pose') is None:failed.append('车辆定位不可用')
                if state.get('depth_active') is False:failed.append('深度状态超过 3 秒未更新')
                if state.get('diagnostics',{}).get('navigation_ready') is False:failed.append('Nav2 服务不可用')
                if self.motion.quality is None:failed.append('尚无采样质量数据')
                elif time.monotonic()-self.motion.quality_received>12:failed.append('采样质量超过 12 秒未更新')
                reason,phase='；'.join(failed or ['导航就绪条件失效'])+'，已停止','fault'
            elif math.hypot(state['pose']['x']-self.origin[0],state['pose']['y']-self.origin[1])>self.data['radius']-.65:
                reason,phase='已接近活动距离限制','distance_reached'
            else:reason=''
        if reason:
            self.data['stop_diagnostics']=dict(reason=reason,health=state.get('diagnostics'),motion_stop=state.get('stop_diagnostics'))
            self.stop(reason,phase);return False
        return True

    def has_action_budget(self,decision):
        remaining=self.data['budget_seconds']-(self.now()-self.data['started'])
        return decision['estimated_seconds']+EXECUTION_MARGIN_SECONDS<=remaining

    def execute(self,decision,turn=False):
        with self.lock:
            if not self.healthy():return False
            try:
                if turn and decision.get('use_pan_tilt') and hasattr(self.motion,'pan_tilt'):
                    response=self.motion.pan_tilt(decision['pan'],decision.get('tilt',-0.2618),self.task)
                elif turn:
                    response=self.motion.turn(decision['yaw'],self.task)
                else:
                    response=self.motion.start(
                        dict(experiment=self.task['id'],goal=[decision['x'],decision['y']]),self.task)
            except ValueError as error:
                if turn or not str(error).startswith(('目标位于障碍','目标尚未观测','目标超出当前导航地图边界')):
                    raise
                decision['dispatch_error']=str(error)
                return False
            token=response['token'];self.data['phase']='gimbal_scanning' if (turn and decision.get('use_pan_tilt')) else ('turning' if turn else 'moving');self.save()
        while self.healthy():
            self.motion.heartbeat(dict(token=token))
            state=self.motion.state()
            if not state['active']:
                if state['phase']=='cancelled' and not state.get('replan_allowed'):
                    self.stop(state['error'] or '运动已取消','fault')
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
                decision,diagnostics=choose_view(grid,quality,pose,self.origin,self.data['radius'],history,
                                     self.data['budget_seconds']-(self.now()-self.data['started'])-EXECUTION_MARGIN_SECONDS,
                                     self.data.get('strategy','quality_view_heuristic_v1'),return_diagnostics=True)
                if not decision:
                    self.data['candidate_diagnostics']=diagnostics
                    if diagnostics['budget_limited']:
                        self.stop('剩余时间不足以完成下一候选观察动作','budget_reached')
                    else:self.stop('没有新的安全候选观察位置；不代表整个环境已探测完毕','no_candidates')
                    break
                if not self.has_action_budget(decision):
                    diagnostics['execution_budget_recheck_rejected']=True
                    self.data['candidate_diagnostics']=diagnostics
                    self.stop('候选计算后剩余时间不足以完成下一观察动作','budget_reached');break
                decision['candidate_diagnostics']=diagnostics
                names={'turn':'云台扫描' if decision.get('use_pan_tilt') else '转向观察','advance':'前进探索','resample':'换位补采'}
                with self.lock:
                    if not self.healthy():break
                    self.data.update(decision=decision,phase='planning',reason=f"{names[decision['kind']]}：预计新增 {decision['expected_new_m2']:.2f} m²，补采收益 {decision['expected_improvement_m2']:.2f} m²；预计 {decision['estimated_seconds']:.0f} 仿真秒")
                    self.save()
                history.append(decision);success=True
                record=dict(decision,success=False,started_seconds=self.now()-self.data['started'],
                            outcome='running',metrics_version=VERSION)
                with self.lock:self.data['history'].append(record);self.save()
                if not self.has_action_budget(decision):
                    history.pop()
                    with self.lock:
                        self.data['history'].pop();self.data['decision']=None
                        diagnostics['execution_budget_recheck_rejected']=True
                        self.data['candidate_diagnostics']=diagnostics
                    self.stop('写入动作记录后剩余时间不足以完成下一观察动作','budget_reached');break
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
                    motion_state=self.motion.state()
                    record.update(success=success and self.data['active'],outcome=('succeeded' if success else 'failed') if self.data['active'] else 'interrupted',
                                  actual_quality_gain_m2=round(roi_gain['quality_gain_m2'],3),roi_gain=roi_gain,
                                  predicted_view_gain=view_gain,
                                  actual_view_utility_m2=utility,
                                  prediction_residual_m2=(view_gain['quality_gain_m2']-decision['expected_quality_gain_m2'] if 'expected_quality_gain_m2' in decision else
                                                         utility-decision['expected_utility_m2'] if utility is not None else None),
                                  elapsed_seconds=round(self.now()-self.data['started'],1),
                                  motion_error=decision.get('dispatch_error',motion_state['error']),
                                  motion_stop=None if 'dispatch_error' in decision else motion_state.get('stop_diagnostics'),
                                  health=motion_state.get('diagnostics'))
                    self.save()
                if not self.data['active']:break
                if not success and decision['kind']!='turn':
                    decision['blocked']=True
                    record['retry_blocked']=True
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
