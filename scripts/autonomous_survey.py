"""Budgeted observation planning. Inputs are sensor maps; no Gazebo world access."""
import heapq
import base64
import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import uuid

import numpy as np
from localization_revisit import RGBDRevisitPolicy, ReobservedAnchorProof, RetiredCheckpoint, MAX_REVISITS, sample_payload
from survey_evaluation import VERSION, EvidenceRecorder, gain, scoped_cells
from experiment_scenes import scene_bounds
from localization_quality import LocalizationDispatchRejected

LEGACY_STRATEGIES = ('nearest_view_v1', 'coverage_view_v1', 'surface_view_v1')
V2_STRATEGIES = ('nearest_view_v2', 'coverage_view_v2', 'surface_view_v2')
V3_STRATEGIES = ('nearest_view_v3', 'coverage_view_v3', 'surface_view_v3')
STRATEGIES = (*V2_STRATEGIES, *V3_STRATEGIES)
EXECUTION_MARGIN_SECONDS = 4.
INACTIVE_POSE_GRACE_SECONDS = 1.
MAX_CONSECUTIVE_ACTION_FAILURES = 3
MAX_RECOVERY_SCANS = 1
# Full gimbal travel at the planner's nominal rate, settling and observation.
RECOVERY_SCAN_SECONDS = 2 * 2.356 / 1.5 + 2. + 3.
EXECUTION_POLICY = 'bounded-recovery-v1'
INITIAL_DEPTH_SCAN_SECONDS = 90.
INITIAL_DEPTH_OBSERVATION_WALL_SECONDS = 20.
FRONTIER_SCANS_PER_STOP = 2
FRONTIER_OBSERVATION_SECONDS = 5.
REVISIT_POSITION_TOLERANCE_M = .05
# Operational wait limit, not a localization performance target. The original
# mission budget continues to run while parked; a dead producer cannot hang it.
LOCALIZATION_RECOVERY_WALL_SECONDS = 15.
LOCALIZATION_STOP_REASON = '激光定位质量失效，已停止本次动作'
TRANSIENT_LOCALIZATION_REASONS = frozenset(('recovering','observation_stale',
    'status_recovering','status_stale','status_missing'))


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


def known_chassis_clearance(costs,x,y,resolution):
    """Existing candidate circumscribed chassis margin, using observed cells only."""
    height,width=costs.shape;n=math.ceil(.65/resolution)
    if x<n or y<n or x+n>=width or y+n>=height:return False
    patch=costs[y-n:y+n+1,x-n:x+n+1]
    yy,xx=np.ogrid[-n:n+1,-n:n+1];disk=xx*xx+yy*yy<=n*n
    return not np.any((patch[disk]<0)|(patch[disk]>=99))


def initial_depth_connectivity(grid,pose,minimum_distance=.8):
    """Only actual known costmap cells connect the parked pose to a move target."""
    info=grid.info;r=float(info.resolution);ox,oy=info.origin.position.x,info.origin.position.y
    costs=np.asarray(grid.data,dtype=np.int16).reshape(info.height,info.width)
    sx,sy=math.floor((pose['x']-ox)/r),math.floor((pose['y']-oy)/r)
    if not (0<=sx<info.width and 0<=sy<info.height):return dict(connected=False,reason='start_outside_map',cells=0)
    if not 0<=costs[sy,sx]<99:return dict(connected=False,reason='start_not_known_passable',cells=0,start_cost=int(costs[sy,sx]))
    seen={(sx,sy)};queue=[(sx,sy)];maximum=0.
    for x,y in queue:
        distance=math.hypot(ox+(x+.5)*r-pose['x'],oy+(y+.5)*r-pose['y'])
        maximum=max(maximum,distance)
        if (distance>=minimum_distance and costs[y,x]<=65 and known_chassis_clearance(costs,x,y,r)):
            return dict(connected=True,cells=len(seen),reachable_distance_m=maximum,
                        target_cost=int(costs[y,x]),chassis_clearance_m=.65)
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx,ny=x+dx,y+dy
            if (0<=nx<info.width and 0<=ny<info.height and (nx,ny) not in seen
                    and 0<=costs[ny,nx]<99):
                seen.add((nx,ny));queue.append((nx,ny))
    return dict(connected=False,reason='no_reachable_clear_target',cells=len(seen),reachable_distance_m=maximum)


def bounded_gimbal_targets(start,waypoints):
    """Actual joint targets with at most 30 degrees of change on either axis."""
    current=list(start)
    for target in waypoints:
        count=math.ceil(max(abs(target[i]-current[i]) for i in (0,1))/(math.pi/6))
        previous=list(current)
        for step in range(1,count+1):
            yield tuple(previous[i]+(target[i]-previous[i])*step/count for i in (0,1))
        current=list(target)


def visible_unknown_frontiers(grid, pose, origin, radius):
    """Find the first unknown cell on camera rays from the parked rover."""
    radius = math.inf if radius is None else radius
    info=grid.info;r=info.resolution;ox,oy=info.origin.position.x,info.origin.position.y
    costs=np.asarray(grid.data,dtype=np.int16).reshape(info.height,info.width)
    targets={}
    for pan in (-2.356,-math.pi/2,-math.pi/4,0.,math.pi/4,math.pi/2,2.356):
        yaw=pose['yaw']+pan
        cx,cy=pose['x']+.435*math.cos(yaw),pose['y']+.435*math.sin(yaw)
        cells=set()
        for angle in np.linspace(yaw-math.pi/6,yaw+math.pi/6,13):
            for distance in np.arange(r/2,5.5,r/2):
                x,y=cx+distance*math.cos(angle),cy+distance*math.sin(angle)
                i,j=math.floor((x-ox)/r),math.floor((y-oy)/r)
                if not (0<=i<info.width and 0<=j<info.height):break
                cost=costs[j,i]
                if cost>=100:break
                if cost<0:
                    center=(ox+(i+.5)*r,oy+(j+.5)*r)
                    if distance>=.25 and math.hypot(center[0]-origin[0],center[1]-origin[1])<=radius:
                        cells.add(center)
                    break
        if cells:targets[pan]=cells
    return targets


def newly_known_cells(grid, points):
    info=grid.info;r=info.resolution;ox,oy=info.origin.position.x,info.origin.position.y
    known=0
    for x,y in points:
        i,j=math.floor((x-ox)/r),math.floor((y-oy)/r)
        if 0<=i<info.width and 0<=j<info.height and grid.data[j*info.width+i]>=0:
            known+=1
    return known


def choose_view(grid,quality,pose,origin,radius,history,remaining,strategy='quality_view_heuristic_v1',return_diagnostics=False,
                _boundary_fallback=False,*,depth_observed_visibility=False):
    """Known-space reachability plus conservative planar camera visibility prediction."""
    radius = math.inf if radius is None else radius
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
    # Traversability uncertainty is separate from optical occlusion. Only
    # actual measured depth points support transparency; an unobserved first
    # frontier is still counted once and stops the ray. Motion remains limited
    # to the original known-space path, target cost and chassis clearance.
    depth_supported=({k for k,c in cells.items()
                      if type(c.get('points')) in (int,float) and math.isfinite(c['points']) and c['points']>0}
                     if depth_observed_visibility else set())
    def blocks_visibility(cost,key):
        return cost>=100 or (cost<0 and (not depth_observed_visibility or key not in depth_supported))
    gradients={k:(c['gx'],c['gy']) for k,c in cells.items() if c.get('gx') is not None and c.get('gy') is not None}
    positions=[(pose['x'],pose['y'],0.,[(pose['x'],pose['y'])])]
    dist_steps = (1.2, 2.0, 3.0) if planner_v3 else (1.5, 3.)
    angle_steps = np.arange(0, 2*math.pi, math.pi/8 if planner_v3 else math.pi/4)
    for distance in dist_steps:
        for angle in angle_steps:
            x,y=pose['x']+distance*math.cos(angle),pose['y']+distance*math.sin(angle)
            ix,iy=index(x,y)
            max_c = 55 if planner_v3 else 65
            if (ix,iy) not in distances or costs[iy,ix]>max_c:continue
            # Circumscribed chassis clearance allows subsequent stationary camera turns.
            if not known_chassis_clearance(costs,ix,iy,resolution):continue
            path=[];key=(ix,iy)
            while key is not None:
                path.append((ox+(key[0]+.5)*resolution,oy+(key[1]+.5)*resolution));key=parents[key]
            positions.append((x,y,distances[(ix,iy)],list(reversed(path))))
    if planner_v3 and strategy=='coverage_view_v3':
        unseen_frontiers = [
            ((k[0]+.5)*qr, (k[1]+.5)*qr)
            for k, c in cells.items()
            if c.get('coverage', 0) < 0.25 and math.hypot((k[0]+.5)*qr-origin[0], (k[1]+.5)*qr-origin[1]) <= radius - 0.75
        ]
        if unseen_frontiers:
            step = max(1, len(unseen_frontiers)//8)
            for fx, fy in unseen_frontiers[::step][:8]:
                f_dir = math.atan2(fy - pose['y'], fx - pose['x'])
                for ds in (1.5, 2.5):
                    cx_p = pose['x'] + ds * math.cos(f_dir)
                    cy_p = pose['y'] + ds * math.sin(f_dir)
                    c_ix, c_iy = index(cx_p, cy_p)
                    if (c_ix, c_iy) in distances and costs[c_iy, c_ix] <= 50:
                        n = math.ceil(.65/resolution)
                        if n <= c_ix < info.width-n and n <= c_iy < info.height-n:
                            patch = costs[c_iy-n:c_iy+n+1, c_ix-n:c_ix+n+1]
                            yy, xx = np.ogrid[-n:n+1, -n:n+1]
                            if not np.any((patch[xx*xx+yy*yy <= n*n] < 0) | (patch[xx*xx+yy*yy <= n*n] >= 99)):
                                if not any(math.hypot(cx_p-p[0], cy_p-p[1]) < 0.3 for p in positions):
                                    path = []
                                    key = (c_ix, c_iy)
                                    while key is not None:
                                        path.append((ox+(key[0]+.5)*resolution, oy+(key[1]+.5)*resolution))
                                        key = parents[key]
                                    positions.append((cx_p, cy_p, distances[(c_ix, c_iy)], list(reversed(path))))
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
                for ds in (2.0, 2.8):
                    sx_pos=cx_def+ds*math.cos(sdir)
                    sy_pos=cy_def+ds*math.sin(sdir)
                    six,siy=index(sx_pos,sy_pos)
                    if (six,siy) not in distances or costs[siy,six]>50:continue
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
                    (not depth_observed_visibility and min(ix,iy,info.width-1-ix,info.height-1-iy)*resolution>3.)):continue
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
                        if blocks_visibility(cost,key):break
                        continue
                    if slope<=s_max:
                        if r>=.6 and in_roi and not is_grazing(key, angle):
                            visible.add(key)
                        max_slope=slope
                    else:
                        if r>=.6 and in_roi and not is_grazing(key, angle):
                            visible.add(key)
                        break
                    if blocks_visibility(cost,key):break
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
                    if blocks_visibility(cost,key):break
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
                    # In planner_v3 stationary scans, do not attempt a 180-deg in-place chassis spin.
                    # Angles outside gimbal pan limits will be observed upon chassis translation.
                    continue
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
                                     return_diagnostics=True,_boundary_fallback=True,
                                     depth_observed_visibility=depth_observed_visibility)
        diagnostics['map_boundary_fallback']=details
        if fallback is not None:fallback['map_boundary_fallback']=True
        return result(fallback)
    moves=[c for c in candidates if c['kind']!='turn']
    if planner_v3 and best is not None and best['kind']=='turn' and moves:
        last_was_turn = bool(history and history[-1].get('kind') == 'turn' and
                             math.hypot(pose['x']-history[-1].get('x', 0), pose['y']-history[-1].get('y', 0)) < 0.6)
        threshold = 1.0 if strategy == 'nearest_view_v3' else 1.8
        if last_was_turn or best.get('expected_new_m2', 0) < threshold:
            best = max(moves, key=lambda c: c['score'])
    elif planner_v2 and best is not None and best['kind']=='turn' and moves:
        fastest=min(moves,key=lambda c:c['estimated_seconds'])
        if best['estimated_seconds']+fastest['estimated_seconds']>remaining:
            best=max(moves,key=lambda c:c['score']);diagnostics['movement_reservation_applied']=True
    if best is not None and objective=='quality_view_heuristic_v1':
        best['expected_quality_gain_m2']=best['expected_utility_m2']
    return result(best)


class AutonomousSurvey:
    def __init__(self,motion,initial_depth_scan=False,localization_revisit_enabled=True):
        self.motion=motion;self.lock=threading.RLock();self.worker=None
        self.initial_depth_scan=bool(initial_depth_scan)
        self.localization_revisit_enabled=bool(localization_revisit_enabled)
        self.revisit_policy=RGBDRevisitPolicy()
        self.revisit_last_sample=None
        self.reset_revisit_capture()
        self.revisit_old_proof=ReobservedAnchorProof();self.revisit_receipt_cursor=None;self.revisit_receipt_snapshot=None
        self.survey_action_identity=None
        self.localization_recovered_counter=None
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
        if (not isinstance(body,dict) or not {'experiment','budget_seconds'} <= set(body)
                or set(body)-{'experiment','budget_seconds','radius','strategy'} or body['experiment']!=task['id']):
            raise ValueError('实验已改变，请刷新后启动自主探测')
        strategy=body.get('strategy','surface_view_v2')
        if strategy not in (*LEGACY_STRATEGIES,*STRATEGIES):raise ValueError('请选择就近观察、覆盖优先或地表补采策略')
        budget,radius=body['budget_seconds'],body.get('radius')
        if (type(budget) not in (int,float) or not math.isfinite(budget) or not 15<=budget<=1800
                or (radius is not None and (type(radius) not in (int,float) or not math.isfinite(radius) or not 2<=radius<=25))):
            raise ValueError('时间预算为 15～1800 仿真秒；活动半径可不设，旧任务指定时为 2～25 米')
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
            self.task=dict(task,region=list(scene_bounds(task.get('scene','survey'))))
            self.revisit_policy=RGBDRevisitPolicy();self.revisit_last_sample=None;self.survey_action_identity=None
            self.localization_recovered_counter=None
            self.reset_revisit_capture()
            self.revisit_old_proof=ReobservedAnchorProof();self.revisit_receipt_cursor=None;self.revisit_receipt_snapshot=None
            self.path=Path(directory)/task['id']/f'autonomy-{strategy}-{uuid.uuid4().hex[:8]}.json'
            self.data=dict(active=True,phase='planning',reason='根据当前观测比较转向、前进与换位补采',
                           origin=self.origin,budget_seconds=budget,radius=radius,started=started,
                           elapsed_seconds=0,history=[],decision=None,archive=self.path.name,strategy=strategy)
            self.data['metrics_version']=VERSION
            self.data['execution_policy']=EXECUTION_POLICY
            self.data['active_localization_revisit']=self.localization_revisit_enabled
            self.data['recovery_attempts']=0
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
        # Quality can change between the initial check, a recovery wait and
        # the locked readiness snapshot. Recheck after a newly observed drop
        # without growing the call stack or waiting while holding motion.lock.
        while self.data['active']:
            result=self._healthy_once()
            if result is not None:return result
        return False

    def _healthy_once(self):
        if not self.data['active']:return False
        if self.now()-self.data['started']>=self.data['budget_seconds']:
            self.stop('时间预算已用尽，已停止运动并保留采集档案','budget_reached');return False
        if time.monotonic()-self.wall_started>3600:
            self.stop('达到一小时运行上限','budget_reached');return False
        state=self.motion.state()
        health=state.get('diagnostics',{}).get('laser_localization',{})
        stopped_for_localization=(state.get('error')==LOCALIZATION_STOP_REASON and
            health.get('fault_counter')!=getattr(self,'localization_recovered_counter',None))
        if health.get('required') and (not health.get('healthy') or stopped_for_localization):
            if not self.wait_for_localization(state):return False
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
        retry_localization=False
        with self.motion.lock:
            state=self.motion.state()
            health=state.get('diagnostics',{}).get('laser_localization',{})
            pose_error=state.get('diagnostics',{}).get('pose_error','')
            if health.get('required') and (not health.get('healthy') or
                    (state.get('pose') is None and pose_error.startswith('激光定位质量失效：'))):
                retry_localization=True;reason=''
            elif not self.motion.grid_fresh():
                reason,phase='导航地图失效，已停止','fault'
            elif not state['ready'] or self.motion.quality is None or time.monotonic()-self.motion.quality_received>12:
                failed=[]
                if state.get('pose') is None:failed.append('车辆定位不可用')
                if state.get('depth_active') is False:failed.append('深度状态超过 3 秒未更新')
                if state.get('diagnostics',{}).get('navigation_ready') is False:failed.append('Nav2 服务不可用')
                if self.motion.quality is None:failed.append('尚无采样质量数据')
                elif time.monotonic()-self.motion.quality_received>12:failed.append('采样质量超过 12 秒未更新')
                reason,phase='；'.join(failed or ['导航就绪条件失效'])+'，已停止','fault'
            elif (self.data['radius'] is not None and
                  math.hypot(state['pose']['x']-self.origin[0],state['pose']['y']-self.origin[1])>self.data['radius']-.65):
                reason,phase='已接近活动距离限制','distance_reached'
            else:reason=''
        if retry_localization:
            return None if self.wait_for_localization(state) else False
        if reason:
            self.data['stop_diagnostics']=dict(reason=reason,health=state.get('diagnostics'),motion_stop=state.get('stop_diagnostics'))
            self.stop(reason,phase);return False
        try:self.update_localization_revisit()
        except (ValueError,TypeError) as error:
            self.stop('定位回访证据不可用：'+str(error),'fault');return False
        return True

    def wait_for_localization(self,state):
        health=state.get('diagnostics',{}).get('laser_localization',{})
        if (not health.get('healthy') and health.get('reason') not in TRANSIENT_LOCALIZATION_REASONS):
            self.stop('定位故障不能自动恢复：'+str(health.get('reason')),'fault');return False
        session=health.get('session_id')
        if not isinstance(session,str) or not session:
            self.stop('定位会话不可用，已停止','fault');return False
        event=dict(started_sim=self.now(),started_wall=time.monotonic(),
            reason=health.get('reason'),session_id=session,fault_counter=health.get('fault_counter'),
            interrupted_request_id=state.get('diagnostics',{}).get('goal_request_id'),outcome='waiting')
        with self.lock:
            if not self.data['active']:return False
            previous_phase=self.data['phase']
            self.data.setdefault('localization_pauses',[]).append(event)
            self.data.update(phase='localization_paused',reason='定位暂时失效，已停车等待恢复')
            self.motion.cancel(LOCALIZATION_STOP_REASON)
            self.save()
        while self.data['active']:
            now=time.monotonic()
            if self.now()-self.data['started']>=self.data['budget_seconds']:
                event['outcome']='budget_reached';self.stop('等待定位恢复时任务时间已用尽','budget_reached');return False
            if now-event['started_wall']>=LOCALIZATION_RECOVERY_WALL_SECONDS or now-self.wall_started>=3600:
                event['outcome']='timeout';self.stop('定位未恢复就绪，已停止并保留采集档案','fault');return False
            current=self.motion.state();quality=current.get('diagnostics',{}).get('laser_localization',{})
            if quality.get('session_id')!=session:
                event['outcome']='session_changed';self.stop('定位会话已改变，不自动恢复旧任务','fault');return False
            if not quality.get('healthy') and quality.get('reason') not in TRANSIENT_LOCALIZATION_REASONS:
                event['outcome']='fault';self.stop('定位故障不能自动恢复：'+str(quality.get('reason')),'fault');return False
            if not current.get('active') and current.get('ready') and quality.get('healthy'):
                with self.lock:
                    if not self.data['active']:break
                    event.update(outcome='recovered',recovered_sim=self.now(),recovered_wall=now,
                        ready_since_stamp=quality.get('ready_since_stamp'),
                        recovered_fault_counter=quality.get('fault_counter'))
                    self.localization_recovered_counter=quality.get('fault_counter')
                    self.data.update(phase=previous_phase,reason='定位已恢复，继续检查未完成任务')
                    self.save()
                return True
            time.sleep(.1)
        event['outcome']='stopped'
        self.save();return False

    def revisit_sample(self):
        # The independent consumer owns validation, freshness and generation.
        # Read its current payload and check result atomically under its lock.
        consumer=getattr(self.motion,'localization_quality',None)
        if not isinstance(getattr(consumer,'payload',None),dict):return None
        with self.motion.localization_lock:
            health=self.motion._localization_state_locked()
            sample=sample_payload(consumer.payload,health)
            self.revisit_receipt_snapshot=None
            if sample is not None and type(getattr(consumer,'revisit_history_epoch',None)) is int:
                self.revisit_receipt_snapshot=consumer.revisit_receipts(self.revisit_receipt_cursor)
            return sample

    def revisit_observation(self,stamp):
        # Query the actual measured TF at this exposure. Latest pose/joint
        # values cannot be attached to an older accepted covariance.
        from rclpy.time import Time
        self.revisit_capture_last_failure=None
        with self.motion.lock:
            joint_stamp=getattr(self.motion,'gimbal_joint_stamp',None)
            joint_received=getattr(self.motion,'gimbal_joint_received',None)
            captured=self.now();wall=time.monotonic()
            if (type(joint_stamp) not in (int,float) or type(joint_received) not in (int,float)
                    or not -.05<=captured-joint_stamp<=.1+1e-9 or not 0<=wall-joint_received<=2.):
                self.revisit_capture_last_failure=dict(reason='joint_feedback_not_fresh',retryable=True);return None
        query=Time(seconds=stamp)
        def measured(parent,child):
            value=self.motion.buffer.lookup_transform(parent,child,query)
            actual=value.header.stamp.sec+value.header.stamp.nanosec/1e9
            if abs(actual-stamp)>1e-8:raise ValueError('Exposure TF timestamp differs')
            q=value.transform.rotation
            if not all(math.isfinite(x) for x in (q.x,q.y,q.z,q.w)) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-6:
                raise ValueError('Invalid measured camera TF')
            return value
        try:
            body=measured('map','base_footprint')
            pan=measured('camera_mount_link','camera_pan_link').transform.rotation
            tilt=measured('camera_pan_link','camera_tilt_link').transform.rotation
            if max(abs(pan.x),abs(pan.y),abs(tilt.x),abs(tilt.z))>1e-6:raise ValueError('Invalid measured camera axes')
            p,q=body.transform.translation,body.transform.rotation
            values=dict(stamp=stamp,x=p.x,y=p.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
                        pan=2*math.atan2(pan.z,pan.w),tilt=-2*math.atan2(tilt.y,tilt.w),
                        joint_feedback_stamp=joint_stamp,joint_feedback_wall_age_s=wall-joint_received,
                        camera_tf_stamp=stamp,captured_sim=captured)
            if not all(math.isfinite(x) for x in values.values()):raise ValueError('Nonfinite measured camera TF')
            return values
        except Exception as error:
            temporary=type(error).__name__ in ('LookupException','ConnectivityException','ExtrapolationException')
            self.revisit_capture_last_failure=dict(reason=('exact_tf_temporarily_unavailable' if temporary else 'invalid_exact_tf'),
                retryable=temporary,error_type=type(error).__name__)
            return None

    def reset_revisit_capture(self):
        self.revisit_preplan_guard=None
        self.revisit_capture_state=None;self.revisit_capture_last_failure=None
        self.revisit_capture_stats=dict(exposures=0,attempts=0,retries=0,attached=0,temporary_failures=0,
            permanent_failures=0,current_exposure_changed=0,attempt_cap_reached=0,recent=[])

    def capture_revisit_observation(self,sample):
        """At most three total immediate attempts across healthy polls, no wait."""
        if not hasattr(self,'revisit_capture_stats'):self.reset_revisit_capture()
        def finish(value):
            if hasattr(self,'data'):
                self.data['localization_revisit_capture']={key:value for key,value in self.revisit_capture_stats.items() if key!='recent'}
            return value
        policy=self.revisit_policy
        if policy.last_observation_stamp==sample['stamp']:return False
        identity={key:sample.get(key) for key in ('session_id','stamp','fault_counter','reference_id')}
        state=self.revisit_capture_state;stats=self.revisit_capture_stats
        if state is None or state['identity']!=identity:
            state=dict(identity=identity,attempts=0,retries=0,status='pending')
            self.revisit_capture_state=state;stats['exposures']+=1
            stats['recent'].append(state);stats['recent']=stats['recent'][-64:]
        if state['status'] in ('attached','permanent_failure','superseded','attempt_cap'):return False
        if state['attempts']>=3:
            state['status']='attempt_cap';stats['attempt_cap_reached']+=1;return finish(False)
        state['attempts']+=1;stats['attempts']+=1
        if state['attempts']>1:state['retries']+=1;stats['retries']+=1
        finish(False)
        self.revisit_capture_last_failure=None
        observation=self.revisit_observation(sample['stamp'])
        if observation is None:
            failure=self.revisit_capture_last_failure or dict(reason='exact_tf_temporarily_unavailable',retryable=True)
            state['last_failure']=dict(failure)
            if failure.get('retryable'):
                stats['temporary_failures']+=1
                if state['attempts']>=3:state['status']='attempt_cap';stats['attempt_cap_reached']+=1
            else:state['status']='permanent_failure';stats['permanent_failures']+=1
            return finish(False)
        # Revalidate the real consumer after TF lookup; a late pose may never
        # be paired with a stale or superseded covariance/session/fault epoch.
        receipt_snapshot=self.revisit_receipt_snapshot
        try:current=self.revisit_sample()
        finally:self.revisit_receipt_snapshot=receipt_snapshot
        if (current is None or current['session_id']!=sample['session_id']
                or current['fault_counter']!=sample['fault_counter']):
            raise ValueError('Localization health changed while capturing revisit observation')
        if any(current.get(key)!=identity[key] for key in ('stamp','reference_id')):
            state['status']='superseded';stats['current_exposure_changed']+=1;return finish(False)
        attached=policy.attach_observation(sample,observation)
        state['status']='attached';state['captured_sim']=observation.get('captured_sim')
        if attached:stats['attached']+=1
        return finish(attached)

    def persist_revisit_plan_snapshot(self,record,grid,pose,remaining,gimbal,map_receipt):
        """Write bounded actual planning evidence once, outside ROS locks."""
        if not hasattr(self,'path'):return
        folder=getattr(getattr(self,'recorder',None),'folder',None)
        if folder is None:folder=self.path.parent/'research'/self.path.stem
        folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
        policy=self.revisit_policy;info=grid.info
        header=getattr(getattr(grid,'header',None),'stamp',None)
        header_stamp=(header.sec+header.nanosec/1e9 if header is not None else None)
        snapshot=dict(schema='revisit-planning-snapshot-v1',captured_sim=self.now(),
            map_snapshot=map_receipt,map_header_stamp=header_stamp,
            navigation_grid=dict(width=info.width,height=info.height,resolution=info.resolution,
                origin=[info.origin.position.x,info.origin.position.y],
                encoding='base64-int8-row-major',data=base64.b64encode(np.asarray(grid.data,dtype=np.int8).tobytes()).decode('ascii')),
            current_pose=dict(pose),remaining_seconds=remaining,gimbal_position=list(gimbal) if gimbal is not None else None,
            region=list(self.task['region']),origin=list(self.origin),radius=self.data['radius'],
            trigger=copy.deepcopy(policy.pending['trigger']),retirement_event=copy.deepcopy(policy.pending.get('retirement_event')),
            attempts=policy.attempts,observations=copy.deepcopy(policy.observations),breadcrumbs=copy.deepcopy(policy.breadcrumbs),
            capture=copy.deepcopy(self.revisit_capture_stats),candidate_diagnostics=copy.deepcopy(policy.plan_diagnostics))
        history=record.setdefault('planning_evidence_history',[])
        suffix='' if not history else f'-refresh-{len(history)}'
        destination=folder/f'revisit-plan-{len(self.data.get("localization_revisits",[])):02d}-{policy.attempts:02d}{suffix}.json'
        contents=(json.dumps(snapshot,ensure_ascii=False,indent=2)+'\n').encode('utf-8')
        temporary=destination.with_suffix('.tmp');temporary.write_bytes(contents);temporary.replace(destination)
        record['planning_evidence']=dict(path=str(destination.relative_to(self.path.parent)),
            sha256=hashlib.sha256(contents).hexdigest(),observations=len(policy.observations),breadcrumbs=len(policy.breadcrumbs),
            capture_attempts=self.revisit_capture_stats['attempts'],candidate_counts={key:value for key,value in policy.plan_diagnostics.items() if key!='rejections'})
        history.append(dict(record['planning_evidence']));record['planning_evidence_history']=history[-2:]

    def preplan_observation_allowed(self,decision):
        guard=getattr(self,'revisit_preplan_guard',None);policy=self.revisit_policy
        if (not isinstance(guard,dict) or guard.get('decision') is not decision
                or decision.get('preplan_unknown_observation') is not True or decision.get('use_pan_tilt') is not True
                or decision.get('kind')!='turn' or not callable(getattr(self.motion,'pan_tilt',None))):return False
        pending=policy.pending;sample=self.revisit_last_sample
        if (pending is not guard['pending'] or policy.running or policy.retirement_generation!=guard['retirement_generation']
                or pending['trigger']!=guard['trigger'] or pending.get('retirement_event') is not None
                or guard['reference_id'] in policy.retired_observation_ids or sample is None
                or sample['session_id']!=guard['trigger']['session_id']
                or sample['fault_counter']!=guard['trigger']['fault_counter']):return False
        identity=guard.get('action_identity')
        return identity is None or (self.survey_action_identity==identity and getattr(self.motion,'token',None)==identity[0])

    def observe_unknown_revisit_clearance(self,record):
        """One owned, bounded gimbal observation before any return plan exists."""
        policy=self.revisit_policy;pending=policy.pending
        if pending is None:return False
        trigger=pending['trigger']
        if not callable(getattr(self.motion,'pan_tilt',None)):return False
        if any(row.get('preplan_unknown_observation') is not None and row['trigger']['session_id']==trigger['session_id']
               and row['trigger']['stamp']==trigger['stamp'] for row in self.data.get('localization_revisits',[])):return False
        from experiment_motion import within_region
        with self.motion.lock:
            state=self.motion.state()
            if (state.get('active') or state.get('phase') not in ('cancelled','succeeded')
                    or state.get('diagnostics',{}).get('goal_request_id')!=record['old_action_terminal']['request_id']
                    or not self.motion.grid_fresh()):return False
            grid=self.motion.grid;pose=self.motion.pose();gimbal=getattr(self.motion,'gimbal_position',None)
            rate=getattr(self.motion,'gimbal_command_rate_rad_s',None)
            before_frames=(self.motion.quality or {}).get('frames')
            remaining=self.data['budget_seconds']-(self.now()-self.data['started'])
        if type(before_frames) is not int or before_frames<0:return False
        costs=np.asarray(grid.data,dtype=np.int16).reshape(grid.info.height,grid.info.width)
        scan=policy.unknown_clearance_scan(pose,grid,costs,known_chassis_clearance,within_region,
            self.task['region'],self.origin,self.data['radius'],remaining,gimbal,gimbal_rate=rate)
        if scan is None:return False
        started=self.now();wall=time.monotonic()
        decision=dict(x=pose['x'],y=pose['y'],kind='turn',use_pan_tilt=True,pan=scan['pan'],tilt=scan['tilt'],
            localization_revisit=True,preplan_unknown_observation=True,estimated_seconds=scan['estimated_seconds'],
            observation_started_sim=started,observation_deadline_sim=started+scan['estimated_seconds']+4.,
            observation_deadline_wall=wall+60.)
        # Mark once before dispatch. Retirement replans cannot erase this record.
        evidence=dict(status='dispatching',started_sim=started,started_wall=wall,reference_id=scan['checkpoint']['reference_id'],
            checkpoint_stamp=scan['checkpoint']['stamp'],unknown_cell_count=len(scan['unknown_cells']),decision=dict(decision),
            before_quality_frames=before_frames,
            return_reserve_seconds=scan['return_reserve_seconds'],fresh_wait_sim_s=scan['fresh_wait_sim_s'],
            fresh_wait_wall_s=scan['fresh_wait_wall_s'])
        record['preplan_unknown_observation']=evidence;policy.attempts+=1;self.save()
        self.revisit_preplan_guard=dict(decision=decision,pending=pending,trigger=dict(trigger),
            retirement_generation=policy.retirement_generation,reference_id=scan['checkpoint']['reference_id'],action_identity=None)
        try:
            if not self.execute(decision,turn=True):
                if self.data['active']:self.stop('回访前有界补观测被中断，已停止','blocked')
                return False
            if not self.healthy() or not self.preplan_observation_allowed(decision):return False
            state=self.motion.state();identity=state.get('diagnostics',{}).get('goal_request_id');token=getattr(self.motion,'token',None)
            if (not isinstance(token,str) or not token or not isinstance(identity,str) or not identity
                    or state.get('active') or state.get('phase')!='succeeded'
                    or self.survey_action_identity!=(token,identity)
                    or self.revisit_preplan_guard.get('action_identity')!=(token,identity)):
                self.stop('补观测没有同标识的真实云台终结，已停止','fault');return False
            with self.motion.lock:
                joint_stamp=getattr(self.motion,'gimbal_joint_stamp',None);received=getattr(self.motion,'gimbal_joint_received',None)
                completed=self.now();completed_wall=time.monotonic()
            if (type(joint_stamp) not in (int,float) or type(received) not in (int,float)
                    or not -.05<=completed-joint_stamp<=.1+1e-9 or not 0<=completed_wall-received<=2.):
                self.stop('补观测终结后缺少新鲜真实云台反馈，已停止','fault');return False
            evidence.update(status='waiting_new_observations',completed_sim=completed,request_id=identity,
                token_fingerprint=hashlib.sha256(token.encode()).hexdigest()[:16],joint_stamp=joint_stamp)
            deadline=completed+scan['fresh_wait_sim_s'];wall_deadline=min(wall+60.,completed_wall+scan['fresh_wait_wall_s'])
            while self.healthy():
                if not self.preplan_observation_allowed(decision):
                    self.stop('补观测事务的定位或参考已改变，已停止','blocked');return False
                with self.motion.lock:
                    latest=self.motion.grid;received_map=getattr(self.motion,'grid_received',0.)
                    stamp=latest.header.stamp.sec+latest.header.stamp.nanosec/1e9 if latest is not None else -1.
                    quality_stamp=(self.motion.quality or {}).get('stamp')
                    quality_frames=(self.motion.quality or {}).get('frames')
                    fresh=self.motion.grid_fresh()
                sample=self.revisit_last_sample
                if (sample['stamp']>completed and type(quality_stamp) in (int,float) and quality_stamp>completed
                        and type(quality_frames) is int and quality_frames>before_frames
                        and fresh and stamp>completed and received_map>completed_wall):
                    evidence.update(status='new_observations_received',new_accepted_stamp=sample['stamp'],
                        new_quality_stamp=quality_stamp,new_map_header_stamp=stamp,new_map_received_wall=received_map,
                        new_quality_frames=quality_frames,
                        fresh_completed_sim=self.now());self.save();return True
                if self.now()>=deadline or time.monotonic()>=wall_deadline:
                    evidence['status']='fresh_observation_timeout';self.stop('补观测后没有真实新帧及更新的导航地图，已停止','blocked');return False
                time.sleep(.1)
            return False
        finally:self.revisit_preplan_guard=None

    def revisit_interrupt_pending(self):
        pending=self.revisit_policy.pending
        return pending is not None and not self.revisit_policy.running and not pending.get('wait_for_gimbal',False)

    def update_localization_revisit(self):
        if not self.localization_revisit_enabled:return
        sample=self.revisit_sample()
        if sample is None:return
        self.revisit_last_sample=sample
        # Quality, retirement and reserve warning are immediate, even when
        # exact exposure TF has not reached the independent parent buffer.
        try:self.revisit_policy.observe(sample,None)
        except RetiredCheckpoint as event:
            # Library retirement invalidates a target, not the healthy RGBD
            # observation. Preserve the original uncertainty transaction and
            # cancel this return before planning any replacement request.
            if self.data.get('localization_revisits'):
                self.data['localization_revisits'][-1].update(status='checkpoint_retired',
                    retirement_event=dict(reference_id=event.reference_id,stamp=event.stamp))
            self.revisit_policy.invalidate_retired_checkpoint(event)
        self.capture_revisit_observation(sample)
        pending=self.revisit_policy.pending
        snapshot=self.revisit_receipt_snapshot
        if snapshot is not None:
            counter=sample.get('fault_counter')
            if type(counter) is not int or counter<0:
                raise ValueError('RGBD evidence has no actual motion fault counter')
            if pending is not None and counter!=pending['trigger']['fault_counter']:
                raise ValueError('Localization outage occurred before reading RGBD evidence')
            self.revisit_old_proof.consume(snapshot['rows'],pending['trigger'] if pending is not None else None,
                                           snapshot['consumer_health_epoch'])
            # Metadata analysis is outside both locks. Check the real cumulative
            # counter again before committing its cursor or any pending proof.
            with self.motion.localization_lock:
                after=self.motion._localization_state_locked()
            if (not after.get('healthy') or after.get('session_id')!=sample['session_id']
                    or type(after.get('fault_counter')) is not int or after['fault_counter']!=counter):
                raise ValueError('Localization health changed while consuming RGBD evidence')
            self.revisit_receipt_cursor=snapshot['cursor'];self.revisit_receipt_snapshot=None
            if pending is not None:
                self.data['localization_old_anchor_evidence']=self.revisit_old_proof.evidence()
        if pending is None or self.revisit_policy.running:return
        state=self.motion.state();identity=state.get('diagnostics',{}).get('goal_request_id')
        if pending.get('wait_for_gimbal') and not state.get('active'):
            if identity!=pending['cancel_request_id'] or state.get('phase')!='succeeded':
                raise ValueError('The bounded gimbal action did not complete successfully')
            pending['wait_for_gimbal']=False
        if pending.get('wait_for_gimbal') and (self.now()>pending['gimbal_deadline_sim'] or time.monotonic()>pending['gimbal_deadline_wall']):
            raise ValueError('The bounded gimbal completion time was exceeded')
        if 'cancel_request_id' not in pending:
            pending['cancel_request_id']=identity
            pending['old_action_active']=bool(state.get('active'))
            token=getattr(self.motion,'token',None)
            pending['old_token_fingerprint']=(hashlib.sha256(token.encode()).hexdigest()[:16] if isinstance(token,str) else None)
            reason=('保存的回访参考已退休，取消当前回访并重新规划' if pending.get('retirement_event') else
                    '定位协方差预算触发主动回访，已取消本次探索动作')
            pending['cancellation_reason']=reason
            if state.get('active'):
                if self.survey_action_identity!=(token,identity):raise ValueError('Active action belongs to another request')
                if state.get('diagnostics',{}).get('goal_request_kind')=='gimbal':
                    # Original gimbal cancel has no asynchronous terminal ACK.
                    # Finish only this already bounded leased command; issue no
                    # further old exploration command before the new revisit.
                    pending['wait_for_gimbal']=True
                    pending['gimbal_deadline_sim']=self.motion.sim_started+self.motion.sim_budget
                    pending['gimbal_deadline_wall']=self.motion.gimbal_deadline
                else:self.motion.cancel(reason)
            self.data['localization_revisit_pending']=dict(trigger=dict(pending['trigger']),old_request_id=identity,
                retirement_event=pending.get('retirement_event'),
                old_token_fingerprint=pending['old_token_fingerprint'],cancellation_reason=reason,
                cancellation_requested=bool(state.get('active') and not pending.get('wait_for_gimbal')),
                waiting_bounded_gimbal_completion=bool(pending.get('wait_for_gimbal')),
                remaining_bounded_gimbal_sim_s=(max(0.,pending['gimbal_deadline_sim']-self.now()) if pending.get('wait_for_gimbal') else 0.))
            self.save()

    def perform_localization_revisit(self):
        # Retired targets restart this bounded attempt loop. An old attempt
        # never resumes its cancelled goals after the replacement succeeds.
        while self.data['active']:
            if self.revisit_policy.attempts>=MAX_REVISITS:
                self.stop('定位回访的有界尝试次数已用尽，已停止','blocked');return False
            generation=self.revisit_policy.retirement_generation
            if self._perform_localization_revisit_attempt():return True
            if (not self.data['active'] or not self.revisit_interrupt_pending()
                    or self.revisit_policy.retirement_generation==generation):return False
        return False

    def _perform_localization_revisit_attempt(self):
        policy=self.revisit_policy;pending=policy.pending
        if pending is None:return True
        record=dict(trigger=dict(pending['trigger']),status='waiting_action_terminal',steps=[],
                    precision_goal_checker='revisit_goal_checker',position_tolerance_m=REVISIT_POSITION_TOLERANCE_M,
                    retirement_event=pending.get('retirement_event'),
                    cancellation=dict(request_id=pending.get('cancel_request_id'),
                        token_fingerprint=pending.get('old_token_fingerprint'),reason=pending.get('cancellation_reason')))
        self.data.setdefault('localization_revisits',[]).append(record);self.save()
        old_id=pending.get('cancel_request_id');deadline=time.monotonic()+10.
        while self.healthy():
            state=self.motion.state();actual_id=state.get('diagnostics',{}).get('goal_request_id')
            if actual_id!=old_id:
                self.stop('回访前原动作标识已改变，已停止','fault');return False
            if not state.get('active') and state.get('phase')!='cancelling':
                if state.get('phase')=='failed':
                    self.stop(state.get('error') or '原回访动作异常失败，未派发新的回访目标','fault');return False
                if pending.get('old_action_active') and state.get('phase') not in ('succeeded','cancelled'):
                    self.stop('原动作尚无真实终结状态，未派发回访目标','blocked');return False
                record['old_action_terminal']=dict(request_id=actual_id,phase=state.get('phase'),observed_sim=self.now(),
                    token_fingerprint=pending.get('old_token_fingerprint'),cancellation_reason=pending.get('cancellation_reason'))
                break
            if time.monotonic()>=deadline:
                self.stop('原动作尚未真实结束，未派发回访目标','blocked');return False
            time.sleep(.1)
        if not self.data['active']:return False
        from experiment_motion import within_region
        with self.motion.lock:
            if not self.motion.grid_fresh():self.stop('回访导航地图失效，已停止','fault');return False
            grid=self.motion.grid;pose=self.motion.pose()
            remaining=self.data['budget_seconds']-(self.now()-self.data['started'])
            gimbal=getattr(self.motion,'gimbal_position',None)
            map_receipt=dict(received_wall=getattr(self.motion,'grid_received',None),captured_wall=time.monotonic(),
                captured_sim=self.now(),fresh=True)
        # Geometry uses the immutable received message snapshot outside motion
        # and localization locks; a current-grid recheck precedes every goal.
        costs=np.asarray(grid.data,dtype=np.int16).reshape(grid.info.height,grid.info.width)
        plan=policy.plan(pose,grid,costs,known_chassis_clearance,within_region,self.task['region'],self.origin,
                         self.data['radius'],remaining,gimbal)
        self.persist_revisit_plan_snapshot(record,grid,pose,remaining,gimbal,map_receipt)
        if plan is None and self.observe_unknown_revisit_clearance(record):
            if not self.healthy():return False
            with self.motion.lock:
                if not self.motion.grid_fresh():self.stop('补观测后导航地图失效，已停止','fault');return False
                grid=self.motion.grid;pose=self.motion.pose();gimbal=getattr(self.motion,'gimbal_position',None)
                remaining=self.data['budget_seconds']-(self.now()-self.data['started'])
                map_receipt=dict(received_wall=self.motion.grid_received,captured_wall=time.monotonic(),captured_sim=self.now(),fresh=True)
            costs=np.asarray(grid.data,dtype=np.int16).reshape(grid.info.height,grid.info.width)
            plan=policy.plan(pose,grid,costs,known_chassis_clearance,within_region,self.task['region'],self.origin,self.data['radius'],remaining,gimbal)
            self.persist_revisit_plan_snapshot(record,grid,pose,remaining,gimbal,map_receipt)
        if not self.data['active']:return False
        if plan is None:
            record['status']='no_safe_budgeted_checkpoint';self.stop('没有当前已观测且预算足够的低不确定度回访位置，已停止','blocked');return False
        policy.begin(plan);record.update(status='returning',plan=plan);self.save()
        checkpoint=plan['checkpoint']
        for goal in plan['goals']:
            if not self.healthy():return False
            with self.motion.lock:
                grid=self.motion.grid;pose=self.motion.pose()
            costs=np.asarray(grid.data,dtype=np.int16).reshape(grid.info.height,grid.info.width)
            from localization_revisit import safe_segment
            ix=math.floor((goal['x']-grid.info.origin.position.x)/grid.info.resolution)
            iy=math.floor((goal['y']-grid.info.origin.position.y)/grid.info.resolution)
            valid=(0<=ix<grid.info.width and 0<=iy<grid.info.height and 0<=costs[iy,ix]<=65
                   and known_chassis_clearance(costs,ix,iy,grid.info.resolution)
                   and safe_segment(pose,goal,grid,costs,known_chassis_clearance,within_region,self.task['region'],self.origin,self.data['radius']))
            if not valid:self.stop('回访路线的当前已观测安全条件已改变，已停止','blocked');return False
            decision=dict(x=goal['x'],y=goal['y'],kind='advance',localization_revisit=True,precise_revisit=True,
                          estimated_seconds=math.hypot(goal['x']-pose['x'],goal['y']-pose['y'])/.06+9.)
            if not self.has_action_budget(decision):self.stop('回访剩余时间预算不足','budget_reached');return False
            if math.hypot(goal['x']-pose['x'],goal['y']-pose['y'])>REVISIT_POSITION_TOLERANCE_M:
                if not self.execute(decision):
                    if self.revisit_interrupt_pending():return False
                    self.stop('回访导航未完成，已停止','blocked');return False
            actual=self.motion.pose()
            goal_distance=math.hypot(goal['x']-actual['x'],goal['y']-actual['y'])
            record['steps'].append(dict(decision,completed_sim=self.now(),actual_pose=dict(actual),
                distance_to_goal_m=goal_distance,
                distance_to_saved_m=math.hypot(checkpoint['x']-actual['x'],checkpoint['y']-actual['y'])))
            self.save()
            if goal_distance>REVISIT_POSITION_TOLERANCE_M:
                self.stop('精确回访导航终结但未实际到达五厘米目标范围，已停止','blocked');return False
        pose=self.motion.pose()
        if math.hypot(pose['x']-checkpoint['x'],pose['y']-checkpoint['y'])>REVISIT_POSITION_TOLERANCE_M:
            self.stop('回访未到达保存的真实观察位置，已停止','blocked');return False
        for restoration in range(2):
            if not self.restore_revisit_view(checkpoint):return False
            if not self.healthy():return False
            pose=self.motion.pose()
            distance=math.hypot(pose['x']-checkpoint['x'],pose['y']-checkpoint['y'])
            record.setdefault('view_restoration_checks',[]).append(dict(
                restored_sim=self.now(),pose=dict(pose),distance_to_saved_m=distance))
            self.save()
            if distance<=REVISIT_POSITION_TOLERANCE_M:
                if restoration and not self.revisit_correction_terminal(record,final_view=True):return False
                break
            if restoration:
                self.stop('回访视角恢复后再次偏离保存位置，纠位次数已用尽，已停止','blocked');return False
            if not self.correct_revisit_view_displacement(checkpoint,record):return False
        arrived=self.now();until=arrived+3.;wall_until=time.monotonic()+10.
        record.update(status='waiting_measured_lower_uncertainty',view_completed_sim=arrived);self.save()
        while self.healthy() and self.now()<until and time.monotonic()<wall_until:
            if self.revisit_interrupt_pending():return False
            sample=self.revisit_last_sample
            strict=sample is not None and policy.prove(sample,arrived)
            old_event=(sample is not None and self.revisit_old_proof.prove(sample,arrived,checkpoint))
            if strict or old_event:
                stamps=policy.proof_stamps if strict else self.revisit_old_proof.proof_stamps
                record.update(status='reanchored',proof_sample=sample,proof_exposure_stamps=list(stamps),
                              proof_kind='same_saved_reference' if strict else 'actual_old_fit_then_three_stable_exposures')
                if old_event:record['old_anchor_evidence']=self.revisit_old_proof.evidence()
                self.revisit_old_proof=ReobservedAnchorProof()
                policy.complete();self.data.pop('localization_revisit_pending',None);self.save();return True
            time.sleep(.1)
        record['old_anchor_evidence']=self.revisit_old_proof.evidence();self.save()
        if self.data['active']:self.stop('回访没有取得三个真实旧参考匹配及严格降低的方差，已停止','blocked')
        return False

    def restore_revisit_view(self,checkpoint):
        pose=self.motion.pose();pan=wrap(checkpoint['yaw']+checkpoint['pan']-pose['yaw'])
        if abs(pan)>2.356:
            decision=dict(x=pose['x'],y=pose['y'],yaw=checkpoint['yaw'],kind='turn',localization_revisit=True,estimated_seconds=math.pi/.5+6.)
            if not self.has_action_budget(decision) or not self.execute(decision,turn=True):
                if self.revisit_interrupt_pending():return False
                if self.data['active']:self.stop('回访车身视角未完成，已停止','blocked')
                return False
            pan=wrap(checkpoint['yaw']+checkpoint['pan']-self.motion.pose()['yaw'])
        if abs(pan)>2.356:self.stop('回访相机视角超出原云台行程，已停止','blocked');return False
        decision=dict(x=checkpoint['x'],y=checkpoint['y'],yaw=checkpoint['yaw'],kind='turn',use_pan_tilt=True,
                      pan=pan,tilt=checkpoint['tilt'],localization_revisit=True,estimated_seconds=math.hypot(2*2.356,.873)/.35+6.)
        if not self.has_action_budget(decision) or not self.execute(decision,turn=True):
            if self.revisit_interrupt_pending():return False
            if self.data['active']:self.stop('回访相机真实观测未完成，已停止','blocked')
            return False
        return True

    def revisit_correction_terminal(self,record,final_view=False):
        if not self.healthy():return False
        state=self.motion.state();diag=state.get('diagnostics',{})
        identity=diag.get('goal_request_id');token=getattr(self.motion,'token',None)
        if (not isinstance(token,str) or not token or not isinstance(identity,str) or not identity
                or state.get('active') or state.get('phase')!='succeeded'
                or self.survey_action_identity!=(token,identity)):
            self.stop('回访纠位前后动作尚无同标识的真实成功终结，已停止','fault');return False
        entry=dict(request_id=identity,phase=state['phase'],observed_sim=self.now(),
                   token_fingerprint=hashlib.sha256(token.encode()).hexdigest()[:16])
        if final_view:
            with self.motion.lock:
                stamp=getattr(self.motion,'gimbal_joint_stamp',None)
                received=getattr(self.motion,'gimbal_joint_received',None)
                values=getattr(self.motion,'gimbal_position',None)
                now=self.now();wall=time.monotonic()
                fresh=(type(stamp) in (int,float) and type(received) in (int,float)
                       and -.05<=now-stamp<=.1+1e-9 and 0<=wall-received<=2.
                       and isinstance(values,(list,tuple)) and len(values)==2
                       and all(type(v) in (int,float) and math.isfinite(v) for v in values))
            if not fresh:
                self.stop('回访纠位后缺少新鲜的真实云台反馈，已停止','fault');return False
            entry.update(joint_stamp=stamp,joint_position=list(values),joint_wall_age_s=wall-received)
        record.setdefault('view_displacement_terminals',[]).append(entry);self.save()
        return True

    def correct_revisit_view_displacement(self,checkpoint,record):
        # One explicit correction repairs measured body motion during view
        # restoration. It never retries an unsuccessful covariance proof.
        if not self.revisit_correction_terminal(record):return False
        trigger=record['trigger']
        if any(row.get('view_restoration_displacement') is not None
               and row['trigger']['session_id']==trigger['session_id'] and row['trigger']['stamp']==trigger['stamp']
               for row in self.data.get('localization_revisits',[])):
            self.stop('本次定位回访的视角恢复位移纠位次数已用尽，已停止','blocked');return False
        if self.revisit_policy.attempts>=MAX_REVISITS:
            self.stop('定位回访的有界尝试次数已用尽，已停止','blocked');return False
        from experiment_motion import within_region
        from localization_revisit import safe_segment
        with self.motion.lock:
            if not self.motion.grid_fresh():
                self.stop('回访纠位导航地图失效，已停止','fault');return False
            grid=self.motion.grid;pose=self.motion.pose()
        costs=np.asarray(grid.data,dtype=np.int16).reshape(grid.info.height,grid.info.width)
        ix=math.floor((checkpoint['x']-grid.info.origin.position.x)/grid.info.resolution)
        iy=math.floor((checkpoint['y']-grid.info.origin.position.y)/grid.info.resolution)
        valid=(0<=ix<grid.info.width and 0<=iy<grid.info.height and 0<=costs[iy,ix]<=65
               and known_chassis_clearance(costs,ix,iy,grid.info.resolution)
               and safe_segment(pose,checkpoint,grid,costs,known_chassis_clearance,within_region,self.task['region'],self.origin,self.data['radius']))
        if not valid:
            self.stop('视角恢复位移的当前已观测纠位路线不安全，已停止','blocked');return False
        decision=dict(x=checkpoint['x'],y=checkpoint['y'],kind='advance',localization_revisit=True,
                      reason='view_restoration_displacement',precise_revisit=True,
                      estimated_seconds=math.hypot(checkpoint['x']-pose['x'],checkpoint['y']-pose['y'])/.06+9.)
        # Reserve a complete original view restoration and proof window before
        # spending this additional action; each action retains its own lease,
        # simulation deadline and original wall timeout in MotionController.
        reserve=math.pi/.5+math.hypot(2*2.356,.873)/.35+12.+3.
        if not self.has_action_budget(dict(decision,estimated_seconds=decision['estimated_seconds']+reserve)):
            self.stop('视角恢复位移的纠位与重新观测预算不足，已停止','budget_reached');return False
        self.revisit_policy.attempts+=1;self.revisit_policy.proof_stamps=[]
        record['view_restoration_displacement']=dict(reason='view_restoration_displacement',
            measured_pose=dict(pose),saved_checkpoint_stamp=checkpoint['stamp'],
            saved_reference_id=checkpoint['reference_id'],attempt=self.revisit_policy.attempts,
            prior_terminal_request_id=record['view_displacement_terminals'][-1]['request_id'],decision=decision)
        self.save()
        if not self.execute(decision):
            if self.revisit_interrupt_pending():return False
            if self.data['active']:self.stop('视角恢复位移纠位未完成，已停止','blocked')
            return False
        if not self.revisit_correction_terminal(record):return False
        pose=self.motion.pose()
        record['view_restoration_displacement'].update(completed_sim=self.now(),actual_pose=dict(pose),
            distance_to_saved_m=math.hypot(pose['x']-checkpoint['x'],pose['y']-checkpoint['y']))
        self.save()
        if math.hypot(pose['x']-checkpoint['x'],pose['y']-checkpoint['y'])>REVISIT_POSITION_TOLERANCE_M:
            self.stop('视角恢复位移纠位未到达保存位置，已停止','blocked');return False
        return True

    def has_action_budget(self,decision):
        remaining=self.data['budget_seconds']-(self.now()-self.data['started'])
        return decision['estimated_seconds']+EXECUTION_MARGIN_SECONDS<=remaining

    def execute(self,decision,turn=False):
        while self.data['active']:
            generation=len(self.data.get('localization_pauses',[]))
            try:
                result=self._execute_once(decision,turn)
            except LocalizationDispatchRejected as error:
                # Dispatch may lose freshness after healthy() returned, even
                # under the motion lock as the independent clock advances.
                # No lease was granted. Wait outside both dispatch locks.
                if not self.data['active']:return False
                state=self.motion.state();health=state.get('diagnostics',{}).get('laser_localization',{})
                if (error.reason not in TRANSIENT_LOCALIZATION_REASONS or not error.session_id
                        or health.get('session_id')!=error.session_id):
                    self.stop('定位派发拒绝不能自动恢复：'+str(error.reason),'fault');return False
                self.data['localization_dispatch_rejections']=self.data.get('localization_dispatch_rejections',0)+1
                if not self.wait_for_localization(state):return False
                result=False
            if result or not self.data['active']:return result
            if len(self.data.get('localization_pauses',[]))==generation:return False
            # Revisit transactions own their evidence/gauge and interruption
            # rules. Do not replay one after a health epoch changes.
            if any(decision.get(k) for k in ('localization_revisit','precise_revisit','preplan_unknown_observation','frontier_observation')):
                return False
            if not self.has_action_budget(decision):
                self.stop('定位恢复后剩余时间不足以继续观察动作','budget_reached');return False
        return False

    def _execute_once(self,decision,turn=False):
        if not self.healthy():return False
        with self.lock:
            if not self.data['active']:return False
            if decision.get('preplan_unknown_observation') and not turn:
                raise ValueError('回访前补观测只允许原地云台动作')
            if self.revisit_interrupt_pending() and not self.preplan_observation_allowed(decision):return False
            try:
                if turn and decision.get('use_pan_tilt') and hasattr(self.motion,'pan_tilt'):
                    response=self.motion.pan_tilt(decision['pan'],decision.get('tilt',-0.2618),self.task)
                elif turn:
                    response=self.motion.turn(decision['yaw'],self.task)
                else:
                    body=dict(experiment=self.task['id'],goal=[decision['x'],decision['y']])
                    response=(self.motion.start(body,self.task,precise_revisit=True) if decision.get('precise_revisit')
                              else self.motion.start(body,self.task))
            except ValueError as error:
                if turn or not str(error).startswith(('目标位于障碍','目标尚未观测','目标超出当前导航地图边界')):
                    raise
                decision['dispatch_error']=str(error)
                return False
            token=response['token'];state=self.motion.state()
            self.survey_action_identity=(token,state.get('diagnostics',{}).get('goal_request_id'))
            if decision.get('preplan_unknown_observation'):
                guard=self.revisit_preplan_guard
                if (not self.preplan_observation_allowed(decision) or not isinstance(token,str) or not token
                        or not isinstance(self.survey_action_identity[1],str) or not self.survey_action_identity[1]
                        or state.get('diagnostics',{}).get('goal_request_kind')!='gimbal'):
                    raise ValueError('补观测云台请求标识不属于当前事务')
                guard['action_identity']=self.survey_action_identity
            if decision.get('localization_revisit') and self.data.get('localization_revisits'):
                current=self.data['localization_revisits'][-1]
                current.setdefault('action_dispatches',[]).append(dict(
                    request_id=self.survey_action_identity[1],token_fingerprint=hashlib.sha256(token.encode()).hexdigest()[:16],
                    kind=state.get('diagnostics',{}).get('goal_request_kind'),dispatched_sim=self.now(),
                    after_terminal_request_id=current.get('old_action_terminal',{}).get('request_id')))
            self.data['phase']='gimbal_scanning' if (turn and decision.get('use_pan_tilt')) else ('turning' if turn else 'moving');self.save()
        generation=len(self.data.get('localization_pauses',[]))
        while self.healthy():
            if len(self.data.get('localization_pauses',[]))!=generation:return False
            if self.revisit_interrupt_pending() and not self.preplan_observation_allowed(decision):return False
            if decision.get('preplan_unknown_observation') and (self.now()<decision['observation_started_sim']-1e-9
                    or self.now()>=decision['observation_deadline_sim'] or time.monotonic()>=decision['observation_deadline_wall']):
                self.stop('回访前有界云台补观测时间已用尽，已停止','blocked');return False
            if decision.get('bootstrap') and (self.now()>=decision['bootstrap_deadline_sim']
                    or time.monotonic()>=decision['bootstrap_deadline_wall']):
                self.stop('启动观察的时间预算已用尽，已停止','budget_reached');return False
            self.motion.heartbeat(dict(token=token))
            state=self.motion.state()
            if not state['active']:
                if state['phase']=='cancelled' and not state.get('replan_allowed'):
                    self.stop(state['error'] or '运动已取消','fault')
                return state['phase']=='succeeded'
            time.sleep(.15)
        return False

    def initial_depth_observation(self):
        """One bounded, leased observation sequence; never authorizes a chassis escape."""
        if not self.healthy():return False
        with self.motion.lock:
            pose=self.motion.pose();connection=initial_depth_connectivity(self.motion.grid,pose)
        evidence=dict(attempted=not connection['connected'],status='not_needed' if connection['connected'] else 'scanning',
                      before_connectivity=connection,history=[],policy='measured-depth-startup-scan-v1')
        self.data['initial_observation']=evidence;self.save()
        if connection['connected']:return True
        if not callable(getattr(self.motion,'pan_tilt',None)):
            evidence['status']='blocked';self.stop('当前模型不支持启动云台观察，已停止','blocked');return False
        deadline=min(self.data['started']+self.data['budget_seconds'],self.now()+INITIAL_DEPTH_SCAN_SECONDS)
        wall_deadline=time.monotonic()+180.
        start=getattr(self.motion,'gimbal_position',None) or [0.,0.]
        waypoints=[(0.,-math.radians(15)),(0.,-math.radians(30)),(0.,-math.radians(35)),
                   (math.pi/6,-math.radians(35)),(math.pi/3,-math.radians(35)),
                   (math.pi/2,-math.radians(35)),(2*math.pi/3,-math.radians(35)),
                   (2.356,-math.radians(35)),(-2.356,-math.radians(35))]
        targets=list(bounded_gimbal_targets(start,waypoints));restoring=False;previous=list(start)
        while targets:
            if not self.healthy():return False
            pan,tilt=targets.pop(0)
            seconds=max(abs(pan-previous[0]),abs(tilt-previous[1]))/1.5+5.
            decision=dict(x=pose['x'],y=pose['y'],yaw=wrap(pose['yaw']+pan),kind='turn',pan=pan,tilt=tilt,
                          use_pan_tilt=True,bootstrap=True,estimated_seconds=seconds,
                          bootstrap_deadline_sim=deadline,bootstrap_deadline_wall=wall_deadline)
            if self.now()+seconds>deadline or time.monotonic()>=wall_deadline or not self.has_action_budget(decision):
                evidence['status']='budget_reached';self.stop('启动观察的时间预算已用尽，已停止','budget_reached');return False
            with self.motion.lock:
                before=dict(self.motion.quality);grid=self.motion.grid
                before_grid=grid.header.stamp.sec+grid.header.stamp.nanosec/1e9
            record=dict(decision,started_sim=self.now(),outcome='running')
            evidence['history'].append(record)
            self.data.update(phase='initial_observation',reason='原地云台补采近车地面，等待真实观测连通通路',decision=decision)
            self.save()
            if not self.execute(decision,turn=True):
                if self.revisit_interrupt_pending():
                    record.update(outcome='interrupted',motion_error='定位预算触发回访；原云台目标未恢复')
                    if not self.perform_localization_revisit():return False
                    continue
                record.update(outcome='failed',motion_error=self.motion.state().get('error'))
                evidence['status']='blocked' if self.data['active'] else self.data['phase']
                if self.data['active']:self.stop('启动云台观察未完成，已停止','blocked')
                return False
            completed=self.now();observed=False;quality=before;grid_stamp=before_grid
            wait_until=time.monotonic()+INITIAL_DEPTH_OBSERVATION_WALL_SECONDS
            while self.healthy() and self.now()<deadline and time.monotonic()<min(wait_until,wall_deadline):
                with self.motion.lock:
                    quality=dict(self.motion.quality);grid=self.motion.grid
                    grid_stamp=grid.header.stamp.sec+grid.header.stamp.nanosec/1e9
                    observed=(quality.get('frames',0)>before.get('frames',0)
                              and quality.get('stamp',0)>before.get('stamp',0)
                              and grid_stamp>max(before_grid,completed))
                if observed:break
                time.sleep(.15)
            if not self.data['active']:return False
            record.update(outcome='succeeded' if observed else 'no_new_observation',completed_sim=completed,
                          end_sim=self.now(),new_frames=quality.get('frames',0)-before.get('frames',0),source_grid_stamp=grid_stamp)
            if not observed:
                if self.now()>=deadline or time.monotonic()>=wall_deadline:
                    evidence['status']='budget_reached';self.stop('启动观察的时间预算已用尽，已停止','budget_reached');return False
                evidence['status']='blocked';self.stop('启动观察未取得新的深度帧及导航地图，已停止','blocked');return False
            with self.motion.lock:
                connection=initial_depth_connectivity(self.motion.grid,self.motion.pose())
            record['connectivity']=connection;evidence['after_connectivity']=connection;self.save();previous=[pan,tilt]
            if restoring:
                if not targets:
                    if connection['connected']:
                        evidence['status']='connected';self.data['phase']='planning';self.save();return True
                    break
            elif connection['connected']:
                # Restore the ordinary forward camera convention before the
                # selected v2/v3 planner resumes. No body motion is requested.
                targets=list(bounded_gimbal_targets(previous,[(0.,-math.radians(15))]));restoring=True
                if not targets:
                    evidence['status']='connected';self.data['phase']='planning';self.save();return True
        evidence['status']='blocked';self.stop('启动观察后仍无已观测的连通通路，已停止','blocked');return False

    def observe_unknown_frontier(self, grid, pose, attempted):
        if not self.initial_depth_scan or not callable(getattr(self.motion,'pan_tilt',None)):
            return False
        targets=visible_unknown_frontiers(grid,pose,self.origin,self.data['radius'])
        self.data['visible_unknown_frontier_cells']=len(set().union(*targets.values()))
        if len(attempted)>=FRONTIER_SCANS_PER_STOP:
            return False
        position=getattr(self.motion,'gimbal_position',None) or [0.,0.]
        options=[p for p in targets if p not in attempted]
        if not options:
            return False
        pan=min(options,key=lambda p:(abs(p-position[0])<.12,-len(targets[p]),abs(p-position[0])))
        tilt=-.5236 if abs(pan-position[0])<.12 and abs(position[1]+.2618)<.12 else -.2618
        rate=getattr(self.motion,'gimbal_command_rate_rad_s',None) or 1.5
        seconds=math.hypot(pan-position[0],tilt-position[1])/rate+3.+FRONTIER_OBSERVATION_SECONDS
        decision=dict(x=pose['x'],y=pose['y'],yaw=wrap(pose['yaw']+pan),kind='turn',
                      use_pan_tilt=True,pan=pan,tilt=tilt,estimated_seconds=seconds,frontier_observation=True)
        if not self.has_action_budget(decision):
            self.stop('剩余时间不足以补观测未知边界，待探测区域保留','budget_reached')
            return False
        attempted.add(pan)
        with self.motion.lock:
            before=self.motion.quality
            before_stamp=grid.header.stamp.sec+grid.header.stamp.nanosec/1e9
        record=dict(decision,targeted_cells=sorted(targets[pan]),started_sim=self.now(),outcome='running')
        with self.lock:
            if not self.data['active']:return False
            self.data.setdefault('frontier_observations',[]).append(record)
            self.data.update(decision=decision,phase='observing',reason='从当前位置用云台补看未知边界，再根据新地图规划')
            self.save()
        if not self.execute(decision,turn=True):
            record.update(outcome='failed' if self.data['active'] else 'interrupted',
                          motion_error=self.motion.state().get('error'))
            self.save()
            return False
        settled=self.now()
        wall_deadline=time.monotonic()+INITIAL_DEPTH_OBSERVATION_WALL_SECONDS
        known=0;grid_stamp=before_stamp
        while self.healthy() and not self.revisit_interrupt_pending() and self.now()<settled+FRONTIER_OBSERVATION_SECONDS and time.monotonic()<wall_deadline:
            with self.motion.lock:
                current=self.motion.grid;quality=self.motion.quality
                if current is not None and self.motion.grid_fresh():
                    grid_stamp=current.header.stamp.sec+current.header.stamp.nanosec/1e9
                    if (grid_stamp>max(before_stamp,settled) and quality.get('frames',0)>before.get('frames',0)
                            and quality.get('stamp',0)>before.get('stamp',0)):
                        known=newly_known_cells(current,targets[pan])
            if known:break
            time.sleep(.15)
        record.update(outcome=('observed' if known else 'no_new_evidence') if self.data['active'] else 'interrupted',
                      newly_known_cells=known,source_grid_stamp=grid_stamp,end_sim=self.now())
        self.save()
        return bool(known)

    def run(self):
        self.wall_started=time.monotonic();history=[];failures=0
        frontier_attempts=set()
        self.data.setdefault('recovery_attempts',0)
        self.data.setdefault('execution_policy',EXECUTION_POLICY)
        try:
            if self.initial_depth_scan and not self.initial_depth_observation():return
            while self.healthy():
                if self.revisit_interrupt_pending():
                    if not self.perform_localization_revisit():break
                    failures=0
                    continue
                with self.motion.lock:
                    grid=self.motion.grid;quality=self.motion.quality;pose=self.motion.pose()
                    if not self.motion.grid_fresh():
                        raise ValueError('导航地图未更新')
                before=quality
                strategy=self.data.get('strategy','quality_view_heuristic_v1')
                if failures>=MAX_CONSECUTIVE_ACTION_FAILURES:
                    if history[-1].get('dispatch_error','').startswith('目标尚未观测'):
                        attempts=len(frontier_attempts)
                        observed=self.observe_unknown_frontier(grid,pose,frontier_attempts)
                        if not self.data['active']:break
                        if observed:failures=0
                        if len(frontier_attempts)>attempts or self.revisit_interrupt_pending():continue
                    if (strategy not in V3_STRATEGIES or
                            self.data['recovery_attempts']>=MAX_RECOVERY_SCANS or
                            not callable(getattr(self.motion,'pan_tilt',None))):
                        self.stop('连续三个候选动作未能执行，恢复机会已用尽或不可用；已停止，请检查场地与导航','fault')
                        break
                    # Reobserve from the parked pose. This cannot authorize a chassis
                    # escape, relax cost thresholds or remove failed-target exclusions.
                    pan=math.pi/2
                    decision=dict(x=pose['x'],y=pose['y'],yaw=wrap(pose['yaw']+pan),
                                  kind='turn',pan=pan,tilt=-0.2618,use_pan_tilt=True,recovery=True,
                                  expected_new_m2=0.,expected_improvement_m2=0.,expected_utility_m2=0.,
                                  visible_cells=[],strategy=strategy,estimated_seconds=RECOVERY_SCAN_SECONDS,
                                  recovery_trigger_failures=failures)
                    diagnostics=dict(budget_limited=False,execution_policy=EXECUTION_POLICY,
                                     recovery_attempt=self.data['recovery_attempts']+1)
                else:
                    decision,diagnostics=choose_view(grid,quality,pose,self.origin,self.data['radius'],history,
                                         self.data['budget_seconds']-(self.now()-self.data['started'])-EXECUTION_MARGIN_SECONDS,
                                         strategy,return_diagnostics=True,
                                         depth_observed_visibility=self.initial_depth_scan)
                if not decision:
                    self.data['candidate_diagnostics']=diagnostics
                    if diagnostics['budget_limited']:
                        self.stop('剩余时间不足以完成下一候选观察动作','budget_reached')
                    else:
                        attempts=len(frontier_attempts)
                        self.observe_unknown_frontier(grid,pose,frontier_attempts)
                        if not self.data['active']:break
                        if len(frontier_attempts)>attempts or self.revisit_interrupt_pending():continue
                        self.stop('暂无新的安全观察位置；未观测区域保留待探测','no_candidates')
                    break
                if not self.has_action_budget(decision):
                    diagnostics['execution_budget_recheck_rejected']=True
                    self.data['candidate_diagnostics']=diagnostics
                    self.stop('候选计算后剩余时间不足以完成下一观察动作','budget_reached');break
                decision['candidate_diagnostics']=diagnostics
                names={'turn':'云台扫描' if decision.get('use_pan_tilt') else '转向观察','advance':'前进探索','resample':'换位补采'}
                if not self.healthy():break
                with self.lock:
                    if not self.data['active']:break
                    reason=('连续动作失败，原地云台重新观测后再规划' if decision.get('recovery') else
                            f"{names[decision['kind']]}：预计新增 {decision['expected_new_m2']:.2f} m²，补采收益 {decision['expected_improvement_m2']:.2f} m²；预计 {decision['estimated_seconds']:.0f} 仿真秒")
                    self.data.update(decision=decision,phase='planning',reason=reason)
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
                    if decision.get('use_pan_tilt') or angle>.12:
                        if decision.get('recovery'):self.data['recovery_attempts']+=1
                        success=self.execute(decision,turn=True)
                if success and self.healthy():
                    self.data['phase']='observing';until=self.now()+3
                    while self.healthy() and self.now()<until:time.sleep(.2)
                current=self.motion.quality
                if decision.get('recovery') and success and self.data['active']:
                    if current.get('frames',0)<=before.get('frames',0):
                        success=False
                        decision['observation_error']='恢复扫描未取得新观测，已停止'
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
                                  motion_error=decision.get('observation_error',decision.get('dispatch_error',motion_state['error'])),
                                  motion_stop=None if 'dispatch_error' in decision else motion_state.get('stop_diagnostics'),
                                  health=motion_state.get('diagnostics'))
                    self.save()
                if not self.data['active']:break
                if self.revisit_interrupt_pending():
                    decision['blocked']=True
                    record.update(outcome='interrupted',success=False,retry_blocked=True,
                                  motion_error='定位预算触发主动回访；原探索动作不会恢复')
                    self.save()
                    if not self.perform_localization_revisit():break
                    failures=0
                    continue
                if decision.get('recovery'):
                    if not success:
                        self.stop('原地重新观测失败，已停止；请检查传感器与导航','fault')
                        break
                    failures=0
                    continue
                if not success:
                    decision['blocked']=True
                    record['retry_blocked']=True
                # A successful stationary scan does not prove failed navigation recovered.
                if not success:failures+=1
                elif decision['kind']!='turn':
                    failures=0
                    frontier_attempts.clear()
        except Exception as error:self.stop(f'自主探测已停止：{error}','fault')
        finally:
            if self.data['active']:self.stop('自主探测结束')
            with self.lock:
                if self.data['history'] and self.data['history'][-1].get('outcome')=='running':
                    self.data['history'][-1].update(outcome='interrupted',success=False,motion_error=self.data.get('reason',''))
                initial=self.data.get('initial_observation')
                if initial and initial['history'] and initial['history'][-1].get('outcome')=='running':
                    initial['history'][-1].update(outcome='interrupted',motion_error=self.data.get('reason',''))
                    initial['status']=self.data['phase']
                self.save()
            if getattr(self,'recorder',None):
                try:self.recorder.finish()
                except Exception as error:
                    self.data['evidence_error']=str(error);self.save()
