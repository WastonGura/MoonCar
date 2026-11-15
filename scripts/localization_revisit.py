"""Planning-only RGBD covariance reserve and measured-view revisit evidence.

No value from this module enters an estimator. A saved observation is only a
candidate target until the caller's current fresh navigation grid validates it.
"""
import heapq
import math

MAX_XY_SIGMA_M = .05
WARN_TRACE_SIGMA_M = MAX_XY_SIGMA_M / math.sqrt(2.)
SAVED_TRACE_SIGMA_M = MAX_XY_SIGMA_M / 2.
NOISE_FLOOR_M = .003
REQUIRED_NEW_EXPOSURES = 3
MAX_REVISITS = 4
MAX_OBSERVATIONS = 64
MAX_BREADCRUMBS = 512
MAX_CHECKPOINT_VALIDATIONS = 4
MAX_RETURN_GOALS = 8
MAX_MERGE_PROBES = 32
# This is the existing parent's final distance-to-saved-observation guard.
SAVED_ARRIVAL_TOLERANCE_M = .25
MAX_NEARBY_CELL_PROBES = 64


class RetiredCheckpoint(ValueError):
    """An actual reference-library event invalidated this planning target."""
    def __init__(self,reference_id,stamp):
        super().__init__('Saved revisit reference was retired')
        self.reference_id=reference_id;self.stamp=stamp


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def sample_payload(payload, health):
    """Copy an already validated payload under the consumer's existing lock."""
    if not isinstance(payload, dict) or payload.get('localization_backend') != 'metric_rgbd_3d': return None
    fit = payload.get('last_fit')
    if (not isinstance(health, dict) or not health.get('healthy') or not payload.get('healthy')
            or not isinstance(fit, dict) or fit.get('accepted') is not True
            or payload.get('session_id') != health.get('session_id')
            or payload.get('sequence') != health.get('sequence')):
        raise ValueError('RGBD revisit requires the current validated healthy producer payload')
    stamp = payload.get('accepted_stamp')
    std = fit.get('position_std_m')
    ref = fit.get('reference_id')
    if (not finite(stamp) or fit.get('exposure_stamp', fit.get('stamp')) != stamp
            or type(ref) is not int or ref < 1 or not isinstance(std, (tuple, list)) or len(std) != 3
            or not all(finite(v) and v >= 0 for v in std)):
        raise ValueError('RGBD revisit uncertainty is missing or belongs to another exposure')
    upper2 = std[0]**2 + std[1]**2
    return dict(session_id=payload['session_id'], sequence=payload['sequence'], stamp=stamp,
                reference_id=ref, position_std_m=list(std), trace_xy_sigma_upper_m=math.sqrt(upper2),
                xy_variance_upper_m2=upper2, xy_variance_lower_m2=max(std[0]**2, std[1]**2),
                fault_counter=health.get('fault_counter'),
                retired_reference_id=((fit.get('view_admission') or {}).get('anchor_adoption') or {}).get('retired_reference_id'))


def line_cells(a, b, grid):
    """Conservative grid supercover, including both neighbors at exact corners."""
    info = grid.info; r = info.resolution; origin = info.origin.position
    ax, ay = (a['x']-origin.x)/r, (a['y']-origin.y)/r
    bx, by = (b['x']-origin.x)/r, (b['y']-origin.y)/r
    x, y, ex, ey = math.floor(ax), math.floor(ay), math.floor(bx), math.floor(by)
    cells = [(x, y)]
    dx, dy = bx-ax, by-ay
    sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
    tx = ((x+1-ax) if dx > 0 else (ax-x))/abs(dx) if dx else math.inf
    ty = ((y+1-ay) if dy > 0 else (ay-y))/abs(dy) if dy else math.inf
    ux, uy = 1/abs(dx) if dx else math.inf, 1/abs(dy) if dy else math.inf
    while (x, y) != (ex, ey):
        # Endpoint roundoff can put one coordinate just below its boundary.
        # A straight segment never leaves an already reached destination cell
        # on that axis; endpoint supercover is added explicitly below.
        nextx=tx if x!=ex else math.inf;nexty=ty if y!=ey else math.inf
        if abs(nextx-nexty) <= 1e-12:
            cells.extend(((x+sx, y), (x, y+sy))); x += sx; y += sy; tx += ux; ty += uy
        elif nextx < nexty: x += sx; tx += ux
        else: y += sy; ty += uy
        cells.append((x, y))
        if len(cells) > 3*(abs(ex-math.floor(ax))+abs(ey-math.floor(ay))+3):
            raise ValueError('Invalid revisit segment traversal')
    if dx == 0 and abs(ax-round(ax)) <= 1e-12:
        cells += [(int(round(ax))+side,y) for _,y in list(cells) for side in (-1,0)]
    if dy == 0 and abs(ay-round(ay)) <= 1e-12:
        cells += [(x,int(round(ay))+side) for x,_ in list(cells) for side in (-1,0)]
    for px,py in ((ax,ay),(bx,by)):
        onx,ony=abs(px-round(px))<=1e-12,abs(py-round(py))<=1e-12
        ix=int(round(px)) if onx else math.floor(px)
        iy=int(round(py)) if ony else math.floor(py)
        cells.append((ix,iy))
        if onx:cells.append((ix-1,iy))
        if ony:cells.append((ix,iy-1))
        if onx and ony:cells.append((ix-1,iy-1))
    return list(dict.fromkeys(cells))


def safe_segment(a, b, grid, costs, clearance, within_region, region, origin, radius):
    radius = math.inf if radius is None else radius
    info = grid.info; r = info.resolution; offset = info.origin.position
    yaw = math.atan2(b['y']-a['y'], b['x']-a['x'])
    for x, y in line_cells(a, b, grid):
        if not (0 <= x < info.width and 0 <= y < info.height and 0 <= costs[y, x] < 99): return False
        px, py = offset.x+(x+.5)*r, offset.y+(y+.5)*r
        if (not clearance(costs, x, y, r) or not within_region(region, px, py, yaw)
                or math.hypot(px-origin[0], py-origin[1]) > radius-.75
                or abs(px) > 24.3 or abs(py) > 24.3): return False
    return True


class RGBDRevisitPolicy:
    def __init__(self):
        self.session = None; self.last_stamp = None; self.observations = []; self.breadcrumbs = []
        self.pending = None; self.running = False; self.attempts = 0; self.proof_stamps = []
        self.retirement_generation=0
        self.last_observation_stamp=None;self.observation_context=None
        self.retired_observation_ids=[];self.plan_diagnostics={}

    def observe(self, sample, observation):
        if self.session is None: self.session = sample['session_id']
        if sample['session_id'] != self.session: raise ValueError('RGBD revisit producer changed')
        if self.pending is not None and sample['fault_counter'] != self.pending['trigger']['fault_counter']:
            raise ValueError('Localization outage occurred during active revisit')
        if self.last_stamp is not None and sample['stamp'] < self.last_stamp:
            raise ValueError('RGBD revisit exposure clock moved backwards')
        if sample['stamp'] == self.last_stamp: return
        self.last_stamp = sample['stamp']
        self.observation_context=dict(sample)
        retired = sample['retired_reference_id']
        if retired is not None:
            self.retired_observation_ids=(self.retired_observation_ids+[retired])[-MAX_OBSERVATIONS:]
            self.observations = [row for row in self.observations if row['reference_id'] != retired]
            if self.running and self.pending.get('checkpoint', {}).get('reference_id') == retired:
                raise RetiredCheckpoint(retired,sample['stamp'])
        if observation is not None:self.attach_observation(sample,observation)
        if not self.running and self.pending is None and sample['trace_xy_sigma_upper_m'] >= WARN_TRACE_SIGMA_M:
            self.pending = dict(trigger=sample)

    def attach_observation(self,sample,observation):
        """Attach one real late exact-exposure pose, without replaying events."""
        context=self.observation_context
        keys=('session_id','stamp','fault_counter','reference_id')
        if (context is None or sample.get('session_id')!=self.session or sample.get('stamp')!=self.last_stamp
                or any(sample.get(key)!=context.get(key) for key in keys)):
            raise ValueError('Late revisit observation does not belong to the latest healthy exposure')
        if (not isinstance(observation,dict) or observation.get('stamp')!=sample['stamp']
                or not all(finite(observation.get(key)) for key in ('stamp','x','y','yaw','pan','tilt'))):
            raise ValueError('Revisit pose and camera view exposure differ or are invalid')
        if sample['reference_id'] in self.retired_observation_ids:
            raise ValueError('Late revisit observation reference was retired')
        if self.last_observation_stamp==sample['stamp']:return False
        # A late pose cannot replace any accepted uncertainty or producer fact.
        row=dict(context)
        for key in ('stamp','x','y','yaw','pan','tilt','joint_feedback_stamp',
                    'joint_feedback_wall_age_s','camera_tf_stamp','captured_sim'):
            if key in observation:row[key]=observation[key]
        if (not self.breadcrumbs or math.hypot(row['x']-self.breadcrumbs[-1]['x'],row['y']-self.breadcrumbs[-1]['y'])>=.10
                or abs(math.atan2(math.sin(row['yaw']-self.breadcrumbs[-1]['yaw']),math.cos(row['yaw']-self.breadcrumbs[-1]['yaw'])))>=math.pi/18):
            self.breadcrumbs.append(row);self.breadcrumbs=self.breadcrumbs[-MAX_BREADCRUMBS:]
        if context['trace_xy_sigma_upper_m']<=SAVED_TRACE_SIGMA_M:
            if (not self.observations or self.observations[-1]['reference_id']!=row['reference_id']
                    or math.hypot(row['x']-self.observations[-1]['x'],row['y']-self.observations[-1]['y'])>=.10
                    or abs(row['pan']-self.observations[-1]['pan'])>=.10 or abs(row['yaw']-self.observations[-1]['yaw'])>=.10):
                self.observations.append(row);self.observations=self.observations[-MAX_OBSERVATIONS:]
        self.last_observation_stamp=sample['stamp']
        return True

    def plan(self, current, grid, costs, clearance, within_region, region, origin, radius, remaining, gimbal):
        radius = math.inf if radius is None else radius
        self.plan_diagnostics=dict(mode='normal_exact',observations=len(self.observations),breadcrumbs=len(self.breadcrumbs),
            cheap_examined=0,cheap_eligible=0,route_validations=0,rejections=[],remaining_seconds=remaining)
        if self.pending is None: return None
        if self.attempts >= MAX_REVISITS: raise ValueError('RGBD revisit attempt limit reached')
        if self.retirement_generation and self.pending.get('retirement_event') is not None:
            self.plan_diagnostics['mode']='retired_nearby'
            return self._plan_retired_nearby(current,grid,costs,clearance,within_region,region,origin,radius,remaining)
        choices = []; clearance_cache={};geometry_checks=0
        def cached_clearance(data,x,y,resolution):
            key=(x,y)
            if key not in clearance_cache:clearance_cache[key]=clearance(data,x,y,resolution)
            return clearance_cache[key]
        def segment(a,b):
            nonlocal geometry_checks
            geometry_checks+=1
            return safe_segment(a,b,grid,costs,cached_clearance,within_region,region,origin,radius)
        ordered=sorted(self.observations,key=lambda row:(math.hypot(row['x']-current['x'],row['y']-current['y']),row['trace_xy_sigma_upper_m']))
        eligible=[]
        def reject(checkpoint,reason):
            self.plan_diagnostics['rejections'].append(dict(stamp=checkpoint['stamp'],reference_id=checkpoint['reference_id'],reason=reason))
        for checkpoint in ordered[:MAX_OBSERVATIONS]:
            self.plan_diagnostics['cheap_examined']+=1
            if self.breadcrumbs and checkpoint['stamp'] < self.breadcrumbs[0]['stamp']:
                reject(checkpoint,'older_than_breadcrumbs');continue
            route = [current] + list(reversed([row for row in self.breadcrumbs if row['stamp'] > checkpoint['stamp']])) + [checkpoint]
            dedup = [route[0]]
            for row in route[1:]:
                if math.hypot(row['x']-dedup[-1]['x'], row['y']-dedup[-1]['y']) >= .02: dedup.append(row)
            if len(dedup) == 1: dedup.append(checkpoint)
            ix = math.floor((checkpoint['x']-grid.info.origin.position.x)/grid.info.resolution)
            iy = math.floor((checkpoint['y']-grid.info.origin.position.y)/grid.info.resolution)
            if not (0 <= ix < grid.info.width and 0 <= iy < grid.info.height):reject(checkpoint,'target_bounds');continue
            if not 0 <= costs[iy, ix] <= 65:reject(checkpoint,'target_cost');continue
            if not cached_clearance(costs,ix,iy,grid.info.resolution):reject(checkpoint,'target_clearance');continue
            px=grid.info.origin.position.x+(ix+.5)*grid.info.resolution
            py=grid.info.origin.position.y+(iy+.5)*grid.info.resolution
            end_yaw=math.atan2(dedup[-1]['y']-dedup[-2]['y'],dedup[-1]['x']-dedup[-2]['x'])
            if not within_region(region,px,py,end_yaw):reject(checkpoint,'target_region');continue
            if math.hypot(px-origin[0],py-origin[1])>radius-.75:reject(checkpoint,'target_radius');continue
            if abs(px)>24.3 or abs(py)>24.3:reject(checkpoint,'target_world_bounds');continue
            eligible.append((checkpoint,dedup))
        self.plan_diagnostics['cheap_eligible']=len(eligible)
        self.plan_diagnostics['route_cap_excluded']=max(0,len(eligible)-MAX_CHECKPOINT_VALIDATIONS)
        for checkpoint,dedup in eligible[:MAX_CHECKPOINT_VALIDATIONS]:
            self.plan_diagnostics['route_validations']+=1
            if not all(segment(a,b) for a,b in zip(dedup,dedup[1:])):reject(checkpoint,'route_segment');continue
            # Merge only after checking every crossed cell, using the same
            # conservative chassis function as ordinary survey targets.
            goals = []; index = 0;merge_probes=0
            while index < len(dedup)-1:
                def target_cost_ok(point):
                    x=math.floor((point['x']-grid.info.origin.position.x)/grid.info.resolution)
                    y=math.floor((point['y']-grid.info.origin.position.y)/grid.info.resolution)
                    return 0<=x<grid.info.width and 0<=y<grid.info.height and 0<=costs[y,x]<=65
                selected=None
                for j in range(len(dedup)-1,index,-1):
                    if merge_probes>=MAX_MERGE_PROBES:break
                    merge_probes+=1
                    if target_cost_ok(dedup[j]) and segment(dedup[index],dedup[j]):selected=j;break
                if selected is None:break
                index=selected;goals.append(dedup[index])
                if len(goals)>=MAX_RETURN_GOALS and index<len(dedup)-1:break
            if index != len(dedup)-1:reject(checkpoint,'route_merge_limit');continue
            length = sum(math.hypot(b['x']-a['x'], b['y']-a['y']) for a, b in zip([current]+goals, goals))
            # Bound view restoration by a full body half-turn plus the full
            # legal gimbal pan/tilt range, rather than a guessed future heading.
            view_seconds = math.pi/.5 + math.hypot(2*2.356, .873)/.35 + 2*6.
            seconds = length/.06 + len(goals)*9. + view_seconds + 3. + 4.
            if seconds > remaining:reject(checkpoint,'remaining_budget');continue
            choices.append(dict(checkpoint=checkpoint, goals=goals, route_length_m=length,
                                estimated_seconds=seconds, policy='measured-rgbd-revisit-v1',
                                geometry_checks=geometry_checks,clearance_cells=len(clearance_cache),merge_probes=merge_probes,
                                budget=dict(translation_speed_m_s=.06, per_navigation_action_s=9.,
                                            view_seconds=view_seconds, proof_and_final_margin_s=7.)))
        if not choices: return None
        return min(choices, key=lambda row:(row['estimated_seconds'], row['checkpoint']['trace_xy_sigma_upper_m']))

    def _plan_retired_nearby(self,current,grid,costs,clearance,within_region,region,origin,radius,remaining):
        radius = math.inf if radius is None else radius
        """Plan a new legal viewpoint near an unchanged saved real observation.

        Only an actual typed retirement transaction enables this bounded search.
        The saved pose, exposure, view and uncertainty remain measurement facts;
        a nearby parking goal is never appended to observation history.
        """
        info=grid.info;r=info.resolution;offset=info.origin.position
        clearance_cache={};target_checks=0
        def cached_clearance(data,x,y,resolution):
            key=(x,y)
            if key not in clearance_cache:clearance_cache[key]=clearance(data,x,y,resolution)
            return clearance_cache[key]
        def target_ok(point,checkpoint):
            nonlocal target_checks
            target_checks+=1
            ix=math.floor((point['x']-offset.x)/r);iy=math.floor((point['y']-offset.y)/r)
            return (0<=ix<info.width and 0<=iy<info.height and 0<=costs[iy,ix]<=65
                    and math.hypot(point['x']-checkpoint['x'],point['y']-checkpoint['y'])<=SAVED_ARRIVAL_TOLERANCE_M
                    and cached_clearance(costs,ix,iy,r)
                    and within_region(region,point['x'],point['y'],checkpoint['yaw'])
                    and math.hypot(point['x']-origin[0],point['y']-origin[1])<=radius-.75
                    and abs(point['x'])<=24.3 and abs(point['y'])<=24.3)
        def nearest_target(checkpoint):
            # Best-first centers avoid constructing an unbounded square when a
            # grid has very small cells. At most64 centers per saved candidate.
            sx=math.floor((checkpoint['x']-offset.x)/r);sy=math.floor((checkpoint['y']-offset.y)/r)
            queue=[];seen={(sx,sy)}
            def push(x,y):
                px,py=offset.x+(x+.5)*r,offset.y+(y+.5)*r
                heapq.heappush(queue,(math.hypot(px-checkpoint['x'],py-checkpoint['y']),x,y,px,py))
            push(sx,sy);points=[dict(x=checkpoint['x'],y=checkpoint['y'])]
            if math.hypot(current['x']-checkpoint['x'],current['y']-checkpoint['y'])<=SAVED_ARRIVAL_TOLERANCE_M:
                points.append(dict(x=current['x'],y=current['y']))
            for _ in range(MAX_NEARBY_CELL_PROBES):
                if not queue:break
                distance,x,y,px,py=heapq.heappop(queue)
                if distance>SAVED_ARRIVAL_TOLERANCE_M:break
                points.append(dict(x=px,y=py))
                for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                    cell=(x+dx,y+dy)
                    if cell not in seen:seen.add(cell);push(*cell)
            points.sort(key=lambda point:(math.hypot(point['x']-checkpoint['x'],point['y']-checkpoint['y']),
                                          math.hypot(point['x']-current['x'],point['y']-current['y'])))
            for point in points:
                if target_ok(point,checkpoint):return dict(point,kind='near_saved_observation')
            return None
        ordered=sorted(self.observations,key=lambda row:(math.hypot(row['x']-current['x'],row['y']-current['y']),row['trace_xy_sigma_upper_m']))
        cheap=[]
        for checkpoint in ordered[:MAX_OBSERVATIONS]:
            standpoint=nearest_target(checkpoint)
            if standpoint is not None:cheap.append((checkpoint,standpoint))
        # An invalid old target cannot consume the entire route-check budget.
        # Prefer distinct real references before another saved view of one ref.
        unique=[];duplicates=[];refs=set()
        for choice in cheap:
            if choice[0]['reference_id'] in refs:duplicates.append(choice)
            else:unique.append(choice);refs.add(choice[0]['reference_id'])
        choices=[];geometry_checks=0
        for checkpoint,standpoint in (unique+duplicates)[:MAX_CHECKPOINT_VALIDATIONS]:
            geometry_checks+=1
            if not safe_segment(current,standpoint,grid,costs,cached_clearance,within_region,region,origin,radius):continue
            length=math.hypot(standpoint['x']-current['x'],standpoint['y']-current['y'])
            view_seconds=math.pi/.5+math.hypot(2*2.356,.873)/.35+2*6.
            seconds=length/.06+9.+view_seconds+3.+4.
            if seconds>remaining:continue
            choices.append(dict(checkpoint=checkpoint,planned_standpoint=standpoint,goals=[standpoint],
                                route_length_m=length,estimated_seconds=seconds,policy='measured-rgbd-revisit-v1',
                                planning_mode='retired_nearby_direct_v1',
                                saved_pose_offset_m=math.hypot(standpoint['x']-checkpoint['x'],standpoint['y']-checkpoint['y']),
                                cheap_saved_candidates=min(len(ordered),MAX_OBSERVATIONS),cheap_legal_candidates=len(cheap),
                                target_checks=target_checks,geometry_checks=geometry_checks,clearance_cells=len(clearance_cache),merge_probes=0,
                                budget=dict(translation_speed_m_s=.06,per_navigation_action_s=9.,view_seconds=view_seconds,
                                            proof_and_final_margin_s=7.)))
        if not choices:return None
        for choice in choices:
            choice.update(geometry_checks=geometry_checks,clearance_cells=len(clearance_cache))
        return min(choices,key=lambda row:(row['estimated_seconds'],row['checkpoint']['trace_xy_sigma_upper_m']))

    def unknown_clearance_scan(self,current,grid,costs,clearance,within_region,region,origin,radius,remaining,gimbal,*,gimbal_rate=.35):
        """Aim once at real unknown cells; this never declares them traversable."""
        radius = math.inf if radius is None else radius
        if (self.pending is None or self.running or self.pending.get('retirement_event') is not None
                or self.plan_diagnostics.get('mode')!='normal_exact' or self.plan_diagnostics.get('cheap_eligible')!=0
                or self.attempts>=MAX_REVISITS-1 or not finite(gimbal_rate) or gimbal_rate<=0
                or not isinstance(gimbal,(list,tuple)) or len(gimbal)!=2 or not all(finite(v) for v in gimbal)):
            return None
        if not safe_segment(current,current,grid,costs,clearance,within_region,region,origin,radius):return None
        info=grid.info;r=info.resolution;offset=info.origin.position;n=math.ceil(.65/r)
        rejected={(row['stamp'],row['reference_id']) for row in self.plan_diagnostics.get('rejections',[])
                  if row.get('reason')=='target_clearance'}
        ordered=sorted(self.observations,key=lambda row:(math.hypot(row['x']-current['x'],row['y']-current['y']),row['trace_xy_sigma_upper_m']))
        for saved in ordered[:MAX_OBSERVATIONS]:
            if (saved['stamp'],saved['reference_id']) not in rejected:continue
            ix=math.floor((saved['x']-offset.x)/r);iy=math.floor((saved['y']-offset.y)/r)
            if (ix<n or iy<n or ix+n>=info.width or iy+n>=info.height
                    or not 0<=costs[iy,ix]<=65 or clearance(costs,ix,iy,r)):continue
            if (not within_region(region,saved['x'],saved['y'],saved['yaw'])
                    or math.hypot(saved['x']-origin[0],saved['y']-origin[1])>radius-.75
                    or abs(saved['x'])>24.3 or abs(saved['y'])>24.3):continue
            unknown=[];hazard=False
            for y in range(iy-n,iy+n+1):
                for x in range(ix-n,ix+n+1):
                    if (x-ix)**2+(y-iy)**2>n*n:continue
                    if costs[y,x]>=99:hazard=True
                    elif costs[y,x]<0:unknown.append(dict(x=offset.x+(x+.5)*r,y=offset.y+(y+.5)*r,cell=[x,y]))
            if hazard or not unknown:continue
            aim=math.atan2(sum(p['y']-current['y'] for p in unknown),sum(p['x']-current['x'] for p in unknown))
            requested=math.atan2(math.sin(aim-current['yaw']),math.cos(aim-current['yaw']))
            pan=max(-2.356,min(2.356,requested))
            # Reuse the survey planner's conservative +/-30 degree horizontal
            # observation sector. New map data, rather than this aim, decides safety.
            if any(abs(math.atan2(math.sin(math.atan2(p['y']-current['y'],p['x']-current['x'])-current['yaw']-pan),
                                  math.cos(math.atan2(p['y']-current['y'],p['x']-current['x'])-current['yaw']-pan)))>math.pi/6 for p in unknown):continue
            tilt=saved['tilt']
            if not finite(tilt) or not -.611<=tilt<=.262:continue
            scan_seconds=math.hypot(pan-gimbal[0],tilt-gimbal[1])/gimbal_rate+6.
            view_seconds=math.pi/.5+math.hypot(2*2.356,.873)/.35+12.
            reserve=math.hypot(saved['x']-current['x'],saved['y']-current['y'])/.06+9.+view_seconds+7.
            if scan_seconds+8.+reserve+4.>remaining:continue
            return dict(checkpoint=saved,pan=pan,tilt=tilt,unknown_cells=unknown,estimated_seconds=scan_seconds,
                        fresh_wait_sim_s=8.,fresh_wait_wall_s=15.,return_reserve_seconds=reserve,gimbal_rate=gimbal_rate)
        return None

    def begin(self, plan):
        self.pending.update(plan); self.running = True; self.attempts += 1; self.proof_stamps = []

    def invalidate_retired_checkpoint(self,event):
        if (not isinstance(event,RetiredCheckpoint) or not self.running or self.pending is None
                or self.pending.get('checkpoint',{}).get('reference_id')!=event.reference_id):
            raise ValueError('Only the active actually retired checkpoint can be replanned')
        trigger=self.pending['trigger']
        self.pending=dict(trigger=trigger,retirement_event=dict(reference_id=event.reference_id,stamp=event.stamp))
        self.running=False;self.proof_stamps=[]
        self.retirement_generation+=1

    def prove(self, sample, arrived_stamp):
        checkpoint = self.pending['checkpoint']; trigger = self.pending['trigger']
        if sample['stamp'] <= arrived_stamp: return False
        qualifies = (sample['session_id'] == checkpoint['session_id']
                     and sample['reference_id'] == checkpoint['reference_id']
                     and sample['fault_counter'] == trigger['fault_counter']
                     and sample['trace_xy_sigma_upper_m'] <= checkpoint['trace_xy_sigma_upper_m']+NOISE_FLOOR_M
                     and sample['xy_variance_upper_m2'] <= trigger['xy_variance_lower_m2']-NOISE_FLOOR_M**2)
        if not qualifies: self.proof_stamps = []; return False
        if not self.proof_stamps or sample['stamp'] > self.proof_stamps[-1]: self.proof_stamps.append(sample['stamp'])
        return len(self.proof_stamps) >= REQUIRED_NEW_EXPOSURES

    def complete(self):
        self.pending = None; self.running = False; self.proof_stamps = []


class ReobservedAnchorProof:
    """Receipt evidence only: an old direct fit, then stable new observations.

    This independent sufficient proof keeps RGBDRevisitPolicy.prove unchanged.
    Neither the old-ID catalogue nor any covariance enters an estimator.
    """
    def __init__(self):
        self.session=None;self.last_stamp=None;self.qualified={};self.trigger=None
        self.eligible={};self.events=[];self.samples=[];self.proof_stamps=[];self.arrived=None
        self.health_epoch=None

    def consume(self,rows,trigger=None,health_epoch=None):
        if type(health_epoch) is not int or health_epoch<0:
            raise ValueError('RGBD evidence needs a validated consumer health epoch')
        if self.health_epoch is not None and health_epoch!=self.health_epoch:
            if self.trigger is not None:raise ValueError('RGBD evidence consumer health epoch changed')
            self.qualified={};self.last_stamp=None
        self.health_epoch=health_epoch
        for row in rows:
            if row.get('consumer_health_epoch')!=health_epoch:continue
            value=sample_payload(row['payload'],row['health'])
            if value is None:raise ValueError('RGBD evidence receipt has another backend')
            # The receipt consumer does not own MotionController's cumulative
            # counter. Do not invent a historical counter for these frames.
            value.pop('fault_counter',None);value['consumer_health_epoch']=health_epoch
            if self.session is None:self.session=value['session_id']
            if value['session_id']!=self.session:raise ValueError('RGBD evidence producer changed')
            if self.last_stamp is not None and value['stamp']<=self.last_stamp:
                raise ValueError('RGBD evidence exposure is not increasing')
            self.last_stamp=value['stamp']
            if self.trigger is None and trigger is not None and value['stamp']>=trigger['stamp']:
                self._freeze(trigger)
            if self.trigger is None:
                if value['trace_xy_sigma_upper_m']<=SAVED_TRACE_SIGMA_M:
                    self.qualified[value['reference_id']]=dict(value)
                    while len(self.qualified)>MAX_OBSERVATIONS:self.qualified.pop(next(iter(self.qualified)))
                continue
            if value['session_id']!=self.trigger['session_id']:
                raise ValueError('RGBD evidence health transaction changed')
            if value['stamp']<=self.trigger['stamp']:continue
            self.samples.append(value);self.samples=self.samples[-128:]
            fit=row['payload']['last_fit'];ref=value['reference_id']
            attempts=fit.get('reference_attempts',[])
            tracking=any(isinstance(item,dict) and item.get('reference_id')==ref
                         and item.get('accepted') is True and item.get('reason')=='accepted' for item in attempts)
            adoption=((fit.get('view_admission') or {}).get('anchor_adoption') or {})
            adopted=(adoption.get('adopted_reference_id')==ref==adoption.get('selected_reference_id')
                     and (tracking or adoption.get('additional_reference_id')==ref
                          and adoption.get('additional_reason')=='accepted'))
            claimed=adoption.get('adopted_reference_id')
            consistent=(('selected_reference_id' not in adoption or adoption['selected_reference_id']==ref)
                        and (claimed is None or claimed==ref==adoption.get('selected_reference_id')))
            if ref in self.eligible and consistent and (adopted if claimed is not None else tracking):
                self.events.append(dict(value,direct_fit_kind='adopted' if adopted else 'tracking',
                                        source_sequence=value['sequence']))
                self.events=self.events[-MAX_OBSERVATIONS:]
        if self.trigger is None and trigger is not None:self._freeze(trigger)

    def _freeze(self,trigger):
        if self.session is not None and trigger['session_id']!=self.session:
            raise ValueError('RGBD evidence trigger producer differs')
        if type(trigger.get('fault_counter')) is not int or trigger['fault_counter']<0:
            raise ValueError('RGBD evidence trigger has no actual motion fault counter')
        self.trigger=dict(trigger)
        self.eligible={ref:dict(value) for ref,value in self.qualified.items()
                       if value['stamp']<trigger['stamp'] and value['consumer_health_epoch']==self.health_epoch}

    def prove(self,sample,arrived_stamp,checkpoint):
        if self.trigger is None:return False
        trigger=self.trigger
        if type(sample.get('fault_counter')) is not int or sample['fault_counter']!=trigger['fault_counter']:
            self.proof_stamps=[];return False
        def qualifies(value):
            return (value['session_id']==trigger['session_id']
                    and value['trace_xy_sigma_upper_m']<=checkpoint['trace_xy_sigma_upper_m']+NOISE_FLOOR_M
                    and value['xy_variance_upper_m2']<=trigger['xy_variance_lower_m2']-NOISE_FLOOR_M**2)
        events=[event for event in self.events if event['stamp']>trigger['stamp'] and qualifies(event)]
        if not events:return False
        if self.arrived!=arrived_stamp:
            self.arrived=arrived_stamp;self.proof_stamps=[];self._stable_last_stamp=arrived_stamp
        for value in self.samples:
            if value['stamp']<=self._stable_last_stamp:continue
            self._stable_last_stamp=value['stamp']
            # A stable exposure must follow both the actual old fit and the
            # final restored camera view, never a heartbeat or earlier frame.
            if value['stamp']<=events[0]['stamp'] or not qualifies(value):self.proof_stamps=[]
            else:self.proof_stamps.append(value['stamp']);self.proof_stamps=self.proof_stamps[-REQUIRED_NEW_EXPOSURES:]
        if sample['stamp']<=arrived_stamp or not qualifies(sample):self.proof_stamps=[];return False
        return len(self.proof_stamps)>=REQUIRED_NEW_EXPOSURES

    def evidence(self):
        return dict(proof_kind='actual_old_fit_then_three_stable_exposures',
                    eligible_old_references={str(ref):dict(value) for ref,value in self.eligible.items()},
                    actual_old_fit_events=[dict(value) for value in self.events],
                    proof_exposure_stamps=list(self.proof_stamps))
