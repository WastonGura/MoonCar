"""Pure formal D=0 checks, pairing and step decisions; never publishes defaults."""
import math
from pathlib import Path
import json
import numpy as np
from r5_csv import read_numeric_csv

INPUT_FIELDS=('input_lx','input_ly','input_lz','input_ax','input_ay','input_az')
WHEELS=('front_left','front_right','rear_left','rear_right')

def pose(row):
    w,x,y,z=[float(row[n]) for n in ('qw','qx','qy','qz')]
    return dict(t=float(row['t']),xyz=[float(row[n]) for n in ('x','y','z')],
                rpy=[math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
                     math.asin(max(-1,min(1,2*(w*y-z*x)))),math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))])


def wheel_budget(active,linear,angular):
    values=np.column_stack([active[w+'_wheel_joint_velocity_cmd'] for w in WHEELS])
    left=(values[:,0]+values[:,2])/2;right=(values[:,1]+values[:,3])/2
    v=(left+right)*.076/2;omega=(right-left)*.076/.3311
    moving=np.flatnonzero(np.max(np.abs(values),axis=1)>1e-9)
    contiguous=bool(len(moving)>0 and np.diff(moving).max(initial=1)==1)
    limits=True
    for target,observed,acceleration in ((linear,v,.5),(angular,omega,1.)):
        limits=limits and bool(np.max(np.abs(observed))<=abs(target)+1e-8 and
            (np.max(np.abs(observed))<=1e-8 if target==0 else np.min(observed*np.sign(target))>=-1e-8))
        limits=limits and bool(np.max(np.abs(np.diff(observed)/active['dt'][1:]),initial=0)<=acceleration+1e-6)
    matched=bool(np.max(np.abs(values[:,0]-values[:,2]))<=1e-10 and np.max(np.abs(values[:,1]-values[:,3]))<=1e-10)
    zero_tail=bool(np.max(np.abs(values[-1]))<=1e-12)
    return dict(passed=bool(contiguous and limits and matched and zero_tail),continuous=contiguous,
        direction_and_acceleration_valid=bool(limits),front_rear_targets_match=matched,zero_tail=zero_tail,
        linear_integral_m=float(np.sum(v*active['dt'])),angular_integral_rad=float(np.sum(omega*active['dt'])),
        active_start_s=float(active['t'][moving[0]]) if len(moving) else None,
        active_end_s=float(active['t'][moving[-1]]+active['dt'][moving[-1]]) if len(moving) else None)

def command_window(data,result,linear=0.,angular=0.):
    active=data[(data['t']>=result['sim_started'])&(data['t']<=result['sim_ended'])]
    fields=INPUT_FIELDS+('input_sequence','input_seen_sim')+tuple(w+'_wheel_joint_velocity_cmd' for w in WHEELS)
    if len(active)<2 or not all(f in data.dtype.names for f in fields):
        return dict(passed=False,reason='Missing native command/servo fields')
    if not all(np.isfinite(active[f]).all() for f in fields):
        return dict(passed=False,reason='Non-finite native command/servo observations')
    fresh=bool((active['input_sequence']>0).all() and (np.diff(active['input_sequence'])>=0).all() and
               np.max(active['t']-active['input_seen_sim'])<=.100001)
    values=np.column_stack([active[f] for f in INPUT_FIELDS])
    zero=np.max(np.abs(values),axis=1)<=1e-12
    if linear==0 and angular==0:
        wheels_zero=all(np.max(np.abs(active[w+'_wheel_joint_velocity_cmd']))<=1e-12 for w in WHEELS)
        return dict(passed=fresh and bool(zero.all()) and wheels_zero,zero_input=bool(zero.all()),
                    zero_wheel_targets=wheels_zero,fresh=fresh)
    expected=np.array([linear,0,0,0,0,angular])
    moving=np.max(np.abs(values-expected),axis=1)<=1e-12
    indices=np.flatnonzero(moving)
    if not len(indices):return dict(passed=False,reason='No commanded motion window')
    first,last=int(indices[0]),int(indices[-1])
    contiguous=bool(np.diff(indices).max(initial=1)==1)
    if last+1>=len(active):return dict(passed=False,reason='Missing received stop')
    begin,end=active[first],active[last+1]
    start=float(begin['input_seen_sim']);stop=float(end['input_seen_sim']);duration=stop-start
    onset_error=start-result['sim_started']-5
    stop_zero=bool(zero[last+1:].all())
    sequence_valid=fresh and contiguous and bool((zero|moving).all()) and bool(zero[:first].all()) and stop_zero
    scheduled=result.get('schedule',{}).get('mode')=='paused_steps'
    step=float(result['schedule']['step']) if scheduled else None
    tolerance=step+1e-9 if scheduled else .05
    timing_valid=abs(onset_error)<=tolerance and abs(duration-20)<=tolerance
    budget=wheel_budget(active,linear,angular) if scheduled else None
    if scheduled:
        motion=result['schedule']['motion'];park=result['schedule']['parked']
        if abs(motion['after']['sim']-motion['before']['sim']-20)>1e-8:
            timing_valid=False
        endpoint=pose(motion['last_step']);start_pose=pose(park['last_step'])
        sequence_valid=sequence_valid and budget['passed']
    else:endpoint=pose(end);start_pose=pose(begin)
    return dict(passed=sequence_valid and timing_valid,fresh=fresh,contiguous=contiguous,
                onset_error_s=onset_error,duration_s=duration,start=start,stop=stop,stop_zero=stop_zero,
                endpoint=endpoint,start_pose=start_pose,mode='paused_steps' if scheduled else 'legacy',step=step,
                wheel_budget=budget,
                reason='' if sequence_valid and timing_valid else 'Received command sequence or timing anomaly')

def formal_observations(folder,result,kind,linear=0.,angular=0.):
    folder=Path(folder);data=read_numeric_csv(folder/'physics-steps.csv',delimiter=',',names=True)
    rows=[json.loads(line) for line in (folder/'observations.jsonl').open()]
    start,end=result['sim_started'],result['sim_ended']
    coverage={}
    for sensor,bound in (('rgb',1.5),('depth',1.5),('scan',.75),('imu',.1),('joints',.1),('truth',.1)):
        samples=[r for r in rows if r['kind']==sensor]
        times=sorted(r['stamp'] for r in samples)
        covered=[t for t in times if start-bound<=t<=end+bound]
        gaps=np.diff(covered)
        complete=len(covered)>1 and covered[0]<=start+bound and covered[-1]>=end-bound and gaps.max(initial=0)<=bound+1e-6
        files=all((folder/r['file']).is_file() for r in samples if 'file' in r)
        coverage[sensor]=dict(passed=bool(complete and files),samples=len(samples),
                              max_gap_s=float(gaps.max(initial=0)),files_present=files)
    imagery=any(r.get('finite',0)>0 for r in rows if r['kind']=='depth') and any(
        r.get('pixel_std',0)>.5 for r in rows if r['kind']=='rgb')
    commands=command_window(data,result,linear if kind=='drive' else 0.,angular if kind=='drive' else 0.)
    confirmations=[]
    for scan in result.get('sweep',[]):
        window=data[(data['t']>=scan['finished']-.3)&(data['t']<=scan['finished'])]
        valid=bool(len(window)>1 and window['t'][-1]-window['t'][0]>=.3-window['dt'][0]*1.01 and all(
            np.max(np.abs(window[n+'_q']-target))<=math.radians(1) and np.max(np.abs(window[n+'_v']))<=.05
            for n,target in zip(('camera_pan_joint','camera_tilt_joint'),scan['target'])))
        confirmations.append(dict(passed=valid,phase=scan['phase']))
    observations=[r for r in rows if start<=r['sim']<=end]
    wall=max(r['wall'] for r in observations)-min(r['wall'] for r in observations)
    resources=[json.loads(line) for line in (folder/'resources.jsonl').open()]
    cpu=np.diff([r['cpu_usage_us'] for r in resources])/1e6/np.diff([r['wall'] for r in resources])
    safe=[r for r in observations if r['kind']=='safe_command']
    safe_gap=max((b['sim']-a['sim'] for a,b in zip(safe,safe[1:])),default=math.inf)
    complete=all(c['passed'] for c in coverage.values()) and imagery and not result.get('errors') and len(resources)>1 and safe_gap<=.100001
    scan_ok=all(s['passed'] and s['phase']=='succeeded' for s in confirmations)
    active=data[(data['t']>=start)&(data['t']<=end)]
    body_gate=True;body=None
    if kind in ('static','gimbal-sweep'):
        xyz=np.column_stack([active[n] for n in ('x','y')])
        drift=float(np.max(np.linalg.norm(xyz-xyz[0],axis=1)))
        w,x,y,z=[active[n] for n in ('qw','qx','qy','qz')]
        yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
        yaw_change=float(np.max(np.abs(np.arctan2(np.sin(yaw-yaw[0]),np.cos(yaw-yaw[0])))))
        roll=np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y));pitch=np.arcsin(np.clip(2*(w*y-z*x),-1,1))
        tilt=float(max(np.max(np.abs(roll)),np.max(np.abs(pitch))))
        body_gate=drift<=.05 and yaw_change<=.05 and tilt<=.1
        body=dict(passed=body_gate,drift_m=drift,yaw_change_rad=yaw_change,max_tilt_rad=tilt)
    return dict(passed=bool(complete and commands['passed'] and scan_ok and body_gate),data_complete=bool(complete),
                sensors=coverage,imagery_observable=bool(imagery),safe_gap_s=safe_gap,command_window=commands,
                success_confirmations=confirmations,per_step_body=body,
                performance=dict(realtime_factor=(end-start)/wall,cpu_core_mean=float(np.mean(cpu)),
                    cpu_core_max=float(np.max(cpu)),memory_peak_bytes=max(r['memory_bytes'] for r in resources)))

def compare_pair(a,b):
    wa,wb=a.get('formal',{}).get('command_window',{}),b.get('formal',{}).get('command_window',{})
    if not(a.get('passed') and b.get('passed') and wa.get('passed') and wb.get('passed')):
        return dict(passed=False,valid=False,reason='Round or received-command gate failed')
    timing=abs(wa['duration_s']-wb['duration_s'])
    paused=wa.get('mode')=='paused_steps' and wb.get('mode')=='paused_steps'
    if (wa.get('mode')=='paused_steps')!=(wb.get('mode')=='paused_steps'):
        return dict(passed=False,valid=False,reason='Different motion endpoint/scheduling conventions')
    # Timestamps are represented as binary floats; an exact 10 ms interval
    # can subtract to 0.0100000000000016. One nanosecond only covers arithmetic.
    tolerance=max(wa['step'],wb['step']) if paused else .01
    if timing>tolerance+1e-9:return dict(passed=False,valid=False,duration_difference_s=timing,reason='Command durations not matched within the scheduling tolerance')
    budgets=None
    if paused:
        ba,bb=wa.get('wheel_budget',{}),wb.get('wheel_budget',{})
        if not(ba.get('passed') and bb.get('passed')):return dict(passed=False,valid=False,reason='Invalid wheel target profile')
        budgets=dict(linear_difference_m=abs(ba['linear_integral_m']-bb['linear_integral_m']),
            angular_difference_rad=abs(ba['angular_integral_rad']-bb['angular_integral_rad']))
        if budgets['linear_difference_m']>.001+1e-9 or budgets['angular_difference_rad']>.002+1e-9:
            return dict(passed=False,valid=False,reason='Wheel command budgets differ',wheel_budget_difference=budgets)
    pa,pb=wa['endpoint'],wb['endpoint']
    distance=math.dist(pa['xyz'][:2],pb['xyz'][:2]);delta=pa['rpy'][2]-pb['rpy'][2]
    yaw=abs(math.atan2(math.sin(delta),math.cos(delta)))
    return dict(passed=distance<=.01 and yaw<=.01,valid=True,position_difference_m=distance,
                yaw_difference_rad=yaw,duration_difference_s=timing,wheel_budget_difference=budgets)

def choose_step(primary,reference=None):
    if len(primary)==6 and all(p['passed'] for p in primary):return dict(step=.001,passed=True,status='retain_1ms')
    if reference is None:return dict(step=None,passed=False,status='needs_025ms_reference')
    if len(reference)==6 and all(p['passed'] for p in reference):return dict(step=.0005,passed=True,status='recommend_05ms')
    return dict(step=.001,passed=False,status='turn_consistency_unresolved')
