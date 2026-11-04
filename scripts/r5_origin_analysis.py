"""Read-only origin/turn diagnostics. Truth never enters the command path."""
import json
import math
from pathlib import Path
import numpy as np
from r5_csv import read_numeric_csv

WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')

def load_lines(path):
    with Path(path).open(encoding='utf-8') as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)

def rotation(q):
    """Quaternion order w,x,y,z; accepts vectorized input."""
    q = np.asarray(q, float)
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    if np.any(~np.isfinite(q)) or np.any(norm < 1e-12):
        raise ValueError('Invalid quaternion')
    w, x, y, z = np.moveaxis(q / norm, -1, 0)
    return np.stack((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
                     2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
                     2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)), axis=-1).reshape(q.shape[:-1]+(3,3))

def unwrap_yaw(data):
    r = rotation(np.column_stack([data[k] for k in ('qw','qx','qy','qz')]))
    return np.unwrap(np.arctan2(r[:,1,0], r[:,0,0]))

def trajectory(t, xyz, yaw):
    if len(t) < 3 or np.any(np.diff(t) <= 0):
        raise ValueError('Missing or nonmonotonic trajectory')
    xy = np.asarray(xyz)[:,:2]
    relative = xy-xy[0]
    velocity = np.gradient(xy, t, axis=0)
    omega = np.gradient(yaw, t)
    valid = np.abs(omega) > .02
    center = np.full_like(xy, np.nan)
    center[valid] = xy[valid]+np.column_stack((-velocity[valid,1],velocity[valid,0]))/omega[valid,None]
    # A circle fit describes a trajectory; it does not prove a fixed physical ICR.
    a = np.column_stack((2*relative[:,0],2*relative[:,1],np.ones(len(xy))))
    fit = None
    if np.linalg.matrix_rank(a) == 3 and np.ptp(yaw) > .2:
        cx,cy,k = np.linalg.lstsq(a,np.sum(relative*relative,axis=1),rcond=None)[0]
        radius = math.sqrt(max(0,k+cx*cx+cy*cy))
        residual = np.linalg.norm(relative-[cx,cy],axis=1)-radius
        fit = dict(center_world_m=(xy[0]+[cx,cy]).tolist(),radius_m=radius,
                   radial_residual_rms_m=float(np.sqrt(np.mean(residual**2))))
    return dict(endpoint_displacement_m=float(np.linalg.norm(relative[-1])),
                maximum_displacement_m=float(np.max(np.linalg.norm(relative,axis=1))),
                path_length_m=float(np.sum(np.linalg.norm(np.diff(xy,axis=0),axis=1))),
                unwrapped_turn_rad=float(yaw[-1]-yaw[0]),circle_fit=fit,
                instantaneous_center_samples=int(valid.sum()),
                instantaneous_center_median_world_m=np.median(center[valid],axis=0).tolist() if valid.any() else None), velocity, omega, center

def contact_summary(folder,start,end):
    out={}
    for wheel in WHEELS:
        stamps=[]; forces=[]; depths=[]; normal_errors=[]; imbalance=[]
        for row in load_lines(Path(folder)/(wheel+'-contact.jsonl')):
            stamp=row.get('header',{}).get('stamp',{})
            t=int(stamp.get('sec',0))+int(stamp.get('nsec',0))*1e-9
            if not start <= t <= end: continue
            stamps.append(t); force=np.zeros(3); available=True
            for contact in row.get('contact',[]):
                for n in contact.get('normal',[]):
                    v=np.array([float(n.get(k,0)) for k in ('x','y','z')]);normal_errors.append(float(np.linalg.norm(v-[0,0,1])))
                depths.extend(float(d) for d in contact.get('depth',[]))
                if not contact.get('wrench'): available=False
                for wrench in contact.get('wrench',[]):
                    one=wrench.get('body1Wrench',{}).get('force');two=wrench.get('body2Wrench',{}).get('force')
                    if one is None or two is None: available=False;continue
                    f1=np.array([float(one.get(k,0)) for k in ('x','y','z')]);f2=np.array([float(two.get(k,0)) for k in ('x','y','z')])
                    force+=f1;imbalance.append(float(np.linalg.norm(f1+f2)))
            if available: forces.append(force)
        gaps=np.diff(stamps)
        complete=bool(len(stamps)>1 and stamps[0]<=start+.1 and stamps[-1]>=end-.1 and np.min(gaps)>=-1e-9 and np.max(gaps)<=.100001)
        out[wheel]=dict(complete=complete,samples=len(stamps),force_samples=len(forces),
            max_gap_s=float(gaps.max()) if len(gaps) else None,
            force_mean_raw=np.mean(forces,axis=0).tolist() if forces else None,
            maximum_depth_m=max(depths,default=None),normal_world_z_error_max=max(normal_errors,default=None),
            action_reaction_residual_max=max(imbalance,default=None),
            frame='Raw ContactSensor force; must independently verify backend frame and body ordering before interpretation')
    return out

def qualify_round(folder,result):
    """No performance drift gate; qualify actual inputs and diagnostic evidence."""
    folder=Path(folder);data=read_numeric_csv(folder/'physics-steps.csv',delimiter=',',names=True)
    start,end=result['sim_started'],result['sim_ended']
    active=data[(data['t']>=start-1e-9)&(data['t']<=end+1e-9)]
    fields=[n for n in data.dtype.names if '_world_' in n or '_collision_' in n or '_axis_' in n or '_fdir1_' in n or n.startswith('com_') or n=='total_mass']
    # Four wheels each: link pose 7, collision pose 7, axis 3, fdir1 3 = 80;
    # center of mass 3 + total mass 1 -> 84 fields.
    world=bool(len(fields)==84 and len(active)>1 and all(np.isfinite(active[n]).all() for n in fields))
    cadence={}
    rows=list(load_lines(folder/'observations.jsonl'))
    for kind in ('ros_wheel_odom','gz_native_odom'):
        stamps=[r['stamp'] for r in rows if r['kind']==kind and start-.1<=r['stamp']<=end+.1]
        gaps=np.diff(stamps)
        cadence[kind]=dict(passed=bool(len(stamps)>1 and stamps[0]<=start+.1 and stamps[-1]>=end-.1 and
            np.min(gaps,initial=0)>=-1e-9 and np.max(gaps,initial=0)<=.100001),
            samples=len(stamps),max_gap_s=float(np.max(gaps,initial=0)))
    contacts=contact_summary(folder,start,end)
    motion=result['schedule']['motion'];duration=motion['after']['sim']-motion['before']['sim']
    iterations=int(motion['after']['iteration']-motion['before']['iteration'])
    grid=bool(len(active)>1 and (np.diff(active['iteration'])==1).all() and
        int(active[-1]['iteration'])==result['schedule']['stop']['after']['iteration'])
    ros_frames={tuple((r['frame_id'],r['child_frame_id'])) for r in rows if r['kind']=='ros_wheel_odom'}
    native_frames={tuple((d.get('key'),tuple(d.get('value',[]))) for d in r['raw'].get('header',{}).get('data',[]))
                   for r in rows if r['kind']=='gz_native_odom'}
    frames=bool(ros_frames and all(a and b for a,b in ros_frames) and native_frames and all(v for v in native_frames))
    return dict(passed=world and grid and frames and all(v['passed'] for v in cadence.values()) and
                all(v['complete'] for v in contacts.values()) and abs(duration-20)<=.001000001 and iterations==20000,
                world_pose_complete=world,per_step_grid=grid,odometry=cadence,contacts=contacts,
                frames_valid=frames,ros_frames=[list(v) for v in sorted(ros_frames)],
                native_frames=[[list(v) for v in f] for f in native_frames],motion_duration_s=duration,motion_steps=iterations)

def odometry_metrics(folder,data,start,end):
    out={};truth_yaw=unwrap_yaw(data)
    for kind in ('ros_wheel_odom','gz_native_odom'):
        rows=[r for r in load_lines(Path(folder)/'observations.jsonl') if r['kind']==kind]
        if not rows:out[kind]=dict(available=False);continue
        t=np.array([r['stamp'] for r in rows]);xyz=[];quats=[]
        for row in rows:
            if kind=='ros_wheel_odom':xyz.append(row['xyz']);quats.append(row['quaternion_wxyz'])
            else:
                p=row['raw'].get('pose',{}).get('position',{});q=row['raw'].get('pose',{}).get('orientation',{})
                xyz.append([float(p.get(k,0)) for k in ('x','y','z')]);quats.append([float(q.get(k,1 if k=='w' else 0)) for k in ('w','x','y','z')])
        xyz=np.array(xyz);r=rotation(quats);yaw=np.unwrap(np.arctan2(r[:,1,0],r[:,0,0]))
        first=int(np.argmin(abs(t-start)));last=int(np.argmin(abs(t-end)))
        gt0=np.array([np.interp(t[first],data['t'],data[k]) for k in ('x','y')]);gtyaw0=np.interp(t[first],data['t'],truth_yaw)
        delta=gtyaw0-yaw[first];rot=np.array([[math.cos(delta),-math.sin(delta)],[math.sin(delta),math.cos(delta)]])
        aligned=(xyz[:,:2]-xyz[first,:2])@rot.T+gt0
        gt=np.column_stack([np.interp(t,data['t'],data[k]) for k in ('x','y')]);gyaw=np.interp(t,data['t'],truth_yaw)
        difference=(yaw-yaw[first])-(gyaw-gtyaw0);difference=np.arctan2(np.sin(difference),np.cos(difference))
        window=(t>=start)&(t<=end)
        out[kind]=dict(available=True,motion_start_sample_s=float(t[first]),motion_end_sample_s=float(t[last]),
            endpoint_position_error_m=float(np.linalg.norm(aligned[last]-gt[last])),maximum_position_error_m=float(np.max(np.linalg.norm(aligned[window]-gt[window],axis=1))),
            endpoint_yaw_error_rad=float(difference[last]),odometry_turn_rad=float(yaw[last]-yaw[first]),
            odometry_displacement_m=float(np.linalg.norm(xyz[last,:2]-xyz[first,:2])),
            alignment='Rigid SE(2) alignment at nearest actual pre-motion sample; no truth fed back',
            sample_offset_from_motion_endpoint_s=float(t[last]-end))
    return out

def analyze(folder, output=None):
    folder=Path(folder); result=json.loads((folder/'result.json').read_text())
    data=read_numeric_csv(folder/'physics-steps.csv',delimiter=',',names=True)
    schedule=result['schedule'];start=schedule['parked']['after']['sim'];end=schedule['motion']['after']['sim']
    moving=data[(data['t']>=start-1e-9)&(data['t']<=end+1e-9)]
    t=moving['t'];xyz=np.column_stack([moving[k] for k in ('x','y','z')]);yaw=unwrap_yaw(moving)
    metrics,vel,omega,centers=trajectory(t,xyz,yaw)
    r=rotation(np.column_stack([moving[k] for k in ('qw','qx','qy','qz')]))
    body_vel=np.einsum('nji,nj->ni',r[:,:2,:2],vel)
    steady=(t>=start+3)&(t<=end-1)
    metrics['steady_body_velocity_mean_m_s']=np.mean(body_vel[steady],axis=0).tolist()
    metrics['steady_yaw_rate_mean_rad_s']=float(np.mean(omega[steady]))
    metrics['stop_extra_displacement_m']=math.dist([float(data[-1][k]) for k in ('x','y')],xyz[-1,:2])
    metrics['wheels']={wheel:dict(velocity_tracking_max_rad_s=float(np.max(np.abs(moving[wheel+'_wheel_joint_v'][steady]-moving[wheel+'_wheel_joint_velocity_cmd'][steady]))),
        actual_velocity_mean_rad_s=float(np.mean(moving[wheel+'_wheel_joint_v'][steady])),
        target_mean_rad_s=float(np.mean(moving[wheel+'_wheel_joint_velocity_cmd'][steady]))) for wheel in WHEELS}
    metrics['contacts']=contact_summary(folder,start+3,end-1)
    metrics['gimbal_velocity_rms_rad_s']={axis:float(np.sqrt(np.mean(moving[axis+'_joint_v'][steady]**2))) for axis in ('camera_pan','camera_tilt')}
    metrics['world_pose_fields_present']=all(w+'_world_x' in data.dtype.names for w in WHEELS)
    if metrics['world_pose_fields_present']:
        geometry={};slip={}
        for wheel in WHEELS:
            p=np.column_stack([moving[wheel+'_world_'+k] for k in ('x','y','z')])
            local=np.einsum('nji,nj->ni',r,p-xyz)
            geometry[wheel]=dict(center_body_mean_m=np.mean(local[steady],axis=0).tolist(),
                                center_body_range_m=np.ptp(local[steady],axis=0).tolist())
            # Bottom-of-wheel velocity on a horizontal flat. This is a kinematic
            # slip estimate, not a replacement for actual contact-point telemetry.
            pv=np.gradient(p,t,axis=0)
            axis=np.column_stack([moving[wheel+'_axis_'+k] for k in ('x','y','z')])
            angular=np.column_stack((np.zeros(len(t)),np.zeros(len(t)),omega))+axis*moving[wheel+'_wheel_joint_v'][:,None]
            bottom=np.tile([0.,0.,-.076],(len(t),1))
            sv=pv+np.cross(angular,bottom)
            sb=np.einsum('nji,nj->ni',r,sv)
            slip[wheel]=dict(bottom_estimate_body_mean_m_s=np.mean(sb[steady],axis=0).tolist(),
                             bottom_estimate_body_rms_m_s=np.sqrt(np.mean(sb[steady]**2,axis=0)).tolist(),
                             assumption='Horizontal plane, bottom offset -radius; use raw contacts for pointwise verification')
        metrics['geometry']=geometry;metrics['kinematic_slip']=slip
        metrics['odometry']=odometry_metrics(folder,data,start,end)
        com=np.column_stack([moving['com_world_'+k] for k in ('x','y','z')])
        local_com=np.einsum('nji,nj->ni',r,com-xyz)
        metrics['center_of_mass_body_mean_m']=np.mean(local_com[steady],axis=0).tolist()
        metrics['total_mass_kg']=float(np.mean(moving['total_mass'][steady]))
        metrics['contact_body_forces']={}
        for wheel in WHEELS:
            fbody=[];loads=[];position_errors=[]
            for row in load_lines(folder/(wheel+'-contact.jsonl')):
                stamp=row.get('header',{}).get('stamp',{});ct=int(stamp.get('sec',0))+int(stamp.get('nsec',0))*1e-9
                if not start+3<=ct<=end-1:continue
                index=int(np.clip(np.searchsorted(t,ct),0,len(t)-1));f=np.zeros(3)
                for c in row.get('contact',[]):
                    for wr in c.get('wrench',[]):
                        raw=wr.get('body1Wrench',{}).get('force',{});f+=np.array([float(raw.get(k,0)) for k in ('x','y','z')])
                    for pt in c.get('position',[]):
                        p=np.array([float(pt.get(k,0)) for k in ('x','y','z')])
                        wheelp=np.array([moving[wheel+'_world_'+k][index] for k in ('x','y','z')])
                        position_errors.append(float(np.linalg.norm(p-wheelp)))
                fbody.append(r[index].T@f);loads.append(f[2])
            metrics['contact_body_forces'][wheel]=dict(mean_body_N=np.mean(fbody,axis=0).tolist() if fbody else None,
                normal_mean_N=float(np.mean(loads)) if loads else None,
                contact_distance_to_wheel_center_range_m=[min(position_errors),max(position_errors)] if position_errors else None,
                frame_status='World-force interpretation conditional on archived gz-physics/Gazebo source audit')
    if output:
        output=Path(output);output.mkdir(parents=True,exist_ok=True)
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(1,3,figsize=(13,4))
        axes[0].plot(xyz[:,0]-xyz[0,0],xyz[:,1]-xyz[0,1]);axes[0].scatter([0],[0],label='start');axes[0].axis('equal')
        axes[0].set(xlabel='world X displacement (m)',ylabel='world Y displacement (m)',title='Body origin trajectory')
        axes[1].plot(t-start,yaw-yaw[0]);axes[1].set(xlabel='motion time (s)',ylabel='unwrapped yaw (rad)')
        for i,label in enumerate(('longitudinal','lateral')):axes[2].plot(t-start,body_vel[:,i],label=label)
        axes[2].set(xlabel='motion time (s)',ylabel='body velocity (m/s)');axes[2].legend()
        fig.tight_layout();fig.savefig(output/'trajectory.png',dpi=160);plt.close(fig)
        np.savetxt(output/'trajectory.csv',np.column_stack((t,xyz,yaw,vel,omega,centers)),delimiter=',',
            header='t,x,y,z,yaw_unwrapped,vx,vy,omega,icr_x,icr_y',comments='')
        (output/'analysis.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')
    return metrics

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--round',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();print(json.dumps(analyze(a.round,a.output),indent=2,allow_nan=False))
