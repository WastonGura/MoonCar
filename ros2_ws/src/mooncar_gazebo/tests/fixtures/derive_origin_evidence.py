"""Offline-only point-contact/coordinate audit; never imports ROS or controls Gazebo."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

def stamp(row):
    s=row.get('header',{}).get('stamp',{})
    return int(s.get('sec',0))+int(s.get('nsec',0))*1e-9

def stamp_ns(row):
    s=row.get('header',{}).get('stamp',{})
    return int(s.get('sec',0))*1_000_000_000+int(s.get('nsec',0))

def vec(v):return np.array([float(v.get(k,0)) for k in ('x','y','z')])

def plot_paths(root,paths):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(8,5))
    for name in ('left-1','right-1','heading90-left','heading90-right'):
        xy=paths[name];ax.plot(xy[:,0],xy[:,1],label=name)
    ax.scatter([0,-.157],[0,0],marker='x',color='black')
    ax.annotate('initial rear axle',(-.157,0),xytext=(-.225,.018))
    ax.annotate('body origin',(0,0),xytext=(.001,.01))
    ax.axis('equal');ax.set(xlabel='initial body X (m)',ylabel='initial body Y (m)')
    ax.set_title('Turning trajectories aligned to initial body frame',pad=38)
    ax.legend(loc='lower left',bbox_to_anchor=(0,1.01),ncol=4,fontsize=9,frameon=False)
    fig.tight_layout();fig.savefig(Path(root)/'turn-comparison.png',dpi=180);plt.close(fig)

def plot_existing(root):
    names={'left-1':'03-left-1','right-1':'04-right-1','heading90-left':'07-heading90-left','heading90-right':'08-heading90-right'}
    paths={}
    for name,folder in names.items():
        data=np.genfromtxt(Path(root)/'campaign'/folder/'origin-analysis/trajectory.csv',names=True,delimiter=',')
        theta=float(data['yaw_unwrapped'][0]);rot=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
        xy=np.column_stack((data['x'],data['y']));paths[name]=(xy-xy[0])@rot
    plot_paths(root,paths)

def run(root):
    root=Path(root);sys.path.insert(0,str(root/'campaign/source/scripts'))
    from r5_origin_analysis import WHEELS,rotation,load_lines,unwrap_yaw,trajectory
    ledger=json.loads((root/'campaign/ledger.json').read_text())
    if len(ledger)!=8 or not all(r.get('passed') for r in ledger):raise RuntimeError('Eight qualified baselines required')
    findings={};paths={};items=[]
    for entry in ledger:
        folder=root/'campaign'/entry['name'];result=json.loads((folder/'result.json').read_text())
        metrics=json.loads((folder/'origin-analysis/analysis.json').read_text())
        data=np.genfromtxt(folder/'physics-steps.csv',names=True,delimiter=',');t=data['t'];tns=np.rint(t*1e9).astype(np.int64)
        xyz=np.column_stack([data[k] for k in ('x','y','z')]);r=rotation(np.column_stack([data[k] for k in ('qw','qx','qy','qz')]))
        yaw=unwrap_yaw(data);start=result['schedule']['parked']['after']['sim'];end=result['schedule']['motion']['after']['sim']
        steady=(t>=start+3)&(t<=end-1);moving=(t>=start-1e-9)&(t<=end+1e-9)
        _,v,omega,icr=trajectory(t,xyz,yaw)
        usable=steady&(abs(omega)>.02)
        body_icr=np.einsum('nji,nj->ni',r[usable,:2,:2],icr[usable]-xyz[usable,:2]) if usable.any() else np.empty((0,2))
        centroid=np.mean([np.column_stack([data[w+'_world_'+k] for k in ('x','y','z')]) for w in WHEELS],axis=0)
        centroid_body=np.einsum('nji,nj->ni',r,centroid-xyz)
        audit=dict(icr_body_median_m=np.median(body_icr,axis=0).tolist() if usable.any() else None,
                   wheel_geometric_center_body_mean_m=np.mean(centroid_body[steady],axis=0).tolist(),wheels={})
        # Direct world-pose observer versus transforming PosePublisher local data.
        frame_errors=[]
        for row in load_lines(folder/'link-poses.jsonl'):
            poses=row.get('pose',[]);model=next((p for p in poses if p.get('name')=='mooncar'),None)
            if model is None:continue
            pt=stamp(model)
            if not start<=pt<=end:continue
            index=int(np.clip(np.searchsorted(tns,stamp_ns(model)),0,len(t)-1))
            if int(tns[index])!=stamp_ns(model):continue
            for w in WHEELS:
                local=next((p for p in poses if p.get('name')=='mooncar::'+w+'_wheel_link'),None)
                if local is None:continue
                frames={d['key']:d.get('value',[]) for d in local.get('header',{}).get('data',[])}
                if frames.get('frame_id')!=['mooncar']:raise RuntimeError('Link pose parent is not the model frame')
                reconstructed=xyz[index]+r[index]@vec(local.get('position',{}))
                direct=np.array([data[w+'_world_'+k][index] for k in ('x','y','z')])
                frame_errors.append(float(np.linalg.norm(reconstructed-direct)))
        audit['local_pose_to_world_audit']=dict(samples=len(frame_errors),max_position_error_m=max(frame_errors,default=None))
        if len(frame_errors)<4 or max(frame_errors)>1e-7:raise RuntimeError('Local/world transform not confirmed: '+entry['name'])
        with (folder/'origin-analysis/contact-point-slip.csv').open('x',newline='') as out:
            writer=csv.writer(out);writer.writerow(('t','wheel','point','slip_body_x','slip_body_y','slip_body_z','force_body_x','force_body_y','force_body_z','normal_world_z'))
            for w in WHEELS:
                p=np.column_stack([data[w+'_world_'+k] for k in ('x','y','z')]);pv=np.gradient(p,t,axis=0)
                wr=rotation(np.column_stack([data[w+'_world_'+k] for k in ('qw','qx','qy','qz')]))
                dr=np.gradient(wr,t,axis=0);skew=np.einsum('nij,nkj->nik',dr,wr)
                av=np.column_stack(((skew[:,2,1]-skew[:,1,2])/2,(skew[:,0,2]-skew[:,2,0])/2,(skew[:,1,0]-skew[:,0,1])/2))
                slips=[];forces=[];ratios=[];fdir_body=[];normals=[];depths=[]
                for row in load_lines(folder/(w+'-contact.jsonl')):
                    ct=stamp(row)
                    if not start+3<=ct<=end-1:continue
                    index=int(np.clip(np.searchsorted(tns,stamp_ns(row)),0,len(t)-1))
                    if int(tns[index])!=stamp_ns(row):raise RuntimeError('Contact/physical step cannot be associated')
                    fdir=np.array([data[w+'_fdir1_'+k][index] for k in ('x','y','z')]);fdir_body.append(r[index].T@fdir)
                    for c in row.get('contact',[]):
                        if w+'_wheel_link' in c.get('collision1',{}).get('name',''):body='body1Wrench'
                        elif w+'_wheel_link' in c.get('collision2',{}).get('name',''):body='body2Wrench'
                        else:raise RuntimeError('Contact ownership unknown')
                        points=c.get('position',[]);wrenches=c.get('wrench',[]);ns=c.get('normal',[])
                        if not points or len(points)!=len(wrenches) or len(points)!=len(ns):raise RuntimeError('Point/wrench/normal association missing')
                        depths.extend(c.get('depth',[]))
                        for j,(point,wrench,n) in enumerate(zip(points,wrenches,ns)):
                            force=wrench.get(body,{}).get('force')
                            if force is None:raise RuntimeError('Missing force is not zero')
                            arm=vec(point)-p[index]
                            slip=r[index].T@(pv[index]+np.cross(av[index],arm))
                            fw=vec(force);fb=r[index].T@fw;normal=vec(n)
                            slips.append(slip);forces.append(fb);normals.append(normal)
                            if abs(fw[2])>1e-12:ratios.append(abs(float(np.dot(fw,fdir)))/abs(fw[2]))
                            writer.writerow((ct,w,j,*slip,*fb,normal[2]))
                audit['wheels'][w]=dict(point_samples=len(slips),contact_slip_body_mean_m_s=np.mean(slips,axis=0).tolist() if slips else None,
                    contact_slip_body_rms_m_s=np.sqrt(np.mean(np.array(slips)**2,axis=0)).tolist() if slips else None,
                    first_friction_direction_body_mean=np.mean(fdir_body,axis=0).tolist() if fdir_body else None,
                    force_along_first_direction_over_normal_mean=float(np.mean(ratios)) if ratios else None,
                    maximum_depth_m=max(depths,default=None),normal_mean_world=np.mean(normals,axis=0).tolist() if normals else None)
        (folder/'origin-analysis/point-contact-audit.json').write_text(json.dumps(audit,indent=2,allow_nan=False)+'\n')
        nload=metrics['contact_body_forces'];front=sum(nload[w]['normal_mean_N'] for w in WHEELS[:2]);rear=sum(nload[w]['normal_mean_N'] for w in WHEELS[2:])
        audit['load_balance']=dict(front_normal_N=front,rear_normal_N=rear,sum_normal_N=front+rear,
            weight_N=metrics['total_mass_kg']*1.62,normal_minus_weight_N=front+rear-metrics['total_mass_kg']*1.62)
        findings[entry['name']]=dict(metrics=metrics,audit=audit)
        # Rotate all trajectories to their initial body orientation for comparison.
        paths[entry['case']]=(xyz[moving,:2]-xyz[moving,:2][0])@r[moving][0,:2,:2]
        images=result['formal']['sensors'];perf=result['formal']['performance']
        items.append(dict(name=entry['name'],qualified=True,endpoint_m=metrics['endpoint_displacement_m'],maximum_m=metrics['maximum_displacement_m'],
            turn_rad=metrics['unwrapped_turn_rad'],icr_body_m=audit['icr_body_median_m'],stop_extra_m=metrics['stop_extra_displacement_m'],
            last_second_stop_m=result['stop_drift_last_second_m'],image_count=len(result['image_content']['content']),
            max_rgb_gap_s=images['rgb']['max_gap_s'],max_depth_gap_s=images['depth']['max_gap_s'],performance=perf,
            wall_seconds=entry['wall_seconds'],evidence_bytes=sum(p.stat().st_size for p in folder.rglob('*') if p.is_file())))
    comparisons={}
    for one,two in (('left-1','left-2'),('right-1','right-2'),('left-1','heading90-left'),('right-1','heading90-right')):
        if paths[one].shape!=paths[two].shape:raise RuntimeError('Motion grids differ')
        delta=np.linalg.norm(paths[one]-paths[two],axis=1)
        comparisons[one+' / '+two]=dict(maximum_path_difference_m=float(delta.max()),endpoint_difference_m=float(delta[-1]))
    summary=dict(attempts=8,qualified=8,remaining_budget=14,rounds=items,frame_and_point_contact_audits=findings,trajectory_comparisons=comparisons,
        mechanism_observed='Origin rotates around the rear axle; front wheel contacts move laterally while rear lateral motion is near zero',
        cause_confirmed=False,repair_verified=False,turn_performance_accepted=False,production_changed=False)
    (root/'findings.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    decision=dict(status='stopped_no_evidence_qualified_software_candidate',baseline_attempts=8,candidate_attempts=0,restore_attempts=0,dart_patch_attempts=0,
        specific_error=None,single_change=None,prediction=None,cause_confirmed=False,repair_verified=False,
        observed='Rear-axle pivot, front lateral slip, rear/front normal load difference, wheel-odometry position/yaw discrepancy',
        hypothesis='Rearward center of mass and unequal normal loads together with the current friction/contact model may select the rear axle as the lateral sticking line',
        limitation='No isolated A/B/A cause test: vehicle/friction/inertia adjustments are outside this round. Measurements do not prove a DART or model-definition software error.',
        stopping_rule='After eight baselines only contact/slip evidence is established; do not invent a patch or scan parameters',
        remaining_problem='Approximately 0.3121 m origin displacement persists; odometry misses lateral translation',production_changed=False)
    (root/'decision.json').write_text(json.dumps(decision,indent=2)+'\n')
    plot_paths(root,paths)
    print(json.dumps(dict(rounds=items,comparisons=comparisons,decision=decision),indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--evidence',default='/evidence');p.add_argument('--plot-existing',action='store_true');a=p.parse_args()
    if a.plot_existing:plot_existing(a.evidence)
    else:run(a.evidence)
