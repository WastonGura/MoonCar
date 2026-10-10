"""Read-only post-run force/trajectory analysis, never changes round verdicts."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys
sys.path.insert(0,'/source/scripts')
from r5_origin_analysis import rotation
p=Path('/evidence');ledger=json.loads((p/'campaign/ledger.json').read_text());metrics=[]
fig,axes=plt.subplots(2,2,figsize=(12,8))
for e in ledger:
    folder=p/'campaign'/e['name'];r=json.loads((folder/'result.json').read_text())
    if not e.get('qualified'):continue
    data=np.genfromtxt(folder/'physics-steps.csv',delimiter=',',names=True)
    raw=np.genfromtxt(folder/(e['name']+'-contact-steps.csv'),delimiter=',',names=True)
    if np.any(raw['state']<0):raise ValueError('Unknown source contact state; no force imputation allowed')
    count=len(data);positive=raw['state']==1;points=raw[positive]
    index=points['iteration'].astype(int)-1;wheel=points['wheel'].astype(int)
    if not np.array_equal(data['iteration'],np.arange(1,count+1)):raise ValueError('Unexpected birth grid')
    # Only explicitly empty source snapshots contribute zero contact force;
    # positive samples contribute their actual world force, never a conditional mean.
    forces=np.zeros((count,4,3));point_counts=np.zeros((count,4),int)
    values=np.column_stack([points[k] for k in ('f1x','f1y','f1z')])
    for k in range(3):np.add.at(forces[:,:,k],(index,wheel),values[:,k])
    np.add.at(point_counts,(index,wheel),1)
    start=r['schedule']['motion']['before']['sim'];end=r['schedule']['motion']['after']['sim']
    motion=(data['t']>=start-1e-9)&(data['t']<=end+1e-9)
    static=(data['t']>=start-3)&(data['t']<start)
    xyz=np.column_stack([data[k] for k in ('x','y','z')]);body_r=rotation(np.column_stack([data[k] for k in ('qw','qx','qy','qz')]))
    xy=xyz[motion,:2]-xyz[motion,:2][0];relative_t=data['t'][motion]-start
    mass=float(np.mean(data['total_mass'][static]));weight=mass*1.62
    loads=np.mean(forces[static,:,2],axis=0)
    record=dict(name=e['name'],profile=e['profile'],qualified=True,endpoint_xy_m=xy[-1].tolist(),
        static_vertical_load_per_wheel_N=loads.tolist(),static_sum_vertical_load_N=float(loads.sum()),
        model_weight_N=weight,static_vertical_load_relative_error=float(abs(loads.sum()-weight)/weight),
        source=r['contact_source_probe']['source'],performance=r['formal']['performance'],wall_seconds=e['wall_seconds'],
        stop_drift_last_second_m=r['stop_drift_last_second_m'],parked=r['parked'],max_tilt_rad=r['max_tilt_rad'],
        allocated_targets_unchanged=r['contact_source_probe']['allocation_target_difference_rad_s']==0,
        local_evidence_bytes=sum(f.stat().st_size for f in folder.rglob('*') if f.is_file()))
    if e['linear']:
        axes[0,0].plot(xy[:,0],xy[:,1]*1000,label=e['name'])
        axes[0,1].plot(relative_t,xy[:,1]*1000,label=e['name'])
        axes[1,0].plot(relative_t,np.sum(forces[motion,:,2],axis=1),label=e['name'])
        axes[1,1].plot(relative_t,np.count_nonzero(point_counts[motion],axis=1),label=e['name'])
        steady=(data['t']>=start+3)&(data['t']<=end-1)
        contact_kinematics={}
        for w,name in enumerate(('front_left','front_right','rear_left','rear_right')):
            selection=points['wheel']==w;records=points[selection];ii=records['iteration'].astype(int)-1
            center=np.column_stack([data[name+'_world_'+k] for k in ('x','y','z')])
            q=rotation(np.column_stack([data[name+'_world_'+k] for k in ('qw','qx','qy','qz')]))
            rd=np.gradient(q,data['t'],axis=0);cross=np.einsum('nij,nkj->nik',rd,q)
            omega=np.column_stack((cross[:,2,1]-cross[:,1,2],cross[:,0,2]-cross[:,2,0],cross[:,1,0]-cross[:,0,1]))*.5
            position=np.column_stack([records[k] for k in ('x','y','z')]);normal=np.column_stack([records[k] for k in ('nx','ny','nz')])
            velocity=np.gradient(center,data['t'],axis=0)[ii]+np.cross(omega[ii],position-center[ii])
            fdir=np.column_stack([data[name+'_fdir1_'+k][ii] for k in ('x','y','z')])
            tangent=fdir-normal*np.sum(fdir*normal,axis=1)[:,None];norm=np.linalg.norm(tangent,axis=1)
            valid=steady[ii]&(norm>1e-10);tangent=tangent[valid]/norm[valid,None]
            slip=np.sum(velocity[valid]*tangent,axis=1)
            force=np.sum(np.column_stack([records[k][valid] for k in ('f1x','f1y','f1z')])*tangent,axis=1)
            item=dict(contact_points=int(valid.sum()),point_velocity_lateral_rms_m_s=float(np.sqrt(np.mean(slip**2))),
                force_lateral_rms_N=float(np.sqrt(np.mean(force**2))),
                velocity_method='World wheel pose derivative plus angular cross actual contact offset; projected onto current contact-plane fdir1')
            if e['profile']=='slip':
                compliance=data[name+'_slip1_cmd'][ii][valid]
                item['compliance_force_velocity_residual_rms_m_s']=float(np.sqrt(np.mean((slip+compliance*force)**2)))
                item['relation']='Engineering diagnostic using finite pose derivatives; not a hardware calibration or acceptance gate'
            contact_kinematics[name]=item
        record['contact_kinematics']=contact_kinematics
    metrics.append(record)
axes[0,0].set(xlabel='World X displacement (m)',ylabel='World Y displacement (mm)',title='Ramp straight trajectory')
axes[0,1].set(xlabel='Motion time (s)',ylabel='World Y displacement (mm)',title='Lateral displacement')
axes[1,0].set(xlabel='Motion time (s)',ylabel='Sum actual vertical contact force (N)',title='Every-step force, including confirmed empty snapshots')
axes[1,1].set(xlabel='Motion time (s)',ylabel='Wheels with contact',title='Read-only ECS contact states')
for ax in axes.flat:
    ax.grid(True)
    if ax.lines:ax.legend(fontsize=6)
fig.tight_layout();fig.savefig(p/'ramp-contact-comparison.png',dpi=160);plt.close(fig)
(p/'round-metrics.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')
print(json.dumps([dict(name=r['name'],endpoint=r['endpoint_xy_m'],static_load_error=r['static_vertical_load_relative_error']) for r in metrics],indent=2))
