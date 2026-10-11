"""Read-only exact-step contact/trajectory comparison, no control feedback."""
import hashlib,json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(__file__).resolve().parent;root=Path('/source') if Path('/source/scripts').is_dir() else p.parents[3]
sys.path.insert(0,str(root/'scripts'))
from audit_r5_ramp_contact import read_obj,classified_edges,edge_distance,normal_angle
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def columns(path,names):
    header=path.open().readline().strip().split(',')
    return np.genfromtxt(path,delimiter=',',names=True,usecols=[header.index(n) for n in names])
summary=json.loads((p/'campaign/summary.json').read_text());ledger=json.loads((p/'campaign/ledger.json').read_text())
out=p/'analysis';out.mkdir(exist_ok=False);reports=[];traces=[];hashes={}
for e in ledger:
    folder=p/'campaign'/e['name'];r=json.loads((folder/'result.json').read_text())
    d=json.loads((folder/'diagnostic-result.json').read_text())
    if not e.get('qualified'):
        reports.append(dict(name=e['name'],profile=e['profile'],case=e['case'],qualified=False,maximum_displacement_m=None,
            max_normal_angle_deg=None,tilted_step_count=None,error=r.get('error'),motion_not_completed=True))
        continue
    m=r['schedule']['motion'];first,last=m['before']['iteration'],m['after']['iteration']
    physics=columns(folder/'physics-steps.csv',['iteration','t','x','y'])
    active=physics[(physics['iteration']>first)&(physics['iteration']<=last)]
    assert np.array_equal(active['iteration'],np.arange(first+1,last+1))
    parked=physics[physics['iteration']==first][0];xy=np.column_stack([active['x']-parked['x'],active['y']-parked['y']])
    c=columns(folder/(e['name']+'-contact-steps.csv'),['iteration','t_ns','wheel','state','extra_complete','x','y','z','nx','ny','nz','other_collision','depth'])
    c=c[(c['iteration']>first)&(c['iteration']<=last)];positive=c[c['state']==1]
    assert np.all(positive['extra_complete']==1)
    assert np.array_equal(np.unique(c['iteration']*4+c['wheel']),(np.arange(first+1,last+1)[:,None]*4+np.arange(4)).ravel())
    points=np.column_stack([positive[n] for n in ('x','y','z')]);normals=np.column_stack([positive[n] for n in ('nx','ny','nz')])
    angles=normal_angle(normals,np.tile([0,0,1],(len(normals),1)));tilted=angles>1e-6
    indices=(positive['iteration']-first-1).astype(int)
    assert np.array_equal(positive['t_ns'],np.rint(active['t'][indices]*1e9))
    stepangle=np.full(len(active),-np.inf);np.maximum.at(stepangle,indices,angles);stepangle[~np.isfinite(stepangle)]=np.nan
    counts=np.bincount(indices,minlength=len(active))
    report=dict(name=e['name'],profile=e['profile'],case=e['case'],qualified=e.get('qualified',False),
        geometric_screen_passed=e['original_vehicle_screen_passed'],motion_steps=len(active),trajectory=r['turn_slip_probe']['trajectory'],
        maximum_displacement_m=float(np.max(np.linalg.norm(xy,axis=1))),endpoint_xy_m=xy[-1].tolist(),
        max_normal_angle_rad=float(np.max(angles)),max_normal_angle_deg=float(np.degrees(np.max(angles))),
        tilted_point_count=int(tilted.sum()),tilted_step_count=int((stepangle>1e-6).sum()),positive_contact_points=len(positive),
        contact_count_range=[int(counts.min()),int(counts.max())],source_empty_steps={str(i):int(np.sum((c['wheel']==i)&(c['state']==0))) for i in range(4)},
        maximum_contact_height_distance_m=float(np.max(abs(points[:,2]))),metadata=d['metadata'],performance=r['formal']['performance'])
    if 'mesh' in e['profile']:
        v,f=read_obj(p/'fixtures'/(e['profile']+'.obj'));v[:,2]-=.05;triangles=v[f]
        low,high=points[:,:2].min(axis=0),points[:,:2].max(axis=0)
        select=(triangles[:,:,:2].min(axis=1)<=high).all(axis=1)&(triangles[:,:,:2].max(axis=1)>=low).all(axis=1)
        assert (v[:,2]==0).all()
        segments=classified_edges(v,f[select]);distances={}
        for category,edges in segments.items():
            if not len(edges):continue
            distances[category]=edge_distance(points,edges)
        report['seam_associations']={k:dict(median_tilted_distance_m=float(np.median(x[tilted])) if tilted.any() else None,
            tilted_within_1mm_fraction=float(np.mean(x[tilted]<=.001)) if tilted.any() else None) for k,x in distances.items()}
        if tilted.any() and len(distances)==2:
            report['tilted_nearest_shared_fraction']=float(np.mean(distances['shared_indices'][tilted]<distances['duplicated_indices'][tilted]-1e-12))
    np.savetxt(out/(e['name']+'-steps.csv'),np.column_stack([active['iteration'],active['t'],xy,np.linalg.norm(xy,axis=1),counts,stepangle]),delimiter=',',header='iteration,t,dx,dy,displacement,contact_count,max_normal_angle',comments='')
    for n in ('result.json','diagnostic-result.json','world.sdf','physics-steps.csv',e['name']+'-contact-steps.csv','actual-diagnostic-libraries.json','collision-metadata.csv'):
        hashes[str((folder/n).relative_to(p))]=digest(folder/n)
    reports.append(report);traces.append((e['name'],active['t']-m['before']['sim'],xy,stepangle))
complete=summary['status']=='diagnostic_complete' and len(reports)==8 and all(x['qualified'] for x in reports)
restoration=[]
if complete:
    for a,b in ((0,6),(1,7)):
        xa,xb=traces[a][2],traces[b][2]
        restoration.append(dict(case=reports[a]['case'],max_trajectory_difference_m=float(np.max(np.linalg.norm(xa-xb,axis=1))),
            normal_angle_difference_rad=abs(reports[a]['max_normal_angle_rad']-reports[b]['max_normal_angle_rad'])))
proof=dict(matrix_complete=complete,attempts=len(ledger),qualified=sum(e.get('qualified',False) for e in ledger),results=reports,
    restoration=restoration,input_hashes=hashes,sole_normal_mechanism_confirmed=False,software_defect_confirmed=False,repair_verified=False,
    production_changed=False,whole_vehicle_accepted=False)
(out/'results.json').write_text(json.dumps(proof,indent=2)+'\n')
fig,axes=plt.subplots(1,3,figsize=(15,4.5))
for name,t,xy,angle in traces:
    axes[0].plot(xy[:,0]*1000,xy[:,1]*1000,label=name)
    axes[1].plot(t,np.linalg.norm(xy,axis=1)*1000,label=name)
    axes[2].plot(t,np.degrees(angle),label=name)
axes[0].set(xlabel='X (mm)',ylabel='Y (mm)',title='Body center trajectories');axes[0].set_aspect('equal',adjustable='datalim')
axes[1].set(xlabel='Motion time (s)',ylabel='Displacement (mm)',title='Actual center displacement')
axes[2].set(xlabel='Motion time (s)',ylabel='Normal difference (deg)',title='Ground contact normals')
for ax in axes:ax.grid(alpha=.3)
fig.legend(*axes[0].get_legend_handles_labels(),loc='lower center',ncol=4,fontsize=8);fig.tight_layout(rect=(0,.14,1,1))
fig.savefig(out/'contact-comparison.png',dpi=160);plt.close(fig)
print(json.dumps(dict(matrix_complete=complete,restoration=restoration,rounds=[{k:r[k] for k in ('name','maximum_displacement_m','max_normal_angle_deg','tilted_step_count','qualified')} for r in reports]),indent=2))
