"""Read-only force-dependent-slip diagnosis of archived qualified trials.

No controller, force, contact parameter or simulation is changed. Cached-pose
derivatives are labelled estimates and never replace native velocity evidence.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from r5_csv import read_numeric_csv
from r5_origin_analysis import rotation, WHEELS


def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        while chunk:=stream.read(4*1024*1024):digest.update(chunk)
    return digest.hexdigest()


def vector(data,prefix):return np.column_stack([data[prefix+k] for k in 'xyz'])


def stats(values):
    values=np.asarray(values)
    if not values.size or not np.isfinite(values).all():raise ValueError('Missing/nonfinite diagnostic samples')
    return dict(mean=float(values.mean()),rms=float(np.sqrt(np.mean(values**2))),
                p95_abs=float(np.quantile(abs(values),.95)),maximum_abs=float(abs(values).max()))


def tangent(direction,normal):
    direction=np.asarray(direction,float);normal=np.asarray(normal,float)
    if not np.isfinite(direction).all() or not np.isfinite(normal).all():raise ValueError('Nonfinite contact frame')
    if np.any(abs(np.linalg.norm(normal,axis=-1)-1)>1e-6):raise ValueError('Contact normal is not unit length')
    result=direction-np.sum(direction*normal,axis=-1,keepdims=True)*normal
    length=np.linalg.norm(result,axis=-1,keepdims=True)
    if np.any(length<1e-8):raise ValueError('Undefined tangent direction')
    return result/length


def slip_residual(velocity,force,compliance,contacts):
    """DART scales per-point compliance by collision contact multiplicity."""
    arrays=[np.asarray(a,float) for a in (velocity,force,compliance,contacts)]
    if not all(np.isfinite(a).all() for a in arrays):raise ValueError('Nonfinite slip-law input')
    if np.any(arrays[2]<0) or np.any(arrays[3]<1) or np.any(arrays[3]!=np.rint(arrays[3])):
        raise ValueError('Invalid compliance/contact multiplicity')
    return arrays[0]+arrays[1]*arrays[2]*arrays[3]


def contact_multiplicity(iterations,self_collision,other_collision):
    """DART 6.13.2 groups by iteration and unordered collision pair."""
    arrays=[np.asarray(a,float) for a in (iterations,self_collision,other_collision)]
    if not all(a.ndim==1 and np.isfinite(a).all() and (a>=0).all() and np.array_equal(a,np.rint(a)) for a in arrays):
        raise ValueError('Invalid contact identity')
    if len({len(a) for a in arrays})!=1 or not len(arrays[0]):raise ValueError('Missing contact identity')
    if np.any(arrays[1]==arrays[2]):raise ValueError('Self contact identity')
    keys=np.column_stack((arrays[0],np.minimum(arrays[1],arrays[2]),np.maximum(arrays[1],arrays[2])))
    _,inverse,count=np.unique(keys,axis=0,return_inverse=True,return_counts=True)
    return count[inverse]


def audit(folder,output):
    folder=Path(folder);out=Path(output);out.mkdir(parents=True,exist_ok=False)
    source_hash=sha(Path(__file__))
    result=json.loads((folder/'result.json').read_text())
    qualification=json.loads((folder/'suspension-qualification.json').read_text())
    if not qualification['diagnostic_qualified']:raise ValueError('Archived trial is not qualified')
    world=ET.parse(folder/'world.sdf');model=world.find("world/model[@name='mooncar']")
    native=model.find("plugin[@name='gz::sim::systems::WheelSlip']")
    if native is None:raise ValueError('No native WheelSlip configuration')
    settings={w:{k:float(native.find(f"wheel[@link_name='{w}_wheel_link']/{k}").text)
                 for k in ('wheel_radius','wheel_normal_force','slip_compliance_lateral')} for w in WHEELS}
    contact_paths=list(folder.glob('*-contact-steps.csv'))
    if len(contact_paths)!=1:raise ValueError('Ambiguous contact evidence')
    paths=[folder/'physics-steps.csv',folder/'suspension.csv',contact_paths[0],folder/'result.json',
           folder/'world.sdf',folder/'suspension-qualification.json']
    print('Reading '+folder.name,flush=True)
    p=read_numeric_csv(paths[0]);s=read_numeric_csv(paths[1]);c=read_numeric_csv(paths[2])
    n=len(p);dt=p['dt'][0]
    if not np.all(np.diff(p['iteration'])==1) or not np.all(p['dt']==dt):raise ValueError('Incomplete physical grid')
    if not np.array_equal(p['iteration'],s['iteration']) or not np.array_equal(p['t'],s['t']):raise ValueError('Suspension grid mismatch')
    ix=np.searchsorted(p['iteration'],c['iteration'])
    if np.any(ix>=n) or not np.array_equal(p['iteration'][ix],c['iteration']) or not np.array_equal(np.rint(p['t'][ix]*1e9),c['t_ns']):
        raise ValueError('Contact grid mismatch')
    if not np.isin(c['state'],(0,1)).all() or not np.isin(c['wheel'],range(4)).all():raise ValueError('Unknown contact ownership/state')
    start=result['schedule']['motion']['before']['sim'];end=result['schedule']['motion']['after']['sim']
    steady=(p['t']>=start+3)&(p['t']<=end-1);moving=(p['t']>=start-1e-9)&(p['t']<=end+1e-9)
    rotation_world=rotation(np.column_stack([p[k] for k in ('qw','qx','qy','qz')]))
    derivative=np.gradient(rotation_world,dt,axis=0)
    angular_matrix=np.einsum('nij,nkj->nik',derivative,rotation_world)
    omega=np.column_stack([(angular_matrix[:,a,b]-angular_matrix[:,b,a])*.5 for a,b in ((2,1),(0,2),(1,0))])
    base=vector(p,'');com=vector(p,'com_world_');mass=p['total_mass'][0]
    if not np.all(p['total_mass']==mass):raise ValueError('Mass changed')
    base_velocity=vector(s,'front_left_wheel_suspension_joint_parent_world_v')
    positive=c['state']==1;cp=c[positive];index=ix[positive];wi=cp['wheel'].astype(int)
    multiplicities=contact_multiplicity(cp['iteration'],cp['self_collision'],cp['other_collision'])
    force=vector(cp,'f1');normal=vector(cp,'n');point=vector(cp,'')
    if not all(np.isfinite(a).all() for a in (force,normal,point)):raise ValueError('Incomplete positive contact')
    if np.max(np.linalg.norm(force+vector(cp,'f2'),axis=1))>1e-6:raise ValueError('Contact action/reaction mismatch')
    net=np.zeros((n,4,3));normals=np.zeros_like(net);counts=np.zeros((n,4),int)
    np.add.at(net,(index,wi),force);np.add.at(counts,(index,wi),1)
    np.add.at(normals,(index,wi),normal*np.sum(force*normal,axis=1)[:,None])
    wheels={};formulas={}
    for j,wheel in enumerate(WHEELS):
        joint=wheel+'_wheel_suspension_joint'
        if not np.array_equal(vector(s,joint+'_parent_world_v'),base_velocity):raise ValueError('Native parent velocities disagree')
        direction=vector(p,wheel+'_fdir1_');axis=vector(p,wheel+'_axis_')
        formulas[wheel]=dict(maximum_command_error=float(np.max(abs(p[wheel+'_slip1_cmd'][steady]-
            abs(p[wheel+'_spin_before'][steady])*settings[wheel]['wheel_radius']/settings[wheel]['wheel_normal_force']*settings[wheel]['slip_compliance_lateral']))),
            maximum_axis_cross_norm=float(np.linalg.norm(np.cross(direction[steady],axis[steady]),axis=1).max()))
        selected=wi==j;k=index[selected];plane=tangent(direction[k],normal[selected])
        # Body linear velocity and passive-joint velocity are native measurements.
        # Angular velocity is estimated from cached orientations, with the
        # resulting limitation kept in the output rather than called exact.
        velocity=base_velocity[k]+np.cross(omega[k],point[selected]-base[k])
        velocity+=s[joint+'_v'][k,None]*rotation_world[k,:,2]
        velocity+=np.cross(axis[k]*p[wheel+'_wheel_joint_v'][k,None],point[selected]-vector(p,wheel+'_world_')[k])
        lateral_velocity=np.sum(velocity*plane,axis=1);lateral_force=np.sum(force[selected]*plane,axis=1)
        normal_force=abs(np.sum(force[selected]*normal[selected],axis=1))
        compliance=p[wheel+'_slip1_cmd'][k];multiplicity=multiplicities[selected]
        residual=slip_residual(lateral_velocity,lateral_force,compliance,multiplicity)
        mu=float(model.find(f"link[@name='{wheel}_wheel_link']/collision/surface/friction/ode/mu").text)
        valid=steady[k]&(normal_force>1e-6)
        unsaturated=valid&(abs(lateral_force)<mu*normal_force*.999)
        wheels[wheel]=dict(compliance_m_s_N=stats(p[wheel+'_slip1_cmd'][steady]),
            contact_point_count=stats(counts[steady,j]),empty_fraction=float((counts[steady,j]==0).mean()),
            pair_contact_multiplicity=stats(multiplicity[valid]),
            lateral_velocity_estimate_m_s=stats(lateral_velocity[valid]),
            corrected_contact_law_residual_estimate_m_s=stats(residual[unsaturated]),
            old_single_point_residual_estimate_m_s=stats((lateral_velocity+compliance*lateral_force)[unsaturated]),
            friction_saturated_point_fraction=float((valid&~unsaturated).sum()/valid.sum()),
            corrected_samples=int(unsaturated.sum()),mu=mu)
    gravity=np.array(list(map(float,world.findtext('world/gravity').split())))
    # Derive the reference from the actual frozen plane-normal-gravity world,
    # rather than duplicating rounded terrain gradients in this diagnostic.
    # On curved terrain this is only a reference direction; the force audit
    # above uses every local contact normal.
    runtime=folder.parent.parent
    manifest_path=runtime/'prepared/manifest.json'
    manifest=json.loads(manifest_path.read_text())
    normal_spec=next(row for row in manifest['rounds'] if row['scene']=='plane_normal')
    normal_path=runtime/Path(normal_spec['world']).relative_to('/diagnosis')
    if sha(normal_path)!=normal_spec['world_sha256']:raise ValueError('Reference world changed')
    normal_gravity=np.array(list(map(float,ET.parse(normal_path).findtext('world/gravity').split())))
    reference=-normal_gravity/np.linalg.norm(normal_gravity)
    own_spec=next(row for row in manifest['rounds'] if row['name']==folder.name)
    if own_spec['scene']=='flat':reference=np.array([0.,0.,1.])
    tangent_gravity=gravity-reference*np.dot(gravity,reference)
    slope_normal=-normal_gravity/np.linalg.norm(normal_gravity)
    downhill=np.r_[slope_normal[:2],0.];downhill/=np.linalg.norm(downhill)
    lateral=rotation_world[:,:,1];lat_tangent=tangent(lateral,np.tile(reference,(n,1)))
    total=net.sum(axis=1);normal_total=normals.sum(axis=1)
    force_tangent=total-normal_total
    # Quasi-static prediction from the already frozen constitutive law, with
    # no fitted coefficient. It omits inertia/curvature and is not a replacement
    # world trajectory or an independently executed causal intervention.
    inv=np.column_stack([1/np.maximum(p[w+'_slip1_cmd'],1e-30) for w in WHEELS]).sum(axis=1)
    estimate=mass*np.einsum('ij,j->i',lat_tangent,tangent_gravity)/inv
    gravity_velocity=lat_tangent*estimate[:,None]
    start_i,end_i=np.flatnonzero(moving)[[0,-1]]
    displacement=com[end_i]-com[start_i]
    out_record=dict(scope='Archived qualified data; zero new physical trials',folder=str(folder),physical_steps=n,
        analyzer_sha256=source_hash,reference_world=str(normal_path),
        gravity_m_s2=gravity.tolist(),reference_tangential_gravity_m_s2=tangent_gravity.tolist(),
        reference_tangential_gravity_force_N=float(mass*np.linalg.norm(tangent_gravity)),
        command_checks=formulas,wheels=wheels,
        reference_endpoint_m=(base[end_i]-base[start_i]).tolist(),com_endpoint_m=displacement.tolist(),
        downhill_com_endpoint_mm=float(displacement@downhill*1000),
        reference_downhill_endpoint_mm=float((base[end_i]-base[start_i])@downhill*1000),
        quasi_static_gravity_only_displacement_estimate_mm=(gravity_velocity[moving].sum(axis=0)*dt*1000).tolist(),
        mean_net_contact_force_N=total[steady].mean(axis=0).tolist(),
        mean_normal_force_N=normal_total[steady].mean(axis=0).tolist(),
        mean_friction_force_N=force_tangent[steady].mean(axis=0).tolist(),
        gravity_work_J=float(mass*gravity@displacement),
        native_base_lateral_speed_m_s=stats(np.sum(base_velocity*lateral,axis=1)[steady]),
        production_changed=False,physical_parameters_changed=False,
        limitations=['Contact velocity uses cached-orientation rate estimate, not a native point-velocity measurement',
            'Quasi-static prediction omits inertia, curvature, steering transients and local normal variation',
            'Archived causal contrasts retain their original batch counts; no new physical acceptance'])
    (out/'summary.json').write_text(json.dumps(out_record,indent=2,allow_nan=False)+'\n')
    if sha(Path(__file__))!=source_hash:raise ValueError('Analyzer changed while reading evidence')
    (out/'sources.json').write_text(json.dumps({str(p.resolve()):sha(p) for p in paths+[manifest_path,normal_path]},indent=2)+'\n')
    print(json.dumps(dict(folder=folder.name,downhill_mm=out_record['reference_downhill_endpoint_mm'],
        predicted_mm=out_record['quasi_static_gravity_only_displacement_estimate_mm'],
        residuals={w:r['corrected_contact_law_residual_estimate_m_s']['rms'] for w,r in wheels.items()})),flush=True)
    return out_record


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--round',required=True)
    parser.add_argument('--output',required=True);args=parser.parse_args();audit(args.round,args.output)
