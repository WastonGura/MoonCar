"""Offline common support model for turns and Nav2 paths; no unknown-ground fill."""
import math
import numpy as np
from swept_resampling import swept_poses,rays
from resampling_dataset import TerrainMap


def action_views(camera,robot,yaw,path=None,window=20.,linear_speed=.2,angular_speed=.5):
    if not all(math.isfinite(v) and v>0 for v in (window,linear_speed,angular_speed)):
        raise ValueError('Positive finite time and nominal speeds required')
    views,_=swept_poses(camera,robot,yaw);views=[(0.,views[0])]
    camera=np.asarray(camera,float).copy();robot=np.asarray(robot,float).copy();elapsed=0.;distance=0.;angle=0.
    def turn(target):
        nonlocal camera,robot,elapsed,angle
        poses,delta=swept_poses(camera,robot,target)
        for i,pose in enumerate(poses[1:],1):views.append((elapsed+delta/angular_speed*i/(len(poses)-1),pose))
        start=math.atan2(robot[1,0],robot[0,0]);a=math.atan2(math.sin(target-start),math.cos(target-start))
        c,s=math.cos(a),math.sin(a);rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        robot[:3,:3]=rotation@robot[:3,:3];camera=poses[-1];elapsed+=delta/angular_speed;angle+=delta
    if path is not None:
        points=np.asarray(path,float)
        if points.ndim!=2 or points.shape[1]!=2 or not 2<=len(points)<=1000 or not np.isfinite(points).all():raise ValueError('Finite bounded XY path required')
        if np.linalg.norm(points[0]-robot[:2,3])>.25 or np.linalg.norm(np.diff(points,axis=0),axis=1).sum()>50:raise ValueError('Path start mismatch or over50m')
        for destination in points[1:]:
            delta=destination-robot[:2,3];length=float(np.linalg.norm(delta))
            if length<1e-8:continue
            turn(math.atan2(delta[1],delta[0]));start=camera.copy()
            for fraction in np.linspace(0,1,math.ceil(length/.1)+1)[1:]:
                pose=start.copy();pose[:2,3]+=fraction*delta
                views.append((elapsed+fraction*length/linear_speed,pose))
            robot[:2,3]=destination;camera=views[-1][1];elapsed+=length/linear_speed;distance+=length
    turn(yaw)
    # ponytail: sampled tangent headings and nominal speeds, not a controller rollout.
    # Keep offline until measured path/heading/time errors justify better dynamics.
    return [p for t,p in views if t<=window],dict(nominal_motion_seconds=elapsed,path_m=distance,turn_rad=angle,
        completed_with_observation_budget=elapsed+3<=window,sampled_poses=int(sum(t<=window for t,p in views)))


def forecast(quality,cloud,camera,robot,yaw,info,pixel_offset,path=None,window=20.):
    poses,cost=action_views(camera,robot,yaw,path,window)
    cloud=np.asarray(cloud,float);resolution=quality['resolution'];voxel=.05
    if cloud.ndim!=2 or cloud.shape[1]!=3 or not np.isfinite(cloud).all() or not len(cloud) or not math.isfinite(resolution) or resolution<2*voxel:
        raise ValueError('Finite observed cloud and supported cell resolution required')
    local=rays(info,pixel_offset);keys=np.floor(cloud[:,:2]/resolution).astype(int);added=[]
    for cell in quality['cells']:
        key=np.array([cell['ix'],cell['iy']]);observed=cloud[np.all(keys==key,axis=1)]
        if not len(observed):raise ValueError('Observed cell has no cloud support')
        height=float(np.median(observed[:,2]))
        # Same unoccluded median-height hypotheses for every action; no scene truth.
        for pose in poses[1:]:
            if np.allclose(pose,poses[0],rtol=0,atol=1e-12):continue
            d=local@pose[:3,:3].T
            t=np.divide(height-pose[2,3],d[:,2],out=np.zeros(len(d)),where=np.abs(d[:,2])>1e-12)
            patch=pose[:3,3]+t[:,None]*d
            mask=(t>=.1)&(t<=6)&np.all(np.floor(patch[:,:2]/resolution).astype(int)==key,axis=1)
            if np.any(mask):added.append(patch[mask])
    def fitted(points):
        # Existing voxel representatives take precedence; hypothetical samples never
        # alter the real archive. One isolated map reuses the exact surface-fit code.
        points=np.asarray(points,dtype=np.float32)
        _,indices=np.unique(np.floor(points/voxel).astype(np.int64),axis=0,return_index=True)
        points=points[np.sort(indices)]
        shadow=TerrainMap(resolution=resolution,source='OFFLINE HYPOTHESES - NEVER OBSERVED DATA')
        for start in range(0,len(points),90000):
            chunk=points[start:start+90000]
            if len(chunk)<3:chunk=np.vstack([chunk,points[:3-len(chunk)]])
            shadow.add_frame(chunk,float(start))
        return {(c['ix'],c['iy']):c['fit_state'] for c in shadow.quality(True)['cells']}
    baseline=fitted(cloud);future=fitted(np.vstack([cloud,*added]));rows=[]
    for cell in quality['cells']:
        key=(cell['ix'],cell['iy']);state=cell['fit_state'];consistent=baseline.get(key)==state
        rows.append(dict(ix=key[0],iy=key[1],baseline_state=state,baseline_consistent=consistent,predicted_state=future.get(key,'missing'),
            recovered=consistent and state in ('too_few_voxels','insufficient_xy_spread') and future.get(key)=='valid',
            lost=consistent and state=='valid' and future.get(key)!='valid'))
    recovered=sum(r['recovered'] for r in rows);lost=sum(r['lost'] for r in rows)
    return rows,dict(model='common-known-surface-v5',**cost,recovered_cells=recovered,lost_cells=lost,
        known_net_m2=(recovered-lost)*resolution**2,known_net_m2_per_budget_second=(recovered-lost)*resolution**2/window,
        inconsistent_baseline_cells=sum(not r['baseline_consistent'] for r in rows),predicted_new_area_m2=None,total_action_rank=None,
        limitations='Known-cell median-height support only; no calibrated occlusion/noise/unknown-surface model. Not an online choice.')


if __name__=='__main__':
    import json
    from scipy.spatial.transform import Rotation
    from swept_resampling import predict
    robot=np.eye(4);camera=np.eye(4);camera[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];camera[:3,3]=[.435,0,.235]
    info=dict(width=320,height=240,k=[277.191,0,160,0,277.191,120,0,0,1])
    cloud=np.array([[2.02,.01,0],[2.08,.01,0],[2.14,.01,0]],dtype=np.float32)
    quality=dict(resolution=.25,cells=[dict(ix=8,iy=0,fit_state='too_few_voxels')])
    original=cloud.copy()
    held,m=forecast(quality,cloud,camera,robot,0.,info,.5)
    assert not held[0]['recovered'] and m['known_net_m2']==0 and m['predicted_new_area_m2'] is None
    a,ac=action_views(camera,robot,.4);b,bc=action_views(camera,robot,.4,[[0,0],[0,0]])
    assert all(np.array_equal(x,y) for x,y in zip(a,b)) and ac==bc
    moved,cost=action_views(camera,robot,0.,[[0,0],[1,0]])
    json.dumps(cost,allow_nan=False)
    assert np.allclose(moved[-1][:3,3],[1.435,0,.235]) and cost['nominal_motion_seconds']==5
    short,cost=action_views(camera,robot,0.,[[0,0],[1,0]],window=2)
    assert short[-1][0,3]<=.835+1e-9 and not cost['completed_with_observation_budget']
    rows,_=forecast(quality,cloud,camera,robot,.4,info,.5)
    assert np.array_equal(cloud,original) and rows[0]['baseline_consistent']
    try:action_views(camera,robot,0.,[[2,0],[3,0]])
    except ValueError:pass
    else:raise AssertionError('Mismatched route start accepted')
    print('PASS identical turn/path semantics, camera lever arm, budget clipping, immutable observations and zero-view baseline')
