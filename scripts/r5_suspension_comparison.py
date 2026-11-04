"""Input identity, rigid restoration and common-angle comparison, without control."""
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from r5_csv import read_numeric_csv
from r5_suspension import WHEELS


def first_crossings(theta,points,grid):
    maximum=np.maximum.accumulate(theta)
    keep=np.r_[True,np.diff(maximum)>1e-10]
    return np.column_stack([np.interp(grid,maximum[keep],points[keep,j]) for j in range(points.shape[1])])


def compare_campaign(campaign,manifest,results,out):
    available={r['name']:r for r in results};cache={}
    def data(index):
        spec=manifest['rounds'][index]
        if spec['name'] not in available:return None
        if index not in cache:cache[index]=read_numeric_csv(campaign/spec['name']/'physics-steps.csv',
            usecols=list(dict.fromkeys(input_fields+trajectory_fields)))
        return cache[index]
    pairs=[];restores=[];angle=[]
    input_fields=['iteration','t','dt']+['input_'+k for k in ('lx','ly','lz','ax','ay','az')]+[w+'_wheel_joint_velocity_cmd' for w in WHEELS]
    trajectory_fields=['iteration','t','dt','x','y','z','qw','qx','qy','qz']+['com_world_'+k for k in 'xyz']+[w+'_wheel_joint_'+k for w in WHEELS for k in ('q','v')]
    for left,right in [(0,2),(1,3),(10,12),(11,13)]:
        a,b=data(left),data(right)
        if a is None or b is None:continue
        equal=all(np.array_equal(a[k],b[k]) for k in input_fields)
        pairs.append(dict(rounds=[manifest['rounds'][i]['name'] for i in (left,right)],passed=equal))
    for base,restore in [(0,14),(1,15)]:
        a,b=data(base),data(restore)
        if a is None or b is None:continue
        restores.append(dict(rounds=[manifest['rounds'][i]['name'] for i in (base,restore)],
            passed=all(np.array_equal(a[k],b[k]) for k in input_fields+trajectory_fields)))
    normal_world=next(s for s in manifest['rounds'] if s['scene']=='plane_normal')['world']
    downhill=-np.array([float(x) for x in ET.parse(normal_world).findtext('world/gravity').split()])[:2]
    downhill/=np.linalg.norm(downhill)
    for index,spec in enumerate(manifest['rounds']):
        if spec['name'] not in available:continue
        row=available[spec['name']];row['reference_slope_downhill_unit_xy']=downhill.tolist()
        row['reference_slope_downhill_endpoint_mm']=float(np.dot(row['endpoint_xy_mm'],downhill))
        row['com_reference_slope_downhill_endpoint_mm']=float(np.dot(row['com_endpoint_mm'][:2],downhill))
    for left,right in [(0,2),(1,3)]:
        tracks=[]
        for index in (left,right):
            p=data(index)
            if p is None:break
            name=manifest['rounds'][index]['name'];import json
            schedule=json.loads((campaign/name/'result.json').read_text())['schedule']['motion']
            p=p[(p['t']>=schedule['before']['sim']-1e-9)&(p['t']<=schedule['after']['sim']+1e-9)]
            w,x,y,z=(p[k] for k in ('qw','qx','qy','qz'))
            yaw=np.unwrap(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))
            theta=(yaw-yaw[0])*np.sign(manifest['rounds'][index]['angular'])
            points=np.column_stack([p[k]-p[k][0] for k in ('x','y','com_world_x','com_world_y')])
            tracks.append((theta,points))
        if len(tracks)!=2:continue
        limit=min(t[0][-1] for t in tracks);grid=np.linspace(0,limit,2001)
        a,b=[first_crossings(t,p,grid) for t,p in tracks]
        target=out/f'common-angle-{left+1:02d}-{right+1:02d}.csv'
        np.savetxt(target,np.column_stack([np.degrees(grid),a,b]),delimiter=',',
            header='angle_deg,rigid_dx,rigid_dy,rigid_com_dx,rigid_com_dy,susp_dx,susp_dy,susp_com_dx,susp_com_dy',comments='')
        angle.append(dict(rounds=[manifest['rounds'][i]['name'] for i in (left,right)],
            common_angle_deg=float(np.degrees(limit)),file=str(target),
            reference_max_mm=[float(np.linalg.norm(t[:,:2],axis=1).max()*1000) for t in (a,b)],
            com_max_mm=[float(np.linalg.norm(t[:,2:],axis=1).max()*1000) for t in (a,b)],
            method='First crossings of monotonically accumulated actual yaw; 2001 common-angle samples'))
    return dict(input_pairs=pairs,rigid_restoration=restores,common_angle=angle,
                passed=len(pairs)==4 and len(restores)==2 and len(angle)==2 and all(r['passed'] for r in pairs+restores))
