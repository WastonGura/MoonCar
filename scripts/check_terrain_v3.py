"""Independent readback of the shipped continuous collision surface."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def obj(path):
    vertices,faces=[],[]
    for line in path.read_text(encoding='ascii').splitlines():
        fields=line.split()
        if fields and fields[0]=='v':vertices.append(list(map(float,fields[1:4])))
        elif fields and fields[0]=='f':faces.append([int(v.split('/')[0])-1 for v in fields[1:]])
    return np.asarray(vertices),np.asarray(faces,dtype=int)


def slope_at(axis,z,x,y):
    c=int(np.clip(np.searchsorted(axis,x)-1,0,len(axis)-2))
    r=int(np.clip(np.searchsorted(axis,y)-1,0,len(axis)-2))
    h=axis[1]-axis[0];u=(x-axis[c])/h;v=(y-axis[r])/h
    sw,se,ne,nw=z[r,c],z[r,c+1],z[r+1,c+1],z[r+1,c]
    gx,gy=((se-sw)/h,(ne-se)/h) if v<=u else ((ne-nw)/h,(nw-sw)/h)
    return float(np.degrees(np.arctan(np.hypot(gx,gy))))


def check(package):
    package=Path(package);folder=package/'models/terrain_v3/survey'
    manifest=json.loads((folder/'manifest.json').read_text())
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    assert sha(package/'worlds/mooncar_survey_v3.sdf')==manifest['world_sha256']
    assert sha(package/'worlds/mooncar_survey_v2.sdf')==manifest['preserved_v2_world_sha256']
    assert sha(folder/'height_grid.npz')==manifest['grid_sha256']
    assert sha(folder/'class_mask.png')==manifest['class_mask_sha256']
    for name,digest in manifest['mesh_sha256'].items():assert sha(folder/name)==digest
    for name,digest in manifest['texture_sha256'].items():assert sha(package/'materials/textures'/name)==digest
    with np.load(folder/'height_grid.npz',allow_pickle=False) as data:
        axis,z,cores=data['x'],data['z'],data['measured_cores']
    n=len(axis);step=float(axis[1]-axis[0]);all_faces=[]
    assert np.isfinite(z).all() and z.shape==(n,n)
    for name in manifest['mesh_parts']:
        vertices,faces=obj(folder/(name+'.obj'))
        rc=np.rint((vertices[:,:2]+15)/step).astype(int)
        assert np.all((rc>=0)&(rc<n))
        np.testing.assert_allclose(vertices[:,:2],np.column_stack((axis[rc[:,0]],axis[rc[:,1]])),atol=1e-6,rtol=0)
        np.testing.assert_allclose(vertices[:,2],z[rc[:,1],rc[:,0]],atol=1e-6,rtol=0)
        all_faces.append((rc[:,1]*n+rc[:,0])[faces])
    faces=np.concatenate(all_faces)
    assert len(faces)==2*(n-1)**2
    assert len(np.unique(np.sort(faces,axis=1),axis=0))==len(faces)
    edges=np.concatenate((faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]))
    edges=np.sort(edges,axis=1)
    unique,counts=np.unique(edges,axis=0,return_counts=True)
    boundary=unique[counts==1]
    assert np.all((counts==1)|(counts==2)) and len(boundary)==4*(n-1)
    rows,cols=np.divmod(boundary,n)
    assert np.all(np.all(rows==0,axis=1)|np.all(rows==n-1,axis=1)|np.all(cols==0,axis=1)|np.all(cols==n-1,axis=1))
    xx,yy=np.meshgrid(axis,axis);points=np.column_stack((xx.ravel(),yy.ravel(),z.ravel()))
    triangles=points[faces];normal=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    assert np.all(normal[:,2]>0)
    area=float(normal[:,2].sum()/2)
    assert abs(area-900)<1e-6
    world=ET.parse(package/'worlds/mooncar_survey_v3.sdf').find('world')
    for model in world.findall('model'):
        assert ET.tostring(model.find('link/collision/geometry'))==ET.tostring(model.find('link/visual/geometry'))
    for element in world.iter():
        if element.tag in ('uri','albedo_map','normal_map','roughness_map'):
            assert (package/element.text.removeprefix('model://mooncar_gazebo/')).is_file()
    slopes=np.degrees(np.arctan2(np.linalg.norm(normal[:,:2],axis=1),normal[:,2]))
    centers=triangles.mean(axis=1);bands=[]
    for cx,cy,size in ((4.8,3.6,4.8),(6.,-5.1,6.)):
        distance=np.minimum.reduce((centers[:,0]-(cx-size/2),cx+size/2-centers[:,0],
                                    centers[:,1]-(cy-size/2),cy+size/2-centers[:,1]))
        selected=(distance>=0)&(distance<=.35)
        exterior=(distance<0)&(distance>=-.35)
        bands.append(dict(center_m=[cx,cy],slope_max_deg=float(slopes[selected].max()),
                          slope_p95_deg=float(np.percentile(slopes[selected],95)),
                          over_18deg_area_m2=float((normal[:,2]/2)[selected&(slopes>=18)].sum()),
                          outside_border_slope_max_deg=float(slopes[exterior].max()),
                          outside_border_slope_p95_deg=float(np.percentile(slopes[exterior],95))))
    known_bug_grade=slope_at(axis,z,3.1833333333333336,-7.883333333333333)
    assert known_bug_grade<18,'Known artificial 49.8 degree seam returned'
    return dict(passed=True,version=manifest['version'],triangles=len(faces),vertices=n*n,
                projected_area_m2=area,holes_or_nonmanifold_edges=0,boundary_edges=len(boundary),
                visual_collision_identical=True,grid_matches_written_mesh=True,
                preserved_v2_world_sha256=manifest['preserved_v2_world_sha256'],
                former_49deg_seam_now_deg=known_bug_grade,patch_border_bands=bands,
                whole_surface_max_slope_deg=float(slopes.max()),
                scope='Geometry and resource verification; genuine steep crater faces are retained, not capped at 18 degrees')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package',type=Path,default=ROOT/'ros2_ws/src/mooncar_gazebo')
    parser.add_argument('--report',type=Path)
    args=parser.parse_args();result=check(args.package)
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8',newline='\n')
    print(json.dumps(result),flush=True)
