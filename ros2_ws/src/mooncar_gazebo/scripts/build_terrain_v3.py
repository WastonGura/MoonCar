"""Build a versioned continuous survey surface; V2 assets remain untouched.

Measured interiors constrain one global, screened minimum-curvature correction.
No rectangular feather band is applied inside the measured data. A common grid
and triangle diagonal serve visual/collision surfaces and material partitions.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from scipy import sparse
from scipy.ndimage import gaussian_filter
from scipy.sparse.linalg import spsolve

import build_terrain_v2 as v2
from generate_terrain_materials import bake_scene
from terrain_scene_detail import rock_variants, sample_ground, write_rocks

PACKAGE = Path(__file__).resolve().parents[1]
STEP = .1
INFLUENCE_LENGTH_M = 2.5  # Engineering interpolation length, not a vehicle limit.
SEED = 20261117


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def laplacian(n):
    off = -np.ones(n*n-1)
    off[np.arange(1,n)*n-1] = 0
    return sparse.diags((-np.ones(n*n-n), off, np.full(n*n,4.), off, -np.ones(n*n-n)),
                        (-n,-1,0,1,n), shape=(n*n,n*n), format='csr')


def background(x,y,step):
    """Coherent multiscale relief, with an explicit parked-vehicle launch pad."""
    rng=np.random.default_rng(SEED)
    result=np.zeros(x.shape)
    for length,amplitude in ((3.5,.025),(1.0,.009),(.25,.0015)):
        relief=gaussian_filter(rng.standard_normal(x.shape),length/step,mode='reflect')
        relief/=max(float(relief.std()),1e-12)
        result+=amplitude*relief
    result+=v2.crater(x,y,-6.,7.,v2.SURVEY_CRATER_RADIUS,
                     v2.SURVEY_CRATER_DEPTH_RATIO*(2*v2.SURVEY_CRATER_RADIUS)/1.45)
    result+=.18*np.exp(-(((x+5)/1.7)**2+((y+6)/1.2)**2))
    return result*v2.smoothstep((np.hypot(x,y)-1.2)/1.0)


def surface(step=STEP):
    count = round(30/step)+1
    axis = np.linspace(-15.,15.,count)
    x,y = np.meshgrid(axis,axis)
    base = background(x,y,step)
    fixed = np.zeros(x.shape,dtype=bool)
    values = np.zeros(x.shape)
    cores = np.zeros(x.shape,dtype=np.uint8)
    inputs = []
    for code, name, source_path, center, size in (
        (1,'fresh_crater',v2.PATCH_SOURCE,v2.PATCH_CENTER,v2.PATCH_SIZE),
        (2,'measured_background',v2.BACKGROUND_SOURCE,v2.BACKGROUND_CENTER,v2.BACKGROUND_SIZE)):
        with np.load(source_path,allow_pickle=False) as source:
            local_x,local_y,data = source['x'],source['y'],source['z']
            selection=(np.abs(x-center[0])<=size/2+1e-8)&(np.abs(y-center[1])<=size/2+1e-8)
            cols=np.rint((x[selection]-center[0]-local_x[0])/(local_x[1]-local_x[0])).astype(int)
            rows=np.rint((y[selection]-center[1]-local_y[0])/(local_y[1]-local_y[0])).astype(int)
            target=data[rows,cols]+(base[selection] if code==1 else 0.)
        values[selection]=target-base[selection]
        fixed[selection]=True;cores[selection]=code
        inputs.append(dict(name=name,center_m=list(center),size_m=size,source_sha256=sha(source_path),
                           scope='Original-scale 5 cm source sampled at shared 10 cm grid; no vertical scaling'))
    # Preserve the existing launch pad, synthetic crater, and domain rim.
    protected=(np.hypot(x,y)<=1.2)|(np.hypot(x+6,y-7)<=3.3)
    protected[:2,:]=protected[-2:,:]=True
    protected[:,:2]=protected[:,-2:]=True
    if np.any(protected & (cores>0)):
        raise ValueError('Measured cores overlap protected geometry')
    fixed|=protected
    operator=laplacian(count)
    damping=(step/INFLUENCE_LENGTH_M)**4
    energy=(operator.T@operator+sparse.eye(count*count,format='csr')*damping).tocsr()
    unknown=np.flatnonzero(~fixed.ravel());known=np.flatnonzero(fixed.ravel())
    matrix=energy[unknown][:,unknown].tocsc()
    rhs=-(energy[unknown][:,known]@values.ravel()[known])
    solution=spsolve(matrix,rhs,permc_spec='MMD_AT_PLUS_A')
    residual=float(np.linalg.norm(matrix@solution-rhs)/max(np.linalg.norm(rhs),1e-12))
    if not np.isfinite(solution).all() or residual>1e-8:
        raise ValueError('Surface solver did not converge')
    correction=values.copy().ravel();correction[unknown]=solution
    z=np.round(base+correction.reshape(x.shape),6)
    if np.max(np.abs(z[fixed]-(base+values)[fixed]))>5.01e-7:
        raise ValueError('Measured/protected constraints changed')
    return axis,z,cores,dict(method='Global screened minimum-curvature residual',
        energy='||L c||^2 + (grid_step/influence_length)^4 ||c||^2, fixed measured cores and protected zones',
        influence_length_m=INFLUENCE_LENGTH_M,relative_linear_residual=residual,
        measured_inputs=inputs,unknown_vertices=len(unknown),fixed_vertices=len(known))


def write_grid_mesh(path,axis,z,normals,labels,class_id):
    cells=np.argwhere(labels==class_id)
    n=len(axis)
    sw=cells[:,0]*n+cells[:,1]
    faces=np.concatenate((np.column_stack((sw,sw+1,sw+n+1)),np.column_stack((sw,sw+n+1,sw+n))))
    ids,indices=np.unique(faces,return_inverse=True)
    faces=indices.reshape(-1,3)+1
    rows,cols=np.divmod(ids,n)
    with path.open('w',encoding='ascii',newline='\n') as stream:
        stream.write('# Terrain V3 shared vertices; visual and collision use this exact mesh\n')
        for r,c in zip(rows,cols):
            stream.write(f'v {axis[c]:.6f} {axis[r]:.6f} {z[r,c]:.6f}\n')
        for r,c in zip(rows,cols):
            stream.write(f'vt {(axis[c]+15)/30:.9f} {(axis[r]+15)/30:.9f}\n')
        for r,c in zip(rows,cols):
            stream.write('vn '+' '.join(f'{v:.8f}' for v in normals[r,c])+'\n')
        for a,b,c in faces:
            stream.write(f'f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}\n')
    return dict(vertices=len(ids),triangles=len(faces),projected_area_m2=len(cells)*STEP**2)


def build(output):
    output=Path(output)
    folder=output/'models/terrain_v3/survey';folder.mkdir(parents=True,exist_ok=True)
    (output/'worlds').mkdir(parents=True,exist_ok=True)
    axis,z,cores,construction=surface()
    xx,yy=np.meshgrid((axis[:-1]+axis[1:])/2,(axis[:-1]+axis[1:])/2)
    center_z=(z[:-1,:-1]+z[1:,1:])/2
    labels=v2.material_ids('survey',xx,yy,center_z)
    gy,gx=np.gradient(z,axis,axis)
    normals=np.stack((-gx,-gy,np.ones_like(z)),axis=-1)
    normals/=np.linalg.norm(normals,axis=-1,keepdims=True)
    models=[];parts={}
    for name,config in v2.CLASSES.items():
        if not np.any(labels==config['id']):continue
        path=folder/f'{name}.obj'
        parts[name]=write_grid_mesh(path,axis,z,normals,labels,config['id'])
        model=ET.fromstring(v2.model_xml('survey',name,config))
        for element in model.iter('uri'):
            element.text=f'model://mooncar_gazebo/models/terrain_v3/survey/{name}.obj'
        for element in model.iter():
            if element.tag in ('albedo_map','normal_map','roughness_map'):
                element.text=element.text.replace('survey_surface','survey_v3_surface')
        models.append(model)
    old_manifest=json.loads((PACKAGE/'models/terrain_v2/survey/manifest.json').read_text())
    boulders=[]
    for index,rock in enumerate(old_manifest['boulders'],1):
        rock=copy.deepcopy(rock)
        x,y,_=rock['position_m'];rx,ry,rz=rock['radii_m']
        ground=sample_ground(axis,axis,z,x,y)
        rock['position_m'][2]=ground+.65*rz;rock['ground_z_m']=ground
        models.append(ET.fromstring(v2.boulder_xml(index,x,y,rock['position_m'][2],rx,ry,rz,rock['variant'])))
        boulders.append(rock)
    variants=rock_variants();components=[];pebbles=[]
    for original in old_manifest['pebbles']['instances']:
        rock=copy.deepcopy(original)
        x,y,_=rock['position_m'];diameter=rock['diameter_m']
        vertices,faces=variants[rock['variant']];vertices=vertices.copy()
        angle=rock['yaw_rad'];c,s=np.cos(angle),np.sin(angle)
        vertices[:,:2]=vertices[:,:2]@np.array([[c,s],[-s,c]])
        vertices[:,:2]*=(diameter/2)/np.linalg.norm(vertices[:,:2],axis=1).max()
        vertices[:,2]*=diameter*.25;vertices[:,:2]+=(x,y)
        ground=sample_ground(axis,axis,z,x,y);center=ground-diameter*.025
        vertices[:,2]+=center
        components.append((vertices,faces))
        rock['position_m'][2]=center;rock['ground_z_m']=ground;pebbles.append(rock)
    write_rocks(folder/'surface_pebbles.obj',components)
    model=ET.fromstring(v2.model_xml('survey','surface_pebbles',{**v2.CLASSES['rock_boulder'],'appearance':'rock_boulder'}))
    for uri in model.iter('uri'):uri.text='model://mooncar_gazebo/models/terrain_v3/survey/surface_pebbles.obj'
    models.append(model)
    # The existing atlas renderer already uses continuous world UV and softly blended materials.
    texture_dir=output/'materials/textures'
    atlas=bake_scene('survey',labels,30.,texture_dir)
    for kind in ('albedo','normal','roughness'):
        (texture_dir/f'survey_surface_{kind}.png').rename(texture_dir/f'survey_v3_surface_{kind}.png')
    tree=ET.ElementTree(ET.fromstring(v2.world_xml('survey',[])))
    world=tree.find('world');world.set('name','mooncar_survey_v3')
    world.find('physics').set('name','mooncar_v3_physics')
    for model in models:world.append(model)
    ET.indent(tree,space='  ')
    world_path=output/'worlds/mooncar_survey_v3.sdf'
    tree.write(world_path,encoding='utf-8',xml_declaration=True)
    np.savez_compressed(folder/'height_grid.npz',x=axis,y=axis,z=z,measured_cores=cores)
    Image.fromarray(np.flipud(labels)).save(folder/'class_mask.png')
    manifest=dict(version='terrain-v3-continuous-v1',world=world_path.name,size_m=30.,grid_step_m=STEP,
        height_min_m=float(z.min()),height_max_m=float(z.max()),construction=construction,seed=SEED,
        class_parameters=v2.CLASSES,mesh_parts=parts,boulders=boulders,pebbles=pebbles,surface_atlas=atlas,
        preserved_v2_world_sha256=sha(PACKAGE/'worlds/mooncar_survey_v2.sdf'),
        source_manifest_sha256=sha(PACKAGE/'models/terrain_v2/survey/manifest.json'),
        world_sha256=sha(world_path),generator_sha256=sha(__file__),
        mesh_sha256={p.name:sha(p) for p in sorted(folder.glob('*.obj'))},
        grid_sha256=sha(folder/'height_grid.npz'),class_mask_sha256=sha(folder/'class_mask.png'),
        shared_rock_sha256={p.name:sha(p) for p in sorted((PACKAGE/'models/terrain_v2').glob('boulder_*.obj'))},
        texture_sha256={p.name:sha(p) for p in sorted(texture_dir.glob('survey_v3_surface_*.png'))},
        scope='Engineered continuous simulation scene with measured shape cores; contact parameters remain engineering assumptions')
    (folder/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(output=str(output),solver=construction,triangles=sum(p['triangles'] for p in parts.values()),
                         height_range=[float(z.min()),float(z.max())])),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',type=Path,required=True)
    build(parser.parse_args().output_root)
