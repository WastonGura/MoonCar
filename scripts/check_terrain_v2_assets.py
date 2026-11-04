"""Independent readback checks for v2 resources and preserved ground geometry."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from scipy.spatial import ConvexHull


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def obj(path):
    vertices,uv,faces = [],[],[]
    for line in path.read_text().splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == 'v': vertices.append(list(map(float,fields[1:4])))
        elif fields[0] == 'vt': uv.append(list(map(float,fields[1:3])))
        elif fields[0] == 'f': faces.append([int(f.split('/')[0])-1 for f in fields[1:]])
    return np.array(vertices),np.array(uv),np.array(faces)


def check(root, baseline=None):
    result = {}
    for scene,size in (('ramp',16),('curve',16),('survey',30)):
        folder = root/'models/terrain_v2'/scene
        world_path = root/f'worlds/mooncar_{scene}_v2.sdf'
        world = ET.parse(world_path).find('world')
        manifest = json.loads((folder/'manifest.json').read_text())
        assert manifest['world_sha256'] == sha(world_path)
        for name,digest in manifest['mesh_sha256'].items(): assert sha(folder/name) == digest,name
        for name,digest in manifest['texture_sha256'].items(): assert sha(root/'materials/textures'/name) == digest,name
        for name,digest in manifest['rock_variant_sha256'].items(): assert sha(folder.parent/name) == digest,name
        for element in world.iter():
            if element.tag in ('uri','albedo_map','normal_map','roughness_map'):
                assert (root.parent/element.text.removeprefix('model://')).is_file(),element.text
        for model in world.findall('model'):
            for link in model.findall('link'):
                collision,visual = link.find('collision/geometry'),link.find('visual/geometry')
                assert ET.tostring(collision) == ET.tostring(visual),model.get('name')
        atlas_sizes = []
        for kind in ('albedo','normal','roughness'):
            with Image.open(root/f'materials/textures/{scene}_surface_{kind}.png') as image:
                image.load(); assert image.size == (size*128,size*128)
                atlas_sizes.append(image.size)
        for path in folder.glob('*.obj'):
            if path.stem == 'surface_pebbles': continue
            vertices,uv,faces = obj(path)
            expected = (vertices[:,:2]+size/2)/size
            assert np.allclose(uv,expected,atol=1e-8,rtol=0),path
            assert np.isfinite(vertices).all() and np.isfinite(uv).all()
        for variant in range(6):
            vertices,_,faces = obj(folder.parent/f'boulder_{variant:02d}.obj')
            unique,inverse = np.unique(vertices,axis=0,return_inverse=True)
            welded = inverse[faces]
            edges = Counter(tuple(sorted((int(f[i]),int(f[(i+1)%3])))) for f in welded for i in range(3))
            assert all(count == 2 for count in edges.values()),'Open rock mesh'
            triangles = vertices[faces]
            volume = np.einsum('ij,ij->i',triangles[:,0],np.cross(triangles[:,1],triangles[:,2])).sum()/6
            assert volume > 0 and abs(volume-ConvexHull(unique).volume) < 1e-8,'Non-convex/inverted rock'
        with np.load(folder/'height_grid.npz') as data:
            xs,ys,z = data['x'],data['y'],data['z']
        def ground(x,y):
            c = np.clip(np.searchsorted(xs,x,side='right')-1,0,len(xs)-2)
            r = np.clip(np.searchsorted(ys,y,side='right')-1,0,len(ys)-2)
            u,v = (x-xs[c])/(xs[c+1]-xs[c]),(y-ys[r])/(ys[r+1]-ys[r])
            a,b,d,e = z[r,c],z[r,c+1],z[r+1,c+1],z[r+1,c]
            return a+u*(b-a)+v*(d-b) if v<=u else a+u*(d-e)+v*(e-a)
        mask = np.flipud(np.asarray(Image.open(folder/'class_mask.png')))
        pebble_vertices,_,pebble_faces = obj(folder/'surface_pebbles.obj')
        for instance in manifest['pebbles']['instances']:
            x,y,center = instance['position_m']; diameter = instance['diameter_m']; radius=diameter/2
            assert .03 <= diameter <= .10
            assert np.hypot(x,y) >= 1.5+radius
            assert not (-radius <= x <= 2.5+radius and abs(y) <= .6+radius)
            if scene=='survey':
                for px,py,width in ((4.8,3.6,4.8),(6,-5.1,6)):
                    assert not (abs(x-px)<=width/2+.5+radius and abs(y-py)<=width/2+.5+radius)
            assert abs(ground(x,y)-instance['ground_z_m']) < 1e-9
            for px,py in ((x,y),(x-radius,y),(x+radius,y),(x,y-radius),(x,y+radius)):
                assert mask[np.searchsorted(ys,py)-1,np.searchsorted(xs,px)-1] in (2,3)
        # Merged gravel remains a set of closed components after on-disk rounding.
        unique,inverse = np.unique(pebble_vertices,axis=0,return_inverse=True)
        edges = Counter(tuple(sorted((int(f[i]),int(f[(i+1)%3])))) for f in inverse[pebble_faces] for i in range(3))
        assert all(count==2 for count in edges.values()),'Open merged pebble component'
        preserved = None
        if baseline:
            oldfolder = baseline/'models/terrain_v2'/scene
            oldmanifest = json.loads((oldfolder/'manifest.json').read_text())
            with np.load(oldfolder/'height_grid.npz') as old:
                # NumPy builds can differ below 1e-16; the written collision vertices
                # below must remain exactly equal at their original precision.
                assert np.allclose(old['z'],z,atol=1e-12,rtol=0) and np.array_equal(old['x'],xs) and np.array_equal(old['y'],ys)
            assert np.array_equal(np.asarray(Image.open(folder/'class_mask.png')),
                                  np.asarray(Image.open(oldfolder/'class_mask.png')))
            for name in oldmanifest['mesh_sha256']:
                original,_,faces = obj(oldfolder/name)
                current,_,newfaces = obj(folder/name)
                assert np.array_equal(original,current) and np.array_equal(faces,newfaces),name
            assert manifest['class_parameters']==oldmanifest['class_parameters']
            assert manifest['measured_patch']==oldmanifest['measured_patch']
            if scene=='survey': assert manifest['measured_background_patch']==oldmanifest['measured_background_patch']
            assert len(manifest['boulders'])==len(oldmanifest['boulders'])
            for new,old in zip(manifest['boulders'],oldmanifest['boulders']):
                assert new['position_m'][:2]==old['position_m'][:2]
                assert new['radii_m'][:2]==old['radii_m'][:2]
            # Per-ground collision coefficients must also survive SDF serialization.
            oldworld = ET.parse(baseline/f'worlds/mooncar_{scene}_v2.sdf').find('world')
            for oldmodel in oldworld.findall('model'):
                oldfriction = oldmodel.find('link/collision/surface/friction')
                newfriction = world.find(f"model[@name='{oldmodel.get('name')}']/link/collision/surface/friction")
                assert ET.tostring(oldfriction)==ET.tostring(newfriction)
            preserved=True
        if scene=='curve':
            model = world.find("model[@name='boulder_01']")
            pose = list(map(float,model.findtext('pose').split()))
            mesh = model.find('link/collision/geometry/mesh')
            vertices,_,_ = obj(root.parent/mesh.findtext('uri').removeprefix('model://'))
            top = pose[2]+vertices[:,2].max()*float(mesh.findtext('scale').split()[2])-ground(pose[0],pose[1])
            assert abs(top-.41) < .002,top
        result[scene] = dict(passed=True,ground_preserved=preserved,atlas_sizes=atlas_sizes,
                             pebbles=manifest['pebbles']['count'],world_sha256=sha(world_path))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('root',type=Path)
    parser.add_argument('--baseline',type=Path)
    parser.add_argument('--report',type=Path)
    args=parser.parse_args()
    report=check(args.root,args.baseline)
    if args.report: args.report.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
