"""Prepare genuinely planar research terrain and reject mislabeled flat scenes."""
import copy
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import itertools
import numpy as np
from audit_r5_mass import sdf_pose


def area(poly):
    return abs(sum(a[0]*b[1]-b[0]*a[1]
                   for a,b in zip(poly,poly[1:]+poly[:1]))) / 2


def clip(poly, a, b, inside):
    """Clip one convex polygon against an oriented triangle edge."""
    result=[]
    def side(p): return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])
    for p,q in zip(poly,poly[1:]+poly[:1]):
        sp,sq=side(p),side(q)
        ip,iq=(sp>=0,sq>=0) if inside else (sp<=0,sq<=0)
        if ip: result.append(p)
        if ip != iq:
            t=sp/(sp-sq);result.append((p[0]+t*(q[0]-p[0]),p[1]+t*(q[1]-p[1])))
    return result


def uncovered_area(triangles, lo, hi):
    remaining=[[(lo,-.5),(hi,-.5),(hi,.5),(lo,.5)]]
    for triangle in triangles:
        a,b,c=triangle
        cross=(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
        if abs(cross)<1e-18: raise ValueError('Degenerate ground triangle')
        if cross<0: triangle=[a,c,b]
        next_remaining=[]
        for polygon in remaining:
            if (max(p[0] for p in polygon)<min(p[0] for p in triangle) or
                min(p[0] for p in polygon)>max(p[0] for p in triangle) or
                max(p[1] for p in polygon)<min(p[1] for p in triangle) or
                min(p[1] for p in polygon)>max(p[1] for p in triangle)):
                next_remaining.append(polygon);continue
            inside=polygon
            for a,b in zip(triangle,triangle[1:]+triangle[:1]):
                outside=clip(inside,a,b,False)
                if len(outside)>=3 and area(outside)>1e-14: next_remaining.append(outside)
                inside=clip(inside,a,b,True)
                if len(inside)<3: break
        remaining=next_remaining
        if not remaining: break
    return sum(area(poly) for poly in remaining)


def obstacle_bounds(model):
    """Conservative transformed collision envelope; reject unsupported frames."""
    for link in model.findall('link'):
        for collision in link.findall('collision'):
            nodes=[model,link,collision]
            if any(n.find('pose') is not None and n.find('pose').get('relative_to') for n in nodes):
                raise ValueError('Unsupported obstacle relative frame')
            matrix=np.eye(4)
            for n in nodes: matrix=matrix@sdf_pose(n.find('pose'))
            g=collision.find('geometry')
            if g.find('box') is not None:
                half=np.array([float(x) for x in g.findtext('box/size').split()])/2
            elif g.find('sphere') is not None:
                half=np.repeat(float(g.findtext('sphere/radius')),3)
            elif g.find('cylinder') is not None:
                r=float(g.findtext('cylinder/radius'));half=np.array([r,r,float(g.findtext('cylinder/length'))/2])
            elif g.find('mesh') is not None:
                mesh=g.find('mesh');points=[]
                for line in Path(mesh.findtext('uri')).read_text().splitlines():
                    f=line.split()
                    if f and f[0]=='v':points.append([float(x) for x in f[1:4]])
                if not points:raise ValueError('Empty obstacle mesh')
                points=np.array(points)*np.array([float(x) for x in mesh.findtext('scale','1 1 1').split()])
                low,high=points.min(axis=0),points.max(axis=0)
                corners=np.array(list(itertools.product(*zip(low,high))))
                transformed=corners@matrix[:3,:3].T+matrix[:3,3]
                yield transformed.min(axis=0),transformed.max(axis=0);continue
            else:raise ValueError('Unsupported obstacle collision geometry')
            corners=np.array(list(itertools.product(*zip(-half,half))))
            transformed=corners@matrix[:3,:3].T+matrix[:3,3]
            yield transformed.min(axis=0),transformed.max(axis=0)


def grounds(tree):
    return [m for m in tree.findall('world/model')
            if m.get('name', '').endswith('_ground')]


def flat_world(original, assets):
    tree = copy.deepcopy(original)
    assets = Path(assets)
    assets.mkdir(parents=True, exist_ok=False)
    replacements = {}
    for model in grounds(tree):
        for uri in model.findall('.//geometry/mesh/uri'):
            source = Path(uri.text)
            if source not in replacements:
                target = assets / source.name
                lines = []
                for line in source.read_text().splitlines():
                    fields = line.split()
                    if fields and fields[0] == 'v':
                        fields[3] = '0'
                        line = ' '.join(fields)
                    elif fields and fields[0] == 'vn':
                        line = 'vn 0 0 1'
                    lines.append(line)
                target.write_text('\n'.join(lines) + '\n')
                replacements[source] = target
            uri.text = str(replacements[source])
    validate_flat(tree, 2.0)
    return tree


def validate_flat(tree, distance):
    """Require planar identity-frame meshes covering the full commanded route.

    The research rover starts at the origin. A 0.5 m margin includes its
    collision envelope; ground classes retain their existing triangle masks.
    """
    vertices = []
    triangles = []
    hashes = {}
    for model in grounds(tree):
        for node in [model] + model.findall('link') + model.findall('.//collision'):
            pose = node.find('pose')
            if pose is not None and (pose.get('relative_to') or
                                     any(float(x) != 0 for x in pose.text.split())):
                raise ValueError('Flat scene ground requires identity poses')
        for mesh in model.findall('.//collision/geometry/mesh'):
            if mesh.findtext('scale', '1 1 1').split() != ['1', '1', '1']:
                raise ValueError('Flat scene ground requires unit mesh scale')
            path = Path(mesh.findtext('uri'))
            hashes[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
            local=[];faces=[]
            for line in path.read_text().splitlines():
                f = line.split()
                if f and f[0] == 'v':
                    point = tuple(float(x) for x in f[1:4])
                    if len(point) != 3 or not np.isfinite(point).all() or abs(point[2]) > 1e-12:
                        raise ValueError('Declared flat scene contains non-planar ground')
                    vertices.append(point)
                    local.append(point)
                elif f and f[0]=='f':
                    if len(f)!=4: raise ValueError('Ground requires triangle faces')
                    try:
                        indices=[int(x.split('/')[0]) for x in f[1:]]
                        indices=[i-1 if i>0 else len(local)+i for i in indices]
                        if any(i<0 for i in indices): raise IndexError()
                        faces.append(indices)
                    except (ValueError,IndexError) as error: raise ValueError('Invalid ground face') from error
            for face in faces:
                try: triangles.append([local[i][:2] for i in face])
                except IndexError as error: raise ValueError('Invalid ground face index') from error
    if not vertices:
        raise ValueError('Flat scene has no ground mesh vertices')
    xs, ys, _ = zip(*vertices)
    lo, hi = min(0, distance) - .5, max(0, distance) + .5
    if min(xs) > lo or max(xs) < hi or min(ys) > -.5 or max(ys) < .5:
        raise ValueError('Flat terrain does not cover the commanded route')
    missing=uncovered_area(triangles,lo,hi)
    if missing>1e-10:
        raise ValueError(f'Flat terrain triangles do not cover route: hole area {missing} m2')
    for model in tree.findall('world/model'):
        if model not in grounds(tree) and model.findtext('static') == 'true':
            for low,high in obstacle_bounds(model):
                if high[0]>=lo and low[0]<=hi and high[1]>=-.5 and low[1]<=.5:
                    raise ValueError('Static obstacle collision envelope lies in the flat route corridor')
    return dict(passed=True,commanded_distance_m=distance,margin_m=.5,
                triangle_count=len(triangles),uncovered_area_m2=missing,mesh_sha256=hashes)


def validate_spec(spec):
    if spec['scene'] == 'flat':
        return validate_flat(ET.parse(spec['world']), spec['linear'] * 20)
    return dict(passed=True, applicable=False)
