"""Deterministic engineering detail, separate from measured ground elevations."""
import math
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

SEED = 20261006


def rock_variants():
    phi = (1 + math.sqrt(5)) / 2
    vertices = np.array([(-1,phi,0),(1,phi,0),(-1,-phi,0),(1,-phi,0),
                         (0,-1,phi),(0,1,phi),(0,-1,-phi),(0,1,-phi),
                         (phi,0,-1),(phi,0,1),(-phi,0,-1),(-phi,0,1)], dtype=float)
    vertices /= np.linalg.norm(vertices, axis=1, keepdims=True)
    hull = ConvexHull(vertices)
    edges = sorted({tuple(sorted((int(face[i]),int(face[(i+1)%3]))))
                    for face in hull.simplices for i in range(3)})
    midpoints = np.array([(vertices[a]+vertices[b])/2 for a,b in edges])
    midpoints /= np.linalg.norm(midpoints,axis=1,keepdims=True)
    sphere = np.concatenate((vertices,midpoints))
    variants = []
    for index in range(6):
        rng = np.random.default_rng(SEED + index)
        points = sphere * rng.uniform(.83,1.17,(len(sphere),1))
        # Normalize extents so caller's nominal x/y/z radii keep their meaning.
        points -= (points.max(axis=0)+points.min(axis=0))/2
        points /= np.abs(points).max(axis=0)
        hull = ConvexHull(points)
        faces = []
        for face in sorted(tuple(sorted(map(int,f))) for f in hull.simplices):
            a,b,c = face
            if np.dot(np.cross(points[b]-points[a],points[c]-points[a]),points[list(face)].mean(axis=0)) < 0:
                face = (a,c,b)
            faces.append(face)
        variants.append((points,np.array(faces,dtype=int)))
    return variants


def write_rocks(path, components):
    """Closed convex components, outward flat normals, seams handled per triangle."""
    with Path(path).open('w',encoding='ascii',newline='\n') as stream:
        stream.write('# Generated engineering rock detail; visual = collision\n')
        base = 1
        for vertices,faces in components:
            for face in faces:
                points = vertices[face]
                normal = np.cross(points[1]-points[0],points[2]-points[0])
                normal /= np.linalg.norm(normal)
                # Planar texture projection per face, avoiding spherical UV seams.
                axis = int(np.argmax(np.abs(normal)))
                uv_axes = [i for i in range(3) if i != axis]
                for point in points:
                    stream.write('v '+' '.join(f'{v:.6f}' for v in point)+'\n')
                    stream.write(f'vt {point[uv_axes[0]]:.6f} {point[uv_axes[1]]:.6f}\n')
                    stream.write('vn '+' '.join(f'{v:.6f}' for v in normal)+'\n')
                stream.write(f'f {base}/{base}/{base} {base+1}/{base+1}/{base+1} {base+2}/{base+2}/{base+2}\n')
                base += 3


def sample_ground(xs,ys,z,x,y):
    """Match the actual quad diagonal (SW,SE,NE) / (SW,NE,NW), not bilinear height."""
    col = int(np.clip(np.searchsorted(xs,x,side='right')-1,0,len(xs)-2))
    row = int(np.clip(np.searchsorted(ys,y,side='right')-1,0,len(ys)-2))
    u,v = (x-xs[col])/(xs[col+1]-xs[col]), (y-ys[row])/(ys[row+1]-ys[row])
    sw,se,ne,nw = z[row,col],z[row,col+1],z[row+1,col+1],z[row+1,col]
    return float(sw + u*(se-sw) + v*(ne-se) if v <= u else sw + u*(ne-nw) + v*(nw-sw))


def excluded(scene,x,y,radius=0):
    if math.hypot(x,y) < 1.5+radius:
        return True
    if -radius <= x <= 2.5+radius and abs(y) <= .6+radius:
        return True
    if scene == 'survey':
        for cx,cy,size in ((4.8,3.6,4.8),(6,-5.1,6)):
            if abs(x-cx) <= size/2+.5+radius and abs(y-cy) <= size/2+.5+radius:
                return True
    return False


def pebbles(scene,xs,ys,z,labels,variants,boulders):
    step = float(xs[1]-xs[0])
    cells = [(int(r),int(c)) for r,c in np.argwhere(np.isin(labels,(2,3)))
             if not excluded(scene,(xs[c]+xs[c+1])/2,(ys[r]+ys[r+1])/2,.05)]
    count = min(120,round(len(cells)*step*step*1.5))
    rng = np.random.default_rng(SEED+{'ramp':1000,'curve':2000,'survey':3000}[scene])
    placed, components = [], []
    for attempt in range(max(1,count)*100):
        if len(placed) >= count or not cells:
            break
        row,col = cells[int(rng.integers(len(cells)))]
        x,y = rng.uniform(xs[col],xs[col+1]),rng.uniform(ys[row],ys[row+1])
        diameter = float(rng.uniform(.03,.10)); radius = diameter/2
        if excluded(scene,x,y,radius):
            continue
        # Entire footprint stays in the declared physical material partition.
        footprint = [(x,y),(x-radius,y),(x+radius,y),(x,y-radius),(x,y+radius)]
        if any(px < xs[0] or px > xs[-1] or py < ys[0] or py > ys[-1] for px,py in footprint):
            continue
        if any(labels[int(np.clip(np.searchsorted(ys,py)-1,0,len(ys)-2)),
                      int(np.clip(np.searchsorted(xs,px)-1,0,len(xs)-2))] not in (2,3) for px,py in footprint):
            continue
        if any(math.hypot(x-p['position_m'][0],y-p['position_m'][1]) < radius+p['diameter_m']/2+.03 for p in placed):
            continue
        if any(((x-b['position_m'][0])/(b['radii_m'][0]+radius))**2+
               ((y-b['position_m'][1])/(b['radii_m'][1]+radius))**2 < 1.2 for b in boulders):
            continue
        variant = int(rng.integers(6)); vertices,faces = variants[variant]
        angle = float(rng.uniform(0,2*math.pi))
        rotation = np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        points = vertices.copy()
        points[:,:2] = points[:,:2] @ rotation.T
        # Diameter records a guaranteed bounding circle, including rotated corners.
        points[:,:2] *= radius/np.linalg.norm(points[:,:2],axis=1).max()
        points[:,2] *= diameter*.25
        points[:,:2] += (x,y)
        ground = sample_ground(xs,ys,z,x,y)
        center = ground-diameter*.025  # Partly embedded, top about 0.225 D above center ground.
        points[:,2] += center
        # Some written vertices must lie inside the actual ground, others above it.
        clearance = [p[2]-sample_ground(xs,ys,z,p[0],p[1]) for p in points]
        if min(clearance) >= -.001 or max(clearance) <= .003:
            continue
        components.append((points,faces))
        placed.append(dict(position_m=[float(x),float(y),float(center)],ground_z_m=ground,
                           diameter_m=diameter,variant=variant,yaw_rad=angle,
                           exposed_vertex_range_m=[float(min(clearance)),float(max(clearance))]))
    if len(placed) != count:
        raise RuntimeError(f'{scene}: placed {len(placed)} of {count} pebbles')
    return components,dict(seed=SEED,count=count,density_per_m2=1.5,eligible_cell_area_m2=len(cells)*step*step,
                           maximum_count=120,diameter_range_m=[.03,.10],instances=placed,
                           status='engineered deterministic detail, not measured rock abundance')
