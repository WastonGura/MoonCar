#!/usr/bin/env python3
"""Collision-only regression of the isolated ODE coplanar contact candidate."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import numpy as np

def load(path):
    with Path(path).open() as stream:
        return list(csv.DictReader(stream))

def contacts(path):
    return [r for r in load(path) if int(r['count'])]

def ordered(rows):
    return sorted([[float(r[k]) for k in ('x','y','z','nx','ny','nz','depth')] for r in rows])

def equal(a,b,tolerance=1e-12):
    a,b=ordered(a),ordered(b)
    return len(a)==len(b) and (not a or np.max(np.abs(np.array(a)-np.array(b)))<=tolerance)

def mesh(path, vertices, faces):
    lines=['v '+' '.join(format(float(x),'.17g') for x in v) for v in vertices]
    for face in faces:
        a,b,c=[np.array(vertices[i-1]) for i in face]
        n=np.cross(b-a,c-a); n/=np.linalg.norm(n)
        lines.append('vn '+' '.join(format(float(x),'.17g') for x in n))
    for index,face in enumerate(faces,1):
        lines.append('f '+' '.join(f'{i}//{index}' for i in face))
    Path(path).write_text('\n'.join(lines)+'\n')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True);p.add_argument('--runtime',required=True)
    p.add_argument('--root-evidence',required=True);p.add_argument('--replay',required=True)
    p.add_argument('--terrain',required=True)
    a=p.parse_args();out=Path(a.output)
    if out.exists(): raise FileExistsError('Refusing to overwrite results')
    out.mkdir(parents=True)
    root=Path(a.root_evidence)
    library=Path(a.runtime)/'libode.so.8.0.2'
    if not library.is_file(): raise FileNotFoundError(library)
    env=dict(os.environ);env['LD_LIBRARY_PATH']=str(library.parent)+':'+env.get('LD_LIBRARY_PATH','')
    def query(name, geometry, poses, patched):
        target=out/(name+('-patched' if patched else '-stock')+'.csv')
        subprocess.run([a.replay,str(geometry),str(poses),str(target)],
                       env=env if patched else os.environ,check=True,timeout=60,
                       stdout=subprocess.DEVNULL)
        return contacts(target)
    one=root/'one-pose.csv'
    original=query('original',root/'actual-two-face-patch.obj',one,False)
    fixed=query('original',root/'actual-two-face-patch.obj',one,True)
    diagonal=query('other-diagonal',root/'actual-patch-other-diagonal.obj',one,True)
    restored=query('restored',root/'actual-two-face-patch.obj',one,False)
    checks={
        'stock_three_fixed_two_restored_three':len(original)==3 and len(fixed)==2 and len(restored)==3,
        'regular_contacts_preserved':equal([r for r in original if float(r['depth'])>0],fixed),
        'triangulation_invariant':equal(fixed,diagonal),
        'restore_unchanged':equal(original,restored),
    }
    vertices=[[-.25,0,0],[0,0,0],[0,.25,0],[-.25,.25,0]]
    faces=[(1,2,3),(1,3,4)]
    crease=[v[:] for v in vertices];crease[3][2]=.02
    mesh(out/'crease.obj',crease,faces)
    mesh(out/'boundary.obj',[[-.25,0,0],[-.23,0,0],[-.23,.25,0],[-.25,.25,0]],faces)
    # A gap crosses the generator between its two ends. Endpoint-only checks
    # would incorrectly bridge this genuine hole.
    mesh(out/'gap.obj',[[-.25,0,0],[-.23,0,0],[-.23,.25,0],[-.25,.25,0],
                        [-.22,0,0],[0,0,0],[0,.25,0],[-.22,.25,0]],
         [(1,2,3),(1,3,4),(5,6,7),(5,7,8)])
    for name in ('crease','boundary','gap'):
        stock=query(name,out/(name+'.obj'),one,False)
        candidate=query(name,out/(name+'.obj'),one,True)
        checks[name+'_retains_stock']=equal(stock,candidate)
    source=load(one)[0];fieldnames=list(source)
    for name,modify in (
        ('separated',lambda r:r.update(z=str(float(r['z'])+.001))),
        ('cap',lambda r:r.update(qw='1',qx='0',qy='0',qz='0',z='.024')),
    ):
        row=dict(source);modify(row)
        pose=out/(name+'-pose.csv')
        with pose.open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=fieldnames);writer.writeheader();writer.writerow(row)
        stock=query(name,root/'actual-two-face-patch.obj',pose,False)
        candidate=query(name,root/'actual-two-face-patch.obj',pose,True)
        checks[name+'_retains_stock']=equal(stock,candidate)
        if name=='separated':checks['separation_not_clamped']=not candidate
    # Rigidly rotate both the terrain and cylinder: the repair is geometric,
    # not hard-coded to Z-up or horizontal terrain.
    angle=.2;rotation=np.array([[1,0,0],[0,np.cos(angle),-np.sin(angle)],[0,np.sin(angle),np.cos(angle)]])
    mesh(out/'rotated.obj',[rotation@v for v in vertices],faces)
    row=dict(source);xyz=rotation@np.array([float(row[k]) for k in ('x','y','z')])
    for k,v in zip(('x','y','z'),xyz):row[k]=format(v,'.17g')
    qw,qx,qy,qz=[float(row[k]) for k in ('qw','qx','qy','qz')]
    c,s=np.cos(angle/2),np.sin(angle/2)
    for k,v in zip(('qw','qx','qy','qz'),(c*qw-s*qx,c*qx+s*qw,c*qy-s*qz,c*qz+s*qy)):row[k]=format(v,'.17g')
    with (out/'rotated-pose.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fieldnames);writer.writeheader();writer.writerow(row)
    candidate=query('rotated',out/'rotated.obj',out/'rotated-pose.csv',True)
    # gz-common's OBJ loader stores vertices as float32; compare to the actual
    # represented plane, rather than an unrounded analytic rotation.
    represented=np.array([rotation@v for v in vertices],dtype=np.float32).astype(float)
    expected_normal=np.cross(represented[1]-represented[0],represented[2]-represented[0])
    expected_normal/=np.linalg.norm(expected_normal)
    checks['rotated_plane_normal']=len(candidate)==2 and all(np.linalg.norm(np.array([float(r[k]) for k in ('nx','ny','nz')])-expected_normal)<1e-12 for r in candidate)
    # All 600 real frozen wheel poses; continuous plate is an independent
    # reference. Evaluate only samples where candidate and mesh differ.
    poses=root/'recorded-wheel-poses.csv'
    query('all-mesh',a.terrain,poses,False);query('all-mesh',a.terrain,poses,True)
    query('all-plate','plate',poses,False);query('all-plate','plate',poses,True)
    def groups(path):
        groups={}
        for r in load(path):
            key=tuple(r[k] for k in ('round','iteration','wheel'))
            groups.setdefault(key,[])
            if int(r['count']):groups[key].append(r)
        return groups
    old=groups(out/'all-mesh-stock.csv');new=groups(out/'all-mesh-patched.csv')
    plate=groups(out/'all-plate-stock.csv');plate_new=groups(out/'all-plate-patched.csv')
    changed=[k for k in old if not equal(old[k],new[k])]
    mismatches=[k for k in changed if not equal(new[k],plate[k],1e-10)]
    checks['plate_unchanged']=all(equal(plate[k],plate_new[k]) for k in plate)
    checks['changed_samples_match_continuous_plate']=not mismatches
    checks['no_new_contact_at_stock_separation']=all(bool(old[k]) or not new[k] for k in old)
    checks={k:bool(v) for k,v in checks.items()}
    report=dict(passed=all(checks.values()),checks=checks,samples=len(old),changed_samples=len(changed),
                changed_not_matching_plate=mismatches,library_sha256=hashlib.sha256(library.read_bytes()).hexdigest())
    (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report));return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
