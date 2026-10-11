"""Offline seam-phase check; no simulation or modification of source evidence."""
import hashlib
import json
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
EVIDENCE = ROOT.parent
RAW = EVIDENCE / 'r5-turn-gate-validation-20261011/campaign/14-pure_turn-ramp-1000us-left'
GEMINI = EVIDENCE / 'r5-v2-offset-cause-refinement-20261011'
MESH = ROOT.parents[3] / 'ros2_ws/src/mooncar_gazebo/models/terrain_v2/ramp/regolith_passable.obj'


def intervals(mask, iterations, times):
    points = np.flatnonzero(mask)
    if not len(points):
        return []
    groups = np.split(points, np.flatnonzero(np.diff(points) != 1) + 1)
    return [{'first_iteration': int(iterations[g[0]]), 'last_iteration': int(iterations[g[-1]]),
             'first_motion_s': float(times[g[0]]), 'last_motion_s': float(times[g[-1]])} for g in groups]


def crossing_windows(p, dx):
    # Nominal lowest cylinder generator for each measured pose, translated in X.
    # A replay prediction, not a simulation of the candidate's changed dynamics.
    center = np.column_stack([p['front_left_world_'+a] for a in 'xyz'])
    center[:, 0] += dx
    axis = np.column_stack([p['front_left_axis_'+a] for a in 'xyz'])
    axis /= np.linalg.norm(axis, axis=1)[:, None]
    vertical = np.array([0., 0., 1.]) - axis*axis[:, 2, None]
    vertical /= np.linalg.norm(vertical, axis=1)[:, None]
    bottom = center - .076*vertical
    ends = [bottom-.025*axis, bottom+.025*axis]
    c0 = ends[0][:, 1] - ends[0][:, 0]
    c1 = ends[1][:, 1] - ends[1][:, 0]
    overlap = (np.minimum(c0,c1) <= .25) & (np.maximum(c0,c1) >= .25)
    fraction = np.divide(.25-c0, c1-c0, out=np.full(len(p),np.nan), where=np.abs(c1-c0)>1e-14)
    point_x = ends[0][:,0]+fraction*(ends[1][:,0]-ends[0][:,0])
    overlap &= (point_x >= -.25) & (point_x <= 0.)
    it = p['iteration'].astype(int)
    return {'shift_x_m': dx,
            'finite_seam_bottom_generator_overlap_windows': intervals(overlap,it,p['t']-5.2),
            'at_iteration_17019': {
                'wheel_center_xyz_m': center[it==17019][0].tolist(),
                'bottom_generator_endpoints_xyz_m': [e[it==17019][0].tolist() for e in ends],
                'minimum_endpoint_perpendicular_distance_m': float(np.minimum(np.abs(c0-.25),np.abs(c1-.25))[it==17019][0]/np.sqrt(2)),
                'finite_segment_overlap': bool(overlap[it==17019][0])}}


def main():
    path = RAW/'physics-steps.csv'
    with path.open() as f:
        header = f.readline().strip().split(',')
    fields = ['iteration','t','x','y','qw','qx','qy','qz']
    fields += ['front_left_world_'+a for a in 'xyz']
    fields += ['front_left_axis_'+a for a in 'xyz']
    p = np.genfromtxt(path, delimiter=',', names=True, usecols=[header.index(f) for f in fields])
    p = p[(p['iteration']>5200)&(p['iteration']<=25200)]
    yaw = np.unwrap(np.arctan2(2*(p['qw']*p['qz']+p['qx']*p['qy']),1-2*(p['qy']**2+p['qz']**2)))
    event = p['iteration']==17019
    # Inspect the actual target seam and confirm the adjacent triangle planes.
    vertices,faces = [],[]
    for line in MESH.read_text().splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0]=='v':
            vertices.append([float(v) for v in parts[1:4]])
        elif parts[0]=='f':
            faces.append([int(v.split('/')[0])-1 for v in parts[1:]])
    vertices = np.array(vertices)
    target_faces = []
    normals = []
    edge_types = Counter()
    for face_idx, face in enumerate(faces):
        tri = vertices[face]
        for a,b in [(0,1),(1,2),(2,0)]:
            v0,v1=tri[a],tri[b]
            delta=v1-v0
            if abs(delta[0])>1e-9 and abs(delta[1])>1e-9:
                edge_types['positive_xy_slope' if delta[0]*delta[1]>0 else 'negative_xy_slope']+=1
            if np.allclose(sorted([v0[:2].tolist(),v1[:2].tolist()]),[[-.25,0.],[0.,.25]],atol=1e-10,rtol=0):
                target_faces.append({'face_index_zero_based':face_idx,'vertices':tri.tolist()})
                n=np.cross(tri[1]-tri[0],tri[2]-tri[0]); n/=np.linalg.norm(n)
                normals.append(n.tolist())
    manifest = json.loads((GEMINI/'manifest.json').read_text(encoding='utf-8-sig'))
    output={'scope':'offline only',
            'input_sha256':{str(path.relative_to(EVIDENCE)):hashlib.sha256(path.read_bytes()).hexdigest(),
                            str(MESH):hashlib.sha256(MESH.read_bytes()).hexdigest()},
            'gemini_manifest_verified':all(hashlib.sha256((GEMINI/k).read_bytes()).hexdigest()==v['sha256'] for k,v in manifest.items()),
            'actual_yaw_at_17019_deg':float(np.degrees(yaw[event][0])),
            'baseline':crossing_windows(p,0.),'shifted_replay':crossing_windows(p,.05),
            'nominal_model_warning':'Replay translates baseline poses only. Earlier changes in contacts can alter yaw, position and subsequent contact forces in a new run.',
            'target_seam_faces':target_faces,'target_seam_face_normals':normals,
            'mesh_diagonal_edge_incidences':dict(edge_types)}
    assert len(target_faces)==2
    (ROOT/'geometry-results.json').write_text(json.dumps(output,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(output,indent=2,allow_nan=False))


if __name__=='__main__':
    main()
