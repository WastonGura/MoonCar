"""Prepare a bounded causal diagnosis; no production files or tuning search."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    old = Path('/oldclosure/prepared-d0/curve-0.001/world.sdf')
    baseline = ET.parse(old)
    grid = np.load('/source/ros2_ws/src/mooncar_gazebo/models/terrain_v2/curve/height_grid.npz')
    x, y, z = (grid[k] for k in ('x', 'y', 'z'))
    ix, iy = int(np.argmin(abs(x))), int(np.argmin(abs(y)))
    gradient = np.array([(z[iy, ix+1]-z[iy, ix-1])/(x[ix+1]-x[ix-1]),
                         (z[iy+1, ix]-z[iy-1, ix])/(y[iy+1]-y[iy-1])])
    normal = np.r_[-gradient, 1.]
    normal /= np.linalg.norm(normal)
    gravity = np.array([0., 0., -1.62])
    normal_g = np.dot(gravity, normal)*normal
    reverse_g = 2*normal_g-gravity
    if not np.isclose(np.linalg.norm(reverse_g), 1.62, atol=1e-14):
        raise ValueError('Gravity reversal changed magnitude')
    rounds = [
        ('01-curve-left', 'original', 'original', 10., True, .2),
        ('02-curve-right', 'original', 'original', 10., True, -.2),
        ('03-plane-left', 'plane', 'original', 10., True, .2),
        ('04-plane-normal-gravity', 'plane', 'normal', 10., True, .2),
        ('05-plane-right', 'plane', 'original', 10., True, -.2),
        ('06-curve-normal-gravity', 'original', 'normal', 10., True, .2),
        ('07-curve-zero-lateral-slip', 'original', 'original', 0., True, .2),
        ('08-curve-native-allocation', 'original', 'original', 10., False, .2),
        ('09-curve-left-restore', 'original', 'original', 10., True, .2),
    ]
    plane_assets = {}
    for uri in baseline.findall('.//collision/geometry/mesh/uri'):
        source = Path(uri.text)
        if source.parent.name == 'curve' and source.name in ('regolith_passable.obj', 'rubble_gravel.obj', 'crater_floor.obj'):
            target = out/'plane-assets'/source.name
            target.parent.mkdir(exist_ok=True)
            lines = []
            for line in source.read_text().splitlines():
                if line.startswith('v '):
                    vx, vy, _ = map(float, line.split()[1:])
                    line = f'v {vx:.6f} {vy:.6f} {gradient[0]*vx+gradient[1]*vy:.12f}'
                elif line.startswith('vn '):
                    line = 'vn '+' '.join(format(v, '.15g') for v in normal)
                lines.append(line)
            target.write_text('\n'.join(lines)+'\n')
            plane_assets[str(source)] = str(target)
    if len(plane_assets) != 3:
        raise ValueError('Ground mesh identity mismatch')
    specifications = []
    for name, shape, g_kind, slip, allocated, angular in rounds:
        root = copy.deepcopy(baseline)
        world = root.find('world')
        g = gravity if g_kind == 'original' else normal_g
        world.find('gravity').text = ' '.join(format(v, '.17g') for v in g)
        if shape == 'plane':
            for uri in root.findall('.//uri'):
                if uri.text in plane_assets:
                    uri.text = plane_assets[uri.text]
        model = root.find("world/model[@name='mooncar']")
        for wheel in model.findall("plugin[@name='gz::sim::systems::WheelSlip']/wheel"):
            wheel.find('slip_compliance_lateral').text = str(slip)
        model.find("plugin[@name='mooncar::AxleAllocation']/enabled").text = 'true' if allocated else 'false'
        folder = out/name/'curve-0.001'
        folder.mkdir(parents=True)
        world_path = folder/'world.sdf'
        root.write(world_path)
        spec = dict(name=name, shape=shape, gravity_kind=g_kind, gravity=g.tolist(),
                    slip_compliance_lateral=slip, allocation_enabled=allocated, angular=angular,
                    world=str(world_path), world_sha256=sha(world_path))
        specifications.append(spec)
    fixture = out/'round_fixture.py'
    text = Path('/oldclosure/prepared-d0/round_fixture.py').read_text()
    text = text.replace('/closure/prepared-d0/formal.sdf', '/oldclosure/prepared-d0/formal.sdf')
    fixture.write_text(text)
    manifest = dict(rounds=specifications, maximum_attempts=len(rounds),
        original_world_sha256=sha(old), fixture=str(fixture), local_gradient=gradient.tolist(),
        normal=normal.tolist(), original_gravity=gravity.tolist(), normal_gravity=normal_g.tolist(),
        reversed_tangent_gravity_not_executed=reverse_g.tolist(),
        plane_assets={k:dict(path=v, sha256=sha(v)) for k,v in plane_assets.items()},
        predictions={
            'gravity': 'Uniform slope + lateral compliance can translate during turns; removing tangent gravity should strongly reduce translation on that plane',
            'support': 'Original nonplanar terrain may retain a pivot offset after removing gravity tangent to the reference plane',
            'allocation': 'A bounded fixed ratio is not presumed to be the dominant cause',
            'zero_slip': 'Suppressing lateral compliance is a mechanism contrast, not a production repair'},
        limitations=['Reference plane is a central-grid estimate, not every local wheel support plane',
                     'Changing gravity may change settled support; parked state must be reported',
                     'Plane replacement also changes wheel support geometry',
                     'No hardware or whole-vehicle acceptance conclusion'],
        original_evidence_not_counted_as_new=True, production_changed=False)
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(dict(prepared=len(rounds), gradient=gradient.tolist(), normal_gravity=normal_g.tolist())))


if __name__ == '__main__':
    main()
