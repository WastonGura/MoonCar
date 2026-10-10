"""Prepare unrun diagnostic worlds; only ground collision representation differs."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from audit_r5_ramp_contact import digest, plate_top, read_obj
import numpy as np


def make_mesh(path, mirrored=False, extent=40., cell=.25, local_top=.05):
    n = round(extent / cell)
    if n < 1 or abs(n * cell - extent) > 1e-12:
        raise ValueError('Grid must exactly tile the plate')
    # Four vertices per quad reproduce the existing terrain generator layout.
    with Path(path).open('x') as stream:
        stream.write('# Engineering diagnostic geometry, not measured terrain\n')
        for j in range(n):
            for i in range(n):
                x, y = -extent / 2 + i * cell, -extent / 2 + j * cell
                for a, b in ((x, y), (x+cell, y), (x+cell, y+cell), (x, y+cell)):
                    stream.write(f'v {a:.17g} {b:.17g} {local_top:.17g}\n')
        for k in range(n*n):
            a, b, c, d = [4*k+i for i in (1, 2, 3, 4)]
            faces = ((a, b, d), (b, c, d)) if mirrored else ((a, b, c), (a, c, d))
            for face in faces:
                stream.write('f '+' '.join(str(v) for v in face)+'\n')
    return dict(vertices=4*n*n, faces=2*n*n, extent_m=extent, cell_m=cell, local_height_m=local_top)


def replace_collision(original, uri):
    result = copy.deepcopy(original)
    geometry = result.find("world/model[@name='plate']/link/collision/geometry")
    geometry.clear()
    mesh = ET.SubElement(geometry, 'mesh')
    ET.SubElement(mesh, 'uri').text = uri
    restored = copy.deepcopy(result)
    old = original.find("world/model[@name='plate']/link/collision/geometry")
    target = restored.find("world/model[@name='plate']/link/collision")
    target.remove(target.find('geometry')); target.insert(0, copy.deepcopy(old))
    reference = copy.deepcopy(original)
    parent = reference.find("world/model[@name='plate']/link/collision")
    parent.remove(parent.find('geometry')); parent.insert(0, copy.deepcopy(old))
    if ET.tostring(restored) != ET.tostring(reference):
        raise ValueError('Unexpected non-ground-collision change')
    return result


def generate(evidence, output):
    evidence, output = Path(evidence), Path(output)
    output.mkdir(exist_ok=False)
    rel = 'campaign/03-pure_turn-flat-1000us-left-1/world.sdf'
    archived = {k.replace('\\', '/'): v for k, v in json.loads((evidence/'archive-manifest.json').read_text())['files'].items()}
    source = evidence/rel
    if digest(source) != archived[rel]['sha256']:
        raise ValueError('Archived template changed')
    original = ET.parse(source).getroot()
    plate = original.find("world/model[@name='plate']")
    if abs(plate_top(plate)) > 1e-12 or plate.findtext('link/collision/geometry/box/size').split() != ['40', '40', '.1']:
        raise ValueError('Unexpected validated plate geometry')
    (output/'plate.sdf').write_bytes(source.read_bytes())
    configurations = {}
    for name, mirrored in (('coplanar_mesh', False), ('mirrored_mesh', True)):
        mesh = output/(name+'.obj'); dimensions = make_mesh(mesh, mirrored)
        v, f = read_obj(mesh)
        normals = np.cross(v[f[:, 1]]-v[f[:, 0]], v[f[:, 2]]-v[f[:, 0]])
        if not np.all(v[:, 2] == .05) or np.any(normals[:, 2] <= 0) or np.any(normals[:, :2]):
            raise ValueError('Diagnostic mesh is not an upward coplanar surface')
        world = replace_collision(original, '/probe/fixtures/'+mesh.name)
        if ET.tostring(world.find("world/model[@name='mooncar']")) != ET.tostring(original.find("world/model[@name='mooncar']")):
            raise ValueError('Vehicle/control/sensor changed')
        ET.ElementTree(world).write(output/(name+'.sdf'))
        configurations[name] = dict(dimensions=dimensions, mesh_sha256=digest(mesh),
            world_sha256=digest(output/(name+'.sdf')), upper_surface_world_z_m=0.)
    proof = dict(template_sha256=digest(source), profiles=configurations, plate_sha256=digest(output/'plate.sdf'),
        vehicle_xml_sha256=hashlib.sha256(ET.tostring(original.find("world/model[@name='mooncar']"))).hexdigest(),
        declared_differences=['Ground box volume versus open coplanar triangle mesh, same upper support surface',
                              'Diagonal orientation within the same quarter-metre grid'],
        unchanged=['vehicle', 'controls', 'sensors', 'friction', 'physics step', 'gravity', 'visual surface'],
        mirrored_is_not_parameter_search=True, executed=False, new_simulations=0,
        scope='Unrun test fixtures only; not a fix or accepted terrain resource',
        runtime_mount_requirement='/evidence is the archived validated runtime read-only; /probe is a new independent run directory')
    (output/'manifest.json').write_text(json.dumps(proof, indent=2)+'\n')
    print(json.dumps(proof, indent=2))
    return proof


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--evidence', required=True); p.add_argument('--output', required=True)
    a = p.parse_args(); generate(a.evidence, a.output)
