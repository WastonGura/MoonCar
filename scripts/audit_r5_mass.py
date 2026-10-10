"""Read-only URDF/SDF mass audit at zero joint positions; no ROS or simulation."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def vector(text, count):
    values = np.array([float(x) for x in text.split()])
    if len(values) != count or not np.isfinite(values).all():
        raise ValueError('Invalid finite vector')
    return values


def transform(xyz=(0., 0., 0.), rpy=(0., 0., 0.)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    t = np.eye(4)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                 [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    t[:3, 3] = xyz
    if not np.isfinite(t).all():
        raise ValueError('Invalid transform')
    return t


def urdf_pose(node):
    if node is None:
        return np.eye(4)
    return transform(vector(node.get('xyz', '0 0 0'), 3), vector(node.get('rpy', '0 0 0'), 3))


def sdf_pose(node):
    if node is None:
        return np.eye(4)
    if node.get('rotation_format', 'euler_rpy') != 'euler_rpy' or node.get('degrees', 'false') != 'false':
        raise ValueError('Unsupported SDF pose format')
    v = vector(node.text or '0 0 0 0 0 0', 6)
    return transform(v[:3], v[3:])


def unique(nodes):
    result = {}
    for n in nodes:
        name = n.get('name')
        if not name or name in result:
            raise ValueError('Missing/duplicate name')
        result[name] = n
    return result


def tensor(node, urdf):
    if node is None:
        raise ValueError('Missing inertia tensor')
    read = node.get if urdf else node.findtext
    values = {k: float(read(k)) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')}
    a = np.array([[values['ixx'], values['ixy'], values['ixz']],
                  [values['ixy'], values['iyy'], values['iyz']],
                  [values['ixz'], values['iyz'], values['izz']]])
    if not np.isfinite(a).all():
        raise ValueError('Nonfinite inertia')
    eig = np.linalg.eigvalsh(a)
    if eig[0] <= 0 or eig[-1] > eig[0] + eig[1] + 1e-10:
        raise ValueError('Nonphysical inertia')
    return a


def part(name, inertial, world, urdf, link_world):
    mnode = inertial.find('mass')
    if mnode is None:
        raise ValueError('Missing mass')
    mass = float(mnode.get('value') if urdf else mnode.text)
    if not np.isfinite(mass) or mass <= 0:
        raise ValueError('Nonpositive/nonfinite mass')
    a = tensor(inertial.find('inertia'), urdf)
    return {'link': name, 'mass_kg': mass, 'com_m': world[:3, 3].tolist(),
            'link_pose_matrix': link_world.tolist(),
            'centroidal_inertia_kg_m2': a.tolist(),
            'inertial_rotation_matrix': world[:3, :3].tolist(),
            'inertia_in_root_kg_m2': (world[:3, :3] @ a @ world[:3, :3].T).tolist()}


def read_urdf(root, frame='base_footprint'):
    if root.tag != 'robot':
        raise ValueError('Expanded URDF required')
    links = unique(root.findall('link'))
    joints = unique(root.findall('joint'))
    parents = {}
    for j in joints.values():
        if j.get('type') not in ('fixed', 'revolute', 'continuous', 'prismatic') or j.find('mimic') is not None:
            raise ValueError('Unsupported zero-position joint')
        a, b = j.find('parent'), j.find('child')
        if a is None or b is None or a.get('link') not in links or b.get('link') not in links:
            raise ValueError('Unknown joint link')
        child = b.get('link')
        if child in parents:
            raise ValueError('Multiple parent joints')
        parents[child] = (a.get('link'), urdf_pose(j.find('origin')))
    roots = set(links) - set(parents)
    if len(roots) != 1:
        raise ValueError('Disconnected/cyclic URDF')
    cache = {next(iter(roots)): np.eye(4)}

    def resolve(name, stack=()):
        if name in cache:
            return cache[name]
        if name in stack or name not in parents:
            raise ValueError('Cyclic/unknown URDF frame')
        parent, local = parents[name]
        cache[name] = resolve(parent, stack + (name,)) @ local
        return cache[name]

    if frame not in links:
        raise ValueError('Requested root missing')
    inverse = np.linalg.inv(resolve(frame))
    parts = []
    massless = []
    for name, link in links.items():
        pose = inverse @ resolve(name)
        i = link.find('inertial')
        if i is None:
            # Only pure kinematic frames are allowed to omit inertial data.
            if link.find('visual') is not None or link.find('collision') is not None:
                raise ValueError('Geometry link missing inertial: ' + name)
            massless.append(name)
            continue
        local = urdf_pose(i.find('origin'))
        p = part(name, i, pose @ local, True, pose)
        p['local_com_m'] = local[:3, 3].tolist()
        p['mount_chain'] = []
        cursor = name
        while cursor in parents:
            parent, mount = parents[cursor]
            p['mount_chain'].append({'child': cursor, 'parent': parent, 'matrix': mount.tolist()})
            cursor = parent
        parts.append(p)
    return parts, massless


def read_sdf(root, frame='base_footprint'):
    model = root.find('model')
    if root.tag != 'sdf' or model is None or len(root.findall('model')) != 1 or model.find('model') is not None:
        raise ValueError('One standalone SDF model required')
    nodes = unique([n for n in model if n.tag in ('link', 'joint', 'frame')])
    cache = {'__model__': np.eye(4), model.get('name'): np.eye(4)}

    def resolve(name, stack=()):
        if name in cache:
            return cache[name]
        if name not in nodes or name in stack:
            raise ValueError('Unknown/cyclic SDF frame')
        n = nodes[name]
        p = n.find('pose')
        default = n.get('attached_to', '__model__') if n.tag == 'frame' else '__model__'
        if n.tag == 'joint':
            default = n.findtext('child')
        parent = (p.get('relative_to') or default) if p is not None else default
        cache[name] = resolve(parent, stack + (name,)) @ sdf_pose(p)
        return cache[name]

    inverse = np.linalg.inv(resolve(frame))
    parts = []
    for n in nodes.values():
        resolve(n.get('name'))
        if n.tag != 'link':
            continue
        i = n.find('inertial')
        if i is None or i.get('auto', 'false') != 'false':
            raise ValueError('Explicit SDF inertial required')
        pose = i.find('pose')
        base = resolve(pose.get('relative_to')) if pose is not None and pose.get('relative_to') else resolve(n.get('name'))
        parts.append(part(n.get('name'), i, inverse @ base @ sdf_pose(pose), False,
                          inverse @ resolve(n.get('name'))))
    return parts


def aggregate(parts):
    if not parts or len({p['link'] for p in parts}) != len(parts):
        raise ValueError('Empty/duplicate mass components')
    m = sum(p['mass_kg'] for p in parts)
    c = sum(p['mass_kg'] * np.array(p['com_m']) for p in parts) / m
    inertia = np.zeros((3, 3))
    for p in parts:
        d = np.array(p['com_m']) - c
        inertia += np.array(p['inertia_in_root_kg_m2']) + p['mass_kg'] * (d @ d * np.eye(3) - np.outer(d, d))
    return {'total_mass_kg': m, 'com_m': c.tolist(), 'inertia_kg_m2': inertia.tolist()}


def contributions(parts, total):
    m, c = total['total_mass_kg'], np.array(total['com_m'])
    for p in parts:
        r = np.array(p['com_m'])
        p['com_contribution_m'] = (p['mass_kg'] * r / m).tolist()
        # Differential sensitivities, not finite interventions or uncertainty bounds.
        p['com_derivative_per_added_g_m'] = ((r-c) / m * .001).tolist()
        p['com_derivative_per_mount_mm_matrix_m'] = (p['mass_kg'] / m * .001 * np.eye(3)).tolist()


CLASSES = {'厂家标称', '实测', '工程假设', '缺失'}


def provenance(parts, catalog):
    if set(catalog['links']) != {p['link'] for p in parts}:
        raise ValueError('Provenance must cover exactly all mass components')
    for p in parts:
        record = catalog['links'][p['link']]
        for field in ('mass', 'local_com', 'mount', 'inertia'):
            item = record[field]
            if item['classification'] not in CLASSES or not item.get('sources'):
                raise ValueError('Unclassified/unsourced parameter')
            if any(s not in catalog['sources'] for s in item['sources']):
                raise ValueError('Unknown provenance source')
        p['provenance'] = record
    entries = catalog['procurement']
    if {e['id'] for e in entries} != {f'{i:02d}' for i in range(1, 21)} or len(entries) != 20:
        raise ValueError('Procurement coverage incomplete/duplicate')
    allowed = {'独立建模', '包含于其他部件', '未建模', '归属待确认'}
    for e in entries:
        if e['model_status'] not in allowed or not e.get('detail'):
            raise ValueError('Procurement classification missing')
        if e.get('nominal_mass_kg') is None:
            e['mass_missing'] = True
    # Formula agreement cannot validate hardware with assumed/missing inputs.
    return all(p['provenance'][f]['classification'] == '实测'
               for p in parts for f in ('mass', 'local_com', 'mount', 'inertia')) and all(
                   e['model_status'] in {'独立建模', '包含于其他部件'} for e in entries)


def compare(a, b):
    delta = {'mass_kg': abs(a['total_mass_kg']-b['total_mass_kg']),
             'com_m': np.abs(np.array(a['com_m'])-b['com_m']).tolist(),
             'inertia_kg_m2': np.abs(np.array(a['inertia_kg_m2'])-b['inertia_kg_m2']).tolist()}
    passed = delta['mass_kg'] <= 1e-6 and max(delta['com_m']) <= 1e-6 and np.max(delta['inertia_kg_m2']) <= 1e-8
    return {'passed': bool(passed), 'absolute_delta': delta,
            'tolerances': {'mass_kg': 1e-6, 'com_each_axis_m': 1e-6, 'inertia_each_entry_kg_m2': 1e-8}}


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def run(urdf, sdf, catalog_path, output):
    inputs = [Path(urdf).resolve(), Path(sdf).resolve(), Path(catalog_path).resolve()]
    output = Path(output).resolve()
    if any(p == output or output in p.parents for p in inputs):
        raise ValueError('Output cannot contain input files')
    output.mkdir(parents=True, exist_ok=False)
    try:
        parts, massless = read_urdf(ET.parse(inputs[0]).getroot())
        sdf_parts = read_sdf(ET.parse(inputs[1]).getroot())
        catalog = json.loads(inputs[2].read_text(encoding='utf-8'))
        for source in catalog['sources'].values():
            snapshot = (inputs[2].parent/source['snapshot']).resolve()
            if inputs[2].parent not in snapshot.parents or hashlib.sha256(snapshot.read_bytes()).hexdigest() != source['sha256']:
                raise ValueError('Invalid source path/hash')
        hardware = provenance(parts, catalog)
        a, b = aggregate(parts), aggregate(sdf_parts)
        contributions(parts, a)
        match = compare(a, b)
        baseline = abs(a['total_mass_kg']-9.186) <= 1e-6 and abs(a['com_m'][0]+.006237889179185711) <= 1e-6
        result = {'passed': bool(match['passed'] and baseline), 'coordinate_frame': 'base_footprint',
                  'joint_positions': 'all zero; wheels zero; gimbal zero',
                  'urdf': a, 'sdf': b, 'crosscheck': match, 'baseline_reproduced': bool(baseline),
                  'hardware_com_confirmed': hardware, 'turn_displacement_repaired': False,
                  'simulation_attempts': 0, 'parts': parts, 'sdf_parts': sdf_parts,
                  'sensitivity_interpretation': 'Differential per +1 g at fixed component centroid; translation per +1 mm along base_footprint axes of this component alone, no child linkage or finite intervention; not uncertainty bounds.',
                  'massless_frames': massless, 'procurement': catalog['procurement'],
                  'conflicts': catalog['conflicts'], 'measurement_priorities': catalog['measurement_priorities'],
                  'input_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
                  'conclusion': '模型自洽，真实质量分布未确认' if match['passed'] and baseline and not hardware else '见分项判定'}
        write_json(output/'audit.json', result)
        with (output/'components.csv').open('w', encoding='utf-8-sig', newline='') as f:
            w = csv.writer(f)
            w.writerow(['link', 'mass_kg', 'com_x_m', 'com_y_m', 'com_z_m', 'x_contribution_mm',
                        'mass_source_class', 'local_com_class', 'mount_class', 'inertia_class',
                        'd_com_x_mm_per_added_g', 'd_com_x_mm_per_mount_x_mm'])
            for p in parts:
                w.writerow([p['link'], p['mass_kg'], *p['com_m'], p['com_contribution_m'][0]*1000,
                            *[p['provenance'][k]['classification'] for k in ('mass', 'local_com', 'mount', 'inertia')],
                            p['com_derivative_per_added_g_m'][0]*1000,
                            p['com_derivative_per_mount_mm_matrix_m'][0][0]*1000])
        return result
    except Exception as e:
        write_json(output/'failure.json', {'passed': False, 'error': str(e), 'simulation_attempts': 0})
        raise


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for flag in ('urdf', 'sdf', 'catalog', 'output'):
        p.add_argument('--'+flag, required=True)
    a = p.parse_args()
    result = run(a.urdf, a.sdf, a.catalog, a.output)
    print(json.dumps({k: result[k] for k in ('passed', 'urdf', 'crosscheck', 'conclusion')}, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
