"""Test-only COM intervention and read-only contact analysis. No ROS control."""
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from r5_csv import read_numeric_csv

from r5_origin_analysis import WHEELS, load_lines, rotation, trajectory, unwrap_yaw


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def pose_transform(element):
    values = list(map(float, element.text.split())) if element is not None and element.text else [0.] * 6
    if len(values) != 6 or not np.isfinite(values).all():
        raise ValueError('Invalid SDF pose')
    x, y, z, roll, pitch, yaw = values
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    transform = np.eye(4)
    transform[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    transform[:3, 3] = [x, y, z]
    return transform


def mass_properties(tree):
    """Resolve SDF frame graph at initial joint positions, then parallel-axis sum."""
    model = tree.getroot().find('model')
    if model is None:
        raise ValueError('Standalone model SDF required')
    nodes = {e.get('name'): e for tag in ('link', 'joint', 'frame') for e in model.findall(tag)}
    cache = {'__model__': np.eye(4), model.get('name'): np.eye(4)}

    def resolve(name, visiting=()):
        if name in cache:
            return cache[name]
        if name not in nodes or name in visiting:
            raise ValueError('Unknown/cyclic SDF frame: ' + str(name))
        node = nodes[name]
        pose = node.find('pose')
        default = node.get('attached_to', '__model__') if node.tag == 'frame' else '__model__'
        if node.tag == 'joint':
            default = node.findtext('child')
        parent = pose.get('relative_to', default) if pose is not None else default
        cache[name] = resolve(parent, visiting + (name,)) @ pose_transform(pose)
        return cache[name]

    parts = []
    for link in model.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            raise ValueError('Missing link inertial')
        mass = float(inertial.findtext('mass'))
        p = inertial.find('pose')
        if p is not None and p.get('relative_to'):
            raise ValueError('Unexpected inertial pose frame')
        transform = resolve(link.get('name')) @ pose_transform(p)
        i = inertial.find('inertia')
        tensor = np.array([[float(i.findtext('ixx')), float(i.findtext('ixy')), float(i.findtext('ixz'))],
                           [float(i.findtext('ixy')), float(i.findtext('iyy')), float(i.findtext('iyz'))],
                           [float(i.findtext('ixz')), float(i.findtext('iyz')), float(i.findtext('izz'))]])
        eig = np.linalg.eigvalsh(tensor)
        if not np.isfinite(tensor).all() or mass <= 0 or eig[0] <= 0 or eig[-1] > eig[0] + eig[1] + 1e-10:
            raise ValueError('Nonphysical mass/inertia')
        orient = transform[:3, :3]
        parts.append((mass, transform[:3, 3], orient @ tensor @ orient.T))
    total = sum(p[0] for p in parts)
    center = sum(m * c for m, c, _ in parts) / total
    tensor = sum(i + m * (np.dot(c-center, c-center)*np.eye(3) - np.outer(c-center, c-center)) for m, c, i in parts)
    return dict(total_mass_kg=total, center_of_mass_m=center.tolist(), inertia_about_total_com_kg_m2=tensor.tolist())


def semantic(element):
    return element.tag, sorted(element.attrib.items()), (element.text or '').strip(), tuple(semantic(e) for e in element)


def verify_single_change(original, mirror):
    old = original.find("./model/link[@name='base_footprint']/inertial/pose")
    new = mirror.find("./model/link[@name='base_footprint']/inertial/pose")
    if old is None or new is None:
        raise ValueError('Merged base inertial pose missing')
    ov, nv = list(map(float, old.text.split())), list(map(float, new.text.split()))
    if ov[1:] != nv[1:] or ov[0] == nv[0]:
        raise ValueError('Intervention must change only inertial X')
    restored = copy.deepcopy(mirror)
    restored.find("./model/link[@name='base_footprint']/inertial/pose").text = old.text
    if semantic(original.getroot()) != semantic(restored.getroot()):
        raise ValueError('Model changed beyond base inertial X')


def generate_models(source, output):
    source, output = Path(source), Path(output)
    output.mkdir()
    original = ET.parse(source)
    before = mass_properties(original)
    mirror = copy.deepcopy(original)
    base = mirror.find("./model/link[@name='base_footprint']/inertial")
    mass = float(base.findtext('mass'))
    values = list(map(float, base.findtext('pose').split()))
    # All base-frame axes are aligned with model axes in the frozen R5 candidate.
    if np.any(np.abs(pose_transform(original.find("./model/link[@name='base_footprint']/pose"))[:3, :3] - np.eye(3)) > 1e-12):
        raise ValueError('Unexpected merged base orientation')
    shift = -2 * before['center_of_mass_m'][0] * before['total_mass_kg'] / mass
    values[0] += shift
    base.find('pose').text = ' '.join(format(v, '.17g') for v in values)
    verify_single_change(original, mirror)
    after = mass_properties(mirror)
    target = np.array(before['center_of_mass_m']); target[0] *= -1
    if not np.allclose(after['center_of_mass_m'], target, atol=1e-12, rtol=0):
        raise ValueError('Whole-model COM was not mirrored')
    (output/'original.sdf').write_bytes(source.read_bytes())
    (output/'restore.sdf').write_bytes(source.read_bytes())
    mirror.write(output/'mirror_x.sdf')
    manifest = dict(passed=True, original=before, mirror_x=after, base_mass_kg=mass,
                    base_inertial_x_delta_m=shift, single_change='base_footprint/inertial/pose X',
                    preserved='Masses, centroidal link tensors/orientations, all other model definitions',
                    interpretation='Engineering hypothetical inertia distribution; not hardware relocation or repair',
                    models={n:sha(output/(n+'.sdf')) for n in ('original', 'mirror_x', 'restore')})
    companion = source.with_suffix('.urdf')
    manifest['tf_urdf'] = {}
    if companion.is_file():
        # ROS state publisher uses unchanged kinematic frames; only Gazebo's
        # standalone SDF receives the explicitly authorized inertial intervention.
        for n in ('original', 'mirror_x', 'restore'):
            (output/(n+'.urdf')).write_bytes(companion.read_bytes())
            manifest['tf_urdf'][n] = sha(companion)
    save(output/'manifest.json', manifest)
    return manifest


def stamp_ns(row):
    stamp = row.get('header', {}).get('stamp', {})
    if 'sec' not in stamp and 'nsec' not in stamp:
        raise ValueError('Source timestamp absent')
    return int(stamp.get('sec', 0))*1_000_000_000 + int(stamp.get('nsec', 0))


def vec(value):
    if value is None:
        raise ValueError('Vector absent, not zero')
    v = np.array([float(value.get(k, 0)) for k in ('x', 'y', 'z')])
    if not np.isfinite(v).all():
        raise ValueError('Nonfinite vector')
    return v


def points_for_wheel(row, wheel):
    """Explicit protobuf zero components are valid; absent point/force is not."""
    contacts = row.get('contact')
    if not contacts:
        raise ValueError('No supporting contact in required analysis window: ' + wheel)
    points = []
    for contact in contacts:
        one = wheel+'_wheel_link' in contact.get('collision1', {}).get('name', '')
        two = wheel+'_wheel_link' in contact.get('collision2', {}).get('name', '')
        if one == two:
            raise ValueError('Ambiguous contact collision ownership')
        arrays = [contact.get(k, []) for k in ('position', 'normal', 'depth', 'wrench')]
        if not arrays[0] or len({len(a) for a in arrays}) != 1:
            raise ValueError('Missing point/normal/depth/wrench association')
        for point, normal, depth, wrench in zip(*arrays):
            f1 = vec(wrench.get('body1Wrench', {}).get('force'))
            f2 = vec(wrench.get('body2Wrench', {}).get('force'))
            n = vec(normal); d = float(depth)
            if abs(np.linalg.norm(n)-1) > 1e-6 or not math.isfinite(d) or d < -1e-9:
                raise ValueError('Invalid contact normal/depth')
            if np.linalg.norm(f1+f2) > 1e-6*max(1., np.linalg.norm(f1)):
                raise ValueError('Contact action/reaction mismatch')
            points.append((vec(point), n, d, f1 if one else f2))
    return points


def diagnose_round(folder, expected, profile, output=None):
    folder = Path(folder)
    result = json.loads((folder/'result.json').read_text())
    output = Path(output) if output is not None else folder/'load-analysis'
    output.mkdir()
    data = read_numeric_csv(folder/'physics-steps.csv', delimiter=',', names=True)
    t = data['t']; ns = np.rint(t*1e9).astype(np.int64)
    start = result['schedule']['parked']['after']['sim']
    end = result['schedule']['motion']['after']['sim']
    last = result['schedule']['stop']['after']['sim']
    xyz = np.column_stack([data[k] for k in ('x', 'y', 'z')])
    r = rotation(np.column_stack([data[k] for k in ('qw', 'qx', 'qy', 'qz')]))
    yaw = unwrap_yaw(data)
    _, _, omega, icr = trajectory(t, xyz, yaw)
    steady = (t >= start+3) & (t <= end-1)
    valid = steady & (abs(omega) > .02)
    local_icr = np.einsum('nji,nj->ni', r[valid, :2, :2], icr[valid]-xyz[valid, :2])
    static = (t >= start-3) & (t < start)
    com = np.column_stack([data['com_world_'+k] for k in ('x', 'y', 'z')])
    local_com = np.einsum('nji,nj->ni', r, com-xyz)
    observed_com = np.mean(local_com[static], axis=0)
    com_error = float(np.linalg.norm(observed_com-np.array(expected['center_of_mass_m'])))
    weight = float(np.mean(data['total_mass'][static]))*1.62
    frame_errors = []
    for row in load_lines(folder/'link-poses.jsonl'):
        poses = row.get('pose', [])
        model = next((p for p in poses if p.get('name') == 'mooncar'), None)
        if model is None:
            continue
        stamp = stamp_ns(model)
        if not round(start*1e9) <= stamp <= round(end*1e9):
            continue
        index = int(np.searchsorted(ns, stamp))
        if index >= len(ns) or ns[index] != stamp:
            raise ValueError('Link pose timestamp has no exact physical step')
        for w in WHEELS:
            local = next((p for p in poses if p.get('name') == 'mooncar::'+w+'_wheel_link'), None)
            if local is None:
                raise ValueError('Required wheel relative pose missing')
            frames = {d['key']:d.get('value', []) for d in local.get('header', {}).get('data', [])}
            if frames.get('frame_id') != ['mooncar']:
                raise ValueError('Relative wheel pose parent frame mismatch')
            direct = np.array([data[w+'_world_'+k][index] for k in ('x', 'y', 'z')])
            frame_errors.append(float(np.linalg.norm(xyz[index]+r[index]@vec(local.get('position'))-direct)))
    if len(frame_errors) < 4 or max(frame_errors) > 1e-7:
        raise ValueError('Independent local/world coordinate audit failed')
    wheel_metrics = {}; all_series = []
    for w in WHEELS:
        p = np.column_stack([data[w+'_world_'+k] for k in ('x', 'y', 'z')])
        pv = np.gradient(p, t, axis=0)
        wr = rotation(np.column_stack([data[w+'_world_'+k] for k in ('qw', 'qx', 'qy', 'qz')]))
        skew = np.einsum('nij,nkj->nik', np.gradient(wr, t, axis=0), wr)
        av = np.column_stack(((skew[:, 2, 1]-skew[:, 1, 2])/2, (skew[:, 0, 2]-skew[:, 2, 0])/2, (skew[:, 1, 0]-skew[:, 0, 1])/2))
        series = []; stamps = []
        for row in load_lines(folder/(w+'-contact.jsonl')):
            stamp = stamp_ns(row)
            if not round((start-3)*1e9) <= stamp <= round(last*1e9):
                continue
            index = int(np.searchsorted(ns, stamp))
            if index >= len(ns) or ns[index] != stamp:
                raise ValueError('Contact timestamp has no exact physical step')
            if stamps and stamp <= stamps[-1]:
                raise ValueError('Duplicate/retrograde contact timestamp')
            stamps.append(stamp)
            points = points_for_wheel(row, w)
            # Average points within this wheel at this instant, then RMS over time.
            slip = np.mean([r[index].T @ (pv[index]+np.cross(av[index], pt-p[index])) for pt, _, _, _ in points], axis=0)
            force = sum((fw for _, _, _, fw in points), np.zeros(3))
            force_body = r[index].T @ force
            series.append((stamp*1e-9, *slip, *force_body, force[2], len(points), max(d for _, _, d, _ in points)))
        series = np.array(series)
        if len(stamps) < 2 or (np.diff(stamps) > 100_000_000).any() or stamps[0] > round((start-3+.1)*1e9) or stamps[-1] < round((last-.1)*1e9):
            raise ValueError('Contact coverage missing')
        sm = (series[:, 0] >= start-3) & (series[:, 0] < start)
        mm = (series[:, 0] >= start+3) & (series[:, 0] <= end-1)
        if not sm.any() or not mm.any():
            raise ValueError('Contact window empty')
        rms = float(np.sqrt(np.mean(series[mm, 2]**2)))
        wheel_metrics[w] = dict(samples=len(stamps), max_gap_s=float(np.max(np.diff(stamps))*1e-9),
                                static_normal_N=float(np.mean(series[sm, 7])),
                                steady_normal_N=float(np.mean(series[mm, 7])), lateral_slip_rms_m_s=rms,
                                aggregation='Point mean per wheel per source timestamp, then time RMS')
        np.savetxt(output/(w+'-contact-series.csv'), series, delimiter=',', comments='',
                   header='t,slip_body_x,slip_body_y,slip_body_z,force_body_x,force_body_y,force_body_z,normal_load_N,points,max_depth_m')
        all_series.append((w, series))
    front = sum(wheel_metrics[w]['static_normal_N'] for w in WHEELS[:2])
    rear = sum(wheel_metrics[w]['static_normal_N'] for w in WHEELS[2:])
    front_slip = float(np.sqrt(np.mean([wheel_metrics[w]['lateral_slip_rms_m_s']**2 for w in WHEELS[:2]])))
    rear_slip = float(np.sqrt(np.mean([wheel_metrics[w]['lateral_slip_rms_m_s']**2 for w in WHEELS[2:]])))
    icr_x = float(np.median(local_icr[:, 0])) if len(local_icr) else None
    angular = result['origin_parameters']['angular']
    expected_axle = .157 if profile == 'mirror_x' else -.157
    prediction = None if angular == 0 else bool(icr_x is not None and abs(icr_x-expected_axle) <= .02 and
        (rear_slip >= .02 and front_slip <= .005 if profile == 'mirror_x' else front_slip >= .02 and rear_slip <= .005))
    park_zero = True
    if angular == 0:
        active = (t >= result['sim_started']) & (t <= last)
        drift = np.linalg.norm(xyz[active, :2]-xyz[active, :2][0], axis=1)
        park_zero = bool(np.max(drift) <= .05 and np.max(abs(yaw[active]-yaw[active][0])) <= .05 and result['max_tilt_rad'] <= .1)
    audit = dict(profile=profile, qualified=bool(com_error <= .0001 and abs(front+rear-weight) <= .05*weight and park_zero),
                 observed_initial_com_m=observed_com.tolist(), com_error_m=com_error,
                 static_front_N=front, static_rear_N=rear, rear_minus_front_N=rear-front, weight_N=weight,
                 static_load_sign_matches_prediction=bool(front-rear >= .1 if profile == 'mirror_x' else rear-front >= .1),
                 icr_body_x_median_m=icr_x, icr_valid_fraction=float(valid.sum()/steady.sum()),
                 front_lateral_rms_m_s=front_slip, rear_lateral_rms_m_s=rear_slip,
                 support_exchange_prediction=prediction, zero_stability_passed=park_zero, wheels=wheel_metrics,
                 steady_window_s=[start+3, end-1], static_window_s=[start-3, start],
                 turn_performance_accepted=False, repair_verified=False)
    audit['local_world_pose_audit'] = dict(samples=len(frame_errors), maximum_error_m=max(frame_errors))
    save(output/'audit.json', audit)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for w, s in all_series:
        axes[0].plot(s[:, 0]-start, s[:, 7], label=w)
        axes[1].plot(s[:, 0]-start, s[:, 2], label=w)
    axes[0].set(ylabel='World vertical contact force (N)', title=profile+' / '+str(angular)+' rad/s')
    axes[1].set(ylabel='Point-mean lateral slip (m/s)', xlabel='Time from motion start (s)')
    axes[0].legend(ncol=2)
    fig.tight_layout(); fig.savefig(output/'loads-and-slip.png', dpi=150); plt.close(fig)
    return audit


def causal_decision(entries):
    if len(entries) != 12 or not all(e.get('passed') for e in entries):
        return dict(cause_confirmed=False, status='incomplete_or_unqualified', repair_verified=False)
    turn_entries = [e for e in entries if e['angular'] != 0]
    confirmed = all(e['audit']['static_load_sign_matches_prediction'] and e['audit']['support_exchange_prediction'] for e in turn_entries)
    return dict(cause_confirmed=bool(confirmed), status='com_influence_confirmed' if confirmed else 'frozen_prediction_not_supported',
                repair_verified=False, turn_performance_accepted=False, production_changed=False,
                interpretation='Test inertial COM intervention affects pivot selection' if confirmed else 'No causal confirmation under the frozen support-exchange prediction',
                limitation='Combined assembly inertia also changes; this does not alone prove a load-only mechanism, a DART defect, or real vehicle behavior')


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True); parser.add_argument('--output', required=True)
    args = parser.parse_args()
    print(json.dumps(generate_models(args.source, args.output), indent=2))
