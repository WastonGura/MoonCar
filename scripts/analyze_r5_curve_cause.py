"""Read-only force/COM/slip audit of complete R5 diagnostic recordings."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from r5_slope_slip_audit import contact_multiplicity

WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')


def vector(rows, prefix):
    return np.column_stack([rows[prefix + a] for a in 'xyz'])


def summarize(values):
    a = np.asarray(values)
    return dict(mean=float(np.mean(a)), rms=float(np.sqrt(np.mean(a*a))),
                p95_abs=float(np.quantile(abs(a), .95)), maximum_abs=float(np.max(abs(a))))


def analyze(folder, output):
    folder, output = Path(folder), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    result = json.loads((folder/'result.json').read_text())
    paths = list(folder.glob('*-contact-steps.csv'))
    if len(paths) != 1:
        raise ValueError('Expected one complete contact recording')
    physics = np.genfromtxt(folder/'physics-steps.csv', delimiter=',', names=True)
    contact = np.genfromtxt(paths[0], delimiter=',', names=True)
    if not np.all(np.diff(physics['iteration']) == 1):
        raise ValueError('Incomplete physics grid')
    if np.any(contact['state'] < 0):
        raise ValueError('Unknown contact state')
    n = len(physics)
    index = np.searchsorted(physics['iteration'], contact['iteration'])
    if np.any(index >= n) or not np.array_equal(physics['iteration'][index], contact['iteration']):
        raise ValueError('Contact/physics iteration mismatch')
    if not np.array_equal(np.rint(physics['t'][index]*1e9), contact['t_ns']):
        raise ValueError('Contact/physics time mismatch')
    positive = contact['state'] == 1
    multiplicity=contact_multiplicity(contact['iteration'][positive],
        contact['self_collision'][positive],contact['other_collision'][positive])
    force = vector(contact, 'f1')
    normal = vector(contact, 'n')
    wheel_force = np.zeros((n, 4, 3))
    normal_force = np.zeros_like(wheel_force)
    presence = np.zeros((n, 4), dtype=bool)
    wi = contact['wheel'].astype(int)
    np.add.at(wheel_force, (index[positive], wi[positive]), force[positive])
    normal_magnitude = np.einsum('ij,ij->i', force[positive], normal[positive])
    np.add.at(normal_force, (index[positive], wi[positive]),
              normal[positive]*normal_magnitude[:, None])
    presence[index[positive], wi[positive]] = True
    net = wheel_force.sum(axis=1)
    normals = normal_force.sum(axis=1)
    friction = net - normals
    dt = float(np.median(physics['dt']))
    t = physics['t']
    com = vector(physics, 'com_world_')
    mass = physics['total_mass']
    if not np.isfinite(com).all() or not np.allclose(mass, mass[0], atol=1e-12):
        raise ValueError('Invalid COM/mass observation')
    velocity = np.gradient(com, dt, axis=0)
    backward_velocity = np.diff(com, axis=0)/dt
    discrete_acceleration = np.diff(backward_velocity, axis=0)/dt
    start = result['schedule']['motion']['before']['sim']
    end = result['schedule']['motion']['after']['sim']
    moving = (t >= start-1e-9) & (t <= end+1e-9)
    steady = (t >= start+3) & (t <= end-1)
    parked = (t >= start-3) & (t <= start)
    s, e = np.flatnonzero(moving)[[0, -1]]
    gravity = np.array([0., 0., -1.62])
    import xml.etree.ElementTree as ET
    if (folder/'world.sdf').exists():
        gravity = np.array(list(map(float, ET.parse(folder/'world.sdf').findtext('world/gravity').split())))
    balances = {}
    for lag in (-1, 0, 1):
        # COM is post-step. Check the declared force timestamp association independently.
        ix = np.arange(2, n-2)
        valid = steady[ix]
        residual = mass[ix, None]*discrete_acceleration[ix-2] - (net[ix+lag]+mass[ix, None]*gravity)
        balances[str(lag)] = [summarize(residual[valid, a]) for a in range(3)]
    q = np.column_stack([physics[k] for k in ('qw', 'qx', 'qy', 'qz')])
    w, x, y, z = q.T
    body_y = np.column_stack((2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x)))
    body_x = np.column_stack((1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)))
    body_z = np.cross(body_x, body_y)
    yaw = np.unwrap(np.arctan2(body_x[:, 1], body_x[:, 0]))
    omega = np.gradient(yaw, dt)
    lateral = np.einsum('ij,ij->i', velocity, body_y)
    gravity_lateral = mass*np.einsum('j,ij->i', gravity, body_y)
    contact_lateral = np.einsum('ij,ij->i', net, body_y)
    normal_lateral = np.einsum('ij,ij->i', normals, body_y)
    friction_lateral = contact_lateral-normal_lateral
    com_displacement = com[e]-com[s]
    ref_displacement = vector(physics, '')[e]-vector(physics, '')[s]
    wheels = {}
    for j, wheel in enumerate(WHEELS):
        center = vector(physics, wheel+'_world_')
        center_velocity = np.gradient(center, dt, axis=0)
        direction = vector(physics, wheel+'_fdir1_')
        # fdir1 is projected onto each contact plane by the backend.
        cp_n = normal[positive]
        cp_dir = direction[index[positive]]
        cp_dir = cp_dir - np.einsum('ij,ij->i', cp_dir, cp_n)[:, None]*cp_n
        cp_dir /= np.linalg.norm(cp_dir, axis=1)[:, None]
        chosen = (wi[positive] == j) & steady[index[positive]]
        lateral_force = np.einsum('ij,ij->i', force[positive], cp_dir)
        # Center velocity is only an approximation: exact contact-point velocity
        # also contains orientation-rate effects. Report this limitation explicitly.
        center_lat = np.einsum('ij,ij->i', center_velocity[index[positive]], cp_dir)
        compliance = physics[wheel+'_slip1_cmd'][index[positive]]
        # DART scales per-point slip compliance by the number of points in
        # that collision pair. Wheel-total counts are wrong at material seams.
        relation = center_lat+compliance*multiplicity*lateral_force
        wheels[wheel] = dict(empty_fraction=float((~presence[steady, j]).mean()),
            mean_vertical_force_N=float(wheel_force[steady, j, 2].mean()),
            slip_compliance_m_s_N=summarize(physics[wheel+'_slip1_cmd'][steady]),
            center_lateral_velocity_m_s=summarize(center_lat[chosen]),
            tangential_force_N=summarize(lateral_force[chosen]),
            approximate_slip_law_residual_m_s=summarize(relation[chosen]))
    samples = []
    for relative in (0., 2., 5., 10., 15., 20.):
        i = int(np.argmin(abs(t-(start+relative))))
        samples.append(dict(relative_s=relative, ref_xy_mm=((vector(physics, '')[i]-vector(physics, '')[s])[:2]*1000).tolist(),
            com_xy_mm=((com[i]-com[s])[:2]*1000).tolist(), yaw_change_deg=float(np.degrees(yaw[i]-yaw[s])),
            lateral_velocity_m_s=float(lateral[i]), gravity_body_lateral_N=float(gravity_lateral[i]),
            contact_body_lateral_N=float(contact_lateral[i]), contacting_wheels=int(presence[i].sum())))
    summary = dict(scope='Read-only archived evidence; no new physical validation',
        source_folder=str(folder), gravity_m_s2=gravity.tolist(), mass_kg=float(mass[0]),
        motion_start_s=float(start), motion_end_s=float(end), complete_steps=n,
        reference_displacement_m=ref_displacement.tolist(), com_displacement_m=com_displacement.tolist(),
        gravity_work_J=float(mass[0]*gravity@com_displacement),
        parked_com_excursion_m=float(np.linalg.norm(com[parked]-com[parked][0], axis=1).max()),
        net_force_balance_by_lag_N=balances, wheels=wheels, samples=samples,
        steady=dict(gravity_body_lateral_N=summarize(gravity_lateral[steady]),
            normal_body_lateral_N=summarize(normal_lateral[steady]),
            friction_body_lateral_N=summarize(friction_lateral[steady]),
            contacting_wheels_mean=float(presence[steady].sum(axis=1).mean())),
        limitations=['Correlation is not a single-factor causal intervention',
            'Point slip law includes DART per-collision-pair contact multiplicity',
            'Wheel center velocity is not exact contact point velocity',
            'Force balance assumes no unrecorded external chassis contact',
            'Fixed horizontal-plane reference cannot represent every nonplanar contact'])
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (folder/'physics-steps.csv', paths[0], folder/'result.json', folder/'world.sdf')}
    (output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    (output/'sources.json').write_text(json.dumps(hashes, indent=2)+'\n')
    export = np.column_stack((t[moving], com[moving]-com[s], yaw[moving]-yaw[s], lateral[moving],
                             gravity_lateral[moving], normal_lateral[moving], friction_lateral[moving], presence[moving].sum(axis=1)))
    np.savetxt(output/'motion.csv', export[::20], delimiter=',',
        header='t,com_dx,com_dy,com_dz,yaw_change,lateral_velocity,gravity_lateral,normal_lateral,friction_lateral,contacting_wheels', comments='')
    print(json.dumps({k:summary[k] for k in ('mass_kg', 'com_displacement_m', 'gravity_work_J', 'parked_com_excursion_m', 'net_force_balance_by_lag_N', 'samples')}, indent=2))
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    analyze(args.folder, args.output)
