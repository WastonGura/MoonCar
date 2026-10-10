#!/usr/bin/env python3
"""Read-only drive/contact audit. Emits an UNAPPLIED proposal, never a model.

The WheelSlip proposal changes contact physics and requires explicit user
authorization. Source/model numbers are engineering inputs, not tire calibration.
"""
from copy import deepcopy
import difflib
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from audit_r5_mass import aggregate, read_sdf

WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')
LATERAL_COMPLIANCE = 10.


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def static_loads(mass, com_x, front_x, rear_x, gravity):
    if not all(math.isfinite(x) for x in (mass, com_x, front_x, rear_x, gravity)):
        raise ValueError('Nonfinite mass, geometry or gravity')
    if mass <= 0 or gravity <= 0 or not rear_x < com_x < front_x:
        raise ValueError('Invalid support geometry or load')
    front = mass*gravity*(com_x-rear_x)/(front_x-rear_x)
    rear = mass*gravity-front
    return dict(front=front, rear=rear)


def anchored_radius(half_wheelbase, vx, omega):
    if not all(math.isfinite(v) for v in (half_wheelbase, vx, omega)) or half_wheelbase <= 0 or omega == 0:
        raise ValueError('Finite nonzero turn and positive wheelbase required')
    return math.hypot(half_wheelbase, vx/omega)


def normalized_slip(speed, force, compliance):
    if not all(math.isfinite(v) for v in (speed, force, compliance)) or force <= 0 or compliance < 0:
        raise ValueError('Invalid WheelSlip normalization')
    return abs(speed)/force*compliance


def propose(model, loads, radius):
    """Build a patch in memory only. No SDF or Xacro is saved or applied."""
    if model.find("plugin[@name='gz::sim::systems::WheelSlip']") is not None:
        raise ValueError('An existing slip system cannot be silently replaced')
    original = ET.tostring(model, encoding='unicode')
    clone = deepcopy(model)
    plugin = ET.SubElement(clone, 'plugin', filename='gz-sim-wheel-slip-system', name='gz::sim::systems::WheelSlip')
    values = {}
    for name in WHEELS:
        wheel = ET.SubElement(plugin, 'wheel', link_name=name+'_wheel_link')
        normal = loads['front' if name.startswith('front') else 'rear']/2
        settings = dict(slip_compliance_lateral=LATERAL_COMPLIANCE, slip_compliance_longitudinal=0.,
                        wheel_normal_force=normal, wheel_radius=radius)
        for key, value in settings.items(): ET.SubElement(wheel, key).text = format(value, '.17g')
        values[name] = settings
    ET.indent(plugin, space='  ', level=2)
    plugin.tail = '\n  '
    rendered = ET.tostring(clone, encoding='unicode')
    clone.remove(plugin)
    if ET.tostring(clone, encoding='unicode') != original:
        raise ValueError('Proposal changed original vehicle definitions')
    patch = ''.join(difflib.unified_diff(original.splitlines(True), rendered.splitlines(True),
                                        fromfile='original-test-model.sdf: model element',
                                        tofile='UNAPPLIED-wheel-slip-proposal.sdf: model element'))
    return dict(status='proposal_only_not_applied', physics_change_requires_approval=True,
                mechanism='Speed-dependent lateral force-dependent slip; zero rolling speed restores zero compliance',
                parameters=values, engineering_assumption=True, hardware_calibrated=False,
                changed_vehicle_fields=0, added_systems=1), patch


def model_audit(path, gravity=1.62):
    root = ET.parse(path).getroot(); model = root.find('model')
    if model is None or len(root.findall('model')) != 1:
        raise ValueError('One standalone expanded model required')
    mass_parts = read_sdf(root); combined = aggregate(mass_parts)
    if abs(combined['com_m'][1]) > 1e-6:
        raise ValueError('This proposal requires symmetric left/right mass distribution')
    parts = {p['link']:p for p in mass_parts}
    drives = model.findall("plugin[@name='gz::sim::systems::DiffDrive']")
    if len(drives) != 1: raise ValueError('Ambiguous drive system')
    drive = drives[0]
    for side in ('left', 'right'):
        joints = [n.text for n in drive.findall(side+'_joint')]
        expected = {w+'_wheel_joint' for w in WHEELS if w.endswith(side)}
        if len(joints) != 2 or set(joints) != expected: raise ValueError('Wrong/duplicate drive joint allocation')
    radius = float(drive.findtext('wheel_radius')); track = float(drive.findtext('wheel_separation'))
    if radius <= 0 or track <= 0: raise ValueError('Invalid drive geometry')
    wheels = {}; positions = {}
    for name in WHEELS:
        link = model.find(f"link[@name='{name}_wheel_link']")
        if link is None or len(link.findall('collision')) != 1: raise ValueError('Missing/ambiguous wheel collision')
        col = link.find('collision'); friction = col.find('surface/friction/ode')
        if friction is None: raise ValueError('Missing wheel friction')
        mu = float(friction.findtext('mu')); mu2 = float(friction.findtext('mu2'))
        slip1 = float(friction.findtext('slip1', '0')); slip2 = float(friction.findtext('slip2', '0'))
        if not all(math.isfinite(x) for x in (mu, mu2, slip1, slip2)) or mu <= 0 or mu2 <= 0 or min(slip1,slip2) < 0:
            raise ValueError('Invalid contact parameters')
        if slip1 != 0 or slip2 != 0: raise ValueError('Baseline already has slip compliance; do not substitute this hypothesis')
        if list(map(float, friction.findtext('fdir1', '').split())) != [0.,0.,1.]: raise ValueError('Unknown friction direction')
        collision_pose = list(map(float, col.findtext('pose', '').split()))
        if len(collision_pose) != 6 or max(abs(v) for v in collision_pose[:3]) > 1e-9 or abs(collision_pose[3]-math.pi/2) > 1e-8 or max(abs(v) for v in collision_pose[4:]) > 1e-9:
            raise ValueError('Cannot prove collision-local Z is the lateral joint axis')
        if abs(float(col.findtext('geometry/cylinder/radius'))-radius) > 1e-9: raise ValueError('Drive/collision radius mismatch')
        joint = model.find(f"joint[@name='{name}_wheel_joint']")
        if joint is None or list(map(float,joint.findtext('axis/xyz','').split())) != [0.,1.,0.]: raise ValueError('Unknown spin axis')
        position = np.array(parts[name+'_wheel_link']['link_pose_matrix'])[:3,3]
        positions[name] = position.tolist()
        wheels[name] = dict(mu=mu,mu2=mu2,slip1=slip1,slip2=slip2,explicit_slip1=friction.find('slip1') is not None,
                            first_friction_axis='collision Z = body -Y, parallel to spin axis')
    fx = positions['front_left'][0]; rx = positions['rear_left'][0]
    if abs(fx+rx) > 1e-9 or any(abs(positions[n][0]-(fx if n.startswith('front') else rx)) > 1e-9 for n in WHEELS):
        raise ValueError('Unknown axle geometry')
    if any(abs(abs(positions[n][1])*2-track) > 1e-9 for n in WHEELS): raise ValueError('Track geometry mismatch')
    loads = static_loads(combined['total_mass_kg'], combined['com_m'][0], fx, rx, gravity)
    proposal, patch = propose(model, loads, radius)
    prediction = dict(status='conditional_analytic_prediction_not_a_Gazebo_result',
                      assumptions='Flat symmetric wheel geometry; unsaturated linear slip; equal absolute wheel speeds; negligible other lateral forces',
                      icr_body_x_m=combined['com_m'][0], maximum_center_offset_for_half_turn_m=2*abs(combined['com_m'][0]),
                      nominal_force_ratio=fx/(track/2)/LATERAL_COMPLIANCE,
                      saturation_limit=min(v['mu'] for v in wheels.values()),
                      caution='Predicted residual is nonzero; transient motion, slopes and step consistency still require actual validation')
    return dict(model_sha256=sha(path),combined=combined,wheels=wheels,wheel_positions_m=positions,
                wheel_radius_m=radius,track_m=track,axle_half_spacing_m=fx,static_loads_N=loads,
                constant_zero_vx_rear_anchored_half_turn_displacement_m=2*fx,
                proposal=proposal,proposal_prediction=prediction), patch


def audit_round(folder):
    folder = Path(folder); result = json.loads((folder/'result.json').read_text())
    if not result.get('passed'): raise ValueError('Historical round not qualified')
    start = result['schedule']['motion']['before']['sim']+3
    end = result['schedule']['motion']['after']['sim']-1
    forces = {}
    for wheel in WHEELS:
        path = folder/'load-analysis'/(wheel+'-contact-series.csv')
        data = np.genfromtxt(path, delimiter=',', names=True)
        if len(data) < 2 or not np.isfinite(data['normal_load_N']).all() or (np.diff(data['t']) <= 0).any() or (np.diff(data['t']) > .100001).any():
            raise ValueError('Invalid/missing contact samples')
        d = data[(data['t'] >= start) & (data['t'] <= end)]
        if not len(d) or d['t'][0] > start+.1 or d['t'][-1] < end-.1 or (d['normal_load_N'] <= 0).any():
            raise ValueError('Missing/unphysical contact load; not zero')
        force = float(np.mean(d['force_body_y'])); normal = float(np.mean(d['normal_load_N']))
        forces[wheel] = dict(force_series_sha256=sha(path),mean_lateral_force_N=force,mean_normal_load_N=normal,
                             absolute_force_normal_ratio=abs(force)/normal,
                             lateral_slip_rms_m_s=float(np.sqrt(np.mean(d['slip_body_y']**2))))
    analysis = json.loads((folder/'origin-analysis/analysis.json').read_text())
    return dict(round=str(folder),physical_csv_sha256=sha(folder/'physics-steps.csv'),forces=forces,
                raw_motion=analysis['unwrapped_turn_rad'],reported_maximum_displacement_m=analysis['maximum_displacement_m'],
                maximum_wheel_tracking_error_rad_s=max(w['velocity_tracking_max_rad_s'] for w in analysis['wheels'].values()),
                qualification_result_sha256=sha(folder/'result.json'))


def run(model, rounds, output):
    output = Path(output); output.mkdir()
    before = sha(model)
    audit, patch = model_audit(model)
    audit.update(rounds=[audit_round(p) for p in rounds], simulation_attempts=0, model_changed=False,
                 goal_achieved=False, all_control_only_methods_ruled_out=False)
    if sha(model) != before: raise RuntimeError('Read-only input changed')
    (output/'audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    (output/'UNAPPLIED-wheel-slip.patch').write_text(patch)
    return audit


if __name__ == '__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--model',required=True)
    p.add_argument('--round',action='append',default=[]);p.add_argument('--output',required=True)
    args=p.parse_args();print(json.dumps(run(args.model,args.round,args.output),indent=2))
