"""Scoped R5/v2 steering repair; old models/worlds retain their original path."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

WORLDS = ('mooncar_ramp_v2', 'mooncar_curve_v2', 'mooncar_survey_v2','mooncar_survey_v3')
LIBRARIES = ('libode.so.8.0.2', 'libmooncar-adaptive-axle-allocation.so', 'libmooncar-turn-slip-control.so')

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def select(model, world, requested):
    if requested not in ('auto', 'legacy', 'r5_v2'):
        raise RuntimeError('Invalid MOONCAR_TURN_PROFILE')
    w = ET.parse(world).find('world')
    compatible = (model == 'mooncar_r5.urdf.xacro' and w is not None and w.get('name') in WORLDS
        and Path(w.findtext("plugin[@name='gz::sim::systems::Physics']/engine/filename", '')).name
        == 'libmooncar-physics7-dartsim.so')
    if requested == 'r5_v2' and not compatible:
        raise RuntimeError('Steering repair requires R5 and a project DART v2 world')
    return ('r5_v2' if compatible else 'legacy') if requested == 'auto' else requested

def verify(root):
    root = Path(root).resolve()
    m = json.loads((root/'manifest.json').read_text())
    if (m.get('ode_version') != '0.16.2' or m.get('gz_sim_major') != 8
            or m.get('gz_physics_version') != '7.8.0'
            or m.get('gimbal_changed') is not True
            or m.get('gimbal_profile') != [20, 0.1, 0, 1, 5]
            or m.get('allocation_profile') != 'independent_laser_feedback_v1'):
        raise RuntimeError('Steering runtime version/profile mismatch')
    if set(m.get('libraries', {})) != set(LIBRARIES):
        raise RuntimeError('Incomplete steering runtime manifest')
    for name in LIBRARIES:
        if sha(root/name) != m['libraries'][name]:
            raise RuntimeError('Steering runtime checksum mismatch: '+name)
    if not (root/'libode.so.8').is_symlink() or (root/'libode.so.8').resolve() != root/'libode.so.8.0.2':
        raise RuntimeError('ODE SONAME must resolve to the verified runtime')
    if sha(m['stock_ode_path']) != m['stock_ode_sha256']:
        raise RuntimeError('Installed ODE changed')
    if sha(m['dart_path']) != m['dart_sha256']:
        raise RuntimeError('Accepted DART runtime changed')
    return m

def augment_urdf(text, root):
    """Append contact/drive systems and use the fixed, previously tested D=0."""
    robot = ET.fromstring(text)
    if robot.get('name') != 'mooncar_r5':
        raise RuntimeError('Wrong vehicle for steering repair')
    if any(p.get('name') in ('mooncar::AxleAllocation', 'mooncar::AdaptiveAxleAllocation', 'mooncar::TurnSlipControl',
            'gz::sim::systems::WheelSlip') for p in robot.findall('gazebo/plugin')):
        raise RuntimeError('Duplicate steering systems')
    controllers=robot.findall("gazebo/plugin[@name='gz::sim::systems::JointPositionController']")
    if len(controllers)!=2:
        raise RuntimeError('Expected both R5 gimbal controllers')
    for controller in controllers:
        expected={'p_gain':20,'i_gain':.1,'d_gain':.5,'i_max':1,'i_min':-1,'cmd_max':5,'cmd_min':-5}
        if any(float(controller.findtext(k,'nan'))!=v for k,v in expected.items()):
            raise RuntimeError('Original R5 gimbal definition changed')
        controller.find('d_gain').text='0'
    extension = ET.SubElement(robot, 'gazebo')
    slip = ET.SubElement(extension, 'plugin', filename='gz-sim-wheel-slip-system', name='gz::sim::systems::WheelSlip')
    for axle, load in (('front', '3.5725146735668791'), ('rear', '3.8681453264331211')):
        for side in ('left', 'right'):
            wheel = ET.SubElement(slip, 'wheel', link_name=f'{axle}_{side}_wheel_link')
            for k, v in (('slip_compliance_lateral', '10'), ('slip_compliance_longitudinal', '0'),
                         ('wheel_normal_force', load), ('wheel_radius', '0.076')):
                ET.SubElement(wheel, k).text = v
    ET.SubElement(extension, 'plugin', filename=str(Path(root)/LIBRARIES[1]), name='mooncar::AdaptiveAxleAllocation')
    p = ET.SubElement(extension, 'plugin', filename=str(Path(root)/LIBRARIES[2]), name='mooncar::TurnSlipControl')
    ET.SubElement(p, 'mode').text = 'pure_turn'
    return ET.tostring(robot, encoding='unicode')
