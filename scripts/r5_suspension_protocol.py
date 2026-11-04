"""Validate the exact frozen 17-round design, parameters, and flat coverage."""
import hashlib
import os
from pathlib import Path
import xml.etree.ElementTree as ET
from r5_flat_scene import validate_spec
from r5_suspension import WHEELS
from r5_physics_profiles import main_step
from r5_robot_description import validate_bound_description
from r5_monitoring import validate_monitoring


def validate_environment(expected):
    if expected.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':raise ValueError('LOCALHOST discovery required')
    if expected.get('ROS_DOMAIN_ID')!='95' or not expected.get('GZ_PARTITION'):raise ValueError('Isolated execution environment required')
    if any(os.environ.get(key,'')!=value for key,value in expected.items()):raise ValueError('Execution environment differs from frozen preparation')
    if Path('/proc/1/cmdline').exists() and not any(x in Path('/proc/1/cmdline').read_bytes() for x in (b'docker-init',b'tini')):
        raise ValueError('Dedicated container requires --init for orphan reaping')
    return dict(passed=True,environment=expected)


def declared_matrix(physics_profile='legacy_1ms'):
    step=main_step(physics_profile)
    return ([('curve',rigid,sign,step,0) for rigid in (True,False) for sign in (.2,-.2)] +
        [(scene,False,sign,step,0) for scene in ('flat','plane','plane_normal') for sign in (.2,-.2)] +
        [('curve',rigid,.2,dt,0) for rigid in (True,False) for dt in (.0005,.00025)] +
        [('curve',True,sign,step,0) for sign in (.2,-.2)] + [('flat',False,0,step,.1)])


def validate_campaign(manifest):
    validate_monitoring(manifest)
    rounds=manifest['rounds']
    if manifest['maximum_attempts']!=17 or len(rounds)!=17 or len({s['name'] for s in rounds})!=17:
        raise ValueError('Exactly seventeen distinct declared rounds required')
    profile=manifest.get('physics_profile',{'name':'legacy_1ms'})
    if profile.get('main_step_s',main_step(profile['name']))!=main_step(profile['name']):raise ValueError('Wrong profile main step')
    if [(s['scene'],s['rigid'],s['angular'],s['step'],s['linear']) for s in rounds]!=declared_matrix(profile['name']):
        raise ValueError('Frozen matrix order/input differs from declared design')
    for spec in rounds:
        if spec.get('ground_reference_profile')!=('qualified_flat_mesh_v1' if spec['scene']=='flat' else 'legacy_scene_grid'):
            raise ValueError('Wrong declared ground reference profile')
        path=Path(spec['world'])
        if hashlib.sha256(path.read_bytes()).hexdigest()!=spec['world_sha256']:raise ValueError('Wrong world hash')
        tree=ET.parse(path)
        if float(tree.findtext('world/physics/max_step_size'))!=spec['step']:raise ValueError('Wrong physical step')
        validate_bound_description(spec)
        validate_spec(spec)
        model=tree.find("world/model[@name='mooncar']")
        for wheel in WHEELS:
            if model.find(f"joint[@name='{wheel}_wheel_joint']") is None:raise ValueError('Missing drive joint')
            joint=model.find(f"joint[@name='{wheel}_wheel_suspension_joint']")
            if spec['rigid']:
                if joint is not None:raise ValueError('Rigid round includes suspension')
                continue
            if joint is None:raise ValueError('Missing suspension joint')
            parameter=manifest['physical_parameters']['parameters'][wheel]
            for key,field in [('k_N_m','dynamics/spring_stiffness'),('c_N_s_m','dynamics/damping'),
                              ('reference_m','dynamics/spring_reference'),('lower_m','limit/lower'),('upper_m','limit/upper')]:
                if abs(float(joint.findtext('axis/'+field))-parameter[key])>1e-10:raise ValueError('Wrong suspension parameter')
        if not spec['rigid']:
            publisher=model.find("plugin[@name='gz::sim::systems::JointStatePublisher']")
            if publisher is None or len(publisher.findall('joint_name'))!=10:raise ValueError('Missing suspension JointState/TF')
    return dict(passed=True,rounds=17)
