"""Derive the opt-in RGBD camera and direct joint bridge.

All unrelated bridge rows are retained. The legacy configuration is returned
without rewriting it. A rendered RGBD profile is an ideal simulation contract,
not an OAK factory calibration or a claim of measured hardware alignment.
"""
import copy
from pathlib import Path
import tempfile

import yaml

PROFILES = ('legacy', 'rgbd_v1')
BASE = '/mooncar/localization_rgbd'


def prepare_bridge_config(source, profile):
    if profile not in PROFILES:
        raise ValueError('Invalid camera_sensor_profile: ' + profile)
    source = Path(source)
    if profile == 'legacy':
        return source
    rows = yaml.safe_load(source.read_text())
    if not isinstance(rows, list) or any(not isinstance(row, dict) for row in rows):
        raise ValueError('Camera profile requires a list of bridge rows')
    selected = {}
    for name in ('/mooncar/depth/image', '/mooncar/depth/camera_info',
            '/mooncar/joint_states_raw'):
        matches = [row for row in rows if row.get('ros_topic_name') == name]
        if len(matches) != 1 or matches[0].get('direction') != 'GZ_TO_ROS':
            raise ValueError('Camera profile requires exactly one GZ_TO_ROS ' + name)
        selected[name] = matches[0]
    if any(row.get('ros_topic_name') in ('/mooncar/localization_rgb/image',
            '/mooncar/localization_rgb/camera_info') for row in rows):
        raise ValueError('Localization RGB topics already configured')
    if any(row.get('ros_topic_name') == '/joint_states' for row in rows):
        raise ValueError('Direct joint state topic already configured')
    depth = selected['/mooncar/depth/image']
    info = selected['/mooncar/depth/camera_info']
    if (depth.get('ros_type_name') != 'sensor_msgs/msg/Image' or
            depth.get('gz_type_name') != 'gz.msgs.Image' or
            info.get('ros_type_name') != 'sensor_msgs/msg/CameraInfo' or
            info.get('gz_type_name') != 'gz.msgs.CameraInfo'):
        raise ValueError('Incompatible depth bridge message types')
    joints = selected['/mooncar/joint_states_raw']
    if (joints.get('ros_type_name') != 'sensor_msgs/msg/JointState' or
            joints.get('gz_type_name') != 'gz.msgs.Model'):
        raise ValueError('Incompatible joint state bridge message types')
    # This profile already samples the upstream Gazebo publisher at 100 Hz.
    # Avoid another ROS-clock-gated throttle between joint exposure and TF/odom.
    # The launch disables that throttle for this same opt-in profile.
    joints['ros_topic_name'] = '/joint_states'
    depth['gz_topic_name'] = BASE + '/depth_image'
    info['gz_topic_name'] = BASE + '/camera_info'
    rgb = copy.deepcopy(depth)
    rgb.update(ros_topic_name='/mooncar/localization_rgb/image', gz_topic_name=BASE + '/image')
    rgb_info = copy.deepcopy(info)
    rgb_info['ros_topic_name'] = '/mooncar/localization_rgb/camera_info'
    rows.extend((rgb, rgb_info))
    with tempfile.NamedTemporaryFile(mode='w', prefix='mooncar-rgbd-bridge-',
            suffix='.yaml', encoding='utf-8', delete=False) as stream:
        yaml.safe_dump(rows, stream, sort_keys=False)
        return Path(stream.name)
