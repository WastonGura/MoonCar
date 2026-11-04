"""Expand real models and resolve launch/bridge contracts without starting nodes."""
import copy
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
DESCRIPTION = PACKAGE.parent / 'mooncar_description/urdf'
LAUNCH = PACKAGE / 'launch'


def load(name):
    spec = importlib.util.spec_from_file_location('camera_test_' + name, LAUNCH / (name + '.py'))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


camera = load('camera_sensor_profile')


class CameraBridgeTests(unittest.TestCase):
    def test_legacy_returns_original_file_bytes(self):
        for name in ('ros_gz_bridge.yaml', 'ros_gz_bridge_r5.yaml'):
            source = PACKAGE / 'config' / name
            before = source.read_bytes()
            self.assertEqual(camera.prepare_bridge_config(source, 'legacy'), source)
            self.assertEqual(source.read_bytes(), before)

    def test_rgbd_preserves_every_unrelated_bridge_and_shares_camera_info(self):
        for name in ('ros_gz_bridge.yaml', 'ros_gz_bridge_r5.yaml'):
            source = PACKAGE / 'config' / name
            before = source.read_bytes()
            original = yaml.safe_load(before)
            generated = camera.prepare_bridge_config(source, 'rgbd_v1')
            self.addCleanup(generated.unlink, missing_ok=True)
            actual = yaml.safe_load(generated.read_text())
            expected = copy.deepcopy(original)
            for row in expected:
                if row.get('ros_topic_name') == '/mooncar/depth/image':
                    row['gz_topic_name'] = camera.BASE + '/depth_image'
                if row.get('ros_topic_name') == '/mooncar/depth/camera_info':
                    row['gz_topic_name'] = camera.BASE + '/camera_info'
                if row.get('ros_topic_name') == '/mooncar/joint_states_raw':
                    row['ros_topic_name'] = '/joint_states'
            self.assertEqual(actual[:len(original)], expected)
            rows = {row['ros_topic_name']:row for row in actual}
            self.assertEqual(rows['/mooncar/localization_rgb/image']['gz_topic_name'], camera.BASE + '/image')
            self.assertEqual(rows['/mooncar/localization_rgb/image']['ros_type_name'], 'sensor_msgs/msg/Image')
            self.assertEqual(rows['/mooncar/localization_rgb/camera_info']['gz_topic_name'],
                rows['/mooncar/depth/camera_info']['gz_topic_name'])
            self.assertEqual(source.read_bytes(), before)
            self.assertEqual(len(actual), len(original) + 2)
            self.assertEqual(sum(row.get('ros_topic_name') == '/joint_states' for row in actual), 1)
            self.assertFalse(any(row.get('ros_topic_name') == '/mooncar/joint_states_raw' for row in actual))

    def test_rgbd_rejects_duplicate_or_incompatible_joint_publishers(self):
        original = yaml.safe_load((PACKAGE / 'config/ros_gz_bridge.yaml').read_text())
        joint = next(row for row in original if row.get('ros_topic_name') == '/mooncar/joint_states_raw')
        variants = []
        variants.append(original + [copy.deepcopy(joint)])
        direct = copy.deepcopy(joint)
        direct['ros_topic_name'] = '/joint_states'
        variants.append(original + [direct])
        for key, value in (('direction','ROS_TO_GZ'), ('ros_type_name','sensor_msgs/msg/Image'),
                ('gz_type_name','gz.msgs.Image')):
            changed = copy.deepcopy(original)
            next(row for row in changed if row.get('ros_topic_name') == '/mooncar/joint_states_raw')[key] = value
            variants.append(changed)
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'conflict.yaml'
            for rows in variants:
                source.write_text(yaml.safe_dump(rows))
                with self.assertRaises(ValueError):camera.prepare_bridge_config(source, 'rgbd_v1')

    def test_invalid_profile_or_malformed_contract_is_rejected(self):
        with self.assertRaises(ValueError):
            camera.prepare_bridge_config(PACKAGE / 'config/ros_gz_bridge.yaml', 'bad')
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'bad.yaml'
            for rows in ([], {}, [{'ros_topic_name':'/mooncar/depth/image'}]):
                source.write_text(yaml.safe_dump(rows))
                with self.assertRaises(ValueError):camera.prepare_bridge_config(source, 'rgbd_v1')


class CameraModelTests(unittest.TestCase):
    def expand(self, model, profile=None, prefix=None):
        command = ['xacro', str(DESCRIPTION / model), 'use_sim_time:=true']
        if profile is not None:command.append('camera_sensor_profile:=' + profile)
        if prefix is not None:command.append('prefix:=' + prefix)
        return subprocess.check_output(command)

    def test_default_and_explicit_legacy_are_byte_identical_for_both_models(self):
        for model in ('mooncar.urdf.xacro', 'mooncar_r5.urdf.xacro'):
            self.assertEqual(self.expand(model), self.expand(model, 'legacy'))

    def test_rgbd_changes_only_sensor_and_joint_observation_rate_preserving_vehicle(self):
        for model in ('mooncar.urdf.xacro', 'mooncar_r5.urdf.xacro'):
            old = ET.fromstring(self.expand(model))
            new = ET.fromstring(self.expand(model, 'rgbd_v1'))
            original = old.find('.//sensor[@name="mooncar_depth_camera"]')
            changed = new.find('.//sensor[@name="mooncar_depth_camera"]')
            self.assertEqual(changed.get('type'), 'rgbd_camera')
            self.assertEqual(changed.findtext('topic'), camera.BASE)
            self.assertEqual(changed.findtext('gz_frame_id'), 'depth_camera_optical_frame')
            self.assertEqual(changed.findtext('camera/optical_frame_id'), 'depth_camera_optical_frame')
            # A single Gazebo RGBD rendering camera supplies both images. Its
            # image dimensions/FOV/pose/rate/clips remain this model's original.
            changed.set('type', original.get('type'))
            changed.find('topic').text = original.findtext('topic')
            changed.find('camera').remove(changed.find('camera/optical_frame_id'))
            path = './/plugin[@name="gz::sim::systems::JointStatePublisher"]/update_rate'
            self.assertEqual(old.findtext(path), '30')
            self.assertEqual(new.findtext(path), '100')
            new.find(path).text = old.findtext(path)
            self.assertEqual(ET.tostring(new), ET.tostring(old))

    def test_r5_support_joint_observation_rate_is_also_camera_scoped(self):
        for profile, expected in (('legacy', '30'), ('rgbd_v1', '100')):
            robot = ET.fromstring(subprocess.check_output(['xacro', str(DESCRIPTION / 'mooncar_r5.urdf.xacro'),
                'suspension_profile:=independent_linear_v1', 'camera_sensor_profile:=' + profile]))
            plugin = robot.find('.//plugin[@name="gz::sim::systems::JointStatePublisher"]')
            self.assertEqual(plugin.findtext('update_rate'), expected)
            self.assertEqual(len(plugin.findall('joint_name')), 10)

    def test_prefixed_rgbd_optical_frame_and_invalid_profile(self):
        tree = ET.fromstring(self.expand('mooncar.urdf.xacro', 'rgbd_v1', 'test_'))
        self.assertEqual(tree.findtext('.//sensor[@type="rgbd_camera"]/camera/optical_frame_id'),
            'test_depth_camera_optical_frame')
        result = subprocess.run(['xacro', str(DESCRIPTION / 'mooncar.urdf.xacro'),
            'camera_sensor_profile:=bad'], capture_output=True)
        self.assertNotEqual(result.returncode, 0)

    def test_turn_adapter_default_bytes_and_rgbd_flag(self):
        for model in ('mooncar.urdf.xacro', 'mooncar_r5.urdf.xacro'):
            command = [sys.executable, str(LAUNCH / 'turn_model.py'), '--model',
                str(DESCRIPTION / model), '--profile', 'legacy', '--root', '/unused']
            self.assertEqual(subprocess.check_output(command), self.expand(model))
            self.assertEqual(subprocess.check_output(command + ['--camera-sensor-profile', 'rgbd_v1']),
                self.expand(model, 'rgbd_v1'))


class CameraLaunchTests(unittest.TestCase):
    def test_real_launch_resolves_profile_scoped_sim_time_joint_and_tf_rates(self):
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument
        from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
        from launch_ros.actions import Node
        from launch_ros.utilities import evaluate_parameters
        module = load('simulation.launch')
        for simulation in ('legacy', 'r5_support_v1'):
            for camera_profile in ('legacy', 'rgbd_v1'):
                description = module.generate_launch_description()
                context = LaunchContext()
                context.launch_configurations.update(camera_sensor_profile=camera_profile,
                    simulation_profile=simulation,_turn_selected='legacy',_turn_root='/unused')
                for action in description.entities:
                    if isinstance(action, DeclareLaunchArgument):action.execute(context)
                nodes = [a for a in description.entities if isinstance(a, Node)]
                throttle = next(a for a in nodes
                    if a._Node__node_name == 'mooncar_joint_state_throttle')
                command = [perform_substitutions(context, normalize_to_list_of_substitutions(part))
                    for part in throttle._Node__arguments]
                index = command.index('/mooncar/joint_states_raw')
                self.assertEqual(command[index+1], '30.0')
                self.assertEqual(throttle.condition.evaluate(context), camera_profile=='legacy')
                params = evaluate_parameters(context, throttle._Node__parameters)[0]
                self.assertIs(params['use_sim_time'], True)
                self.assertIs(params['use_wall_clock'], False)
                publisher = next(a for a in nodes if 'robot_state_publisher' in str(a.node_executable))
                # Evaluate the real typed rate dictionary without executing the
                # separate robot_description command again.
                params = evaluate_parameters(context, publisher._Node__parameters[1:])[0]
                expected = 100.0 if camera_profile=='rgbd_v1' else (30.0 if simulation=='r5_support_v1' else 20.0)
                self.assertEqual(params['publish_frequency'], expected)
                self.assertIsInstance(params['publish_frequency'], float)

    def test_each_resolved_profile_has_one_joint_state_publisher(self):
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument
        from launch_ros.actions import Node
        module = load('simulation.launch')
        for model, bridge_file in (('mooncar.urdf.xacro','ros_gz_bridge.yaml'),
                ('mooncar_r5.urdf.xacro','ros_gz_bridge_r5.yaml')):
            for profile in ('legacy','rgbd_v1'):
                context = LaunchContext()
                context.launch_configurations.update(camera_sensor_profile=profile, model_file=model)
                description = module.generate_launch_description()
                for action in description.entities:
                    if isinstance(action, DeclareLaunchArgument):action.execute(context)
                throttle = next(action for action in description.entities if isinstance(action,Node)
                    and action._Node__node_name == 'mooncar_joint_state_throttle')
                source = PACKAGE/'config'/bridge_file
                selected = camera.prepare_bridge_config(source,profile)
                if selected != source:self.addCleanup(selected.unlink,missing_ok=True)
                rows = yaml.safe_load(selected.read_text())
                direct = [row for row in rows if row.get('ros_topic_name') == '/joint_states'
                    and row.get('direction') == 'GZ_TO_ROS']
                self.assertEqual(len(direct) + int(throttle.condition.evaluate(context)), 1)
                if profile == 'legacy':
                    self.assertTrue(any(row.get('ros_topic_name') == '/mooncar/joint_states_raw' for row in rows))
                else:
                    self.assertEqual(direct[0]['gz_topic_name'],'joint_states')

    def test_real_launch_resolves_rgbd_description_and_generated_bridge(self):
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
        from launch_ros.actions import Node
        from launch_ros.utilities import evaluate_parameters
        module = load('simulation.launch')
        for profile in ('legacy', 'rgbd_v1'):
            description = module.generate_launch_description()
            context = LaunchContext()
            context.launch_configurations.update(camera_sensor_profile=profile,
                _turn_selected='legacy', _turn_root='/unused')
            for action in description.entities:
                if isinstance(action, DeclareLaunchArgument):action.execute(context)
            declared = next(action for action in description.entities
                if isinstance(action, DeclareLaunchArgument) and action.name == 'camera_sensor_profile')
            self.assertEqual(declared.default_value[0].text, 'legacy')
            publisher = next(action for action in description.entities if isinstance(action, Node)
                and 'robot_state_publisher' in str(action.node_executable))
            values = evaluate_parameters(context, publisher._Node__parameters)
            robot = ET.fromstring(values[0]['robot_description'])
            self.assertEqual(robot.find('.//sensor[@name="mooncar_depth_camera"]').get('type'),
                'rgbd_camera' if profile == 'rgbd_v1' else 'depth')
            with patch.dict(os.environ, {'MOONCAR_IMAGE_DDS_PROFILE':'legacy'}):
                actions = module.checked_image_bridge(context)
            bridge = next(action for action in actions if isinstance(action, IncludeLaunchDescription))
            config = dict(bridge.launch_arguments)['config_file']
            if profile == 'rgbd_v1':
                generated = Path(config)
                self.addCleanup(generated.unlink, missing_ok=True)
                rows = yaml.safe_load(generated.read_text())
                self.assertTrue(any(row.get('gz_topic_name') == camera.BASE + '/image' for row in rows))
                self.assertTrue(any(isinstance(action, RegisterEventHandler) for action in actions))
            else:
                self.assertTrue(str(config).endswith('/config/ros_gz_bridge.yaml'))


if __name__ == '__main__':
    unittest.main()
