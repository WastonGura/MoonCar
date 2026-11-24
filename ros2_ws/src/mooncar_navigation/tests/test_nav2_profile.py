"""Verify profile-aware Nav2 footprint and inflation alignment for legacy vs R5."""
import importlib.util
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import yaml


NAV_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = NAV_ROOT.parent


def load_nav2_profile():
    spec = importlib.util.spec_from_file_location(
        'mooncar_nav2_profile', NAV_ROOT / 'launch' / 'nav2_profile.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Nav2ProfileTest(unittest.TestCase):
    def test_r5_yaml_matches_laser_filter_and_encloses_urdf(self):
        r5_nav = yaml.safe_load((NAV_ROOT / 'config' / 'nav2_params_r5.yaml').read_text(encoding='utf-8'))
        laser = yaml.safe_load((SRC_ROOT / 'mooncar_gazebo' / 'config' / 'laser_filter_r5.yaml').read_text(encoding='utf-8'))
        box = laser['scan_to_scan_filter_chain']['ros__parameters']['filter1']['params']

        # Read R5 URDF wheel geometry directly from xacro properties.
        urdf_tree = ET.parse(SRC_ROOT / 'mooncar_description' / 'urdf' / 'mooncar_r5.urdf.xacro')
        props = {
            elem.attrib['name']: elem.attrib['value']
            for elem in urdf_tree.getroot().iter()
            if elem.tag.endswith('property') and 'name' in elem.attrib and 'value' in elem.attrib
        }
        urdf_half_l = float(props['wheel_x_offset']) + float(props['wheel_radius'])
        urdf_half_w = float(props['wheel_y_offset']) + float(props['wheel_width']) / 2.0

        for name in ('local_costmap', 'global_costmap'):
            params = r5_nav[name][name]['ros__parameters']
            fp = yaml.safe_load(params['footprint'])
            xs = [p[0] for p in fp]
            ys = [p[1] for p in fp]
            self.assertEqual((min(xs), max(xs)), (box['min_x'], box['max_x']))
            self.assertEqual((min(ys), max(ys)), (box['min_y'], box['max_y']))
            self.assertGreater(max(xs), urdf_half_l)
            self.assertGreater(max(ys), urdf_half_w)

            circumscribed = math.hypot(max(xs), max(ys))
            inflation = params['inflation_layer']['inflation_radius']
            self.assertGreater(inflation, circumscribed + 0.08)
            self.assertAlmostEqual(inflation, 0.45)
            self.assertAlmostEqual(params['inflation_layer']['cost_scaling_factor'], 4.0)

        rpp = r5_nav['controller_server']['ros__parameters']['FollowPath']
        self.assertAlmostEqual(rpp['cost_scaling_dist'], 0.45)

    def test_apply_nav2_profile_matches_shipped_yamls(self):
        mod = load_nav2_profile()
        legacy_nav = yaml.safe_load((NAV_ROOT / 'config' / 'nav2_params.yaml').read_text(encoding='utf-8'))
        r5_nav = yaml.safe_load((NAV_ROOT / 'config' / 'nav2_params_r5.yaml').read_text(encoding='utf-8'))

        self.assertEqual(mod.apply_nav2_profile(legacy_nav, 'legacy'), legacy_nav)
        self.assertEqual(mod.apply_nav2_profile(legacy_nav, 'r5_support_v1'), r5_nav)
        with self.assertRaises(ValueError):
            mod.apply_nav2_profile(legacy_nav, 'unknown_profile')

    def test_terrain_navigation_launch_applies_simulation_profile(self):
        try:
            from launch import LaunchContext
            from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
            from launch.utilities import perform_substitutions
            from ament_index_python.packages import get_package_share_directory
            get_package_share_directory('mooncar_navigation')
        except Exception as exc:
            self.skipTest(f'ROS launch environment unavailable: {exc}')

        spec = importlib.util.spec_from_file_location(
            'terrain_navigation_launch_under_test',
            NAV_ROOT / 'launch' / 'terrain_navigation.launch.py',
        )
        launch_mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(launch_mod)

        for profile, expected_fp, expected_infl in (
            ('legacy', '[[-0.43, -0.36], [-0.43, 0.36], [0.43, 0.36], [0.43, -0.36]]', 0.65),
            ('r5_support_v1', '[[-0.25, -0.22], [-0.25, 0.22], [0.25, 0.22], [0.25, -0.22]]', 0.45),
        ):
            description = launch_mod.generate_launch_description()
            context = LaunchContext()
            for action in description.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            context.launch_configurations['simulation_profile'] = profile
            context.launch_configurations['require_workbench'] = 'true'
            opaque = next(a for a in description.entities if isinstance(a, OpaqueFunction))
            actions = opaque.execute(context)
            nav_inc = next(a for a in actions if isinstance(a, IncludeLaunchDescription))
            raw_args = dict(nav_inc.launch_arguments)
            params_val = raw_args['params_file']
            params_path = Path(params_val if isinstance(params_val, str) else params_val.perform(context))
            self.addCleanup(params_path.unlink, missing_ok=True)
            resolved = yaml.safe_load(params_path.read_text(encoding='utf-8'))
            for name in ('local_costmap', 'global_costmap'):
                cm = resolved[name][name]['ros__parameters']
                self.assertEqual(cm['footprint'], expected_fp)
                self.assertAlmostEqual(cm['inflation_layer']['inflation_radius'], expected_infl)
            if profile == 'r5_support_v1':
                controller = resolved['controller_server']['ros__parameters']
                self.assertAlmostEqual(controller['FollowPath']['cost_scaling_dist'], 0.45)
                self.assertAlmostEqual(controller['RevisitFollowPath']['cost_scaling_dist'], 0.45)
                self.assertAlmostEqual(controller['SlopeFollowPath']['cost_scaling_dist'], 0.45)
            controller = resolved['controller_server']['ros__parameters']
            self.assertIn('SlopeFollowPath', controller['controller_plugins'])
            self.assertTrue(controller['FollowPath']['use_rotate_to_heading'])
            self.assertFalse(controller['SlopeFollowPath']['use_rotate_to_heading'])
            self.assertFalse(controller['SlopeFollowPath']['allow_reversing'])

    def test_slope_safe_behavior_tree_disables_spin_and_uses_slope_controller(self):
        bt_path = NAV_ROOT / 'config' / 'behavior_trees' / 'navigate_to_pose_slope_safe.xml'
        self.assertTrue(bt_path.is_file())
        tree = ET.parse(bt_path)
        followers = tree.findall('.//FollowPath')
        self.assertEqual(len(followers), 1)
        self.assertEqual(followers[0].get('controller_id'), 'SlopeFollowPath')
        self.assertEqual(len(tree.findall('.//Spin')), 0)
        self.assertEqual(len(tree.findall('.//BackUp')), 0)

    def test_navigation_launch_default_params_follow_simulation_profile(self):
        try:
            from launch import LaunchContext
            from launch.actions import DeclareLaunchArgument
            from ament_index_python.packages import get_package_share_directory
            get_package_share_directory('mooncar_navigation')
        except Exception as exc:
            self.skipTest(f'ROS launch environment unavailable: {exc}')

        spec = importlib.util.spec_from_file_location(
            'navigation_launch_under_test',
            NAV_ROOT / 'launch' / 'navigation.launch.py',
        )
        launch_mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(launch_mod)

        for profile, expected_name in (
            ('legacy', 'nav2_params.yaml'),
            ('r5_support_v1', 'nav2_params_r5.yaml'),
        ):
            description = launch_mod.generate_launch_description()
            context = LaunchContext()
            context.launch_configurations['map'] = '/tmp/unused.yaml'
            context.launch_configurations['simulation_profile'] = profile
            for action in description.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            self.assertTrue(context.launch_configurations['params_file'].endswith(expected_name))


if __name__ == '__main__':
    unittest.main()
