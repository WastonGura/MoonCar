"""Resolve the scene arguments through actual ROS launch descriptions; start no nodes."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[4]
try:
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch_ros.actions import Node
except ModuleNotFoundError:
    ROS_LAUNCH_AVAILABLE = False
    LaunchResolution = object
else:
    ROS_LAUNCH_AVAILABLE = True
    # Share the established launch evaluator, including its typed parameter
    # resolution and all four localization quality guard checks.
    spec = importlib.util.spec_from_file_location(
        'localization_default_checks', ROOT / 'ros2_ws/src/mooncar_slam/tests/test_localization_defaults.py')
    checks = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checks)
    LaunchResolution = checks.LaunchResolution


@unittest.skipUnless(ROS_LAUNCH_AVAILABLE, 'ROS launch is available in the ROS container')
class ExperimentLaunchTests(LaunchResolution, unittest.TestCase):
    def check_launch(self, supplied, world_name, height, render_engine, source):
        with checks.localization_environment(), tempfile.TemporaryDirectory() as directory:
            description = checks.module(checks.CONTAINER).generate_launch_description()
            arguments = dict(initial_x='0', initial_y='0', initial_yaw='0',
                             output_directory=directory, capture_gui='false')
            arguments.update(supplied)
            context = checks.prepare(description, arguments)
            terrain_description, terrain_context = checks.descend(
                checks.included(description.entities, context, 'terrain_navigation.launch.py'), context)
            self.assert_choice(self.terrain(terrain_description, terrain_context), laser=True, direct=True)
            actions = []
            for action in terrain_description.entities:
                if isinstance(action, OpaqueFunction):
                    actions.extend(action.execute(terrain_context) or [])
                elif not isinstance(action, DeclareLaunchArgument):
                    actions.append(action)
            nav_description, nav_context = checks.descend(
                checks.included(actions, terrain_context, 'navigation.launch.py'), terrain_context)
            self.addCleanup(Path(nav_context.launch_configurations['params_file']).unlink, missing_ok=True)
            _, simulation_context = checks.descend(
                checks.included(nav_description.entities, nav_context, 'simulation.launch.py'), nav_context)
            self.assertEqual(simulation_context.launch_configurations['world_name'], world_name)
            self.assertEqual(float(simulation_context.launch_configurations['spawn_z']), height)
            self.assertEqual(simulation_context.launch_configurations['gz_args'],
                             '-r -s -v 2 --render-engine ' + render_engine)
            self.assertEqual(simulation_context.launch_configurations['world'], directory + '/survey_yard.sdf')
            recorder_description, recorder_context = checks.descend(
                checks.included(description.entities, context, 'record_map.launch.py'), context)
            recorder = next(action for action in recorder_description.entities
                            if isinstance(action, Node)
                            and checks.executable(action, recorder_context) == 'map_recorder.py')
            parameters = checks.parameters(recorder, recorder_context)
            self.assertEqual(parameters['source'], source)
            self.assertIs(parameters['require_laser_localization'], True)

    def test_v2_arguments_reach_actual_simulator_and_recorder_with_default_direct(self):
        source = 'Gazebo Terrain V2 survey 30×30 米（追加四个定位标柱）；测试来源'
        self.check_launch(dict(world_name='mooncar_survey_v2', initial_z='-.00053',
                               render_engine='ogre2', source_description=source),
                          'mooncar_survey_v2', -.00053, 'ogre2', source)

    def test_old_container_launch_keeps_legacy_world_height_and_renderer(self):
        source = 'Gazebo 50×50 米仿真；激光在线建图定位、深度观测采集；历史深度位姿未做回环修正'
        self.check_launch({}, 'lunar_yard', 0., 'ogre', source)


if __name__ == '__main__':
    unittest.main()
