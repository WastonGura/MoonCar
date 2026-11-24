"""Resolve real ROS launch actions and constructor gates without starting Gazebo."""
from contextlib import contextmanager
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock,patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions,perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import yaml

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
from simulation_experiment import SimulationExperiment

TERRAIN=ROOT/'ros2_ws/src/mooncar_navigation/launch/terrain_navigation.launch.py'
CONTAINER=ROOT/'containers/experiment.launch.py'
LASER_ENV=('MOONCAR_LASER_ODOMETRY','MOONCAR_LASER_ODOMETRY_MODE')


def module(path):
    spec=importlib.util.spec_from_file_location('defaults_'+path.stem.replace('.','_'),path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result


def text(context,value):
    return perform_substitutions(context,normalize_to_list_of_substitutions(value))


def prepare(description,arguments=None,parent=None):
    context=LaunchContext()
    if parent is not None:
        context.launch_configurations.update(parent.launch_configurations)
    context.launch_configurations.update(arguments or {})
    for action in description.entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    return context


def included(actions,context,filename):
    for action in actions:
        if isinstance(action,IncludeLaunchDescription):
            source=action.launch_description_source
            # location becomes an actual path after the public loader resolves
            # substitutions. Loading descriptions does not execute their nodes.
            source.get_launch_description(context)
            if Path(source.location).name==filename:return action
    raise AssertionError(f'Included launch description not found: {filename}')


def descend(include,parent):
    arguments={name:text(parent,value) for name,value in include.launch_arguments}
    description=include.launch_description_source.get_launch_description(parent)
    return description,prepare(description,arguments,parent=parent)


def executable(node,context):return text(context,node.node_executable)


def parameters(node,context):
    # Jazzy exposes normalized Node parameters only through this storage; use
    # launch_ros' real evaluator for typed bools and substitutions.
    values={}
    for item in evaluate_parameters(context,node._Node__parameters):
        if isinstance(item,dict):values.update(item)
    return values


@contextmanager
def localization_environment(values=None):
    with patch.dict(os.environ):
        for name in LASER_ENV:os.environ.pop(name,None)
        os.environ.update(MOONCAR_EXPERIMENT_ROS_DOMAIN_ID='226',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        os.environ.pop('ROS_STATIC_PEERS',None);os.environ.pop('ROS_LOCALHOST_ONLY',None)
        os.environ.update(values or {})
        yield


class LaunchResolution:
    def terrain(self,description,context):
        actions=[]
        for action in description.entities:
            if isinstance(action,OpaqueFunction):actions.extend(action.execute(context) or [])
            elif not isinstance(action,DeclareLaunchArgument):actions.append(action)
        navigation=included(actions,context,'navigation.launch.py')
        nav_description,nav_context=descend(navigation,context)
        params_path=Path(nav_context.launch_configurations['params_file'])
        self.addCleanup(params_path.unlink,missing_ok=True)
        nav_params=yaml.safe_load(params_path.read_text())
        simulation=included(nav_description.entities,nav_context,'simulation.launch.py')
        sim_description,sim_context=descend(simulation,nav_context)
        relay=next(action for action in sim_description.entities if isinstance(action,Node)
                   and executable(action,sim_context)=='cmd_vel_safety_relay.py')
        terrain=next(action for action in actions if isinstance(action,Node)
                     and executable(action,context)=='terrain_node.py')
        laser=[action for action in actions if isinstance(action,Node)
               and executable(action,context)=='laser_odometry_node.py']
        depth=[action for action in actions if isinstance(action,Node)
               and executable(action,context)=='terrain_odometry_node.py']
        rgbd=[action for action in actions if isinstance(action,Node)
              and executable(action,context) in ('rgbd_odometry_node.py','rgbd_window_odometry.py')]
        depth_map=[action for action in actions if isinstance(action,Node)
                   and executable(action,context)=='depth_localization_map.py']
        static_edges=[]
        for action in actions:
            if isinstance(action,Node) and executable(action,context)=='static_transform_publisher':
                arguments=[text(context,value) for value in action._Node__arguments]
                static_edges.append((arguments[arguments.index('--frame-id')+1],
                                     arguments[arguments.index('--child-frame-id')+1]))
        ekf=next(action for action in nav_description.entities if isinstance(action,Node)
                 and executable(action,nav_context)=='ekf_node')
        mapper=included(nav_description.entities,nav_context,'online_async_launch.py')
        ekf_parameters=yaml.safe_load(Path(nav_context.launch_configurations['ekf_params_file']).read_text())['mooncar_ekf']['ros__parameters']
        return dict(laser=parameters(laser[0],context) if laser else None,
            depth=parameters(depth[0],context) if depth else None,
            rgbd=parameters(rgbd[0],context) if rgbd else None,
            rgbd_executable=executable(rgbd[0],context) if rgbd else None,
            depth_map=parameters(depth_map[0],context) if depth_map else None,
            backend_counts=(len(laser),len(depth),len(rgbd),len(depth_map)),static_edges=static_edges,
            ekf_enabled=ekf.condition.evaluate(nav_context),ekf_parameters=ekf_parameters,
            mapper_enabled=mapper.condition.evaluate(nav_context),
            online=nav_context.launch_configurations['online_mapping']=='true',
            terrain=parameters(terrain,context),relay=parameters(relay,sim_context),
            nav_params=nav_params,wheel_tf=sim_context.launch_configurations['publish_odom_tf'],
            camera_sensor_profile=sim_context.launch_configurations['camera_sensor_profile'])

    def terrain_choice(self,arguments=None):
        description=module(TERRAIN).generate_launch_description()
        context=prepare(description,arguments or {})
        return self.terrain(description,context)

    def container_choice(self,environment=None,arguments=None):
        with localization_environment(environment),tempfile.TemporaryDirectory() as directory:
            description=module(CONTAINER).generate_launch_description()
            supplied=dict(initial_x='0',initial_y='0',initial_yaw='.37',
                output_directory=directory,capture_gui='false')
            supplied.update(arguments or {})
            context=prepare(description,supplied)
            terrain_description,terrain_context=descend(included(description.entities,context,'terrain_navigation.launch.py'),context)
            result=self.terrain(terrain_description,terrain_context)
            recorder_description,recorder_context=descend(included(description.entities,context,'record_map.launch.py'),context)
            recorder=next(action for action in recorder_description.entities if isinstance(action,Node)
                          and executable(action,recorder_context)=='map_recorder.py')
            result['recorder']=parameters(recorder,recorder_context)
            return result

    def assert_choice(self,result,backend):
        laser=backend is not None
        direct=backend=='direct'
        depth=backend in ('terrain','rgbd','rgbd_window')
        rgbd=backend in ('rgbd','rgbd_window')
        self.assertEqual(result['ekf_enabled'],backend not in ('direct','terrain','rgbd','rgbd_window'))
        self.assertEqual(result['wheel_tf'],'false')
        self.assertIs(result['terrain']['require_laser_localization'],laser)
        self.assertIs(result['relay']['require_laser_localization'],laser)
        self.assertIs(result['relay']['require_terrain'],True)
        if 'recorder' in result:self.assertIs(result['recorder']['require_laser_localization'],laser)
        self.assertEqual(result['backend_counts'],(int(laser and not depth),int(depth and not rgbd),int(rgbd),int(depth)))
        self.assertEqual(result['camera_sensor_profile'],'rgbd_v1' if rgbd else 'legacy')
        if rgbd:
            self.assertEqual(result['rgbd']['rgbd_image_reliability'],'reliable')
            self.assertEqual(result['rgbd_executable'],'rgbd_window_odometry.py' if backend=='rgbd_window' else 'rgbd_odometry_node.py')
        self.assertEqual(result['mapper_enabled'],result['online'] and not depth)
        if depth:
            self.assertEqual(result['static_edges'],[('map','survey_odom')])
            self.assertIs(result['rgbd' if rgbd else 'depth']['use_sim_time'],True)
            self.assertIs(result['depth_map']['use_sim_time'],True)
            for costmap in ('local_costmap','global_costmap'):
                plugins=result['nav_params'][costmap][costmap]['ros__parameters']['plugins']
                self.assertNotIn('obstacle_layer',plugins)
                self.assertIn('terrain_obstacles',plugins)
                self.assertIn('terrain_zones',plugins)
                self.assertIn('terrain_traversability',plugins)
        else:
            self.assertEqual(result['static_edges'],[])
        if laser and not depth:
            self.assertIs(result['laser']['direct_mode'],direct)
            self.assertEqual(result['laser']['imu_heading_reference'],'startup' if direct else 'world')
        else:self.assertIsNone(result['laser'])
        if direct:
            mapper=result['nav_params']['slam_toolbox']['ros__parameters']
            self.assertEqual(mapper['scan_topic'],'/mooncar/scan_localized')
            self.assertIs(mapper['use_scan_matching'],False)
            self.assertIs(mapper['do_loop_closing'],False)

class LaunchDefaultTests(LaunchResolution,unittest.TestCase):
    def test_online_default_selects_rgbd_single_tf_and_quality_guards(self):
        self.assert_choice(self.terrain_choice({'online_mapping':'true'}),backend='rgbd_window')

    def test_explicit_terrain_preserves_depth_only_backend_and_legacy_camera(self):
        self.assert_choice(self.terrain_choice({'online_mapping':'true','laser_odometry_mode':'terrain'}),backend='terrain')
        self.assert_choice(self.terrain_choice({'online_mapping':'true','laser_odometry_mode':'terrain',
                                              'laser_odometry':'false'}),backend=None)

    def test_explicit_direct_preserves_laser_backend_and_single_tf(self):
        result=self.terrain_choice({'online_mapping':'true','laser_odometry_mode':'direct'})
        self.assert_choice(result,backend='direct')

    def test_explicit_rgbd_uses_aligned_camera_and_single_tf_with_original_guards(self):
        self.assert_choice(self.terrain_choice({'online_mapping':'true','laser_odometry_mode':'rgbd'}),backend='rgbd')
        self.assert_choice(self.terrain_choice({'online_mapping':'true','laser_odometry_mode':'rgbd',
                                              'laser_odometry':'false'}),backend=None)

    def test_no_argument_offline_keeps_original_ekf(self):
        result=self.terrain_choice()
        self.assert_choice(result,backend=None)
        self.assertNotIn('odom1',result['ekf_parameters'])
        self.assertNotIn('slam_toolbox',result['nav_params'])

    def test_explicit_false_online_restores_baseline_ekf(self):
        result=self.terrain_choice({'online_mapping':'true','laser_odometry':'false'})
        self.assert_choice(result,backend=None)
        self.assertNotIn('odom1',result['ekf_parameters'])
        self.assertEqual(result['nav_params']['slam_toolbox']['ros__parameters']['scan_topic'],'/mooncar/scan_level')

    def test_explicit_true_ekf_preserves_original_laser_candidate(self):
        result=self.terrain_choice({'online_mapping':'true','laser_odometry':'true','laser_odometry_mode':'ekf'})
        self.assert_choice(result,backend='ekf')
        self.assertEqual(result['ekf_parameters']['odom1'],'/mooncar/laser_odometry')
        self.assertEqual(result['nav_params']['slam_toolbox']['ros__parameters']['scan_topic'],'/mooncar/scan_level')

    def test_container_no_environment_uses_rgbd_and_all_launch_guards(self):
        result=self.container_choice()
        self.assert_choice(result,backend='rgbd_window')
        self.assertIs(result['relay']['require_workbench'],True)

    def test_container_environment_overrides_select_fallbacks(self):
        for environment,backend in [({'MOONCAR_LASER_ODOMETRY':'false'},None),
                ({'MOONCAR_LASER_ODOMETRY':'true','MOONCAR_LASER_ODOMETRY_MODE':'terrain'},'terrain'),
                ({'MOONCAR_LASER_ODOMETRY':'true','MOONCAR_LASER_ODOMETRY_MODE':'ekf'},'ekf'),
                ({'MOONCAR_LASER_ODOMETRY':'true','MOONCAR_LASER_ODOMETRY_MODE':'direct'},'direct'),
                ({'MOONCAR_LASER_ODOMETRY':'true','MOONCAR_LASER_ODOMETRY_MODE':'rgbd'},'rgbd')]:
            with self.subTest(environment=environment):
                self.assert_choice(self.container_choice(environment),backend=backend)

    def test_offline_explicit_laser_is_rejected(self):
        for mode in ('terrain','rgbd','rgbd_window','direct','ekf'):
            with self.subTest(mode=mode),self.assertRaisesRegex(ValueError,'online_mapping=true'):
                self.terrain_choice({'laser_odometry':'true','laser_odometry_mode':mode})


class WorkbenchDefaultTests(LaunchResolution,unittest.TestCase):
    def test_constructor_default_guard_and_child_remain_consistent_after_environment_change(self):
        with localization_environment(),tempfile.TemporaryDirectory() as directory:
            experiment=SimulationExperiment(directory)
            try:
                self.assertIs(experiment.laser_odometry,True)
                self.assertEqual(experiment.laser_odometry_mode,'rgbd_window')
                self.assertFalse(experiment.autonomy.localization_revisit_enabled)
                self.assertEqual(experiment.motion.gimbal_command_rate_rad_s,.35)
                self.assertIs(experiment.motion.localization_quality.required,True)
                self.assertIsNotNone(experiment.motion.localization_subscription)
                # A changed caller environment must not silently select a child
                # backend inconsistent with the guard already constructed.
                os.environ.update(MOONCAR_LASER_ODOMETRY='false',MOONCAR_LASER_ODOMETRY_MODE='ekf')
                with patch('simulation_experiment.subprocess.Popen',return_value=Mock()) as spawn:
                    self.assertEqual(experiment.apply(dict(version=1,region=[-4,-4,7,4],start=[0,0,0],
                        scene='visible_ramp_v1',gui=False)),{'accepted':True})
                    experiment.worker.join(timeout=5)
                    self.assertFalse(experiment.worker.is_alive())
                    spawn.assert_called_once()
                    argv=spawn.call_args.args[0]
                    child_arguments=dict(value.split(':=',1) for value in argv if ':=' in value)
                    self.assertEqual(child_arguments['laser_odometry_mode'],'rgbd_window')
                    self.assertIn('多关键帧局部地图与滑动窗口联合定位',child_arguments['source_description'])
                    self.assert_choice(self.container_choice(spawn.call_args.kwargs['env'],child_arguments),backend='rgbd_window')
            finally:
                # Popen was intercepted, so this mock owns no process group.
                experiment.process=None
                experiment.close()

    def test_constructor_explicit_fallbacks_match_motion_guard(self):
        for environment,laser,mode in [({'MOONCAR_LASER_ODOMETRY':'false'},False,'rgbd_window'),
                ({'MOONCAR_LASER_ODOMETRY':'true','MOONCAR_LASER_ODOMETRY_MODE':'terrain'},True,'terrain'),
                ({'MOONCAR_LASER_ODOMETRY':'false','MOONCAR_LASER_ODOMETRY_MODE':'terrain'},False,'terrain'),
                ({'MOONCAR_LASER_ODOMETRY':'false','MOONCAR_LASER_ODOMETRY_MODE':'rgbd'},False,'rgbd'),
                ({'MOONCAR_LASER_ODOMETRY':'true','MOONCAR_LASER_ODOMETRY_MODE':'ekf'},True,'ekf'),
                ({'MOONCAR_LASER_ODOMETRY':'true','MOONCAR_LASER_ODOMETRY_MODE':'direct'},True,'direct'),
                ({'MOONCAR_LASER_ODOMETRY':'true','MOONCAR_LASER_ODOMETRY_MODE':'rgbd'},True,'rgbd')]:
            with self.subTest(environment=environment),localization_environment(environment),tempfile.TemporaryDirectory() as directory:
                experiment=SimulationExperiment(directory)
                try:
                    self.assertIs(experiment.laser_odometry,laser)
                    self.assertEqual(experiment.laser_odometry_mode,mode)
                    self.assertEqual(experiment.motion.gimbal_command_rate_rad_s,.35 if laser and mode=='rgbd' else None)
                    self.assertIs(experiment.motion.localization_quality.required,laser)
                    self.assertEqual(experiment.motion.localization_subscription is not None,laser)
                finally:experiment.close()


if __name__=='__main__':
    print('Real launch resolution and isolated ROS constructor checks; Gazebo launches=0.',flush=True)
    unittest.main()
