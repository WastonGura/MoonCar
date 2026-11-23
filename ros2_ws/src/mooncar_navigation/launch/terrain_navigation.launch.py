import os
import copy
import importlib.util
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def configure_workbench_goal_checkers(params, navigation):
    """Select goal-local accuracy without changing ordinary position goals."""
    controller=params['controller_server']['ros__parameters']
    controller['goal_checker'].update(plugin='nav2_controller::PositionGoalChecker',xy_goal_tolerance=.25)
    controller['goal_checker_plugins']=['goal_checker','revisit_goal_checker']
    controller['revisit_goal_checker']=dict(plugin='nav2_controller::PositionGoalChecker',
        stateful=True,xy_goal_tolerance=.05)
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=os.path.join(
        navigation,'config','behavior_trees','navigate_to_pose_workbench.xml')


def configure_revisit_controller(params):
    """Keep ordinary tracking unchanged and use slower close-range pursuit."""
    controller=params['controller_server']['ros__parameters']
    revisit=copy.deepcopy(controller['FollowPath'])
    revisit.update(desired_linear_vel=.06,lookahead_dist=.15,
        min_lookahead_dist=.10,max_lookahead_dist=.20,
        min_approach_linear_velocity=.02,regulated_linear_scaling_min_speed=.02,
        rotate_to_heading_angular_vel=.35,rotate_to_heading_min_angle=.35)
    controller['RevisitFollowPath']=revisit
    plugins=list(controller['controller_plugins'])
    if 'RevisitFollowPath' not in plugins:plugins.append('RevisitFollowPath')
    controller['controller_plugins']=plugins


def configure_slope_safe_controller(params):
    """Task-scoped slope-safe controller without implicit in-place rotation.

    Ordinary flat-ground FollowPath remains untouched. SlopeFollowPath disables
    use_rotate_to_heading and reversing, and enforces curvature-regulated forward
    pursuit so transits off sloped terrain do not trigger wheel-breakaway spins.
    """
    controller=params['controller_server']['ros__parameters']
    slope_ctrl=copy.deepcopy(controller['FollowPath'])
    slope_ctrl.update(
        desired_linear_vel=.14,
        lookahead_dist=.50,
        min_lookahead_dist=.30,
        max_lookahead_dist=.70,
        use_rotate_to_heading=False,
        allow_reversing=False,
        use_regulated_linear_velocity_scaling=True,
        regulated_linear_scaling_min_radius=.90,
        regulated_linear_scaling_min_speed=.06,
        max_angular_accel=2.0,
    )
    controller['SlopeFollowPath']=slope_ctrl
    plugins=list(controller['controller_plugins'])
    if 'SlopeFollowPath' not in plugins:plugins.append('SlopeFollowPath')
    controller['controller_plugins']=plugins


def _load_nav2_profile():
    spec = importlib.util.spec_from_file_location(
        'mooncar_nav2_profile', Path(__file__).resolve().with_name('nav2_profile.py')
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def setup(context):
    navigation = get_package_share_directory('mooncar_navigation')
    terrain = get_package_share_directory('mooncar_terrain')
    with open(os.path.join(navigation, 'config', 'nav2_params.yaml')) as stream:
        params = yaml.safe_load(stream)
    simulation_profile = LaunchConfiguration('simulation_profile').perform(context)
    _load_nav2_profile().apply_nav2_profile(params, simulation_profile, inplace=True)
    online = LaunchConfiguration('online_mapping').perform(context).lower()=='true'
    laser_setting = LaunchConfiguration('laser_odometry').perform(context).lower()
    if laser_setting not in ('true','false'):
        raise ValueError('laser_odometry must be true or false')
    laser_candidate = laser_setting=='true'
    laser_mode=LaunchConfiguration('laser_odometry_mode').perform(context).lower()
    if laser_mode not in ('ekf','direct','terrain','rgbd','rgbd_window'):raise ValueError('Invalid laser_odometry_mode')
    depth_localization=laser_candidate and laser_mode in ('terrain','rgbd','rgbd_window')
    rgbd_localization=laser_candidate and laser_mode in ('rgbd','rgbd_window')
    direct_laser=laser_candidate and laser_mode in ('direct','terrain','rgbd','rgbd_window')
    if laser_candidate and not online:
        raise ValueError('laser_odometry candidate requires online_mapping=true')
    if online:
        slam=get_package_share_directory('mooncar_slam')
        with open(os.path.join(slam,'config','mapper_params_online_async.yaml')) as stream:
            params.update(yaml.safe_load(stream))
        params['slam_toolbox']['ros__parameters'].update(
            odom_frame='survey_odom', resolution=.10, restamp_tf=True)
        params['slam_toolbox']['ros__parameters']['scan_topic'] = '/mooncar/scan_localized' if direct_laser else '/mooncar/scan_level'
        if direct_laser:
            # The metric backend already localizes against measured geometry.
            # A fresh mapper builds occupancy from those poses; a second coarse
            # grid match must not move both the map and its TF away from them.
            params['slam_toolbox']['ros__parameters'].update(
                use_scan_matching=False, do_loop_closing=False)
        params['planner_server']['ros__parameters']['GridBased'].update(allow_unknown=False,tolerance=.05)
    if LaunchConfiguration('require_workbench').perform(context).lower()=='true':
        # The web UI requests a position, not a terminal heading.
        configure_workbench_goal_checkers(params,navigation)
        configure_revisit_controller(params)
        configure_slope_safe_controller(params)
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        if online and name == 'global_costmap':
            # Keep the 8 m task ROI inside the costmap while lidar rays clear observed space.
            costmap.update(rolling_window=True, width=20, height=20, resolution=.10)
        # Jazzy StaticLayer reads this from the costmap namespace. In a rolling
        # unknown-space map, max merging cannot write lethal 254 over unknown 255.
        costmap['use_maximum'] = not (online and name == 'global_costmap')
        costmap['obstacle_layer']['scan']['expected_update_rate'] = 0.5
        if online:costmap['obstacle_layer']['scan']['inf_is_valid'] = True
        if depth_localization:
            # A tilted horizontal beam can hit the supported ground. Measured
            # depth supplies the terrain obstacle layer and observed map here.
            costmap['plugins'].remove('obstacle_layer')
        costmap['plugins'].insert(-1, 'terrain_obstacles')
        costmap['plugins'].insert(-1, 'terrain_zones')
        costmap['plugins'].insert(-1, 'terrain_traversability')
        costmap['terrain_zones'] = {
            'plugin': 'mooncar_terrain::RiskOverlay', 'map_topic': '/mooncar/terrain/keepout',
            'map_subscribe_transient_local': True,
        }
        costmap['terrain_traversability'] = {
            'plugin': 'mooncar_terrain::RiskOverlay', 'map_topic': '/mooncar/terrain/traversability_map',
            'map_subscribe_transient_local': True,
            'subscribe_to_updates': False,
            'trinary_costmap': False,
            'lethal_cost_threshold': 100,
        }
        costmap['terrain_obstacles'] = {
            'plugin': 'nav2_costmap_2d::ObstacleLayer', 'observation_sources': 'terrain',
            'terrain': {
            'topic': '/mooncar/terrain/obstacles', 'data_type': 'PointCloud2',
            'sensor_frame': 'base_footprint', 'marking': True, 'clearing': False,
            'observation_persistence': 20.0,
            'obstacle_max_range': 6.5, 'max_obstacle_height': 1.0,
            },
        }
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(params, stream)
        params_path = stream.name

    def cleanup(context):
        os.unlink(params_path)
        return []

    nodes = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='mooncar_terrain', executable='terrain_node.py',
             parameters=[{'use_sim_time': True, 'depth_pixel_offset': 0.5,
                          'resolution': .20 if depth_localization else .30,
                          'traversability_resolution': .20,
                          'require_laser_localization': laser_candidate,
                          'frame': 'map' if online else 'odom'}], output='screen'),
        Node(package='mooncar_terrain', executable='keepout_node.py', parameters=[{
            'use_sim_time': True, 'zones_file': os.path.join(terrain, 'config', 'zones.yaml'),
        }], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(navigation, 'launch', 'navigation.launch.py')), launch_arguments={
                'map': LaunchConfiguration('map'), 'params_file': params_path,
                'online_mapping': LaunchConfiguration('online_mapping'),
                'simulation_profile': LaunchConfiguration('simulation_profile'),
                'world': LaunchConfiguration('world'),
                'world_name': LaunchConfiguration('world_name'),
                'gz_args': LaunchConfiguration('gz_args'),
                'use_rviz': LaunchConfiguration('use_rviz'), 'require_terrain': 'true',
                'require_workbench': LaunchConfiguration('require_workbench'),
                'require_laser_localization': 'true' if laser_candidate else 'false',
                'use_ekf': 'false' if direct_laser else 'true',
                'use_slam_mapper': 'false' if depth_localization else 'true',
                'camera_sensor_profile': 'rgbd_v1' if rgbd_localization else 'legacy',
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_z': LaunchConfiguration('initial_z'),
                'initial_yaw': LaunchConfiguration('initial_yaw'),
                'ekf_params_file': os.path.join(get_package_share_directory('mooncar_slam'),'config',
                    'ekf_laser_candidate.yaml' if laser_candidate else 'ekf.yaml'),
            }.items()),
    ]
    if online:
        nodes.insert(1, Node(package='mooncar_terrain', executable='level_scan_filter.py',
                             parameters=[{'use_sim_time': True}], output='screen'))
    if depth_localization:
        executable=('rgbd_window_odometry.py' if laser_mode=='rgbd_window' else
                    'rgbd_odometry_node.py' if rgbd_localization else 'terrain_odometry_node.py')
        nodes.insert(1, Node(package='mooncar_slam',executable=executable,
            **({'name':'mooncar_rgbd_window_odometry'} if laser_mode=='rgbd_window' else {}),
            parameters=[dict(use_sim_time=True,**({'rgbd_image_reliability':'reliable'} if rgbd_localization else {}))],output='screen',
            additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'} if rgbd_localization else {}))
        nodes.insert(1, Node(package='mooncar_slam',executable='depth_localization_map.py',
            parameters=[{'use_sim_time': True}],output='screen'))
        nodes.insert(1, Node(package='tf2_ros',executable='static_transform_publisher',
            arguments=['--frame-id','map','--child-frame-id','survey_odom'],output='screen'))
    elif laser_candidate:
        nodes.insert(1, Node(package='mooncar_slam',executable='laser_odometry_node.py',
            parameters=[{'use_sim_time': True,'direct_mode':direct_laser,
                         'imu_heading_reference':'startup' if direct_laser else 'world',
                         'heading_origin':float(LaunchConfiguration('initial_yaw').perform(context))}],output='screen'))
    return nodes


def generate_launch_description():
    navigation = get_package_share_directory('mooncar_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('laser_odometry_mode', default_value='rgbd_window'),
        DeclareLaunchArgument('map', default_value=os.path.join(navigation, 'maps', 'lunar_yard.yaml')),
        DeclareLaunchArgument('online_mapping', default_value='false'),
        # Online surveys use measured local poses; offline AMCL keeps its EKF.
        DeclareLaunchArgument('laser_odometry', default_value=LaunchConfiguration('online_mapping')),
        DeclareLaunchArgument('simulation_profile',default_value='legacy'),
        DeclareLaunchArgument('world', default_value=os.path.join(get_package_share_directory('mooncar_gazebo'),'worlds','lunar_yard.sdf')),
        DeclareLaunchArgument('world_name', default_value='lunar_yard'),
        DeclareLaunchArgument('gz_args', default_value='-r -v 2 --render-engine ogre'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('require_workbench', default_value='false'),
        DeclareLaunchArgument('initial_x', default_value='0.5'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_z', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        OpaqueFunction(function=setup),
    ])
