import os
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


def setup(context):
    navigation = get_package_share_directory('mooncar_navigation')
    terrain = get_package_share_directory('mooncar_terrain')
    with open(os.path.join(navigation, 'config', 'nav2_params.yaml')) as stream:
        params = yaml.safe_load(stream)
    online = LaunchConfiguration('online_mapping').perform(context).lower()=='true'
    if online:
        slam=get_package_share_directory('mooncar_slam')
        with open(os.path.join(slam,'config','mapper_params_online_async.yaml')) as stream:
            params.update(yaml.safe_load(stream))
        params['slam_toolbox']['ros__parameters'].update(
            odom_frame='survey_odom', resolution=.10, restamp_tf=True)
        params['slam_toolbox']['ros__parameters']['scan_topic'] = '/mooncar/scan_level'
        params['planner_server']['ros__parameters']['GridBased'].update(allow_unknown=False,tolerance=.05)
    if LaunchConfiguration('require_workbench').perform(context).lower()=='true':
        # The web UI requests a position, not a terminal heading.
        params['controller_server']['ros__parameters']['goal_checker'].update(
            plugin='nav2_controller::PositionGoalChecker',xy_goal_tolerance=.25)
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
        costmap['plugins'].insert(-1, 'terrain_obstacles')
        costmap['plugins'].insert(-1, 'terrain_zones')
        costmap['plugins'].insert(-1, 'terrain_traversability')
        if online and name == 'global_costmap':
            # In a rolling StaticLayer, unknown keepout cells overwrite free cells;
            # ray clearing must run afterward. Max merging still retains keepout hits.
            costmap['plugins'].remove('terrain_zones')
            costmap['plugins'].insert(1, 'terrain_zones')
            costmap['plugins'].remove('terrain_traversability')
            costmap['plugins'].insert(2, 'terrain_traversability')
        costmap['terrain_zones'] = {
            'plugin': 'nav2_costmap_2d::StaticLayer', 'map_topic': '/mooncar/terrain/keepout',
            'map_subscribe_transient_local': True,
        }
        costmap['terrain_traversability'] = {
            'plugin': 'nav2_costmap_2d::StaticLayer', 'map_topic': '/mooncar/terrain/traversability_map',
            'map_subscribe_transient_local': True,
            'subscribe_to_updates': False,
            'trinary_costmap': False,
            'use_maximum': True,
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
                          'frame': 'map' if online else 'odom'}], output='screen'),
        Node(package='mooncar_terrain', executable='keepout_node.py', parameters=[{
            'use_sim_time': True, 'zones_file': os.path.join(terrain, 'config', 'zones.yaml'),
        }], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(navigation, 'launch', 'navigation.launch.py')), launch_arguments={
                'map': LaunchConfiguration('map'), 'params_file': params_path,
                'online_mapping': LaunchConfiguration('online_mapping'),
                'world': LaunchConfiguration('world'),
                'gz_args': LaunchConfiguration('gz_args'),
                'use_rviz': LaunchConfiguration('use_rviz'), 'require_terrain': 'true',
                'require_workbench': LaunchConfiguration('require_workbench'),
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_yaw': LaunchConfiguration('initial_yaw'),
            }.items()),
    ]
    if online:
        nodes.insert(1, Node(package='mooncar_terrain', executable='level_scan_filter.py',
                             parameters=[{'use_sim_time': True}], output='screen'))
    return nodes


def generate_launch_description():
    navigation = get_package_share_directory('mooncar_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(navigation, 'maps', 'lunar_yard.yaml')),
        DeclareLaunchArgument('online_mapping', default_value='false'),
        DeclareLaunchArgument('world', default_value=os.path.join(get_package_share_directory('mooncar_gazebo'),'worlds','lunar_yard.sdf')),
        DeclareLaunchArgument('gz_args', default_value='-r -v 2 --render-engine ogre'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('require_workbench', default_value='false'),
        DeclareLaunchArgument('initial_x', default_value='0.5'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        OpaqueFunction(function=setup),
    ])
