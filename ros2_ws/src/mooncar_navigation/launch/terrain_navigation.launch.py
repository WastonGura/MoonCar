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
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        # Jazzy StaticLayer reads this from the costmap namespace, not the plugin.
        costmap['use_maximum'] = True
        costmap['obstacle_layer']['scan']['expected_update_rate'] = 0.5
        costmap['plugins'].insert(-1, 'terrain_obstacles')
        costmap['plugins'].insert(-1, 'terrain_zones')
        costmap['terrain_zones'] = {
            'plugin': 'nav2_costmap_2d::StaticLayer', 'map_topic': '/mooncar/terrain/keepout',
            'map_subscribe_transient_local': True,
        }
        costmap['terrain_obstacles'] = {
            'plugin': 'nav2_costmap_2d::ObstacleLayer', 'observation_sources': 'terrain',
            'terrain': {
            'topic': '/mooncar/terrain/obstacles', 'data_type': 'PointCloud2',
            'sensor_frame': 'base_footprint', 'marking': True, 'clearing': False,
            'obstacle_max_range': 6.5, 'max_obstacle_height': 1.0,
            },
        }
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(params, stream)
        params_path = stream.name

    def cleanup(context):
        os.unlink(params_path)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='mooncar_terrain', executable='terrain_node.py',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='mooncar_terrain', executable='keepout_node.py', parameters=[{
            'use_sim_time': True, 'zones_file': os.path.join(terrain, 'config', 'zones.yaml'),
        }], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(navigation, 'launch', 'navigation.launch.py')), launch_arguments={
                'map': LaunchConfiguration('map'), 'params_file': params_path,
                'gz_args': LaunchConfiguration('gz_args'),
                'use_rviz': LaunchConfiguration('use_rviz'), 'require_terrain': 'true',
            }.items()),
    ]


def generate_launch_description():
    navigation = get_package_share_directory('mooncar_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(navigation, 'maps', 'lunar_yard.yaml')),
        DeclareLaunchArgument('gz_args', default_value='-r -v 2 --render-engine ogre'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=setup),
    ])
