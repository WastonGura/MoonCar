"""Opt-in recorder; run alongside mapping or terrain_navigation, not a motor bringup."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('source', default_value='Gazebo depth observations; simulation, not physical measurements'),
        DeclareLaunchArgument('frame', default_value='map'),
        DeclareLaunchArgument('require_laser_localization', default_value='false'),
        DeclareLaunchArgument('uplink_url', default_value=''),
        DeclareLaunchArgument('uplink_token_file', default_value=''),
        DeclareLaunchArgument('uplink_task_file', default_value=''),
        DeclareLaunchArgument('output_directory', default_value=str(Path.home() / '.local/share/mooncar/maps')),
        Node(package='mooncar_terrain', executable='map_recorder.py', output='screen',
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                          'source': ParameterValue(LaunchConfiguration('source'), value_type=str),
                          'frame': LaunchConfiguration('frame'),
                          'require_laser_localization': ParameterValue(
                              LaunchConfiguration('require_laser_localization'), value_type=bool),
                          'output_directory': LaunchConfiguration('output_directory'),
                          'uplink_url': ParameterValue(LaunchConfiguration('uplink_url'), value_type=str),
                          'uplink_token_file': ParameterValue(LaunchConfiguration('uplink_token_file'), value_type=str),
                          'uplink_task_file': ParameterValue(LaunchConfiguration('uplink_task_file'), value_type=str)}]),
    ])
