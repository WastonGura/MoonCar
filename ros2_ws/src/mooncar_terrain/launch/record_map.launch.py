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
        DeclareLaunchArgument('output_directory', default_value=str(Path.home() / '.local/share/mooncar/maps')),
        Node(package='mooncar_terrain', executable='map_recorder.py', output='screen',
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                          'source': ParameterValue(LaunchConfiguration('source'), value_type=str),
                          'frame': LaunchConfiguration('frame'),
                          'output_directory': LaunchConfiguration('output_directory')}]),
    ])
