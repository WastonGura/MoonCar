"""Fresh Gazebo GUI, localization and recorder; motion requires a leased Nav2 goal."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('initial_x'), DeclareLaunchArgument('initial_y'),
        DeclareLaunchArgument('initial_yaw'), DeclareLaunchArgument('output_directory'),
        DeclareLaunchArgument('capture_gui',default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('mooncar_navigation'), 'launch', 'terrain_navigation.launch.py'])), launch_arguments={
                'initial_x': LaunchConfiguration('initial_x'), 'initial_y': LaunchConfiguration('initial_y'),
                'initial_yaw': LaunchConfiguration('initial_yaw'), 'use_rviz': 'false',
                'require_workbench': 'true', 'gz_args': '-r -s -v 2 --render-engine ogre',
                'online_mapping':'true', 'world':[LaunchConfiguration('output_directory'),'/survey_yard.sdf'],
            }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('mooncar_terrain'), 'launch', 'record_map.launch.py'])), launch_arguments={
                'output_directory': LaunchConfiguration('output_directory'),
                'source': 'Gazebo 50×50 米仿真；激光在线建图定位、深度观测采集；历史深度位姿未做回环修正',
            }.items()),
        ExecuteProcess(cmd=['python3',str(Path(__file__).with_name('start_scene_gui.py')),LaunchConfiguration('output_directory')],
                       condition=IfCondition(LaunchConfiguration('capture_gui'))),
        ExecuteProcess(cmd=['python3',str(Path(__file__).with_name('capture_scene.py')),LaunchConfiguration('output_directory')],
                       condition=IfCondition(LaunchConfiguration('capture_gui'))),
    ])
