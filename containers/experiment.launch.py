"""Fresh Gazebo GUI, localization and recorder; motion requires a leased Nav2 goal."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('initial_x'), DeclareLaunchArgument('initial_y'),
        DeclareLaunchArgument('initial_z',default_value='0'),
        DeclareLaunchArgument('world_name',default_value='lunar_yard'),
        DeclareLaunchArgument('render_engine',default_value='ogre',choices=['ogre','ogre2']),
        DeclareLaunchArgument('initial_yaw'), DeclareLaunchArgument('output_directory'),
        DeclareLaunchArgument('capture_gui',default_value='true'),
        DeclareLaunchArgument('simulation_profile',default_value='legacy'),
        DeclareLaunchArgument('laser_odometry',default_value=EnvironmentVariable('MOONCAR_LASER_ODOMETRY',default_value='true')),
        DeclareLaunchArgument('laser_odometry_mode',default_value=EnvironmentVariable('MOONCAR_LASER_ODOMETRY_MODE',default_value='rgbd_window')),
        DeclareLaunchArgument('uplink_url', default_value=EnvironmentVariable('MOONCAR_UPLINK_URL', default_value='')),
        DeclareLaunchArgument('uplink_token_file', default_value=EnvironmentVariable('MOONCAR_UPLINK_TOKEN_FILE', default_value='')),
        DeclareLaunchArgument('uplink_task_file', default_value=EnvironmentVariable('MOONCAR_UPLINK_TASK_FILE', default_value='')),
        DeclareLaunchArgument('source_description', default_value='Gazebo 仿真；定位方案以本次启动参数为准；深度观测采集；历史深度位姿未做回环修正'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('mooncar_navigation'), 'launch', 'terrain_navigation.launch.py'])), launch_arguments={
                'initial_x': LaunchConfiguration('initial_x'), 'initial_y': LaunchConfiguration('initial_y'),
                'initial_z': LaunchConfiguration('initial_z'),
                'world_name': LaunchConfiguration('world_name'),
                'initial_yaw': LaunchConfiguration('initial_yaw'), 'use_rviz': 'false',
                'require_workbench': 'true', 'gz_args': ['-r -s -v 2 --render-engine ',LaunchConfiguration('render_engine')],
                'online_mapping':'true', 'world':[LaunchConfiguration('output_directory'),'/survey_yard.sdf'],
                'simulation_profile':LaunchConfiguration('simulation_profile'),
                'laser_odometry':LaunchConfiguration('laser_odometry'),
                'laser_odometry_mode':LaunchConfiguration('laser_odometry_mode'),
            }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('mooncar_terrain'), 'launch', 'record_map.launch.py'])), launch_arguments={
                'output_directory': LaunchConfiguration('output_directory'),
                'require_laser_localization': LaunchConfiguration('laser_odometry'),
                'source': LaunchConfiguration('source_description'),
                'uplink_url': LaunchConfiguration('uplink_url'),
                'uplink_token_file': LaunchConfiguration('uplink_token_file'),
                'uplink_task_file': LaunchConfiguration('uplink_task_file'),
            }.items()),
        ExecuteProcess(cmd=['python3',str(Path(__file__).with_name('start_scene_gui.py')),LaunchConfiguration('output_directory'),
                            LaunchConfiguration('world_name'),LaunchConfiguration('render_engine')],
                       condition=IfCondition(LaunchConfiguration('capture_gui'))),
        ExecuteProcess(cmd=['python3',str(Path(__file__).with_name('capture_scene.py')),LaunchConfiguration('output_directory')],
                       condition=IfCondition(LaunchConfiguration('capture_gui'))),
    ])
