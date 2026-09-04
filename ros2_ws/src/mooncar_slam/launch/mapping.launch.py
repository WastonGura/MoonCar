from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz_args = LaunchConfiguration("gz_args")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ekf_params = LaunchConfiguration("ekf_params_file")
    slam_params = LaunchConfiguration("slam_params_file")
    rviz_config = LaunchConfiguration("rviz_config")

    simulation_launch = PathJoinSubstitution(
        [FindPackageShare("mooncar_gazebo"), "launch", "simulation.launch.py"]
    )
    slam_launch = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
    )
    default_ekf_params = PathJoinSubstitution(
        [FindPackageShare("mooncar_slam"), "config", "ekf.yaml"]
    )
    default_slam_params = PathJoinSubstitution(
        [
            FindPackageShare("mooncar_slam"),
            "config",
            "mapper_params_online_async.yaml",
        ]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("mooncar_slam"), "rviz", "mapping.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gz_args", default_value="-r -v 3 --render-engine-gui ogre"
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("ekf_params_file", default_value=default_ekf_params),
            DeclareLaunchArgument("slam_params_file", default_value=default_slam_params),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_launch),
                launch_arguments={
                    "gz_args": gz_args,
                    "publish_odom_tf": "false",
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="mooncar_ekf",
                parameters=[ekf_params, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    "autostart": "true",
                    "slam_params_file": slam_params,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="mooncar_mapping_rviz",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
