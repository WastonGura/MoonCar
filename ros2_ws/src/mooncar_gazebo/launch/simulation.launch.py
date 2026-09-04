from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz_args = LaunchConfiguration("gz_args")
    odom_track_width = LaunchConfiguration("odom_track_width")
    robot_name = LaunchConfiguration("robot_name")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")

    robot_model = PathJoinSubstitution(
        [FindPackageShare("mooncar_description"), "urdf", "mooncar.urdf.xacro"]
    )
    bridge_config = PathJoinSubstitution(
        [FindPackageShare("mooncar_gazebo"), "config", "ros_gz_bridge.yaml"]
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare("mooncar_gazebo"), "worlds", "lunar_yard.sdf"]
    )
    gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )
    bridge_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_bridge"), "launch", "ros_gz_bridge.launch.py"]
    )

    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                robot_model,
                " use_sim_time:=",
                use_sim_time,
            ]
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument('software_rendering', default_value='false'),
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1',
                                   condition=IfCondition(LaunchConfiguration('software_rendering'))),
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Path to the Gazebo world SDF file.",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="lunar_yard",
                description="Gazebo world name used by spawn services.",
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value="-r -v 3",
                description="Arguments passed to `gz sim` before the world path.",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="mooncar",
                description="Entity name used when spawning the rover.",
            ),
            DeclareLaunchArgument(
                "publish_odom_tf",
                default_value="true",
                description="Publish odom -> base_footprint from wheel odometry.",
            ),
            DeclareLaunchArgument(
                "odom_track_width",
                default_value="0.83",
                description="Effective skid-steer track width for simulated odometry.",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("require_terrain", default_value="false"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time for ROS nodes.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={
                    "gz_args": [gz_args, " ", world],
                    "on_exit_shutdown": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge_launch),
                launch_arguments={
                    "bridge_name": "mooncar_gz_bridge",
                    "config_file": bridge_config,
                }.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[robot_description, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="mooncar_control",
                executable="cmd_vel_safety_relay.py",
                name="mooncar_cmd_vel_safety_relay",
                parameters=[{"use_sim_time": use_sim_time,
                             "require_terrain": LaunchConfiguration("require_terrain")}],
                output="screen",
            ),
            Node(
                package="mooncar_control",
                executable="wheel_odometry.py",
                name="mooncar_wheel_odometry",
                parameters=[
                    {
                        "publish_tf": publish_odom_tf,
                        "track_width": odom_track_width,
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        name="spawn_mooncar",
                        arguments=[
                            "-world",
                            world_name,
                            "-topic",
                            "/robot_description",
                            "-name",
                            robot_name,
                            "-allow_renaming",
                            "false",
                            "-x",
                            spawn_x,
                            "-y",
                            spawn_y,
                            "-z",
                            spawn_z,
                            "-Y",
                            spawn_yaw,
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
