from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz_args = LaunchConfiguration("gz_args")
    robot_name = LaunchConfiguration("robot_name")
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
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
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
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[robot_description, {"use_sim_time": use_sim_time}],
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
