from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz_args = LaunchConfiguration("gz_args")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    simulation_launch = PathJoinSubstitution(
        [FindPackageShare("mooncar_gazebo"), "launch", "simulation.launch.py"]
    )
    default_params = PathJoinSubstitution(
        [FindPackageShare("mooncar_navigation"), "config", "nav2_params.yaml"]
    )
    ekf_params = PathJoinSubstitution(
        [FindPackageShare("mooncar_slam"), "config", "ekf.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("mooncar_navigation"), "rviz", "navigation.rviz"]
    )

    node_parameters = [params_file, {"use_sim_time": use_sim_time}]
    localization_nodes = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[
                params_file,
                {"yaml_filename": map_yaml, "use_sim_time": use_sim_time},
            ],
            output="screen",
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            parameters=[
                params_file,
                {
                    "initial_pose.x": ParameterValue(initial_x, value_type=float),
                    "initial_pose.y": ParameterValue(initial_y, value_type=float),
                    "initial_pose.yaw": ParameterValue(initial_yaw, value_type=float),
                    "use_sim_time": use_sim_time,
                },
            ],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            parameters=[
                {"autostart": True, "use_sim_time": use_sim_time},
                {"node_names": ["map_server", "amcl"]},
            ],
            output="screen",
        ),
    ]
    navigation_nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            parameters=node_parameters,
            output="screen",
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            parameters=node_parameters,
            output="screen",
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            parameters=node_parameters,
            output="screen",
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            parameters=node_parameters,
            output="screen",
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            parameters=node_parameters,
            output="screen",
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            parameters=node_parameters,
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[
                {"autostart": True, "use_sim_time": use_sim_time},
                {
                    "node_names": [
                        "controller_server",
                        "smoother_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "waypoint_follower",
                    ]
                },
            ],
            output="screen",
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map", description="Absolute path to a saved occupancy-map YAML file."
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Nav2 parameter file.",
            ),
            DeclareLaunchArgument(
                "gz_args", default_value="-r -v 3 --render-engine-gui ogre"
            ),
            DeclareLaunchArgument("initial_x", default_value="0.5"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("require_terrain", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_launch),
                launch_arguments={
                    "gz_args": gz_args,
                    "publish_odom_tf": "false",
                    "spawn_x": initial_x,
                    "spawn_y": initial_y,
                    "spawn_yaw": initial_yaw,
                    "use_sim_time": use_sim_time,
                    "require_terrain": LaunchConfiguration("require_terrain"),
                }.items(),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="mooncar_ekf",
                parameters=[ekf_params, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            TimerAction(
                period=5.0,
                actions=[
                    *localization_nodes,
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="mooncar_navigation_rviz",
                        arguments=["-d", rviz_config],
                        parameters=[{"use_sim_time": use_sim_time}],
                        condition=IfCondition(use_rviz),
                        output="screen",
                    ),
                ],
            ),
            TimerAction(period=10.0, actions=navigation_nodes),
        ]
    )
