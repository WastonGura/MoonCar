from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    prefix = LaunchConfiguration("prefix")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    model_path = PathJoinSubstitution(
        [FindPackageShare("mooncar_description"), "urdf", "mooncar.urdf.xacro"]
    )
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("mooncar_description"), "rviz", "mooncar_description.rviz"]
    )

    robot_description = {
        "robot_description": ParameterValue(Command(
            [
                FindExecutable(name="xacro"),
                " ",
                model_path,
                " prefix:=",
                prefix,
                " use_sim_time:=",
                use_sim_time,
            ]
        ), value_type=str)
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "prefix",
                default_value="",
                description="Optional prefix applied to all robot links and joints.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the MoonCar description view.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulated time when publishing robot state.",
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
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_path],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
