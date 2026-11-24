from pathlib import Path
import importlib.util
import os

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, SetLaunchConfiguration, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EqualsSubstitution, FindExecutable, IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def checked_physics_world(context):
    spec = importlib.util.spec_from_file_location('mooncar_physics_guard', Path(__file__).with_name('physics_guard.py'))
    guard = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(guard)
    world = LaunchConfiguration('world').perform(context)
    support_world=None
    if LaunchConfiguration('simulation_profile').perform(context)=='r5_support_v1':
        module_spec=importlib.util.spec_from_file_location('support_world',Path(__file__).with_name('support_profile.py'))
        module=importlib.util.module_from_spec(module_spec);module_spec.loader.exec_module(module)
        support_world=module.prepare_world(world)
    checked = guard.prepare_world(support_world or world, os.environ.get('MOONCAR_DART_PLUGIN_ROOT'),
                                  FindPackageShare('mooncar_gazebo').perform(context))
    if support_world:Path(support_world).unlink(missing_ok=True)
    if checked == world:
        return []
    def cleanup(_context):
        Path(checked).unlink(missing_ok=True)
        return []
    return [SetLaunchConfiguration('world', checked),
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))]


def checked_image_bridge(context):
    profile_spec = importlib.util.spec_from_file_location('mooncar_camera_sensor_profile',
        Path(__file__).with_name('camera_sensor_profile.py'))
    profile_adapter = importlib.util.module_from_spec(profile_spec)
    profile_spec.loader.exec_module(profile_adapter)
    profile = LaunchConfiguration('camera_sensor_profile', default='legacy').perform(context)
    source = Path(FindPackageShare('mooncar_gazebo').perform(context)) / 'config' / LaunchConfiguration('bridge_config_file').perform(context)
    selected = profile_adapter.prepare_bridge_config(source, profile)
    cleanup_actions = []
    if selected != source:
        context.launch_configurations['bridge_config_file'] = str(selected)
        def cleanup(_context):
            selected.unlink(missing_ok=True)
            return []
        cleanup_actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))
    spec = importlib.util.spec_from_file_location('mooncar_image_dds_launch', Path(__file__).with_name('image_dds_launch.py'))
    adapter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapter)
    try:
        return cleanup_actions + adapter.launch_bridge(context)
    except BaseException:
        if selected != source:
            selected.unlink(missing_ok=True)
        raise


def turn_adapter(context, operation):
    spec = importlib.util.spec_from_file_location('mooncar_turn_launch', Path(__file__).with_name('turn_launch.py'))
    adapter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapter)
    return getattr(adapter, operation)(context)


def checked_turn_profile(context):
    return turn_adapter(context, 'configure')


def checked_simulator(context):
    return turn_adapter(context, 'simulator')


def generate_launch_description():
    product = Path('/sys/class/dmi/id/product_name')
    vmware = product.is_file() and 'VMware' in product.read_text()
    gz_args = LaunchConfiguration("gz_args")
    bridge_config_file = LaunchConfiguration("bridge_config_file")
    model_file = LaunchConfiguration("model_file")
    is_r5 = EqualsSubstitution(model_file, "mooncar_r5.urdf.xacro")
    laser_filter_file = LaunchConfiguration("laser_filter_file")
    odom_track_width = LaunchConfiguration("odom_track_width")
    odom_wheel_radius = LaunchConfiguration("odom_wheel_radius")
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
        [FindPackageShare("mooncar_description"), "urdf", model_file]
    )
    bridge_config = PathJoinSubstitution(
        [FindPackageShare("mooncar_gazebo"), "config", bridge_config_file]
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
        "robot_description": ParameterValue(Command(
            [
                FindExecutable(name="python3"), " ",
                PathJoinSubstitution([FindPackageShare("mooncar_gazebo"), "launch", "turn_model.py"]),
                " --model ",
                robot_model,
                " --profile ", LaunchConfiguration('_turn_selected'),
                " --root ", LaunchConfiguration('_turn_root'),
                " --use-sim-time ",
                use_sim_time,
                " --camera-sensor-profile ", LaunchConfiguration('camera_sensor_profile'),
            ]
        ), value_type=str)
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument('simulation_profile',default_value='legacy',description='legacy or opt-in r5_support_v1 (uncalibrated support candidate)'),
            DeclareLaunchArgument('camera_sensor_profile', default_value='legacy', choices=['legacy','rgbd_v1'],
                description='legacy depth or opt-in aligned simulated RGBD; no hardware calibration implied'),
            DeclareLaunchArgument('software_rendering', default_value='true' if vmware else 'false'),
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1',
                                   condition=IfCondition(LaunchConfiguration('software_rendering'))),
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                PathJoinSubstitution([FindPackageShare('mooncar_gazebo'), '..']),
            ),
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
                default_value="-r -v 3 --render-engine ogre",
                description="Arguments passed to `gz sim` before the world path.",
            ),
            DeclareLaunchArgument(
                "model_file",
                default_value=IfElseSubstitution(EqualsSubstitution(LaunchConfiguration('simulation_profile'),'r5_support_v1'),'mooncar_r5.urdf.xacro','mooncar.urdf.xacro'),
                description="URDF Xacro filename inside mooncar_description/urdf.",
            ),
            DeclareLaunchArgument(
                "bridge_config_file",
                default_value=IfElseSubstitution(is_r5, "ros_gz_bridge_r5.yaml", "ros_gz_bridge.yaml"),
                description="Bridge YAML filename; default follows the selected model.",
            ),
            DeclareLaunchArgument(
                "laser_filter_file",
                default_value=IfElseSubstitution(is_r5, "laser_filter_r5.yaml", "laser_filter.yaml"),
                description="Body-return filter YAML filename; default follows the selected model.",
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
                default_value=IfElseSubstitution(is_r5, "0.3311", "0.83"),
                description="Odometry track width; R5 geometric estimate needs skid-steer calibration.",
            ),
            DeclareLaunchArgument(
                "odom_wheel_radius",
                default_value=IfElseSubstitution(is_r5, "0.076", "0.16"),
                description="Wheel radius for simulated odometry; default follows the selected model.",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("require_terrain", default_value="false"),
            DeclareLaunchArgument("require_workbench", default_value="false"),
            DeclareLaunchArgument("require_laser_localization", default_value="false"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time for ROS nodes.",
            ),
            OpaqueFunction(function=checked_turn_profile),
            OpaqueFunction(function=checked_physics_world),
            OpaqueFunction(function=checked_simulator),
            OpaqueFunction(function=checked_image_bridge),
            Node(
                package="topic_tools",
                executable="throttle",
                name="mooncar_clock_throttle",
                arguments=["messages", "/mooncar/clock_raw", "100.0", "/clock_out"],
                parameters=[{"use_sim_time": True, "use_wall_clock": False}],
                # Read the raw clock, not our own throttled output.
                remappings=[("/clock", "/mooncar/clock_raw"), ("/clock_out", "/clock")],
                output="screen",
            ),
            Node(
                # Legacy retains its original raw-to-30 Hz chain. RGBD bridges
                # the already 100 Hz Gazebo joint stream directly, without a
                # second ROS-clock gate before timestamped gimbal TF and odom.
                package="topic_tools",
                executable="throttle",
                name="mooncar_joint_state_throttle",
                arguments=["messages", "/mooncar/joint_states_raw", "30.0", "/joint_states"],
                parameters=[{"use_sim_time": use_sim_time, "use_wall_clock": False}],
                condition=UnlessCondition(EqualsSubstitution(
                    LaunchConfiguration('camera_sensor_profile'),'rgbd_v1')),
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[robot_description, {"use_sim_time": use_sim_time,
                    'publish_frequency': ParameterValue(IfElseSubstitution(
                        EqualsSubstitution(LaunchConfiguration('camera_sensor_profile'),'rgbd_v1'),'100.0',
                        IfElseSubstitution(EqualsSubstitution(LaunchConfiguration('simulation_profile'),'r5_support_v1'),'30.0','20.0')),value_type=float)}],
                output="screen",
            ),
            Node(
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                parameters=[PathJoinSubstitution([
                    FindPackageShare("mooncar_gazebo"), "config", laser_filter_file
                ]), {"use_sim_time": use_sim_time}],
                remappings=[("scan", "/mooncar/scan_raw"),
                            ("scan_filtered", "/mooncar/scan")],
                output="screen",
            ),
            Node(
                package="mooncar_control",
                executable="cmd_vel_safety_relay.py",
                name="mooncar_cmd_vel_safety_relay",
                parameters=[{"use_sim_time": use_sim_time,
                             "require_terrain": LaunchConfiguration("require_terrain"),
                             "require_workbench": LaunchConfiguration("require_workbench"),
                             "require_laser_localization": ParameterValue(
                                 LaunchConfiguration("require_laser_localization"), value_type=bool)}],
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
                        "wheel_radius": odom_wheel_radius,
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
