"""Bench bringup only: sensor drivers and calibrated transforms are site-specific."""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    port = LaunchConfiguration('port').perform(context)
    ticks = int(LaunchConfiguration('encoder_ticks_per_rev').perform(context))
    radius = float(LaunchConfiguration('wheel_radius').perform(context))
    track = float(LaunchConfiguration('track_width').perform(context))
    if not port or ticks <= 0 or not all(math.isfinite(v) and v > 0 for v in (radius, track)):
        raise ValueError('Hardware requires port, measured encoder counts, radius and effective track')
    return [
        Node(package='mooncar_control', executable='serial_bridge.py', parameters=[{
            'port': port, 'encoder_ticks_per_rev': ticks, 'use_sim_time': False,
        }], output='screen'),
        Node(package='mooncar_control', executable='cmd_vel_safety_relay.py',
             parameters=[{'use_sim_time': False}], output='screen'),
        Node(package='mooncar_control', executable='wheel_odometry.py', parameters=[{
            'wheel_radius': radius, 'track_width': track, 'publish_tf': True,
            'use_sim_time': False,
        }], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port'), DeclareLaunchArgument('encoder_ticks_per_rev'),
        DeclareLaunchArgument('wheel_radius'), DeclareLaunchArgument('track_width'),
        OpaqueFunction(function=setup),
    ])
