# mooncar_control

Simulation control package for the MoonCar ROS2 stack.

## M3 control architecture

M3 uses Gazebo Sim's built-in `gz::sim::systems::DiffDrive` plugin as the first simulated drive controller. ROS2 commands are bridged into Gazebo with `ros_gz_bridge`; no custom hardware-facing controller is introduced yet.

This keeps the first moving prototype small and repeatable:

- ROS2 publishes `geometry_msgs/msg/Twist` on `/cmd_vel`.
- `mooncar_cmd_vel_safety_relay` clamps `/cmd_vel`, applies a timeout, and republishes `/mooncar/cmd_vel_safe`.
- `ros_gz_bridge` forwards `/mooncar/cmd_vel_safe` to Gazebo `/cmd_vel`.
- Gazebo `DiffDrive` maps commands to the four skid-steer wheel joints.
- Gazebo publishes wheel `joint_states`.
- `mooncar_wheel_odometry` integrates wheel joint states into `/odom` and publishes `odom -> base_footprint` TF.

## Drive joints

Left side:

- `front_left_wheel_joint`
- `rear_left_wheel_joint`

Right side:

- `front_right_wheel_joint`
- `rear_right_wheel_joint`

The wheel radius is `0.16 m`. The placeholder track width is derived from the model's left/right wheel centers: `0.59 m`.

## Command convention

- Positive `/cmd_vel.linear.x` drives forward.
- Negative `/cmd_vel.linear.x` drives backward.
- Positive `/cmd_vel.angular.z` turns left.
- Negative `/cmd_vel.angular.z` turns right.

## Initial limits

The Gazebo drive plugin uses conservative limits for early terrain simulation:

- Max linear velocity: `0.35 m/s`
- Max angular velocity: `0.8 rad/s`
- Max linear acceleration: `0.5 m/s^2`
- Max angular acceleration: `1.0 rad/s^2`

These values are placeholders and should be revisited when wheel motors, rover mass, and terrain constraints are known.

## Verification direction

The accepted M3 path is:

```text
/cmd_vel -> safety relay -> ros_gz_bridge -> Gazebo DiffDrive -> wheel joints -> /joint_states -> wheel odometry -> /odom
```

Future hardware control should keep `/cmd_vel`, odometry, and joint state topic contracts stable while replacing the Gazebo-only implementation.
