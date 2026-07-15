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

## Verify on the Ubuntu VM

Terminal 1:

```bash
cd ~/MoonCar
git pull --ff-only origin master
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mooncar_description mooncar_control mooncar_gazebo
source install/setup.bash
ros2 launch mooncar_gazebo simulation.launch.py gz_args:="-r -s -v 3"
```

Terminal 2:

```bash
cd ~/MoonCar/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run mooncar_control drive_smoke_test.py
```

Useful inspection commands:

```bash
ros2 topic list | grep -E '/cmd_vel|/mooncar/cmd_vel_safe|/joint_states|/odom|/tf'
ros2 topic echo --once /joint_states
ros2 topic echo --once /odom
```

Manual command example:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.16}, angular: {z: 0.0}}" --rate 10
```

Expected result:

- The rover moves forward for positive `linear.x`.
- Wheel positions in `/joint_states` change.
- `/odom.pose.pose.position.x` changes during forward motion.
- `/cmd_vel` automatically times out through `mooncar_cmd_vel_safety_relay` when commands stop.

## Troubleshooting

- If `/mooncar/cmd_vel_safe` is missing, check that `mooncar_cmd_vel_safety_relay` started in the launch log.
- If `/joint_states` is missing, check the Gazebo `JointStatePublisher` plugin and bridge config.
- If `/odom` is missing, check that `mooncar_wheel_odometry` is running and receiving `/joint_states`.
- If the rover does not move but `/mooncar/cmd_vel_safe` is active, inspect Gazebo `/cmd_vel` and the DiffDrive plugin configuration.
