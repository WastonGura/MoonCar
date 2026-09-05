# mooncar_gazebo

Gazebo simulation package for the MoonCar ROS2 stack.

## Worlds

- `worlds/lunar_yard.sdf`: 50 m lunar-like yard with low gravity, a stable spawn area, a gentle ramp, a shallow crater, a ridge, and scattered rocks for mobility and mapping tests.

## Launch

Start the M2 simulation stack:

```bash
ros2 launch mooncar_gazebo simulation.launch.py
```

For a server-only VM smoke test:

```bash
ros2 launch mooncar_gazebo simulation.launch.py gz_args:="-r -s -v 3 --render-engine ogre"
```

The launch starts Gazebo, publishes the MoonCar description, spawns the rover into the `lunar_yard` world, and bridges `/clock` into ROS2.

Gazebo's per-step clock is bridged to `/mooncar/clock_raw`. A standard
`topic_tools` throttle forwards it to `/clock` at up to 100 Hz of simulation
time, so every ROS node need not process all 1,000 physics ticks per second.
The throttle reads the raw clock itself; pause and backward time jumps remain
simulation-time driven. The physical integration step stays at 1 ms.

The launch selects Ogre for both GUI and sensors. VMware defaults to CPU
software rendering because the virtual GPU previously produced invalid sensor
measurements. Server-only rendering still requires an X display; acceptance
scripts provide Xvfb when needed.

M3 control topics bridged by this launch:

- `/mooncar/cmd_vel_safe` from ROS2 to Gazebo `/cmd_vel`
- `/mooncar/joint_states_raw` from Gazebo, then `/joint_states` through the
  standard `topic_tools` throttle at up to 30 Hz of simulation time. Gazebo 8.11
  ignores the model's `JointStatePublisher/update_rate`; without the throttle,
  every physics iteration propagates through wheel odometry. Message timestamps
  and positions are preserved. `ros2 topic hz` measures wall-time arrival rate,
  so it reports less than 30 Hz when the simulation runs slower than real time.

M3 ROS control nodes started by this launch:

- `mooncar_cmd_vel_safety_relay`: clamps `/cmd_vel` and times out stale commands.
- `mooncar_wheel_odometry`: integrates `/joint_states` and publishes `/odom` plus `odom -> base_footprint` TF.

## M2 sensor topics

Gazebo sensor placeholders are attached to frames from `mooncar_description` and bridged into ROS2:

- `/mooncar/scan` from `lidar_link`
- `/mooncar/imu` from `imu_link`
- `/mooncar/depth/image` from `depth_camera_optical_frame`
- `/mooncar/depth/camera_info` from `depth_camera_optical_frame`

## Verify on the Ubuntu VM

From the active VM clone:

```bash
cd ~/MoonCar
git pull --ff-only origin main
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mooncar_description mooncar_gazebo
source install/setup.bash
```

Run a server-only smoke test:

```bash
ros2 launch mooncar_gazebo simulation.launch.py gz_args:="-r -s -v 3 --render-engine ogre"
```

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/MoonCar/ros2_ws
source install/setup.bash
ros2 topic list | grep -E '/clock|/mooncar'
ros2 topic echo --once /clock
```

For M3 drive control checks, see `mooncar_control/README.md`.

Expected early M2 behavior:

- Gazebo loads `lunar_yard.sdf`.
- `spawn_mooncar` reports entity creation successful.
- `/clock` is available in ROS2.
- Sensor topics are expected to appear after Gazebo sensor systems initialize; rendering-backed topics may require a graphical or headless-rendering capable VM session.

## Troubleshooting

- If Gazebo cannot find the world file, rebuild `mooncar_gazebo` and source `install/setup.bash`.
- If `/clock` is missing, confirm `mooncar_gazebo/config/ros_gz_bridge.yaml` is installed and that `mooncar_gz_bridge` started.
- If sensor topics are missing in server-only mode, rerun with GUI or a headless rendering setup before treating it as a model failure.
- If the rover does not spawn, confirm `/robot_description` is being published by `robot_state_publisher`.

## Follow-up dependencies

- M3 control should attach wheel control to the existing four wheel joints.
- M4 SLAM should use `/mooncar/scan` and `lidar_link`.
- Nav2 work should treat `base_footprint` as the planar robot base frame.
- Hardware selection should replace placeholder sensor rates, noise values, masses, and poses.
