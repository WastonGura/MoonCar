# mooncar_slam

SLAM Toolbox integration for the MoonCar Gazebo simulation.

## Mapping input contract

The mapping stack uses simulation time and expects these inputs from
`mooncar_gazebo`:

| Input | Frame or relationship |
| --- | --- |
| `/mooncar/scan` | `lidar_link` laser scans |
| `/odom` | Raw wheel odometry |
| `/mooncar/imu` | `imu_link` orientation and angular velocity |
| TF | `odom -> base_footprint -> ... -> lidar_link` |

Verify the contract while the simulation is running:

```bash
ros2 topic echo --once /mooncar/scan
ros2 topic echo --once /odom
ros2 topic echo --once /mooncar/imu
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint lidar_link
```

Each command must produce data or a transform. A topic name appearing in
`ros2 topic list` is not sufficient evidence that its sensor is publishing.

## Build

```bash
cd ~/MoonCar/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run mapping

Start Gazebo, filtered odometry, SLAM Toolbox, and RViz:

```bash
ros2 launch mooncar_slam mapping.launch.py
```

For a VM server-only check:

```bash
ros2 launch mooncar_slam mapping.launch.py \
  use_rviz:=false gz_args:="-r -s -v 2"
```

Drive the rover from another sourced terminal:

```bash
ros2 run mooncar_control keyboard_teleop.py
```

The scripted M3 motion sequence is useful for repeatable checks:

```bash
ros2 run mooncar_control drive_smoke_test.py
```

## Save and verify a map

After the rover has observed part of the world, run:

```bash
ros2 run mooncar_slam mapping_smoke_test.sh "$HOME/mooncar_map"
```

Success produces `~/mooncar_map.yaml` and `~/mooncar_map.pgm`. The check exits
nonzero if laser scans, filtered odometry, the map, either required transform,
or saved map files are missing.

Expected mapping outputs:

- `/odometry/filtered`
- `/map`
- `map -> odom -> base_footprint -> lidar_link`

The EKF parameters are simulation defaults. Wheel dimensions, sensor
covariances, and IMU behavior must be calibrated again for physical hardware.
Rendering-backed Gazebo sensors may require a graphical session when the VM
cannot provide headless rendering.
