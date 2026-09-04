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
