# mooncar_navigation

Nav2 integration package for saved-map localization and autonomous navigation
in the MoonCar Gazebo simulation.

## Runtime contract

Nav2 will reuse the completed simulation and filtered-odometry pipeline. It
must not start SLAM Toolbox while localizing against a saved map.

| Interface | Type | Owner or consumer |
| --- | --- | --- |
| Map YAML | Nav2 map-server input | Created by the M4 mapping workflow |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Map server publishes; global costmap consumes |
| `/mooncar/scan` | `sensor_msgs/msg/LaserScan` | Gazebo publishes; AMCL and costmaps consume |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF publishes; controller consumes |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 controller publishes; MoonCar safety relay consumes |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | BT Navigator provides |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz or an acceptance client publishes for AMCL |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo publishes for simulation time |

The command path remains:

```text
Nav2 /cmd_vel -> safety relay -> /mooncar/cmd_vel_safe -> Gazebo DiffDrive
```

## TF ownership

Only one node owns each dynamic transform in localization mode:

| Transform | Publisher |
| --- | --- |
| `map -> odom` | AMCL |
| `odom -> base_footprint` | `mooncar_ekf` |
| `base_footprint -> base_link -> sensor frames` | `robot_state_publisher` |

Wheel odometry continues to publish `/odom`, but its `publish_tf` parameter
must be `false` whenever the EKF is running. SLAM Toolbox must remain stopped
so it cannot compete with AMCL for `map -> odom`.

## VM prerequisites

Install the Nav2 runtime on Ubuntu 24.04 with ROS2 Jazzy:

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

The active VM has been verified with Nav2 `1.3.12`. Required runtime packages
include AMCL, map server, planner, controller, behaviors, BT Navigator,
lifecycle manager, and waypoint follower.

## Saved map input

Generate the map before starting localization:

```bash
~/MoonCar/scripts/accept_m4.sh
```

The default output is `~/mooncar_m4_acceptance.yaml` with its adjacent PGM
image. The M5 launch will accept the YAML path as an explicit argument rather
than silently choosing a map.
