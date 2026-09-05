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

## Navigation parameters

`config/nav2_params.yaml` configures AMCL, the map server, NavFn, Regulated
Pure Pursuit, local and global costmaps, recovery behaviors, waypoint
following, and their lifecycle managers. The costmaps use the rover's
`0.86 m x 0.72 m` footprint and consume `/mooncar/scan` for obstacle updates.

## Run navigation

For the repeatable end-to-end acceptance run, use:

```bash
~/MoonCar/scripts/accept_m5.sh
```

Use `--server-only` for a headless run. The commands below are retained for
manual development and custom maps.

Start Gazebo, filtered odometry, saved-map localization, Nav2, and RViz:

```bash
ros2 launch mooncar_navigation navigation.launch.py \
  map:="$HOME/mooncar_m4_acceptance.yaml"
```

The `map` argument is required. For a server-only VM check:

```bash
ros2 launch mooncar_navigation navigation.launch.py \
  map:="$HOME/mooncar_m4_acceptance.yaml" \
  use_rviz:=false gz_args:="-r -s -v 2 --render-engine ogre"
```

The launch keeps wheel-odometry TF disabled, starts the EKF as the sole
`odom -> base_footprint` publisher, and uses AMCL as the sole `map -> odom`
publisher.

The default initial pose is `x=0.5, y=0.0, yaw=0.0`, safely inside maps made
by the M4 acceptance run. Override `initial_x`, `initial_y`, and `initial_yaw`
when using a map created from a different starting pose.

In another sourced terminal, send the repeatable acceptance goal:

```bash
ros2 run mooncar_navigation navigation_smoke_test.py
```

The test exits successfully only when Nav2 reports success and filtered
odometry shows at least `0.5 m` of travel. Override the defaults with ROS
parameters such as `--ros-args -p goal_x:=2.0 -p goal_timeout_s:=180.0`.
For simulation, also pass `-p use_sim_time:=true` so the navigation deadline
uses `/clock`. The one-command acceptance script sets this automatically.
`wall_timeout_s` (600 seconds) and `clock_stall_timeout_s` (30 seconds) bound
slow or paused runs independently of simulation time.
The simulated controller uses `8.0 rad/s^2` and a `0.8 rad/s` rotation command.
Wheel collisions define axle-aligned lateral and longitudinal friction so
the four-wheel chassis can skid-steer at low turn rates. Gazebo's drivetrain
still limits actual commanded acceleration to `1.0 rad/s^2`, and the safety
relay caps rotation at `0.8 rad/s`. These are simulation settings, not validated
motor-bench parameters. The one-command scripts select Fast DDS and provide
Xvfb when no display is available; the manual command above requires an X display.

Run the timeout regression checks in a sourced ROS environment with
`python3 -m unittest discover -s src/mooncar_navigation/tests` from `ros2_ws`.

## Lunar-yard obstacle route

The package includes a fixed SLAM observation map for navigation tuning
(314 x 238 cells at 0.05 m, with unknown cells preserved; see `maps/README.md`):

```bash
MAP="$(ros2 pkg prefix mooncar_navigation)/share/mooncar_navigation/maps/lunar_yard.yaml"
ros2 launch mooncar_navigation navigation.launch.py map:="$MAP"
```

The tested obstacle route starts at `(0.5, 0.0)` and ends at `(3.5, 3.0)`.
The M6 terrain profile is required for rocks below the laser scan plane;
the basic M5 profile only uses the saved map and live 2D laser:

```bash
ros2 run mooncar_navigation navigation_smoke_test.py --ros-args \
  -p goal_x:=3.5 -p goal_y:=3.0 \
  -p goal_timeout_s:=180.0 -p min_displacement_m:=2.0
```

The costmaps use the rover footprint plus `0.65 m` inflation, mark laser
obstacles to `6 m`, and raytrace clearing observations to `8 m`. Unknown cells
remain traversable for sparse SLAM maps, while laser collision checking remains
enabled. Simulated wheel odometry retains the nominal `0.83 m` effective track
setting; raw wheel yaw is not ground truth and is not fused by the EKF. The progress checker
requires `0.10 m` within `20 s`; recovery remains bounded to clear, spin, wait,
and backup behaviors before returning failure. The EKF takes forward velocity
and a zero lateral-velocity constraint from wheel odometry, with heading from
the IMU. Wheel velocity uncertainties are nominal and require real calibration.

## M6 acceptance

`~/MoonCar/scripts/accept_m6.sh` adds depth terrain risks, prior keepout zones,
new-obstacle replanning and two consecutive goals. Success requires Gazebo
ground-truth position error below 0.4 m for each goal and bounded EKF covariance,
not merely a successful Nav2 action. Results are written to
`~/mooncar_m6_acceptance.json`; unknown or nearly collinear depth samples are
not labeled as safe terrain. This is a geometric prototype, not soil recognition.
