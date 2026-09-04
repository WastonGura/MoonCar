# Working Context

Last updated: 2026-09-05

## Current Direction

MoonCar is a simulation-first ROS2 project for a four-wheel lunar-terrain
rover. Development currently targets a repeatable Gazebo, SLAM, Nav2, and
terrain-aware autonomy pipeline before physical hardware integration.

The final deadline in `docs/requirements.md` is 2027-01-10. No rover hardware
has been purchased yet, so hardware procurement must run in parallel with the
remaining simulation milestones if a physical demonstration is required.

## Repository

- Windows path: `D:\Document\Repositories\MoonCar`
- GitHub: `https://github.com/WastonGura/MoonCar.git`
- Default branch: `main`
- Ubuntu clone: `~/MoonCar`
- Workflow: Issue -> branch/worktree -> build/runtime verification -> merge

## Active VM Environment

- IP: `192.168.248.128`
- User: `homura`
- OS: Ubuntu 24.04.4 LTS
- ROS2: Jazzy
- Gazebo Sim: 8.11.0
- Resources: 4 vCPU, 8 GB RAM, 60 GB disk
- Services: SSH and open-vm-tools enabled
- ROS environment: `/opt/ros/jazzy/setup.bash` sourced from `~/.bashrc`

The VM uses NAT networking. Its Ubuntu package source is the Tsinghua mirror;
the ROS repository remains `packages.ros.org`.

## Milestone Status

- M0 project foundation: complete
- M1 rover description and visualization: complete
- M2 Gazebo lunar simulation: complete
- M3 simulated control stack: complete and re-verified on the active VM
- M4 SLAM mapping pipeline: complete, Issues #19 through #24
- M5 Nav2 autonomous navigation: complete, Issues #30 through #35
- M6 terrain-aware autonomy and hardware readiness: planned

The post-M2 Gazebo GUI polish task remains open as Issue #12 and does not block
the mapping pipeline.

## Verified M3 Baseline

The active VM builds all eight packages. Server-only Gazebo starts the
`lunar_yard` world, spawns MoonCar, publishes sensor topics, accepts scripted
drive commands, and publishes odometry. The latest drive check reported about
0.608 m of forward travel followed by rotation.

## Completed M4 Pipeline

M4 adds:

1. Runtime validation of `/mooncar/scan`, `/mooncar/imu`, `/odom`, and TF.
2. `robot_localization` filtered odometry at `/odometry/filtered`.
3. SLAM Toolbox online asynchronous mapping at 0.05 m resolution.
4. One-command mapping bringup with optional RViz.
5. A failing smoke check that saves `.yaml` and `.pgm` map artifacts.

The VM acceptance run received laser, IMU, filtered odometry, map, and TF data.
After the scripted drive sequence, server-only Gazebo saved a 58 x 44 map at
0.05 m/pixel. The GUI launch also kept Gazebo, SLAM Toolbox, EKF, and RViz alive
for a 90-second smoke run without a process failure.

## Completed M5 Pipeline

M5 adds saved-map localization, NavFn planning, Regulated Pure Pursuit control,
bounded recovery behavior, and a one-command autonomous goal acceptance run.
The verified lunar-yard route from `(0.5, 0.0)` to `(3.5, 3.0)` crosses rock
marker A on a straight line; Nav2 routed around it and completed `4.22 m` of
filtered travel. A goal inside the ramp timed out, cancelled, and returned the
safety command to zero.

Run the complete GUI acceptance with:

```bash
~/MoonCar/scripts/accept_m5.sh
```

Use `--server-only` for a headless VM check.

## VM Build Workflow

```bash
cd ~/MoonCar/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Run M4 mapping with:

```bash
ros2 launch mooncar_slam mapping.launch.py
```
