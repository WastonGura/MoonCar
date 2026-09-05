# Working Context

Last updated: 2026-09-05

## Current Direction

MoonCar is a simulation-first ROS2 project for a four-wheel lunar-terrain
rover. The M6 software prototype now passes the full GUI acceptance, including
terrain constraints, new-obstacle replanning and two ground-truth-checked goals.
The next development stage is hardware commissioning, not an unverified motor launch.

The final deadline in `docs/requirements.md` is 2027-01-10. No rover hardware
has been purchased yet. Board pinout, motor/current data and independent stop
circuitry must be confirmed before HAL integration and powered bench testing.

## Repository

Performance follow-up #58 / PR #59 adds native `topic_tools` wheel/clock
throttles and `accept_m6.sh --rviz-only` (also supported by M5). Runtime
`00bdcf3` passed isolated RViz-only and full-GUI M6, with truth errors below
0.074 m. Clock pause/resume and time-reset forwarding passed. Short RTF
windows were about 0.325 RViz-only / 0.295 full GUI versus 0.235 baseline;
these are not controlled speedup guarantees. See
`reports/performance_2026-09-05.md` for evidence and limitations.
Vehicle speed, sensor resolution and 1 ms physics step are unchanged.
Nav2 action acknowledgement is 200 ms; goal/safety deadlines are unchanged.

- Windows path: `D:\Document\Repositories\MoonCar`
- GitHub: `https://github.com/WastonGura/MoonCar.git`
- Active development branch: `main`; GitHub default remains historical `master`
- Always explicitly target `main` for PRs; do not assume closing keywords close issues
- M6 worktree: `D:\Document\Repositories\MoonCar-m6`, PR #51 and follow-up PR #57
- Original Windows worktree has unrelated local edits and was left untouched
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
- M6 terrain-aware autonomy and hardware readiness: complete; PR #57 resolves the production regression found after PR #51
- M7 hardware commissioning: open, Issues #52 through #56, all `hardware-required`

Issue #12 GUI polish is closed. The M6 GUI additionally removes helper-plugin
description text from the scene and includes a working RViz Nav2 panel.

## M6 Delivery Baseline

Runtime code and map `de10bfa` passed isolated headless and default GUI acceptance
on 2026-09-05 in `/home/homura/MoonCar-m6.KPloMc`. A third full run in the
production directory `/home/homura/MoonCar` also passed. PR #51's 0.52 m
repeat error reopened Issue #48; `4de123d` also
failed a repeat at 0.53 m. Neither single-pass result is the delivery baseline.

- 18 Python regression tests plus portable C11 firmware checks with UBSan passed.
- Eight ROS packages built. GitHub Software checks passed.
- Live laser: 256 finite varied returns; depth: 3185 projected points.
- Keepout center clearance: 1.250 m; new-obstacle replanning clearance: 0.899 m.
- Headless goals `(1.5, 0)` and `(3.5, 3)`: truth errors 0.067 m and 0.067 m.
- GUI goals: truth errors 0.079 m and 0.052 m (unchanged 0.400 m acceptance gate).
- Production-directory headless goals: truth errors 0.074 m and 0.075 m.
- M4 loop survey saved a 314 x 238 map at 0.05 m/pixel, the fixed navigation map.
- 37775/74732 map cells are observed; bounds are not full surveyed coverage.
- Six laser-visible perimeter boulders provide features; featureless localization is not established.
- Captured Gazebo and RViz screenshots confirm visible scenes and active Nav2/localization.
- Human mouse-click navigation is not part of the automated goal evidence.

Read `reports/delivery_2026-09-05.md`, `docs/hardware_handoff.md` and
`docs/uart_protocol.md` before continuing. Do not claim hardware readiness from
the PTY emulator: there is no board HAL, flashable image or validated physical stop.

VM-specific fixes: software-rendered Ogre (both sensors and GUI), Fast DDS
for acceptance, Xvfb for SSH runs without DISPLAY, staggered lifecycle startup,
timestamped depth TF buffering, degenerate scanline rejection, nonholonomic EKF
constraint, self-scan filtering, axle-aligned wheel friction, and 1 Hz lifecycle
readiness polling. Unbounded rapid-response polling had flooded lifecycle services.
The original two managers are retained so RViz reports both states correctly.
Real covariance, friction,
wheel dimensions and PID settings must be measured again on the bench.

Run `~/MoonCar/scripts/accept_m6.sh` (GUI) or add `--server-only`; use
`~/MoonCar/scripts/check_software.sh` for the logic/UART regression only.
The acceptance script clears old evidence first and returns nonzero on failure.
Use `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` in additional ROS terminals.
Software checks disable ROS discovery to isolate synthetic encoder telemetry.
Manual mapping experiments that accidentally resolved `/home/homura/MoonCar`
instead of the isolated workspace were discarded. Use a fresh terminal, source
the intended overlay explicitly, and verify `ros2 pkg prefix mooncar_gazebo`
before comparing experiments. Final loop recording used the correct isolated
world and SLAM TF, not ground truth as a control input.

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
