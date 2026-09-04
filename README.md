# MoonCar

MoonCar is a ROS2-based four-wheel lunar-terrain rover prototype for mapping, route planning, and obstacle avoidance in simulated lunar-like environments, with later migration to physical hardware.

## Current Focus

The project is currently simulation-first:

1. Build and validate the ROS2/Gazebo simulation pipeline.
2. Implement SLAM and Nav2 navigation in simulated lunar terrain.
3. Add terrain-aware traversability analysis.
4. Migrate stable ROS2 components to Raspberry Pi hardware after procurement.

## Repository Layout

```text
docs/          Project documents, plans, reports, and meeting notes
ros2_ws/       ROS2 workspace, added after structure confirmation
firmware/      Portable STM32 control core and host tests; board HAL is hardware-gated
scripts/       Build, environment, and deployment helper scripts
assets/        Images, models, and presentation assets
reports/       Innovation-project report and defense materials
```

See `docs/architecture.md` and `docs/simulation_plan.md` for the current technical plan.

## One-command Acceptance

On Ubuntu 24.04 with ROS 2 Jazzy and the project dependencies installed:

```bash
~/MoonCar/scripts/accept_m6.sh
```

Use `--server-only` to omit Gazebo GUI and RViz. The script builds the workspace,
runs software/UART regression checks, tests terrain keepout and a newly introduced
obstacle, then drives two goals. Exit status is nonzero on failure. Close any previous
MoonCar simulation before running; press Enter after a GUI run to close its processes.
SSH runs without a display need Ubuntu packages `xvfb` and `xauth`. The scripts
start a private virtual display and clean it up automatically. VMware defaults
to software-rendered Ogre for both sensor data and the GUI; a visible scene alone
does not prove that the virtual GPU produces valid laser/depth measurements.
Acceptance uses Jazzy's `rmw_fastrtps_cpp` by default (resolve workspace
dependencies with rosdep).
Use the same `RMW_IMPLEMENTATION` in additional ROS terminals. An explicit
environment override is preserved.
Evidence is written to `~/mooncar_m6_acceptance.json` and
`/tmp/mooncar_m6_acceptance.log`. A JSON result is created only on success.

M4 mapping remains available through `scripts/accept_m4.sh`; M5 navigation through
`scripts/accept_m5.sh`. Software-only checks: `scripts/check_software.sh`.

See [hardware handoff](docs/hardware_handoff.md) and [UART v1](docs/uart_protocol.md)
before connecting real motors. The hardware launch is a disarmed bench profile,
not an already validated autonomous real-rover deployment.

The [M6 delivery report](reports/delivery_2026-09-05.md) includes actual VM
acceptance results, screenshots, known limits and the five hardware-gated M7 issues.
