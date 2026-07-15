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
firmware/      STM32 firmware, added during hardware stage
scripts/       Build, environment, and deployment helper scripts
assets/        Images, models, and presentation assets
reports/       Innovation-project report and defense materials
```

See `docs/architecture.md` and `docs/simulation_plan.md` for the current technical plan.
