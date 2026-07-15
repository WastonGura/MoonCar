# MoonCar System Architecture

## Project Positioning

MoonCar is a ground-based lunar-terrain rover prototype for innovation-project validation. The goal is not to build a flight-ready lunar rover, but to demonstrate mapping, terrain-aware route planning, and obstacle avoidance in simulated lunar-like terrain, then migrate the validated ROS2 system to a physical four-wheel rover.

## Development Stages

1. Simulation-first development on Ubuntu 24.04 + ROS2 Jazzy + Gazebo.
2. Hardware integration after procurement, with Raspberry Pi as the onboard computer.
3. STM32-based low-level motor, encoder, and near-field sensor control.
4. Final report, experiment records, demos, and defense materials.

## Runtime Roles

| Platform | Stage | Responsibility |
| --- | --- | --- |
| Windows host | Development management | Documentation, Git coordination, writing and review |
| Ubuntu VM | Current main development | ROS2 build, Gazebo simulation, SLAM/Nav2 validation |
| Raspberry Pi | Physical rover stage | Onboard ROS2 runtime, sensor drivers, navigation nodes |
| STM32 | Physical rover stage | Motor PWM, encoder odometry, infrared safety layer |

## ROS2 Module Plan

| Package | Responsibility |
| --- | --- |
| mooncar_description | URDF/Xacro model, links, joints, sensor mounting frames |
| mooncar_gazebo | Gazebo worlds, lunar terrain, simulation launch files |
| mooncar_bringup | Integrated startup entry points and shared parameters |
| mooncar_control | Differential/skid-steer control interfaces and odometry adapters |
| mooncar_slam | SLAM Toolbox configuration and mapping launch files |
| mooncar_navigation | Nav2 configuration, costmaps, behavior tree configuration |
| mooncar_terrain | Depth/point-cloud terrain traversability analysis |
| mooncar_msgs | Custom messages if standard ROS2 messages are insufficient |

## Mapping Target

The baseline requirement is a 2D occupancy grid for navigation. The extended target is terrain-aware mapping, using depth or point-cloud data to estimate slope, pits, roughness, and traversability. A full 3D map is treated as an advanced deliverable after the 2D navigation pipeline is stable.
