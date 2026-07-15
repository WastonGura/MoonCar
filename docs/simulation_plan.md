# Simulation Plan

## Goal

Build the project around simulation first. The initial milestone is to run a four-wheel rover model in Gazebo, publish simulated sensor topics, build a map with SLAM Toolbox, and navigate to a selected goal with Nav2.

## Milestones

1. Initialize ROS2 workspace and package skeletons.
2. Create the rover description with a skid-steer/four-wheel differential drive model.
3. Add simulated lidar, IMU, and depth camera frames/topics.
4. Create a lunar-like Gazebo world with rocks, slopes, pits, and uneven ground.
5. Bring up SLAM Toolbox and save 2D occupancy maps.
6. Bring up Nav2 with obstacle avoidance.
7. Add terrain traversability costmap experiments from depth/point-cloud data.
8. Record experiments for the innovation-project report.

## Current VM Environment

| Item | Status |
| --- | --- |
| Ubuntu | 24.04.4 LTS |
| ROS2 | Jazzy |
| Gazebo | 8.11.0 |
| Nav2 | Installed |
| SLAM Toolbox | Installed |
| robot_localization | Installed |
| ros_gz_sim / ros_gz_bridge | Installed |
| colcon | Installed |

## Repository Synchronization

The recommended workflow is to keep the same GitHub repository cloned on Windows, the Ubuntu VM, and later the Raspberry Pi.

```text
Windows repo -> commit/push -> Ubuntu VM pull/build/test
Ubuntu VM changes -> commit/push -> Windows pull/review
Raspberry Pi later -> pull stable onboard runtime
```
