# ROS2 Workspace

This workspace contains the ROS2 packages for MoonCar simulation, navigation, mapping, and later hardware integration.

## Build

From a Linux environment with ROS2 Jazzy sourced:

```bash
cd ~/MoonCar/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Package Roles

| Package | Role |
| --- | --- |
| mooncar_description | Rover model, URDF/Xacro, meshes, RViz files |
| mooncar_gazebo | Gazebo worlds and simulation launch files |
| mooncar_bringup | Integrated launch entry points |
| mooncar_control | Control and odometry adapters |
| mooncar_slam | SLAM Toolbox launch/config files |
| mooncar_navigation | Nav2 launch/config files |
| mooncar_terrain | Terrain traversability experiments |
| mooncar_msgs | Custom message definitions when needed |
