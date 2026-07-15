# Working Context

Last updated: 2026-07-15

## Current Direction

MoonCar is a simulation-first ROS2 project for a four-wheel lunar-terrain rover prototype. The final project still targets a physical rover, but no hardware has been purchased yet, so current development should focus on Gazebo simulation, SLAM, Nav2 navigation, and terrain-aware planning.

## Repository

- Formal repository path on Windows: `D:\Document\Repositories\MoonCar`
- GitHub remote: `https://github.com/WastonGura/MoonCar.git`
- Main branch: `master`
- Commit style: commit after each small concrete function or document milestone, with specific messages.

## Development Machine Plan

- Windows host: documentation, Git coordination, Codex-side editing.
- Ubuntu VM: ROS2 build, Gazebo simulation, runtime verification.
- Raspberry Pi: later onboard deployment after hardware procurement.
- STM32: later low-level motor, encoder, and near-field sensor control.

## Confirmed VM Environment

- VM IP: `192.168.1.15`
- VM user: `jiker`
- OS: Ubuntu 24.04.4 LTS
- ROS2: Jazzy
- Gazebo: 8.11.0
- Installed/available: Nav2, SLAM Toolbox, robot_localization, ros_gz_sim, ros_gz_bridge, colcon, CMake, Python 3.12, Git.

## SSH/GitHub Status

A dedicated SSH key for GitHub was generated on the VM:

```text
~/.ssh/id_ed25519_mooncar_github
~/.ssh/id_ed25519_mooncar_github.pub
```

The VM SSH config has a `github.com` entry that uses this key. The public key has been added to GitHub account SSH keys and authentication has been verified with `ssh -T git@github.com`.

Active VM clone location:

```text
~/MoonCar
```

Earlier temporary build verification was performed from:

```text
~/MoonCar_local_verify
```

## ROS2 Workspace State

The repository contains a buildable skeleton workspace at:

```text
ros2_ws/
```

Current packages:

- `mooncar_description`
- `mooncar_gazebo`
- `mooncar_bringup`
- `mooncar_control`
- `mooncar_slam`
- `mooncar_navigation`
- `mooncar_terrain`
- `mooncar_msgs`

Build verification on the VM passed from both the temporary copy and the active SSH clone:

```text
Temporary copy: Summary: 8 packages finished [7.64s]
Active clone: Summary: 8 packages finished [12.3s]
```

## Next Recommended Milestone

1. Keep the VM repository synchronized with GitHub using SSH:

```bash
cd ~/MoonCar
git pull --ff-only origin master
```

2. Build from the VM clone:

```bash
cd ~/MoonCar/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

3. Start implementing `mooncar_description` with a four-wheel skid-steer URDF/Xacro rover model.

