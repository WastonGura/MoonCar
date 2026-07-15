# mooncar_description

Robot description package for the MoonCar rover simulation model.

## Model entry point

The top-level model is:

```text
urdf/mooncar.urdf.xacro
```

It currently defines a valid rover root structure with:

- `base_footprint`
- `base_link`
- four continuous skid-steer wheel joints
- fixed placeholder sensor mounts
- placeholder chassis and wheel dimensions
- visual, collision, and inertial properties for the chassis and wheels
- reusable material and inertial macros in `urdf/common/`

The dimensions are conservative placeholders until hardware is selected.

## Frame and joint names

Physical links:

- `base_link`
- `front_left_wheel_link`
- `front_right_wheel_link`
- `rear_left_wheel_link`
- `rear_right_wheel_link`
- `lidar_link`
- `depth_camera_link`
- `depth_camera_optical_frame`
- `imu_link`
- `nav_reference_link`

Wheel joints:

- `front_left_wheel_joint`
- `front_right_wheel_joint`
- `rear_left_wheel_joint`
- `rear_right_wheel_joint`

The wheel joints rotate around their local `Y` axis. Left and right wheels use the same positive joint axis so controller sign conventions can be handled explicitly in the control layer.

Sensor frames are fixed to `base_link`. The camera optical frame follows the ROS optical frame convention: positive `Z` forward, positive `X` right, and positive `Y` down.

## Placeholder dimensions

These values are simulation placeholders and should be revised once physical hardware is selected:

| Item | Value |
| --- | ---: |
| Chassis length | `0.72 m` |
| Chassis width | `0.48 m` |
| Chassis height | `0.18 m` |
| Chassis mass | `12.0 kg` |
| Ground clearance | `0.12 m` |
| Wheel radius | `0.16 m` |
| Wheel width | `0.11 m` |
| Wheel mass | `1.2 kg` |

Sensor placeholders relative to `base_link`:

| Frame | Pose intent |
| --- | --- |
| `lidar_link` | Top-front center mount for 2D scan simulation |
| `depth_camera_link` | Forward-facing front camera body |
| `depth_camera_optical_frame` | Optical frame for RGB/depth image topics |
| `imu_link` | Center-top IMU reference |
| `nav_reference_link` | Rear-top navigation or antenna reference point |

## Visualization launch

Inspect the model without Gazebo:

```bash
ros2 launch mooncar_description display.launch.py
```

Useful launch arguments:

```bash
ros2 launch mooncar_description display.launch.py use_rviz:=false
ros2 launch mooncar_description display.launch.py prefix:=mooncar_
```

The launch file starts `joint_state_publisher`, `robot_state_publisher`, and optionally RViz with `rviz/mooncar_description.rviz`.

## Verify on the Ubuntu VM

From the active VM clone:

```bash
cd ~/MoonCar/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mooncar_description
source install/setup.bash
ros2 run xacro xacro src/mooncar_description/urdf/mooncar.urdf.xacro > /tmp/mooncar.urdf
ros2 launch mooncar_description display.launch.py use_rviz:=false
```

The Xacro command should finish without missing include or package lookup errors.
The launch command should start `joint_state_publisher` and `robot_state_publisher` without missing package or resource errors.

## Follow-up dependencies

- Gazebo work should reuse the existing physical link and joint names instead of introducing a parallel model.
- Control work should bind to the four wheel joints listed above.
- SLAM and Nav2 work should reference `lidar_link`, `depth_camera_optical_frame`, `imu_link`, and `base_footprint` consistently.
- Hardware-specific dimensions, masses, and sensor poses should be updated here when parts are selected.
