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
- placeholder chassis and wheel dimensions
- visual, collision, and inertial properties for the chassis and wheels
- reusable material and inertial macros in `urdf/common/`

The dimensions are conservative placeholders until hardware is selected. Sensor frames will be added in a later focused milestone.

## Frame and joint names

Physical links:

- `base_link`
- `front_left_wheel_link`
- `front_right_wheel_link`
- `rear_left_wheel_link`
- `rear_right_wheel_link`

Wheel joints:

- `front_left_wheel_joint`
- `front_right_wheel_joint`
- `rear_left_wheel_joint`
- `rear_right_wheel_joint`

The wheel joints rotate around their local `Y` axis. Left and right wheels use the same positive joint axis so controller sign conventions can be handled explicitly in the control layer.

## Verify on the Ubuntu VM

From the active VM clone:

```bash
cd ~/MoonCar/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mooncar_description
source install/setup.bash
ros2 run xacro xacro src/mooncar_description/urdf/mooncar.urdf.xacro > /tmp/mooncar.urdf
```

The Xacro command should finish without missing include or package lookup errors.
