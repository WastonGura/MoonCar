# mooncar_description

Robot description package for the MoonCar rover simulation model.

## Model entry point

The top-level model is:

```text
urdf/mooncar.urdf.xacro
```

It currently defines a minimal, valid rover root structure with:

- `base_footprint`
- `base_link`
- placeholder chassis dimensions and mass properties
- reusable material and inertial macros in `urdf/common/`

The placeholder chassis is intentionally small in scope. Four-wheel skid-steer links, joints, and sensor frames will be added in the next focused milestones.

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
