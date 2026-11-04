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

## R5 proposal model

`urdf/mooncar_r5.urdf.xacro` is a separate proposed-hardware model. The legacy
`mooncar.urdf.xacro` remains the model for existing benchmark runs.

The 2026-09-08 procurement proposal supplies a bare R5 size of approximately
466 × 381.1 × 152 mm, 152 mm tires, and a P7120S battery of 177 × 146 × 78 mm
and 3.05 kg. The R5 model uses the stated tire radius (0.076 m), a separate
battery body, and nominal RGB plus depth camera sensors for the proposed
OAK-D-Lite. The camera optical origins are coincident placeholders, not an
OAK-D-Lite calibration.

The following are **simulation estimates**, not supplier measurements:

| Item | Current R5 model estimate |
| --- | ---: |
| Wheel center separation, front to rear | 0.314 m |
| Wheel center separation, left to right | 0.3311 m |
| Wheel width | 0.05 m |
| Central chassis box | 0.32 × 0.25 × 0.08 m |
| Central chassis ground clearance | 0.06 m |
| Provisional bare chassis mass split | 4.1 kg body + 4 × 0.4 kg wheels |
| Battery mount center, relative to `base_link` | (-0.035, 0, 0.088) m |

The wheel positions place the nominal tire collision envelope at 0.466 ×
0.3811 m. They do not establish the actual R5 wheelbase or track. The assembled
model mass is 9.186 kg before unmodeled controllers, wiring, brackets and four
TFmini Plus modules. The actual assembled mass, center of gravity, mounts,
suspension, wheel friction and effective skid-steer odometry width still need
supplier drawings or physical measurement. The model does not claim hardware
safety or terrain performance acceptance.

The research pan-tilt camera joints remain available in this variant; the
2026-09-08 purchase list has no pan-tilt actuator, so these joints do not
represent purchased hardware. The four TFmini Plus units are also not placed
until bracket locations are decided.


### R5 suspension research candidate (2026-10-12)

The R5 Xacro accepts `suspension_profile:=independent_linear_v1`; the default
`rigid` preserves the previous expanded model byte for byte. This candidate adds
four passive body-Z prismatic joints while preserving the four wheel drive joint
names and tire collisions. It does not add position-holding control.

The provisional travel is +/-20 mm, spring reference -10 mm, and damping ratio
0.7. Each original 0.4 kg unsprung assembly is split into a 0.05 kg carrier and a
0.35 kg rotating wheel with proportionally split inertia. At neutral travel the
assembled mass, COM and inertia remain unchanged. Carriers have no extra contact
geometry; this is a linear suspension approximation, not measured R5 kinematics.

The frozen 9.186 kg model gives sprung mass 7.586 kg. Its static sprung loads are
2.9245146736 N per front wheel and 3.2201453264 N per rear wheel under 1.62 m/s^2.
Using 10 mm nominal compression gives front/rear stiffness 292.4514674/322.0145326
N/m and damping 32.1680202/35.4197915 N*s/m. These defaults are specific to that
mass/mount configuration: the experiment independently re-derives and verifies
them, and must reject a model whose derived values differ. Hardware measurements
must replace these assumptions before claiming real-rover fidelity.

Build and verification use the independent suspension fixture and native DART
spring/damper dynamics. Production turn defaults and release approval are
unchanged. See `reports/R5_SUSPENSION_20261012.md` for actual validation status.

The 2026-10-13 revalidation explains the approximately 1 mm static offset from
native implicit spring/damping and contact velocity corrections. The latest
independent 12-trial diagnostic batch passes, but the new 17-round campaign
stopped on independent witness-writer drain failure and has no qualified rounds.
This remains an uncalibrated research model, with production default `rigid`.
See `reports/R5_SUSPENSION_REVALIDATION_20261013.md` for the evidence and failures.

### R5 sensor reference configuration (2026-10-01)

| Item | Updated model value | Source / interpretation |
| --- | --- | --- |
| S3 body envelope / mass | 55.6 × 55.6 × 41.3 mm / 115 g | [SLAMTEC](https://www.slamtec.com/en/s3/spec); simplified cylindrical body, mounting hardware separate |
| S3 scan density | 3200 rays at 10 Hz; 0.1125° per ray | Nominal 32 kHz / 10 Hz; uniform geometric scan, not the hardware's timing/distortion model |
| S3 range | 0.05–15 m | Manufacturer's 10% target-reflectivity reference; `s3_max_range` can select another documented range. Gazebo does not simulate reflectivity-dependent detection |
| OAK-D-Lite body / mass | X(thickness)=17.5, Y(width)=91, Z(height)=28 mm / 61 g | [Luxonis](https://checkout.luxonis.com/products/oak-d-lite-1); visual body, collision and analytic inertia share dimensions |
| OAK depth | 640 × 480, nominal horizontal FOV 73°, near 0.35 m, far 8 m, 10 Hz | Non-extended 480P reference; 8 m is the retained application cap within the manufacturer's ideal range |
| OAK RGB | 640 × 480, nominal horizontal FOV 69°, 10 Hz | Ideal uncropped 4:3 pinhole approximation; selected output budget, not maximum sensor resolution or factory intrinsics |
| BNO055 SEN0374 PCB | X=19, Y=21, substrate Z=1 mm | [DFRobot fabrication drawing](https://dfimg.dfrobot.com/wiki/21796/SEN0374_bno055-intelligent-9-axis-sensor_schematics_v1.pdf), PDF pp.2/13/14. PCB-only visual; populated height, mass and noise remain provisional |

Depth is still an ideal single-camera depth render, not a binocular disparity simulation.
The 75 mm stereo lens spacing is shown in the simplified visual only; it does not
provide the RGB-to-depth transform. The RGB/depth optical origins still need factory
calibration and actual mount measurements. The S3's ±30 mm accuracy is not a Gaussian
standard deviation; the existing simulated noise has not been calibrated.

Evidence, before/after expanded models, formula/source table and one short ROS-bridge
sensor capture: `reports/evidence/terrain-v2/20261001T212454+0900/`.

Remaining parameters have stable `TODO(hw:...)` markers beside the model inputs.
The [hardware parameter checklist](../../../docs/hardware_parameter_gaps.md)
records current placeholders, available manufacturer data, and what closes each
gap. The 2026-10-02 PCB update changes only the visual substrate: the 35 mm total
IMU collision envelope and 80 g mass are still estimates. It does not calibrate
the IMU or change the assembled mass.

Inspect the new body without replacing the legacy model:

```bash
ros2 launch mooncar_description display.launch.py \
  model_file:=mooncar_r5.urdf.xacro rviz_config_file:=mooncar_r5.rviz
```

## Frame and joint names

Physical links:

- `base_link`
- `front_left_wheel_link`
- `front_right_wheel_link`
- `rear_left_wheel_link`
- `rear_right_wheel_link`
- `lidar_link`
- `lidar_scan_frame` (scan plane above the visual housing)
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
