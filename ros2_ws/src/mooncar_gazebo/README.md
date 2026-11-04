# mooncar_gazebo

Gazebo simulation package for the MoonCar ROS2 stack.

## Worlds

- `worlds/lunar_yard.sdf`: 50 m lunar-like yard with low gravity, a stable spawn area, a gentle ramp, a shallow crater, a ridge, and scattered rocks for mobility and mapping tests.

## Launch

The accepted, uncalibrated support candidate has a separate explicit entry:

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export MOONCAR_DART_PLUGIN_ROOT=/opt/mooncar/physics/dart-7.8.0
export MOONCAR_IMAGE_DDS_RUNTIME=/opt/mooncar/image-dds/rmw-8.4.4
export MOONCAR_SUPPORT_RUNTIME=/opt/mooncar/turn/r5-v2
ros2 launch mooncar_gazebo simulation.launch.py simulation_profile:=r5_support_v1
```

Use the pinned runtime image and a freshly built overlay. This profile selects
R5, `independent_linear_v1`, `r5_precision_v1` (0.25 ms), the accepted fixed
axle allocation and pure-turn lateral compliance, gimbal D=0 and 30 Hz robot
TF. It checks the accepted runtime library hashes and records the actually
loaded libraries and generated world. `MOONCAR_SUPPORT_RECEIPT_ROOT` chooses
the receipt directory; otherwise it is printed in the launch log.

The workbench exposes the same choice under **仿真车辆 → R5 支撑候选**;
saved tasks preserve `simulation_profile`. Navigation launches also forward
this argument. Existing task files and ordinary launches keep `legacy`.
Selecting only `model_file:=mooncar_r5.urdf.xacro` still means the rigid R5
model, not the support candidate. This candidate does not enable adaptive
laser-feedback steering, change production defaults or cancel physical slope
slip. Its parameters remain uncalibrated and have not been validated on hardware.

The proposed R5 body is opt-in. Selecting its model automatically selects the
RGB bridge, 0.076 m odometry wheel radius and R5 body-return filter. Its
0.3311 m odometry track width is the estimated wheel-center separation; skid
steering requires later calibration. Existing runs keep the legacy defaults.

```bash
ros2 launch mooncar_gazebo simulation.launch.py \
  model_file:=mooncar_r5.urdf.xacro
```

`bridge_config_file`, `laser_filter_file`, `odom_wheel_radius` and
`odom_track_width` remain explicit overrides. The R5 filter uses an engineered
0.25 m by 0.22 m half-box around the current model; the old 0.43 m by 0.36 m
half-box could discard nearby external obstacles on the smaller rover.
This is a self-return filter, not a validated safe stopping distance.

This selects the new car model and exposes `/mooncar/rgb/image` and
`/mooncar/rgb/camera_info`. Existing Nav2 footprints and terrain planner
dimensions still describe the legacy car; R5 navigation and new benchmark
results require their own calibrated configuration.

Start the M2 simulation stack:

```bash
ros2 launch mooncar_gazebo simulation.launch.py
```

For a server-only VM smoke test:

```bash
ros2 launch mooncar_gazebo simulation.launch.py gz_args:="-r -s -v 3 --render-engine ogre"
```

The launch starts Gazebo, publishes the MoonCar description, spawns the rover into the `lunar_yard` world, and bridges `/clock` into ROS2.

Gazebo's per-step clock is bridged to `/mooncar/clock_raw`. A standard
`topic_tools` throttle forwards it to `/clock` at up to 100 Hz of simulation
time, so every ROS node need not process all 1,000 physics ticks per second.
The throttle reads the raw clock itself; pause and backward time jumps remain
simulation-time driven. The physical integration step stays at 1 ms.

The launch selects Ogre for both GUI and sensors. VMware defaults to CPU
software rendering because the virtual GPU previously produced invalid sensor
measurements. Server-only rendering still requires an X display; acceptance
scripts provide Xvfb when needed.

M3 control topics bridged by this launch:

- `/mooncar/cmd_vel_safe` from ROS2 to Gazebo `/cmd_vel`
- `/mooncar/joint_states_raw` from Gazebo, then `/joint_states` through the
  standard `topic_tools` throttle at up to 30 Hz of simulation time. Gazebo 8.11
  ignores the model's `JointStatePublisher/update_rate`; without the throttle,
  every physics iteration propagates through wheel odometry. Message timestamps
  and positions are preserved. `ros2 topic hz` measures wall-time arrival rate,
  so it reports less than 30 Hz when the simulation runs slower than real time.

M3 ROS control nodes started by this launch:

- `mooncar_cmd_vel_safety_relay`: clamps `/cmd_vel` and times out stale commands.
- `mooncar_wheel_odometry`: integrates `/joint_states` and publishes `/odom` plus `odom -> base_footprint` TF.

## M2 sensor topics

Gazebo sensor placeholders are attached to frames from `mooncar_description` and bridged into ROS2:

- `/mooncar/scan` from `lidar_link`
- `/mooncar/imu` from `imu_link`
- `/mooncar/depth/image` from `depth_camera_optical_frame`
- `/mooncar/depth/camera_info` from `depth_camera_optical_frame`

## Verify on the Ubuntu VM

From the active VM clone:

```bash
cd ~/MoonCar
git pull --ff-only origin main
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mooncar_description mooncar_gazebo
source install/setup.bash
```

Run a server-only smoke test:

```bash
ros2 launch mooncar_gazebo simulation.launch.py gz_args:="-r -s -v 3 --render-engine ogre"
```

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/MoonCar/ros2_ws
source install/setup.bash
ros2 topic list | grep -E '/clock|/mooncar'
ros2 topic echo --once /clock
```

For M3 drive control checks, see `mooncar_control/README.md`.

Expected early M2 behavior:

- Gazebo loads `lunar_yard.sdf`.
- `spawn_mooncar` reports entity creation successful.
- `/clock` is available in ROS2.
- Sensor topics are expected to appear after Gazebo sensor systems initialize; rendering-backed topics may require a graphical or headless-rendering capable VM session.

## Troubleshooting

- If Gazebo cannot find the world file, rebuild `mooncar_gazebo` and source `install/setup.bash`.
- If `/clock` is missing, confirm `mooncar_gazebo/config/ros_gz_bridge.yaml` is installed and that `mooncar_gz_bridge` started.
- If sensor topics are missing in server-only mode, rerun with GUI or a headless rendering setup before treating it as a model failure.
- If the rover does not spawn, confirm `/robot_description` is being published by `robot_state_publisher`.

## Follow-up dependencies

- M3 control should attach wheel control to the existing four wheel joints.
- M4 SLAM should use `/mooncar/scan` and `lidar_link`.
- Nav2 work should treat `base_footprint` as the planar robot base frame.
- Hardware selection should replace placeholder sensor rates, noise values, masses, and poses.

## Terrain v2 scenes (separate from the acceptance worlds)

### Material and geometry detail update — 2026-10-06

The three v2 worlds now use continuous world-coordinate surface atlases at
128 pixels/metre (2048² for ramp/curve, 3840² for survey), with 0.15 m Gaussian
visual blending between classes. Collision material partitions retain their
existing coefficients. Six deterministic convex rock variants and merged static
pebble geometry add actual silhouettes and depth returns. Each scene contains
46 / 99 / 31 small pebbles respectively, with 3–10 cm diameters. The measured
patches plus 0.5 m margins, origin safety area and straight test corridor exclude
new pebbles. Ground collision vertices and measured source data are preserved.

This is reproducible engineering scene detail, not measured lunar material or
rock-distribution calibration. Visual and collision geometry share the same
mesh. Shader normals do not add geometric depth. New shadows use the existing
sun direction/intensity and ambient light 0.22 / 0.22 / 0.24.

Generate into a fresh package resource directory before replacing assets:

```bash
python3 ros2_ws/src/mooncar_gazebo/scripts/generate_terrain_materials.py \
  --out /tmp/mooncar-v2-stage/mooncar_gazebo/materials
python3 ros2_ws/src/mooncar_gazebo/scripts/build_terrain_v2.py \
  --output-root /tmp/mooncar-v2-stage/mooncar_gazebo
python3 scripts/check_terrain_v2_assets.py /tmp/mooncar-v2-stage/mooncar_gazebo
```

Both generators use seed 20261006 by default. NumPy, SciPy and Pillow are
required. Measured NPZ inputs are read from the source package even when an
output root is supplied. Baseline checks accept `--baseline <old-package-root>`.
The compatibility `boulder.obj` alias names variant zero; new worlds explicitly
reference their selected variants. Manifest additions describe atlas settings,
variant hashes, generator hashes and every pebble placement.

Current validation status is **partial**: resource/geometry checks, deterministic
generation, nine actual Gazebo views and paired depth readback passed; the
three R5 straight-drive smoke trials failed because the body flips and moves
abnormally even while parked. A zero-command baseline test on the old ramp
world reproduces about 4.83 m of drift in 5 simulation seconds; removing new
pebbles also reproduces it. The cause is not yet established. Do not treat this
as a passed vehicle/scene integration test. Full evidence is under
`reports/evidence/terrain-v2/improvement-20261006/` at repository root.

`worlds/mooncar_{ramp,curve,survey}_v2.sdf` are opt-in worlds for the next scene-development stage. They do not replace `lunar_yard.sdf` or the worlds and data used by the earlier strategy comparisons.

| World | Terrain feature | Surface classes |
| --- | --- | --- |
| `ramp` | Derived 0.596008 m ramp rise; center-route maximum grade 8.86° | Regolith, soft sand, gravel, rock |
| `curve` | Route between 6.1 m and 11.2 m craters | Regolith, gravel, crater floor |
| `survey` | One measured fresh-crater patch, one parameterized crater and mounds | All five classes |

![Terrain v2 top views and marked X–Z/Y–Z sections](docs/terrain_v2_three_views.png)

![Measured crater patch and two profiles](docs/ce4_measured_patch_detail.png)

The dashed lines in each plan view locate its two sections; profile vertical scales are expanded for legibility. The outlined 4.8 m square in `survey` is derived from an actual Chang'e-4 rover-camera DTM. The rest of the ground is a synthetic 2.5D height surface. None of these worlds is a complete reconstruction of a lunar landing site.
The shared top-view colour bar shows relative elevation in metres: dark grey is lower, neutral grey is near zero, and pale tan is higher. It uses one scale across all three scenes. These colours do not identify surface materials; the separate `class_mask.png` files carry simulation class labels.

### Terrain evidence and its limits

| Parameter | Basis | What is implemented |
| --- | --- | --- |
| `ramp` slope | [Ding et al., Nature Communications 2024](https://doi.org/10.1038/s41467-024-46233-8), p.3, reports a maximum 8.86° slope in one Yutu-2 route segment | Center-route mesh segments now reach 8.86°, including background relief; derived ramp rise is 0.596008 m over the retained 6 m layout. Maximum whole-surface gradient is 9.2459° because lateral taper also contributes. This is a reference-scale analogue, not a route reconstruction or a global slope cap |
| `curve` crater diameters | The same route study reports adjacent craters of 6.1 m and 11.2 m diameter | Two parameterized bowls at those diameters; their positions and exact profiles are engineered |
| Typical crater depth | [Wu et al., Earth and Planetary Science Letters 2021](https://doi.org/10.1016/j.epsl.2020.116666) reports mean crater depth/diameter 0.061 at the Chang'e-4 site | Retained for `curve`; `survey` now uses the separate degraded-crater observation below |
| `survey` synthetic crater | [Guo et al., Geophysical Research Letters 2021](https://doi.org/10.1029/2021GL094931), pp.3-4, describes region R3 with diameter ~4.5 m and depth/diameter ~0.07 | Replaces the manually selected 5.2 m diameter and generic 0.061 ratio. Nominal rim-to-floor depth is 0.315 m; profile and center (-6,7) remain engineered, separate from the measured fresh-crater patch |
| Fresh crater geometry | [Ding et al., Geophysical Research Letters 2020](https://doi.org/10.1029/2020GL087361) reports a roughly 2.2 m crater with 0.35 m rim-to-floor depth; [the authors' source DTM](https://doi.org/10.5281/zenodo.3732816) is 5 mm/pixel | A valid 4.8 m square is cropped without changing horizontal or vertical scale, low-pass filtered and sampled at 5 cm, detrended against its border plane, then feathered over 0.35 m into `survey` |
| Site rock abundance | Wu et al. fits a size-frequency model with abundance parameter `k = 0.21%` | Five resolved `survey` boulders are a deterministic sample, not 0.21% total-area coverage. At the 0.25 m diameter cutoff, Equation 6 predicts 0.04078% area coverage by larger rocks; their elliptical-footprint proxy is 0.03848%. Smaller rocks are absent from collision geometry, and exact shapes and positions remain engineered |
| One `curve` reference rock | Ding et al. 2024, p.7, reports the Longji target boulder height as approximately 0.41 m | `boulder_01` vertical scale is derived from the written collision mesh to place its top about 0.41 m above ground at its center. XY dimensions, shape, burial and position remain engineered; the measurement does not set every rock's height |

The two `curve` boulders sit near crater rims, a qualitative placement consistent with [Watkins et al.'s boulder-distance study](https://doi.org/10.1029/2019JE005963). That study concerns much larger craters, so it does not supply a quantitative distribution for these metre-scale bowls.

The [terrain parameter checklist](../../../docs/terrain_parameter_gaps.md) records
remaining geometry/contact assumptions beside stable `TODO(terrain:...)` source
markers. The 2026-10-02 reference-height change and independent collision readback
are archived in `reports/evidence/terrain-v2/20261002T025024+0900/`.

[Guo et al. (2021)](https://doi.org/10.1029/2021GL094931) report a median RMS height of about 4 mm in 125 mm windows and publish a wider 5 mm/pixel Chang'e-4 DTM. The original raster has a large central void and 69.4727% valid pixels. A fully valid 6 m square is now sampled at 5 cm and inserted at (6,-5.1) m in `survey`, replacing the local synthetic mound/background. The 0.35 m border blends to the coarse mesh; the 5.3 m interior uses the derivative alone. It retains original scale and regional slope, subtracting only a center-height datum. Placement, filtering and boundary blending are engineering choices. The rest of the world remains synthetic; 5 cm sampling does not establish the paper's millimeter roughness at a 125 mm window.

The new derivative is `data/terrain_sources/ce4_background_5cm.npz`. Recreate it with `scripts/prepare_ce4_background.py <path-to-CE4_Day_9_10_DEM_5mm.tif>` in an environment with SciPy, NumPy and Pillow. The source MD5 and crop indices are documented in the [source README](data/terrain_sources/README.md); geometry checks and the retained boundary failures are archived in `reports/evidence/terrain-v2/20261002T214221+0900/`. Fine collision meshes are `ce4_background_regolith_passable.obj` and `ce4_background_rubble_gravel.obj`. These names reflect engineered labels, not measured mineral properties. The old fresh-crater mesh is unchanged.

The source file `data/terrain_sources/CE-4_Fresh_Crater_DEM.mat` is reproduced under its [CC BY 4.0 Zenodo record](https://doi.org/10.5281/zenodo.3732816), credited to Chunyu Ding, Zhiyong Xiao, Bo Wu, Yuan Li, Nils C. Prieur, Yuzhen Cai, Yan Su and Jun Cui. Its published MD5 is `39a21d550aa9376c2497b26324bb76b9`. `scripts/prepare_ce4_crater.py` verifies that hash and creates `data/terrain_sources/ce4_fresh_crater_5cm.npz`; it needs SciPy only for this conversion. The source image's north-up orientation is retained. The valid crop uses array rows 400–1360 and columns 150–1110 (zero-based); the resulting patch and its transform are recorded in `survey/manifest.json`.

The five surface classes are **simulation labels**, not material identifications from the DTM. Their contact coefficients, procedural textures, soft-ground patches, mounds, and standalone rock positions have not been calibrated against wheel-soil tests. The lunar soil internal friction angle and cohesion inferred by Ding et al. cannot be inserted directly as Gazebo/Bullet contact friction coefficients. The measured patch captures geometry only; its surrounding 30 m field is not measured terrain.
[Ding et al. (2024)](https://doi.org/10.1038/s41467-024-46233-8) infer a Chang'e-4 soil internal friction angle of 21.5–42.0°, cohesion of 520–3154 Pa, and lateral wheel–soil external friction angle of 8.3–16.5°. [Hurrell et al. (2025)](https://doi.org/10.1007/s11214-025-01164-8) measure single-wheel traction coefficient as drawbar force divided by normal load across slip ratios in lunar simulant. These are useful calibration targets, but none directly specifies a rigid-contact SDF `mu` or `mu2`. The current five terrain pairs (`mu` 0.52–1.00, `mu2` 0.48–0.92) and the R5 wheel's directional coefficients (0.20/0.85) are uncalibrated simulator choices. Before claiming mobility fidelity, compare wheel traction–slip and sinkage against tests at the rover's wheel geometry and load, then freeze a fitted contact model or adopt a deformable-soil model.
The source DTM was reconstructed from rover imagery using shape from shading. The selected crop contains no `-100` no-data cells, but the original study notes uncertainty in shadowed and blurred areas; its heights are not direct laser measurements.

The generator is `scripts/build_terrain_v2.py`. It writes each class's OBJ surface into `models/terrain_v2/<scene>/` and uses the same OBJ for rendering and collision. The class meshes meet along boundaries, forming one continuous ground surface. The measured patch replaces a square of the coarse survey mesh with a 5 cm mesh; their boundary heights match. Separate faceted boulders have matching visual and collision meshes; their poses and sizes are in `manifest.json`. Each scene also includes `height_grid.npz`, a north-up `class_mask.png` with class IDs from `manifest.json`, and hashes for the world, meshes and textures. The survey height grid contains a 30 cm sample of the patch; the full 5 cm shape is in `ce4_crater_patch.obj`. The mask labels ground only; boulders are separate objects. These are top-down simulation labels, not camera semantic-segmentation images. `materials/textures/` contains procedural albedo, normal and roughness maps with a restrained neutral palette. These are not calibrated lunar reflectance data. Model training still needs domain variation and real imagery; colour must not be treated as a reliable class key.

The three v2 worlds now use project-local DART built against gz-physics 7.8.0,
with a 1 ms step. Bullet reproduced R5 flips at zero commanded speed, including
on a simple plate. This is a compatibility repair, not a vehicle or soil
calibration. R5 dimensions, mass, inertias, friction and scene geometry were
retained; v1 worlds keep their original backend and step. The
[R5 report](../../../reports/evidence/terrain-v2/r5-stability-20261006/README.md)
records six passed scene trials, gimbal feedback/timeout checks and the remaining
small gimbal oscillation. Earlier failed trials above remain historical evidence.

Build the pinned plugin using the [container recipe](../../../containers/README.md#r5-v2-物理后端).
The launch checks its manifest version and library SHA256, then resolves
collision mesh resources in a temporary world copy. Missing or mismatched
plugins stop launch before Gazebo starts; there is no fallback.
Run with the installed R5 model and RGB bridge, for example:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export MOONCAR_DART_PLUGIN_ROOT=/opt/mooncar/physics/dart-7.8.0
world=$(ros2 pkg prefix mooncar_gazebo)/share/mooncar_gazebo/worlds/mooncar_ramp_v2.sdf
ros2 launch mooncar_gazebo simulation.launch.py \
  world:="$world" world_name:=mooncar_ramp_v2 \
  gz_args:="-r -v 3 --render-engine ogre2" \
  model_file:=mooncar_r5.urdf.xacro bridge_config_file:=ros_gz_bridge_r5.yaml \
  spawn_z:=0.03 \
  odom_wheel_radius:=0.076 odom_track_width:=0.3311
```

Change both `ramp` strings to `curve` or `survey` for the other worlds. A headless server needs Xvfb and software rendering in the current VM setup. The generator requires Python with NumPy and Pillow; rerun it after changing terrain shapes or class assignments, then rebuild `mooncar_gazebo`. If regenerating the measured patch from the original `.mat`, run `scripts/prepare_ce4_crater.py` first with SciPy.

The friction coefficients in each SDF are **simulation hypotheses**, not measured lunar soil properties. DART imports the retained `<ode>` contact fields through the repaired mesh constructor; the four primitive/mesh and zero/nonzero tests check that entry, without calibrating its numbers. Deformable soil, wheel sinkage, material aging and photorealistic lunar imagery are outside this version. Scene labels and geometry are independent of the old 54-run and later 18-run results; new evaluations need their own frozen protocol and evidence folder.
