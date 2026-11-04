# Scoped R5/v2 steering runtime

The unreleased candidate combines ODE 0.16.2 coplanar cylinder contact repair,
pure-turn lateral compliance (10 / 0), experimental sensor-qualified front/rear
wheel allocation, and P=20 / I=0.1 / D=0 gimbal controllers. It is an engineering
simulation profile, not measured hardware calibration.

The former fixed allocation is retained for historical diagnostic replay only.
New candidate manifests require `independent_laser_feedback_v1` and the adaptive
actuator. It uses private, timestamped ROS feedback from filtered LaserScan and
IMU; it never treats wheel odometry as independent slip evidence. The nominal
flat-load ratio is an initial estimate, not an unconditional runtime multiplier.
Feedback expiry, invalid registration, duplicate/future packets and unavailable
scan geometry inhibit pure turns; straight/zero native commands are untouched.
The allocation remains in the previous 5% envelope. Feedback quality is an
engineering gate, not a real-sensor calibration. This new candidate is unapproved
until sensor observability and complete physical acceptance have passed.

The 2026-10-12 curve shadow test did not qualify this feedback: only 131 of 200
motion samples passed the controller's sensor gates, and post-run truth checking
also found errors in accepted registrations. No adaptive physical actuation was
run. Removing an unconditional fixed factor is not proof that terrain steering
has been repaired. See `reports/R5_ADAPTIVE_TURN_20261012.md`.

The actuator systems preserve the original command path and priority order.
They never consume body truth or measured contacts. CSV outputs are optional:
without an `output` element they perform no per-step file writes; with one they
retain the checked records and actual-pause flush receipts. Occupied evidence
paths are rejected rather than overwritten.

Build with `containers/package_r5_turn.py` in the original Jazzy image. Supply
the independently verified ODE artifact; the packager checks its exact hash,
compiles these systems against the installed Gazebo 8, and writes a manifest.
Build `containers/r5-turn.Dockerfile` using that package output as its context.
Do not add the runtime directory to a global LD_LIBRARY_PATH.

`MOONCAR_TURN_PROFILE=legacy|auto|r5_v2` controls selection. Auto is eligible only
for the R5 xacro and the three project DART v2 worlds. An explicitly requested
repair in another environment fails. The checked simulator supervisor verifies
the actual ODE, DART and actuator libraries before reporting ready. Dependencies,
hash mismatch, missing systems or readiness timeout terminate startup.

The repository's `turn_defaults.json` remains legacy until release evidence
passes. `turn_release.json` currently blocks activation: curve v2 still moved
171.735 mm on its nonplanar start area. Auto keeps legacy while this gate is
closed, and explicit `r5_v2` fails. The independent fixtures can inspect the
candidate; it is not a published three-scene fix.

Explicit legacy selection restores the original model/controller and
simulator path. Old models, v1 worlds, ROS topics and existing launch arguments
are preserved. Only the simulator child receives the repaired ODE search path;
the separately checked image bridge retains its own DDS configuration.
