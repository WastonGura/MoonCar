# MoonCar

MoonCar is a ROS2-based four-wheel rover prototype for terrain surveying in simulated lunar-like environments, with later migration to physical hardware. Its main output is terrain information with measured coverage and geometric accuracy. Mapping, observation planning, navigation, and obstacle avoidance work together to support that survey task.

## Current Focus

Latest (2026-11-04): the [matched fixed-view/gimbal observation trial](reports/CAMERA_MODE_COMPARISON_20261104.md) completed both six-action V2 ramp tasks. Gimbal observation held the body within 1mm in this simulation and shortened the formal mission from54.66 to48.34 simulated seconds; body-spin observation displaced the body by0.673m. Fixed view retained more valid fitted area and lower measured localization error in this single pair, with pre-observation differences already present. Defaults remain unchanged; this is a controlled observation-action comparison, not full autonomous-policy or hardware acceptance. Shutdown receipt and I/O limitations are recorded in the report. Earlier entries below retain their historical version scope.

The [follow-up cause audit](reports/FIELD_SLOPE_FAILURE_CAUSES_20261102.md) reproduced both stop paths without product edits or new worlds: dispatch-time transient localization rejection escapes mission recovery, and one spatial-support failure discards otherwise tracked identities, leaving no historical reassociation path. The ramp frontend diagnostics matched all676 recorded frames. Both fixes remain unimplemented in this audit.

The requested [full-field coverage and V2 slope tests](reports/FIELD_COVERAGE_AND_SLOPE_20261102.md) have now each run once and both remain unqualified. Coverage-priority exploration stopped on navigation/turn readiness after372 simulated seconds, observing224.31/900m² (24.92%). The separate shipped-ramp trial lost track support at an actual3.47° body tilt before reaching its main slope, turns or descent. Both failures and raw evidence are preserved; product code and safety gates were unchanged. The earlier successful bounded mission below retains its narrower scope.

The [final-build V2 rerun](reports/WINDOW_FINAL_RECHECK_V2_20261102.md) ended normally on its task budget:575.9 simulation seconds from a600-s budget,29 completed actions and37.58m of travel. All228 brief localization waits recovered; persistent track loss and the health-snapshot exit did not recur. XY error P95 was6.81cm (max7.09cm). Pauses remain frequent; complete field coverage, steeper slopes and hardware remain separate validation work. The source and all17 installed window modules were checked, and the final backend import was retained in the actual server process. Earlier entries below preserve their prior result/version scope.

The [shared-track continuity repair](reports/WINDOW_TRACK_CONTINUITY_REPAIR_20261102.md) now refreshes keyframes when overlap declines. Both failed recorded tails recover offline, and the correctly installed candidate had no persistent support loss over a294.4-s mission/17.45m. That mission stopped on a separate health-snapshot race, now fixed with captured-condition replay and regressions (77 relevant checks passed). The final race fix has not repeated the full600-s mission; frequent brief pauses, complete field coverage and slope traversal remain open. A first online attempt accidentally loaded the old installed package and is preserved separately, not counted as candidate evidence.

The next-stage [region/endurance/slope validation](reports/WINDOW_EXPANSION_VALIDATION_20261102.md) has now begun. Both the full-region natural attempt and the separate slope approach stopped safely after persistent shared-track support loss. Full-field, long-duration and slope traversal remain unqualified; the slope route was not reached. Hardware is not yet complete, so [hardware work is preparation only](docs/window_field_validation.md). The successful local integration below remains a scoped result.

2026-11-02: the default workbench backend is now **`rgbd_window`**. The multi-keyframe/window estimator publishes the actual navigation pose, TF and health; the old RGBD producer is not started and active revisit is disabled for this mode. One natural V2 mission completed 12 actions and11.82m of travel, with3 brief localization pauses recovered and normal task-budget completion. A shutdown-recording issue was fixed and checked separately after the live capture. See [current configuration](docs/window_localization.md) and [integration/validation report](reports/WINDOW_NAVIGATION_V2_20261102.md). Earlier paragraphs below describe historical sidecar trials, not the current default.

The [fixed-input full-chain AB/BA comparison](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/FULL_CHAIN_AB_20261023.md) replayed the same complete sensor stream and deadlines through the versions before/after the latest IMU change, with identical resources and real JSON-flush completion measurement. Each of four runs produced 489/491 planned outputs, lost 3/492 raw pairs, and had zero successful outputs over 0.5 seconds. Paired P95 values were 130.5→132.7ms and 130.5→125.2ms: no predeclared overall improvement, and no reproduction of the earlier large live regression. Same-stamp position/covariance results were identical. This isolated replay excludes Gazebo/default concurrency and does not complete live acceptance. No product sources changed; reports below retain their historical scope.

The latest [IMU reception fix and V2 trial](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/SHADOW_IMU_RECEPTION_V2_20261023.md) changes only the passive sidecar's IMU subscription to reliable KEEP_LAST200 and adds bounded timing/readiness diagnostics. All 26 targeted checks passed. The single trial had no IMU-bracket timeout and shortened the maximum delivery gap from 0.646 to 0.327 simulation seconds, but overall discards rose to 5.78%, wall P95 to 0.518 seconds, and late successful outputs to 27. Real-time validation remains incomplete; the visual frontend was the largest internal stage in 25 of those 27 outliers. Default localization, revisit, estimator math and resources remain unchanged. Reports below retain their historical scope.

The [remaining-tail audit and V2 trial](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/SHADOW_TAIL_V2_20261023.md) traced the previous 16 replacements and three late outputs, then removed duplicate keyframe assembly/factorization through an explicit one-use posterior handoff. All 55 checks and 126 paired numerical/state comparisons passed; paired keyframe P95 fell 25.42% offline. The sole live trial did not improve overall availability: discards rose 3.32%→4.76%, wall P95 rose 0.377→0.415 seconds, and successful wall-late outputs rose 3→7. The remaining drops/late outputs are unresolved. Default localization, revisit, vision frontend, resource policy and hardware scope remain unchanged.

The [vision sampling optimization and V2 trial](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/VISION_FRONTEND_V2_20261023.md) reuses per-feature interpolation weights across the existing 21×21 image patch. All 26 vision checks passed and all 126 recorded outputs/features remained bitwise equal; paired frontend P95 fell 32.18% and whole-process total wall time fell 23.60%. The single live motion trial reduced discards from 15.42% to 3.32% and end-to-end P95 from 0.606 to 0.377 seconds, meeting the 0.5-second P95 reference. Remaining drops and wall-time outliers prevent a fully continuous real-time claim. Default localization, revisit, resources and hardware scope are unchanged; older reports below retain their original scope.

The [cache-enabled V2 shadow trial](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/CACHED_QUERY_SHADOW_V2_20261023.md) completed translation and turning with the offline cache explicitly enabled. All 39 adapter/cache checks passed. Compared with the previous trial, paired discards fell from 24.95% to 15.42% and end-to-end wall P95 fell from 0.914 to 0.606 seconds, but only 394/467 plans produced outputs and wall P95 still exceeds the 0.5-second reference. Online timing and continuous availability remain unqualified. Default localization and revisit behavior are unchanged; the following reports retain their earlier scope.

The [ordinary-frame query follow-up](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/QUERY_FRAME_OFFLINE_20261023.md) batches input validation and residual calculation while preserving the solver and cache construction. On the same frozen clip, paired ordinary-query total wall time fell 26.22% and P95 fell 14.83%; the latter missed the predefined 20% improvement gate. All 126 results stayed numerically equivalent and 17 checks passed. The original legacy-relative offline speed gates now pass, but online timing remains unqualified. No workbench, default-localization, or revisit change was made.

The [cache-construction follow-up](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/QUERY_CACHE_BUILD_OFFLINE_20261023.md) removed redundant whole-estimator deep copying. On the exact same frozen clip, paired cache-build P95 fell 37.40% and all-plan backend P95 fell 17.05%, with ordinary-query timing effectively unchanged. All 126 results remained numerically equivalent and 12 checks passed. The construction-specific gates passed; the original overall query-speed gate and online qualification remain open. Default localization and revisit behavior are unchanged.

The [2026-10-23 offline cached-query prototype](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/QUERY_CACHE_OFFLINE_20261023.md) preserved positions and full covariance on 126 recorded exposures. Three paired timings reduced ordinary-query P95 by a median 29.7447% and total backend wall time by 23.017%, including cache construction. The query target was 30%, and keyframe/cache tail latency increased, so the result remains correct but below the full performance gate. No online/default/revisit change or new V2 run was made.

The [latest V2 throughput trial](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/SHADOW_THROUGHPUT_V2_20261022.md) completed translation and turning after an equivalent feature-spacing optimization and an explicit background scheduling trial. Paired exposure discards fell from 37.33% to 24.95%, but end-to-end wall latency P95 rose from 0.609 to 0.914 seconds while simulation arrivals slowed. Real-time continuity remains unqualified; the default localization and revisit flow are unchanged. The older reports below retain their original scope.

The [2026-10-22 localization development log](D:/Documents/Projects/Repositories/MoonCar-mapping/docs/work_logs/2026-10-22-rgbd-window-localization.md) records the progression from reference/revisit repairs to the independent RGBD sliding-window prototype, its failed tracking attempts, the successful same-video comparison, and the first independent live adapter. The [single V2 shadow trial](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/RGBD_WINDOW_SHADOW_V2_20261022.md) did not qualify online timing or complete its predefined motion route. The [latency repair and second V2 shadow trial](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/RGBD_WINDOW_LATENCY_V2_20261022.md) reduced observed latency and output gaps but still did not qualify real-time performance; the default backend became stale before the body motion route began. The [default RGBD timeout repair and motion trial](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/DEFAULT_RGBD_TIMEOUT_V2_20261022.md) completed the guarded V2 route using startup CPU separation with zero observed default before/after-compute timeouts; the shadow now has moving/turning error samples but still lacks continuous online availability. The [shadow continuity repair](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/SHADOW_CONTINUITY_V2_20261022.md) fixed premature plan expiration and completed one moving/turning V2 trial: observed maximum delivery gap fell from 2.051 to 0.454 simulation seconds, but drop rate and observation freshness still prevent default use. The default backend is unchanged; earlier milestones below retain their original validation scope.

The project is currently simulation-first:

1. Build and validate the ROS2/Gazebo simulation pipeline.
2. Implement SLAM and Nav2 navigation in simulated lunar terrain.
3. Add terrain-aware traversability analysis.
4. Migrate stable ROS2 components to Raspberry Pi hardware after procurement.

The [simulation validation scope](docs/simulation_validation_scope.md) ties accuracy targets and important parameters to these uses. Model optimization is paused; the existing baseline is retained and the contact-integral research candidate remains frozen.

The [terrain-survey software backlog](docs/terrain_survey_software_backlog.md) records existing capabilities, software gaps, and the next hardware-independent priority: a reproducible offline survey task and results pipeline.

The [offline survey workflow](docs/offline_survey.md) now provides task validation, archive/folder replay, baseline map reconstruction, quality reports, exports, and read-only configuration checks through `scripts/offline_survey.py`.

The [live survey workflow](docs/live_survey.md) connects ordered observation uploads to the existing workbench, with incremental maps, task quality, stale-data status, recoverable queues, and downloadable result packages. The ROS recorder's uplink is opt-in; hardware and wireless field acceptance remain pending.

Current [mapping evaluation v1.2](docs/mapping_acceptance.md) treats coverage as descriptive data, with no minimum coverage gate. Geometry precision, missing data, reference evidence, and runtime faults remain visible; archived v1/v1.1 results retain their original policy.

The [fresh Gazebo integration report](reports/LIVE_GAZEBO_SURVEY_VALIDATION_20261019.md) records live capture, turn/pause/disconnection recovery, and terrain-truth comparison. The sequence completed, with localization timing, incomplete coverage, and shutdown issues retained; it is not an all-gates-pass claim.

Online terrain surveys and the workbench now default to [local metric localization](docs/local_metric_localization.md), using fresh laser measurements and IMU heading. The [default-switch record](reports/LOCALIZATION_DEFAULT_20261020.md) covers startup and explicit rollback; the [repair report](reports/LOCALIZATION_REPAIR_20261020.md) records the fixed-reference validation scope. Offline AMCL and the standalone M4 mapping entry remain separate.

The workbench also offers [Terrain V2](docs/workbench_terrain_v2.md): the original 30×30 m survey terrain with four explicit physical localization markers for the current rover's horizontal laser. Scene selection, spawn checks and recorded provenance distinguish this setup from the original 50×50 m survey scene.

## Repository Layout

```text
docs/          Project documents, plans, reports, and meeting notes
ros2_ws/       ROS2 workspace, added after structure confirmation
firmware/      Portable STM32 control core and host tests; board HAL is hardware-gated
scripts/       Build, environment, and deployment helper scripts
assets/        Images, models, and presentation assets
reports/       Innovation-project report and defense materials
```

See `docs/architecture.md` and `docs/simulation_plan.md` for the current technical plan.

## One-command Acceptance

On Ubuntu 24.04 with ROS 2 Jazzy and the project dependencies installed:

```bash
~/MoonCar/scripts/accept_m6.sh
```

Use `--rviz-only` for navigation with RViz but no Gazebo 3D window, or
`--server-only` to omit both windows. No flag retains the full GUI. These modes
also work with `accept_m5.sh`. The script builds the workspace,
runs software/UART regression checks, tests terrain keepout and a newly introduced
obstacle, then drives two goals. Exit status is nonzero on failure. Close any previous
MoonCar simulation before running; press Enter after a GUI run to close its processes.
SSH runs without a display need Ubuntu packages `xvfb` and `xauth`. The scripts
start a private virtual display and clean it up automatically. VMware defaults
to software-rendered Ogre for both sensor data and the GUI; a visible scene alone
does not prove that the virtual GPU produces valid laser/depth measurements.
Acceptance uses Jazzy's `rmw_fastrtps_cpp` by default (resolve workspace
dependencies with rosdep).
The simulation uses `topic_tools` to cap wheel-state delivery at 30 Hz and clock
notifications at 100 Hz of simulation time. Raw samples remain on
`/mooncar/joint_states_raw` and `/mooncar/clock_raw`. This does not change
commanded speed, sensor resolution, or the physics step.
Use the same `RMW_IMPLEMENTATION` in additional ROS terminals. An explicit
environment override is preserved.
Evidence is written to `~/mooncar_m6_acceptance.json` and
`/tmp/mooncar_m6_acceptance.log`. A JSON result is created only on success.

M4 mapping remains available through `scripts/accept_m4.sh`; M5 navigation through
`scripts/accept_m5.sh`. Software-only checks: `scripts/check_software.sh`.

See [hardware handoff](docs/hardware_handoff.md) and [UART v1](docs/uart_protocol.md)
before connecting real motors. The hardware launch is a disarmed bench profile,
not an already validated autonomous real-rover deployment.

The [M6 delivery report](reports/delivery_2026-09-05.md) includes actual VM
acceptance results, screenshots, known limits and the five hardware-gated M7 issues.
