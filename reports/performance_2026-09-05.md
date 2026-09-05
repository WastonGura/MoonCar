# Simulation performance follow-up

Issue #58 / PR #59. Runtime code: `00bdcf3`, following baseline `14b88f5`.
Ubuntu 24.04.4, ROS Jazzy, Gazebo 8.11.0, VMware 4 vCPU / 8 GB,
CPU-rendered Ogre and Fast DDS. Windows settings were not changed.

## Changes

- Forward wheel states at up to 30 Hz of simulation time using native
  `topic_tools`; retain raw data on `/mooncar/joint_states_raw`.
- Forward the simulation clock at up to 100 Hz instead of broadcasting every
  1 ms physics step to all ROS nodes. The clock throttle reads
  `/mooncar/clock_raw`, not its own output. Original timestamps are preserved.
- Add `--rviz-only` to M5/M6 acceptance. No flag still opens both windows;
  `--server-only` opens neither. Sensors continue rendering in every mode.
- Allow 200 ms rather than 20 ms for Nav2 action acknowledgement. Intermediate
  runs repeatedly failed on acknowledgement, not the navigation goal deadline.
  Goal deadlines, collision checks, the 0.4 m truth-error gate and the safety
  relay's command timeout are unchanged.

No changes to commanded speed, physics timestep, sensor resolution, map,
friction, localization tuning or hardware control were made.

## Measurements

RTF means elapsed simulation time divided by elapsed real time. A value of
0.32 still means approximately three times slower than real time.

| Configuration | Samples | Window RTF | Median reported RTF |
| --- | ---: | ---: | ---: |
| Original full GUI | 100 | 0.235 | 0.185 |
| Wheel throttle only, full GUI | 100 | 0.203 | 0.150 |
| Wheel throttle only, RViz only | 200 | 0.260 | 0.191 |
| Final clock/wheel throttles, RViz only | 200 | 0.325 | 0.252 |
| Final clock/wheel throttles, full GUI | 200 | 0.295 | 0.211 |

These are short 10-20 second windows, not a controlled statistical benchmark:
the first two were stationary, final windows overlapped acceptance motion, and
host scheduling varied. They show useful improvement, not a guaranteed speedup
or real-time capability. Wheel throttling alone was insufficient. The earlier
single-render-thread experiment was rejected because it performed worse.

Final RViz-only run: wheel states 24.75 Hz in simulation time; a separate
10-second clock probe measured 93.45 Hz. Caps are not exact target rates because
delivery and clock sampling are discrete. `ros2 topic hz` instead measures
wall-time arrivals and will show lower rates in slow simulation.

## Verification

- Eight packages built; 19 Python tests and the C11/UBSan regression passed.
- RViz-only M6 passed live lidar/depth, keepout detour, introduced-obstacle
  replanning and both navigation goals. Truth errors: 0.07385 / 0.07392 m.
- A visible RViz window showed the map and rover; no Gazebo GUI process/window
  was present in RViz-only mode.
- Clock pause held at 84.868 s; resume advanced to 85.958 s; a time-only reset
  resumed at 1.631 s. This verifies clock forwarding, not navigation-state reset
  support. Restart the acceptance script for a fresh mission.
- Full-GUI M6 passed the same gates, with truth errors 0.07047 / 0.06690 m;
  the Gazebo screenshot confirms a rendered scene. Production deployment is
  verified separately before closing Issue #58.

Raw sampled stats, first/last-window summaries, mission JSON and screenshots
are in [evidence/performance58](evidence/performance58/).

## Use

```bash
~/MoonCar/scripts/accept_m6.sh --rviz-only
```

Close existing MoonCar simulations first. The script builds, tests and drives
the two goals automatically. Press Enter after success to stop its processes.
Use no flag when the 3D rover view is needed, or `--server-only` for unattended
checks. Resolve the new `topic_tools` dependency with the documented rosdep step
on other machines; it is installed on the development VM.

This improves a CPU-rendered VM; it does not establish real-vehicle speed or
outdoor reliability. Faster rendering hardware/native Linux is a separate
environment change and has not been verified here.

## Sources

- [Gazebo 8.11 joint-state publisher](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.11.0/src/systems/joint_state_publisher/JointStatePublisher.cc): the installed version does not consume `update_rate`.
- [Native Jazzy throttle](https://github.com/ros-tooling/topic_tools/blob/jazzy/topic_tools/src/throttle_node.cpp): simulation-clock rate control and backward-jump handling.
- [Nav2 action acknowledgement](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_behavior_tree/include/nav2_behavior_tree/bt_action_node.hpp): server-response timeout is separate from goal execution.
