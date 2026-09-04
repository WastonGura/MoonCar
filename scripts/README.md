# Scripts

## M4 GUI acceptance

Run the complete build, Gazebo/RViz launch, drive check, and mapping check from
the Ubuntu VM with one command:

```bash
~/MoonCar/scripts/accept_m4.sh
```

After the checks pass, inspect the GUI and press Enter in the terminal to stop
the launched processes. The default map is saved as
`~/mooncar_m4_acceptance.yaml` and `~/mooncar_m4_acceptance.pgm`.

Pass a different map prefix as the only argument when needed:

```bash
~/MoonCar/scripts/accept_m4.sh "$HOME/maps/test_run"
```

## M5 navigation acceptance

Build and run the complete Gazebo, localization, Nav2, RViz, and obstacle-goal
workflow with:

```bash
~/MoonCar/scripts/accept_m5.sh
```

The rover starts at `(0.5, 0.0)` and navigates to `(3.5, 3.0)`, routing around
rock marker A. A passing run ends with `M5 acceptance passed.` and leaves the
GUI open until Enter is pressed. The launch log is
`/tmp/mooncar_m5_acceptance.log`.

The navigation deadline is 180 seconds of simulation time. GUI rendering can
make the VMware simulation slower than real time; progress prints ROS elapsed
time, wall elapsed time, and odometry every 15 seconds. A 600-second wall limit
and a 30-second stopped-clock watchdog still cancel stalled runs.

Run without Gazebo or RViz windows, or select another saved map:

```bash
~/MoonCar/scripts/accept_m5.sh --server-only
~/MoonCar/scripts/accept_m5.sh --server-only "$HOME/maps/site.yaml"
```

During the run, `/map`, `/mooncar/scan`, `/odometry/filtered`, `/cmd_vel`, and
`/clock` are available, and `/navigate_to_pose` accepts the goal. `map_server`,
`amcl`, `controller_server`, `planner_server`, `behavior_server`, and
`bt_navigator` must all reach the `active` lifecycle state. Any failed check
returns nonzero and stops the launched process group.
