# mooncar_gazebo

Gazebo simulation package for the MoonCar ROS2 stack.

## Worlds

- `worlds/lunar_yard.sdf`: lightweight lunar-like yard with low gravity, a stable spawn area, a gentle ramp, and small rock markers for later mobility tests.

## Launch

Start the M2 simulation stack:

```bash
ros2 launch mooncar_gazebo simulation.launch.py
```

For a server-only VM smoke test:

```bash
ros2 launch mooncar_gazebo simulation.launch.py gz_args:="-r -s -v 3"
```

The launch starts Gazebo, publishes the MoonCar description, spawns the rover into the `lunar_yard` world, and bridges `/clock` into ROS2.
