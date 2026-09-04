# Lunar Yard Acceptance Map

`lunar_yard.pgm` is a SLAM Toolbox observation map, saved on 2026-09-05
using runtime `9b01742` and `scripts/accept_m4.sh --server-only` in the
Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic VM. The output image was renamed
without changing pixels. The YAML retains the saved resolution and origin.

The 300 x 202 grid has 0.05 m cells (15 x 10.1 m bounds). Gray cells remain
unobserved, not certified traversable. This is not a complete survey of the
50 x 50 m world. It replaces the approximate full-footprint drawing that did
not match laser-height cross sections and biased AMCL localization.

The recording used `/mooncar/scan` after the rover-body box filter, the
nominal directional wheel friction, and the M4 scripted drive. Regenerate
after changing world geometry or sensor transforms; validate goal error
against Gazebo truth before replacing this fixed regression fixture.
