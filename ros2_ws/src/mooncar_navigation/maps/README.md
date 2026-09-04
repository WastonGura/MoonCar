# Lunar Yard Acceptance Map

`lunar_yard.pgm` is a SLAM Toolbox observation map, saved on 2026-09-05
using runtime `a8c9513` and the M4 SLAM Toolbox launch in the
Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic VM. The output image was renamed
without changing pixels. The YAML retains the saved resolution and origin.

The 314 x 238 grid has 0.05 m cells (15.7 x 11.9 m bounds): 36898 free,
877 occupied, and 36957 unknown cells. Gray cells remain
unobserved, not certified traversable. This is not a complete survey of the
50 x 50 m world. It replaces the approximate full-footprint drawing that did
not match laser-height cross sections and biased AMCL localization.

The recording used `/mooncar/scan` after the rover-body box filter, the
nominal directional wheel friction, and a supervised known-corridor route:
`(0,0) -> (0,4) -> (6.2,4) -> (6.2,-2) -> (0,-2) -> (0,0)`.
The recording controller used SLAM TF, not Gazebo truth. At the last stop,
SLAM reported `(0.00058,-0.14855)` and Gazebo truth `(0.01588,-0.12743)`;
this is a single simulation closure check, not a real-world accuracy claim.
The world includes six laser-visible perimeter boulders. This fixture does
not establish localization on a featureless plain. Save with the existing
`mooncar_slam mapping_smoke_test.sh` after traversing the route. Regenerate
after changing world geometry or sensor transforms; validate goal error
against Gazebo truth before replacing this fixed regression fixture.
