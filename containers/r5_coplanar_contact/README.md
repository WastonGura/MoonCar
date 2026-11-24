# R5 coplanar cylinder contact candidate

This repairs an ODE 0.16.2 cylinder/trimesh manifold artifact on fully covered,
coplanar patches. It is an isolated test candidate; installed ODE, the accepted
gz-physics DART library, formal models and default profiles are unchanged.

The broadphase still supplies the actual triangles. The helper checks their
orientation and plane, computes the actual cylinder support generator, clips it
against each triangle, and requires the union of the covered intervals to span
the entire generator. It then emits the two true surface endpoints with the
plane normal and their own penetration depths. This removes artificial internal
triangle boundaries without replacing terrain with an infinite plane.

Boundary, gap, crease, opposite winding, cap, separated-endpoint, oversized
broadphase and custom triangle-callback cases keep the original collider.
The 1e-12 geometry tolerance is for double-precision geometric comparisons;
there is no change to mass, friction, compliance, controls or contact material.

Build in a fresh Jazzy test container with the matching ODE / system libccd:

```sh
python3 containers/build_ode_coplanar.py --output /new/candidate \
  --archive /archive/ode_0.16.2.orig.tar.gz
```

Required build tools include autoconf, automake, libtool, make, a C++ compiler,
and the installed libccd development package. Build uses the Ubuntu package's
autotools configuration and system libccd. It configures in source to avoid
ODE's VPATH include precedence loading the archive's single-precision header.
The actual library precision is checked before a manifest is accepted.

Run the native API contract executable against both original and candidate
libraries, and run `scripts/check_r5_coplanar_contact.py` against the archived
actual wheel poses. A candidate must pass those checks before a Gazebo campaign.

`scripts/check_r5_coplanar_round.py` routes the candidate path only to `gz sim`,
captures its actual loaded ODE hash, and preserves the existing real ROS safety
relay, bridge, actuator, observation and paused-step command chain. The bounded
campaign entry freezes inputs and rejects failed or changed-signature rounds.

The evidence directory `reports/evidence/terrain-v2/r5-coplanar-fix-20261012/`
contains the source archive reference, failed builds, successful manifest,
collision checks, frozen campaign, complete measurements and final report.
Do not treat the contact candidate as proof of hardware skid-turn performance
or as validation of arbitrary noncoplanar terrain.
