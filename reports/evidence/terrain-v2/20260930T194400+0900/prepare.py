"""Generate an isolated rigid-ground single-wheel measurement check."""

from math import cos, pi, sin
from pathlib import Path


HERE = Path(__file__).resolve().parent
GRAVITY = 9.81
WHEEL_MASS = 1.18
SUSPENDED_MASS = 24.5 / GRAVITY - WHEEL_MASS
CORE_RADIUS = 0.08
NOMINAL_RADIUS = 0.10
WHEEL_SPEED = 0.2


def friction(mu=0.6):
    return f"""<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode>
      <bullet><friction>{mu}</friction><friction2>{mu}</friction2></bullet></friction></surface>"""


def grouser(index):
    angle = index * 2 * pi / 14
    x, z = 0.09 * sin(angle), -0.09 * cos(angle)
    return f"""<collision name="grouser_{index}">
      <pose>{x:.8f} 0 {z:.8f} 0 {angle:.8f} 0</pose>
      <geometry><box><size>0.020 0.080 0.020</size></box></geometry>{friction()}
    </collision>"""


def world(slip, ground_mu=0.6):
    assert slip in tuple(index / 10 for index in range(6))
    vx = NOMINAL_RADIUS * WHEEL_SPEED * (1 - slip)
    return f"""<?xml version="1.0"?>
<sdf version="1.10"><world name="wheel_bench">
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
    <engine><filename>gz-physics-bullet-featherstone-plugin</filename></engine>
  </plugin>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-forcetorque-system" name="gz::sim::systems::ForceTorque"/>
  <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
  <physics name="wheel_bench_physics" type="ode"><max_step_size>0.001</max_step_size><real_time_factor>0</real_time_factor></physics>
  <gravity>0 0 -{GRAVITY}</gravity>
  <model name="ground"><static>true</static><link name="surface">
    <collision name="ground_collision"><pose>0 0 -0.05 0 0 0</pose>
      <geometry><box><size>4 2 0.1</size></box></geometry>{friction(ground_mu)}</collision>
    <sensor name="ground_contact" type="contact"><always_on>true</always_on><update_rate>100</update_rate>
      <contact><collision>ground_collision</collision></contact></sensor>
  </link></model>
  <model name="rig">
    <link name="rail"><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial></link>
    <link name="x_carriage"><pose>0 0 0.13 0 0 0</pose>
      <inertial><mass>0.1</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial></link>
    <link name="z_carriage"><pose>0 0 0.13 0 0 0</pose>
      <inertial><mass>{SUSPENDED_MASS:.10f}</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial></link>
    <link name="wheel"><pose>0 0 0.13 0 0 0</pose>
      <inertial><mass>{WHEEL_MASS}</mass><inertia><ixx>0.003</ixx><iyy>0.006</iyy><izz>0.003</izz></inertia></inertial>
      <collision name="wheel_core"><pose>0 0 0 1.5707963268 0 0</pose>
        <geometry><cylinder><radius>{CORE_RADIUS}</radius><length>0.080</length></cylinder></geometry>{friction()}</collision>
      {''.join(grouser(i) for i in range(14))}
    </link>
    <joint name="rail_fixed" type="fixed"><parent>world</parent><child>rail</child></joint>
    <joint name="translate_x" type="prismatic"><parent>rail</parent><child>x_carriage</child>
      <axis><xyz>1 0 0</xyz></axis>
      <sensor name="x_force" type="force_torque"><always_on>true</always_on><update_rate>100</update_rate></sensor>
    </joint>
    <joint name="float_z" type="prismatic"><parent>x_carriage</parent><child>z_carriage</child>
      <axis><xyz>0 0 1</xyz></axis>
      <sensor name="z_force" type="force_torque"><always_on>true</always_on><update_rate>100</update_rate></sensor>
    </joint>
    <joint name="rotate_wheel" type="revolute"><parent>z_carriage</parent><child>wheel</child>
      <axis><xyz>0 1 0</xyz></axis>
    </joint>
    <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
      <joint_name>translate_x</joint_name><initial_velocity>{vx:.8f}</initial_velocity>
    </plugin>
    <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
      <joint_name>rotate_wheel</joint_name><initial_velocity>{WHEEL_SPEED}</initial_velocity>
    </plugin>
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
  </model>
</world></sdf>"""


if __name__ == "__main__":
    assert abs(SUSPENDED_MASS + WHEEL_MASS - 24.5 / GRAVITY) < 1e-10
    for slip in (index / 10 for index in range(6)):
        (HERE / f"slip_{int(slip * 100):02}.sdf").write_text(world(slip), encoding="utf-8")
        print(f"slip {slip:.0%}: v_x = {NOMINAL_RADIUS * WHEEL_SPEED * (1 - slip):.3f} m/s")
    (HERE / "slip_50_noground.sdf").write_text(
        world(0.5).replace('<model name="ground">', '<model name="ground"><pose>0 0 -1000 0 0 0</pose>'),
        encoding="utf-8",
    )
    (HERE / "slip_50_known_force.sdf").write_text(
        world(0.5).replace(
            '<model name="ground">',
            '''<plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench">
              <persistent><entity_name>rig::wheel</entity_name><entity_type>link</entity_type>
              <force>-5 0 0</force></persistent></plugin><model name="ground">''',
        ),
        encoding="utf-8",
    )
    for mu in (0.2, 1.2):
        for slip in (0.0, 0.5):
            (HERE / f"ground_mu{int(mu * 10):02}_slip_{int(slip * 100):02}.sdf").write_text(
                world(slip, ground_mu=mu), encoding="utf-8")
