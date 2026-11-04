"""The optional v2 repair must fail closed without affecting legacy worlds."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

PACKAGE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('guard',PACKAGE/'launch/physics_guard.py')
guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)
sys.path.insert(0,str(PACKAGE.parents[2]/'scripts'))
from check_r5_physics import body_samples, last_message, stability


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.world=self.root/'world.sdf'
        self.world.write_text('''<sdf><world><plugin name="gz::sim::systems::Physics"><engine>
        <filename>libmooncar-physics7-dartsim.so</filename></engine></plugin><model><link><collision>
        <geometry><mesh><uri>model://mooncar_gazebo/models/ground.obj</uri></mesh></geometry>
        <surface><friction><ode><mu>.8</mu><mu2>.72</mu2></ode></friction></surface>
        </collision></link></model></world></sdf>''')
        (self.root/'models').mkdir();(self.root/'models/ground.obj').write_text('fixture')
        self.library=self.root/guard.PLUGIN_NAME;self.library.write_bytes(b'fixture library')
        self.manifest=dict(gz_physics_version='7.8.0',library_sha256=hashlib.sha256(self.library.read_bytes()).hexdigest())
        self.save_manifest()

    def save_manifest(self):
        (self.root/'manifest.json').write_text(json.dumps(self.manifest))

    def tearDown(self):self.temp.cleanup()

    def test_missing_library_or_configuration_never_falls_back(self):
        with self.assertRaisesRegex(RuntimeError,'requires'):
            guard.prepare_world(self.world,None,self.root,'7.8.0')
        self.library.unlink()
        with self.assertRaisesRegex(RuntimeError,'missing'):
            guard.prepare_world(self.world,self.root,self.root,'7.8.0')

    def test_runtime_version_and_library_integrity_are_enforced(self):
        with self.assertRaisesRegex(RuntimeError,'version mismatch'):
            guard.prepare_world(self.world,self.root,self.root,'7.9.0')
        self.library.write_bytes(b'changed')
        with self.assertRaisesRegex(RuntimeError,'checksum'):
            guard.prepare_world(self.world,self.root,self.root,'7.8.0')

    def test_resolved_copy_preserves_geometry_and_friction(self):
        before=self.world.read_bytes()
        output=Path(guard.prepare_world(self.world,self.root,self.root,'7.8.0'))
        try:
            tree=ET.parse(output)
            self.assertEqual(tree.findtext('.//mesh/uri'),str(self.root/'models/ground.obj'))
            self.assertEqual(tree.findtext('.//ode/mu'),'.8')
            self.assertEqual(tree.findtext('.//ode/mu2'),'.72')
            self.assertEqual(self.world.read_bytes(),before)
        finally:output.unlink()

    def test_missing_collision_resource_is_rejected(self):
        (self.root/'models/ground.obj').unlink()
        with self.assertRaisesRegex(RuntimeError,'Unresolved'):
            guard.prepare_world(self.world,self.root,self.root,'7.8.0')

    def test_legacy_world_needs_no_optional_runtime(self):
        self.world.write_text(self.world.read_text().replace(guard.PLUGIN_NAME,'gz-physics-bullet-featherstone-plugin'))
        self.assertEqual(guard.prepare_world(self.world,None,self.root,'7.9.0'),str(self.world))

    def test_auto_control_profile_requires_r5_and_v2_dart(self):
        tree=ET.parse(self.world);tree.find('world').set('name','mooncar_ramp_v2');tree.write(self.world)
        self.assertEqual(guard.resolve_control_profile(self.world,'mooncar_r5.urdf.xacro'),'r5_dart_v2')
        self.assertEqual(guard.resolve_control_profile(self.world,'mooncar.urdf.xacro'),'legacy')
        self.assertEqual(guard.resolve_control_profile(self.world,'mooncar_r5.urdf.xacro','legacy'),'legacy')
        tree.find('world').set('name','lunar_yard');tree.write(self.world)
        self.assertEqual(guard.resolve_control_profile(self.world,'mooncar_r5.urdf.xacro'),'legacy')
        with self.assertRaisesRegex(RuntimeError,'requires'):
            guard.resolve_control_profile(self.world,'mooncar_r5.urdf.xacro','r5_dart_v2')

    def test_pose_vector_uses_actual_per_pose_stamp(self):
        rows=[dict(pose=[dict(name='mooncar',header=dict(stamp=dict(sec=4,nsec=50000000)),position=dict(z=.01),orientation=dict(w=1))])]
        self.assertEqual(body_samples(rows)[0]['t'],4.05)

    def test_missing_or_gapped_truth_cannot_pass(self):
        self.assertFalse(stability([])['passed'])
        rows=[dict(t=0,xyz=[0,0,0],rpy=[0,0,0]),dict(t=.2,xyz=[0,0,0],rpy=[0,0,0])]
        self.assertFalse(stability(rows)['passed'])

    def test_cli_queued_stats_messages_use_latest(self):
        self.assertEqual(last_message('{"iterations": 2}\n{"iterations": 3}')['iterations'],3)

    def test_generator_backend_choice_changes_only_runtime_configuration(self):
        sys.path.insert(0,str(PACKAGE/'scripts'))
        import build_terrain_v2 as build
        old_backend,old_step=build.PHYSICS_BACKEND,build.PHYSICS_STEP
        trees=[]
        try:
            for backend,step in [('bullet',.002),('dart',.001)]:
                build.PHYSICS_BACKEND,build.PHYSICS_STEP=backend,step
                tree=ET.fromstring(build.world_xml('ramp',['<model name="fixture"><static>true</static></model>']))
                self.assertEqual(tree.findtext('./world/physics/max_step_size'),str(step))
                self.assertIn(build.PHYSICS_ENGINES[backend],tree.findtext('./world/plugin/engine/filename'))
                trees.append(tree)
            self.assertEqual(ET.tostring(trees[0].find('./world/model')),ET.tostring(trees[1].find('./world/model')))
        finally:build.PHYSICS_BACKEND,build.PHYSICS_STEP=old_backend,old_step

    def test_checked_in_worlds_match_generator_defaults(self):
        sys.path.insert(0,str(PACKAGE/'scripts'))
        import build_terrain_v2 as build
        for scene in ('ramp','curve','survey'):
            tree=ET.parse(PACKAGE/f'worlds/mooncar_{scene}_v2.sdf')
            self.assertEqual(tree.findtext('./world/plugin/engine/filename'),build.PHYSICS_ENGINES[build.PHYSICS_BACKEND])
            self.assertEqual(float(tree.findtext('./world/physics/max_step_size')),build.PHYSICS_STEP)


if __name__=='__main__':unittest.main()
