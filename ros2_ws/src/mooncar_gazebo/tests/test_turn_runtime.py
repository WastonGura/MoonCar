"""Selection and preservation contracts for the R5 steering integration."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

source=Path(__file__).parents[1]/'launch/turn_guard.py'
spec=importlib.util.spec_from_file_location('turn_guard_tests',source)
guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)

class TestTurnRuntime(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.world=self.root/'world.sdf'
        self.world.write_text('<sdf><world name="mooncar_ramp_v2"><plugin name="gz::sim::systems::Physics"><engine><filename>libmooncar-physics7-dartsim.so</filename></engine></plugin></world></sdf>')
    def tearDown(self):self.tmp.cleanup()
    def test_selection(self):
        self.assertEqual(guard.select('mooncar_r5.urdf.xacro',self.world,'auto'),'r5_v2')
        self.assertEqual(guard.select('mooncar_r5.urdf.xacro',self.world,'legacy'),'legacy')
        self.assertEqual(guard.select('mooncar.urdf.xacro',self.world,'auto'),'legacy')
        with self.assertRaises(RuntimeError):guard.select('mooncar.urdf.xacro',self.world,'r5_v2')
        with self.assertRaises(RuntimeError):guard.select('mooncar_r5.urdf.xacro',self.world,'typo')
    def test_other_world_backend(self):
        for change in ('mooncar_ramp_v2','libmooncar-physics7-dartsim.so'):
            text=self.world.read_text();self.world.write_text(text.replace(change,'legacy'))
            self.assertEqual(guard.select('mooncar_r5.urdf.xacro',self.world,'auto'),'legacy')
            self.world.write_text(text)
    def robot(self):
        controller='<plugin name="gz::sim::systems::JointPositionController"><p_gain>20</p_gain><i_gain>0.1</i_gain><d_gain>0.5</d_gain><i_max>1</i_max><i_min>-1</i_min><cmd_max>5</cmd_max><cmd_min>-5</cmd_min></plugin>'
        return '<robot name="mooncar_r5"><link name="base"/><gazebo>'+controller*2+'</gazebo></robot>'
    def test_model_preservation(self):
        original=self.robot()
        changed=ET.fromstring(guard.augment_urdf(original,'/runtime'))
        extra=changed[-1];changed.remove(extra)
        for c in changed.findall('gazebo/plugin'):c.find('d_gain').text='0.5'
        self.assertEqual(ET.tostring(changed),ET.tostring(ET.fromstring(original)))
        self.assertEqual(len(extra.findall('plugin')),3)
        self.assertEqual(len(extra.findall("plugin[@name='gz::sim::systems::WheelSlip']/wheel")),4)
        self.assertEqual(extra.findtext("plugin[@name='mooncar::TurnSlipControl']/mode"),'pure_turn')
        self.assertIsNone(extra.find('plugin/output'))
        adaptive=extra.find("plugin[@name='mooncar::AdaptiveAxleAllocation']")
        self.assertIsNotNone(adaptive)
        self.assertIsNone(adaptive.find('front_factor'))
        self.assertIsNone(extra.find("plugin[@name='mooncar::AxleAllocation']"))
    def test_duplicates_and_wrong_model(self):
        with self.assertRaises(RuntimeError):guard.augment_urdf('<robot name="other"/>','/r')
        with self.assertRaises(RuntimeError):guard.augment_urdf(guard.augment_urdf(self.robot(),'/r'),'/r')
    def test_runtime_integrity(self):
        root=self.root
        for name in guard.LIBRARIES:(root/name).write_bytes(name.encode())
        (root/'libode.so.8').symlink_to('libode.so.8.0.2')
        m=dict(ode_version='0.16.2',gz_sim_major=8,gz_physics_version='7.8.0',gimbal_changed=True,gimbal_profile=[20,.1,0,1,5],
            allocation_profile='independent_laser_feedback_v1',libraries={n:guard.sha(root/n) for n in guard.LIBRARIES},stock_ode_path=str(root/guard.LIBRARIES[0]),
            stock_ode_sha256=guard.sha(root/guard.LIBRARIES[0]),dart_path=str(root/guard.LIBRARIES[1]),dart_sha256=guard.sha(root/guard.LIBRARIES[1]))
        (root/'manifest.json').write_text(json.dumps(m));self.assertEqual(guard.verify(root),m)
        for key,value in (('ode_version','wrong'),('gimbal_changed',False),('gz_physics_version','7.6.0'),('allocation_profile','fixed')):
            bad=dict(m);bad[key]=value;(root/'manifest.json').write_text(json.dumps(bad))
            with self.assertRaises(RuntimeError):guard.verify(root)
        (root/'manifest.json').write_text(json.dumps(m));(root/guard.LIBRARIES[2]).write_bytes(b'wrong')
        with self.assertRaises(RuntimeError):guard.verify(root)
    def adapter(self):
        spec=importlib.util.spec_from_file_location('turn_launch_tests',source.with_name('turn_launch.py'))
        value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value)
        return value
    def context(self):
        from launch import LaunchContext
        context=LaunchContext()
        context.launch_configurations.update(model_file='mooncar_r5.urdf.xacro',world=str(self.world))
        (self.root/'config').mkdir(exist_ok=True)
        (self.root/'config/turn_defaults.json').write_text('{"profile":"auto"}')
        (self.root/'config/turn_release.json').write_text('{"approved":false,"reason":"failed curve gate"}')
        return context
    def test_unapproved_auto_uses_legacy(self):
        adapter=self.adapter();context=self.context()
        share=type('Share',(),{'perform':lambda _,context:str(self.root)})()
        with patch.object(adapter,'FindPackageShare',return_value=share),patch.dict('os.environ',{'MOONCAR_TURN_PROFILE':'auto'}):
            actions=adapter.configure(context)
        for action in actions[:2]:action.execute(context)
        self.assertEqual(context.launch_configurations['_turn_selected'],'legacy')
    def test_unapproved_explicit_fails(self):
        adapter=self.adapter();context=self.context()
        share=type('Share',(),{'perform':lambda _,context:str(self.root)})()
        with patch.object(adapter,'FindPackageShare',return_value=share),patch.dict('os.environ',{'MOONCAR_TURN_PROFILE':'r5_v2'}):
            with self.assertRaisesRegex(RuntimeError,'failed curve gate'):adapter.configure(context)
    def test_selected_missing_library_never_falls_back(self):
        adapter=self.adapter();context=self.context()
        (self.root/'config/turn_release.json').write_text('{"approved":true}')
        share=type('Share',(),{'perform':lambda _,context:str(self.root)})()
        with patch.object(adapter,'FindPackageShare',return_value=share),patch.object(adapter,'guard',return_value=guard),\
             patch.object(guard,'verify',side_effect=RuntimeError('Missing runtime')),patch.dict('os.environ',{'MOONCAR_TURN_PROFILE':'auto'}):
            with self.assertRaisesRegex(RuntimeError,'Missing runtime'):adapter.configure(context)
    def test_legacy_never_reads_repair_approval(self):
        adapter=self.adapter();context=self.context()
        context.launch_configurations['model_file']='mooncar.urdf.xacro'
        (self.root/'config/turn_release.json').write_text('invalid unused candidate file')
        share=type('Share',(),{'perform':lambda _,context:str(self.root)})()
        with patch.object(adapter,'FindPackageShare',return_value=share),patch.dict('os.environ',{'MOONCAR_TURN_PROFILE':'auto'}):
            actions=adapter.configure(context)
        for action in actions[:2]:action.execute(context)
        self.assertEqual(context.launch_configurations['_turn_selected'],'legacy')

if __name__=='__main__':unittest.main()
