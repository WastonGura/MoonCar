import importlib.util, json, os, tempfile, unittest
import subprocess, sys
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[4]
spec=importlib.util.spec_from_file_location('production_image_guard',Path(__file__).resolve().parents[1]/'launch/image_dds_guard.py')
guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)

class ImageDdsProductionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.world=self.root/'world.sdf';self.world.write_text('<sdf><world name="mooncar_ramp_v2"/></sdf>')
        self.env={'ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST','MOONCAR_IMAGE_DDS_PROFILE':'auto'}
    def tearDown(self):self.tmp.cleanup()
    def choose(self,model='mooncar_r5.urdf.xacro',env=None):return guard.select(model,self.world,env or self.env)['selected']
    def test_eligible_auto(self):self.assertEqual(self.choose(),'shm_tx8m')
    def test_all_three_worlds(self):
        for name in guard.WORLDS:
            self.world.write_text('<sdf><world name="'+name+'"/></sdf>');self.assertEqual(self.choose(),'shm_tx8m')
    def test_old_model_untouched(self):self.assertEqual(self.choose('mooncar.urdf.xacro'),'legacy')
    def test_v1_untouched(self):
        self.world.write_text('<sdf><world name="lunar_yard"/></sdf>');self.assertEqual(self.choose(),'legacy')
    def test_other_discovery_untouched(self):
        for discovery in ('SUBNET','SYSTEM_DEFAULT','OFF'):
            self.assertEqual(self.choose(env=dict(self.env,ROS_AUTOMATIC_DISCOVERY_RANGE=discovery)),'legacy')
    def test_explicit_old(self):self.assertEqual(self.choose(env=dict(self.env,MOONCAR_IMAGE_DDS_PROFILE='legacy')),'legacy')
    def test_explicit_unsupported(self):
        with self.assertRaises(RuntimeError):self.choose('mooncar.urdf.xacro',dict(self.env,MOONCAR_IMAGE_DDS_PROFILE='shm_tx8m'))
    def test_invalid_profile(self):
        with self.assertRaises(RuntimeError):self.choose(env=dict(self.env,MOONCAR_IMAGE_DDS_PROFILE='bad'))
    def test_missing_runtime(self):
        with self.assertRaises(FileNotFoundError):guard.verify(self.root)
    def test_bridge_environment_isolated(self):
        base=dict(self.env,LD_LIBRARY_PATH='/opt/ros/jazzy/lib')
        with patch.object(guard,'verify',return_value={}):child=guard.bridge_env(base,self.root,self.root/'receipt')
        self.assertNotIn('MOONCAR_IMAGE_DDS_TX_BYTES',base)
        self.assertEqual(child['MOONCAR_IMAGE_DDS_TX_BYTES'],'8388608')
        self.assertEqual(base['LD_LIBRARY_PATH'],'/opt/ros/jazzy/lib')
    def test_diagnostic_environment_rejected(self):
        with patch.object(guard,'verify',return_value={}):
            with self.assertRaises(RuntimeError):guard.bridge_env(dict(self.env,MOONCAR_DDS_TRACE='/tmp/trace'),self.root,'receipt')
    def test_wrong_runtime_version(self):
        (self.root/'manifest.json').write_text(json.dumps(dict(versions={},capacity_bytes=8388608,diagnostic_instrumentation=False)))
        with self.assertRaises(RuntimeError):guard.verify(self.root)
    def test_wrong_capacity_receipt(self):
        file=self.root/'receipt';file.write_text(json.dumps(dict(pid=1,segment_bytes=524288)))
        with patch.object(guard,'verify',return_value={}):
            with self.assertRaises(RuntimeError):guard.process_receipt(1,file,self.root)
    def test_wrong_process_receipt(self):
        file=self.root/'receipt';file.write_text(json.dumps(dict(pid=2,segment_bytes=8388608,port_queue_capacity=512,user_transports=2,builtin_transports=False,participant_created=True)))
        with patch.object(guard,'verify',return_value={}):
            with self.assertRaises(RuntimeError):guard.process_receipt(1,file,self.root)
    def test_missing_receipt_timeout(self):
        with patch.object(guard,'bridge_env',return_value=dict(os.environ)):
            with self.assertRaises(TimeoutError):guard.supervise('/bin/sleep',['5'],self.root,self.root/'logs',timeout=.05)
    def test_bridge_exit_before_receipt(self):
        with patch.object(guard,'bridge_env',return_value=dict(os.environ)):
            with self.assertRaises(RuntimeError):guard.supervise('/bin/false',[],self.root,self.root/'logs',timeout=.2)
    def test_observed_model_keeps_xacro_prefix(self):
        script=ROOT/'scripts/check_image_dds_launch.py'
        self.assertTrue(script.is_file())
        spec=importlib.util.spec_from_file_location('image_launch_fixture',script)
        fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)
        source='<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:arg name="prefix" default=""/><link name="$(arg prefix)base"/></robot>'
        tree=ET.fromstring(source);serialized=ET.tostring(tree,encoding='unicode')
        self.assertIn('<xacro:arg',serialized);self.assertNotIn('ns0:',serialized)
        import xacro
        file=self.root/'model.xacro';file.write_text(serialized)
        self.assertIn('name="base"',xacro.process_file(str(file)).toxml())
    def test_ns0_reproduces_startup_failure(self):
        import xacro
        file=self.root/'bad.xacro';file.write_text('<robot xmlns:ns0="http://www.ros.org/wiki/xacro"><ns0:arg name="prefix" default=""/><link name="$(arg prefix)base"/></robot>')
        with self.assertRaises(xacro.XacroException):xacro.process_file(str(file))
    def test_old_launch_without_r5_arguments_uses_legacy_bridge(self):
        from launch import LaunchContext
        spec=importlib.util.spec_from_file_location('image_launch_adapter',Path(__file__).resolve().parents[1]/'launch/image_dds_launch.py')
        adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)
        context=LaunchContext();context.launch_configurations['world']=str(self.world)
        with patch.dict(os.environ,{'MOONCAR_IMAGE_DDS_PROFILE':'auto'}):
            actions=adapter.launch_bridge(context)
        from launch.actions import IncludeLaunchDescription
        self.assertTrue(any(isinstance(a,IncludeLaunchDescription) for a in actions))

if __name__=='__main__':unittest.main()
