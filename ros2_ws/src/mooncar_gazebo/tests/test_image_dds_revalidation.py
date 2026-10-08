"""Pre-freeze gates for actual-model instrumentation and failed-round cleanup."""
import importlib.util, json, os, subprocess, sys, tempfile, unittest
from pathlib import Path
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
import image_dds_validation as checks
spec=importlib.util.spec_from_file_location('dds_revalidation_guard',ROOT/'ros2_ws/src/mooncar_gazebo/launch/image_dds_guard.py')
guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)

class RevalidationTests(unittest.TestCase):
    def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
    def tearDown(self):self.tmp.cleanup()
    def test_incomplete_receipt_is_not_ready(self):
        p=self.root/'receipt';self.assertIsNone(guard.complete_receipt(p))
        p.write_text('{"pid":');self.assertIsNone(guard.complete_receipt(p))
        p.write_text('{"pid":10}');self.assertEqual(guard.complete_receipt(p),{'pid':10})
    def test_valid_wrong_receipt_is_not_retried_as_partial(self):
        p=self.root/'receipt';p.write_text('{"pid":1}')
        with patch.object(guard,'verify',return_value={}):
            with self.assertRaises(RuntimeError):guard.process_receipt(1,p,self.root)
    def test_config_default_without_env_and_explicit_override(self):
        p=self.root/'config';p.write_text('{"profile":"auto"}')
        world=self.root/'world';world.write_text('<sdf><world name="mooncar_ramp_v2"/></sdf>')
        env={'ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}
        self.assertEqual(guard.select('mooncar_r5.urdf.xacro',world,env,guard.default_profile(p))['selected'],'shm_tx8m')
        self.assertEqual(guard.select('mooncar_r5.urdf.xacro',world,dict(env,MOONCAR_IMAGE_DDS_PROFILE='legacy'),guard.default_profile(p))['selected'],'legacy')
    def test_bad_default_fails(self):
        p=self.root/'config';p.write_text('{"profile":"unexpected"}')
        with self.assertRaises(RuntimeError):guard.default_profile(p)
    def test_instrumentation_keeps_xacro_prefix_and_unchanged_physics(self):
        import xml.etree.ElementTree as ET
        p=self.root/'model';p.write_text('<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:arg name="prefix" default=""/><link name="$(arg prefix)base"/></robot>')
        out=self.root/'observed';checks.observed_model(p,out,self.root/'steps')
        self.assertIn('xacro:arg',out.read_text());self.assertNotIn('ns0:',out.read_text())
        self.assertEqual(checks.semantic(ET.parse(p).getroot()),checks.semantic(checks.strip_observers(ET.parse(out).getroot())))
    def test_installed_mismatch_fails(self):
        source=self.root/'source';p=source/'ros2_ws/src/mooncar_gazebo/launch/a.py';p.parent.mkdir(parents=True);p.write_text('current')
        with self.assertRaises(RuntimeError):checks.installed_files(source,self.root/'install')
    def test_actual_models_preflight_evidence(self):
        # Full xacro / gz sdf preflight is run before freezing, independently of simulations.
        p=Path('/evidence/derived-preflight/result.json')
        if not p.is_file():self.skipTest('No container model preflight evidence')
        result=json.loads(p.read_text())
        self.assertEqual(set(result['models']),{'mooncar_r5.urdf.xacro','mooncar.urdf.xacro'})
        self.assertTrue(all(m['urdf_equal'] and m['sdf_equal'] for m in result['models'].values()))
    def test_cleanup_of_already_exited_child(self):
        p=subprocess.Popen([sys.executable,'-c','pass'],start_new_session=True);p.wait();checks.stop_process(p);self.assertEqual(p.returncode,0)
    def test_images_missing_receipt_or_file_fail(self):
        self.assertFalse(checks.image_files(self.root,[dict(kind='rgb',file='missing.png',archive_completed_wall=2),dict(kind='depth',file='missing.npz')])['passed'])
    def test_frozen_campaign_stops_after_first_failure(self):
        spec=importlib.util.spec_from_file_location('image_campaign',ROOT/'scripts/run_image_dds_production_campaign.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        # Guard conditions run before spawning Gazebo; old evidence cannot fill a new campaign.
        (self.root/'existing').mkdir()
        with self.assertRaises(FileExistsError):module.run(self.root/'existing')

if __name__=='__main__':unittest.main()
