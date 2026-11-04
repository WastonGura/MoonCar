"""Explicit support integration, rejection boundaries and frozen actuator values."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

LAUNCH=Path(__file__).parents[1]/'launch'
sys.path.insert(0,str(LAUNCH))
import support_profile as support
import image_dds_guard

class SupportTests(unittest.TestCase):
    def test_precision_copy_preserves_scene_and_legacy_input(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'world.sdf'
            path.write_text('''<sdf><world name="lunar_yard"><gravity>0 0 -1.62</gravity>
                <physics><max_step_size>.001</max_step_size></physics>
                <plugin name="gz::sim::systems::Physics"/><model name="ground"><pose>1 2 3 0 0 0</pose></model></world></sdf>''')
            before=path.read_bytes();output=Path(support.prepare_world(path))
            try:
                self.assertEqual(ET.parse(output).findtext('./world/physics/max_step_size'),'0.00025')
                self.assertEqual(ET.parse(output).findtext('./world/model/pose'),'1 2 3 0 0 0')
                env={'ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'}
                self.assertEqual(image_dds_guard.select('mooncar_r5.urdf.xacro',output,env,'auto')['selected'],'legacy')
                self.assertEqual(image_dds_guard.select('mooncar_r5.urdf.xacro',output,env,'auto',True)['selected'],'shm_tx8m')
                self.assertEqual(path.read_bytes(),before)
            finally:output.unlink()
            path.write_text(before.decode().replace('-1.62','-9.81'))
            with self.assertRaisesRegex(RuntimeError,'gravity'):support.prepare_world(path)

    def test_fixed_candidate_does_not_enable_adaptive_controller(self):
        control='<plugin name="gz::sim::systems::JointPositionController"><p_gain>20</p_gain><i_gain>.1</i_gain><d_gain>.5</d_gain><i_max>1</i_max><i_min>-1</i_min><cmd_max>5</cmd_max><cmd_min>-5</cmd_min></plugin>'
        tree=ET.fromstring(support.augment_urdf('<robot name="mooncar_r5"><gazebo>'+control*2+'</gazebo></robot>','/runtime'))
        self.assertIsNone(tree.find("gazebo/plugin[@name='mooncar::AdaptiveAxleAllocation']"))
        allocation=tree.find("gazebo/plugin[@name='mooncar::AxleAllocation']")
        self.assertAlmostEqual(float(allocation.findtext('front_factor')),.96026822178862603)
        self.assertEqual(tree.findtext("gazebo/plugin[@name='gz::sim::systems::JointPositionController']/d_gain"),'0')

if __name__=='__main__':unittest.main()
