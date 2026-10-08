"""Validate declared model streams without weakening R5 acquisition gates."""
import json,sys,tempfile,unittest
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
import image_dds_validation as checks

class CapabilityTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.sdf=self.root/'model.sdf';self.bridge=self.root/'bridge.yaml'
    def tearDown(self):self.tmp.cleanup()
    def model(self,images):
        model=ET.SubElement(ET.Element('sdf'),'model')
        for kind in images:
            sensor=ET.SubElement(model,'sensor',name=kind,type='camera' if kind=='rgb' else 'depth')
            ET.SubElement(sensor,'topic').text='/mooncar/'+kind
        ET.ElementTree(model).write(self.sdf)
        self.bridge.write_text(json.dumps([dict(ros_topic_name='/mooncar/'+k+'/image',gz_topic_name='/mooncar/'+k,
            ros_type_name='sensor_msgs/msg/Image',gz_type_name='gz.msgs.Image',direction='GZ_TO_ROS') for k in images]))
    def tail(self):
        return [dict(kind=k,sim=10.,stamp=10.) for k in checks.stream_bounds()]
    def images(self,kinds):
        records=[]
        for kind in kinds:
            file=kind+('.png' if kind=='rgb' else '.npz')
            if kind=='rgb':Image.fromarray(np.arange(48,dtype=np.uint8).reshape(4,4,3)).save(self.root/file)
            else:np.savez_compressed(self.root/file,depth=np.ones((4,4)))
            records.append(dict(kind=kind,file=file,archive_completed_wall=100.))
        return records
    def test_original_depth_only_model_is_explicitly_supported(self):
        self.model(['depth']);p=checks.collection_profile(self.sdf,self.bridge,True)
        self.assertEqual(p['images'],['depth']);self.assertNotIn('rgb',p['readiness'])
        self.assertIn('rgb',p['absent_images']);self.assertEqual(p['bridge_sha256'],checks.sha(self.bridge))
    def test_r5_missing_rgb_is_not_accepted(self):
        self.model(['depth'])
        with self.assertRaises(RuntimeError):checks.collection_profile(self.sdf,self.bridge)
    def test_r5_requires_both_declared_streams(self):
        self.model(['rgb','depth']);p=checks.collection_profile(self.sdf,self.bridge)
        self.assertEqual(set(p['images']),{'rgb','depth'});self.assertEqual(p['absent_images'],{})
    def test_bridge_missing_declared_image_fails(self):
        self.model(['rgb','depth']);self.bridge.write_text('[]')
        with self.assertRaises(RuntimeError):checks.collection_profile(self.sdf,self.bridge)
    def test_wrong_bridge_direction_and_duplicate_route_fail(self):
        self.model(['depth']);routes=json.loads(self.bridge.read_text())
        self.bridge.write_text(json.dumps(routes*2))
        with self.assertRaises(RuntimeError):checks.collection_profile(self.sdf,self.bridge,True)
        routes[0]['direction']='ROS_TO_GZ';self.bridge.write_text(json.dumps(routes))
        with self.assertRaises(RuntimeError):checks.collection_profile(self.sdf,self.bridge,True)
    def test_legacy_tail_does_not_require_undeclared_rgb(self):
        records=[r for r in self.tail() if r['kind']!='rgb']
        self.assertTrue(checks.reception_tail(records,10.,10.,['depth'])['passed'])
        self.assertFalse(checks.reception_tail(records,10.,10.)['passed'])
    def test_legacy_depth_and_other_stream_gaps_still_fail(self):
        for k in ('depth','imu','joints','truth','safe_command','gz_command'):
            records=[r for r in self.tail() if r['kind'] not in ('rgb',k)]
            self.assertFalse(checks.reception_tail(records,10.,10.,['depth'])['passed'],k)
    def test_depth_only_files_pass_only_declared_legacy_profile(self):
        records=self.images(['depth'])
        self.assertTrue(checks.image_files(self.root,records,['depth'])['passed'])
        self.assertFalse(checks.image_files(self.root,records)['passed'])
    def test_both_r5_files_are_checked(self):
        records=self.images(['rgb','depth'])
        self.assertTrue(checks.image_files(self.root,records)['passed'])
        (self.root/'rgb.png').unlink();self.assertFalse(checks.image_files(self.root,records)['passed'])
    def test_legacy_invalid_depth_and_missing_receipt_fail(self):
        records=self.images(['depth']);np.savez_compressed(self.root/'depth.npz',depth=np.zeros((4,4)))
        self.assertFalse(checks.image_files(self.root,records,['depth'])['passed'])
        records=self.images(['depth']);records[0].pop('archive_completed_wall')
        self.assertFalse(checks.image_files(self.root,records,['depth'])['passed'])
    def test_real_expanded_models_declare_the_expected_streams(self):
        p=Path('/evidence/derived-preflight/result.json')
        if not p.exists():self.skipTest('Actual installed model preflight required')
        models=json.loads(p.read_text())['models']
        self.assertEqual(models['mooncar.urdf.xacro']['collection']['images'],['depth'])
        self.assertEqual(models['mooncar_r5.urdf.xacro']['collection']['images'],['depth','rgb'])

if __name__=='__main__':unittest.main()
