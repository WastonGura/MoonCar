import math
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from depth_projection_evaluation import first_hit, evaluate
from terrain_geometry import project_depth


class RayTests(unittest.TestCase):
    def test_nearest_visible_faces_caps_rotation_and_misses(self):
        identity=np.eye(4);plane=('ground','plane',np.array([50,50]),identity)
        distance,owner=first_hit([plane],[0,0,2],[[0,0,-2],[0,0,1],[1,0,0]])
        self.assertEqual(distance[0],1);self.assertEqual(owner.tolist(),[0,-1,-1])
        pose=identity.copy();pose[:3,3]=[2,0,1]
        box=('box','box',np.array([.5,.5,.5]),pose)
        distance,owner=first_hit([plane,box],[0,0,1],[[1,0,0],[1,0,-1]])
        self.assertEqual(distance.tolist(),[1.5,1.]);self.assertEqual(owner.tolist(),[1,0])
        # The camera sees the near side at x=1.5, not the box's top at z=1.5.
        self.assertEqual(first_hit([box],[2,0,1],[[1,0,0]])[0][0],.5)
        pose=identity.copy();pose[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
        ellipsoid=('ellipsoid','ellipsoid',np.array([2,1,.5]),pose)
        self.assertAlmostEqual(first_hit([ellipsoid],[-3,0,0],[[1,0,0]])[0][0],2)
        cylinder=('cylinder','cylinder',np.array([1,1]),identity)
        self.assertEqual(first_hit([cylinder],[0,0,3],[[0,0,-1]])[0][0],2)
        self.assertEqual(first_hit([cylinder],[-3,0,0],[[1,0,0]])[0][0],2)
        self.assertTrue(math.isinf(first_hit([cylinder],[-3,0,2],[[1,0,0]])[0][0]))
        with self.assertRaises(ValueError):first_hit([plane],[0,0,float('nan')],[[0,0,1]])

    def test_raw_roundtrip_and_absolute_hypotheses_after_producer_correction(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);folder=root/'capture';folder.mkdir()
            def save(name,value):(folder/name).write_text(json.dumps(value))
            save('manifest.json',{'status':'captured','projection_pixel_offset':.5})
            save('camera-info.json',dict(width=4,height=4,k=[10,0,2,0,10,2,0,0,1]))
            save('depth-format.json',dict(width=4,height=4,step=16,encoding='32FC1',bigendian=False))
            (folder/'survey_yard.sdf').write_text('<sdf><world name="test"><model name="ground"><static>true</static><pose>0 0 0 0 0.2 0</pose><link name="l"><visual name="v"><geometry><plane><size>50 50</size></plane></geometry></visual></link></model></world></sdf>')
            # Ray x=-.15, world z=1-t: intersection with z=-tan(.2)*x.
            depth=1/(1+.15*math.tan(.2))
            raw=np.full((4,4),depth,dtype='<f4')
            points=project_depth(raw.tobytes(),4,4,16,'32FC1',False,(10,10,2,2),pixel_offset=.5).astype(np.float32)
            camera=np.diag([1.,-1.,-1.,1.]);camera[2,3]=1
            np.savez(folder/'samples.npz',points=points,offsets=[0,1],stamp=[1.],truth_camera=[camera])
            np.savez(folder/'raw-depth.npz',data=[np.frombuffer(raw.tobytes(),dtype=np.uint8)],stamps=[1.])
            with patch('depth_projection_evaluation.plot'),patch('builtins.print'):
                evaluate(folder,root/'result')
            result=json.loads((root/'result/summary.json').read_text())
            self.assertEqual(result['raw_frames_bitwise_reprojected'],1)
            self.assertLess(result['recovered_pixel_max_fraction'],1e-5)
            self.assertLess(result['groups'][3]['rmse_m'],1e-6)
            self.assertGreater(result['groups'][0]['rmse_m'],.005)


if __name__=='__main__':unittest.main()
