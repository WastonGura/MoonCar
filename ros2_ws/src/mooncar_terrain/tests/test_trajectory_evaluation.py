import math
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from trajectory_evaluation import interpolate,camera_extrinsic,ground_masks


class TrajectoryTests(unittest.TestCase):
    def test_bracket_rotation_and_optical_height(self):
        def pose(x,degrees):
            a=math.radians(degrees)/2
            return [x,0,0,0,0,math.sin(a),math.cos(a)]
        samples=[(1.,pose(0,179)),(1.02,pose(2,-179))]
        matrix,gap=interpolate(samples,1.01)
        np.testing.assert_allclose(matrix[:3,3],[1,0,0],atol=1e-12)
        np.testing.assert_allclose(matrix[:3,:3],np.diag([-1,-1,1]),atol=1e-12)
        self.assertAlmostEqual(gap,.02)
        np.testing.assert_allclose(interpolate(samples,1.)[0][:3,3],[0,0,0])
        for stamp in (.99,1.03):
            with self.assertRaises(ValueError):interpolate(samples,stamp)
        with self.assertRaises(ValueError):interpolate([(1.,pose(0,0)),(1.2,pose(2,0))],1.1)
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'robot.sdf'
            path.write_text('<sdf><model name="mooncar"><link name="base_footprint"><sensor name="mooncar_depth_camera"><pose>0.435 0 0.235 0 0 0</pose><gz_frame_id>depth_camera_optical_frame</gz_frame_id></sensor></link></model></sdf>')
            extrinsic,frame=camera_extrinsic(path)
            self.assertEqual(frame,'depth_camera_optical_frame')
            # One metre ahead with 0.235 m optical-down reaches world ground, no Z correction.
            np.testing.assert_allclose(extrinsic@np.array([0,.235,1,1]),[1.435,0,0,1])
        shapes=[('floor','plane',np.array([50,50]),np.eye(4)),('rock','ellipsoid',np.ones(3),np.eye(4))]
        points=np.array([[.5,0,2],[1.1,0,0],[2,0,100.]])
        _,footprint,interior=ground_masks(shapes,points)
        self.assertEqual(footprint.tolist(),[False,True,True])
        # Even a huge Z residual stays included if its XY is in the independent interior mask.
        self.assertEqual(interior.tolist(),[False,False,True])


if __name__=='__main__':unittest.main()
