"""Measurement preservation and transaction isolation for the hot-path cache."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(HERE),str(HERE.parent/'scripts')]
import metric_rgbd_odometry as module
from test_metric_rgbd_odometry import PinholePlane


class HotPathTests(unittest.TestCase):
    def test_patch_samples_keep_every_original_value(self):
        rng=np.random.default_rng(123)
        image=rng.integers(0,256,(240,320),dtype=np.uint8)
        pixels=rng.uniform([13,13],[306,226],size=(97,2)).astype(np.float32)
        for dtype in (np.float32,np.float64):
            self.compare_samples(image,pixels.astype(dtype))
        self.compare_samples(image,pixels[:0])

    def compare_samples(self,image,pixels):
        offsets=np.stack(np.meshgrid(np.arange(-10,11),np.arange(-10,11)),axis=-1).reshape(-1,2)
        sites=pixels[:,None,:]+offsets;xy=np.floor(sites).astype(int)
        x,y=xy[...,0],xy[...,1];a,b=(sites-xy)[...,0],(sites-xy)[...,1]
        expected=((1-a)*(1-b)*image[y,x]+a*(1-b)*image[y,x+1]
            +(1-a)*b*image[y+1,x]+a*b*image[y+1,x+1])
        actual=module.MetricRGBDOdometry._sample_identity_patch(image,pixels)
        np.testing.assert_array_equal(actual,expected)

    def test_nonstandard_precision_border_and_strided_image_keep_original_sampler(self):
        image=np.arange(240*640,dtype=np.uint16).reshape(240,640)[:,::2]
        sites=np.array([[11.25,11.125],[220.123456789,110.234567891]],dtype=np.float64)
        self.compare_samples(image,sites)
        self.compare_samples(image.astype(np.uint8),sites)

    def fixture(self):
        core=module.MetricRGBDOdometry();scene=PinholePlane()
        image,grid,model=scene.frame(35.)
        result=core.update(0.,image,grid,0.,0.,0.,camera_model=model)
        self.assertTrue(result['accepted']);self.addCleanup(core.close)
        return core,core.references[-1],(image,grid,model)

    def test_projection_cache_uses_pose_and_cannot_be_mutated_by_a_caller(self):
        core,ref,_=self.fixture();core._view_motion_cache={}
        original=core._compute_projected_view_motion
        with patch.object(core,'_compute_projected_view_motion',wraps=original) as run:
            first=core._projected_view_motion(ref)
            first['visible_reference_features']=-1
            self.assertEqual(core._projected_view_motion(ref),original(ref))
            self.assertEqual(run.call_count,1)
            core._projected_view_motion(ref,body_xyz=core.xyz+np.array([.01,0,0]))
            core._projected_view_motion(dict(ref))
            self.assertEqual(run.call_count,3)

    def test_projection_cache_is_bounded_and_disabled_outside_update(self):
        core,ref,_=self.fixture()
        self.assertIsNone(core._view_motion_cache)
        core._view_motion_cache={}
        for n in range(140):core._projected_view_motion(ref,body_xyz=np.array([n*.001,0.,0.]))
        self.assertEqual(len(core._view_motion_cache),128)

    def test_update_releases_cache_on_success_and_exception(self):
        core,ref,(image,grid,model)=self.fixture()
        result=core.update(.1,image,grid,0.,0.,0.,camera_model=model)
        self.assertTrue(result['accepted']);self.assertIsNone(core._view_motion_cache)
        with patch.object(core,'_update_identity_transaction',side_effect=RuntimeError('failed')):
            with self.assertRaises(RuntimeError):core.update(.2,image,grid,0.,0.,0.,camera_model=model)
        self.assertIsNone(core._view_motion_cache)


if __name__=='__main__':unittest.main()
