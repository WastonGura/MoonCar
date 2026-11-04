"""Analytic measured-image/depth regressions; no ROS, map, or simulator input."""
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from metric_rgbd_odometry import MetricRGBDOdometry, _ConflictingRegisteredView


class TexturedPlane:
    def __init__(self, seed=7):
        rng = np.random.default_rng(seed)
        self.texture = cv2.GaussianBlur(rng.integers(0, 256, (900, 4000), dtype=np.uint8), (3, 3), .5)
        # Resolved, nonrepeating features rather than labels supplied to the core.
        for _ in range(1400):
            x, y = rng.integers(8, 3992), rng.integers(8, 892)
            cv2.circle(self.texture, (int(x), int(y)), int(rng.integers(2, 7)), int(rng.integers(256)), -1)

    def frame(self, body=(0., 0., 0.), view=(0., 0.), slope=0., noise=0., seed=0, camera_yaw=0.):
        height, width, ppm = 240, 320, 80.
        # Orthographic planar camera is an analytic calibrated sensor. A shifted
        # field of view changes measured feature identities and body-relative XYZ.
        ox, oy = int(round(400+(body[0]+view[0])*ppm)), int(round(300+(body[1]+view[1])*ppm))
        image = self.texture[oy:oy+height, ox:ox+width].copy()
        v, u = np.mgrid[:height, :width]
        world_x, world_y = (u+ox-400)/ppm, (v+oy-300)/ppm
        if camera_yaw:
            c, s = np.cos(camera_yaw), np.sin(camera_yaw)
            du, dv = (u-width/2)/ppm, (v-height/2)/ppm
            world_x = c*du-s*dv+(ox-400+width/2)/ppm
            world_y = s*du+c*dv+(oy-300+height/2)/ppm
            image = cv2.remap(self.texture, (world_x*ppm+400).astype(np.float32),
                              (world_y*ppm+300).astype(np.float32), cv2.INTER_LINEAR)
        grid = np.stack((world_x-body[0], world_y-body[1],
                         slope*world_x-body[2]+np.ones_like(world_x)), axis=-1)
        if noise:
            grid += np.random.default_rng(seed).normal(0, noise, grid.shape)
        return image, grid


class PinholePlane:
    """Calibrated plane rendering, with actual camera rotation about its center."""
    def __init__(self):
        rng=np.random.default_rng(321)
        self.texture=cv2.GaussianBlur(rng.integers(0,256,(1600,2000),dtype=np.uint8),(5,5),.9)
        self.K=np.array([[277.,0.,160.],[0.,277.,120.],[0.,0.,1.]])

    def frame(self,degrees,body=(0.,0.,0.)):
        body=np.asarray(body,dtype=float)
        angle=np.deg2rad(degrees);c,s=np.cos(angle),np.sin(angle)
        pitch=np.array([[c,0.,s],[0.,1.,0.],[-s,0.,c]])
        orientation=pitch@np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
        lever=np.array([.45,0.,.24]);v,u=np.mgrid[:240,:320]
        rays=np.stack(((u+.5-160.)/277.,(v+.5-120.)/277.,np.ones_like(u)),axis=-1)@orientation.T
        depth=-(lever[2]+body[2])/rays[:,:,2]
        grid=body+lever+rays*depth[:,:,None]
        image=cv2.remap(self.texture,(grid[:,:,0]*160.+128.).astype(np.float32),
                       (grid[:,:,1]*160.+800.).astype(np.float32),cv2.INTER_LINEAR,borderValue=90)
        valid=np.isfinite(depth)&(depth>=.1)&(depth<=6.)
        grid[~valid]=np.nan
        return image,grid-body,dict(K=self.K.copy(),rotation=orientation,translation=lever)


class RGBDTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(1)
        cls.scene = TexturedPlane()

    def test_textured_flat_plane_measures_xyz_and_tilted_plane(self):
        for slope in (0., .12):
            core = MetricRGBDOdometry()
            image, points = self.scene.frame(slope=slope)
            self.assertTrue(core.update(0., image, points, 0., 0., 0.)['accepted'])
            truth = np.array([.05, .025, .012])
            image, points = self.scene.frame(truth, slope=slope)
            result = core.update(.1, image, points, 0., .51, 0.)
            self.assertTrue(result['accepted'], result)
            np.testing.assert_allclose(result['xyz'], truth, atol=.006)
            self.assertGreaterEqual(result['inliers'], 24)
            self.assertGreater(np.linalg.eigvalsh(result['covariance']).min(), 0.)
            self.assertGreaterEqual(np.diag(result['measurement_covariance']).min(), .003**2)

    def test_long_range_biased_wheels_keyframe_bounds(self):
        core = MetricRGBDOdometry()
        image, points = self.scene.frame()
        self.assertTrue(core.update(0., image, points, 0., 0., 0.)['accepted'])
        origin = core.references[0]
        errors = []
        for n, x in enumerate(np.arange(.05, 12.001, .05), 1):
            image, points = self.scene.frame((x, 0., 0.))
            result = core.update(n*.1, image, points, 0., .51, 0.)
            self.assertTrue(result['accepted'], (n, result))
            errors.append(np.linalg.norm(result['xyz'][:2]-[x, 0.]))
            self.assertLessEqual(len(core.references), 20)
        self.assertLess(max(errors), .03)
        self.assertIs(core.references[0], origin)
        self.assertEqual(len(core.references), 20)

    def test_turning_imu_oriented_measurements_with_biased_wheels(self):
        core = MetricRGBDOdometry()
        image, points = self.scene.frame()
        self.assertTrue(core.update(0., image, points, 0., 0., 0.)['accepted'])
        truth = np.zeros(3)
        for n in range(1, 46):
            previous_yaw, yaw = (n-1)*np.pi/90, n*np.pi/90
            truth[:2] += .025*np.array([np.cos(previous_yaw), np.sin(previous_yaw)])
            image, points = self.scene.frame(truth, camera_yaw=yaw)
            result = core.update(n*.1, image, points, yaw, .255, np.pi/9)
            self.assertTrue(result['accepted'], (n, result))
            self.assertLess(np.linalg.norm(result['xyz']-truth), .015)

    def test_stationary_gimbal_fov_handover_and_return(self):
        core = MetricRGBDOdometry()
        image, points = self.scene.frame()
        self.assertTrue(core.update(0., image, points, 0., 0., 0.)['accepted'])
        origin = core.references[0]
        offsets = list(np.arange(.1, 6.01, .1))+list(np.arange(5.9, -.01, -.1))
        for n, offset in enumerate(offsets, 1):
            image, points = self.scene.frame(view=(offset, 0.))
            result = core.update(n*.1, image, points, 0., 0., 0.)
            self.assertTrue(result['accepted'], (n, result))
            self.assertLess(np.linalg.norm(result['xyz']), .01)
        self.assertGreater(len(core.references), 3)
        self.assertIs(core.references[0], origin)
        self.assertEqual(result['reference_id'], origin['id'])

    def test_static_fresh_measurements_do_not_grow_references(self):
        core = MetricRGBDOdometry()
        for n in range(12):
            image, points = self.scene.frame(noise=.0005, seed=n)
            result = core.update(n*.1, image, points, 0., 0., 0.)
            self.assertTrue(result['accepted'], result)
            self.assertLess(np.linalg.norm(result['xyz']), .002)
        self.assertEqual(len(core.references), 1)

    def test_untextured_planar_depth_cannot_create_identity(self):
        image, points = self.scene.frame()
        result = MetricRGBDOdometry().update(0., np.full_like(image, 128), points, 0., 0., 0.)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'], 'insufficient_rgbd_texture_depth')

    def test_low_contrast_measured_texture_survives_brightness_change(self):
        core = MetricRGBDOdometry()
        image, points = self.scene.frame()
        weak = np.rint(90.+(image.astype(float)-128.)*.08).astype(np.uint8)
        self.assertTrue(core.update(0., weak, points, 0., 0., 0.)['accepted'])
        for n in range(1, 9):
            truth = np.array([n*.025, 0., n*.001])
            image, points = self.scene.frame(truth)
            weak = np.rint(90.+n+(image.astype(float)-128.)*.08).astype(np.uint8)
            result = core.update(n*.1, weak, points, 0., .255, 0.)
            self.assertTrue(result['accepted'], (n, result))
            self.assertLess(np.linalg.norm(result['xyz']-truth), .012)

    def test_independent_fresh_intensity_noise_cannot_track_translation(self):
        core = MetricRGBDOdometry()
        image, points = self.scene.frame()
        rng = np.random.default_rng(133)
        # An initial coordinate anchor alone is not a tracked measurement. None
        # of the next independent noisy frames may supply a fresh accepted pose.
        noise = np.clip(np.rint(90.+rng.normal(0., 1., image.shape)), 0, 255).astype(np.uint8)
        core.update(0., noise, points, 0., 0., 0.)
        for n in range(1, 7):
            noise = np.clip(np.rint(90.+rng.normal(0., 1., image.shape)), 0, 255).astype(np.uint8)
            result = core.update(n*.1, noise, points, 0., 0., 0.)
            self.assertFalse(result['accepted'], result)
            self.assertNotIn('xyz', result)

    def test_uniform_brightness_changes_still_have_no_texture_identity(self):
        core = MetricRGBDOdometry()
        image, points = self.scene.frame()
        for n in range(6):
            result = core.update(n*.1, np.full_like(image, 80+n*4), points, 0., 0., 0.)
            self.assertFalse(result['accepted'])
            self.assertNotIn('xyz', result)

    def test_unobserved_sky_is_not_spatial_support_denominator(self):
        core = MetricRGBDOdometry()
        image, points = self.scene.frame()
        points[:150] = np.nan
        result = core.update(0., image, points, 0., 0., 0.)
        self.assertTrue(result['accepted'], result)
        self.assertEqual(result['reference_support']['current']['measured_roi'], [0, 150, 320, 240])
        cluster = points.copy()
        cluster[:, :290] = np.nan
        self.assertFalse(MetricRGBDOdometry().update(0., image, cluster, 0., 0., 0.)['accepted'])

    def test_two_competing_visual_metric_modes_do_not_use_wheels_to_choose(self):
        core = MetricRGBDOdometry()
        image, points = self.scene.frame()
        self.assertTrue(core.update(0., image, points, 0., 0., 0.)['accepted'])
        ambiguous = points.copy()
        ambiguous[:, :176, 0] += .06
        ambiguous[:, 176:, 0] -= .06
        result = core.update(.1, image, ambiguous, 0., -.60, 0.)
        self.assertFalse(result['accepted'], result)
        self.assertNotIn('xyz', result)
        self.assertIn(result['reason'], ('ambiguous_rgbd_translation', 'rgbd_correspondence_consensus'))

    def test_strong_second_translation_mode_is_rejected_after_refinement(self):
        # Identified feature measurements isolate the estimator from detector
        # sampling. Both spatially distributed peaks are exact 3D consensuses;
        # the 58-of-96 primary peak passes the 55% gate but is still ambiguous.
        rng = np.random.default_rng(344)
        pixels = np.array([(20+80*x+8*(j%4), 20+80*y+14*(j//4))
                           for y in range(3) for x in range(4) for j in range(8)], dtype=float)
        points = np.column_stack((pixels*.01, np.ones(96)))
        features = dict(pixels=pixels, points=points.copy(),
                        covariance=np.repeat((.003**2*np.eye(3))[None], 96, axis=0),
                        descriptors=rng.integers(0,256,(96,32),dtype=np.uint8),
                        image_shape=(240,320), measured_roi=np.array([0,0,320,240]))
        reference = dict(features, world=points.copy(), anchor_covariance=np.zeros((3,3)),id=1)
        current = dict(features, points=points.copy())
        current['points'][:58,0] -= .06
        current['points'][58:,0] += .06
        result, reason = MetricRGBDOdometry()._fit(current, np.array([.06,0.,0.]),reference)
        self.assertIsNone(result)
        self.assertEqual(reason, 'ambiguous_rgbd_translation')

    def test_missing_depth_invalid_shape_and_clock_gap_never_accept(self):
        core = MetricRGBDOdometry()
        image, points = self.scene.frame()
        self.assertFalse(core.update(0., image, points[:-1], 0., 0., 0.)['accepted'])
        self.assertFalse(core.update(0., image, np.full_like(points, np.nan), 0., 0., 0.)['accepted'])
        self.assertTrue(core.update(0., image, points, 0., 0., 0.)['accepted'])
        for stamp in (0., -.1, 1.1, 1.2):
            result = core.update(stamp, image, points, 0., 0., 0.)
            self.assertFalse(result['accepted'])
            self.assertNotIn('xyz', result)
        core.close()
        self.assertEqual(core.update(.1, image, points, 0., 0., 0.)['reason'], 'rgbd_core_closed')

    def test_initial_moving_is_latched(self):
        core = MetricRGBDOdometry()
        image, points = self.scene.frame()
        self.assertFalse(core.update(0., image, points, 0., .02, 0.)['accepted'])
        self.assertEqual(core.update(.1, image, points, 0., 0., 0.)['reason'],
                         'initialization_motion_requires_new_session')

    def test_missing_wheel_hint_cannot_initialize_or_bypass_initial_motion_latch(self):
        core=MetricRGBDOdometry();image,points=self.scene.frame()
        result=core.update(0.,image,points,0.,None,None)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'],'missing_wheel_hint_for_initialization')
        self.assertFalse(result['wheel_prediction_used'])
        self.assertIsNone(core.stamp)
        self.assertEqual(len(core.references),0)
        self.assertFalse(core.update(.1,image,points,0.,.02,0.)['accepted'])
        result=core.update(.2,image,points,0.,None,None)
        self.assertEqual(result['reason'],'initialization_motion_requires_new_session')
        self.assertNotIn('xyz',result)

    def test_missing_wheel_hint_measures_translation_z_and_side_slip(self):
        for direction in (np.array([.025,.0125,.001]),np.array([0.,.025,.001])):
            core=MetricRGBDOdometry();image,points=self.scene.frame()
            self.assertTrue(core.update(0.,image,points,0.,0.,0.)['accepted'])
            for number in range(1,9):
                truth=number*direction;image,points=self.scene.frame(truth)
                result=core.update(number*.1,image,points,0.,None,None)
                self.assertTrue(result['accepted'],(number,result))
                self.assertFalse(result['wheel_prediction_used'])
                np.testing.assert_allclose(result['xyz'],truth,atol=.006)

    def test_missing_wheel_hint_after_reject_uses_last_registered_pose(self):
        core=MetricRGBDOdometry();image,points=self.scene.frame()
        self.assertTrue(core.update(0.,image,points,0.,0.,0.)['accepted'])
        previous=core.xyz.copy();real_fit=core._fit
        core._fit=lambda *args:(None,'test_rejected_identity')
        result=core.update(.1,image,points,0.,.5,0.)
        self.assertFalse(result['accepted']);self.assertTrue(result['wheel_prediction_used'])
        self.assertNotIn('xyz',result)
        np.testing.assert_array_equal(core.xyz,previous)
        core._fit=real_fit
        image,points=self.scene.frame(body=(0.,.025,0.))
        result=core.update(.2,image,points,0.)
        self.assertTrue(result['accepted'],result)
        self.assertFalse(result['wheel_prediction_used'])
        np.testing.assert_allclose(result['xyz'],[0.,.025,0.],atol=.006)
        previous=core.xyz.copy();stamp=core.stamp
        for vx,wz in ((None,0.),(0.,None),(float('nan'),0.),(0.,float('inf'))):
            result=core.update(.3,image,points,0.,vx,wz)
            self.assertFalse(result['accepted']);self.assertFalse(result['wheel_prediction_used'])
            self.assertEqual(result['reason'],'invalid_rgbd_input')
            self.assertEqual(core.stamp,stamp)
            np.testing.assert_array_equal(core.xyz,previous)

    def test_missing_wheel_hint_long_range_uses_measured_keyframes(self):
        core=MetricRGBDOdometry();image,points=self.scene.frame()
        self.assertTrue(core.update(0.,image,points,0.,0.,0.)['accepted'])
        origin=core.references[0]
        errors=[]
        for number in range(1,141):
            truth=np.array([number*.05,0.,0.]);image,points=self.scene.frame(truth)
            result=core.update(number*.1,image,points,0.,None,None)
            self.assertTrue(result['accepted'],(number,result))
            self.assertFalse(result['wheel_prediction_used'])
            self.assertLessEqual(len(core.references),20)
            errors.append(np.linalg.norm(result['xyz']-truth))
        self.assertGreater(len(core.references),5)
        self.assertIs(core.references[0],origin)
        self.assertLess(max(errors),.03)

    def test_missing_wheel_hint_static_noise_does_not_grow_or_generate_pose(self):
        core=MetricRGBDOdometry();image,points=self.scene.frame()
        self.assertTrue(core.update(0.,image,points,0.,0.,0.)['accepted'])
        for number in range(1,13):
            image,points=self.scene.frame(noise=.0005,seed=number)
            result=core.update(number*.1,image,points,0.,None,None)
            self.assertTrue(result['accepted'],result)
            self.assertFalse(result['wheel_prediction_used'])
            self.assertLess(np.linalg.norm(result['xyz']),.002)
        self.assertEqual(len(core.references),1)
        result=core.update(1.3,np.full_like(image,90),points,0.,None,None)
        self.assertFalse(result['accepted']);self.assertFalse(result['wheel_prediction_used'])
        self.assertNotIn('xyz',result)

    def test_missing_wheel_hint_imu_turn_admits_measured_keyframe(self):
        core=MetricRGBDOdometry();image,points=self.scene.frame()
        self.assertTrue(core.update(0.,image,points,0.,0.,0.)['accepted'])
        for number in range(1,11):
            yaw=number*.035;image,points=self.scene.frame(camera_yaw=yaw)
            result=core.update(number*.1,image,points,yaw,None,None)
            self.assertTrue(result['accepted'],(number,result))
            self.assertFalse(result['wheel_prediction_used'])
            self.assertLess(np.linalg.norm(result['xyz']),.015)
        self.assertGreater(len(core.references),1)

    def test_optional_camera_model_is_validated_and_copied_without_pose_feedback(self):
        image, points = self.scene.frame()
        model=dict(K=np.array([[277.,0.,160.],[0.,277.,120.],[0.,0.,1.]]),
                   rotation=np.eye(3),translation=np.array([.45,0.,.24]))
        core = MetricRGBDOdometry()
        result=core.update(0.,image,points,0.,0.,0.,camera_model=model)
        self.assertTrue(result['accepted'],result)
        model['translation'][:]=99.
        np.testing.assert_allclose(core.last_camera_model['translation'],[.45,0.,.24])
        np.testing.assert_allclose(result['xyz'],0.,atol=1e-12)
        # Valid pose/search metadata cannot replace a measured image identity.
        result=core.update(.1,np.full_like(image,90),points,0.,0.,0.,camera_model=model)
        self.assertFalse(result['accepted'])
        self.assertNotIn('xyz',result)

    def test_invalid_camera_model_never_initializes_or_renews_stamp(self):
        image, points = self.scene.frame()
        models=[{},dict(K=np.eye(3),rotation=np.eye(2),translation=np.zeros(3)),
                dict(K=np.eye(3),rotation=np.diag([-1.,1.,1.]),translation=np.zeros(3)),
                dict(K=np.eye(3),rotation=np.eye(3),translation=np.array([np.nan,0.,0.]))]
        for model in models:
            core=MetricRGBDOdometry()
            result=core.update(0.,image,points,0.,0.,0.,camera_model=model)
            self.assertEqual(result['reason'],'invalid_rgbd_camera_model')
            self.assertIsNone(core.stamp)
            self.assertEqual(len(core.references),0)

    def test_camera_view_handoff_reuses_registered_views_with_small_metric_centroid_change(self):
        scene=PinholePlane();core=MetricRGBDOdometry();first=None;counts=[]
        passes=(list(np.linspace(35.,50.,16)),list(np.linspace(49.,35.,15)),list(np.linspace(36.,50.,15)))
        sequence=0
        for sweep in passes:
            for angle in sweep:
                image,points,model=scene.frame(angle)
                if first is None:first=np.nanmean(points,axis=(0,1))
                result=core.update(sequence*.1,image,points,0.,0.,0.,camera_model=model)
                self.assertTrue(result['accepted'],(sequence,angle,result))
                self.assertLess(np.linalg.norm(result['xyz']),.015)
                sequence+=1
            counts.append(len(core.references))
        self.assertLess(np.linalg.norm(first-np.nanmean(points,axis=(0,1))),.30)
        self.assertGreaterEqual(counts[0],2)
        self.assertEqual(counts[1:],counts[:1]*2)

    def test_static_camera_model_does_not_grow_view_references(self):
        scene=PinholePlane();core=MetricRGBDOdometry();image,points,model=scene.frame(40.)
        for index in range(8):
            result=core.update(index*.1,image,points,0.,0.,0.,camera_model=model)
            self.assertTrue(result['accepted'],result)
        self.assertEqual(len(core.references),1)

    def test_calibrated_patch_scale_hands_off_before_old_view_is_exhausted(self):
        scene=PinholePlane();core=MetricRGBDOdometry()
        for index,angle in enumerate((35.,36.,37.,38.,39.)):
            image,points,model=scene.frame(angle)
            wheel=(0.,0.) if index==0 else (None,None)
            result=core.update(index*.1,image,points,0.,*wheel,camera_model=model)
            self.assertTrue(result['accepted'],(angle,result))
            self.assertLess(np.linalg.norm(result['xyz']),.015)
        # At this focal length, four degrees moves the measured image beyond
        # half a 31-pixel descriptor patch but remains below the old .10-rad
        # policy. An accepted RGBD fit must already have registered a new view.
        self.assertGreaterEqual(len(core.references),2)
        count=len(core.references)
        for index,angle in enumerate((39.01,38.99,39.,39.01),start=5):
            image,points,model=scene.frame(angle)
            result=core.update(index*.1,image,points,0.,None,None,camera_model=model)
            self.assertTrue(result['accepted'],result)
        self.assertEqual(len(core.references),count)
        image,points,model=scene.frame(43.)
        result=core.update(.9,image,points-np.array([.25,0.,0.]),0.,None,None,camera_model=model)
        self.assertFalse(result['accepted']);self.assertNotIn('xyz',result)
        self.assertEqual(len(core.references),count)

    def test_conflicting_registered_view_rejects_without_pose_or_reference_commit(self):
        scene=PinholePlane();core=MetricRGBDOdometry();image,points,model=scene.frame(35.)
        first=core.update(0.,image,points,0.,0.,0.,camera_model=model)
        self.assertTrue(first['accepted'],first)
        features=core._features(image,points)
        # Fault injection: registered image identities with incompatible stored
        # anchors. Camera metadata chooses which prior view is checked; it never
        # supplies the pose. The original origin/latest fits agree, while the
        # independently qualified older view has a strong 4 cm contradiction.
        origin=core.references[0]
        origin['camera_model']=core._camera_model(scene.frame(5.)[2])
        old=core._reference(features,np.eye(3)*.001**2,origin['id'])
        old['world'][:,0]+=.04;old['self_overlap']=first['overlap']
        latest=core._reference(features,np.eye(3)*.001**2,origin['id'])
        latest['camera_model']=core._camera_model(scene.frame(80.)[2]);latest['self_overlap']=first['overlap']
        core.references.extend([old,latest]);core._activate(latest)
        core.xyz=np.array([.002,0.,0.]);before=core.xyz.copy()
        identities=[id(reference) for reference in core.references];number=core.reference_number
        result=core.update(.1,image,points,0.,0.,0.,camera_model=model)
        self.assertFalse(result['accepted'],result)
        self.assertEqual(result['reason'],'ambiguous_rgbd_reference_translation')
        self.assertNotIn('xyz',result)
        np.testing.assert_array_equal(core.xyz,before)
        self.assertEqual([id(reference) for reference in core.references],identities)
        self.assertEqual(core.reference_number,number)
        self.assertEqual(core._active_view_id,latest['id'])

    def test_projected_parallax_hands_off_small_translation_and_reuses_return(self):
        scene=PinholePlane();core=MetricRGBDOdometry();counts=[];sequence=0
        origin=None
        passes=(np.arange(0.,.281,.01),np.arange(.27,-.001,-.01),np.arange(.01,.281,.01))
        for sweep in passes:
            for distance in sweep:
                body=np.array([distance,0.,0.]);image,points,model=scene.frame(40.,body=body)
                wheel=(0.,0.) if sequence==0 else (None,None)
                result=core.update(sequence*.1,image,points,0.,*wheel,camera_model=model)
                self.assertTrue(result['accepted'],(sequence,distance,result))
                np.testing.assert_allclose(result['xyz'],body,atol=.012)
                self.assertFalse(result['wheel_prediction_used'])
                self.assertLessEqual(len(core.references),20)
                if origin is None:origin=core.references[0]
                self.assertIs(core.references[0],origin)
                sequence+=1
            counts.append(len(core.references))
        self.assertGreater(counts[0],1)  # All body displacements stay below .30m.
        self.assertEqual(counts[1:],counts[:1]*2)
        # Fresh stationary exposures after motion cannot append more anchors.
        for _ in range(8):
            result=core.update(sequence*.1,image,points,0.,None,None,camera_model=model)
            self.assertTrue(result['accepted'],result);sequence+=1
        self.assertEqual(len(core.references),counts[-1])

    def test_invalid_or_insufficient_projection_never_proves_same_view(self):
        scene=PinholePlane();core=MetricRGBDOdometry();image,points,model=scene.frame(40.)
        first=core.update(0.,image,points,0.,0.,0.,camera_model=model)
        self.assertTrue(first['accepted'],first)
        original=core.references[0];copy=dict(original)
        behind=model['translation']-model['rotation'][:,2]
        copy['world']=np.tile(behind,(len(original['world']),1))
        self.assertFalse(core._near_view(copy))
        self.assertEqual(core._projected_view_motion(copy)['visible_reference_features'],0)
        copy['world'][:23]=original['world'][:23]
        self.assertFalse(core._near_view(copy))
        self.assertIsNone(core._projected_view_motion(copy)['projected_median_shift_px'])
        copy['world'][:]=np.nan
        self.assertFalse(core._near_view(copy))
        result=core.update(.1,image,points,0.,None,None,camera_model={'K':model['K']})
        self.assertEqual(result['reason'],'invalid_rgbd_camera_model')
        self.assertEqual(len(core.references),1)
        np.testing.assert_array_equal(core.xyz,np.zeros(3))

    def test_twenty_colocated_views_select_actual_visible_old_view_within_five_attempts(self):
        scene=PinholePlane();core=MetricRGBDOdometry()
        image,points,model=scene.frame(20.)
        self.assertTrue(core.update(0.,image,points,0.,0.,0.,camera_model=model)['accepted'])
        origin=core.references[0];target=None
        # Measured camera views share a body position up to submillimeter pose
        # differences. These differences cannot rank a 45deg image behind
        # incompatible views just because their anchors are slightly closer.
        angles=[21.,22.,23.,24.,25.,26.,27.,28.,29.,45.,55.,58.,61.,64.,67.,70.,73.,76.,80.]
        for index,angle in enumerate(angles,1):
            body=np.array([index*.00002,0.,0.]);image,points,model=scene.frame(angle,body)
            features=core._features(image,points);self.assertIsNotNone(features)
            core.xyz=body;core.last_camera_model=core._camera_model(model)
            reference=core._reference(features,np.eye(3)*.003**2,origin['id'])
            reference['self_overlap']=1.;core.references.append(reference)
            if angle==45.:target=reference
        self.assertEqual(len(core.references),20);self.assertIsNotNone(target)
        core.xyz=np.zeros(3);core._activate(target)
        image,points,model=scene.frame(45.)
        original_order=[ref['id'] for ref in core.references]
        remote=core.references[1]
        for index,active in enumerate((target['id'],999999,remote['id'],remote['id']),1):
            if active==remote['id']:
                # A far, behind-camera anchor with the same angle must not
                # outrank the actually visible local view, even if once active.
                remote['camera_model']=core._camera_model(model)
                remote['xyz']=(np.array([-100.,0.,0.]) if index==3 else
                               model['rotation'][:,2]*100.)
                remote['world']=remote['points']+remote['xyz']
                # The last case is in the RGB field of view but beyond the
                # actual metric-depth6m range; it must not gain active priority.
                if index==4:
                    motion=core._projected_view_motion(remote)
                    self.assertEqual(motion['visible_reference_features'],0)
            core._active_view_id=active
            result=core.update(index*.1,image,points,0.,None,None,camera_model=model)
            self.assertTrue(result['accepted'],result)
            np.testing.assert_allclose(result['xyz'],np.zeros(3),atol=.008)
            attempts=result['reference_attempts']
            self.assertLessEqual(len(attempts),5)
            self.assertEqual([row['reference_id'] for row in attempts[:2]],
                             [origin['id'],core.references[-1]['id']])
            self.assertEqual(attempts[2]['reference_id'],target['id'])
            self.assertTrue(attempts[2]['accepted'])
            self.assertEqual([ref['id'] for ref in core.references],original_order)
            self.assertEqual(core._active_view_id,target['id'])
        before=core._active_view_id
        result=core.update(.5,np.full_like(image,90),points,0.,None,None,camera_model=model)
        self.assertFalse(result['accepted']);self.assertEqual(result['reference_attempts'],[])
        self.assertEqual(core._active_view_id,before)
        self.assertEqual([ref['id'] for ref in core.references],original_order)

    def test_reference_attempt_receipt_is_bounded_fresh_and_keeps_original_failures(self):
        core=MetricRGBDOdometry();image,points=self.scene.frame()
        first=core.update(0.,image,points,0.,0.,0.)
        self.assertEqual(len(first['reference_attempts']),1)
        self.assertTrue(first['reference_attempts'][0]['accepted'])
        image,points=self.scene.frame(body=(.25,0.,0.))
        rejected=core.update(.1,image,points,0.,None,None)
        self.assertFalse(rejected['accepted'])
        self.assertEqual(len(rejected['reference_attempts']),1)
        self.assertEqual(rejected['reference_attempts'][0]['reason'],'rgbd_correction_jump')
        self.assertFalse(rejected['reference_attempts'][0]['accepted'])
        # Receipt dictionaries do not alias later calls or a caller's changes.
        rejected['reference_attempts'][0]['reason']='caller_changed_copy'
        image,points=self.scene.frame()
        accepted=core.update(.2,image,points,0.,None,None)
        self.assertTrue(accepted['accepted']);self.assertEqual(accepted['reference_attempts'][0]['reason'],'accepted')
        core.close()
        closed=core.update(.3,image,points,0.,None,None)
        self.assertEqual(closed['reference_attempts'],[])

    def test_mutual_knn_backward_subset_preserves_original_pairs_and_ties(self):
        rng=np.random.default_rng(991);core=MetricRGBDOdometry()
        for trial in range(4):
            reference=rng.integers(0,256,(500,32),dtype=np.uint8)
            current=rng.integers(0,256,(500,32),dtype=np.uint8)
            current[:80]=reference[:80];current[0]=current[1];reference[8]=reference[9]
            features=dict(descriptors=current,pixels=rng.uniform([0.,0.],[319.,239.],(500,2)),
                          measured_roi=np.array([0,0,320,240]))
            # Frozen mutual-match definition is the independent equivalence
            # oracle, including descriptor ties, ratio, sort, and density cap.
            matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
            forward=matcher.knnMatch(current,reference,k=2)
            backward=matcher.knnMatch(reference,current,k=2)
            reverse={p[0].queryIdx:p[0].trainIdx for p in backward if len(p)==2
                     and p[0].distance<=48 and p[0].distance<.70*p[1].distance}
            pairs=[p[0] for p in forward if len(p)==2 and p[0].distance<=48
                   and p[0].distance<.70*p[1].distance and reverse.get(p[0].trainIdx)==p[0].queryIdx]
            pairs.sort(key=lambda match:(match.distance,match.queryIdx))
            counts={};expected=[]
            for match in pairs:
                u,v=features['pixels'][match.queryIdx];cell=(min(3,int(u/320*4)),min(2,int(v/240*3)))
                if counts.get(cell,0)<8:
                    counts[cell]=counts.get(cell,0)+1;expected.append((match.queryIdx,match.trainIdx))
            np.testing.assert_array_equal(core._matches(features,dict(descriptors=reference)),
                                          np.asarray(expected,dtype=int).reshape(-1,2))

    def test_wrong_depth_correspondence_and_jump_rejected(self):
        image, points = self.scene.frame()
        core = MetricRGBDOdometry()
        self.assertTrue(core.update(0., image, points, 0., 0., 0.)['accepted'])
        scrambled = points.copy()
        scrambled[:, :, 0] += .055*np.sin(np.mgrid[:240, :320][1]*.17)
        result = core.update(.1, image, scrambled, 0., 0., 0.)
        self.assertFalse(result['accepted'], result)
        self.assertNotIn('xyz', result)
        jumped = points-np.array([.25, 0., 0.])
        result = core.update(.2, image, jumped, 0., 0., 0.)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'], 'rgbd_correction_jump')

    def test_spatially_collinear_features_do_not_prove_translation(self):
        image, points = self.scene.frame()
        points[:, :, 1] = 0.
        self.assertEqual(MetricRGBDOdometry().update(0., image, points, 0., 0., 0.)['reason'],
                         'insufficient_rgbd_spatial_support')

    @staticmethod
    def _identified_precision_fixture():
        # Actual identified measurements isolate support qualification from ORB
        # sampling; descriptors still pass the original mutual/ratio matcher.
        rng = np.random.default_rng(821)
        pixels = np.array([(20+80*x+8*(j%4), 20+80*y+14*(j//4))
                           for y in range(3) for x in range(4) for j in range(8)], dtype=float)
        points = np.column_stack((.4+pixels[:, 0]*.002, (pixels[:, 1]-120)*.003,
                                  np.full(96, .5)))
        features = dict(pixels=pixels, points=points,
                        covariance=np.repeat((.003**2*np.eye(3))[None], 96, axis=0),
                        descriptors=rng.integers(0,256,(96,32),dtype=np.uint8),
                        image_shape=(240,320), measured_roi=np.array([0,0,320,240]))
        return features

    def test_precision_support_keeps_near_metric_diversity_with_far_weak_returns(self):
        core = MetricRGBDOdometry(); reference = self._identified_precision_fixture()
        # Two distant observations per image cell have long attitude levers and
        # coarser metric pixel uncertainty. They elongate unweighted full PCA.
        far = np.arange(96)%8 >= 6
        reference['points'][far, 0] += 5.
        reference['covariance'][far] = .02**2*np.eye(3)
        reference.update(world=reference['points'].copy(), anchor_covariance=np.zeros((3,3)), id=1)
        truth = np.array([.025,.01,.005])
        current = dict(reference, points=reference['points']-truth)
        self.assertFalse(core._spatial_support(reference, np.arange(96))[0])
        fit, reason = core._fit(current, np.zeros(3), reference)
        self.assertEqual(reason, 'accepted'); self.assertIsNotNone(fit)
        np.testing.assert_allclose(fit['xyz'], truth, atol=1e-12)
        self.assertEqual(fit['inliers'], 96)
        self.assertEqual(fit['reference_support']['precision_support']['selected'], 72)
        for support in (fit['reference_support']['current'], fit['reference_support']['reference']):
            self.assertEqual(support['image_grid_cells'], 12)
            self.assertTrue(np.all(np.asarray(support['image_span']) >= .30))
            eigen = support['metric_support_eigenvalues_m2']
            self.assertGreaterEqual(eigen[1], .0025)
            self.assertGreaterEqual(eigen[1], .02*eigen[2])

    def test_precision_support_cannot_borrow_weak_only_image_cells(self):
        core = MetricRGBDOdometry(); reference = self._identified_precision_fixture()
        # Only the first five image cells have precise returns. Full support
        # has twelve cells but its weak long levers fail metric support; the
        # precision fallback cannot borrow those extra image cells.
        reference['covariance'][40:] = .025**2*np.eye(3)
        reference['points'][40:, 0] += 5.
        reference.update(world=reference['points'].copy(), anchor_covariance=np.zeros((3,3)), id=1)
        current = dict(reference, points=reference['points']-np.array([.02,0.,0.]))
        self.assertFalse(core._spatial_support(reference, np.arange(96))[0])
        selected, details = core._precision_support(current, reference, np.arange(96), np.arange(96), np.ones(96,dtype=bool))
        self.assertEqual(details['selected'], 40)
        self.assertEqual(core._spatial_support(reference, selected)[1]['image_grid_cells'], 5)
        fit, reason = core._fit(current, np.zeros(3), reference)
        self.assertIsNone(fit); self.assertEqual(reason, 'insufficient_rgbd_spatial_support')

    def test_equal_precision_support_retains_every_identity_and_uses_body_levers(self):
        core = MetricRGBDOdometry(); features = self._identified_precision_fixture()
        # Equal measurement covariance and equal lever norms give exactly the
        # old full support; unrelated registered world position cannot rank it.
        angles = np.linspace(0.,2*np.pi,96,endpoint=False)
        features['points'] = np.column_stack((np.cos(angles),np.sin(angles),np.full(96,.5)))
        reference = dict(features, world=features['points']+np.array([100.,-50.,3.]))
        all_indices = np.arange(96); mask = np.ones(96,dtype=bool)
        selected, details = core._precision_support(features, reference, all_indices, all_indices, mask)
        np.testing.assert_array_equal(selected, all_indices)
        self.assertEqual(details['selected'], 96)
        current_support = core._spatial_support(features, selected)
        self.assertEqual(current_support, core._spatial_support(features, all_indices))

    def test_precision_tier_cannot_borrow_weak_metric_diversity_or_fewer_than_24(self):
        for precise_count in (16, 48):
            core = MetricRGBDOdometry(); reference = self._identified_precision_fixture()
            reference['covariance'][precise_count:] = .025**2*np.eye(3)
            reference['points'][precise_count:, 0] += 5.
            if precise_count == 48:
                # Image coverage alone cannot replace actual 2D metric support.
                reference['points'][:precise_count, 1] = 0.
            reference.update(world=reference['points'].copy(), anchor_covariance=np.zeros((3,3)), id=1)
            current = dict(reference, points=reference['points']-np.array([.02,0.,0.]))
            self.assertFalse(core._spatial_support(reference, np.arange(96))[0])
            fit, reason = core._fit(current, np.zeros(3), reference)
            self.assertIsNone(fit); self.assertEqual(reason, 'insufficient_rgbd_spatial_support')

    def test_precision_threshold_uses_identity_measurements_before_consensus(self):
        core = MetricRGBDOdometry(); features = self._identified_precision_fixture()
        indices = np.arange(96); all_inliers = np.ones(96,dtype=bool)
        features['covariance'][1:] = .025**2*np.eye(3)
        selected, first = core._precision_support(features, features, indices, indices, all_inliers)
        self.assertEqual(selected.tolist(), [0])
        # Losing the best measured identity to metric consensus cannot enlarge
        # the precision tier until the remaining imprecise support passes.
        all_inliers[0] = False
        selected, second = core._precision_support(features, features, indices, indices, all_inliers)
        self.assertEqual(selected.tolist(), [])
        self.assertEqual(first['pair_sigma_limit_m'], second['pair_sigma_limit_m'])

    def test_full_support_remains_sufficient_when_precision_layer_is_sparse(self):
        core = MetricRGBDOdometry(); reference = self._identified_precision_fixture()
        reference['covariance'][40:] = .025**2*np.eye(3)
        reference.update(world=reference['points'].copy(), anchor_covariance=np.zeros((3,3)), id=1)
        current = dict(reference, points=reference['points']-np.array([.02,0.,0.]))
        all_indices = np.arange(96)
        selected, _ = core._precision_support(current, reference, all_indices, all_indices, np.ones(96,dtype=bool))
        self.assertFalse(core._spatial_support(reference, selected)[0])
        self.assertTrue(core._spatial_support(reference, all_indices)[0])
        fit, reason = core._fit(current, np.zeros(3), reference)
        self.assertEqual(reason, 'accepted'); self.assertIsNotNone(fit)
        self.assertEqual(fit['reference_support']['spatial_support_layer'], 'full_identity_inliers')
        self.assertIsNone(fit['reference_support']['precision_support'])

    def test_two_support_layers_cannot_mix_different_frames_evidence(self):
        first = self._identified_precision_fixture(); pixels = first['pixels'].copy()
        # The same 40 precise identities cover five cells in one exposure and
        # six in the other. Far identities complete full image coverage.
        near = np.array([cell*8+j for cell in range(6) for j in range(7)][:40])
        remaining = np.setdiff1d(np.arange(96), near)
        first['pixels'] = pixels[np.r_[near, remaining]]
        indices = np.arange(96)
        first['points'] = np.column_stack((.4+(indices%5)*.1+np.where(indices>=40,5.,0.),
            (indices%8-3.5)*.12*1.2689193172518967, np.full(96,.5)))
        first['covariance'][40:] = .025**2*np.eye(3)
        second = dict(first, pixels=pixels, points=first['points']-np.array([.02,0.,0.]))
        # Submillimetre measured differences straddle the original 0.02 PCA
        # threshold; consensus still agrees on all 96 genuine identities.
        second['points'][:,1] *= 1.001
        for reference_features, current in ((first, second), (second, first)):
            core = MetricRGBDOdometry()
            reference = dict(reference_features, world=reference_features['points'].copy(),
                             anchor_covariance=np.zeros((3,3)), id=1)
            pairs = core._matches(current, reference)
            self.assertEqual(len(pairs),96)
            selected, _ = core._precision_support(current, reference, *pairs.T, np.ones(96,dtype=bool))
            full = [core._spatial_support(features, pairs[:,axis])[0]
                    for axis,features in enumerate((current,reference))]
            precise = [core._spatial_support(features, pairs[selected,axis])[0]
                       for axis,features in enumerate((current,reference))]
            self.assertEqual(sorted(full),[False,True])
            self.assertEqual(precise,[not good for good in full])
            fit, reason = core._fit(current,np.zeros(3),reference)
            self.assertIsNone(fit); self.assertEqual(reason,'insufficient_rgbd_spatial_support')

    def test_weak_competing_mode_is_not_hidden_by_precision_support(self):
        core = MetricRGBDOdometry(); reference = self._identified_precision_fixture()
        reference['covariance'][58:] = .008**2*np.eye(3)
        reference.update(world=reference['points'].copy(), anchor_covariance=np.zeros((3,3)), id=1)
        current = dict(reference, points=reference['points'].copy())
        current['points'][:58,0] -= .06
        current['points'][58:,0] += .06
        # The second identity mode is weaker but has 38 genuine observations.
        # It must be rejected before precision support can drop its members.
        fit, reason = core._fit(current, np.array([.06,0.,0.]), reference)
        self.assertIsNone(fit); self.assertEqual(reason, 'ambiguous_rgbd_translation')

    def test_support_handoff_uses_each_exposures_roi_and_requires_real_parallax(self):
        core = MetricRGBDOdometry(); reference = dict(id=7)
        core._frame_fits[7] = dict(reference_support=dict(
            current=dict(image_span=[.4,.5],measured_roi=[10,20,330,120]),
            reference=dict(image_span=[.34,.5],measured_roi=[0,0,100,240])))
        core._projected_view_motion = lambda ref:dict(projected_median_shift_px=7.74)
        result = core._support_handoff(reference)
        self.assertAlmostEqual(result['minimum_span_margin_px'],4.)
        self.assertFalse(result['needed'])
        core._projected_view_motion = lambda ref:dict(projected_median_shift_px=7.75)
        self.assertTrue(core._support_handoff(reference)['needed'])
        del core._frame_fits[7]
        self.assertFalse(core._support_handoff(reference)['needed'])

    def test_stationary_fresh_measurement_noise_near_span_boundary_adds_no_anchors(self):
        scene = PinholePlane(); core = MetricRGBDOdometry(); rng = np.random.default_rng(184)
        accepted, minimum_margin, maximum_parallax = 0, np.inf, 0.
        for number in range(31):
            image, points, model = scene.frame(35.)
            # A genuinely textured strip leaves spatial coverage close to its
            # boundary. Independently timed RGB/depth/attitude noise changes
            # the measured feature subset; no wheel hint claims zero motion.
            image[:60] = 90; image[160:] = 90
            if number:
                image = np.clip(image.astype(float)+rng.normal(0,.25,image.shape),0,255).astype(np.uint8)
                points += rng.normal(0,.0005,points.shape)
            yaw = np.deg2rad(.02)*np.sin(number)
            c,s = np.cos(yaw),np.sin(yaw)
            orientation = np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
            points = points@orientation.T
            model['rotation'] = orientation@model['rotation']
            model['translation'] = orientation@model['translation']
            result = core.update(number*.1,image,points,yaw,
                0. if number == 0 else None,0. if number == 0 else None,camera_model=model)
            self.assertEqual(len(core.references),1)
            if result['accepted']:
                accepted += 1
                for side in ('current','reference'):
                    support = result['reference_support'][side]
                    dimensions = np.array(support['measured_roi'][2:])-support['measured_roi'][:2]
                    minimum_margin = min(minimum_margin,*((np.array(support['image_span'])-.30)*dimensions))
                self.assertLess(np.linalg.norm(result['xyz']),.01)
            else:
                self.assertNotIn('xyz',result)
            shift = core._projected_view_motion(core.references[0])['projected_median_shift_px']
            maximum_parallax = max(maximum_parallax,shift or 0.)
        self.assertGreaterEqual(accepted,25)
        self.assertLess(minimum_margin,15.5)
        self.assertLess(maximum_parallax,1.)

    def test_support_margin_hands_off_before_half_patch_and_reuses_real_revisits(self):
        scene = PinholePlane(); core = MetricRGBDOdometry(); handoff = None
        for number,x in enumerate((0.,.01,.02,.03,0.,.03,0.,.03)):
            image,points,model = scene.frame(35.,body=(x,0.,0.))
            image[:60] = 90; image[180:] = 90
            result = core.update(number*.1,image,points,0.,
                0. if number == 0 else None,0. if number == 0 else None,camera_model=model)
            self.assertTrue(result['accepted'],result)
            np.testing.assert_allclose(result['xyz'],[x,0.,0.],atol=.006)
            self.assertEqual(len(core.references),1 if number < 3 else 2)
            if number == 3: handoff = result['view_admission']['support_handoff']
        self.assertTrue(handoff['needed'])
        self.assertLess(handoff['projected_reference_motion']['projected_median_shift_px'],15.5)
        self.assertGreaterEqual(handoff['projected_reference_motion']['projected_median_shift_px'],7.75)

    def test_selected_real_handoff_is_not_suppressed_by_unfitted_active_view(self):
        scene = PinholePlane(); core = MetricRGBDOdometry()
        image,points,model = scene.frame(35.)
        image[:60] = 90; image[180:] = 90
        self.assertTrue(core.update(0.,image,points,0.,0.,0.,camera_model=model)['accepted'])
        origin = core.references[0]
        moved_image,moved_points,_ = scene.frame(35.,body=(.03,0.,0.))
        moved_image[:60] = 90; moved_image[180:] = 90
        core.xyz = np.array([.03,0.,0.])
        active = core._reference(core._features(moved_image,moved_points),np.eye(3)*.0001,origin['id'])
        active['self_overlap'] = 1.
        core.xyz = np.zeros(3)
        latest = core._reference(core._features(image,points),np.eye(3)*.0002,origin['id'])
        latest['descriptors'][:] = 0
        latest['self_overlap'] = 1.
        core.references.extend((active,latest)); core._activate(active)
        # Origin/latest are the bounded tracking attempts. The older activated
        # view has no fit cache entry until admission independently checks it.
        calls = []; original_install = core._install_reference
        def record_install(*args):
            calls.append(tuple(core._frame_fits))
            return original_install(*args)
        core._install_reference = record_install
        result = core.update(.1,moved_image,moved_points,0.,None,None,camera_model=model)
        self.assertTrue(result['accepted'],result)
        self.assertEqual(result['reference_id'],origin['id'])
        self.assertTrue(calls)
        self.assertNotIn(active['id'],calls[0])
        self.assertTrue(result['view_admission']['support_handoff']['needed'])
        self.assertEqual(result['view_admission']['support_handoff']['reference_id'],origin['id'])
        self.assertIsNotNone(core._frame_fits[active['id']])
        self.assertEqual(core._active_view_id,active['id'])
        self.assertEqual(len(core.references),3)

    def test_identity_pixel_scale_metadata_comes_from_actual_both_frame_matches(self):
        core = MetricRGBDOdometry(); image,points = self.scene.frame()
        previous = core._features(image,points)
        current = core._features(image,points)
        self.assertEqual(len(current['pixel_scales']),len(current['points']))
        self.assertTrue(np.all(current['pixel_scales'] >= 1.))
        reference = core._reference(previous,np.zeros((3,3)),None)
        pairs = core._matches(current,reference)
        fit,reason = core._fit(current,np.zeros(3),reference)
        self.assertEqual(reason,'accepted')
        scales = fit['reference_support']['identity_pixel_scale_max']
        self.assertEqual(scales['current'],float(current['pixel_scales'][pairs[:,0]].max()))
        self.assertEqual(scales['reference'],float(reference['pixel_scales'][pairs[:,1]].max()))

    def test_residual_reserve_uses_different_actual_scales_and_pair_pixel_noise(self):
        core = MetricRGBDOdometry(); reference = dict(id=7)
        scales = dict(current=1.2,reference=1.2**5)
        core._frame_fits[7] = dict(rmse_m=.0122,reference_support=dict(
            current=dict(image_span=[.8,.8],measured_roi=[0,0,320,240]),
            reference=dict(image_span=[.8,.8],measured_roi=[0,0,320,240]),
            identity_pixel_scale_max=scales))
        budget = 3*.5*np.sqrt(scales['current']**2+scales['reference']**2)
        core._projected_view_motion = lambda ref:dict(projected_median_shift_px=budget-.001)
        result = core._support_handoff(reference)
        self.assertAlmostEqual(result['residual_minimum_parallax_px'],budget)
        self.assertAlmostEqual(result['residual_margin_m'],.0028)
        self.assertFalse(result['needed'])
        core._projected_view_motion = lambda ref:dict(projected_median_shift_px=budget+.001)
        result = core._support_handoff(reference)
        self.assertTrue(result['needed']); self.assertTrue(result['residual_triggered'])
        self.assertFalse(result['span_triggered'])
        core._frame_fits[7]['rmse_m'] = .0119
        self.assertFalse(core._support_handoff(reference)['needed'])

    def test_static_near_residual_gate_with_independent_sensor_noise_does_not_grow(self):
        scene = PinholePlane(); core = MetricRGBDOdometry(); rng = np.random.default_rng(382)
        accepted,near_limit,maximum_parallax = 0,0,0.
        for number in range(21):
            image,points,model = scene.frame(0.)
            if number:
                image = np.clip(image.astype(float)+rng.normal(0,.6,image.shape),0,255).astype(np.uint8)
            optical = (points-model['translation'])@model['rotation']
            rays = optical/optical[:,:,2:]
            # Independent 12mm depth noise along the actual optical ray leaves
            # static reprojection meaningful. It is deliberately harsher than
            # the 3mm floor and produces accepted residuals near the 15mm gate.
            optical += rays*rng.normal(0,.012,optical.shape[:2])[:,:,None]
            valid = np.isfinite(optical).all(axis=2)&(optical[:,:,2]>=.1)&(optical[:,:,2]<=6.)
            points = model['translation']+optical@model['rotation'].T
            points[~valid] = np.nan
            yaw = np.deg2rad(.02)*np.sin(number)
            c,s = np.cos(yaw),np.sin(yaw)
            orientation = np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
            points = points@orientation.T
            model['rotation'] = orientation@model['rotation']
            model['translation'] = orientation@model['translation']
            result = core.update(number*.1,image,points,yaw,
                0. if number == 0 else None,0. if number == 0 else None,camera_model=model)
            self.assertEqual(len(core.references),1)
            if result['accepted']:
                accepted += 1
                near_limit += result['rmse_m'] >= .012
                self.assertLessEqual(result['rmse_m'],.015)
            else:
                self.assertNotIn('xyz',result)
            shift = core._projected_view_motion(core.references[0])['projected_median_shift_px']
            maximum_parallax = max(maximum_parallax,shift or 0.)
        self.assertGreaterEqual(accepted,10)
        self.assertGreaterEqual(near_limit,5)
        self.assertLess(maximum_parallax,1.)

    def test_selected_parallax_and_overlap_are_not_taken_from_unfitted_active_view(self):
        scene = PinholePlane(); core = MetricRGBDOdometry()
        image,points,model = scene.frame(35.)
        self.assertTrue(core.update(0.,image,points,0.,0.,0.,camera_model=model)['accepted'])
        origin = core.references[0]; origin['self_overlap'] = 1.
        moved_image,moved_points,_ = scene.frame(35.,body=(.08,0.,0.))
        core.xyz = np.array([.08,0.,0.])
        active = core._reference(core._features(moved_image,moved_points),np.eye(3)*.0001,origin['id'])
        active['self_overlap'] = .001
        core.xyz = np.zeros(3)
        latest = core._reference(core._features(image,points),np.eye(3)*.0002,origin['id'])
        latest['descriptors'][:] = 0; latest['self_overlap'] = 1.
        core.references.extend((active,latest)); core._activate(active)
        calls = []; original_install = core._install_reference
        def record_install(*args):
            calls.append(tuple(core._frame_fits))
            return original_install(*args)
        core._install_reference = record_install
        result = core.update(.1,moved_image,moved_points,0.,None,None,camera_model=model)
        self.assertTrue(result['accepted'],result)
        self.assertEqual(result['reference_id'],origin['id'])
        self.assertTrue(calls); self.assertNotIn(active['id'],calls[0])
        admission = result['view_admission']
        self.assertGreaterEqual(admission['projected_reference_motion']['projected_median_shift_px'],15.5)
        self.assertFalse(admission['support_handoff']['needed'])
        self.assertTrue(admission['overlap_declined'])
        self.assertGreater(result['overlap'],active['self_overlap'])
        self.assertEqual(core._active_view_id,active['id'])

    def test_just_evicted_legacy_reference_without_camera_metadata_is_safe(self):
        scene = PinholePlane(); core = MetricRGBDOdometry(); image,points,model = scene.frame(35.)
        self.assertTrue(core.update(0.,image,points,0.,0.,0.)['accepted'])
        origin = core.references[0]
        legacy = core._reference(core._features(image,points),np.eye(3)*.0001,origin['id'])
        self.assertIsNone(legacy['camera_model']); self.assertNotIn('self_overlap',legacy)
        origin['descriptors'][:] = 0
        core.references.append(legacy); core.max_frames = 2
        core.last_reference = (np.array([-.31,0.,0.]),0.)
        result = core.update(.1,image,points,0.,None,None,camera_model=model)
        self.assertTrue(result['accepted'],result)
        self.assertEqual(result['reference_id'],legacy['id'])
        self.assertNotIn(legacy['id'],[ref['id'] for ref in core.references])
        self.assertEqual(len(core.references),2)

    def test_anchor_covariance_is_common_not_averaged_by_matches(self):
        image, points = self.scene.frame()
        core = MetricRGBDOdometry()
        self.assertTrue(core.update(0., image, points, 0., 0., 0.)['accepted'])
        core.references[0]['anchor_covariance'] = np.diag([.02**2, .03**2, .01**2])
        result = core.update(.1, image, points, 0., 0., 0.)
        self.assertTrue(result['accepted'], result)
        np.testing.assert_allclose(result['covariance']-result['measurement_covariance'],
                                   core.references[0]['anchor_covariance'])
        core.references[0]['anchor_covariance'] = np.eye(3)*.06**2
        self.assertFalse(core.update(.2, image, points, 0., 0., 0.)['accepted'])


class AnchorRetentionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(1)
        cls.scene=PinholePlane()

    def library(self,latest_offset=0.,latest_sigma=.025):
        core=MetricRGBDOdometry();image,points,model=self.scene.frame(35.)
        first=core.update(0.,image,points,0.,0.,0.,camera_model=model)
        self.assertTrue(first['accepted']);origin=core.references[0]
        old=core._reference(core._features(image,points),np.eye(3)*.004**2,origin['id'])
        old['self_overlap']=1.;core.references.append(old)
        body=np.array([.07,0.,0.]);image,points,model=self.scene.frame(35.,body)
        core.xyz=body.copy();core.last_camera_model=core._camera_model(model)
        latest=core._reference(core._features(image,points),np.eye(3)*latest_sigma**2,old['id'])
        latest['xyz']+=np.array([latest_offset,0.,0.]);latest['world']+=np.array([latest_offset,0.,0.])
        latest['self_overlap']=1.;core.references.append(latest);core._activate(latest)
        origin['descriptors'][:]=0
        # A prior genuine measured view is far enough behind the present pose
        # to require a new registered anchor; no command creates this trigger.
        core.last_reference=(np.array([-.31,0.,0.]),0.)
        return core,origin,old,latest

    def test_one_extra_real_old_fit_atomically_replaces_pose_covariance_and_parent(self):
        core,origin,old,latest=self.library(latest_offset=.002)
        image,points,model=self.scene.frame(35.,body=(.10,0.,0.))
        result=core.update(.1,image,points,0.,None,None,camera_model=model)
        self.assertTrue(result['accepted'],result)
        self.assertEqual(result['reference_id'],old['id'])
        self.assertNotEqual(result['reference_id'],latest['id'])
        np.testing.assert_allclose(result['xyz'],core._frame_fits[old['id']]['xyz'],atol=1e-14)
        np.testing.assert_allclose(core.xyz,result['xyz'],atol=1e-14)
        np.testing.assert_allclose(result['covariance'],core._frame_fits[old['id']]['covariance'],atol=1e-14)
        np.testing.assert_allclose(result['covariance']-result['measurement_covariance'],old['anchor_covariance'],atol=1e-14)
        adoption=result['view_admission']['anchor_adoption']
        self.assertEqual(adoption['additional_fit_attempts'],1)
        self.assertEqual(adoption['additional_reference_id'],old['id'])
        self.assertEqual(adoption['additional_reason'],'accepted')
        self.assertEqual(adoption['adopted_reference_id'],old['id'])
        self.assertEqual(core.references[-1]['parent'],old['id'])
        # The origin already failed genuine tracking. It must not be retried
        # or consume the one extra adoption search ahead of an untried view.
        self.assertFalse(result['reference_attempts'][0]['accepted'])
        self.assertEqual(result['reference_attempts'][0]['reference_id'],origin['id'])

    def test_strong_inconsistent_old_fit_rolls_back_entire_measurement(self):
        core,origin,old,latest=self.library(latest_offset=.065,latest_sigma=.0045)
        before=(core.xyz.copy(),list(core.references),core.reference_number,core._active_view_id,core.last_reference)
        image,points,model=self.scene.frame(35.,body=(.10,0.,0.))
        # Both independent image/depth identities pass the original gates;
        # their deliberately conflicting registered anchors must remain a fault.
        result=core.update(.1,image,points,0.,None,None,camera_model=model)
        self.assertFalse(result['accepted'],result)
        self.assertEqual(result['reason'],'ambiguous_rgbd_reference_translation')
        self.assertNotIn('xyz',result)
        np.testing.assert_array_equal(core.xyz,before[0])
        self.assertEqual([id(x) for x in core.references],[id(x) for x in before[1]])
        self.assertEqual(core.reference_number,before[2]);self.assertEqual(core._active_view_id,before[3])
        np.testing.assert_array_equal(core.last_reference[0],before[4][0])

    def test_bounded_retention_keeps_precise_diverse_measured_views_and_pins(self):
        core=MetricRGBDOdometry();image,points,model=self.scene.frame(20.)
        self.assertTrue(core.update(0.,image,points,0.,0.,0.,camera_model=model)['accepted'])
        origin=core.references[0]
        def make(angle,sigma):
            image,points,model=self.scene.frame(angle);core.last_camera_model=core._camera_model(model)
            ref=core._reference(core._features(image,points),np.eye(3)*sigma**2,origin['id'])
            ref['self_overlap']=1.;core.references.append(ref);return ref
        precise=make(35.,.004)
        for angle in (36.,37.,38.,39.,40.,41.,42.,43.,44.,45.,46.,47.,48.,49.,50.,51.,52.):make(angle,.03)
        active=make(65.,.02);core._activate(active)
        newest=make(53.,.035)
        self.assertEqual(len(core.references),21)
        core._anchor_adoption={};core._retire_redundant_reference(active['id'])
        ids={ref['id'] for ref in core.references}
        self.assertEqual(len(ids),20)
        for ref in (origin,precise,active,newest):self.assertIn(ref['id'],ids)
        retired=core._anchor_adoption['retired_reference_id']
        self.assertNotIn(retired,ids)
        # Camera metadata supplies coverage only, never an accepted RGB identity.
        distant=dict(precise,xyz=np.array([100.,0.,0.]))
        self.assertEqual(core._sparse_visible_fraction(precise,distant),0.)
        incoming=make(54.,.036)
        core.references.pop()
        observed=[];original=core._sparse_visible_fraction
        def observe(left,right):
            observed.append(len(core.references))
            return original(left,right)
        core._sparse_visible_fraction=observe
        core._retire_redundant_reference(active['id'],incoming=incoming)
        self.assertTrue(observed)
        self.assertTrue(all(count<=20 for count in observed))
        self.assertEqual(len(core.references),20)

    def test_existing_fit_is_used_without_extra_search_or_self_anchor_reuse(self):
        core,origin,old,latest=self.library()
        image,points,model=self.scene.frame(35.,body=(.10,0.,0.))
        core.last_camera_model=core._camera_model(model)
        features=core._features(image,points);core._current_features=features
        core._frame_fits={};core._frame_prediction=None
        current,why=core._fit(features,core.xyz,latest);self.assertEqual(why,'accepted')
        measured,why=core._fit(features,core.xyz,old);self.assertEqual(why,'accepted')
        core.xyz=current['xyz'].copy();core._entry_reference_ids={origin['id'],old['id'],latest['id']}
        core._attempted_references=[dict(reference_id=r['id'],accepted=False) for r in (origin,old,latest)]
        # A current-exposure self-validation can have a tempting small C, but
        # it is absent from entry IDs and is never an independent old fit.
        self_ref=core._reference(features,np.zeros((3,3)),latest['id']);core.references.append(self_ref)
        local,_=core._fit(features,core.xyz,self_ref)
        self.assertIsNotNone(local)
        selected=core._prefer_measured_anchor(features,dict(current))
        self.assertEqual(selected['reference_id'],old['id'])
        self.assertEqual(core._anchor_adoption['additional_fit_attempts'],0)
        np.testing.assert_array_equal(selected['covariance'],measured['covariance'])

    def test_added_fit_cannot_hide_nontransitive_existing_strong_mode(self):
        core,origin,old,latest=self.library()
        image,points,model=self.scene.frame(35.,body=(.10,0.,0.))
        features=core._features(image,points);core.last_camera_model=core._camera_model(model)
        def measured(reference,position,variance):
            return dict(xyz=np.array([position,0.,0.]),covariance=np.eye(3)*variance,
                        inliers=60,reference_id=reference['id'])
        current=measured(latest,0.,.001)
        other=measured(origin,.14,.0012)
        extra_fit=measured(old,-.03,.0001)
        def statistic(left,right):
            delta=left['xyz']-right['xyz']
            return delta@np.linalg.solve(left['covariance']+right['covariance'],delta)
        self.assertLess(statistic(current,other),core.residual_chi_square)
        self.assertLess(statistic(current,extra_fit),core.residual_chi_square)
        self.assertGreater(statistic(other,extra_fit),core.residual_chi_square)
        core._entry_reference_ids={origin['id'],old['id'],latest['id']}
        core._frame_fits={origin['id']:other,old['id']:extra_fit,latest['id']:current}
        core._attempted_references=[]
        with self.assertRaises(_ConflictingRegisteredView):
            core._prefer_measured_anchor(features,current)

    def test_failed_self_validation_is_counted_and_never_resets_anchor_covariance(self):
        core=MetricRGBDOdometry();image,points,model=self.scene.frame(35.)
        self.assertTrue(core.update(0.,image,points,0.,0.,0.,camera_model=model)['accepted'])
        origin=core.references[0]
        image,points,model=self.scene.frame(35.,body=(.10,0.,0.))
        core.last_camera_model=core._camera_model(model);features=core._features(image,points)
        measured,reason=core._fit(features,np.array([.10,0.,0.]),origin)
        self.assertEqual(reason,'accepted')
        increment=np.linalg.eigvalsh(measured['measurement_covariance'][:2,:2]).max()
        origin['anchor_covariance']=np.eye(3)*(.05**2-1.5*increment)
        original_covariance=origin['anchor_covariance'].copy()
        core.last_reference=(np.array([-.31,0.,0.]),0.)
        # The search remains at a recent genuinely registered pose; a wheel
        # prediction is unnecessary for the small current displacement.
        core.xyz=np.array([.03,0.,0.])
        result=core.update(.1,image,points,0.,None,None,camera_model=model)
        self.assertTrue(result['accepted'],result)
        self.assertLessEqual(np.linalg.eigvalsh(result['covariance'][:2,:2]).max(),.05**2)
        self.assertEqual(result['reference_build_failures'],1)
        self.assertEqual(result['view_admission']['reference_build_attempts'][-1]['reason'],'uncertain_rgbd_translation')
        self.assertEqual(len(core.references),1)
        np.testing.assert_array_equal(origin['anchor_covariance'],original_covariance)
        np.testing.assert_allclose(result['covariance']-result['measurement_covariance'],original_covariance,atol=1e-14)


    def test_retention_cache_preserves_choice_and_bounds_pair_work(self):
        core,origin,old,latest=self.library()
        def make(angle,sigma):
            image,points,model=self.scene.frame(angle)
            core.last_camera_model=core._camera_model(model)
            ref=core._reference(core._features(image,points),np.eye(3)*sigma**2,origin['id'])
            ref['self_overlap']=1.
            return ref
        for angle in range(36,53):core.references.append(make(angle,.02+angle*.0001))
        self.assertEqual(len(core.references),20)
        source_calls=[];original=core._sparse_visible_fraction
        def observed(left,right):
            self.assertLessEqual(len(core._retention_coverage),210)
            source_calls.append((left['id'],right['id']))
            return original(left,right)
        core._sparse_visible_fraction=observed
        core._retire_redundant_reference(latest['id'],incoming=make(53.,.035))
        first_calls=len(source_calls);self.assertGreater(first_calls,40)
        ids={ref['id'] for ref in core.references}
        self.assertLessEqual(len(core._retention_coverage),190)
        self.assertTrue(all(set(pair)<=ids for pair in core._retention_coverage))
        oracle=MetricRGBDOdometry();oracle.references=list(core.references);oracle._active_view_id=core._active_view_id
        oracle._anchor_adoption={}
        incoming=make(54.,.036);source_calls.clear()
        risk_calls=[];original_risk=core._anchor_risk
        def risk(ref):
            risk_calls.append(ref['id']);return original_risk(ref)
        core._anchor_risk=risk
        core._retire_redundant_reference(latest['id'],incoming=incoming)
        oracle._retire_redundant_reference(latest['id'],incoming=incoming)
        self.assertEqual([ref['id'] for ref in core.references],[ref['id'] for ref in oracle.references])
        self.assertEqual(core._anchor_adoption['retired_reference_id'],oracle._anchor_adoption['retired_reference_id'])
        self.assertLessEqual(len(source_calls),40)
        self.assertEqual(len(risk_calls),21);self.assertEqual(len(set(risk_calls)),21)
        self.assertLessEqual(len(core._retention_coverage),190)
        self.assertLessEqual(len(core._retention_references),20)

    def test_same_id_new_reference_invalidates_geometry_cache_and_close_releases_it(self):
        core,origin,old,latest=self.library()
        core._prune_retention_cache(core.references)
        core._retention_coverage={(origin['id'],old['id']):.75,(old['id'],latest['id']):.85}
        replacement=dict(old,world=old['world']+np.array([100.,0.,0.]),xyz=old['xyz']+np.array([100.,0.,0.]))
        core.references[1]=replacement
        core._prune_retention_cache(core.references)
        self.assertFalse(any(old['id'] in pair for pair in core._retention_coverage))
        self.assertIs(core._retention_references[old['id']],replacement)
        core.close()
        self.assertEqual(core._retention_coverage,{})
        self.assertEqual(core._retention_references,{})
        fresh=MetricRGBDOdometry()
        self.assertEqual(fresh._retention_coverage,{})

    def test_strong_conflict_rolls_back_and_invalidates_persistent_pair_cache(self):
        core,origin,old,latest=self.library(latest_offset=.065,latest_sigma=.0045)
        core._prune_retention_cache(core.references)
        core._retention_coverage={(old['id'],latest['id']):.9}
        before_number=core.reference_number
        image,points,model=self.scene.frame(35.,body=(.10,0.,0.))
        result=core.update(.1,image,points,0.,None,None,camera_model=model)
        self.assertEqual(result['reason'],'ambiguous_rgbd_reference_translation')
        self.assertEqual(core.reference_number,before_number)
        self.assertEqual(core._retention_coverage,{})
        self.assertEqual(core._retention_references,{})

    def test_dynamic_current_view_adoption_never_uses_persistent_retention_cache(self):
        core,origin,old,latest=self.library()
        core._retention_coverage={(origin['id'],old['id']):.123}
        before=dict(core._retention_coverage);calls=[];original=core._sparse_visible_fraction
        def dynamic(left,right):
            if 'id' not in left or 'id' not in right:
                current=left if 'id' not in left else right
                calls.append(current['xyz'].copy())
            return original(left,right)
        core._sparse_visible_fraction=dynamic
        for body in (.10,.12):
            image,points,model=self.scene.frame(35.,body=(body,0.,0.))
            core.last_camera_model=core._camera_model(model);features=core._features(image,points)
            core._current_features=features;core._frame_fits={};core._frame_prediction=None
            fit,reason=core._fit(features,np.array([body,0.,0.]),latest)
            self.assertEqual(reason,'accepted')
            core.xyz=fit['xyz'].copy();core._entry_reference_ids={ref['id'] for ref in core.references}
            core._attempted_references=[dict(reference_id=origin['id']),dict(reference_id=latest['id'])]
            core._prefer_measured_anchor(features,dict(fit))
            self.assertEqual(core._retention_coverage,before)
        self.assertGreaterEqual(len(calls),4)
        self.assertGreater(np.ptp(np.array(calls)[:,0]),.01)

class LatestTemplateRefreshTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(1)
        cls.scene=PinholePlane()

    def library(self,latest_body=.01,residual_template=True):
        core=MetricRGBDOdometry();image,points,model=self.scene.frame(35.)
        self.assertTrue(core.update(0.,image,points,0.,0.,0.,camera_model=model)['accepted'])
        origin=core.references[0]
        image,points,model=self.scene.frame(35.,body=(latest_body,0.,0.))
        v,u=np.indices(image.shape)
        # An independently measured smooth error field in this template.
        # Its own exposure is identifiable, but the next true measurement
        # must still pass the original 15mm registration residual gate.
        if residual_template:
            points[:,:,0]+=.020*np.sin(v*.07);points[:,:,1]+=.020*np.cos(u*.07)
        core.xyz=np.array([latest_body,0.,0.]);core.last_camera_model=core._camera_model(model)
        latest=core._reference(core._features(image,points),np.eye(3)*.00001,origin['id'])
        own,reason=core._fit(core._current_features,core.xyz,latest)
        self.assertEqual(reason,'accepted')
        latest['self_overlap']=own['overlap']
        if not residual_template:latest['descriptors'][:]=0
        core.references.append(latest);core._activate(latest)
        return core,origin,latest

    def test_latest_tracking_failure_and_old_success_save_resolvable_new_view(self):
        core,origin,latest=self.library()
        image,points,model=self.scene.frame(35.,body=(.04,0.,0.))
        result=core.update(.1,image,points,0.,None,None,camera_model=model)
        self.assertTrue(result['accepted'],result)
        details=result['view_admission']['latest_template_handoff']
        self.assertTrue(details['needed'])
        self.assertEqual(details['latest_reference_id'],latest['id'])
        self.assertEqual(details['latest_tracking_reason'],'rgbd_registration_residual')
        self.assertLess(details['accepted_reference_motion']['projected_median_shift_px'],15.5)
        self.assertLess(details['latest_reference_motion']['projected_median_shift_px'],15.5)
        self.assertFalse(result['view_admission']['support_handoff']['needed'])
        self.assertEqual(result['reference_id'],origin['id'])
        self.assertEqual(len(core.references),3)
        self.assertEqual(core.references[-1]['parent'],result['reference_id'])
        self.assertEqual(core.references[-1]['id'],core._active_view_id)
        # The fresh reference is self-validated, never returned as independent
        # pose/covariance evidence. Anchor uncertainty is added only once.
        np.testing.assert_array_equal(result['xyz'],core._frame_fits[origin['id']]['xyz'])
        np.testing.assert_array_equal(result['covariance'],core._frame_fits[origin['id']]['covariance'])
        np.testing.assert_array_equal(core.references[-1]['anchor_covariance'],result['covariance'])
        self.assertEqual(result['view_admission']['reference_build_attempts'][-1]['reason'],'accepted')

    def test_static_at_latest_with_older_parallax_and_fresh_noise_does_not_refresh(self):
        core,origin,latest=self.library(latest_body=.04,residual_template=False);rng=np.random.default_rng(7741)
        maximum_latest_shift=0.;old_resolvable=0
        for n in range(1,13):
            image,points,model=self.scene.frame(35.,body=(.04,0.,0.))
            image=np.clip(image.astype(float)+rng.normal(0,.15,image.shape),0,255).astype(np.uint8)
            points+=rng.normal(0,.0004,points.shape)
            result=core.update(n*.1,image,points,0.,None,None,camera_model=model)
            self.assertTrue(result['accepted'],result)
            self.assertEqual(len(core.references),2)
            # Isolate the dual-parallax policy from the rejection label. This
            # stationary image fixture actually rejects the latest identity;
            # the residual label below is a helper-only policy input, not a
            # claim that its real registration failed the residual gate.
            attempt=next(item for item in core._attempted_references if item['reference_id']==latest['id'])
            self.assertFalse(attempt['accepted'])
            actual_reason=attempt['reason'];attempt['reason']='rgbd_registration_residual'
            details=core._latest_template_handoff(origin)
            attempt['reason']=actual_reason
            self.assertFalse(details['needed'])
            old_resolvable+=details['accepted_reference_motion']['projected_median_shift_px']>=details['accepted_minimum_parallax_px']
            maximum_latest_shift=max(maximum_latest_shift,details['latest_reference_motion']['projected_median_shift_px'])
            self.assertLess(details['latest_reference_motion']['projected_median_shift_px'],details['latest_minimum_parallax_px'])
        self.assertGreaterEqual(old_resolvable,10)
        self.assertLess(maximum_latest_shift,1.)

    def test_refresh_requires_actual_latest_tracking_reject_and_both_pixel_budgets(self):
        core,origin,latest=self.library()
        image,points,model=self.scene.frame(35.,body=(.04,0.,0.))
        core.last_camera_model=core._camera_model(model);current=core._features(image,points)
        fit,why=core._fit(current,np.array([.04,0.,0.]),origin)
        self.assertEqual(why,'accepted');core.xyz=fit['xyz'].copy()
        core._entry_latest_reference_id=latest['id']
        core._attempted_references=[dict(reference_id=origin['id'],accepted=False,reason='expected_origin_failure')]
        self.assertFalse(core._latest_template_handoff(origin)['needed'])
        core._attempted_references=[dict(reference_id=latest['id'],accepted=True,reason='accepted')]
        self.assertFalse(core._latest_template_handoff(origin)['needed'])
        for reason in ('insufficient_rgbd_identity_matches','insufficient_rgbd_spatial_support','rgbd_correspondence_consensus','ambiguous_rgbd_translation','rgbd_correction_jump','uncertain_rgbd_translation'):
            core._attempted_references[0].update(accepted=False,reason=reason)
            self.assertFalse(core._latest_template_handoff(origin)['needed'],reason)
        core._attempted_references[0].update(accepted=False,reason='rgbd_registration_residual')
        self.assertTrue(core._latest_template_handoff(origin)['needed'])
        self.assertFalse(core._latest_template_handoff(latest)['needed'])
        projection=core._projected_view_motion
        core._projected_view_motion=lambda ref:dict(projected_median_shift_px=.01 if ref is latest else 10.)
        self.assertFalse(core._latest_template_handoff(origin)['needed'])
        core._projected_view_motion=lambda ref:dict(projected_median_shift_px=10. if ref is latest else .01)
        self.assertFalse(core._latest_template_handoff(origin)['needed'])
        core._projected_view_motion=projection
        before=latest['pixel_scales'];latest['pixel_scales']=np.array([np.nan])
        self.assertFalse(core._latest_template_handoff(origin)['needed'])
        latest['pixel_scales']=before

    def test_forced_refresh_keeps_registered_view_conflict_and_rollback(self):
        core,origin,latest=self.library()
        before=(core.xyz.copy(),list(core.references),core.reference_number,core._active_view_id)
        core._retention_coverage={(1,2):.3};core._retention_references={1:origin,2:latest}
        image,points,model=self.scene.frame(35.,body=(.04,0.,0.))
        calls=[];original=core._registered_view
        def conflicting(features,current):
            self.assertTrue(core._refresh_latest_template)
            calls.append(current['reference_id'])
            original(features,current)
            # This is the existing registered-view fault boundary; no new
            # source exception/rollback path or independent pose is invented.
            raise _ConflictingRegisteredView()
        core._registered_view=conflicting
        result=core.update(.1,image,points,0.,None,None,camera_model=model)
        self.assertFalse(result['accepted']);self.assertEqual(result['reason'],'ambiguous_rgbd_reference_translation')
        self.assertTrue(calls);self.assertNotIn('xyz',result)
        np.testing.assert_array_equal(core.xyz,before[0])
        self.assertEqual([id(r) for r in core.references],[id(r) for r in before[1]])
        self.assertEqual(core.reference_number,before[2]);self.assertEqual(core._active_view_id,before[3])
        self.assertEqual(core._retention_coverage,{})
        self.assertEqual(core._retention_references,{})



def _exhaustive_rgbd_hamming_pairs(current,reference):
    """Original identity definition, using ALL reverse queries independently."""
    matcher=cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=False)
    forward=matcher.knnMatch(current['descriptors'],reference['descriptors'],k=2)
    preliminary=[rows[0] for rows in forward if len(rows)==2 and rows[0].distance<=48
                 and rows[0].distance<.70*rows[1].distance]
    if not preliminary:return np.empty((0,2),dtype=int)
    backward=matcher.knnMatch(reference['descriptors'],current['descriptors'],k=2)
    reverse={rows[0].queryIdx:rows[0].trainIdx for rows in backward
             if len(rows)==2 and rows[0].distance<=48 and rows[0].distance<.70*rows[1].distance}
    pairs=[match for match in preliminary if reverse.get(match.trainIdx)==match.queryIdx]
    pairs.sort(key=lambda match:(match.distance,match.queryIdx))
    roi=current['measured_roi'];width,height=roi[2:]-roi[:2]
    counts,kept={},[]
    for match in pairs:
        u,v=current['pixels'][match.queryIdx]-roi[:2]
        cell=(min(3,int(u/width*4)),min(2,int(v/height*3)))
        if counts.get(cell,0)<8:
            counts[cell]=counts.get(cell,0)+1;kept.append((match.queryIdx,match.trainIdx))
    return np.asarray(kept,dtype=int).reshape(-1,2)


class ExactDescriptorEqualityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(1)

    @staticmethod
    def features(descriptors):
        n=len(descriptors);i=np.arange(n)
        pixels=np.column_stack((10+(i%32)*9.,10+((i//32)%24)*9.))
        points=np.column_stack(((pixels[:,0]-155.)*.01,(pixels[:,1]-115.)*.01,np.ones(n)))
        return dict(descriptors=descriptors,pixels=pixels,points=points,
            covariance=np.tile(np.eye(3)*.000009,(n,1,1)),pixel_scales=np.ones(n),
            image_shape=(240,320),measured_roi=np.array([0,0,320,240]))

    def compare(self,left,right,fast):
        core=MetricRGBDOdometry();current=self.features(left);reference=dict(descriptors=right)
        class CountMatcher:
            def __init__(self,matcher):self.matcher=matcher;self.calls=0
            def knnMatch(self,*args,**kwargs):
                self.calls+=1;return self.matcher.knnMatch(*args,**kwargs)
        measured=CountMatcher(core._matcher);core._matcher=measured
        try:expected=_exhaustive_rgbd_hamming_pairs(current,reference)
        except cv2.error:
            with self.assertRaises(cv2.error):core._matches(current,reference)
            self.assertGreater(measured.calls,0);return
        actual=core._matches(current,reference)
        self.assertEqual(actual.shape,expected.shape);self.assertEqual(actual.dtype,expected.dtype)
        self.assertEqual(actual.tobytes(),expected.tobytes())
        self.assertIs(core._matcher,measured)
        if fast:self.assertEqual(measured.calls,0)
        else:self.assertGreater(measured.calls,0)
        return actual

    def test_unique_ordered_tables_empty_single_and_density_match_exhaustive_BF(self):
        for n in (0,1,2,24,96,1400):
            with self.subTest(n=n):
                descriptors=np.random.default_rng(n+751).integers(0,256,(n,32),dtype=np.uint8)
                pairs=self.compare(descriptors,descriptors.copy(),True)
                self.assertLessEqual(len(pairs),96)
                self.assertTrue(np.array_equal(pairs[:,0],pairs[:,1]))

    def test_repeated_zero_rows_and_mixed_duplicates_keep_strict_ratio(self):
        descriptors=np.zeros((80,32),dtype=np.uint8)
        self.assertEqual(len(self.compare(descriptors,descriptors.copy(),True)),0)
        descriptors=np.random.default_rng(4).integers(0,256,(96,32),dtype=np.uint8)
        descriptors[48:72]=descriptors[:24];descriptors[72:]=descriptors[:24]
        pairs=self.compare(descriptors,descriptors.copy(),True)
        self.assertTrue(all(24<=i<48 and i==j for i,j in pairs))

    def test_tied_positive_second_neighbors_and_zero_ties_match_BF(self):
        descriptors=np.zeros((64,32),dtype=np.uint8)
        for i in range(64):descriptors[i,i//8]=1<<(i%8)
        self.compare(descriptors,descriptors.copy(),True)
        descriptors[16:24]=descriptors[:8]
        self.compare(descriptors,descriptors.copy(),True)

    def test_reordering_size_or_one_bit_difference_preserves_original_search(self):
        descriptors=np.random.default_rng(5).integers(0,256,(100,32),dtype=np.uint8)
        self.compare(descriptors,descriptors[::-1].copy(),False)
        self.compare(descriptors,descriptors[:80].copy(),False)
        changed=descriptors.copy();changed[-1,0]^=1
        self.compare(descriptors,changed,False)

    def test_dtype_width_and_strided_storage_do_not_change_identity_semantics(self):
        descriptors=np.random.default_rng(6).integers(0,256,(60,32),dtype=np.uint8)
        self.compare(descriptors.astype(np.float32),descriptors.astype(np.float32),False)
        self.compare(descriptors[:,:16].copy(),descriptors[:,:16].copy(),False)
        storage=np.zeros((60,64),dtype=np.uint8);storage[:,::2]=descriptors
        self.compare(storage[:,::2],descriptors.copy(),True)

    def test_self_registration_all_fields_and_covariance_bytes_equal_BF_oracle(self):
        import json
        def native(value):
            if isinstance(value,np.ndarray):return value.tolist()
            if isinstance(value,np.generic):return value.item()
            if isinstance(value,dict):return {k:native(v) for k,v in value.items()}
            if isinstance(value,(list,tuple)):return [native(v) for v in value]
            return value
        descriptors=np.random.default_rng(7).integers(0,256,(768,32),dtype=np.uint8)
        for repeated in (False,True):
            if repeated:descriptors[32:48]=descriptors[:16]
            baseline,candidate=MetricRGBDOdometry(),MetricRGBDOdometry()
            baseline._matches=_exhaustive_rgbd_hamming_pairs
            features=self.features(descriptors)
            left=baseline._reference(features,np.eye(3)*.00001,None)
            right=candidate._reference(features,np.eye(3)*.00001,None)
            expected,reason1=baseline._fit(features,np.zeros(3),left)
            actual,reason2=candidate._fit(features,np.zeros(3),right)
            self.assertEqual(reason1,reason2);self.assertIsNotNone(expected)
            self.assertEqual(json.dumps(native(expected),sort_keys=True),json.dumps(native(actual),sort_keys=True))
            for key in ('xyz','covariance','measurement_covariance'):
                self.assertEqual(expected[key].tobytes(),actual[key].tobytes())

    def test_equal_descriptors_still_require_original_jump_and_uncertainty_gates(self):
        features=self.features(np.random.default_rng(99).integers(0,256,(768,32),dtype=np.uint8))
        for offset,anchor_sigma,reason in ((.20,0.,'rgbd_correction_jump'),(0.,.05,'uncertain_rgbd_translation')):
            baseline,candidate=MetricRGBDOdometry(),MetricRGBDOdometry()
            baseline._matches=_exhaustive_rgbd_hamming_pairs
            baseline.xyz=candidate.xyz=np.array([offset,0.,0.])
            left=baseline._reference(features,np.eye(3)*anchor_sigma**2,None)
            right=candidate._reference(features,np.eye(3)*anchor_sigma**2,None)
            expected,reason1=baseline._fit(features,np.zeros(3),left)
            actual,reason2=candidate._fit(features,np.zeros(3),right)
            self.assertIsNone(expected);self.assertIsNone(actual)
            self.assertEqual(reason1,reason);self.assertEqual(reason2,reason)



class MeasurementRefinementTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(1);cls.scene=PinholePlane()

    def measured_pair(self,body=.03):
        core=MetricRGBDOdometry();image,grid,model=self.scene.frame(35.)
        first=core._features(image,grid);core.xyz=np.zeros(3);core.yaw=0.
        core.last_camera_model=core._camera_model(model)
        reference=core._reference(first,np.zeros((3,3)),None)
        image,grid,model=self.scene.frame(35.,body=(body,0.,0.))
        current=core._features(image,grid);pairs=core._matches(current,reference)
        self.assertGreaterEqual(len(pairs),24)
        return core,first,reference,current,pairs

    def test_owned_templates_and_same_exposure_token_skip_only_flow_not_original_gates(self):
        core,first,reference,current,pairs=self.measured_pair()
        self.assertIsNotNone(first['_measurement_token'])
        self.assertIs(reference['_measurement_token'],first['_measurement_token'])
        for key in ('_measured_gray','_measured_grid'):
            self.assertFalse(np.shares_memory(first[key],reference[key]))
            before=reference[key].copy();first[key].flat[0]=0
            np.testing.assert_array_equal(reference[key],before)
        self.assertIs(core._refine_identified_measurements(first,reference,core._matches(first,reference)),first)
        uncertain=core._reference(first,np.eye(3)*.05**2,None)
        fit,reason=core._fit(first,np.zeros(3),uncertain)
        self.assertIsNone(fit);self.assertEqual(reason,'uncertain_rgbd_translation')
        # Two timestamps are not one measurement even for identical pixels.
        again=core._features(self.scene.frame(35.)[0],self.scene.frame(35.)[1])
        self.assertIsNot(again['_measurement_token'],first['_measurement_token'])
        missing=dict(current);missing.pop('_measurement_token')
        self.assertIs(core._refine_identified_measurements(missing,reference,pairs),missing)

    def test_per_reference_private_measurements_preserve_pairs_and_covariance_floors(self):
        core,first,reference,current,pairs=self.measured_pair()
        saved={key:current[key].copy() for key in ('pixels','points','covariance','descriptors')}
        ref_saved={key:reference[key].copy() for key in ('pixels','points','covariance','world','_measured_gray','_measured_grid')}
        refined=core._refine_identified_measurements(current,reference,pairs)
        self.assertGreater(core._last_measurement_refinement['applied'],0)
        self.assertIsNot(refined,current)
        for key,value in saved.items():np.testing.assert_array_equal(current[key],value)
        for key,value in ref_saved.items():np.testing.assert_array_equal(reference[key],value)
        self.assertFalse(np.shares_memory(refined['pixels'],current['pixels']))
        self.assertFalse(np.shares_memory(refined['points'],current['points']))
        self.assertFalse(np.shares_memory(refined['covariance'],current['covariance']))
        self.assertGreaterEqual(float(np.linalg.eigvalsh(refined['covariance']-current['covariance']).min()),-1e-14)
        self.assertGreaterEqual(float(np.linalg.eigvalsh(refined['covariance']).min()),.003**2-1e-14)
        calls=[];original=core._matches
        def record(left,right):
            calls.append(left['pixels'].copy());return original(left,right)
        core._matches=record
        fit,reason=core._fit(current,np.array([.03,0.,0.]),reference)
        self.assertEqual(reason,'accepted');self.assertEqual(len(calls),1)
        np.testing.assert_array_equal(calls[0],saved['pixels'])
        self.assertEqual(fit['identity_matches'],len(pairs))
        self.assertEqual(fit['reference_support']['measurement_refinement']['identity_pairs'],len(pairs))
        for key,value in saved.items():np.testing.assert_array_equal(current[key],value)

    def test_invalid_depth_borders_and_worsened_actual_photo_keep_original_measurements(self):
        core,first,reference,current,pairs=self.measured_pair()
        for bad in ('depth','border'):
            altered=dict(current)
            if bad=='depth':altered['_measured_grid']=np.full_like(current['_measured_grid'],np.nan)
            else:altered['pixels']=np.zeros_like(current['pixels'])
            result=core._refine_identified_measurements(altered,reference,pairs)
            self.assertIs(result,altered)
        # Same measured image gives zero seed SSD. A bogus but bounded forward
        # endpoint must fall back, even when a solver claims success and FB=0.
        image,grid,_=self.scene.frame(35.)
        current=core._features(image,grid);pairs=core._matches(current,reference)
        actual=cv2.calcOpticalFlowPyrLK;calls=[]
        def wrong(left,right,p0,p1,**kwargs):
            calls.append(1);q=p1.copy()
            if len(calls)==1:q[:,:,0]+=.25
            return q,np.ones((len(q),1),np.uint8),np.zeros((len(q),1),np.float32)
        cv2.calcOpticalFlowPyrLK=wrong
        try:result=core._refine_identified_measurements(current,reference,pairs)
        finally:cv2.calcOpticalFlowPyrLK=actual
        self.assertEqual(len(calls),2);self.assertIs(result,current)
        self.assertEqual(core._last_measurement_refinement['applied'],0)

    def test_fresh_static_sensor_noise_does_not_accumulate_reference_anchors(self):
        core=MetricRGBDOdometry();accepted=0
        for i in range(12):
            image,grid,model=self.scene.frame(35.)
            rng=np.random.default_rng(100+i)
            gray=np.clip(image.astype(int)+rng.integers(-1,2,image.shape),0,255).astype(np.uint8)
            noisy=grid+rng.normal(0,.001,grid.shape)
            result=core.update(i*.1,gray,noisy,0.,0.,0.,camera_model=model)
            if result['accepted']:
                accepted+=1;self.assertLess(np.linalg.norm(result['xyz']),.01)
            self.assertLessEqual(len(core.references),1)
        self.assertGreaterEqual(accepted,10)
        self.assertEqual(len(core.references),1)

    def test_original_spatial_gate_uses_reference_local_refined_pixel_coordinates(self):
        core,first,reference,current,pairs=self.measured_pair()
        original=core._refine_identified_measurements
        def collinear(features,ref,pair_ids):
            local=dict(features);local['pixels']=features['pixels'].copy()
            local['pixels'][:,1]=120.
            return local
        core._refine_identified_measurements=collinear
        fit,reason=core._fit(current,np.array([.03,0.,0.]),reference)
        self.assertIsNone(fit);self.assertEqual(reason,'insufficient_rgbd_spatial_support')
        core._refine_identified_measurements=original




class SupportTrendMaintenanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(1)
        cls.scene=PinholePlane()

    def fixture(self,previous=45,current=27,shift=4.):
        core=MetricRGBDOdometry();rng=np.random.default_rng(9102)
        u,v=np.meshgrid(np.linspace(30,290,12),np.linspace(30,210,8));pixels=np.column_stack((u.ravel(),v.ravel()))
        K=np.array([[277.,0.,160.],[0.,277.,120.],[0.,0.,1.]])
        model=dict(K=K,rotation=np.eye(3),translation=np.zeros(3))
        def features(px):
            return dict(descriptors=rng.integers(0,256,(96,32),dtype=np.uint8),pixels=px.copy(),
                points=np.column_stack(((px[:,0]+.5-160.)*2/277.,(px[:,1]+.5-120.)*2/277.,np.full(96,2.))),
                covariance=np.tile(np.eye(3)*9e-6,(96,1,1)),pixel_scales=np.ones(96),image_shape=(240,320),measured_roi=np.array([0.,0.,320.,240.]))
        old=features(pixels);now=features(pixels+[shift,0]);now['descriptors']=old['descriptors'].copy()
        core.last_camera_model=core._camera_model(model);core.yaw=0.
        reference=core._reference(old,np.eye(3)*1e-5,None);reference['self_overlap']=1.
        core.references=[reference];core._activate(reference)
        core.xyz=np.array([-shift*2/277.,0.,0.]);core._current_features=now
        support=dict(image_span=[.8,.8],measured_roi=[0,0,320,240])
        fit=dict(xyz=core.xyz.copy(),covariance=np.eye(3)*1e-5,reference_id=reference['id'],identity_matches=current,
                 inliers=current,overlap=current/96,rmse_m=.002,
                 reference_support=dict(current=dict(support),reference=dict(support),identity_pixel_scale_max=dict(current=1.,reference=1.)))
        core._frame_fits={reference['id']:fit};core._attempted_references=[dict(reference_id=reference['id'],accepted=True,reason='accepted')]
        core._entry_reference_ids={reference['id']};core._entry_latest_reference_id=reference['id']
        core._identity_previous=dict(reference_id=reference['id'],identity_matches=previous,
            features={k:old[k].copy() for k in ('descriptors','pixels','pixel_scales')},image_shape=(240,320),K=K.copy(),stamp_ns=100000000,interval_ns=100000000)
        return core,reference,old,now,fit,model

    def test_declining_same_stage_support_uses_one_actual_adjacent_match(self):
        core,ref,old,now,fit,model=self.fixture()
        result=core._identity_support_reserve(ref)
        self.assertTrue(result['needed']);self.assertEqual(result['one_more_same_decline_count'],9)
        self.assertGreaterEqual(result['adjacent_motion']['adjacent_identity_pairs'],24)
        self.assertGreater(result['adjacent_motion']['median_step_over_pair_3sigma'],1.)
        core._identity_support_reserve(ref);core._support_handoff(ref)
        self.assertEqual(core._identity_adjacent_match_count,1)
        self.assertFalse(core._identity_original_maintenance(ref,fit))

    def test_stable_rising_sufficient_reserve_or_other_reference_never_match(self):
        for previous,current in ((45,45),(27,45),(50,40)):
            core,ref,*_=self.fixture(previous,current)
            self.assertFalse(core._identity_support_reserve(ref)['needed']);self.assertEqual(core._identity_adjacent_match_count,0)
        core,ref,*_=self.fixture();core._identity_previous['reference_id']=999
        self.assertFalse(core._identity_support_reserve(ref)['needed']);self.assertEqual(core._identity_adjacent_match_count,0)

    def test_stationary_noise_far_from_old_template_does_not_refresh(self):
        core,ref,old,now,fit,model=self.fixture(shift=0.)
        core.xyz=np.array([-.09,0.,0.]);fit['xyz']=core.xyz.copy()
        now['pixels']+=np.random.default_rng(911).normal(0,.12,now['pixels'].shape)
        self.assertGreater(core._projected_view_motion(ref)['projected_median_shift_px'],10.)
        result=core._identity_support_reserve(ref)
        self.assertFalse(result['needed']);self.assertEqual(core._identity_adjacent_match_count,1)
        self.assertFalse(core._identity_refresh)

    def test_only_depleted_template_changes_reuse_and_returned_current_view_can_reuse(self):
        core,ref,old,now,fit,model=self.fixture()
        self.assertTrue(core._identity_support_reserve(ref)['needed'])
        self.assertFalse(core._near_view(ref))
        other=dict(ref);other['id']=2;other['xyz']=np.array([4*2/277.,0.,0.]);other['world']=other['points']+other['xyz']
        core.references.append(other);core._entry_reference_ids.add(2)
        other_fit=dict(fit,reference_id=2);core._frame_fits[2]=other_fit
        shift=core._projected_view_motion(other)['projected_median_shift_px']
        self.assertGreater(shift,3*.5*np.sqrt(2.));self.assertLess(shift,15.5)
        self.assertTrue(core._near_view(other))
        # Return to the actual registered view: adjacent movement alone must
        # not defeat reuse of an already current, independently accepted view.
        core.xyz=ref['xyz'].copy();self.assertTrue(core._near_view(ref))

    def test_history_owned_cadence_gaps_jitter_calibration_reject_exception_and_close(self):
        image,points,model=self.scene.frame(35.);core=MetricRGBDOdometry()
        for stamp in (0.,.1,.2):self.assertTrue(core.update(stamp,image,points,0.,0.,0.,camera_model=model)['accepted'])
        self.assertEqual(core._identity_history['interval_ns'],100000000)
        self.assertFalse(np.shares_memory(core._identity_history['features']['descriptors'],core._current_features['descriptors']))
        dynamic=dict(model);angle=.001;dynamic['rotation']=model['rotation']@np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1.]])
        self.assertTrue(core.update(.3,image,points,0.,None,None,camera_model=dynamic)['accepted'])
        self.assertEqual(core._identity_history['interval_ns'],100000000)
        self.assertTrue(core.update(.5,image,points,0.,None,None,camera_model=model)['accepted']);self.assertIsNone(core._identity_history['interval_ns'])
        self.assertTrue(core.update(.600000001,image,points,0.,None,None,camera_model=model)['accepted'])
        self.assertTrue(core.update(.7,image,points,0.,None,None,camera_model=model)['accepted']);self.assertIsNone(core._identity_history['interval_ns'])
        changed=dict(model,K=model['K'].copy());changed['K'][0,0]+=1.
        self.assertTrue(core.update(.8,image,points,0.,None,None,camera_model=changed)['accepted']);self.assertIsNone(core._identity_history['interval_ns'])
        self.assertFalse(core.update(.9,np.zeros_like(image),points,0.,None,None,camera_model=model)['accepted']);self.assertIsNone(core._identity_history)
        self.assertTrue(core.update(1.,image,points,0.,None,None,camera_model=model)['accepted'])
        with self.assertRaises(TypeError):core.update('bad',image,points,0.,None,None,camera_model=model)
        self.assertIsNone(core._identity_history)
        core.close();self.assertIsNone(core._identity_previous)

    def test_adoption_change_cancels_only_identity_admission_and_commits_final_count(self):
        from unittest.mock import patch
        parent=sys.modules['metric_rgbd_odometry']._MetricRGBDReferenceMaintenance
        for original in (False,True):
            core,ref,old,now,fit,model=self.fixture();core._identity_support_reserve(ref)
            core._identity_only_admission=not original
            adopted=dict(fit,reference_id=2,identity_matches=73)
            with patch.object(parent,'_prefer_measured_anchor',return_value=adopted):core._prefer_measured_anchor(now,fit)
            self.assertFalse(core._identity_refresh);self.assertTrue(core._identity_cancelled_after_adoption)
            self.assertEqual(core._identity_skip_admission,not original)
        core,ref,old,now,fit,model=self.fixture();core.stamp=.1;core._identity_history=core._identity_previous
        adopted=dict(fit,accepted=True,reason='accepted',reference_id=2,identity_matches=73)
        def accepted(*args,**kwargs):core.stamp=.2;core._current_features=now;return adopted
        with patch.object(parent,'update',side_effect=accepted):core.update(.2,np.zeros((240,320),np.uint8),np.zeros((240,320,3)),0.,None,None,camera_model=model)
        self.assertEqual(core._identity_history['reference_id'],2);self.assertEqual(core._identity_history['identity_matches'],73)
        # Independent original outer AND changed conditions stay independent.
        core,ref,old,now,fit,model=self.fixture(shift=0.);fit['overlap']=ref['self_overlap']
        angle=.07;core.last_camera_model=dict(model,rotation=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1.]]))
        self.assertFalse(core._identity_original_maintenance(ref,fit))
        core.last_camera_model=core._camera_model(model);fit['overlap']=.5
        self.assertFalse(core._identity_original_maintenance(ref,fit))

    def test_actual_registered_conflict_rolls_back_and_clears_trend_history(self):
        fixture=AnchorRetentionTests();fixture.scene=self.scene
        core,origin,old,latest=fixture.library(latest_offset=.065,latest_sigma=.0045)
        self.assertIsNotNone(core._identity_history);before=[r['id'] for r in core.references];number=core.reference_number
        image,points,model=self.scene.frame(35.,body=(.10,0.,0.))
        result=core.update(.1,image,points,0.,None,None,camera_model=model)
        self.assertEqual(result['reason'],'ambiguous_rgbd_reference_translation')
        self.assertEqual([r['id'] for r in core.references],before);self.assertEqual(core.reference_number,number)
        self.assertIsNone(core._identity_history)

    def test_reserve_cannot_install_self_uncertain_template(self):
        core,ref,old,now,fit,model=self.fixture();self.assertTrue(core._identity_support_reserve(ref)['needed'])
        before=[r['id'] for r in core.references]
        core._install_reference(now,np.eye(3)*.05**2,ref['id'])
        self.assertEqual([r['id'] for r in core.references],before)
        self.assertEqual(core._reference_build_attempts[-1]['reason'],'uncertain_rgbd_translation')


if __name__ == '__main__':
    unittest.main()
