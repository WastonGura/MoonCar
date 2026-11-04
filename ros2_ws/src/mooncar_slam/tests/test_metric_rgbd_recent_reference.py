"""Recent measured-template transaction boundaries; no ROS or archived inputs.

Actual images exercise ordinary tracking, raw ownership and the covariance
gate. Controlled fit returns isolate role budgets and the pairwise conflict
transaction; they are not evidence of new visual identities.
"""
import copy
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

TEST_DIRECTORY = Path(__file__).resolve().parent
sys.path[:0] = [str(TEST_DIRECTORY), str(TEST_DIRECTORY.parent/'scripts')]
import metric_rgbd_odometry as module
from test_metric_rgbd_odometry import PinholePlane


class RecentMeasuredReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(1)
        cls.scene = PinholePlane()

    def normal_core(self):
        core = module.MetricRGBDOdometry()
        self.addCleanup(core.close)
        for n in range(3):
            image, grid, model = self.scene.frame(35.)
            result = core.update(n*.1, image, grid, 0., 0., 0., camera_model=model)
            self.assertTrue(result['accepted'], result)
        return core

    def transaction(self, covariance=None):
        """Twenty measured-view fixtures, plus one owned unregistered record."""
        core = module.MetricRGBDOdometry()
        self.addCleanup(core.close)
        image, grid, model = self.scene.frame(35.)
        previous = core._features(image, grid)
        core.last_camera_model = core._camera_model(model)
        core.xyz = np.zeros(3); core.yaw = 0.; core.stamp = .2
        cov = np.diag([.01**2, .012**2, .013**2]) if covariance is None else covariance
        result = dict(xyz=core.xyz.copy(), covariance=cov.copy(), yaw=0., reference_id=20)
        core._retain_recent_measurement(result, previous, 200_000_000)
        base = core._reference(previous, np.eye(3)*1e-4, None)
        base['self_overlap'] = 1.
        core.references = [dict(base, id=n, parent=None) for n in range(1, 21)]
        core.reference_number = 20
        core._active_view_id = 20
        core.last_reference = (np.zeros(3), 0.)
        core.last_reference_view = previous['points'].mean(axis=0)
        core.last_reference_overlap = 1.
        new_image, new_grid, _ = self.scene.frame(35., body=(.02, 0., 0.))
        current = core._features(new_image, new_grid)
        core._frame_prediction = np.zeros(3)
        core._frame_fits = {}; core._anchor_adoption = {}
        core._recent_fit_calls = 5
        attempts = [dict(reference_id=n, reason='insufficient_rgbd_identity_matches', accepted=False)
                    for n in (1, 20, 19, 18, 17)]
        core._attempted_references = copy.deepcopy(attempts)
        rejected = dict(accepted=False, reason='insufficient_rgbd_identity_matches',
                        wheel_prediction_used=False, reference_attempts=attempts)
        core._identity_history = dict(stamp_ns=200_000_000, interval_ns=100_000_000,
            reference_id=20, identity_matches=40,
            features={key:previous[key].copy() for key in ('descriptors','pixels','pixel_scales')},
            image_shape=tuple(previous['image_shape']), K=model['K'].copy())
        return core, current, rejected

    def role_fits(self, core, *, cross_reason=None, near_offsets=None):
        """A fit-role oracle, with real covariance conflict arithmetic retained."""
        saved = core._recent_measurement
        near_offsets = near_offsets or {}
        observed = []
        def fitted(features, prediction, reference):
            core._recent_fit_calls += 1
            role = ('previous_self' if features is saved['features'] else
                    'current_previous' if reference['id'] == 21 else 'registered_conflict')
            observed.append((role, reference['id'], reference['anchor_covariance'].copy()))
            if role == 'current_previous' and cross_reason:
                core._frame_fits[reference['id']] = None
                return None, cross_reason
            if role == 'registered_conflict' and reference['id'] not in near_offsets:
                core._frame_fits[reference['id']] = None
                return None, 'insufficient_rgbd_identity_matches'
            xyz = saved['xyz'].copy()
            if role == 'registered_conflict': xyz[0] += near_offsets[reference['id']]
            # Deliberately distinguish selfC from the accepted-frame anchorC.
            covariance = np.eye(3)*(1e-3 if role == 'previous_self' else 1e-4)
            value = dict(xyz=xyz, covariance=covariance, measurement_covariance=np.eye(3)*9e-6,
                         reference_id=reference['id'], identity_matches=40, inliers=40,
                         overlap=.8, rmse_m=.004, reference_support={})
            core._frame_fits[reference['id']] = value
            return value, 'accepted'
        return fitted, observed

    def call_recent(self, core, current, rejected, cadence=True):
        return core._try_recent_measurement(rejected, current, 300_000_000, 200_000_000, cadence)

    def test_ordinary_success_has_no_rescue_fit_or_result_changes(self):
        ordinary = module.MetricRGBDOdometry(); detached = module.MetricRGBDOdometry()
        self.addCleanup(ordinary.close); self.addCleanup(detached.close)
        def plain(value):
            if isinstance(value,np.ndarray): return value.tolist()
            if isinstance(value,np.generic): return value.item()
            if isinstance(value,dict): return {k:plain(v) for k,v in value.items()}
            if isinstance(value,(tuple,list)): return [plain(v) for v in value]
            return value
        with patch.object(ordinary,'_try_recent_measurement',side_effect=AssertionError('ordinary fit must not rescue')), \
                patch.object(detached,'_retain_recent_measurement',return_value=None):
            for n in range(5):
                image, grid, model = self.scene.frame(35., body=(n*.01,0.,0.))
                left = ordinary.update(n*.1,image,grid,0.,0.,0.,camera_model=model)
                right = detached.update(n*.1,image,grid,0.,0.,0.,camera_model=model)
                self.assertTrue(left['accepted'],left)
                self.assertEqual(plain(left),plain(right))
                self.assertNotIn('recent_reference',left)
                self.assertEqual([r['id'] for r in ordinary.references],[r['id'] for r in detached.references])
                self.assertEqual(ordinary._recent_fit_calls,detached._recent_fit_calls)

    def test_only_five_complete_failed_attempts_can_enter_rescue(self):
        core, current, rejected = self.transaction()
        variants = [dict(rejected,reference_attempts=rejected['reference_attempts'][:4]),
                    dict(rejected,reference_attempts=[dict(r,accepted=(i==0)) for i,r in enumerate(rejected['reference_attempts'])])]
        for reason in ('ambiguous_rgbd_reference_translation','invalid_rgbd_input',
                       'nonincreasing_rgbd_stamp','rgbd_gap_requires_new_session'):
            attempts=copy.deepcopy(rejected['reference_attempts']);attempts[0]['reason']=reason
            variants.append(dict(rejected,reference_attempts=attempts))
        with patch.object(core,'_fit',side_effect=AssertionError('forbidden rescue fit')):
            for value in variants:self.assertIs(self.call_recent(core,current,value),value)
            self.assertIs(self.call_recent(core,current,rejected,cadence=False),rejected)
            core._recent_measurement['stamp_ns']-=1
            self.assertIs(self.call_recent(core,current,rejected),rejected)

    def test_rescue_total_nine_six_tracking_and_twenty_capacity_past_stamp(self):
        core, current, rejected = self.transaction(); saved=core._recent_measurement
        accepted_cov=saved['covariance'].copy(); fitted, observed=self.role_fits(core)
        with patch.object(core,'_fit',side_effect=fitted), \
                patch.object(core,'_angular_near_view',side_effect=lambda ref:ref['id'] in (2,3)), \
                patch.object(core,'_projected_view_motion',side_effect=lambda ref:dict(projected_median_shift_px=ref['id'])):
            result=self.call_recent(core,current,rejected)
        self.assertTrue(result['accepted'],result)
        self.assertEqual([r[0] for r in observed],['previous_self','current_previous','registered_conflict','registered_conflict'])
        self.assertEqual(result['recent_reference']['total_fit_calls'],9)
        self.assertEqual(len(result['reference_attempts']),6)
        self.assertEqual(len(core.references),20)
        self.assertEqual(core.references[0]['id'],1)
        new=next(ref for ref in core.references if ref['id']==result['reference_id'])
        np.testing.assert_array_equal(new['anchor_covariance'],accepted_cov)
        self.assertFalse(np.array_equal(new['anchor_covariance'],np.eye(3)*1e-3))
        self.assertEqual(new['measurement_stamp_ns'],200_000_000)
        self.assertEqual(result['stamp'],.3)
        self.assertEqual(result['recent_reference']['measurement_exposure_stamp'],.2)
        self.assertEqual(result['view_admission']['recent_reference_registration']['measurement_exposure_stamp'],.2)

    def test_previous_self_uncertain_cannot_be_replaced_by_its_self_covariance(self):
        core,current,rejected=self.transaction(np.eye(3)*.05**2)
        before=[r['id'] for r in core.references]
        result=self.call_recent(core,current,rejected)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['recent_reference']['self_validation_reason'],'uncertain_rgbd_translation')
        self.assertEqual(result['recent_reference']['reason'],'previous_self_rejected')
        self.assertEqual(core._recent_fit_calls,6)
        self.assertEqual([r['id'] for r in core.references],before)
        self.assertEqual(core.reference_number,20)
        self.assertNotIn(21,core._frame_fits)

    def test_cross_frame_rejection_preserves_library_pose_and_number(self):
        core,current,rejected=self.transaction();before=list(core.references);xyz=core.xyz.copy()
        fitted,observed=self.role_fits(core,cross_reason='rgbd_registration_residual')
        with patch.object(core,'_fit',side_effect=fitted):result=self.call_recent(core,current,rejected)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['recent_reference']['cross_frame_reason'],'rgbd_registration_residual')
        self.assertEqual(len(observed),2)
        self.assertEqual([id(r)for r in core.references],[id(r)for r in before])
        np.testing.assert_array_equal(core.xyz,xyz)
        self.assertEqual(core.reference_number,20)

    def test_pairwise_nontransitive_strong_conflict_cannot_hide_behind_current_fit(self):
        core,current,rejected=self.transaction();before=list(core.references);xyz=core.xyz.copy()
        displacement=.75*math.sqrt(core.residual_chi_square*2e-4)
        fitted,observed=self.role_fits(core,near_offsets={2:displacement,3:-displacement})
        with patch.object(core,'_fit',side_effect=fitted), \
                patch.object(core,'_angular_near_view',side_effect=lambda ref:ref['id'] in (2,3)), \
                patch.object(core,'_projected_view_motion',side_effect=lambda ref:dict(projected_median_shift_px=ref['id'])):
            result=self.call_recent(core,current,rejected)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'],'ambiguous_rgbd_reference_translation')
        self.assertEqual(result['recent_reference']['reason'],'registered_view_conflict')
        self.assertEqual(len(observed),4)
        self.assertEqual([id(r)for r in core.references],[id(r)for r in before])
        np.testing.assert_array_equal(core.xyz,xyz)
        self.assertEqual(core.reference_number,20)
        self.assertNotIn(21,core._frame_fits)

    def test_owned_actual_raw_grid_pose_covariance_and_camera_model(self):
        core=module.MetricRGBDOdometry();self.addCleanup(core.close)
        image,grid,model=self.scene.frame(35.)
        for n in range(3):self.assertTrue(core.update(n*.1,image,grid,0.,0.,0.,camera_model=model)['accepted'])
        record=core._recent_measurement
        saved={key:record[key].copy() for key in ('xyz','covariance')}
        rawgrid=record['features']['_measured_grid'].copy();gray=record['features']['_measured_gray'].copy()
        camera={key:value.copy() for key,value in record['camera_model'].items()}
        self.assertFalse(np.shares_memory(record['features']['_measured_grid'],grid))
        self.assertEqual(record['stamp_ns'],200_000_000)
        self.assertEqual(record['yaw'],0.)
        image[:]=0;grid[:]=np.nan
        for value in model.values():value[:]=77.
        np.testing.assert_array_equal(record['features']['_measured_grid'],rawgrid)
        np.testing.assert_array_equal(record['features']['_measured_gray'],gray)
        for key,value in saved.items():np.testing.assert_array_equal(record[key],value)
        for key,value in camera.items():np.testing.assert_array_equal(record['camera_model'][key],value)

    def test_calibration_reject_gap_exception_and_close_cannot_leak_a_rescue(self):
        core,current,rejected=self.transaction()
        current_model=core.last_camera_model
        changed=dict(current_model,K=current_model['K'].copy());changed['K'][0,0]+=1.
        core.last_camera_model=changed
        with patch.object(core,'_fit',side_effect=AssertionError('changed calibration cannot rescue')):
            self.assertIs(self.call_recent(core,current,rejected),rejected)
        normal=self.normal_core();image,grid,model=self.scene.frame(35.)
        result=normal.update(.3,np.zeros_like(image),grid,0.,0.,0.,camera_model=model)
        self.assertFalse(result['accepted']);self.assertIsNone(normal._recent_measurement)
        fresh=self.normal_core()
        with self.assertRaises((TypeError,ValueError)):
            fresh.update('bad',image,grid,0.,0.,0.,camera_model=model)
        self.assertIsNone(fresh._recent_measurement)
        closed=self.normal_core();closed.close();self.assertIsNone(closed._recent_measurement)
        gap=self.normal_core()
        value=gap.update(.5,image,grid,0.,0.,0.,camera_model=model)
        self.assertTrue(value['accepted']);self.assertIsNone(gap._identity_history['interval_ns'])
        self.assertEqual(gap._recent_measurement['stamp_ns'],500_000_000)

    def test_exception_after_retention_mutation_restores_entire_accepted_library(self):
        core,current,rejected=self.transaction();before=list(core.references);xyz=core.xyz.copy()
        fitted,_=self.role_fits(core)
        original=core._retire_redundant_reference
        def fail_after_commit(*args,**kwargs):
            original(*args,**kwargs)
            self.assertEqual(len(core.references),20)
            raise RuntimeError('injected after provisional reference commit')
        with patch.object(module._MetricRGBDReferenceMaintenance,'update',return_value=rejected), \
                patch.object(core,'_fit',side_effect=fitted), \
                patch.object(core,'_angular_near_view',return_value=False), \
                patch.object(core,'_retire_redundant_reference',side_effect=fail_after_commit):
            with self.assertRaisesRegex(RuntimeError,'provisional reference commit'):
                core.update(.3,np.zeros((240,320),np.uint8),np.zeros((240,320,3)),0.,0.,0.,camera_model=core.last_camera_model)
        self.assertEqual([id(r)for r in core.references],[id(r)for r in before])
        np.testing.assert_array_equal(core.xyz,xyz)
        self.assertEqual(core.reference_number,20)
        self.assertIsNone(core._recent_measurement)
        self.assertIsNone(core._identity_history)


if __name__ == '__main__':
    unittest.main()
