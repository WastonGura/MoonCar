"""Late pose attachment stays exact, idempotent and quality fenced."""
from pathlib import Path
import base64
import hashlib
import json
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

HERE=Path(__file__).resolve().parent
ROOT=next(parent for parent in HERE.parents if (parent/'scripts/autonomous_survey.py').is_file())
sys.path[:0]=[str(HERE),str(ROOT/'scripts'),str(ROOT/'ros2_ws/src/mooncar_terrain/tests')]
import autonomous_survey
import localization_revisit as policy
import test_localization_revisit as original
import test_localization_revisit_integration as actor


class RevisitLateAttachmentTests(unittest.TestCase):
    def test_late_exact_pose_attaches_once_without_reobserving_quality(self):
        tracker=policy.RGBDRevisitPolicy();sample=original.sample(1.)
        tracker.observe(sample,None)
        self.assertEqual(tracker.last_stamp,1.)
        self.assertEqual(tracker.observations,[])
        self.assertTrue(tracker.attach_observation(sample,original.observation(1.)))
        self.assertEqual(len(tracker.observations),1)
        self.assertEqual(len(tracker.breadcrumbs),1)
        self.assertFalse(tracker.attach_observation(sample,original.observation(1.)))
        self.assertEqual((len(tracker.observations),len(tracker.breadcrumbs)),(1,1))

    def test_warning_and_retirement_apply_before_any_pose_retry(self):
        tracker=policy.RGBDRevisitPolicy();tracker.observe(original.sample(1.),original.observation(1.))
        sample=original.sample(2.,ref=20,std=(.03,.025,.03));sample['retired_reference_id']=7
        tracker.observe(sample,None)
        trigger=tracker.pending['trigger']
        self.assertEqual(tracker.observations,[])
        self.assertIs(trigger,sample)
        self.assertTrue(tracker.attach_observation(sample,original.observation(2.)))
        self.assertFalse(tracker.attach_observation(sample,original.observation(2.)))
        self.assertIs(tracker.pending['trigger'],trigger)
        self.assertEqual(tracker.retirement_generation,0)
        self.assertEqual(tracker.observations,[])

    def test_wrong_exposure_session_counter_or_old_pose_never_attach(self):
        for kind in ('wrong_pose_stamp','old_sample','new_sample','session','fault'):
            with self.subTest(kind=kind):
                tracker=policy.RGBDRevisitPolicy();sample=original.sample(1.)
                tracker.observe(sample,None);bad=dict(sample);observation=original.observation(1.)
                if kind=='wrong_pose_stamp':observation['stamp']=.9
                elif kind=='old_sample':bad['stamp']=.9
                elif kind=='new_sample':bad['stamp']=1.1
                elif kind=='session':bad['session_id']='other'
                else:bad['fault_counter']=1
                with self.assertRaises(ValueError):tracker.attach_observation(bad,observation)
                self.assertEqual(tracker.observations,[])
                self.assertEqual(tracker.breadcrumbs,[])

    def test_new_quality_exposure_prevents_late_attachment_of_old_one(self):
        tracker=policy.RGBDRevisitPolicy();old=original.sample(1.)
        tracker.observe(old,None);tracker.observe(original.sample(1.1),None)
        with self.assertRaises(ValueError):tracker.attach_observation(old,original.observation(1.))
        self.assertEqual(tracker.observations,[])

    def test_typed_retirement_replan_can_attach_new_exposure_without_replaying_event(self):
        tracker=policy.RGBDRevisitPolicy();tracker.observe(original.sample(1.),original.observation(1.))
        trigger=original.sample(2.,ref=20,std=(.03,.025,.03));tracker.observe(trigger,None)
        tracker.pending['checkpoint']=dict(original.sample(1.),**original.observation(1.))
        tracker.running=True
        sample=original.sample(2.1,ref=20,std=(.03,.025,.03));sample['retired_reference_id']=7
        with self.assertRaises(policy.RetiredCheckpoint)as caught:tracker.observe(sample,None)
        tracker.invalidate_retired_checkpoint(caught.exception)
        self.assertTrue(tracker.attach_observation(sample,original.observation(2.1)))
        self.assertFalse(tracker.attach_observation(sample,original.observation(2.1)))
        self.assertIs(tracker.pending['trigger'],trigger)
        self.assertEqual(tracker.retirement_generation,1)
        self.assertEqual(tracker.retired_observation_ids.count(7),1)

    def test_pose_extra_fields_cannot_replace_frozen_measurement_facts(self):
        tracker=policy.RGBDRevisitPolicy();sample=original.sample(1.)
        tracker.observe(sample,None)
        observation=dict(original.observation(1.),reference_id=999,session_id='invented',
            position_std_m=[0.,0.,0.],trace_xy_sigma_upper_m=0.,fault_counter=999)
        self.assertTrue(tracker.attach_observation(sample,observation))
        row=tracker.observations[0]
        for key in ('reference_id','session_id','position_std_m','trace_xy_sigma_upper_m','fault_counter'):
            self.assertEqual(row[key],sample[key])


class RevisitExactCaptureRetryTests(unittest.TestCase):
    def setUp(self):
        self.original=actor.RevisitIntegrationTests()
        self.original.setUp();self.addCleanup(self.original.doCleanups)

    def fixture(self):
        survey,motion,state=self.original.fixture()
        sample=original.sample(10.,std=(.012,.010,.014))
        survey.revisit_policy.observe(sample,None)
        survey.revisit_sample=Mock(return_value=sample)
        return survey,motion,sample

    def test_transient_exact_tf_miss_then_same_exposure_pose_attaches_once(self):
        survey,motion,sample=self.fixture();observation=original.observation(10.)
        survey.revisit_observation=Mock(side_effect=[None,observation])
        with patch.object(autonomous_survey.time,'sleep',side_effect=AssertionError('capture must never wait')):
            self.assertFalse(survey.capture_revisit_observation(sample))
            self.assertTrue(survey.capture_revisit_observation(sample))
            self.assertFalse(survey.capture_revisit_observation(sample))
        self.assertEqual(survey.revisit_observation.call_args_list[0].args,(10.,))
        self.assertEqual(survey.revisit_observation.call_args_list[1].args,(10.,))
        self.assertEqual(survey.revisit_observation.call_count,2)
        self.assertEqual(len(survey.revisit_policy.observations),1)
        self.assertEqual(survey.revisit_capture_stats['retries'],1)
        self.assertEqual(survey.revisit_capture_stats['attached'],1)

    def test_exact_tf_attempts_are_bounded_to_three_and_new_exposure_gets_new_budget(self):
        survey,motion,sample=self.fixture();survey.revisit_observation=Mock(return_value=None)
        for _ in range(8):self.assertFalse(survey.capture_revisit_observation(sample))
        self.assertEqual(survey.revisit_observation.call_count,3)
        self.assertEqual(survey.revisit_capture_stats['attempts'],3)
        self.assertEqual(survey.revisit_capture_stats['retries'],2)
        newer=original.sample(10.1,std=(.012,.010,.014))
        survey.revisit_policy.observe(newer,None);survey.revisit_sample.return_value=newer
        self.assertFalse(survey.capture_revisit_observation(newer))
        self.assertEqual(survey.revisit_observation.call_count,4)
        self.assertEqual(survey.revisit_capture_stats['exposures'],2)

    def test_invalid_exact_tf_is_permanent_not_a_retryable_missing_transform(self):
        survey,motion,sample=self.fixture()
        def invalid(stamp):
            survey.revisit_capture_last_failure=dict(reason='invalid_exact_tf',retryable=False)
            return None
        survey.revisit_observation=Mock(side_effect=invalid)
        for _ in range(5):self.assertFalse(survey.capture_revisit_observation(sample))
        self.assertEqual(survey.revisit_observation.call_count,1)
        self.assertEqual(survey.revisit_capture_stats['permanent_failures'],1)
        self.assertEqual(survey.revisit_policy.observations,[])

    def test_quality_or_session_change_after_lookup_cannot_attach(self):
        for kind in ('unhealthy','session','fault'):
            with self.subTest(kind=kind):
                survey,motion,sample=self.fixture()
                survey.revisit_observation=Mock(return_value=original.observation(10.))
                if kind=='unhealthy':survey.revisit_sample.side_effect=ValueError('actual quality fault')
                else:
                    current=dict(sample)
                    current['session_id'if kind=='session'else'fault_counter']='other'if kind=='session'else 1
                    survey.revisit_sample.return_value=current
                with self.assertRaises(ValueError):survey.capture_revisit_observation(sample)
                self.assertEqual(survey.revisit_policy.observations,[])

    def test_superseded_exposure_or_selected_reference_cannot_attach_or_retry(self):
        for kind in ('stamp','reference_id'):
            with self.subTest(kind=kind):
                survey,motion,sample=self.fixture()
                survey.revisit_observation=Mock(return_value=original.observation(10.))
                current=dict(sample);current[kind]=10.1 if kind=='stamp'else 99
                survey.revisit_sample.return_value=current
                self.assertFalse(survey.capture_revisit_observation(sample))
                self.assertFalse(survey.capture_revisit_observation(sample))
                self.assertEqual(survey.revisit_observation.call_count,1)
                self.assertEqual(survey.revisit_policy.observations,[])

    def test_wrong_actual_tf_exposure_never_becomes_a_checkpoint(self):
        survey,motion,sample=self.fixture()
        survey.revisit_observation=Mock(return_value=original.observation(9.9))
        with self.assertRaises(ValueError):survey.capture_revisit_observation(sample)
        self.assertEqual(survey.revisit_policy.observations,[])

    def test_pose_retry_does_not_delay_actual_warning_or_cancel(self):
        survey,motion,state=self.original.fixture()
        survey.revisit_observation=Mock(return_value=None)
        with patch.object(autonomous_survey.time,'sleep',side_effect=AssertionError('no quality delay')):
            survey.update_localization_revisit()
        self.assertIsNotNone(survey.revisit_policy.pending)
        self.assertEqual(survey.revisit_policy.last_stamp,10.)
        motion.cancel.assert_called_once()
        self.assertEqual(survey.revisit_capture_stats['attempts'],1)

    def test_revalidation_preserves_outer_receipt_transaction_on_success_or_failure(self):
        for kind in ('success','superseded','fault'):
            with self.subTest(kind=kind):
                survey,motion,sample=self.fixture()
                outer=dict(cursor=(2,8),rows=[dict(source='outer')])
                survey.revisit_receipt_snapshot=outer;survey.revisit_receipt_cursor=(2,7)
                survey.revisit_observation=Mock(return_value=original.observation(10.))
                def checked():
                    survey.revisit_receipt_snapshot=dict(cursor=(2,9),rows=[dict(source='inner')])
                    if kind=='fault':raise ValueError('actual health fault during revalidation')
                    return dict(sample,stamp=10.1)if kind=='superseded'else sample
                survey.revisit_sample=Mock(side_effect=checked)
                if kind=='fault':
                    with self.assertRaises(ValueError):survey.capture_revisit_observation(sample)
                else:self.assertEqual(survey.capture_revisit_observation(sample),kind=='success')
                self.assertIs(survey.revisit_receipt_snapshot,outer)
                self.assertEqual(survey.revisit_receipt_cursor,(2,7))

    def test_research_sidecar_writes_actual_bounded_inputs_and_only_reference_in_state(self):
        survey,motion,sample=self.fixture()
        survey.revisit_policy.attach_observation(sample,original.observation(10.))
        trigger=original.sample(10.1,ref=20,std=(.03,.025,.03))
        survey.revisit_policy.observe(trigger,None)
        survey.revisit_policy.plan_diagnostics=dict(mode='normal_exact',route_validations=0,
            rejections=[dict(stamp=10.,reference_id=7,reason='target_cost')])
        survey.revisit_capture_stats['attempts']=2
        grid=motion.grid;grid.header=NS(stamp=NS(sec=9,nanosec=123000000))
        pose=dict(x=.1,y=.2,yaw=.3);receipt=dict(captured_sim=10.,source='actual_test_grid')
        with tempfile.TemporaryDirectory()as folder:
            survey.path=Path(folder)/'autonomy-test.json'
            record={}
            survey.persist_revisit_plan_snapshot(record,grid,pose,77.,[.1,-.2],receipt)
            self.assertEqual(set(record),{'planning_evidence'})
            reference=record['planning_evidence']
            self.assertEqual(set(reference),{'path','sha256','observations','breadcrumbs','capture_attempts','candidate_counts'})
            destination=survey.path.parent/reference['path'];contents=destination.read_bytes()
            self.assertEqual(hashlib.sha256(contents).hexdigest(),reference['sha256'])
            value=json.loads(contents)
            self.assertEqual(value['observations'],survey.revisit_policy.observations)
            self.assertEqual(value['breadcrumbs'],survey.revisit_policy.breadcrumbs)
            self.assertEqual(value['current_pose'],pose)
            self.assertEqual(value['map_header_stamp'],9.123)
            self.assertEqual(value['map_snapshot'],receipt)
            self.assertEqual(value['capture']['attempts'],2)
            self.assertEqual(value['candidate_diagnostics']['rejections'][0]['reason'],'target_cost')
            self.assertEqual(base64.b64decode(value['navigation_grid']['data']),bytes([0])*400)
            self.assertEqual((reference['observations'],reference['breadcrumbs']),(1,1))
            self.assertNotIn('rejections',reference['candidate_counts'])


if __name__=='__main__':unittest.main()
