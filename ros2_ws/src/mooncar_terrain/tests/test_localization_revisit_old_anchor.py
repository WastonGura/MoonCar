"""Actual healthy receipt provenance and independent old-anchor proof.

No motion, ROS process, estimator update, or synthetic historical Motion
counter is used. Receipt fixtures pass through the actual consumer API.
"""
import copy
from pathlib import Path
import sys
import unittest

HERE=Path(__file__).resolve().parent
ROOT=next(parent for parent in HERE.parents if (parent/'scripts/autonomous_survey.py').is_file())
sys.path[:0]=[str(HERE),str(ROOT/'scripts')]
from localization_quality import LocalizationQuality
from localization_revisit import ReobservedAnchorProof, RGBDRevisitPolicy, sample_payload


class ReceiptFixtures:
    def __init__(self,required=True):
        self.consumer=LocalizationQuality(required=required)
        self.sequence=0

    def emit(self,stamp,ref=7,std=(.012,.010,.014),*,attempts=None,adoption=None,
             backend='metric_rgbd_3d',session='producer-a',accepted_stamp=None,healthy=True):
        self.sequence+=1
        actual=stamp if accepted_stamp is None else accepted_stamp
        fit=dict(accepted=True,exposure_stamp=actual,reference_id=ref,position_std_m=list(std),
                 reference_attempts=([dict(reference_id=ref,reason='accepted',accepted=True)]
                                     if attempts is None else copy.deepcopy(attempts)))
        if adoption is not None:fit['view_admission']=dict(anchor_adoption=copy.deepcopy(adoption))
        value=dict(schema_version=1,localization_backend=backend,session_id=session,
            sequence=self.sequence,stamp=stamp,accepted_stamp=actual,ready_since_stamp=1.,
            healthy=healthy,reason='healthy'if healthy else'observation_stale',
            successful_streak=3 if healthy else 0,last_fit=fit)
        wall=100.+stamp
        self.consumer.observe(value,wall)
        answer=self.consumer.check(stamp,wall)
        return value,answer

    @staticmethod
    def live(value,answer,counter=0):
        # Only the actual live Motion check supplies this cumulative counter.
        return sample_payload(value,dict(answer,fault_counter=counter))


class OldAnchorReceiptTests(unittest.TestCase):
    def setup_proof(self):
        fixture=ReceiptFixtures()
        value,health=fixture.emit(1.)
        checkpoint=fixture.live(value,health)
        value,health=fixture.emit(2.,50,(.030,.025,.030))
        trigger=fixture.live(value,health)
        proof=ReobservedAnchorProof()
        self.consume(fixture,proof,trigger)
        return fixture,proof,trigger,checkpoint

    def consume(self,fixture,proof,trigger,cursor=None):
        batch=fixture.consumer.revisit_receipts(cursor)
        proof.consume(batch['rows'],trigger,batch['consumer_health_epoch'])
        return batch['cursor']

    def event_and_stable(self,fixture,proof,trigger,*,event_ref=7,attempts=None,adoption=None,
                         arrived=5.5,child=200):
        cursor=fixture.consumer.revisit_receipts()['cursor']
        fixture.emit(3.,event_ref,attempts=attempts,adoption=adoption)
        live=None
        for stamp in (6.,6.1,6.2):
            value,health=fixture.emit(stamp,child)
            live=fixture.live(value,health)
        self.consume(fixture,proof,trigger,cursor)
        return live,arrived

    def test_receipts_distinct_exposure_not_heartbeat_and_actual_wall_times(self):
        fixture=ReceiptFixtures()
        first,health=fixture.emit(1.)
        self.assertTrue(health['healthy'])
        batch=fixture.consumer.revisit_receipts()
        row=batch['rows'][0]
        self.assertNotIn('fault_counter',row['health'])
        self.assertEqual((row['received_wall'],row['checked_wall'],row['checked_sim']),(101.,101.,1.))
        cursor=batch['cursor']
        fixture.consumer.check(1.01,101.1)
        fixture.emit(1.02,accepted_stamp=1.)
        self.assertEqual(fixture.consumer.revisit_receipts(cursor)['rows'],[])
        fixture.emit(1.1)
        self.assertEqual([r['payload']['accepted_stamp']for r in fixture.consumer.revisit_receipts(cursor)['rows']],[1.1])
        self.assertEqual(first['accepted_stamp'],1.)

    def test_disabled_other_backend_and_unhealthy_never_record(self):
        for required,backend in ((False,'metric_rgbd_3d'),(True,'metric_laser_2d')):
            with self.subTest(required=required,backend=backend):
                fixture=ReceiptFixtures(required)
                fixture.emit(1.,backend=backend)
                self.assertEqual(fixture.consumer.revisit_receipts()['rows'],[])
        fixture=ReceiptFixtures()
        fixture.emit(1.,healthy=False)
        self.assertEqual(fixture.consumer.revisit_receipts()['rows'],[])

    def test_history_detaches_input_and_exported_aliases(self):
        fixture=ReceiptFixtures()
        value,health=fixture.emit(1.,adoption=dict(adopted_reference_id=7,selected_reference_id=7))
        expected=fixture.consumer.revisit_receipts()
        value['last_fit']['position_std_m'][0]=123.
        value['last_fit']['reference_attempts'][0]['accepted']=False
        value['last_fit']['view_admission']['anchor_adoption']['selected_reference_id']=99
        health['healthy']=False
        exported=fixture.consumer.revisit_receipts()
        self.assertEqual(exported,expected)
        exported['rows'][0]['payload']['last_fit']['position_std_m'][0]=456.
        exported['rows'][0]['health']['healthy']=False
        self.assertEqual(fixture.consumer.revisit_receipts(),expected)

    def test_bounded_history_explicit_overflow_and_reset_cursor(self):
        fixture=ReceiptFixtures()
        initial=fixture.consumer.revisit_receipts()['cursor']
        for index in range(130):fixture.emit(1.+index*.01)
        batch=fixture.consumer.revisit_receipts()
        self.assertEqual(len(batch['rows']),128)
        self.assertEqual(batch['rows'][0]['cursor'],3)
        with self.assertRaises(ValueError):fixture.consumer.revisit_receipts(initial)
        for cursor in ((True,0),(batch['cursor'][0],999),('wrong',1)):
            with self.subTest(cursor=cursor),self.assertRaises(ValueError):fixture.consumer.revisit_receipts(cursor)
        fixture.consumer.reset()
        with self.assertRaises(ValueError):fixture.consumer.revisit_receipts(batch['cursor'])
        self.assertEqual(fixture.consumer.revisit_receipts()['rows'],[])

    def test_health_epoch_discards_preoutage_catalogue_and_blocks_active_proof(self):
        fixture=ReceiptFixtures();fixture.emit(1.)
        batch=fixture.consumer.revisit_receipts();proof=ReobservedAnchorProof()
        proof.consume(batch['rows'],health_epoch=batch['consumer_health_epoch'])
        self.assertIn(7,proof.qualified)
        fixture.consumer.check(1.31,101.31)
        after=fixture.consumer.revisit_receipts()
        self.assertGreater(after['consumer_health_epoch'],batch['consumer_health_epoch'])
        proof.consume(after['rows'],health_epoch=after['consumer_health_epoch'])
        self.assertEqual(proof.qualified,{})
        fixture,proof,trigger,checkpoint=self.setup_proof()
        with self.assertRaises(ValueError):proof.consume([],trigger,1)

    def test_final_selected_actual_old_fit_then_three_new_postview_exposures(self):
        fixture,proof,trigger,checkpoint=self.setup_proof()
        live,arrived=self.event_and_stable(fixture,proof,trigger)
        self.assertTrue(proof.prove(live,arrived,checkpoint))
        evidence=proof.evidence()
        self.assertEqual(list(evidence['eligible_old_references']),['7'])
        self.assertEqual(evidence['actual_old_fit_events'][0]['reference_id'],7)
        self.assertEqual(evidence['actual_old_fit_events'][0]['direct_fit_kind'],'tracking')
        self.assertEqual(evidence['proof_exposure_stamps'],[6.,6.1,6.2])
        self.assertEqual(live['reference_id'],200)
        strict=RGBDRevisitPolicy();strict.pending=dict(checkpoint=checkpoint,trigger=trigger)
        self.assertFalse(strict.prove(live,arrived))

    def test_additional_fit_requires_final_adoption_not_unused_fit_or_ancestry(self):
        for label,ref,attempts,adoption,expected in (
            ('real_adoption',7,[dict(reference_id=200,reason='accepted',accepted=True)],
             dict(adopted_reference_id=7,selected_reference_id=7,additional_reference_id=7,additional_reason='accepted'),True),
            ('unused_fit',200,[dict(reference_id=7,reason='accepted',accepted=True)],None,False),
            ('ancestry_only',200,[],dict(adopted_reference_id=7,selected_reference_id=200),False),
            ('contradictory_claim',7,[dict(reference_id=7,reason='accepted',accepted=True)],
             dict(adopted_reference_id=200,selected_reference_id=7),False),
            ('failed_additional',7,[],dict(adopted_reference_id=7,selected_reference_id=7,
             additional_reference_id=7,additional_reason='uncertain'),False),
            ('covariance_only',200,[],None,False)):
            with self.subTest(label=label):
                fixture,proof,trigger,checkpoint=self.setup_proof()
                live,arrived=self.event_and_stable(fixture,proof,trigger,event_ref=ref,attempts=attempts,adoption=adoption)
                self.assertEqual(proof.prove(live,arrived,checkpoint),expected)

    def test_frozen_catalogue_rejects_trigger_and_new_low_uncertainty_ids(self):
        fixture,proof,trigger,checkpoint=self.setup_proof()
        cursor=fixture.consumer.revisit_receipts()['cursor']
        fixture.emit(2.5,99)
        fixture.emit(3.,99)
        live=None
        for stamp in (6.,6.1,6.2):
            value,health=fixture.emit(stamp,200);live=fixture.live(value,health)
        self.consume(fixture,proof,trigger,cursor)
        self.assertEqual(set(proof.eligible),{7})
        self.assertEqual(proof.events,[])
        self.assertFalse(proof.prove(live,5.5,checkpoint))

    def test_three_distinct_new_exposures_reset_on_bad_frame_and_new_view(self):
        fixture,proof,trigger,checkpoint=self.setup_proof()
        live,arrived=self.event_and_stable(fixture,proof,trigger)
        self.assertTrue(proof.prove(live,arrived,checkpoint))
        self.assertFalse(proof.prove(live,6.15,checkpoint))
        cursor=fixture.consumer.revisit_receipts()['cursor']
        value,health=fixture.emit(6.3,200,(.04,.04,.04));bad=fixture.live(value,health)
        self.consume(fixture,proof,trigger,cursor)
        self.assertFalse(proof.prove(bad,6.15,checkpoint))
        for stamp in (6.4,6.5):
            cursor=fixture.consumer.revisit_receipts()['cursor'];value,health=fixture.emit(stamp,200)
            self.consume(fixture,proof,trigger,cursor);live=fixture.live(value,health)
            self.assertFalse(proof.prove(live,6.15,checkpoint))
            self.assertFalse(proof.prove(live,6.15,checkpoint))
        cursor=fixture.consumer.revisit_receipts()['cursor'];value,health=fixture.emit(6.6,200)
        self.consume(fixture,proof,trigger,cursor)
        self.assertTrue(proof.prove(fixture.live(value,health),6.15,checkpoint))

    def test_live_motion_counter_session_and_receipt_exposure_mismatch_fail_closed(self):
        fixture,proof,trigger,checkpoint=self.setup_proof()
        live,arrived=self.event_and_stable(fixture,proof,trigger)
        for counter in (None,True,1):
            with self.subTest(counter=counter):
                self.assertFalse(proof.prove(dict(live,fault_counter=counter),arrived,checkpoint))
        self.assertFalse(proof.prove(dict(live,session_id='other'),arrived,checkpoint))
        fixture=ReceiptFixtures();value,health=fixture.emit(1.)
        row=fixture.consumer.revisit_receipts()['rows'][0]
        for mutate in ('sequence','stamp','std','session'):
            candidate=copy.deepcopy(row)
            if mutate=='sequence':candidate['payload']['sequence']+=1
            elif mutate=='stamp':candidate['payload']['last_fit']['exposure_stamp']=.9
            elif mutate=='std':candidate['payload']['last_fit']['position_std_m'][0]=float('nan')
            else:candidate['payload']['session_id']='other'
            with self.subTest(mutate=mutate),self.assertRaises(ValueError):
                ReobservedAnchorProof().consume([candidate],health_epoch=0)

    def test_duplicate_receipt_and_producer_switch_cannot_manufacture_exposures(self):
        fixture=ReceiptFixtures();fixture.emit(1.)
        batch=fixture.consumer.revisit_receipts();proof=ReobservedAnchorProof()
        proof.consume(batch['rows'],health_epoch=0)
        with self.assertRaises(ValueError):proof.consume(batch['rows'],health_epoch=0)
        fixture.emit(1.1,session='other-producer')
        self.assertFalse(fixture.consumer.check(1.1,101.1)['healthy'])
        self.assertEqual(len(fixture.consumer.revisit_receipts()['rows']),1)
        with self.assertRaises(ValueError):ReobservedAnchorProof().consume([],health_epoch=True)


if __name__=='__main__':unittest.main()
