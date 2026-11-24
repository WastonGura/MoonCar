"""Exercise real asynchronous workers and bounded failure paths."""
import json
from pathlib import Path
import queue
import sys
import tempfile
import time
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from r5_capture_io import EventWriter,ImageWriter,capture_hash,require_image_file,dds_info
from r5_capture_analysis import coverage
from run_r5_d0_acceptance import validate_calibration,CampaignStop
from run_r5_capture_campaign import qualify_input_pairs

def image(**changes):
    return dict(kind='rgb',stamp=1.,data=bytes([0,1,2]*4),height=2,width=2,step=6,encoding='rgb8',bigendian=False,**changes)

class CaptureTests(unittest.TestCase):
    def test_jazzy_dict_metadata_and_object_metadata_are_supported(self):
        from types import SimpleNamespace
        value=dict(source_timestamp=123,received_timestamp=456)
        expected=dict(dds_source_timestamp_ns=123,dds_received_timestamp_ns=456)
        self.assertEqual(dds_info(value),expected);self.assertEqual(dds_info(SimpleNamespace(**value)),expected)
        with self.assertRaises(KeyError):dds_info({})

    def test_slow_encoder_does_not_block_ingress_and_preserves_receipt_time(self):
        with tempfile.TemporaryDirectory() as folder:
            errors=[];done=[];writer=ImageWriter(folder,errors,done.append)
            received=time.monotonic();row=dict(kind='rgb',stamp=1.,sim=1.,wall=received)
            start=time.monotonic();writer.submit(image(test_delay=.2),row)
            self.assertLess(time.monotonic()-start,.1)
            status=writer.finish(time.monotonic()+10)
            self.assertTrue(status['drained']);self.assertFalse(errors);self.assertEqual(status['selected'],status['completed'])
            self.assertEqual(done[0]['wall'],received);self.assertGreater(done[0]['archive_completed_wall'],received)
            self.assertGreaterEqual(status['maximum_encode_seconds'],.2)

    def test_slow_json_writer_drains_all_events_without_mutating_rows(self):
        with tempfile.TemporaryDirectory() as folder:
            errors=[];writer=EventWriter(folder,errors,test_delay=.03)
            row=dict(event_id=1,kind='imu',wall=2,sim=3,stamp=3)
            start=time.monotonic()
            for index in range(1,5):writer.submit(dict(row,event_id=index))
            self.assertLess(time.monotonic()-start,.1)
            status=writer.finish(time.monotonic()+3)
            self.assertTrue(status['drained']);self.assertEqual(status['written'],4);self.assertFalse(errors)
            self.assertNotIn('json_archive_started_wall',row)
            data=[json.loads(line) for line in (Path(folder)/'observations.jsonl').read_text().splitlines()]
            receipts=[json.loads(line) for line in (Path(folder)/'writer-receipts.jsonl').read_text().splitlines()]
            self.assertEqual([r['event_id'] for r in data],[r['event_id'] for r in receipts])

    def test_event_overflow_is_explicit(self):
        with tempfile.TemporaryDirectory() as folder:
            errors=[];writer=EventWriter(folder,errors,capacity=1,test_delay=.3)
            writer.submit(dict(event_id=1));until=time.monotonic()+2
            while not writer.queue.empty() and time.monotonic()<until:time.sleep(.005)
            writer.submit(dict(event_id=2))
            with self.assertRaisesRegex(RuntimeError,'overflow'):writer.submit(dict(event_id=3))
            writer.finish(time.monotonic()+3)

    def test_image_byte_overflow_is_explicit(self):
        with tempfile.TemporaryDirectory() as folder:
            errors=[];writer=ImageWriter(folder,errors,lambda _:None,max_bytes=1)
            with self.assertRaisesRegex(RuntimeError,'overflow'):writer.submit(image(),{})
            writer.finish(time.monotonic()+10)

    def test_image_item_overflow_is_explicit(self):
        with tempfile.TemporaryDirectory() as folder:
            errors=[];writer=ImageWriter(folder,errors,lambda _:None,max_items=1)
            writer.submit(image(test_delay=.3),dict(wall=1))
            with self.assertRaisesRegex(RuntimeError,'overflow'):writer.submit(image(),{})
            writer.finish(time.monotonic()+10)

    def test_encoding_error_marks_archive_incomplete(self):
        with tempfile.TemporaryDirectory() as folder:
            errors=[];writer=ImageWriter(folder,errors,lambda _:None)
            job=image();job['encoding']='unsupported';writer.submit(job,dict(wall=1))
            status=writer.finish(time.monotonic()+10)
            self.assertFalse(status['drained']);self.assertTrue(errors);self.assertEqual(status['pending'],1)

    def test_worker_exit_is_not_success(self):
        with tempfile.TemporaryDirectory() as folder:
            errors=[];writer=ImageWriter(folder,errors,lambda _:None)
            writer.process.terminate();writer.process.join(2)
            status=writer.finish(time.monotonic()+3)
            self.assertFalse(status['drained']);self.assertTrue(errors)

    def test_missing_or_empty_image_is_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(RuntimeError):require_image_file(folder,'missing.png')
            (Path(folder)/'empty.png').touch()
            with self.assertRaises(RuntimeError):require_image_file(folder,'empty.png')

    def test_invalid_json_write_is_reported_as_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            errors=[];writer=EventWriter(folder,errors)
            writer.submit(dict(event_id=1,invalid=float('nan')))
            writer.finish(time.monotonic()+2)
            self.assertTrue(errors)

    def test_drain_deadline_cannot_be_reported_as_success(self):
        with tempfile.TemporaryDirectory() as folder:
            errors=[];writer=EventWriter(folder,errors,test_delay=.3)
            writer.submit(dict(event_id=1))
            status=writer.finish(time.monotonic()+.02)
            self.assertFalse(status['drained']);self.assertTrue(errors)
            writer.thread.join(2)

    def test_coverage_checks_both_tail_and_internal_gap(self):
        self.assertTrue(coverage([0,.1,.2],0,.2)['passed'])
        self.assertFalse(coverage([0,.1],0,.3)['passed'])
        self.assertFalse(coverage([0,.2],0,.2)['passed'])

    def test_capture_signature_cannot_mix_profiles(self):
        signature=dict(model_sha256='m',plugin_sha256='p',observer_sha256='o',source_sha256={'a':'s'},capture_config_sha256=capture_hash('async'))
        rows=[dict(step=step,passed=True,signature=signature) for step in (.001,.0005)]
        with tempfile.TemporaryDirectory() as folder:
            (Path(folder)/'calibration-ledger.json').write_text(json.dumps(rows))
            self.assertEqual(validate_calibration(folder,signature),2)
            with self.assertRaises(CampaignStop):validate_calibration(folder,dict(signature,capture_config_sha256=capture_hash('legacy')))

    def test_input_calibration_does_not_claim_vehicle_geometry_acceptance(self):
        rows=[]
        for repeat in (0,1):
            for step in (.001,.0005):
                window=dict(passed=True,mode='paused_steps',step=step,duration_s=20,
                    wheel_budget=dict(passed=True,linear_integral_m=0,angular_integral_rad=4),
                    endpoint=dict(xyz=[.03 if step==.0005 else 0,0,0],rpy=[0,0,0]))
                rows.append(dict(signature=dict(model='fixed'),result=dict(passed=True,formal=dict(command_window=window))))
        pairs=qualify_input_pairs(rows)
        self.assertTrue(all(p['valid'] for p in pairs));self.assertTrue(all(not p['passed'] for p in pairs))
        with self.assertRaises(CampaignStop):qualify_input_pairs(rows[:3])
        rows[-1]['signature']['model']='changed'
        with self.assertRaises(CampaignStop):qualify_input_pairs(rows)

if __name__=='__main__':unittest.main()
