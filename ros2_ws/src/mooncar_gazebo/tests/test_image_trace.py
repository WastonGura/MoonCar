"""Image timestamp boundaries and read-only layered evidence gates."""
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from r5_image_trace import decision,analyze_images
from r5_capture_io import capture_hash,EventWriter,image_options
from run_r5_rgb_gap_campaign import check_attempt_budget,run,CampaignStop

class ImageTraceTests(unittest.TestCase):
    def test_exact_nanosecond_boundary(self):
        self.assertFalse(decision(15_099_999_999,14_100_000_000,'nanoseconds'))
        self.assertTrue(decision(15_100_000_000,14_100_000_000,'nanoseconds'))
        self.assertTrue(decision(15_100_000_001,14_100_000_000,'nanoseconds'))

    def test_float_boundary_reproduces_without_new_sampling_strategy(self):
        self.assertFalse(decision(8_700_000_000,7_700_000_000,'legacy'))
        self.assertTrue(decision(8_700_000_000,7_700_000_000,'nanoseconds'))

    def test_initial_duplicate_and_out_of_order(self):
        self.assertTrue(decision(0,None,'nanoseconds'))
        self.assertFalse(decision(10,10,'nanoseconds'))
        self.assertFalse(decision(9,10,'nanoseconds'))
        with self.assertRaises(ValueError):decision(1,0,'invalid')

    def test_profiles_and_observer_hash_cannot_mix(self):
        legacy=capture_hash('async')
        header=capture_hash('async',image_trace='headers',image_observer_sha256='one')
        fixed=capture_hash('async',image_trace='headers',image_sampling='nanoseconds',image_observer_sha256='one')
        self.assertEqual(len({legacy,header,fixed}),3)
        self.assertNotEqual(header,capture_hash('async',image_trace='headers',image_observer_sha256='two'))
        self.assertEqual(legacy,'0429e34e9d0afa513ac7af279f6316c7613f9a06dee0a135b13fe53b6583312f')

    def test_diagnostic_calibration_and_total_limits(self):
        check_attempt_budget([dict(stage='diagnostic')]*7,'diagnostic')
        for entries,stage in (([dict(stage='diagnostic')]*8,'diagnostic'),
            ([dict(stage='calibration')]*4,'calibration'),
            ([dict(stage='diagnostic')]*8+[dict(stage='calibration')]*4,'calibration')):
            with self.assertRaises(CampaignStop):check_attempt_budget(entries,stage)

    def test_stopped_campaign_cannot_resume_or_start_calibration(self):
        with tempfile.TemporaryDirectory() as folder:
            Path(folder,'campaign-stop.json').write_text('{}')
            with self.assertRaises(CampaignStop):run(SimpleNamespace(output=folder))

    def fixture(self,folder,missing=None,archive=True,wrong=False,native_error=False):
        def write(name,rows):Path(folder,name).write_text(''.join(json.dumps(r)+'\n' for r in rows))
        native=[];probe=[];main=[]
        for kind in ('rgb','depth'):
            for ns in (0,1_000_000_000,2_000_000_000):
                native.append(dict(kind=kind,stamp_ns=ns))
                if missing!='probe':probe.append(dict(kind=kind,stamp_ns=ns))
                if missing!='main':
                    main.append(dict(kind='image_trace',image_kind=kind,stamp_ns=ns,selected=not wrong,
                        previous_selected_ns=None if ns==0 else ns-1_000_000_000,sampling_mode='nanoseconds'))
                    if archive:main.append(dict(kind=kind,stamp=ns/1e9))
        write('native-images.jsonl',native);write('ros-probe.jsonl',probe);write('observations.jsonl',main)
        for i,r in enumerate(probe):r['event_id']=i+1
        write('ros-probe.jsonl',probe);write('probe-receipts.jsonl',[dict(event_id=r['event_id']) for r in probe])
        Path(folder,'probe-status.json').write_text(json.dumps(dict(events=dict(written=len(probe),drained=True),errors=[])))
        Path(folder,'native-images-status.json').write_text(json.dumps(dict(received=len(native),written=len(native),drained=not native_error,errors=['write failed'] if native_error else [])))
        return analyze_images(folder,dict(sim_started=0,sim_ended=2,sensor_counts=dict(rgb=3,depth=3)))

    def test_complete_layers_preserve_source_gap_as_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            value=self.fixture(folder)
            self.assertTrue(value['passed']);self.assertEqual(value['images']['rgb']['native_max_gap_s'],1.)

    def test_missing_probe_main_and_archive_fail(self):
        for missing,archive in (('probe',True),('main',True),(None,False)):
            with self.subTest(missing=missing,archive=archive),tempfile.TemporaryDirectory() as folder:
                self.assertFalse(self.fixture(folder,missing,archive)['passed'])

    def test_wrong_decision_and_native_observer_failure_fail(self):
        for changes in (dict(wrong=True),dict(native_error=True)):
            with tempfile.TemporaryDirectory() as folder:self.assertFalse(self.fixture(folder,**changes)['passed'])

    def test_header_writer_is_bounded_and_drains_with_receipts(self):
        with tempfile.TemporaryDirectory() as folder:
            errors=[];writer=EventWriter(folder,errors,capacity=2,test_delay=.02,filename='headers.jsonl',receipt_filename='receipts.jsonl')
            writer.submit(dict(event_id=1,stamp_ns=17_800_000_000));status=writer.finish(time.monotonic()+2)
            self.assertTrue(status['drained']);self.assertFalse(errors)
            rows=[json.loads(x) for x in Path(folder,'headers.jsonl').read_text().splitlines()]
            self.assertEqual(rows[0]['stamp_ns'],17_800_000_000)
            self.assertEqual(json.loads(Path(folder,'receipts.jsonl').read_text())['event_id'],1)

if __name__=='__main__':unittest.main()
