import argparse
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from r5_image_delivery import bridge_stages,writer_complete,choose_candidate,delivery_arguments
from r5_capture_io import capture_hash,image_options,image_argv,image_receive_info
from r5_rtps_headers import rtps_messages
from run_r5_image_delivery_campaign import stage_allowed,CampaignStop

class DeliveryTests(unittest.TestCase):
    def test_default_compatibility(self):
        p=argparse.ArgumentParser();delivery_arguments(p);args=p.parse_args([])
        self.assertEqual(image_options(args),dict(image_trace='off',image_sampling='legacy',image_observer_sha256=None))
        self.assertEqual(capture_hash('async',**image_options(args)),'0429e34e9d0afa513ac7af279f6316c7613f9a06dee0a135b13fe53b6583312f')
    def test_profiles_transmitted_and_signed(self):
        args=SimpleNamespace(image_bridge='traced',image_delivery_profile='legacy')
        self.assertIn('--image-bridge',image_argv(args))
        first=capture_hash('async',**image_options(args));args.image_delivery_profile='udp_rx4m'
        self.assertNotEqual(first,capture_hash('async',**image_options(args)))
    def test_bridge_stage_missing_duplicate_or_exception_fails(self):
        rows=[dict(stream='rgb',stamp_ns=100,stage=s) for s in ('entry','converted','publish_begin','publish_return')]
        self.assertTrue(bridge_stages(rows)['passed'])
        for bad in (rows[:-1],rows+[rows[-1]],rows+[dict(stream='rgb',stamp_ns=100,stage='exception',exception=True)]):
            self.assertFalse(bridge_stages(bad)['passed'])
    def test_writer_overflow_exit_and_drain_incomplete_fail(self):
        status=dict(received=2,written=2,drained=True,failed=False)
        self.assertTrue(writer_complete([1,2],status))
        for change in (dict(drained=False),dict(failed=True),dict(received=3),dict(written=1)):
            self.assertFalse(writer_complete([1,2],dict(status,**change)))
    def test_udp_needs_payload_and_overflow(self):
        self.assertIsNone(choose_candidate(dict(payload_udp_confirmed=True)))
        self.assertIsNone(choose_candidate(dict(overflow_correlated=True)))
        self.assertEqual(choose_candidate(dict(payload_udp_confirmed=True,overflow_correlated=True)),'udp_rx4m')
    def test_process_isolation_requires_backlog_correlation(self):
        self.assertIsNone(choose_candidate(dict(cpp_continuous=True)))
        self.assertEqual(choose_candidate(dict(cpp_continuous=True,python_backlog_correlated=True)),'isolated_rx')
        self.assertIsNone(choose_candidate(dict(cpp_continuous=True,python_backlog_correlated=True,contradictory=True)))
    def test_missing_sequence_is_null(self):
        row=image_receive_info(dict(source_timestamp=100,received_timestamp=0))
        self.assertIsNone(row['publication_sequence_number']);self.assertIsNone(row['publisher_gid'])
    def test_unverified_xml_or_isolation_cannot_silently_enable_repair(self):
        from check_r5_runtime import run
        for profile in ('udp_rx4m','isolated_rx'):
            with self.assertRaisesRegex(RuntimeError,'no silent fallback'):
                run(SimpleNamespace(image_bridge='stock',image_delivery_profile=profile,dds_config='present-but-not-effective.xml'))
    def test_missing_observation_or_completion_file_fails(self):
        import tempfile
        from r5_image_delivery import analyze_delivery
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(FileNotFoundError):analyze_delivery(folder,dict(sim_started=0,sim_ended=1))
    def test_stock_unavailable_bridge_count_is_not_zero(self):
        import tempfile
        from r5_image_delivery import analyze_delivery
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder);rows=[dict(kind=k,stamp_ns=0,publication_sequence=None) for k in ('rgb','depth')]
            (path/'cpp-images.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            (path/'cpp-images.jsonl.status.json').write_text(json.dumps(dict(received=2,written=2,drained=True,failed=False)))
            (path/'delivery-transport-status.json').write_text(json.dumps(dict(drained=True,errors=[])))
            for name in ('native-images.jsonl','ros-probe.jsonl','observations.jsonl'):(path/name).write_text('')
            value=analyze_delivery(folder,dict(sim_started=0,sim_ended=1))
            self.assertIsNone(value['images']['rgb']['counts']['bridge_return'])
            self.assertIsNone(value['images']['rgb']['max_gap_s']['bridge_return'])
    def test_no_calibration_with_incomplete_baselines(self):
        with self.assertRaises(CampaignStop):stage_allowed([], 'calibrate','legacy')
        with self.assertRaises(CampaignStop):stage_allowed([dict(name='01-stock-1',stage='diagnostic')],'repair','udp_rx4m')
    def test_unreproduced_baselines_allowed_only_without_repair(self):
        entries=[dict(name=f'{i}-{b}-1',stage='diagnostic',passed=True) for i,b in enumerate(('stock','stock','traced','traced'))]
        stage_allowed(entries,'calibrate','legacy')
        with self.assertRaises(CampaignStop):stage_allowed(entries,'calibrate','udp_rx4m')
    def test_rtps_discovery_does_not_identify_image_guid(self):
        payload=b'RTPS'+bytes([2,3,1,15])+bytes(range(12))
        body=bytes(8)+bytes.fromhex('000003c2')+struct.pack('<iI',0,17)
        records=rtps_messages(payload+bytes([0x15,1])+struct.pack('<H',len(body))+body)
        self.assertEqual(records[0]['sequence'],17)
        self.assertEqual(records[0]['writer_guid'],bytes(range(12)).hex()+'000003c2')
        self.assertEqual(rtps_messages(b'incomplete'),[])

if __name__=='__main__':unittest.main()
