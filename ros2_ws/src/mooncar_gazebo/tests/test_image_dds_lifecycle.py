"""Historical mapping attribution must be evidenced, never guessed."""
import json, os, queue, sys, tempfile, threading, unittest
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[4];sys.path.insert(0,str(ROOT/'scripts'))
import image_dds_ownership as ownership
from image_dds_validation import active_receiver_gate

class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.path=Path(self.tmp.name)/'absent'
        self.mapping=dict(path=str(self.path),device=3,inode=55,deleted=True)
        self.segment=dict(path=str(self.path),device=3,inode=55,owners=[dict(pid=123,start_ticks=10)])
        self.history=ownership.Lifecycle.__new__(ownership.Lifecycle)
        self.history.lock=threading.Lock();self.history.history={(3,55):dict(event='created_owner',segment=self.segment,first_seen_wall=1.)}
        self.history.queue=queue.Queue(8192);self.history.errors=[];self.history.stop=threading.Event()
    def tearDown(self):self.tmp.cleanup()
    def retired(self,mapping=None):
        with patch.object(ownership,'identity',side_effect=FileNotFoundError):return self.history.retired(mapping or self.mapping)
    def test_exact_deleted_mapping_and_owner_exit(self):
        proof=self.retired();self.assertEqual(proof['event'],'retired_remote_mapping');self.assertFalse(proof['participant_active'])
        self.assertEqual(self.history.queue.get_nowait()['segment']['owners'][0]['pid'],123)
    def test_unknown_inode_is_failure(self):self.assertIsNone(self.retired(dict(self.mapping,inode=56)))
    def test_same_name_different_device_is_failure(self):self.assertIsNone(self.retired(dict(self.mapping,device=4)))
    def test_different_name_same_inode_is_failure(self):self.assertIsNone(self.retired(dict(self.mapping,path=str(self.path)+'other')))
    def test_live_mapping_cannot_use_retirement(self):self.assertIsNone(self.retired(dict(self.mapping,deleted=False)))
    def test_creator_still_alive_cannot_be_retired(self):
        with patch.object(ownership,'identity',return_value=dict(pid=123,start_ticks=10)):
            self.assertIsNone(self.history.retired(self.mapping))
    def test_reused_pid_does_not_become_original_creator(self):
        with patch.object(ownership,'identity',return_value=dict(pid=123,start_ticks=11)):
            self.assertIsNotNone(self.history.retired(self.mapping))
    def test_recreated_path_cannot_use_old_history(self):
        self.path.write_bytes(b'new');self.assertIsNone(self.retired())
    def test_map_parser_keeps_inode_and_deleted_status(self):
        row=ownership.mapped_files('100-200 rw-s 0 00:93 55 /dev/shm/fastrtps_test (deleted)')['/dev/shm/fastrtps_test']
        self.assertEqual(row['device'],os.makedev(0,0x93));self.assertEqual(row['inode'],55);self.assertTrue(row['deleted'])
    def test_metadata_queue_overflow_fails(self):
        self.history.queue=queue.Queue(1);self.history.emit({'one':1});self.history.emit({'two':2})
        self.assertTrue(self.history.errors);self.assertTrue(self.history.stop.is_set())
    def test_writer_failure_fails(self):
        self.history.path=Path(self.tmp.name);self.history.write();self.assertTrue(self.history.errors)
    def test_real_transient_participant_reproduction(self):
        p=Path(os.environ.get('R5_DDS_PREFLIGHT_ROOT','/evidence'))/'lifecycle-preflight/result.json'
        if not p.exists():self.skipTest('Current container evidence required')
        result=json.loads(p.read_text());self.assertTrue(result['passed']);self.assertTrue(result['stale_mapping_reproduced'])
        self.assertFalse(result['without_history']['verified']);self.assertTrue(result['after_exit']['verified'])
    def test_unknown_mapping_without_history_still_fails(self):
        self.history.history.clear();self.assertIsNone(self.retired())
    def active_proofs(self):
        first={'main':dict(pid=1,start_ticks=100,verified=True),'capture-probe':dict(pid=2,start_ticks=200,verified=True),'all_participants':{'passed':True}}
        final=dict(passed=True,processes={'1':dict(pid=1,start_ticks=100,verified=True),'2':dict(pid=2,start_ticks=200,verified=True)})
        return first,final
    def test_active_gate_requires_both_actual_snapshots(self):
        first,final=self.active_proofs();self.assertTrue(active_receiver_gate(first,final))
        final['passed']=False;self.assertFalse(active_receiver_gate(first,final))
    def test_bad_initial_mapping_still_rejects(self):
        first,final=self.active_proofs();first['main']['verified']=False;self.assertFalse(active_receiver_gate(first,final))
    def test_active_receiver_pid_reuse_rejects(self):
        first,final=self.active_proofs();final['processes']['1']['start_ticks']=101;self.assertFalse(active_receiver_gate(first,final))
    def test_missing_receiver_before_shutdown_rejects(self):
        first,final=self.active_proofs();del final['processes']['2'];self.assertFalse(active_receiver_gate(first,final))
    def test_real_teardown_proof_is_taken_while_peers_alive(self):
        p=Path(os.environ.get('R5_DDS_PREFLIGHT_ROOT','/evidence'))/'teardown-preflight/result.json'
        if not p.exists():self.skipTest('Current container evidence required')
        result=json.loads(p.read_text());self.assertTrue(result['passed']);self.assertTrue(result['active_gate'])
        self.assertTrue(active_receiver_gate(result['initial'],result['before_peer_shutdown']))
        self.assertTrue(result['post_shutdown_gate_not_used'])

if __name__=='__main__':unittest.main()
