"""Acceptance-critical pause continuity, lock ownership and completion checks."""
import concurrent.futures, json, os, subprocess, sys, tempfile, time, unittest
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[4];sys.path.insert(0,str(ROOT/'scripts'))
import image_dds_ownership as ownership
from image_dds_pause import QueryWorker,wait_query,actual_pause
from image_dds_validation import paused_state,pause_flush,reception_tail

class BoundaryTests(unittest.TestCase):
    def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
    def tearDown(self):self.tmp.cleanup()
    def test_query_continues_pump(self):
        worker=QueryWorker();calls=[]
        try:
            wait_query(worker,lambda:time.sleep(.07),lambda:(calls.append(time.monotonic()),time.sleep(.002)),time.monotonic()+1)
            self.assertGreater(len(calls),5)
        finally:worker.close()
    def test_only_one_query(self):
        worker=QueryWorker()
        try:
            worker.submit(time.sleep,.05)
            with self.assertRaises(RuntimeError):worker.submit(time.sleep,.01)
        finally:worker.close()
    def test_timeout_cancels_child(self):
        worker=QueryWorker();until=time.monotonic()+.08
        with self.assertRaises(TimeoutError):wait_query(worker,worker.command,lambda:time.sleep(.002),until,[sys.executable,'-c','import time;time.sleep(10)'],until)
        worker.close();self.assertIsNone(worker.child)
    def test_query_exit_propagates(self):
        worker=QueryWorker();until=time.monotonic()+2
        try:
            with self.assertRaises(RuntimeError):wait_query(worker,worker.command,lambda:time.sleep(.002),until,[sys.executable,'-c','raise SystemExit(3)'],until)
        finally:worker.close()
    def test_stale_pause_rejected_without_second_request(self):
        worker=QueryWorker();values=iter(['data: true',json.dumps(dict(paused=True,iterations='1',simTime={'sec':'1'})),
            json.dumps(dict(paused=True,iterations='2000',simTime={'sec':'2'}))]);calls=[]
        def command(argv,deadline):calls.append(argv);return next(values)
        worker.command=command
        try:
            result=actual_pause(worker,'test',lambda:time.sleep(.001),time.monotonic()+1,paused_state,json.loads,2.)
            self.assertEqual(result['state']['iteration'],2000)
            self.assertEqual(sum('service' in argv for argv in calls),1)
        finally:worker.close()
    def test_flush_requires_csv_and_marker_same_last_step(self):
        path=self.root/'steps.csv';path.write_text('iteration,t,x\n10,0.01,0\n');Path(str(path)+'.paused').write_text('10,0.01')
        self.assertTrue(pause_flush(path,{'iteration':10,'sim':.01})['passed'])
        Path(str(path)+'.paused').write_text('9,0.009')
        self.assertFalse(pause_flush(path,{'iteration':10,'sim':.01})['passed'])
        Path(str(path)+'.paused').write_text('10,0.01');path.write_text('iteration,t,x\n9,0.009,0\n')
        self.assertFalse(pause_flush(path,{'iteration':10,'sim':.01})['passed'])
    def test_missing_csv_fails(self):self.assertFalse(pause_flush(self.root/'missing',{'iteration':1,'sim':.001})['passed'])
    def fake_process(self,paths):
        paths=list(paths)+['/opt/ros/jazzy/lib/librmw_fastrtps_cpp.so']
        p=self.root/'123';p.mkdir();(p/'maps').write_text('\n'.join('a-b rw-s 0 00:00 1 '+s for s in paths))
        (p/'environ').write_bytes(b'');(p/'cmdline').write_bytes(b'python\0recorder\0')
        return {'pid':123,'start_ticks':10}
    def segments(self,owner=123,payload=524288,remote=False,resolved=True):
        rows=[dict(path='/dev/shm/fastrtps_own',payload_bytes=payload,ownership_resolved=resolved,owners=[dict(pid=owner,start_ticks=10)])]
        if remote:rows.append(dict(path='/dev/shm/fastrtps_remote',payload_bytes=8388608,ownership_resolved=True,owners=[dict(pid=456,start_ticks=20)]))
        return {'segments':rows}
    def classify(self,data,role='ros-node',require=True):
        with patch.object(ownership,'identity',return_value={'pid':123,'start_ticks':10}):
            return ownership.classify(123,role,data,proc=self.root,require_participant=require)
    def test_remote_8m_does_not_change_own_capacity(self):
        self.fake_process(['/dev/shm/fastrtps_own','/dev/shm/fastrtps_remote'])
        row=self.classify(self.segments(remote=True));self.assertTrue(row['verified']);self.assertEqual(len(row['remote_mappings']),1)
    def test_missing_lock_owner_fails(self):
        self.fake_process(['/dev/shm/fastrtps_own']);self.assertFalse(self.classify(self.segments(resolved=False))['verified'])
    def test_inherited_descriptor_is_not_ownership(self):
        self.fake_process(['/dev/shm/fastrtps_own']);row=self.classify(self.segments(owner=456))
        self.assertFalse(row['verified']);self.assertFalse(row['participant']);self.assertEqual(len(row['remote_mappings']),1)
    def test_wrong_own_capacity_fails(self):
        self.fake_process(['/dev/shm/fastrtps_own']);self.assertFalse(self.classify(self.segments(payload=8388608))['verified'])
    def test_archive_library_without_participant(self):
        self.fake_process(['/opt/ros/jazzy/lib/librmw_fastrtps_cpp.so'])
        row=self.classify({'segments':[]},'archive-worker',False)
        self.assertTrue(row['verified']);self.assertFalse(row['participant'])
    def test_archive_unexpected_participant_fails(self):
        self.fake_process(['/dev/shm/fastrtps_own']);self.assertFalse(self.classify(self.segments(),'archive-worker',False)['verified'])
    def test_removed_mapping_is_not_ignored(self):
        self.fake_process(['/dev/shm/fastrtps_gone (deleted)']);self.assertFalse(self.classify({'segments':[]})['verified'])
    def test_pid_exit_not_zero_capacity(self):
        with self.assertRaises(FileNotFoundError):ownership.classify(999,'ros-node',{'segments':[]},proc=self.root)
    def test_pid_reuse_fails(self):
        self.fake_process(['/dev/shm/fastrtps_own'])
        with patch.object(ownership,'identity',side_effect=[{'pid':123,'start_ticks':10},{'pid':123,'start_ticks':11}]):
            self.assertFalse(ownership.classify(123,'ros-node',self.segments(),proc=self.root)['verified'])
    def test_lock_parser_retains_holder_pid_and_inode(self):
        row=ownership.lock_rows('lock:\t1: FLOCK ADVISORY WRITE 55 00:93:123 0 EOF')[0]
        self.assertEqual(row,dict(pid=55,device=os.makedev(0,0x93),inode=123))
    def test_lock_inode_and_owner_must_both_match(self):
        locks={(10,3,4):{'pid':10},(20,3,4):{'pid':20}}
        processes={10:dict(start_ticks=1,descriptors=[dict(device=3,inode=4)]),20:dict(start_ticks=2,descriptors=[dict(device=3,inode=4)])}
        self.assertEqual(len(ownership.lock_owners(locks,processes,3,4)),2) # Ambiguous: inventory cannot resolve ownership.
        self.assertEqual(ownership.lock_owners(locks,processes,3,5),[])
        self.assertEqual(ownership.lock_owners(locks,{},3,4),[]) # Holder exited.
    def test_inherited_fd_does_not_replace_holder(self):
        locks={(10,3,4):{'pid':10}}
        processes={20:dict(start_ticks=2,descriptors=[dict(device=3,inode=4)])}
        self.assertEqual(ownership.lock_owners(locks,processes,3,4),[])
    def test_real_ros_preflight(self):
        p=Path('/evidence/boundary-preflight-final/result.json')
        if not p.exists():self.skipTest('Container preflight only')
        result=json.loads(p.read_text());self.assertTrue(result['passed']);self.assertTrue(result['real_ros']['remote_bridge_mapping'])
        self.assertTrue(all(r['passed'] for r in result['delayed_queries']))

if __name__=='__main__':unittest.main()
