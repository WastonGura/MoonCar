"""Monitoring cannot create container processes or turn pending data into proof."""
import copy,json,os,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[4];sys.path.insert(0,str(ROOT/'scripts'))
import image_dds_ownership as ownership
from r5_monitoring import monitoring_profile,validate_monitoring
from r5_host_progress import snapshot,read_shared

class ProgressTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
    def write(self,stage,value):
        path=self.root/stage/('campaign.json' if stage=='campaign' else 'ledger.json')
        path.parent.mkdir(exist_ok=True);path.write_text(json.dumps(value));return path
    def formal(self):return dict(status='running',maximum_attempts=17,attempts=[dict(name='curve-left',started_wall=100,
        qualification=dict(diagnostic_qualified=True)),dict(name='curve-right',started_wall=110)],monitoring_profile=monitoring_profile())
    def test_no_ledger_is_pending(self):
        result=snapshot(self.root,120);self.assertEqual(result['status'],'pending');self.assertNotIn('qualified',result)
    def test_reads_actual_counts_and_elapsed(self):
        self.write('campaign',self.formal());result=snapshot(self.root,125)
        self.assertEqual((result['started'],result['qualified'],result['elapsed_s']),(2,1,15))
        self.assertEqual(result['purpose'],'progress_only');self.assertEqual(result['current'],'curve-right')
    def test_stopped_legacy_batch_remains_stopped(self):
        value=self.formal();value.pop('monitoring_profile');value['status']='stopped';value['reason']='permission'
        self.write('campaign',value);result=snapshot(self.root,125)
        self.assertEqual(result['status'],'stopped');self.assertIsNone(result['declared_monitoring_profile'])
        self.assertEqual(result['qualified'],1);self.assertEqual(result['reason'],'permission')
    def test_partial_current_ledger_does_not_fall_back_to_old_success(self):
        self.write('diagnostic',dict(status='completed',maximum_attempts=12,attempts=[]))
        path=self.write('campaign',self.formal());self.assertEqual(snapshot(self.root,125)['qualified'],1)
        path.write_text('{"status":');result=snapshot(self.root,125)
        self.assertEqual(result['stage'],'campaign');self.assertEqual(result['status'],'pending');self.assertNotIn('qualified',result)
    def test_unreadable_progress_never_qualifies(self):
        self.write('campaign',self.formal())
        with patch('r5_host_progress.read_shared',side_effect=PermissionError(13,'Denied')):
            result=snapshot(self.root,125)
        self.assertEqual(result['status'],'pending');self.assertNotIn('qualified',result)
    def test_invalid_count_time_or_row_is_pending(self):
        for field,value in [('maximum_attempts',True),('maximum_attempts',1),('attempts',[None])]:
            record=self.formal();record[field]=value;self.write('campaign',record)
            self.assertEqual(snapshot(self.root,125)['status'],'pending')
        record=self.formal();record['attempts'][-1]['started_wall']=float('nan');self.write('campaign',record)
        self.assertEqual(snapshot(self.root,125)['status'],'pending')
    def test_diagnostic_counts_require_cleanup_and_no_failure(self):
        self.write('diagnostic',dict(status='running',maximum_attempts=12,attempts=[dict(name='a',started_wall=110,cleanup_passed=True,failure=None),
            dict(name='b',started_wall=120,cleanup_passed=True,failure='bad')]))
        self.assertEqual(snapshot(self.root,125)['completed'],1)
    def test_shared_read_allows_open_writer(self):
        path=self.root/'progress.json'
        with path.open('wb') as stream:
            stream.write(b'{"tick":1}');stream.flush()
            self.assertEqual(read_shared(path),b'{"tick":1}')
    def test_monitoring_profile_is_explicit_and_not_shared_mutable_state(self):
        record=monitoring_profile();self.assertEqual(validate_monitoring(dict(monitoring_profile=record)),record)
        invalid=copy.deepcopy(record);invalid['container_exec_during_observation']=True
        for manifest in ({},dict(monitoring_profile=invalid),dict(monitoring_profile='legacy_exec')):
            with self.assertRaises(ValueError):validate_monitoring(manifest)
        record['name']='changed';self.assertEqual(monitoring_profile()['name'],'host_files_v1')

class InventoryEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        self.proc=self.root/'proc';self.process=self.proc/'17';self.fd=self.process/'fd'/'0';self.fd.parent.mkdir(parents=True)
        self.fd.write_bytes(b'fd');(self.process/'fdinfo').mkdir();self.shm=self.root/'shm';self.shm.mkdir()
        fields=['S','1']+['0']*17+['12345']
        (self.process/'stat').write_text('17 (runc:[2:INIT]) '+' '.join(fields))
        (self.process/'status').write_text('State:\tS\nUid:\t0 0 0 0\nCapEff:\tffff\n')
        (self.process/'cmdline').write_bytes(b'runc\0init\0')
    def inventory(self):return ownership.inventory(self.proc,self.shm)
    def assertEvidence(self,error,operation,path):
        self.assertEqual(error['operation'],operation);self.assertEqual(error['path'],str(path));self.assertEqual(error['errno'],13)
        self.assertEqual(error['before']['identity'],dict(pid=17,start_ticks=12345));self.assertEqual(error['before']['state'],'S')
        self.assertIn('PermissionError',error['traceback'])
    def test_fd_readlink_permission_is_still_fatal_with_full_context(self):
        with patch.object(os,'readlink',side_effect=PermissionError(13,'Denied',str(self.fd))):result=self.inventory()
        self.assertNotIn(17,result['processes']);self.assertEqual(len(result['errors']),1)
        error=result['errors'][0];self.assertEvidence(error,'readlink',self.fd)
        self.assertEqual(error['after']['commandline'],'runc init ')
        self.assertEqual(error['after']['identity'],error['before']['identity'])
    def test_exiting_process_does_not_erase_permission_failure(self):
        def denied(path):
            (self.process/'stat').unlink();raise PermissionError(13,'Denied',str(path))
        with patch.object(os,'readlink',side_effect=denied):result=self.inventory()
        self.assertEvidence(result['errors'][0],'readlink',self.fd);self.assertIn('stat_error',result['errors'][0]['after'])
    def test_fd_directory_permission_has_exact_operation(self):
        original=Path.iterdir
        def denied(path):
            if path==self.fd.parent:raise PermissionError(13,'Denied',str(path))
            return original(path)
        with patch.object(Path,'iterdir',denied):result=self.inventory()
        self.assertEvidence(result['errors'][0],'list_fds',self.fd.parent)
    def test_fdinfo_permission_is_not_mislabeled_as_readlink(self):
        target=self.process/'fdinfo'/'0';original=Path.read_text
        def denied(path,*args,**kwargs):
            if path==target:raise PermissionError(13,'Denied',str(path))
            return original(path,*args,**kwargs)
        with patch.object(os,'readlink',return_value=str(self.shm)+'/fastrtps_test_el'),patch.object(Path,'read_text',denied):result=self.inventory()
        self.assertEvidence(result['errors'][0],'read_fdinfo',target)
    def test_ordinary_disappearing_fd_retains_existing_policy(self):
        with patch.object(os,'readlink',side_effect=FileNotFoundError):result=self.inventory()
        self.assertFalse(result['errors']);self.assertEqual(result['processes'][17]['start_ticks'],12345)
    def test_state_changes_do_not_change_process_identity(self):
        before=ownership.identity(17,self.proc);text=(self.process/'stat').read_text()
        (self.process/'stat').write_text(text.replace(') S ',') R '))
        self.assertEqual(ownership.identity(17,self.proc),before)
    def test_pid_reuse_during_scan_is_still_rejected(self):
        def reused(path):
            stat=self.process/'stat';stat.write_text(stat.read_text().replace('12345','99999'))
            return '/dev/null'
        with patch.object(os,'readlink',side_effect=reused):result=self.inventory()
        self.assertNotIn(17,result['processes']);error=result['errors'][0]
        self.assertEqual(error['operation'],'read_identity_after')
        self.assertEqual(error['before']['identity']['start_ticks'],12345)
        self.assertEqual(error['after']['identity']['start_ticks'],99999)

if __name__=='__main__':unittest.main()
