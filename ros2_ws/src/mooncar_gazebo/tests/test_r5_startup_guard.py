"""Test-only guard preserves fail-closed proof and the original timeout."""
import contextlib,importlib.util,io,json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import Mock,patch
ROOT=Path(__file__).resolve().parents[4];sys.path.insert(0,str(ROOT/'scripts'))
from r5_test_startup_guard import generate

class GuardTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.p=Path(self.temp.name)
        self.source=ROOT/'ros2_ws/src/mooncar_gazebo/launch/image_dds_guard.py';self.target=self.p/'guard.py'
        self.manifest=generate(self.source,self.target)
        spec=importlib.util.spec_from_file_location('test_guard',self.target);self.g=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.g)
        self.child=Mock(pid=42);self.child.poll.return_value=None;self.child.wait.return_value=0
        self.folder=self.p/'runtime';self.error_log=io.StringIO()
    def run_guard(self,receipt,proof,timeout=1.):
        with patch.object(self.g.subprocess,'Popen',return_value=self.child),patch.object(self.g,'bridge_env',return_value={}), \
             patch.object(self.g,'complete_receipt',side_effect=receipt),patch.object(self.g,'process_receipt',side_effect=proof), \
             contextlib.redirect_stderr(self.error_log),contextlib.redirect_stdout(io.StringIO()):
            return self.g.supervise('bridge',[],self.p,self.folder,timeout)
    def test_partial_receipt_remains_pending(self):
        self.assertEqual(self.run_guard([None,{}],[{'pid':42}]),0)
        self.assertEqual(json.loads((self.folder/'ready.json').read_text()),{'pid':42})
    def test_receipt_enodata_retries_and_traces(self):
        self.assertEqual(self.run_guard([OSError(61,'No data available'),{}],[{'pid':42}]),0)
        self.assertIn('"errno": 61',self.error_log.getvalue());self.assertIn('traceback',self.error_log.getvalue())
    def test_runtime_proof_enodata_never_reports_premature_ready(self):
        def proof(*args):
            self.assertFalse((self.folder/'ready.json').exists())
            if not getattr(self,'retried',False):self.retried=True;raise OSError(61,'No data available')
            return {'pid':42,'segment_bytes':8388608}
        self.assertEqual(self.run_guard([{},{}],proof),0)
        self.assertEqual(json.loads((self.folder/'ready.json').read_text())['segment_bytes'],8388608)
    def test_permanent_enodata_times_out_and_closes(self):
        with self.assertRaises(TimeoutError):self.run_guard(OSError(61,'No data available'),[],.03)
        self.assertFalse((self.folder/'ready.json').exists());self.child.terminate.assert_called_once()
    def test_nonretryable_io_is_not_suppressed(self):
        with self.assertRaises(PermissionError):self.run_guard(PermissionError(13,'Denied'),[])
        self.assertFalse((self.folder/'ready.json').exists())
    def test_capacity_hash_or_version_failure_is_not_suppressed(self):
        for reason in ('capacity mismatch','hash mismatch','version mismatch'):
            with self.subTest(reason=reason):
                self.folder=self.p/reason
                with self.assertRaises(RuntimeError):self.run_guard([{}],RuntimeError(reason))
                self.assertFalse((self.folder/'ready.json').exists())
    def test_child_exit_rejects_ready(self):
        self.child.poll.return_value=1
        with self.assertRaises(RuntimeError):self.run_guard([{}],[{'pid':42}])
        self.assertFalse((self.folder/'ready.json').exists())
    def test_generation_rejects_changed_base_and_overwrite(self):
        with self.assertRaises(FileExistsError):generate(self.source,self.target)
        modified=self.p/'modified.py';modified.write_bytes(self.source.read_bytes()+b'\n')
        with self.assertRaises(ValueError):generate(modified,self.p/'other.py')
        self.assertFalse(self.manifest['production_changed']);self.assertFalse(self.manifest['timeout_changed'])

if __name__=='__main__':unittest.main()
