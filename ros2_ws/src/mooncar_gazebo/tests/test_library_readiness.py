"""No Gazebo: startup identity and completeness regression."""
import json,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from r5_library_readiness import Readiness
class ReceiptTests(unittest.TestCase):
    def test_incomplete_then_complete_preserves_both(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'samples';r=Readiness(p)
            self.assertIsNone(r.observe(dict(pid=7,passed=False,libraries={'contact':'hash'})))
            accepted=r.observe(dict(pid=7,passed=True,libraries={'contact':'hash','observer':'hash'}))
            self.assertTrue(accepted['passed']);r.close()
            rows=[json.loads(x) for x in p.read_text().splitlines()]
            self.assertFalse(rows[0]['receipt']['passed']);self.assertTrue(rows[1]['receipt']['passed'])
    def test_missing_stays_missing(self):
        with tempfile.TemporaryDirectory() as d:
            r=Readiness(Path(d)/'samples');self.assertIsNone(r.observe(None));self.assertIsNone(r.accepted);r.close()
    def test_overflow_and_changed_pid_fail(self):
        with tempfile.TemporaryDirectory() as d:
            r=Readiness(Path(d)/'samples',limit=1);r.observe(dict(pid=7,passed=False))
            with self.assertRaises(RuntimeError):r.observe(dict(pid=7,passed=True))
            r.close();r=Readiness(Path(d)/'samples2');r.observe(dict(pid=7,passed=False))
            with self.assertRaises(RuntimeError):r.observe(dict(pid=8,passed=True))
            r.close()
    def test_bad_or_missing_pass_is_not_ready(self):
        with tempfile.TemporaryDirectory() as d:
            r=Readiness(Path(d)/'samples')
            for flag in (False,None,'true',1):self.assertIsNone(r.observe(dict(pid=7,passed=flag)))
            r.close()
if __name__=='__main__':unittest.main()
