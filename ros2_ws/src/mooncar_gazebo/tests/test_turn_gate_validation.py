"""Private routing preserves the fixed matrix and the primary startup error."""
import importlib,json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[4];sys.path.insert(0,str(ROOT/'scripts'))
import check_r5_turn_gate_validation as adapter
import run_r5_turn_gate_validation as campaign
from r5_test_startup_guard import generate

class EntryTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.p=Path(self.temp.name)
        self.guard=self.p/'test-guard.py';generate(ROOT/'ros2_ws/src/mooncar_gazebo/launch/image_dds_guard.py',self.guard)
        self.patch=patch.object(adapter,'TEST_GUARD',str(self.guard));self.patch.start();self.addCleanup(self.patch.stop)
    def test_fixed24_matrix_and_pairing_unchanged(self):
        self.assertEqual(campaign.MATRIX,campaign.base.MATRIX);self.assertEqual(len(campaign.MATRIX),24)
        for i in range(2,6):self.assertEqual(campaign.MATRIX[i][3:],campaign.MATRIX[i+4][3:])
        self.assertEqual(campaign.MATRIX[10][3],'zero');self.assertEqual(campaign.MATRIX[-1][0],'restore')
    def test_only_the_private_bridge_guard_is_redirected(self):
        original=['python3',adapter.GUARD,'--root','fixed']
        self.assertEqual(adapter.redirect(original)[1],str(self.guard));self.assertEqual(original[1],adapter.GUARD)
        other=['ros2','run','mooncar_control','safe_node'];self.assertEqual(adapter.redirect(other),other)
    def test_signature_mismatch_fails(self):
        self.guard.write_bytes(self.guard.read_bytes()+b'\n')
        with self.assertRaises(RuntimeError):adapter.redirect(['python3',adapter.GUARD])
    def test_startup_root_error_is_retained(self):
        folder=self.p/'round';folder.mkdir()
        record=dict(passed=False,error='Runtime exited',schedule_phases=[],turn_slip_error='Missing grid')
        (folder/'result.json').write_text(json.dumps(record))
        original=importlib.import_module('check_r5_turn_slip_round')
        with patch.object(original,'main',return_value=1),patch.object(sys,'argv',['test','--output',str(self.p),'--name','round']):
            self.assertEqual(adapter.main(),1)
        result=json.loads((folder/'result.json').read_text())
        self.assertEqual(result['secondary_collection_error'],'Missing grid')
        self.assertIn('Runtime exited',result['turn_slip_error']);self.assertFalse(result['passed'])

if __name__=='__main__':unittest.main()
