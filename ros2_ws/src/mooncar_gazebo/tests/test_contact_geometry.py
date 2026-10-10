"""Evidence gates, not vehicle acceptance or a physical substitute."""
import copy,json,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from r5_contact_geometry import MATRIX,SCREEN_ERROR,qualify,metadata

class GeometryTests(unittest.TestCase):
    def fixture(self):
        r={k:True for k in ('passed','raw_fixture_passed','data_complete','ground_contact_gate','stop_zero','observer_flush_confirmed','receiver_verified')}
        for k in ('parked','step_observation','capture_analysis','image_trace_analysis','dds','formal','origin_qualification','image_content','lifecycle_capture'):r[k]={'passed':True}
        r.update(capture={'drained':True},max_tilt_rad=0.,stop_drift_last_second_m=0.,errors=[])
        r['turn_slip_probe']=dict(gate={'passed':True},allocation={'passed':True},source={'proof':{'passed':True}},trajectory={'maximum_displacement_m':.001})
        return r
    def test_exact_matrix(self):
        self.assertEqual(len(MATRIX),8);self.assertEqual(MATRIX[-2][0],'restore_plate')
        self.assertEqual([x[2] for x in MATRIX],[.2,-.2]*4)
    def test_complete_round(self):self.assertTrue(qualify(self.fixture(),0)['eligible'])
    def test_only_declared_screen_can_fail(self):
        r=self.fixture();r.update(passed=False,turn_slip_error=SCREEN_ERROR)
        r['turn_slip_probe']['trajectory']['maximum_displacement_m']=.02
        proof=qualify(r,1);self.assertTrue(proof['eligible']);self.assertFalse(proof['geometric_screen_passed'])
        self.assertFalse(proof['vehicle_accepted']);self.assertFalse(r['passed'])
        r['production_error']='missing sample';self.assertFalse(qualify(r,1)['eligible'])
    def test_false_or_missing_gates_fail(self):
        r=self.fixture()
        for k in ('formal','origin_qualification','capture_analysis','lifecycle_capture'):
            broken=copy.deepcopy(r);broken[k]={};self.assertFalse(qualify(broken,0)['eligible'])
        r['turn_slip_probe']['source']['proof']['passed']=False;self.assertFalse(qualify(r,0)['eligible'])
    def test_bad_exit_nan_or_tilt_fail(self):
        r=self.fixture();self.assertFalse(qualify(r,1)['eligible'])
        r['max_tilt_rad']=.11;self.assertFalse(qualify(r,0)['eligible'])
        r=self.fixture();r['turn_slip_probe']['trajectory']['maximum_displacement_m']=float('nan')
        self.assertFalse(qualify(r,0)['eligible'])
    def test_metadata_missing_identity_is_not_zero(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'meta.csv';p.write_text('iteration,t_ns,kind,entity,value\n1,1000,collision_detector,1,MISSING\n1,1000,collision,7,plate::surface::plate\n')
            r=metadata(p,[7]);self.assertIsNone(r['collision_detector'])
            with self.assertRaises(ValueError):metadata(p,[8])

if __name__=='__main__':unittest.main()
