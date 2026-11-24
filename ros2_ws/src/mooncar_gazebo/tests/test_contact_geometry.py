"""Evidence gates, not vehicle acceptance or a physical substitute."""
import copy,json,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from r5_contact_geometry import MATRIX,SCREEN_ERROR,qualify,metadata,mesh_receipt
from prepare_r5_contact_geometry import make_mesh
from audit_r5_ramp_contact import read_obj
import numpy as np

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
    def test_generated_normals_referenced_without_geometry_change(self):
        with tempfile.TemporaryDirectory() as d:
            faces=[]
            for mirror in (False,True):
                p=Path(d)/str(mirror);make_mesh(p,mirror,extent=.5,cell=.25)
                lines=p.read_text().splitlines()
                self.assertEqual([x for x in lines if x.startswith('vn ')],['vn 0 0 1'])
                self.assertTrue(all(x.endswith('//1') for line in lines if line.startswith('f ') for x in line.split()[1:]))
                vertices,indices=read_obj(p);self.assertEqual(len(indices),8);self.assertTrue(np.all(vertices[:,2]==.05))
                normals=np.cross(vertices[indices[:,1]]-vertices[indices[:,0]],vertices[indices[:,2]]-vertices[indices[:,0]])
                self.assertTrue(np.all(normals[:,2]>0));self.assertTrue(np.all(normals[:,:2]==0));faces.append(indices)
            self.assertFalse(np.array_equal(faces[0],faces[1]))
    def test_native_receipt_requires_both_actual_fixtures(self):
        r=dict(passed=True,new_simulations=0,meshes=[dict(path='/probe/fixtures/'+n+'.obj',passed=True,vertices=6,normals=6,indices=6) for n in ('coplanar_mesh','mirrored_mesh')])
        self.assertTrue(mesh_receipt(r))
        for key,value in (('normals',0),('vertices',0),('path','/probe/future-fixtures/copied.obj'),('indices',5),('passed',False)):
            broken=copy.deepcopy(r);broken['meshes'][0][key]=value;self.assertFalse(mesh_receipt(broken))
        r['meshes'].pop();self.assertFalse(mesh_receipt(r))

if __name__=='__main__':unittest.main()
