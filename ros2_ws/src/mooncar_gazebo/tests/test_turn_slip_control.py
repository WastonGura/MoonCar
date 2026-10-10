import importlib.util,json,sys,tempfile,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[4];sys.path.insert(0,str(ROOT/'scripts'))
from r5_turn_slip_control import generate,qualify,pure_turn,WHEELS

class TurnSlipTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.p=Path(self.temp.name)
        self.params={w:dict(wheel_radius=.076,wheel_normal_force=4.,slip_compliance_lateral=10.) for w in WHEELS}
        self.result=dict(sim_started=.001,sim_ended=.003,schedule=dict(stop=dict(after=dict(iteration=3,sim=.003))))
    def fixture(self,mode='pure_turn'):
        targets=np.array([[0,0,0,0],[-.96,.96,-1.04,1.04],[1,1,1,1]])
        phy_fields=['iteration','t','dt']+[w+s for w in WHEELS for s in ('_wheel_joint_velocity_cmd','_spin_before','_slip_present','_slip1_cmd','_slip2_cmd')]
        gate_fields=['iteration','t','dt','pure_turn','control_mode']+[w+s for w in WHEELS for s in ('_target','_present','_before1','_before2','_after1','_after2')]
        self.phy=np.zeros(3,dtype=[(k,float) for k in phy_fields]);self.gate=np.zeros(3,dtype=[(k,float) for k in gate_fields])
        pure=pure_turn(targets)
        for data in (self.phy,self.gate):data['iteration']=[1,2,3];data['t']=[.001,.002,.003];data['dt']=.001
        self.gate['pure_turn']=pure;self.gate['control_mode']=mode=='pure_turn'
        for i,w in enumerate(WHEELS):
            native=abs(targets[:,i])*.076/4.*10.;expected=native.copy()
            if mode=='pure_turn':expected[~pure]=0
            self.phy[w+'_wheel_joint_velocity_cmd']=targets[:,i];self.phy[w+'_spin_before']=targets[:,i]
            self.phy[w+'_slip_present']=1;self.phy[w+'_slip1_cmd']=expected
            self.gate[w+'_target']=targets[:,i];self.gate[w+'_present']=1;self.gate[w+'_before1']=native;self.gate[w+'_after1']=expected
    def check(self,mode='pure_turn'):
        path=self.p/'gate.csv';np.savetxt(path,np.column_stack([self.gate[k] for k in self.gate.dtype.names]),delimiter=',',header=','.join(self.gate.dtype.names),comments='')
        Path(str(path)+'.paused').write_text('3,.003\n')
        return qualify(self.phy,self.result,path,mode,self.params)
    def test_pure_turn_predicate_for_zero_straight_reverse_mixed_and_both_turns(self):
        self.assertEqual(pure_turn([[0,0,0,0],[1,1,1,1],[-1,-1,-1,-1],[-1,1,-1,1],[1,-1,1,-1],[.5,1.5,.5,1.5]]).tolist(),[False,False,False,True,True,False])
        with self.assertRaises(ValueError):pure_turn([[0,0,0,float('nan')]])
    def test_actual_before_after_and_observer_match_both_modes(self):
        for mode in ('always','pure_turn'):
            self.fixture(mode);self.assertTrue(self.check(mode)['passed'])
    def test_forged_contact_echo_does_not_replace_actual_observer(self):
        self.fixture();self.phy['front_left_slip1_cmd'][-1]=.19
        with self.assertRaises(ValueError):self.check()
    def test_native_formula_unknown_and_longitudinal_change_rejected(self):
        self.fixture();self.gate['front_left_before1'][1]=0
        with self.assertRaises(ValueError):self.check()
        self.fixture();self.gate['front_left_present'][1]=0
        with self.assertRaises(ValueError):self.check()
        self.fixture();self.gate['front_left_after2'][1]=.1
        with self.assertRaises(ValueError):self.check()
    def test_wrong_mode_or_wheel_target_rejected(self):
        self.fixture();self.gate['control_mode'][1]=0
        with self.assertRaises(ValueError):self.check()
        self.fixture();self.gate['front_left_target'][1]=-.5
        with self.assertRaises(ValueError):self.check()
    def test_duplicate_grid_rejected(self):
        self.fixture();self.gate['iteration'][2]=2
        with self.assertRaises(ValueError):self.check()
    def test_single_mode_model_change_and_restore_hash(self):
        source=ROOT/'reports/evidence/terrain-v2/r5-axle-allocation-20261010/allocator-models/candidate.sdf'
        manifest=generate(source,self.p/'models')
        self.assertEqual(manifest['models']['always'],manifest['models']['restore'])
        self.assertNotEqual(manifest['models']['always'],manifest['models']['pure_turn'])
    def test_entry_matrix_and_corresponding_pairs(self):
        spec=importlib.util.spec_from_file_location('turn_entry',ROOT/'scripts/run_r5_turn_slip.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        self.assertEqual(len(module.MATRIX),24)
        for i in range(2,6):
            self.assertEqual(module.MATRIX[i][3:],module.MATRIX[i+4][3:]);self.assertEqual(module.MATRIX[i][2],.001);self.assertEqual(module.MATRIX[i+4][2],.0005)

if __name__=='__main__':unittest.main()
