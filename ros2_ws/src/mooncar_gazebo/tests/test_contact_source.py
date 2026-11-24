import copy,importlib.util,json,sys,tempfile,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[4];sys.path.insert(0,str(ROOT/'scripts'))
from r5_contact_source import contact_grid,topic_relation,MATRIX

HEADER='iteration,t_ns,dt_ns,wheel,self_collision,other_collision,state,contact_count,point_count,contact_index,point_index,extra_complete,x,y,z,nx,ny,nz,depth,f1x,f1y,f1z,f2x,f2y,f2z'

class ContactSourceTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.p=Path(self.temp.name)
        self.schedule=dict(parked=dict(before=dict(iteration=0)),stop=dict(after=dict(iteration=3,sim=.003)))
        self.physics=np.array([(i,i*.001,.001) for i in range(1,4)],dtype=[('iteration',float),('t',float),('dt',float)])
        self.rows=[]
        for i in range(1,4):
            for w in range(4):
                self.rows.append([i,i*1_000_000,1_000_000,w,10+w,99,1,1,1,0,0,1,0,0,0,0,0,1,.0001,0,0,3,0,0,-3])
    def write(self):
        path=self.p/'steps.csv';np.savetxt(path,np.array(self.rows),delimiter=',',header=HEADER,comments='')
        Path(str(path)+'.paused').write_text(f'3,3000000,{len(self.rows)},{len(self.rows)}\n');return path
    def qualify(self):return contact_grid(self.write(),self.physics,self.schedule,[10,11,12,13])
    def test_complete_force_association_and_paused_writer(self):
        proof,_=self.qualify();self.assertTrue(proof['passed']);self.assertEqual(proof['steps_per_wheel'],3)
    def test_explicit_empty_accepted_without_fabricated_zero_force(self):
        self.rows[-1][6:12]=[0,0,0,-1,-1,0];self.rows[-1][12:]=[float('nan')]*13
        proof,_=self.qualify();self.assertEqual(proof['wheels']['rear_right']['empty_steps'],1)
    def test_unknown_component_is_not_no_contact(self):
        self.rows[-1][6]=-1
        with self.assertRaises(ValueError):self.qualify()
    def test_no_backend_contact_support_fails(self):
        for row in self.rows:row[6:12]=[0,0,0,-1,-1,0];row[12:]=[float('nan')]*13
        with self.assertRaises(ValueError):self.qualify()
    def test_missing_wheel_step_and_duplicate_point_fail(self):
        original=copy.deepcopy(self.rows);self.rows.pop()
        with self.assertRaises(ValueError):self.qualify()
        self.rows=original;self.rows.append(copy.deepcopy(self.rows[-1]))
        with self.assertRaises(ValueError):self.qualify()
    def test_force_and_collision_identity_fail(self):
        self.rows[-1][-1]=3
        with self.assertRaises(ValueError):self.qualify()
        self.rows[-1][-1]=-3;self.rows[-1][4]=99
        with self.assertRaises(ValueError):self.qualify()
    def test_missing_extra_data_or_fabricated_empty_fails(self):
        self.rows[-1][11]=0
        with self.assertRaises(ValueError):self.qualify()
        self.rows[-1][6:12]=[0,0,0,-1,-1,0]
        with self.assertRaises(ValueError):self.qualify()
    def test_wrong_simulation_time_and_pause_receipt_fail(self):
        self.rows[-1][1]+=1
        with self.assertRaises(ValueError):self.qualify()
        self.rows[-1][1]-=1;path=self.write();Path(str(path)+'.paused').write_text('2,2000000,12,12')
        with self.assertRaises(ValueError):contact_grid(path,self.physics,self.schedule,[10,11,12,13])
    def test_source_no_contact_and_actual_topic_loss_are_separate(self):
        self.rows[-1][6:12]=[0,0,0,-1,-1,0];self.rows[-1][12:]=[float('nan')]*13
        _,rows=self.qualify()
        for w in ('front_left','front_right','rear_left','rear_right'):
            (self.p/(w+'-contact.jsonl')).write_text(json.dumps(dict(header=dict(stamp=dict(sec=0,nsec=1_000_000))))+'\n')
        observed=topic_relation(self.p,rows)
        self.assertEqual(observed['front_left']['positive_steps_without_topic'],2)
        self.assertEqual(observed['rear_right']['source_empty_steps'],1)
        self.assertEqual(observed['rear_right']['positive_steps_without_topic'],1)
    def test_frozen_matrix_and_entry_import(self):
        self.assertEqual(len(MATRIX),6);self.assertEqual([x[0] for x in MATRIX],['original']*3+['slip']*2+['restore'])
        spec=importlib.util.spec_from_file_location('contact_entry',ROOT/'scripts/run_r5_contact_source.py')
        spec.loader.exec_module(importlib.util.module_from_spec(spec))

if __name__=='__main__':unittest.main()
