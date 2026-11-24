from pathlib import Path
import json,sys,tempfile,unittest
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[4];sys.path.insert(0,str(ROOT/'scripts'))
from r5_axle_allocation import factors,allocate,generate,qualify_budget
from r5_load_diagnostic import semantic
from r5_d0_analysis import WHEELS


class AxleAllocationTests(unittest.TestCase):
    def test_derived_factors_preserve_budget_and_equal_compliance(self):
        n,m=3.572514673566879,3.868145326433121;r=factors(n,m)
        self.assertAlmostEqual(r['front']+r['rear'],2)
        self.assertAlmostEqual(r['front']/n,r['rear']/m)
        self.assertAlmostEqual(r['front'],.9602682217886253)

    def test_bad_or_excessive_estimates_rejected(self):
        for a,b in ((0,1),(float('nan'),1),(1,2)):
            with self.assertRaises(ValueError):factors(a,b)

    def test_only_pure_turn_changes_actual_targets(self):
        before=np.array([[0,0,0,0],[1,1,1,1],[-1,1,-1,1],[1,-1,1,-1],[.5,1,.5,1]])
        after,active=allocate(before,dict(front=.96,rear=1.04),True)
        self.assertEqual(active.tolist(),[False,False,True,True,False])
        self.assertTrue(np.array_equal(after[[0,1,4]],before[[0,1,4]]))
        self.assertTrue(np.allclose((after[:,:2]+after[:,2:])/2,before[:,:2]))
        self.assertTrue(np.array_equal(allocate(before,dict(front=.96,rear=1.04),False)[0],before))

    def test_unknown_native_order_nonfinite_fail(self):
        for a in ([[1,-1,2,-1]],[[float('nan'),1,0,1]]):
            with self.assertRaises(ValueError):allocate(a,dict(front=.96,rear=1.04),True)

    def test_model_only_enable_flag_changes_and_restore_exact(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);(d/'slip').mkdir()
            snapshot=Path(__file__).resolve().parent/'fixtures'
            tree=ET.parse(snapshot/'r5-d0.sdf')
            parameters=json.loads((snapshot/'slip-models/manifest.json').read_text())['parameters']
            plugin=ET.SubElement(tree.find('model'),'plugin',filename='gz-sim-wheel-slip-system',name='gz::sim::systems::WheelSlip')
            for w,settings in parameters.items():
                wheel=ET.SubElement(plugin,'wheel',link_name=w+'_wheel_link')
                for key,value in settings.items():ET.SubElement(wheel,key).text=format(value,'.17g')
            # Preserve the Linux snapshot's LF bytes on Windows as well.
            (d/'slip/candidate.sdf').write_bytes(ET.tostring(tree.getroot()))
            (d/'slip/candidate.urdf').write_bytes((snapshot/'r5-d0.urdf').read_bytes())
            original=ET.parse(d/'slip/candidate.sdf');manifest=generate(d/'slip/candidate.sdf',d/'control')
            self.assertTrue(manifest['physical_and_contact_parameters_unchanged'])
            self.assertEqual((d/'control/baseline.sdf').read_bytes(),(d/'control/restore.sdf').read_bytes())
            candidate=ET.parse(d/'control/candidate.sdf');candidate.find("model/plugin[@name='mooncar::AxleAllocation']/enabled").text='false'
            self.assertEqual(semantic(candidate.getroot()),semantic(ET.parse(d/'control/baseline.sdf').getroot()))
            candidate.find('model').remove(candidate.find("model/plugin[@name='mooncar::AxleAllocation']"))
            self.assertEqual(semantic(candidate.getroot()),semantic(original.getroot()))

    def data(self):
        dtype=[('iteration',float),('t',float),('dt',float)]+[(w+'_wheel_joint_velocity_cmd',float) for w in WHEELS]
        d=np.zeros(3,dtype=dtype);d['iteration']=[1,2,3];d['t']=[.001,.002,.003];d['dt']=.001
        native=np.array([[0,0,0,0],[-.001,.001,-.001,.001],[0,0,0,0]])
        r=dict(front=.96,rear=1.04);after,applied=allocate(native,r,True)
        for i,w in enumerate(WHEELS):d[w+'_wheel_joint_velocity_cmd']=after[:,i]
        rows=np.column_stack((d['iteration'],d['t'],d['dt'],np.ones(3),applied.astype(int),native,after))
        header='iteration,t,dt,enabled,applied,'+','.join(w+s for s in ('_native','_allocated') for w in WHEELS)
        return d,r,rows,header

    def check(self,alter=None):
        d,r,rows,header=self.data()
        if alter:alter(d,rows)
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'commands.csv';np.savetxt(p,rows,delimiter=',',header=header,comments='')
            return qualify_budget(d,p,0,.2,True,r)

    def test_independent_logged_targets_match_actual_and_preserve_means(self):
        result=self.check();self.assertTrue(result['passed']);self.assertFalse(result['front_rear_targets_match'])
        self.assertTrue(result['mean_side_targets_preserved']);self.assertTrue(result['raw_targets_preserved'])

    def test_forged_observer_targets_fail(self):
        self.assertFalse(self.check(lambda d,r:d['rear_left_wheel_joint_velocity_cmd'].__setitem__(1,-.01))['passed'])

    def test_wrong_flags_and_lost_step_fail(self):
        self.assertFalse(self.check(lambda d,r:r.__setitem__((1,4),0))['passed'])
        self.assertFalse(self.check(lambda d,r:r.__setitem__((1,0),5))['passed'])

    def test_source_timestamp_mismatch_fails(self):
        self.assertFalse(self.check(lambda d,r:r.__setitem__((1,1),.0021))['passed'])


if __name__=='__main__':unittest.main()
