"""Incomplete or aliased native observations cannot satisfy control acceptance."""
from pathlib import Path
import sys
import json
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from r5_control_analysis import analyze_steps

class StepObservationTests(unittest.TestCase):
    def check(self,defect=None):
        fields=['iteration','t','dt','x','y','z','qw','qx','qy','qz']
        for joint in ('camera_pan_joint','camera_tilt_joint','front_left_wheel_joint','front_right_wheel_joint','rear_left_wheel_joint','rear_right_wheel_joint'):
            fields += [joint+'_q',joint+'_v',joint+'_force_cmd']
        data=np.zeros((3001,len(fields)))
        data[:,0]=np.arange(3001);data[:,1]=np.arange(3001)*.001;data[:,2]=.001
        if defect=='gap':data=np.delete(data,1000,axis=0)
        if defect=='velocity':data[:,fields.index('camera_pan_joint_v')]=.2
        if defect=='torque':data[:,fields.index('camera_tilt_joint_force_cmd')]=np.nan
        if defect=='short':data=data[:-20]
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'data.csv';np.savetxt(p,data,delimiter=',',header=','.join(fields),comments='')
            return analyze_steps(p,dict(sim_started=0.,sim_ended=3.,sweep=[dict(target=[0.,0.],finished=0.)]))

    def test_complete_native_hold_passes(self):
        self.assertTrue(json.loads(json.dumps(self.check()))['passed'])
    def test_missing_step_tail_or_torque_fails(self):
        for defect in ('gap','short','torque'):self.assertFalse(self.check(defect)['passed'],defect)
    def test_small_position_with_large_velocity_fails(self):self.assertFalse(self.check('velocity')['passed'])

if __name__=='__main__':unittest.main()
