import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from check_r5_adaptive_turn_round import feedback_accuracy


class ShadowAccuracyTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.folder=Path(self.tmp.name)
        (self.folder/'result.json').write_text(json.dumps({'schedule':{'motion':{
            'before':{'sim':0},'after':{'sim':20}}}}))
        self.data=np.zeros((201,8));self.data[:,0]=np.arange(201)/10;self.data[:,4]=1
        self.write()
        self.decisions=[dict(stamp_ns=i*100000000,registration={'valid':True,'yaw':0},
            measured_body_delta_m=[0.,0.]) for i in range(201)]
    def tearDown(self):self.tmp.cleanup()
    def write(self):
        np.savetxt(self.folder/'physics-steps.csv',self.data,delimiter=',',
            header='t,x,y,z,qw,qx,qy,qz',comments='')
    def test_matching_independent_trajectory(self):
        result=feedback_accuracy(self.folder,self.decisions)
        self.assertTrue(result['passed']);self.assertEqual(result['compared_samples'],200)
    def test_zero_report_does_not_hide_real_translation(self):
        self.data[:,1]=np.arange(201)*.001;self.write()
        result=feedback_accuracy(self.folder,self.decisions)
        self.assertFalse(result['passed']);self.assertAlmostEqual(result['maximum_position_error_m'],.2)
    def test_invalid_pair_cannot_be_skipped(self):
        self.decisions[20]['registration']['valid']=False
        result=feedback_accuracy(self.folder,self.decisions)
        self.assertFalse(result['passed']);self.assertEqual(result['qualified_prefix'],19)


if __name__=='__main__':unittest.main()
