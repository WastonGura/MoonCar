"""Native stimulus integrity, paired convergence and bounded formal acceptance."""
from pathlib import Path
import sys
import unittest
import json
import tempfile
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from r5_d0_analysis import command_window,formal_observations,compare_pair,choose_step,INPUT_FIELDS,WHEELS
from run_r5_d0_acceptance import check_budget,campaign_exit_code,CampaignStop

class D0Tests(unittest.TestCase):
    def data(self):
        fields=('t','x','y','z','qw','qx','qy','qz')+INPUT_FIELDS+('input_sequence','input_seen_sim')+tuple(w+'_wheel_joint_velocity_cmd' for w in WHEELS)
        data=np.zeros(28001,dtype=[(f,float) for f in fields]);t=np.arange(len(data))*.001
        data['t']=t;data['qw']=1.;data['input_sequence']=np.arange(len(data))+1;data['input_seen_sim']=t
        data['input_lx'][(t>=5)&(t<25)]=.1
        return data
    def result(self,x=0,yaw=0,duration=20):
        return dict(passed=True,formal=dict(command_window=dict(passed=True,duration_s=duration,
            endpoint=dict(xyz=[x,0,0],rpy=[0,0,yaw]))))
    def test_native_window_requires_expected_motion_and_stop(self):
        data=self.data();base=dict(sim_started=0,sim_ended=28)
        self.assertTrue(command_window(data,base,.1,0)['passed'])
        data['input_lx'][10000]=0
        self.assertFalse(command_window(data,base,.1,0)['passed'])
        data=self.data();data['input_lx'][-1]=.1
        self.assertFalse(command_window(data,base,.1,0)['passed'])
    def test_stale_or_wrong_axis_commands_fail(self):
        data=self.data();base=dict(sim_started=0,sim_ended=28)
        data['input_seen_sim']-=.2
        self.assertFalse(command_window(data,base,.1,0)['passed'])
        data=self.data();data['input_ly'][10000]=.01
        self.assertFalse(command_window(data,base,.1,0)['passed'])
    def test_missing_servo_commands_fail(self):
        data=self.data();data[WHEELS[0]+'_wheel_joint_velocity_cmd'][6000]=np.nan
        self.assertFalse(command_window(data,dict(sim_started=0,sim_ended=28),.1,0)['passed'])
    def test_pair_limits_and_unmatched_command_duration(self):
        self.assertTrue(compare_pair(self.result(),self.result(.009,.009))['passed'])
        self.assertFalse(compare_pair(self.result(),self.result(.011,0))['passed'])
        self.assertFalse(compare_pair(self.result(),self.result(0,.011))['passed'])
        self.assertFalse(compare_pair(self.result(),self.result(duration=20.02))['valid'])
        self.assertTrue(compare_pair(self.result(),self.result(duration=20.01))['valid'])
        self.assertFalse(compare_pair(self.result(),self.result(duration=20.0100001))['valid'])
    def test_wrapped_yaw_and_failed_round(self):
        self.assertTrue(compare_pair(self.result(yaw=3.14),self.result(yaw=-3.14))['passed'])
        failed=self.result();failed['passed']=False
        self.assertFalse(compare_pair(failed,self.result())['valid'])
    def test_step_choice_never_recommends_reference_step(self):
        good=[dict(passed=True)]*6;bad=[dict(passed=False)]*6
        self.assertEqual(choose_step(good)['step'],.001)
        self.assertIsNone(choose_step(bad)['step'])
        self.assertEqual(choose_step(bad,good)['step'],.0005)
        self.assertEqual(choose_step(bad,bad)['status'],'turn_consistency_unresolved')
        self.assertFalse(choose_step(good[:5],bad)['passed'])
    def test_formal_budget_separate_from_old_diagnostics(self):
        check_budget([{}]*33)
        with self.assertRaises(CampaignStop):check_budget([{}]*34)
        self.assertEqual(campaign_exit_code('passed'),0)
        for status in ('running','stopped','completed_not_passed'):
            self.assertNotEqual(campaign_exit_code(status),0)

    def capture(self,folder):
        data=self.data()
        np.savetxt(folder/'physics-steps.csv',data,delimiter=',',header=','.join(data.dtype.names),comments='')
        rows=[]
        for kind,period in (('rgb',1.),('depth',1.),('scan',.5),('imu',.05),('joints',.05),('truth',.05),('safe_command',.05)):
            for t in np.arange(0,28.00001,period):
                row=dict(kind=kind,stamp=float(t),sim=float(t),wall=float(t)*2)
                if kind in ('rgb','depth'):
                    row.update(file=kind+'-sample',finite=100,pixel_std=10.)
                    (folder/row['file']).touch()
                rows.append(row)
        rows.sort(key=lambda r:r['sim'])
        (folder/'observations.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
        (folder/'resources.jsonl').write_text(''.join(json.dumps(dict(wall=t,cpu_usage_us=t*1000000,memory_bytes=1024))+'\n' for t in (0,56)))
        return rows

    def check_capture(self,folder):
        return formal_observations(folder,dict(sim_started=0,sim_ended=28,errors=[]),'drive',.1,0.)

    def test_archived_sensor_files_are_required(self):
        with tempfile.TemporaryDirectory() as name:
            folder=Path(name);self.capture(folder)
            self.assertTrue(self.check_capture(folder)['passed'])
            (folder/'depth-sample').unlink()
            self.assertFalse(self.check_capture(folder)['data_complete'])

    def test_regular_joint_capture_gap_fails(self):
        with tempfile.TemporaryDirectory() as name:
            folder=Path(name);rows=self.capture(folder)
            rows=[r for r in rows if r['kind']!='joints' or not 10<r['stamp']<10.3]
            (folder/'observations.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            self.assertFalse(self.check_capture(folder)['sensors']['joints']['passed'])

    def test_safe_command_capture_gap_fails(self):
        with tempfile.TemporaryDirectory() as name:
            folder=Path(name);rows=self.capture(folder)
            rows=[r for r in rows if r['kind']!='safe_command' or not 10<r['sim']<10.3]
            (folder/'observations.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            self.assertFalse(self.check_capture(folder)['data_complete'])

if __name__=='__main__':unittest.main()
