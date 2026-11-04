"""Timing qualification must use physical progress and actual wheel targets."""
import json
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from r5_step_schedule import step_count,confirmed,pause_marker,stats_state,last_step,PausedSteps
from r5_d0_analysis import command_window,compare_pair,INPUT_FIELDS,WHEELS
from run_r5_d0_acceptance import validate_calibration,CampaignStop


class PausedTests(unittest.TestCase):
    def test_actual_gazebo_json_time_is_not_silently_zeroed(self):
        self.assertAlmostEqual(stats_state(dict(simTime=dict(sec='2',nsec=300000000),iterations='2300',paused=True),1)['sim'],2.3)
        self.assertEqual(stats_state(dict(sim_time=dict(sec='2'),iterations='2'),1)['sim'],2)
        with self.assertRaises(ValueError):stats_state(dict(iterations='100'),1)

    def test_unavailable_wheel_force_is_null_without_rewriting_raw_csv(self):
        with tempfile.TemporaryDirectory() as name:
            path=Path(name)/'steps.csv';raw='iteration,t,wheel_force_cmd\n100,0.1,nan\n';path.write_text(raw)
            value=last_step(path);self.assertIsNone(value['wheel_force_cmd'])
            json.dumps(value,allow_nan=False);self.assertEqual(path.read_text(),raw)

    def test_integer_steps_do_not_round_a_partial_step(self):
        self.assertEqual(step_count(20,.00025),80000)
        for seconds,step in ((2,.3),(0,.001),(1,0)):
            with self.assertRaises(ValueError):step_count(seconds,step)

    def test_two_recent_consecutive_confirmations_require_all_axes(self):
        row=lambda t,v:dict(kind='safe_command',wall=t,values=v)
        target=[.1,0,0,0,0,0]
        self.assertFalse(confirmed([row(1,target)],'safe_command',target,0))
        self.assertTrue(confirmed([row(1,target),row(2,target)],'safe_command',target,0))
        self.assertFalse(confirmed([row(1,target),row(2,[0]*6),row(3,target)],'safe_command',target,0))
        self.assertFalse(confirmed([row(1,target),row(2,target)],'safe_command',target,1.5))
        self.assertFalse(confirmed([row(1,[.1]),row(2,[.1])],'safe_command',target,0))

    def test_command_ack_timeout_does_not_advance_physics(self):
        obj=PausedSteps.__new__(PausedSteps);obj.deadline=10;obj.lock=threading.Lock()
        obj.node=SimpleNamespace(lock=threading.Lock(),rows=[]);obj.wait_state=Mock(return_value=dict(iteration=0,sim=0))
        obj.healthy=Mock();obj.advance=Mock()
        with patch('r5_step_schedule.time.monotonic',side_effect=[0,6]):
            with self.assertRaises(TimeoutError):obj.acknowledge(.1,0)
        obj.advance.assert_not_called()

    def test_service_delivery_is_not_paused_step_completion(self):
        obj=PausedSteps.__new__(PausedSteps);obj.deadline=10;obj.healthy=Mock()
        obj.latest=dict(iteration=20,sim=.02,wall=1,paused=True)
        obj.condition=Mock();obj.condition.__enter__=Mock();obj.condition.__exit__=Mock(return_value=False)
        obj.condition.wait.side_effect=lambda _:setattr(obj,'latest',dict(iteration=50,sim=.05,wall=2,paused=True))
        with patch('r5_step_schedule.time.monotonic',return_value=1):
            self.assertEqual(obj.wait_state(50)['iteration'],50)
        obj.condition.wait.assert_called_once()
        obj.latest['iteration']=51
        with patch('r5_step_schedule.time.monotonic',return_value=1):
            with self.assertRaises(RuntimeError):obj.wait_state(50)

    def test_stale_flush_confirmation_fails(self):
        with tempfile.TemporaryDirectory() as name:
            path=Path(name)/'marker';path.write_text('5000,5\n')
            self.assertTrue(pause_marker(path,5000,5))
            self.assertFalse(pause_marker(path,25000,25))
            self.assertFalse(pause_marker(path,5000,5.001))

    def fixture(self):
        fields=('t','dt','x','y','z','qw','qx','qy','qz')+INPUT_FIELDS+('input_sequence','input_seen_sim')+tuple(w+'_wheel_joint_velocity_cmd' for w in WHEELS)
        data=np.zeros(28001,dtype=[(name,float) for name in fields]);t=np.arange(len(data))*.001
        data['t']=t;data['dt']=.001;data['qw']=1;data['input_sequence']=np.arange(len(data))+1;data['input_seen_sim']=t
        data['input_lx'][(t>=5)&(t<25)]=.1
        speed=np.minimum(np.clip((t-5)*.5,0,.1),np.clip((25.2-t)*.5,0,.1))
        for w in WHEELS:data[w+'_wheel_joint_velocity_cmd']=speed/.076
        row=lambda index:{name:float(data[name][index]) for name in fields}
        result=dict(sim_started=0,sim_ended=28,schedule=dict(mode='paused_steps',step=.001,
            parked=dict(last_step=row(5000)),motion=dict(before=dict(sim=5),after=dict(sim=25),last_step=row(25000))))
        return data,result

    def test_native_input_and_original_acceleration_profile(self):
        data,result=self.fixture();window=command_window(data,result,.1,0)
        self.assertTrue(window['passed']);self.assertAlmostEqual(window['wheel_budget']['linear_integral_m'],2,places=8)
        self.assertEqual(window['endpoint']['t'],25)
        data['input_seen_sim'][25000:]+=.003
        self.assertFalse(command_window(data,result,.1,0)['passed'])

    def test_wrong_wheel_direction_or_target_dropout_fails(self):
        data,result=self.fixture();data['front_left_wheel_joint_velocity_cmd'][10000]*=-1
        self.assertFalse(command_window(data,result,.1,0)['passed'])
        data,result=self.fixture()
        for w in WHEELS:data[w+'_wheel_joint_velocity_cmd'][10000]=0
        self.assertFalse(command_window(data,result,.1,0)['passed'])

    def test_pair_requires_step_timing_and_matching_control_budget(self):
        data,result=self.fixture();a=dict(passed=True,formal=dict(command_window=command_window(data,result,.1,0)))
        b=json.loads(json.dumps(a));b['formal']['command_window']['step']=.0005
        self.assertTrue(compare_pair(a,b)['passed'])
        b['formal']['command_window']['duration_s']+=.002
        self.assertFalse(compare_pair(a,b)['valid'])
        b=json.loads(json.dumps(a));b['formal']['command_window']['wheel_budget']['angular_integral_rad']=.0021
        self.assertFalse(compare_pair(a,b)['valid'])

    def test_calibration_cannot_reuse_another_signature_or_exceed_budget(self):
        signature=dict(model_sha256='m',plugin_sha256='p',observer_sha256='o',source_sha256={'a':'s'})
        rows=[dict(step=step,passed=True,signature=signature) for step in (.001,.0005)]
        with tempfile.TemporaryDirectory() as name:
            path=Path(name)/'calibration-ledger.json';path.write_text(json.dumps(rows))
            self.assertEqual(validate_calibration(name,signature),2)
            with self.assertRaises(CampaignStop):validate_calibration(name,dict(signature,plugin_sha256='other'))
            path.write_text(json.dumps(rows*3))
            with self.assertRaises(CampaignStop):validate_calibration(name,signature)

if __name__=='__main__':unittest.main()
