"""Acceptance must reject missing or contradictory independent measurements."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from check_survey_recovery_runtime import evaluate
from simulation_experiment import experiment_domain


class RuntimeAcceptanceTests(unittest.TestCase):
    def measurements(self):
        history = [dict(success=False,retry_blocked=True) for _ in range(3)]
        history += [dict(recovery=True,success=True)]
        history += [dict(success=False,retry_blocked=True) for _ in range(3)]
        state = dict(history=history,recovery_attempts=1,active=False,phase='fault',reason='expected failure')
        rows = [dict(kind='recovery_begin',wall=1.,frames=1)]
        for wall in (1.1,1.5,1.9):
            rows.extend([dict(kind='truth',wall=wall,x=0.,y=0.,yaw=0.),
                         dict(kind='joints',wall=wall,camera_pan_joint=1.5708,camera_tilt_joint=-.2618),
                         dict(kind='quality',wall=wall,frames=2),
                         dict(kind='depth',wall=wall,finite_returns=200)])
        rows += [dict(kind='safe_command',wall=1.1+i*.1,zero=True,values=[0.]*6) for i in range(6)]
        rows += [dict(kind='recovery_end',wall=2.,action_success_wall=1.95),
                 dict(kind='mission_end',wall=3.)]
        rows += [dict(kind='safe_command',wall=3.1+i*.1,zero=True,values=[0.]*6) for i in range(22)]
        calls = [dict(injected=True) for _ in range(6)]
        return state,rows,calls

    def test_measured_fault_stop_passes(self):
        self.assertTrue(evaluate('fault',*self.measurements(),[])['passed'])

    def test_missing_measurement_stream_cannot_pass(self):
        for kind in ('truth','joints','quality','depth','safe_command'):
            state,rows,calls = self.measurements()
            self.assertFalse(evaluate('fault',state,[r for r in rows if r['kind']!=kind],calls,[])['passed'],kind)

    def test_early_timer_success_is_rejected(self):
        state,rows,calls = self.measurements()
        next(r for r in rows if r['kind']=='recovery_end')['action_success_wall']=1.05
        report=evaluate('fault',state,rows,calls,[])
        self.assertFalse(report['checks']['joint_arrival_before_action_success'])

    def test_chassis_motion_extra_goal_and_old_frames_are_rejected(self):
        for fault in ('movement','extra_goal','old_frames','nonzero','short_stop'):
            state,rows,calls = self.measurements()
            if fault=='movement': next(r for r in rows if r['kind']=='truth')['x']=.1
            if fault=='extra_goal': calls.append(dict(injected=False))
            if fault=='old_frames':
                for row in rows:
                    if row['kind']=='quality': row['frames']=1
            if fault=='nonzero': next(r for r in rows if r['kind']=='safe_command')['values'][0]=.1
            if fault=='short_stop': rows=[r for r in rows if r['wall']<4.]
            self.assertFalse(evaluate('fault',state,rows,calls,[])['passed'],fault)

    def test_speed_claim_requires_raw_finite_components(self):
        for values in ([],[None]*6,[float('nan')]*6,[1.]*6):
            state,rows,calls=self.measurements()
            for row in rows:
                if row['kind']=='safe_command':row['values']=values
            self.assertFalse(evaluate('fault',state,rows,calls,[])['passed'])

    def test_natural_window_preserves_early_budget_guard_and_rejects_fault(self):
        state,rows,calls=self.measurements()
        state.update(phase='budget_reached',elapsed_seconds=75.2,started=0.)
        rows.append(dict(kind='observation_end',wall=5.3,sim=90.))
        report=evaluate('natural',state,rows,[],[])
        self.assertTrue(report['passed'])
        self.assertEqual(report['metrics']['mission_elapsed_seconds'],75.2)
        self.assertEqual(report['metrics']['observation_window_seconds'],90.)
        rows[-1]['sim']=89.
        self.assertFalse(evaluate('natural',state,rows,[],[])['passed'])
        rows[-1]['sim']=90.;state['phase']='fault'
        self.assertFalse(evaluate('natural',state,rows,[],[])['passed'])

    def test_domain_default_override_and_invalid_values(self):
        with patch.dict('os.environ',{},clear=True):
            self.assertEqual(experiment_domain(),'89')
        with patch.dict('os.environ',{'MOONCAR_EXPERIMENT_ROS_DOMAIN_ID':'90'}):
            self.assertEqual(experiment_domain(),'90')
        for value in ('-1','233','90.0','hello',''):
            with patch.dict('os.environ',{'MOONCAR_EXPERIMENT_ROS_DOMAIN_ID':value}):
                with self.assertRaises(ValueError): experiment_domain()


if __name__=='__main__': unittest.main()
