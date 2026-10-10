import importlib.util
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
from r5_turn_control_probe import dither, command_window, AMPLITUDE, ACCELERATION, PERIOD


class ProbeTests(unittest.TestCase):
    def test_stop_and_nonturn_do_not_dither(self):
        for t in (-1., 0., 20., 21.):
            self.assertEqual(dither(t, .2), 0)
        self.assertEqual(dither(.1, 0), 0)

    def test_clock_rejected(self):
        for t in (float('nan'), float('inf')):
            with self.assertRaises(ValueError): dither(t, .2)

    def test_continuous_zero_mean_bounded_profile(self):
        t = np.linspace(0, PERIOD, 4001)
        v = np.array([dither(x, .2) for x in t])
        self.assertLessEqual(max(abs(v)), AMPLITUDE+1e-12)
        self.assertLessEqual(max(abs(np.diff(v)/np.diff(t))), ACCELERATION+1e-9)
        self.assertAlmostEqual(float(np.trapz(v, t)), 0, places=12)
        self.assertAlmostEqual(dither(.1, .2), AMPLITUDE)
        self.assertAlmostEqual(dither(.3, -.2), -AMPLITUDE)

    def test_invalid_request_not_mislabeled_as_zero_command(self):
        with self.assertRaises(ValueError): command_window(None, {}, .1, .2)
        with self.assertRaises(ValueError): command_window(None, {}, 0, 0)

    def fixture(self):
        step = .001; t = np.arange(28001)*step
        names = ['t', 'dt', 'iteration', 'input_sequence', 'input_seen_sim']
        names += ['input_lx','input_ly','input_lz','input_ax','input_ay','input_az']
        names += [w+'_wheel_joint_velocity_cmd' for w in ('front_left','front_right','rear_left','rear_right')]
        names += ['x','y','z','qw','qx','qy','qz']
        data = np.zeros(len(t), dtype=[(n, float) for n in names])
        data['t'] = t; data['dt'] = step; data['iteration'] = np.arange(len(t)); data['input_sequence'] = 1
        data['input_seen_sim'] = t; data['qw'] = 1
        moving = (t > 5) & (t <= 25); data['input_az'][moving] = .2
        data['input_lx'][moving] = [dither(x-5, .2) for x in t[moving]]
        omega = np.minimum(.2, np.maximum(0, t-5)) - np.minimum(.2, np.maximum(0, t-25))
        for w in ('front_left','rear_left'): data[w+'_wheel_joint_velocity_cmd'] = (data['input_lx']-omega*.3311/2)/.076
        for w in ('front_right','rear_right'): data[w+'_wheel_joint_velocity_cmd'] = (data['input_lx']+omega*.3311/2)/.076
        row = lambda i: {n:float(data[n][i]) for n in names}
        result = dict(sim_started=0., sim_ended=28., schedule=dict(mode='paused_steps',step=step,
            parked=dict(after=dict(sim=5.),last_step=row(5000)),
            motion=dict(before=dict(sim=5.,iteration=5000),after=dict(sim=25.,iteration=25000),last_step=row(25000))))
        return data, result

    def test_intentional_dynamic_input_is_qualified(self):
        data, result = self.fixture()
        self.assertTrue(command_window(data, result, 0, .2)['passed'])

    def test_yaw_dropout_and_stale_input_fail(self):
        for change in ('drop', 'stale', 'missing_step', 'residual'):
            data, result = self.fixture()
            if change == 'drop': data['input_az'][12000] = 0
            if change == 'stale': data['input_seen_sim'][12000] -= .2
            if change == 'missing_step': data = np.delete(data, 12000)
            if change == 'residual': data['input_lx'][-1] = .01
            with self.subTest(change=change): self.assertFalse(command_window(data,result,0,.2)['passed'])

    def test_original_acceleration_and_axle_matching_required(self):
        data, result = self.fixture(); data['front_left_wheel_joint_velocity_cmd'][12000] += .1
        self.assertFalse(command_window(data,result,0,.2)['passed'])


if __name__ == '__main__': unittest.main()
