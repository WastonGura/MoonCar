"""Real leased pan/tilt gateway and loopback HTTP routes; no ROS world."""
import ast
from contextlib import contextmanager
import http.client
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT/'scripts'))
from map_workbench import create_server
from simulation_experiment import SimulationExperiment


class ShadowPanTiltGatewayTests(unittest.TestCase):
    def fixture(self):
        simulation = SimulationExperiment.__new__(SimulationExperiment)
        simulation.lock = threading.RLock()
        simulation.phase = 'starting'
        simulation.current = dict(id='experiment-owned', region=[-15., -15., 15., 15.])
        simulation.process = Mock();simulation.process.poll.return_value = None
        simulation.autonomy = Mock();simulation.autonomy.state.return_value = dict(active=False)
        simulation.motion = Mock()
        simulation.motion.pan_tilt.return_value = dict(token='opaque-original-motion-lease')
        return simulation

    def test_real_command_forwards_exact_body_to_existing_motion_and_returns_its_lease(self):
        simulation = self.fixture()
        response = simulation.motion.pan_tilt.return_value
        for pan, tilt in ((0., -.2618), (2.356, -.611), (-2.356, .262)):
            body = dict(experiment='experiment-owned', pan=pan, tilt=tilt)
            self.assertIs(simulation.command('pan_tilt', body), response)
            simulation.motion.pan_tilt.assert_called_with(pan, tilt, simulation.current)
            self.assertIs(simulation.motion.pan_tilt.call_args.args[2], simulation.current)
        simulation.autonomy.start.assert_not_called()
        simulation.motion.start.assert_not_called();simulation.motion.turn.assert_not_called()

    def test_exact_keys_and_owned_experiment_are_required_before_motion_dispatch(self):
        valid = dict(experiment='experiment-owned', pan=0., tilt=-.26)
        bad = [None, [], {}, {k:v for k,v in valid.items() if k != 'pan'},
               dict(valid, token='not-a-new-input'), dict(valid, extra=True),
               dict(valid, experiment='experiment-stale')]
        for body in bad:
            with self.subTest(body=body):
                simulation = self.fixture()
                with self.assertRaises(ValueError):simulation.command('pan_tilt', body)
                simulation.motion.pan_tilt.assert_not_called()

    def test_active_autonomy_cannot_share_the_pan_tilt_action(self):
        simulation = self.fixture();simulation.autonomy.state.return_value = dict(active=True)
        with self.assertRaises(ValueError):
            simulation.command('pan_tilt', dict(experiment='experiment-owned', pan=0., tilt=-.26))
        simulation.motion.pan_tilt.assert_not_called();simulation.autonomy.stop.assert_not_called()

    def test_inactive_phase_missing_task_or_dead_process_cannot_dispatch(self):
        for kind in ('idle', 'stopped', 'failed', 'recording', 'missing_task', 'dead_process'):
            with self.subTest(kind=kind):
                simulation = self.fixture()
                if kind == 'missing_task':simulation.current = None
                elif kind == 'dead_process':simulation.process.poll.return_value = 1
                else:simulation.phase = kind
                with self.assertRaises(ValueError):
                    simulation.command('pan_tilt', dict(experiment='experiment-owned', pan=0., tilt=-.26))
                simulation.motion.pan_tilt.assert_not_called()

    def test_nonfinite_and_angle_errors_are_delegated_to_the_real_motion_contract(self):
        tree = ast.parse((ROOT/'scripts/experiment_motion.py').read_text(encoding='utf-8-sig'))
        class_ = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'ExperimentMotion')
        method = next(node for node in class_.body if isinstance(node, ast.FunctionDef) and node.name == 'pan_tilt')
        namespace = dict(math=math)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])),
                     'actual_motion_pan_tilt_contract', 'exec'), namespace)
        actual = namespace['pan_tilt']
        for pan, tilt in ((float('nan'), 0.), (float('inf'), 0.), (True, 0.), (2.357, 0.),
                          (0., float('nan')), (0., float('-inf')), (0., -.612), (0., .263)):
            with self.subTest(pan=pan, tilt=tilt):
                simulation = self.fixture()
                simulation.motion.pan_tilt.side_effect = lambda p,t,task: actual(SimpleNamespace(), p, t, task)
                with self.assertRaises(ValueError):
                    simulation.command('pan_tilt', dict(experiment='experiment-owned', pan=pan, tilt=tilt))
                simulation.motion.pan_tilt.assert_called_once_with(pan, tilt, simulation.current)

    @contextmanager
    def server(self, simulation):
        with tempfile.TemporaryDirectory() as directory:
            server = create_server(directory, host='127.0.0.1', port=0, simulation=simulation)
            thread = threading.Thread(target=lambda:server.serve_forever(poll_interval=.01), daemon=True)
            thread.start()
            try:yield server
            finally:
                server.shutdown();server.server_close();thread.join(timeout=2.)
                self.assertFalse(thread.is_alive())

    def post(self, server, body, origin=None):
        client = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=2.)
        try:
            headers = {'Content-Type':'application/json'}
            if origin is not None:headers['Origin'] = origin
            client.request('POST', '/api/simulation/pan_tilt', json.dumps(body), headers)
            response = client.getresponse()
            return response.status, json.loads(response.read())
        finally:client.close()

    def test_real_same_origin_http_route_reaches_real_simulation_command(self):
        simulation = self.fixture();simulation.command = Mock(wraps=simulation.command)
        body = dict(experiment='experiment-owned', pan=.1, tilt=-.26)
        with self.server(simulation) as server:
            status, result = self.post(server, body, f'http://127.0.0.1:{server.server_port}')
        self.assertEqual(status, 200)
        self.assertEqual(result, dict(token='opaque-original-motion-lease'))
        simulation.command.assert_called_once_with('pan_tilt', body)
        simulation.motion.pan_tilt.assert_called_once_with(.1, -.26, simulation.current)

    def test_http_bad_or_missing_origin_and_stale_experiment_are_rejected(self):
        simulation = self.fixture();simulation.command = Mock(wraps=simulation.command)
        body = dict(experiment='experiment-owned', pan=.1, tilt=-.26)
        with self.server(simulation) as server:
            for origin in ('http://external.invalid', None):
                with self.subTest(origin=origin):
                    status, result = self.post(server, body, origin)
                    self.assertEqual(status, 403);self.assertIn('error', result)
            simulation.command.assert_not_called();simulation.motion.pan_tilt.assert_not_called()
            status, result = self.post(server, dict(body, experiment='experiment-stale'),
                                       f'http://127.0.0.1:{server.server_port}')
            self.assertEqual(status, 400);self.assertIn('error', result)
        simulation.motion.pan_tilt.assert_not_called()


if __name__ == '__main__':
    unittest.main()
