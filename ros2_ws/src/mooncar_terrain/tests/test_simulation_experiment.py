"""Task boundary checks and HTTP isolation; real spawn evidence is recorded separately."""
import http.client
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from urllib.parse import quote
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'scripts'))
from simulation_experiment import SimulationExperiment, validate_task, survey_world
from map_workbench import create_server, SimulationProxy
from terrain_map import TerrainMap


class ExperimentTests(unittest.TestCase):
    def test_startup_refreshes_missing_tf_once(self):
        with tempfile.TemporaryDirectory() as directory:
            sim=SimulationExperiment.__new__(SimulationExperiment)
            sim.lock=threading.Lock();sim.directory=Path(directory)
            sim.current=dict(id='experiment-test',start=[0.,0.,0.]);sim.phase='starting';sim.error=''
            sim.process=Mock();sim.process.poll.return_value=None;sim.started_at=100.;sim.tf_refreshed=False
            sim.motion=Mock();sim.motion.start_checked=False
            sim.motion.state.side_effect=[dict(ready=False,pose=None,diagnostics={}),
                                          dict(ready=True,pose=dict(x=0.,y=0.),diagnostics={}),
                                          dict(ready=True,pose=dict(x=0.,y=0.),diagnostics={})]
            (sim.directory/'experiment-test').mkdir()
            with patch('simulation_experiment.time.monotonic',return_value=111.):
                self.assertTrue(sim.state()['motion']['ready'])
                self.assertTrue(sim.state()['motion']['ready'])
            sim.motion.refresh_tf_listener.assert_called_once_with()

    def test_readiness_waits_for_current_trial_start(self):
        with tempfile.TemporaryDirectory() as directory:
            sim=SimulationExperiment.__new__(SimulationExperiment)
            sim.lock=threading.Lock();sim.directory=Path(directory)
            sim.current=dict(id='experiment-test',start=[6.5,-2.,2.4]);sim.phase='starting';sim.error=''
            sim.process=Mock();sim.process.poll.return_value=None;sim.started_at=__import__('time').monotonic()
            (sim.directory/'experiment-test').mkdir()
            sim.motion=Mock();sim.motion.start_checked=False
            sim.motion.state.return_value=dict(ready=True,pose=dict(x=-1.4,y=1.55),diagnostics={})
            self.assertFalse(sim.state()['motion']['ready'])
            self.assertGreater(sim.state()['motion']['diagnostics']['planned_start_error_m'],8)
            sim.motion.state.return_value=dict(ready=True,pose=dict(x=6.5,y=-2.),diagnostics={})
            self.assertTrue(sim.state()['motion']['ready'])
            sim.motion.start_checked=True
            sim.motion.state.return_value=dict(ready=True,pose=dict(x=7.5,y=-2.),diagnostics={})
            self.assertTrue(sim.state()['motion']['ready'])

    def test_observation_turn_is_scoped_to_current_inactive_autonomy(self):
        sim=SimulationExperiment.__new__(SimulationExperiment);sim.lock=threading.Lock()
        sim.phase='starting';sim.current=dict(id='current',region=[-5,-5,5,5])
        sim.process=Mock();sim.process.poll.return_value=None
        sim.autonomy=Mock();sim.autonomy.state.return_value=dict(active=False)
        sim.motion=Mock();sim.motion.turn.return_value=dict(token='lease')
        self.assertEqual(sim.command('turn',dict(experiment='current',yaw=0.)),dict(token='lease'))
        sim.motion.turn.assert_called_once_with(0.,sim.current)
        for body in (dict(experiment='old',yaw=0.),dict(experiment='current'),dict(experiment='current',yaw=0.,extra=1)):
            with self.assertRaises(ValueError):sim.command('turn',body)
        sim.autonomy.state.return_value=dict(active=True)
        with self.assertRaises(ValueError):sim.command('turn',dict(experiment='current',yaw=0.))
        self.assertEqual(sim.motion.turn.call_count,1)

    def test_reset_waits_for_previous_publishers_to_exit(self):
        sim=SimulationExperiment.__new__(SimulationExperiment)
        sim.motion=Mock();sim.process=Mock();sim.process.pid=123456;sim.log=None
        calls=Mock();calls.attach_mock(sim.motion,'motion');calls.attach_mock(sim.process,'process')
        with patch('simulation_experiment.os.killpg'),patch('simulation_experiment.Path.glob',return_value=[]):
            sim._stop()
        names=[c[0] for c in calls.mock_calls]
        self.assertLess(names.index('motion.cancel'),names.index('process.wait'))
        self.assertLess(names.index('process.wait'),names.index('motion.reset'))

    def test_windows_proxy_keeps_browser_requests_same_origin(self):
        with self.assertRaises(ValueError):SimulationProxy('http://external.invalid:8766')
        with tempfile.TemporaryDirectory() as directory:
            target=Mock();target.state.return_value=dict(enabled=True,phase='idle');target.command.return_value=dict(token='test')
            backend=create_server(directory,port=0,simulation=target)
            proxy=SimulationProxy(f'http://127.0.0.1:{backend.server_port}')
            frontend=create_server(directory,port=0,simulation=proxy)
            threads=[threading.Thread(target=server.serve_forever,daemon=True) for server in (backend,frontend)]
            for thread in threads:thread.start()
            client=http.client.HTTPConnection('127.0.0.1',frontend.server_port)
            try:
                client.request('GET','/api/simulation');response=client.getresponse()
                self.assertEqual(json.loads(response.read()),dict(enabled=True,phase='idle'))
                body=dict(experiment='test',goal=[4.8,3])
                for origin,status in [('http://external.invalid',403),(f'http://127.0.0.1:{frontend.server_port}',200)]:
                    client.request('POST','/api/simulation/move',json.dumps(body),{'Content-Type':'application/json','Origin':origin})
                    response=client.getresponse();self.assertEqual(response.status,status);response.read()
                target.command.assert_called_once_with('move',body)
            finally:
                client.close()
                for server in (frontend,backend):server.shutdown();server.server_close()
                for thread in threads:thread.join()

    def test_missing_sensor_archive_and_exited_launch_report_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            sim=SimulationExperiment.__new__(SimulationExperiment)
            sim.lock=threading.Lock();sim.directory=Path(directory)
            sim.current=dict(id='experiment-test');sim.phase='starting';sim.error='';sim.started_at=10
            (sim.directory/'experiment-test').mkdir()
            sim.process=Mock();sim.process.poll.return_value=None
            with patch('simulation_experiment.time.monotonic',return_value=131):
                self.assertEqual(sim.state()['phase'],'failed')
            saved=json.loads((sim.directory/'experiment-test/startup-failure.json').read_text())
            self.assertEqual(saved['archive_count'],0)
            self.assertEqual(saved['elapsed_wall_seconds'],121)
            sim.process.poll.return_value=1
            self.assertEqual(sim.state()['phase'],'failed')
            self.assertEqual(json.loads((sim.directory/'experiment-test/startup-failure.json').read_text()),saved)
            sim.phase='stopped'
            self.assertEqual(sim.state()['phase'],'stopped')

    def test_task_validation_and_nested_archive_api(self):
        world=survey_world()
        self.assertEqual(world.find('.//plane/size').text.strip(),'50 50')
        self.assertEqual(len([m for m in world.findall('./world/model') if m.get('name','').startswith('survey_landmark_')]),32)
        self.assertEqual(validate_task(dict(version=1,region=[-25,-25,25,25],start=[3.5,3,0]))['region'],[-25,-25,25,25])
        with self.assertRaises(ValueError):validate_task(dict(version=1,region=[-25,-25,25,25],start=[-20,-20,0]))
        valid = dict(version=1, region=[-1,-2,6,4], start=[.5,0,0])
        self.assertEqual(validate_task(valid), valid)
        self.assertFalse(validate_task(dict(valid,gui=False))['gui'])
        for gui in (0,'false',None):
            with self.assertRaises(ValueError):validate_task(dict(valid,gui=gui))
        for scene in ('visible_ramp_v1','visible_curve_v1'):
            task=dict(valid,start=[0,-.6,.463648],scene=scene)
            self.assertEqual(validate_task(task),task)
            self.assertEqual(len(survey_world(scene).findall('./world/model')),6)
            with self.assertRaises(ValueError):validate_task(dict(task,start=[3,0,0]))
        for scene in ('../world.sdf','unknown',None,[],True):
            with self.assertRaises(ValueError):validate_task(dict(valid,scene=scene))
        self.assertEqual(validate_task(dict(valid,start=[3.5,3,math.pi/2]))['start'], [3.5,3,math.pi/2])
        invalid = [dict(valid,start=[2,1.4,0]), dict(valid,start=[4,0,0]),
                   dict(valid,start=[True,0,0]),dict(valid,start=[float('nan'),0,0]),
                   dict(valid,start=[26,0,0]),dict(valid,start=[0,0,4]),
                   dict(valid,region=[1,1,0,0]),dict(valid,version=True),dict(valid,extra='x')]
        for task in invalid:
            with self.assertRaises(ValueError):validate_task(task)
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)/'experiment-test';folder.mkdir()
            terrain=TerrainMap(source='unit test');terrain.add_frame([[0,0,0],[.1,0,0],[0,.1,0]],1);terrain.save(folder/'survey.npz')
            evidence=folder/'research/autonomy-test';evidence.mkdir(parents=True);terrain.save(evidence/'observations.npz')
            simulation=Mock();simulation.state.return_value=dict(enabled=True,phase='idle');simulation.apply.return_value=dict(accepted=True)
            server=create_server(directory,port=0,simulation=simulation)
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            client=http.client.HTTPConnection('127.0.0.1',server.server_port)
            try:
                client.request('GET','/api/task?task='+quote(json.dumps(valid)))
                response=client.getresponse();self.assertEqual(response.status,200)
                self.assertIn('mooncar-experiment.json',response.getheader('Content-Disposition'))
                self.assertEqual(json.loads(response.read()),valid)
                for path in ['/api/sessions','/api/info?session=experiment-test/survey.npz','/api/simulation']:
                    client.request('GET',path);response=client.getresponse();self.assertEqual(response.status,200);body=json.loads(response.read())
                    if path=='/api/sessions':self.assertEqual(body['sessions'],[dict(name='experiment-test/survey.npz')])
                for origin,status in [('http://external.invalid',403),(f'http://127.0.0.1:{server.server_port}',200)]:
                    client.request('POST','/api/simulation/apply',json.dumps(valid),{'Content-Type':'application/json','Origin':origin})
                    response=client.getresponse();self.assertEqual(response.status,status);response.read()
                simulation.apply.assert_called_once_with(valid)
                for path in ['../survey.npz','experiment-test/../survey.npz','%2Fetc%2Ftest.npz','experiment-test%5Csurvey.npz']:
                    client.request('GET','/api/info?session='+path);response=client.getresponse();self.assertEqual(response.status,400);response.read()
            finally:
                client.close();server.shutdown();server.server_close();thread.join()


if __name__=='__main__':unittest.main()
