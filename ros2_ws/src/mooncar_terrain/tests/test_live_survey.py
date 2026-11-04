import copy
import http.client
import io
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
import zipfile
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
from offline_survey import create_example, reconstruct, task_cells, quality_rows, load_reference
from survey_contract import read_json, load_input
from live_survey import LiveSurvey, signature
from survey_uplink import SurveyUplink
from map_workbench import create_server


class LiveSurveyTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name); self.input = self.base/'input'
        create_example(self.input)
        self.task_path = self.input/'task.json'; self.task = read_json(self.task_path)
        self.frames, self.contract, _, _ = load_input(self.task, self.task_path)
        self.now = [0.]
        self.receiver = LiveSurvey(self.task_path, self.base/'received', monotonic=lambda:self.now[0])

    def payload(self, index, stamp=None):
        f = self.frames[index]
        value = dict(format='mooncar-live-frame-v1', session=self.receiver.session, sequence=index,
                     contract=self.contract, stamp_s=f.stamp if stamp is None else stamp, points=f.points.tolist(),
                     pose=dict(translation_m=f.translation.tolist(), quaternion_xyzw=f.quaternion.tolist()))
        if f.robot is not None:
            value['robot_pose'] = dict(translation_m=f.robot[:3].tolist(), quaternion_xyzw=f.robot[3:].tolist())
        return value

    def server(self):
        token = 'test-credential-'+'x'*40
        server = create_server(self.base/'archives', port=0, live_survey=self.receiver, live_token=token)
        (self.base/'archives').mkdir(exist_ok=True)
        thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
        self.addCleanup(lambda:(server.shutdown(), server.server_close(), thread.join()))
        token_file = self.base/'credential'; token_file.write_text(token)
        return server, token_file

    def test_incremental_equals_offline_at_every_prefix_without_future_data(self):
        ref, _, qualified = load_reference(self.task, self.task_path, task_cells(self.task))
        self.assertEqual(self.receiver.snapshot()['connection_state'], 'waiting')
        for i in range(len(self.frames)):
            self.receiver.ingest(self.payload(i))
            expected, observed, log = reconstruct(self.task, self.frames[:i+1], task_cells(self.task))
            rows, accuracy = quality_rows(self.task, expected, observed, task_cells(self.task), ref, qualified)
            actual = self.receiver.snapshot()
            self.assertTrue(np.array_equal(self.receiver.terrain.cloud(), expected.cloud()))
            self.assertEqual(actual['quality_cells'], rows)
            self.assertEqual(actual['accuracy'], accuracy)
            self.assertEqual(actual['map'], expected.snapshot())
            self.assertEqual(actual['frame_counts']['outside_time_window'], sum(r['status']=='outside_time_window' for r in log))
        self.assertEqual(actual['coverage']['state_counts']['qualified'], 55)
        self.assertEqual(actual['frame_counts']['outside_time_window'], 1)

    def test_duplicate_is_not_refused_but_never_fused_twice(self):
        with self.assertRaises(ValueError): self.receiver.ingest(self.payload(1))
        first = self.payload(0); self.receiver.ingest(first)
        cloud = self.receiver.terrain.cloud()
        self.assertTrue(self.receiver.ingest(first)['duplicate'])
        self.assertTrue(np.array_equal(cloud, self.receiver.terrain.cloud()))
        first['points'][0][2] += .1
        with self.assertRaises(ValueError): self.receiver.ingest(first)
        self.assertEqual(self.receiver.snapshot()['accepted_frames'], 1)

    def test_invalid_contract_geometry_and_clock_are_rejected_before_commit(self):
        original = self.payload(0)
        bad = []
        for field, value in [('unit','mm'), ('clock','utc'), ('frame','other'), ('pose_version','changed')]:
            p = copy.deepcopy(original); p['contract'][field] = value; bad.append(p)
        for alteration in ('nan','quaternion','shape','session','sequence','world'):
            p = copy.deepcopy(original)
            if alteration == 'nan': p['points'][0][0] = float('nan')
            if alteration == 'quaternion': p['pose']['quaternion_xyzw'] = [0,0,0,2]
            if alteration == 'shape': p['points'] = [[1,2]]
            if alteration == 'session': p['session'] = 'another'
            if alteration == 'sequence': p['sequence'] = True
            if alteration == 'world': p['points'][0][0] = 20000
            bad.append(p)
        for value in bad:
            with self.assertRaises(ValueError): self.receiver.ingest(value)
        self.assertEqual(read_json(self.base/'received/frames/manifest.json')['frames'], [])
        self.receiver.ingest(original)
        with self.assertRaises(ValueError): self.receiver.ingest(self.payload(1, original['stamp_s']))
        self.assertEqual(self.receiver.snapshot()['accepted_frames'], 1)

    def test_capacity_rejection_is_explicit(self):
        self.receiver.total_points = 5_000_000
        with self.assertRaisesRegex(ValueError, 'capacity'): self.receiver.ingest(self.payload(0))
        self.assertEqual(self.receiver.snapshot()['accepted_frames'], 0)

    def test_stale_and_restart_do_not_claim_fresh_sensor_data(self):
        self.receiver.ingest(self.payload(0))
        self.now[0] = 6
        self.assertEqual(self.receiver.snapshot()['connection_state'], 'stale')
        self.receiver.ingest(self.payload(0))
        self.assertEqual(self.receiver.snapshot()['connection_state'], 'stale')
        restarted = LiveSurvey(self.task_path, self.base/'received')
        self.assertEqual(restarted.session, self.receiver.session)
        self.assertEqual(restarted.snapshot()['connection_state'], 'historical')
        self.assertTrue(np.array_equal(restarted.terrain.cloud(), self.receiver.terrain.cloud()))
        self.assertTrue(restarted.ingest(self.payload(0))['duplicate'])
        changed = copy.deepcopy(self.task); changed['region']['bounds_m'][0] -= 1
        path = self.base/'other.json'; path.write_text(json.dumps(changed))
        with self.assertRaises(ValueError): LiveSurvey(path, self.base/'received')

    def test_commit_interruption_before_manifest_can_be_retried_after_restart(self):
        import live_survey
        real = live_survey.os.replace
        def fail_manifest(source, target):
            if Path(target).name == 'manifest.json': raise OSError('simulated disk interruption')
            return real(source, target)
        with patch.object(live_survey.os, 'replace', side_effect=fail_manifest):
            with self.assertRaises(OSError): self.receiver.ingest(self.payload(0))
        restarted = LiveSurvey(self.task_path, self.base/'received')
        self.assertEqual(restarted.snapshot()['accepted_frames'], 0)
        self.assertTrue(restarted.ingest(self.payload(0))['accepted'])
        self.assertEqual(restarted.snapshot()['accepted_frames'], 1)

    def test_no_reference_remains_unknown_even_when_coverage_is_good(self):
        value = copy.deepcopy(self.task); value.pop('reference'); value.pop('configuration')
        path = self.base/'no-ref.json'; path.write_text(json.dumps(value))
        receiver = LiveSurvey(path, self.base/'no-ref-output')
        for i in range(3):
            p = self.payload(i); p['session'] = receiver.session; receiver.ingest(p)
        result = receiver.snapshot()
        self.assertFalse(result['accuracy']['evaluable'])
        self.assertIsNone(result['accuracy']['P'])
        self.assertEqual(result['coverage']['state_counts']['qualified'], 0)
        self.assertGreater(result['coverage']['state_counts']['fitted_accuracy_unknown'], 0)
        self.assertEqual(result['configuration']['status'], 'not_checked')

    def test_receiver_folder_is_directly_replayable_by_offline_adapter(self):
        for i in range(4): self.receiver.ingest(self.payload(i))
        path = self.base/'received/task.json'; task = read_json(path)
        frames, _, _, _ = load_input(task, path)
        terrain, observed, _ = reconstruct(task, frames, task_cells(task))
        self.assertTrue(np.array_equal(terrain.cloud(), self.receiver.terrain.cloud()))
        self.assertEqual(observed, self.receiver.observed)
        self.assertNotIn('map', self.receiver.snapshot(after=4))
        self.assertIn('map', self.receiver.snapshot(after=3))

    def test_export_http_packages_received_prefix_with_replay_and_no_completion_claim(self):
        for i in range(4): self.receiver.ingest(self.payload(i))
        server, _ = self.server()
        connection = http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=20)
        self.addCleanup(connection.close)
        connection.request('GET', '/api/survey/export?session='+self.receiver.session)
        response=connection.getresponse();self.assertEqual(response.status,200)
        self.assertEqual(response.getheader('Content-Type'),'application/zip')
        blob=response.read()
        with zipfile.ZipFile(io.BytesIO(blob)) as archive:
            checks=json.loads(archive.read('checksums.json'))
            for name, sha in checks.items(): self.assertEqual(hashlib.sha256(archive.read(name)).hexdigest(),sha)
            self.assertEqual(json.loads(archive.read('live-snapshot.json'))['task_completion'],'not_asserted')
            self.assertEqual(json.loads(archive.read('summary.json'))['coverage'],self.receiver.snapshot()['coverage'] | {
                'task_grid_area_m2':4.0,'rectangle_area_m2':4})
            self.assertIn('task-replay.json',archive.namelist())
            self.assertIn('report.html',archive.namelist())
        self.assertEqual(self.receiver.export_zip(self.receiver.session)[0],blob)
        with self.assertRaises(ValueError): self.receiver.export_zip('wrong-session')

    def test_restart_detects_changed_manifest_or_stored_points(self):
        self.receiver.ingest(self.payload(0))
        path=self.base/'received/frames/manifest.json';value=read_json(path)
        value['frames'][0]['stamp_s']+=1;path.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError,'manifest'): LiveSurvey(self.task_path,self.base/'received')

    def test_http_auth_origin_body_bounds_and_live_assets(self):
        server, credential = self.server()
        connection = http.client.HTTPConnection('127.0.0.1', server.server_port)
        self.addCleanup(connection.close)
        body = json.dumps(self.payload(0))
        good = {'Content-Type':'application/json', 'Authorization':'Bearer '+credential.read_text()}
        for headers, expected in [({},403), (dict(good, Origin=f'http://127.0.0.1:{server.server_port}'),403),
                                  (dict(good, Authorization='Bearer wrong'),403), (good,200)]:
            connection.request('POST','/api/survey/frame',body,headers)
            response=connection.getresponse(); self.assertEqual(response.status,expected); response.read()
        connection.request('GET','/api/survey?after=1')
        response=connection.getresponse(); value=json.loads(response.read()); self.assertNotIn('map',value)
        connection.request('GET','/api/survey/handshake')
        response=connection.getresponse(); self.assertEqual(response.status,403); response.read()
        for content in ('{"x":NaN}', '{"x":1,"x":2}'):
            connection.request('POST','/api/survey/frame',content,good)
            response=connection.getresponse(); self.assertEqual(response.status,400); response.read()
        connection.request('POST','/api/survey/frame','{}',dict(good, **{'Content-Length':'12000001'}))
        response=connection.getresponse(); self.assertEqual(response.status,400); response.read()
        connection.request('GET','/live_survey.js')
        response=connection.getresponse(); self.assertEqual(response.status,200); self.assertIn(b'liveSurveyPanel',response.read())

    def test_uplink_disconnect_queue_and_lost_ack_retry_are_durable(self):
        server, credential = self.server()
        client = SurveyUplink(f'http://127.0.0.1:{server.server_port}',credential,self.task,self.base/'queue',autostart=False)
        self.addCleanup(client.close)
        f=self.frames[0]; client.enqueue(f.points,f.stamp,f.translation,f.quaternion)
        with patch.object(client, 'request', side_effect=OSError('disconnected')):
            with self.assertRaises(OSError): client.drain_once()
        self.assertEqual(client.status()['queued'],1)
        request=client.request
        def lost_ack(path, body=None):
            value=request(path,body)
            if path.endswith('/frame'): raise OSError('response lost after server committed')
            return value
        with patch.object(client,'request',side_effect=lost_ack):
            with self.assertRaises(OSError): client.drain_once()
        self.assertEqual(self.receiver.snapshot()['accepted_frames'],1)
        self.assertEqual(client.status()['acknowledged'],0)
        self.assertTrue(client.drain_once())
        self.assertEqual(client.status()['queued'],0)
        self.assertEqual(self.receiver.snapshot()['accepted_frames'],1)
        restored=SurveyUplink(client.url,credential,self.task,self.base/'queue',autostart=False)
        self.addCleanup(restored.close)
        self.assertFalse(restored.drain_once())
        self.assertTrue((self.base/'queue/frame-00000.json').exists())

    def test_uplink_rejects_changed_task_receiver_and_remote_http(self):
        server, credential=self.server()
        changed=copy.deepcopy(self.task);changed['targets']['height_tolerance_m']*=2
        client=SurveyUplink(f'http://127.0.0.1:{server.server_port}',credential,changed,self.base/'queue',autostart=False)
        self.addCleanup(client.close)
        f=self.frames[0];client.enqueue(f.points,f.stamp,f.translation,f.quaternion)
        with self.assertRaisesRegex(ValueError,'differs'): client.drain_once()
        self.assertEqual(self.receiver.snapshot()['accepted_frames'],0)
        with self.assertRaises(ValueError): SurveyUplink('http://192.0.2.1:8765',credential,self.task,self.base/'other')


if __name__ == '__main__': unittest.main()
