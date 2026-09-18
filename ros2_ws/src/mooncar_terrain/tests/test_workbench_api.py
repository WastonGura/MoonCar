import http.client
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts'))
from map_workbench import create_server
from terrain_map import TerrainMap


class WorkbenchApiTests(unittest.TestCase):
    def test_replay_reuses_fusion_without_future_points_or_stale_archives(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.npz'
            terrain = TerrainMap(source='replay test')
            for i in range(7):
                terrain.add_frame([[0., 0., 0.], [.1, 0., 0.], [0., .1, 0.]], i + 1.,
                                  translation=(i, 0., 0.), robot_translation=(i, 0., 0.),
                                  robot_quaternion=(0., 0., 0., 1.))
            terrain.save(path)
            expected = {i: TerrainMap.load(path, until=i).snapshot() for i in range(8)}
            server = create_server(directory, port=0)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=5)
            calls = []
            original = TerrainMap.add_frame
            def counted(instance, *args, **kwargs):
                calls.append(args[1])
                return original(instance, *args, **kwargs)
            def read(frame):
                connection.request('GET', f'/api/map?session=test.npz&frame={frame}')
                response = connection.getresponse()
                self.assertEqual(response.status, 200)
                return json.loads(response.read())
            try:
                with patch.object(TerrainMap, 'add_frame', counted):
                    for frame in (7, 0, 1, 2, 3, 4, 5, 6, 2, 3):
                        result = read(frame)
                        result.pop('archive_updated_ns')
                        self.assertEqual(result, expected[frame])
                    # Full validation/fusion once, then 6 new frames, then a 3-frame rewind.
                    self.assertEqual(len(calls), 7 + 6 + 3)
                    before = len(calls)
                    read(3)
                    self.assertEqual(len(calls), before)
                previous_version = read(3)['archive_updated_ns']
                terrain.source = 'replacement archive'
                terrain.save(path)
                result = read(3)
                self.assertNotEqual(result['archive_updated_ns'], previous_version)
                self.assertEqual(result['source'], 'replacement archive')
            finally:
                connection.close()
                server.shutdown()
                server.server_close()
                thread.join()

    def test_archive_replay_bad_input_and_origin(self):
        with tempfile.TemporaryDirectory() as directory:
            terrain = TerrainMap(source='unit test, not vehicle data')
            terrain.add_frame([[0., 0., 0.], [.1, 0., 0.], [0., .1, 0.]], 1.)
            terrain.save(Path(directory) / 'test.npz')
            (Path(directory) / 'broken.npz').write_bytes(b'not a zip')
            server = create_server(directory, port=0)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=5)
            try:
                connection.request('GET', '/api/navigation')
                response = connection.getresponse()
                self.assertFalse(json.loads(response.read())['enabled'])
                for path, expected in [('/api/map?session=test.npz&frame=1', 200),
                                       ('/api/map?session=test.npz&frame=0', 200),
                                       ('/api/map?session=../test.npz', 400),
                                       ('/api/map?session=broken.npz', 400),
                                       ('/api/map?session=test.npz&frame=9', 400),
                                       ('/vendor/../../docs/requirements.md', 400)]:
                    connection.request('GET', path)
                    response = connection.getresponse()
                    self.assertEqual(response.status, expected, path)
                    body = json.loads(response.read())
                    if path.endswith('frame=0'):
                        self.assertEqual(body['points'], [])
                request = json.dumps(dict(session='test.npz', frame=1, start=[0, 0, 0], goal=[1, 1]))
                for origin, expected in [('http://external.invalid', 403),
                                         (f'http://127.0.0.1:{server.server_port}', 200)]:
                    connection.request('POST', '/api/plan', request,
                                       {'Content-Type': 'application/json', 'Origin': origin})
                    response = connection.getresponse()
                    self.assertEqual(response.status, expected)
                    response.read()
            finally:
                connection.close()
                server.shutdown()
                server.server_close()
                thread.join()


if __name__ == '__main__':
    unittest.main()
