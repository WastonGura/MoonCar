#!/usr/bin/env python3
"""Loopback archive viewer with an explicitly enabled Nav2 navigation gateway."""
import argparse
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
import signal
import threading
import zipfile
from urllib.parse import parse_qs, urlsplit
from urllib.request import Request, ProxyHandler, build_opener
from urllib.error import HTTPError, URLError

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/mooncar_terrain/scripts'))
from terrain_map import TerrainMap
from terrain_planner import TerrainPlanner
from simulation_experiment import validate_task


class SimulationProxy:
    """Windows serves the frontend locally; only known control calls reach the container."""
    def __init__(self,url):
        parts=urlsplit(url)
        if parts.scheme!='http' or parts.hostname not in ('127.0.0.1','localhost') or not parts.port or parts.path or parts.query or parts.fragment or parts.username:
            raise ValueError('Simulation URL must be an HTTP loopback origin with an explicit port')
        self.url=url;self.opener=build_opener(ProxyHandler({}))

    def request(self,path,body=None,image=False):
        data=None if body is None else json.dumps(body,allow_nan=False).encode()
        request=Request(self.url+'/api/simulation'+path,data=data,headers={'Origin':self.url,'Content-Type':'application/json'})
        try:
            with self.opener.open(request,timeout=5) as response:
                result=response.read()
                return result if image else json.loads(result)
        except HTTPError as error:
            try:detail=json.loads(error.read()).get('error','仿真请求失败')
            except Exception:detail='仿真请求失败'
            raise ValueError(detail) from error
        except (URLError,TimeoutError) as error:
            raise ValueError('仿真服务未连接，请检查 Docker 工作台服务') from error

    def state(self):return self.request('')
    def apply(self,body):return self.request('/apply',body)
    def stop(self):return self.request('/stop',{})
    def command(self,action,body):return self.request('/'+action,body)
    def image(self):return self.request('/scene.jpg',image=True)
    def close(self):pass


def create_server(directory, port=8765, host='127.0.0.1', navigator=None, simulation=None):
    directory = Path(directory).resolve()

    def archive(name):
        if not isinstance(name, str) or '\\' in name or '..' in name.split('/') or Path(name).is_absolute() or not name.endswith('.npz'):
            raise ValueError('Invalid archive name')
        path = (directory / name).resolve()
        if not path.is_relative_to(directory) or not path.is_file():
            raise ValueError('Archive not found')
        return path

    replay_lock = threading.Lock()
    cursor = None
    cursor_source = None

    @lru_cache(maxsize=1)
    def load(path, modified):
        return TerrainMap.load(path)

    @lru_cache(maxsize=4)
    def snapshot(path, modified, frame):
        nonlocal cursor, cursor_source
        source = load(path, modified)
        if frame is None:
            frame = len(source.frames)
        if not isinstance(frame, int) or not 0 <= frame <= len(source.frames):
            raise ValueError('Invalid replay frame count')
        if frame == len(source.frames):
            terrain = source
        else:
            if cursor_source is not source or len(cursor.frames) > frame:
                cursor = TerrainMap(source.resolution, source.voxel_size, source.frame, source.source)
                cursor_source = source
            for i in range(len(cursor.frames), frame):
                stamp, points, translation, quaternion = source.frames[i]
                pose = source.robot_poses[i]
                cursor.add_frame(points, stamp, translation, quaternion,
                                 pose[:3] if pose is not None else None,
                                 pose[3:] if pose is not None else None)
            terrain = cursor
        result = terrain.snapshot()
        result['archive_updated_ns'] = source.archive_updated_ns
        return result

    def read(name, frame=None):
        path = archive(name)
        # ponytail: one replay cursor; serialize clients until multiple simultaneous viewers matter.
        with replay_lock:
            return snapshot(str(path), path.stat().st_mtime_ns, frame)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            if len(args) < 2 or not str(args[1]).startswith('2'):
                super().log_message(fmt, *args)

        def send(self, body, status=200, mime='application/json; charset=utf-8', attachment=None):
            payload = json.dumps(body, ensure_ascii=False, allow_nan=False).encode() if isinstance(body, (dict, list)) else body
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(payload)))
            if attachment:
                self.send_header('Content-Disposition', f'attachment; filename="{attachment}"')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self'; img-src 'self' data:; connect-src 'self'; object-src 'none'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(payload)

        def allowed_host(self):
            return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}')

        def do_GET(self):
            if not self.allowed_host():
                return self.send({'error': 'Loopback host required'}, 403)
            parts = urlsplit(self.path)
            query = parse_qs(parts.query)
            try:
                if parts.path == '/api/sessions':
                    return self.send(dict(sessions=[dict(name=p.relative_to(directory).as_posix()) for p in sorted(directory.rglob('*.npz')) if p.resolve().is_relative_to(directory) and 'research' not in p.relative_to(directory).parts]))
                if parts.path == '/api/simulation':
                    return self.send(simulation.state() if simulation else dict(enabled=False))
                if parts.path == '/api/simulation/scene.jpg':
                    if simulation is None:raise ValueError('当前服务未启用仿真')
                    return self.send(simulation.image(),mime='image/jpeg')
                if parts.path == '/api/task':
                    value = query.get('task', [''])[0]
                    if len(value.encode()) > 8192:
                        raise ValueError('Task file too large')
                    return self.send(validate_task(json.loads(value)), attachment='mooncar-experiment.json')
                if parts.path == '/api/navigation':
                    return self.send(navigator.state() if navigator else dict(enabled=False))
                if parts.path in ('/api/map', '/api/info', '/api/archive'):
                    name = query.get('session', [''])[0]
                    if parts.path == '/api/archive':
                        return self.send(archive(name).read_bytes(), mime='application/octet-stream')
                    if parts.path == '/api/info':
                        return self.send(TerrainMap.inspect(archive(name)))
                    frame = int(query['frame'][0]) if 'frame' in query else None
                    return self.send(read(name, frame))
                files = {'/': 'index.html', '/index.html': 'index.html', '/app.js': 'app.js',
                         '/replay_cache.js': 'replay_cache.js', '/experiment_editor.js': 'experiment_editor.js', '/style.css': 'style.css'}
                if parts.path in files:
                    path = ROOT / 'web' / files[parts.path]
                elif parts.path.startswith('/vendor/'):
                    base = (ROOT / 'web/node_modules/three').resolve()
                    path = (base / parts.path[len('/vendor/'):]).resolve()
                    if not path.is_relative_to(base) or path.suffix != '.js':
                        raise ValueError('Invalid asset path')
                else:
                    return self.send({'error': 'Not found'}, 404)
                mime = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8'}[path.suffix]
                return self.send(path.read_bytes(), mime=mime)
            except (ValueError, KeyError, OSError, TypeError, zipfile.BadZipFile) as error:
                return self.send({'error': str(error)}, 400)

        def do_POST(self):
            expected = f'http://{self.headers.get("Host")}'
            if not self.allowed_host() or self.headers.get('Origin') != expected:
                return self.send({'error': 'Same-origin request required'}, 403)
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 8192 or self.headers.get('Content-Type') != 'application/json':
                    raise ValueError('Expected a bounded JSON request')
                body = json.loads(self.rfile.read(size))
                if self.path in ('/api/simulation/apply', '/api/simulation/stop'):
                    if simulation is None:
                        raise ValueError('当前服务未启用仿真实验模式')
                    return self.send(simulation.apply(body) if self.path.endswith('/apply') else simulation.stop())
                if self.path in ('/api/simulation/move','/api/simulation/turn','/api/simulation/heartbeat','/api/simulation/cancel','/api/simulation/explore','/api/simulation/explore_stop'):
                    if simulation is None:raise ValueError('当前服务未启用仿真')
                    return self.send(simulation.command(self.path.rsplit('/',1)[1],body))
                if self.path.startswith('/api/navigation/'):
                    if navigator is None:
                        raise ValueError('Navigation is not enabled on this server')
                    if self.path == '/api/navigation/start':
                        if body['frame'] != read(body['session'])['frames']:
                            raise ValueError('Only the latest complete archive can be used for navigation')
                        return self.send(navigator.start(body['session'], body['goal'], body.get('yaw', 0.), body.get('mode', 'full')))
                    if self.path == '/api/navigation/heartbeat':
                        return self.send(navigator.heartbeat(body['token']))
                    if self.path == '/api/navigation/cancel':
                        return self.send(navigator.cancel(body['token']))
                    return self.send({'error': 'Not found'}, 404)
                if self.path != '/api/plan':
                    return self.send({'error': 'Not found'}, 404)
                terrain = read(body['session'], body['frame'])
                planner = TerrainPlanner(terrain['cells'], terrain['resolution'],
                                         navigator.planner_node.planning_vehicle if navigator else None)
                return self.send(dict(results=[planner.plan(body['start'], body['goal'], mode=mode)
                                                for mode in ('fixed', 'direction', 'uncertainty', 'full')]))
            except (ValueError, KeyError, OSError, TypeError, zipfile.BadZipFile) as error:
                return self.send({'error': str(error)}, 400)

    return ThreadingHTTPServer((host, port), Handler)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sessions', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--host', choices=('127.0.0.1', '0.0.0.0'), default='127.0.0.1',
                        help='Use 0.0.0.0 only inside a container published on host loopback')
    parser.add_argument('--ros-archive', type=Path, help='Opt in to Nav2 execution for this exact archive')
    parser.add_argument('--ros-sim-time', action='store_true')
    parser.add_argument('--simulation', action='store_true', help='Own isolated Gazebo setup and opt-in leased Nav2 goals in a sourced Ubuntu ROS environment')
    parser.add_argument('--simulation-url', help='Windows frontend: forward simulation controls to this loopback origin')
    args = parser.parse_args()
    args.sessions.mkdir(parents=True, exist_ok=True)
    if sum(bool(v) for v in (args.simulation,args.ros_archive,args.simulation_url))>1:
        parser.error('Choose one simulation, simulation-url, or ros-archive mode')
    simulation = None
    if args.simulation_url:
        simulation=SimulationProxy(args.simulation_url)
    if args.simulation:
        from simulation_experiment import SimulationExperiment
        simulation = SimulationExperiment(args.sessions)
    navigator = executor = ros_thread = planner_node = None
    if args.ros_archive:
        if args.ros_archive.resolve().parent != args.sessions.resolve():
            parser.error('--ros-archive must be inside --sessions')
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.signals import SignalHandlerOptions
        from terrain_planner_node import TerrainPlannerNode
        from navigation_gateway import NavigationGateway
        # Keep ROS alive until pending Nav2 actions are cancelled and acknowledged.
        rclpy.init(args=['--ros-args', '-p', f'use_sim_time:={str(args.ros_sim_time).lower()}'],
                   signal_handler_options=SignalHandlerOptions.NO)
        planner_node = TerrainPlannerNode()
        planner_node.set_parameters([Parameter('archive', value=str(args.ros_archive.resolve()))])
        navigator = NavigationGateway(planner_node)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(planner_node)
        executor.add_node(navigator)
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()
    server = create_server(args.sessions, args.port, args.host, navigator, simulation)
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    print(f'MoonCar archive viewer: http://127.0.0.1:{server.server_port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if simulation:
            simulation.close()
        if navigator:
            navigator.close()
            executor.shutdown()
            ros_thread.join(timeout=5.)
            navigator.destroy_node()
            planner_node.destroy_node()
            rclpy.shutdown()
        server.server_close()
