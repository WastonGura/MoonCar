"""Opt-in store-and-forward observation transport; never publishes motion commands."""
import hashlib
import json
import os
from pathlib import Path
import threading
from urllib.parse import urlsplit
from urllib.request import Request, build_opener, ProxyHandler, HTTPRedirectHandler


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix+'.tmp')
    with temporary.open('w', encoding='utf-8') as handle:
        json.dump(value, handle, allow_nan=False)
        handle.flush(); os.fsync(handle.fileno())
    os.replace(temporary, path)


class SurveyUplink:
    def __init__(self, url, token_file, task, spool, autostart=True):
        parts = urlsplit(url)
        if parts.scheme != 'http' or parts.hostname not in ('127.0.0.1', 'localhost') or not parts.port or parts.path or parts.query or parts.fragment or parts.username:
            raise ValueError('Uplink requires an HTTP loopback origin; use an SSH tunnel for another computer')
        self.url = url
        self.token = Path(token_file).read_text(encoding='utf-8').strip()
        if not 32 <= len(self.token) <= 256 or not self.token.isascii() or any(c.isspace() for c in self.token):
            raise ValueError('Invalid uplink token file')
        self.task = task
        identity = {key: task[key] for key in ('format', 'id', 'coordinates', 'region', 'time', 'grid', 'targets', 'source')}
        self.signature = hashlib.sha256(json.dumps(identity, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        self.contract = dict(format='mooncar-survey-input-v1', **task['coordinates'],
            clock=task['time']['clock'], source_kind=task['source']['kind'],
            calibration_version=task['source']['calibration_version'], pose_version=task['source']['pose_version'])
        self.spool = Path(spool); self.spool.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock(); self.stop = threading.Event(); self.worker = None
        self.opener = build_opener(ProxyHandler({}), NoRedirect())
        self.error = None
        state_path = self.spool/'state.json'
        if state_path.exists():
            self.state = json.loads(state_path.read_text(encoding='utf-8'))
            if self.state['task_signature'] != self.signature:
                raise ValueError('Uplink spool belongs to another task')
        else:
            if list(self.spool.glob('frame-*.json')): raise ValueError('Spool state missing; preserve files and use a new spool')
            self.state = dict(task_signature=self.signature, session=None, acked=0)
            atomic_json(state_path, self.state)
        files = sorted(self.spool.glob('frame-*.json'))
        if [p.name for p in files] != [f'frame-{i:05d}.json' for i in range(len(files))]:
            raise ValueError('Uplink spool has a sequence gap')
        self.count = len(files)
        if not 0 <= self.state['acked'] <= self.count: raise ValueError('Invalid uplink acknowledgement state')
        self.last_stamp = json.loads(files[-1].read_text())['stamp_s'] if files else None
        if autostart:
            self.worker = threading.Thread(target=self.run, daemon=True); self.worker.start()

    def enqueue(self, points, stamp, translation, quaternion, robot_translation=None, robot_quaternion=None):
        with self.lock:
            if self.last_stamp is not None and stamp <= self.last_stamp:
                raise ValueError('Uplink timestamp must strictly increase; use a new spool after restart/clock reset')
            if self.count >= 5000 or sum(p.stat().st_size for p in self.spool.glob('frame-*.json')) > 500_000_000:
                raise ValueError('Uplink spool capacity reached; preserve it and start a new task')
            points = points.tolist() if hasattr(points, 'tolist') else points
            as_list = lambda v: v.tolist() if hasattr(v, 'tolist') else list(v)
            payload = dict(format='mooncar-live-frame-v1', sequence=self.count, contract=self.contract,
                stamp_s=float(stamp), points=points, pose=dict(translation_m=as_list(translation), quaternion_xyzw=as_list(quaternion)))
            if robot_translation is not None and robot_quaternion is not None:
                payload['robot_pose'] = dict(translation_m=as_list(robot_translation), quaternion_xyzw=as_list(robot_quaternion))
            atomic_json(self.spool/f'frame-{self.count:05d}.json', payload)
            self.count += 1; self.last_stamp = float(stamp)

    def request(self, path, body=None):
        data = None if body is None else json.dumps(body, allow_nan=False).encode()
        req = Request(self.url+path, data=data, headers={'Authorization': 'Bearer '+self.token, 'Content-Type': 'application/json'})
        with self.opener.open(req, timeout=5) as response:
            return json.loads(response.read())

    def drain_once(self):
        with self.lock:
            if self.state['acked'] >= self.count: return False
        remote = self.request('/api/survey/handshake')
        if remote['task_signature'] != self.signature or remote['contract'] != self.contract:
            raise ValueError('Receiver task/coordinate/clock contract differs from sender')
        with self.lock:
            if self.state['session'] is not None and self.state['session'] != remote['session']:
                raise ValueError('Receiver session changed; preserve spool and start a new task')
            self.state['session'] = remote['session']; atomic_json(self.spool/'state.json', self.state)
            index = self.state['acked']
        payload = json.loads((self.spool/f'frame-{index:05d}.json').read_text(encoding='utf-8'))
        payload['session'] = remote['session']
        result = self.request('/api/survey/frame', payload)
        if result.get('accepted') is not True or result.get('next_sequence') != index+1:
            raise ValueError('Unexpected receiver acknowledgement')
        with self.lock:
            self.state['acked'] = index+1; atomic_json(self.spool/'state.json', self.state)
        self.error = None
        return True

    def status(self):
        with self.lock:
            return dict(queued=self.count-self.state['acked'], acknowledged=self.state['acked'], error=self.error)

    def run(self):
        while not self.stop.is_set():
            try:
                if self.drain_once(): continue
                self.stop.wait(.1)
            except Exception as error:
                # Retain the original frame; retries cannot advance the ACK cursor.
                self.error = str(error); self.stop.wait(1.)

    def close(self):
        self.stop.set()
        if self.worker: self.worker.join(timeout=6.)
