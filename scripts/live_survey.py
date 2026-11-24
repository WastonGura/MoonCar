"""Durable, ordered observation receiver. No ROS, motor control, or new terrain model."""
import copy
import hashlib
import json
import os
from pathlib import Path
import secrets
import shutil
import threading
import time
import zipfile

import numpy as np
from survey_contract import (Frame, TerrainMap, INPUT_FORMAT, keys, number, pose,
                             read_json, validate_task, validate_input_contract)
from terrain_geometry import transform_points
from offline_survey import task_cells, fuse_frame, quality_rows, load_reference, STATES, run_task
from survey_configuration import audit_configuration


def signature(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def task_signature(task):
    return signature({key: task[key] for key in ('format', 'id', 'coordinates', 'region',
                                                'time', 'grid', 'targets', 'source')})


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as handle:
        json.dump(value, handle, ensure_ascii=False, allow_nan=False)
        handle.flush(); os.fsync(handle.fileno())
    os.replace(temporary, path)


class LiveSurvey:
    def __init__(self, task_path, output, monotonic=time.monotonic):
        original = validate_task(read_json(task_path))
        self.task_signature = task_signature(original)
        self.directory = Path(output).resolve()
        self.lock = threading.Lock(); self.monotonic = monotonic
        self.last_received = None; self.cached = None; self.fault = None
        self.exported = None
        marker = self.directory/'receiver.json'
        if marker.exists():
            saved = read_json(marker)
            if saved['task_signature'] != self.task_signature:
                raise ValueError('Live directory belongs to another task; choose a new directory')
            self.session = saved['session']
            self.task = validate_task(read_json(self.directory/'task.json'))
        else:
            if self.directory.exists():
                raise ValueError('Live output must be new or a completed receiver directory')
            self.directory.mkdir(parents=True)
            self.session = secrets.token_hex(16)
            self.task = copy.deepcopy(original)
            self.task['input'] = dict(format='folder-v1', path='frames')
            # Freeze local reference/configuration inputs before accepting any frames.
            base = Path(task_path).resolve().parent
            for key in ('reference', 'configuration'):
                if key not in original: continue
                value = read_json((base/original[key]['path']).resolve())
                parent = (base/original[key]['path']).resolve().parent
                if key == 'reference' and value.get('alignment', {}).get('evidence_file'):
                    source = (parent/value['alignment']['evidence_file']).resolve()
                    shutil.copyfile(source, self.directory/'alignment-evidence.bin')
                    value['alignment']['evidence_file'] = 'alignment-evidence.bin'
                if key == 'configuration':
                    for i, row in enumerate(value.get('source_files', [])):
                        source = (parent/row['path']).resolve()
                        if source.is_file():
                            target = self.directory/f'configuration-source-{i}.txt'
                            shutil.copyfile(source, target); row['path'] = target.name
                atomic_json(self.directory/(key+'.json'), value)
                self.task[key]['path'] = key+'.json'
            atomic_json(self.directory/'task.json', self.task)
        self.contract = dict(format=INPUT_FORMAT, **self.task['coordinates'],
            clock=self.task['time']['clock'], source_kind=self.task['source']['kind'],
            calibration_version=self.task['source']['calibration_version'], pose_version=self.task['source']['pose_version'])
        self.cells = task_cells(self.task)
        self.reference, _, self.qualified = load_reference(self.task, self.directory/'task.json', self.cells)
        self.configuration = dict(status='not_checked', checks=[])
        if 'configuration' in self.task:
            value = read_json(self.directory/self.task['configuration']['path'])
            for row in value.get('source_files', []):
                row['path'] = str((self.directory/row['path']).resolve())
            self.configuration = audit_configuration(value)
        self.terrain = TerrainMap(self.task['grid']['cell_m'], self.task['grid']['voxel_m'],
                                  self.task['coordinates']['frame'], self.task['source']['description'])
        self.observed = {}; self.rows = []; self.log = []; self.total_points = 0
        frames = self.directory/'frames'; frames.mkdir(exist_ok=True)
        manifest = frames/'manifest.json'
        if manifest.exists():
            value = read_json(manifest)
            validate_input_contract(value['contract'], self.task)
            self.rows = value['frames']
            for index, row in enumerate(self.rows):
                payload = read_json(frames/f'{index:05d}.json')
                if signature(payload) != read_json(frames/f'{index:05d}.receipt.json')['sha256']:
                    raise ValueError('Stored live frame checksum mismatch')
                f = self.validate_frame(payload, index)
                expected_row = dict(stamp_s=f.stamp, points_file=f'{index:05d}.npy', pose=payload['pose'])
                if 'robot_pose' in payload: expected_row['robot_pose'] = payload['robot_pose']
                if row != expected_row or (index and f.stamp <= self.rows[index-1]['stamp_s']):
                    raise ValueError('Stored frame manifest/timestamp differs from receipt')
                if index >= 5000 or self.total_points+len(f.points) > 5_000_000:
                    raise ValueError('Stored live task exceeds capacity')
                if not np.array_equal(np.load(frames/row['points_file'], allow_pickle=False), f.points):
                    raise ValueError('Stored live points differ from receipt')
                self.log.append(fuse_frame(self.task, f, self.cells, self.terrain, self.observed))
                self.total_points += len(f.points)
        else:
            atomic_json(manifest, dict(contract=self.contract, frames=[]))
        if not marker.exists():
            atomic_json(marker, dict(session=self.session, task_signature=self.task_signature))

    def validate_frame(self, payload, expected=None):
        keys(payload, ('format', 'session', 'sequence', 'contract', 'stamp_s', 'points', 'pose'), ('robot_pose',))
        if payload['format'] != 'mooncar-live-frame-v1' or payload['session'] != self.session:
            raise ValueError('Wrong live format/session')
        seq = payload['sequence']
        if type(seq) is not int or not 0 <= seq < 5000 or (expected is not None and seq != expected):
            raise ValueError('Invalid frame sequence')
        validate_input_contract(payload['contract'], self.task)
        stamp = number(payload['stamp_s'], 'frame timestamp')
        t, q = pose(payload['pose'])
        raw = np.asarray(payload['points'])
        if raw.dtype.kind not in 'fiu': raise ValueError('Sensor XYZ must be numeric')
        points = np.asarray(raw, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or not 3 <= len(points) <= 100000:
            raise ValueError('Expected 3..100000 sensor XYZ samples')
        world = transform_points(points.astype(np.float32), t, q)
        if stamp < 0 or not np.isfinite(points).all() or np.max(abs(points)) > 10000 or not np.isfinite(world).all() or np.max(abs(world)) > 10000:
            raise ValueError('Invalid timestamp/points/world coordinates')
        robot = None
        if 'robot_pose' in payload:
            rt, rq = pose(payload['robot_pose']); robot = np.concatenate((rt, rq))
        return Frame(stamp, points, t, q, robot, 'Ordered live uplink; input declaration checked')

    def handshake(self):
        with self.lock:
            return dict(session=self.session, task_id=self.task['id'], task_signature=self.task_signature,
                        next_sequence=len(self.rows), contract=self.contract)

    def ingest(self, payload):
        with self.lock:
            if self.fault: raise ValueError('Receiver fault; restart and recover saved frames: '+self.fault)
            f = self.validate_frame(payload)
            seq = payload['sequence']; sha = signature(payload)
            frames = self.directory/'frames'
            if seq < len(self.rows):
                if read_json(frames/f'{seq:05d}.receipt.json')['sha256'] != sha:
                    raise ValueError('Sequence already contains a different frame')
                return dict(accepted=True, duplicate=True, next_sequence=len(self.rows))
            if seq != len(self.rows): raise ValueError('Missing earlier frame; resend in order')
            if self.rows and f.stamp <= self.rows[-1]['stamp_s']:
                raise ValueError('Clock reversed or repeated; start a new task/session')
            if self.total_points+len(f.points) > 5_000_000:
                raise ValueError('Live task capacity reached; start a new task')
            row = dict(stamp_s=f.stamp, points_file=f'{seq:05d}.npy', pose=payload['pose'])
            if 'robot_pose' in payload: row['robot_pose'] = payload['robot_pose']
            # Commit original frames before ACK. A lost response can be retried exactly once.
            with (frames/(row['points_file']+'.tmp')).open('wb') as handle:
                np.save(handle, f.points, allow_pickle=False); handle.flush(); os.fsync(handle.fileno())
            os.replace(frames/(row['points_file']+'.tmp'), frames/row['points_file'])
            atomic_json(frames/f'{seq:05d}.json', payload)
            atomic_json(frames/f'{seq:05d}.receipt.json', dict(sha256=sha))
            atomic_json(frames/'manifest.json', dict(contract=self.contract, frames=self.rows+[row]))
            try:
                self.log.append(fuse_frame(self.task, f, self.cells, self.terrain, self.observed))
            except Exception as error:
                self.fault = str(error); raise
            self.rows.append(row); self.total_points += len(f.points)
            self.last_received = self.monotonic(); self.cached = None
            return dict(accepted=True, duplicate=False, next_sequence=len(self.rows))

    def snapshot(self, after=None):
        with self.lock:
            age = None if self.last_received is None else max(0, self.monotonic()-self.last_received)
            state = 'fault' if self.fault else 'waiting' if not self.rows else 'historical' if age is None else 'stale' if age > 5 else 'receiving'
            result = dict(enabled=True, session=self.session, revision=len(self.rows),
                          connection_state=state, seconds_since_received=age, error=self.fault,
                          accepted_frames=len(self.rows), raw_points=self.total_points,
                          latest_stamp_s=self.rows[-1]['stamp_s'] if self.rows else None)
            if after == len(self.rows): return result
            if self.cached is None:
                rows, accuracy = quality_rows(self.task, self.terrain, self.observed, self.cells, self.reference, self.qualified)
                states = {state: sum(row['state'] == state for row in rows) for state in STATES}
                n = len(rows)
                coverage = dict(task_cells=n, observed_cells=sum(c['observed'] for c in rows),
                                fitted_cells=sum(c['valid'] for c in rows), state_counts=states)
                coverage.update(observed_fraction=coverage['observed_cells']/n, fitted_fraction=coverage['fitted_cells']/n)
                self.cached = dict(task=self.task, coverage=coverage, quality_cells=rows,
                    accuracy=accuracy or dict(status='no_independent_reference', evaluable=False,
                                              P=None, S=None, height_rmse_m=None, slope_mae_deg=None),
                    configuration=self.configuration, map=self.terrain.snapshot(),
                    frame_counts=dict(fused=len(self.terrain.frames), outside_time_window=sum(r['status']=='outside_time_window' for r in self.log)),
                    physical_accuracy_certified=False, safety_certified=False)
            result.update(self.cached)
            return result

    def export_zip(self, session):
        """Freeze a received prefix while the existing offline workflow packages it."""
        with self.lock:
            if session != self.session or not self.rows or self.fault:
                raise ValueError('No exportable frames in this live session')
            revision = len(self.rows)
            if self.exported is None or self.exported[0] != revision:
                output = self.directory/'exports'/f'prefix-{revision:05d}-{secrets.token_hex(4)}'
                run_task(self.directory/'task.json', output)
                atomic_json(output/'live-snapshot.json', dict(session=self.session, accepted_frames=revision,
                    latest_stamp_s=self.rows[-1]['stamp_s'], task_completion='not_asserted',
                    meaning='Export of received frames only. This does not certify task completion or physical accuracy.'))
                sums = read_json(output/'checksums.json')
                sums['live-snapshot.json'] = hashlib.sha256((output/'live-snapshot.json').read_bytes()).hexdigest()
                atomic_json(output/'checksums.json', sums)
                target = output.with_suffix('.zip')
                temporary = target.with_suffix('.zip.tmp')
                with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
                    for path in sorted(output.rglob('*')):
                        if path.is_file(): archive.write(path, path.relative_to(output).as_posix())
                os.replace(temporary, target)
                self.exported = (revision, target)
            return self.exported[1].read_bytes(), f'mooncar-survey-{revision:05d}.zip'
