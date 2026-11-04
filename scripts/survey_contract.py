"""Hardware-free survey contracts. Coordinates are gravity-aligned local metres."""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/mooncar_terrain/scripts'))
from terrain_geometry import project_depth
from terrain_map import TerrainMap

TASK_FORMAT = 'mooncar-survey-task-v1'
INPUT_FORMAT = 'mooncar-survey-input-v1'
MAX_CELLS = 100000


def read_json(path):
    path = Path(path)
    if path.stat().st_size > 20_000_000:
        raise ValueError('JSON input exceeds 20 MB')
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('Duplicate JSON key: ' + key)
            result[key] = value
        return result
    def invalid(value):
        raise ValueError('Non-finite JSON number: ' + value)
    return json.loads(path.read_text(encoding='utf-8-sig'), object_pairs_hook=unique,
                      parse_constant=invalid)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def number(value, label, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(label + ' must be a finite ' + ('positive number' if positive else 'number'))
    return float(value)


def named(value, label):
    if not isinstance(value, str) or not value.strip() or len(value) > 1000:
        raise ValueError(label + ' must be a nonempty string of at most 1000 characters')
    return value


def keys(value, required, optional=()):
    if not isinstance(value, dict) or set(value) - set(required) - set(optional) or set(required) - set(value):
        raise ValueError('Expected fields: ' + ', '.join(required) + '; optional: ' + ', '.join(optional))


def grid_indices(task):
    x0, y0, x1, y1 = task['region']['bounds_m']
    r = task['grid']['cell_m']
    # Half-open region, membership defined by existing grid cell centres.
    return (math.ceil(x0 / r - .5), math.ceil(y0 / r - .5),
            math.ceil(x1 / r - .5), math.ceil(y1 / r - .5))


def validate_task(task):
    keys(task, ('format', 'id', 'coordinates', 'region', 'time', 'grid', 'targets', 'source', 'input'),
         ('reference', 'configuration'))
    if task['format'] != TASK_FORMAT:
        raise ValueError('Unsupported survey task format')
    named(task['id'], 'task id')
    c = task['coordinates']
    keys(c, ('frame', 'unit', 'axes', 'vertical_datum'))
    named(c['frame'], 'frame'); named(c['vertical_datum'], 'vertical datum')
    if c['unit'] != 'm' or c['axes'] != 'ENU':
        raise ValueError('Only gravity-aligned ENU metres are supported')
    region = task['region']; keys(region, ('type', 'bounds_m'))
    if region['type'] != 'rectangle' or not isinstance(region['bounds_m'], list) or len(region['bounds_m']) != 4:
        raise ValueError('v1 requires a rectangle with four bounds; no implicit shape conversion')
    bounds = [number(x, 'region bound') for x in region['bounds_m']]
    if max(map(abs, bounds)) > 10000 or bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
        raise ValueError('Invalid region bounds')
    time = task['time']; keys(time, ('clock', 'start_s', 'end_s'))
    if time['clock'] not in ('simulation', 'monotonic', 'utc'):
        raise ValueError('Explicit simulation, monotonic or utc clock required')
    if number(time['start_s'], 'start') < 0 or number(time['end_s'], 'end') <= time['start_s']:
        raise ValueError('Invalid absolute survey time window')
    g = task['grid']; keys(g, ('cell_m', 'voxel_m'))
    cell = number(g['cell_m'], 'cell size', True); voxel = number(g['voxel_m'], 'voxel size', True)
    if voxel > cell / 2:
        raise ValueError('Voxel size must be at most half a cell')
    ix0, iy0, ix1, iy1 = grid_indices(task)
    if not 0 < (ix1 - ix0) * (iy1 - iy0) <= MAX_CELLS or ix0 >= ix1 or iy0 >= iy1:
        raise ValueError('Task must contain 1..100000 cell centres')
    targets = task['targets']
    keys(targets, ('height_tolerance_m', 'slope_tolerance_deg', 'minimum_precision', 'status', 'basis'), ('minimum_coverage',))
    for key in ('height_tolerance_m', 'slope_tolerance_deg', 'minimum_precision'):
        number(targets[key], key, True)
    if targets['slope_tolerance_deg'] > 90 or targets['minimum_precision'] > 1:
        raise ValueError('Invalid target degrees/fractions')
    # Historical tasks retain their signed content; this field no longer gates results.
    if 'minimum_coverage' in targets:
        if not 0 < number(targets['minimum_coverage'], 'legacy minimum_coverage') <= 1:
            raise ValueError('Invalid legacy coverage fraction')
    if targets['status'] not in ('provisional', 'task_specific'):
        raise ValueError('Target status must be provisional or task_specific')
    named(targets['basis'], 'target basis')
    source = task['source']; keys(source, ('kind', 'description', 'calibration_version', 'pose_version'))
    if source['kind'] not in ('synthetic', 'gazebo', 'physical', 'unknown'):
        raise ValueError('Explicit source kind required')
    for key in ('description', 'calibration_version', 'pose_version'):
        named(source[key], key)
    inp = task['input']; keys(inp, ('format', 'path'), ('contract',))
    if inp['format'] not in ('observations-v2', 'folder-v1'):
        raise ValueError('Unsupported input adapter')
    named(inp['path'], 'input path')
    if inp['format'] == 'observations-v2' and 'contract' not in inp:
        raise ValueError('Existing NPZ requires an explicit input contract sidecar')
    if 'contract' in inp: named(inp['contract'], 'contract path')
    for key in ('reference', 'configuration'):
        if key in task:
            keys(task[key], ('path',)); named(task[key]['path'], key + ' path')
    return task


def validate_input_contract(contract, task):
    keys(contract, ('format', 'frame', 'unit', 'axes', 'vertical_datum', 'clock', 'source_kind', 'calibration_version', 'pose_version'))
    if contract['format'] != INPUT_FORMAT:
        raise ValueError('Unsupported input contract')
    expected = dict(task['coordinates'], clock=task['time']['clock'], source_kind=task['source']['kind'],
                    calibration_version=task['source']['calibration_version'], pose_version=task['source']['pose_version'])
    for key, value in expected.items():
        if contract.get(key) != value:
            raise ValueError('Input/task contract mismatch: ' + key)
    return contract


def local_file(base, value):
    if not isinstance(value, str) or Path(value).is_absolute():
        raise ValueError('Folder references must be relative files')
    result = (base / value).resolve()
    if not result.is_relative_to(base.resolve()) or not result.is_file():
        raise ValueError('Missing or escaping folder input: ' + value)
    return result


@dataclass
class Frame:
    stamp: float
    points: np.ndarray
    translation: np.ndarray
    quaternion: np.ndarray
    robot: object = None
    note: str = ''


def pose(value):
    keys(value, ('translation_m', 'quaternion_xyzw'))
    t = np.asarray(value['translation_m'], dtype=float); q = np.asarray(value['quaternion_xyzw'], dtype=float)
    if t.shape != (3,) or q.shape != (4,) or not np.isfinite(t).all() or not np.isfinite(q).all():
        raise ValueError('Invalid sensor-to-session pose')
    if max(abs(t)) > 10000 or abs(np.linalg.norm(q) - 1) > .001:
        raise ValueError('Pose requires bounded metres and a unit quaternion')
    return t, q


def load_input(task, task_path):
    base = Path(task_path).resolve().parent
    inp = task['input']; source = (base / inp['path']).resolve()
    files = [Path(task_path).resolve()]
    if inp['format'] == 'observations-v2':
        sidecar = (base / inp['contract']).resolve()
        contract = validate_input_contract(read_json(sidecar), task)
        meta = TerrainMap.inspect(source)
        if meta['format'] != 'mooncar-observations-v2' or meta['frame'] != contract['frame']:
            raise ValueError('NPZ frame/format differs from declared contract')
        # Existing API performs full archive validation, including frames after cutoff.
        model = TerrainMap.load(source)
        frames = [Frame(stamp, points, t, q, robot) for (stamp, points, t, q), robot in zip(model.frames, model.robot_poses)]
        files += [source, sidecar]
    else:
        if not source.is_dir(): raise ValueError('folder-v1 input must be a directory')
        manifest_path = source / 'manifest.json'; manifest = read_json(manifest_path)
        keys(manifest, ('contract', 'frames'), ('camera',))
        contract = validate_input_contract(manifest['contract'], task)
        if not isinstance(manifest['frames'], list) or len(manifest['frames']) > 5000:
            raise ValueError('At most 5000 frames supported')
        frames = []; files.append(manifest_path); total_points = 0
        for row in manifest['frames']:
            keys(row, ('stamp_s', 'pose'), ('points_file', 'depth_file', 'robot_pose'))
            if ('points_file' in row) == ('depth_file' in row):
                raise ValueError('Exactly one points_file or depth_file required')
            t, q = pose(row['pose']); file = local_file(source, row.get('points_file', row.get('depth_file')))
            if file.suffix != '.npy' or file.stat().st_size > 150_000_000:
                raise ValueError('Bounded, non-pickle .npy arrays required')
            value = np.load(file, allow_pickle=False, mmap_mode='r'); files.append(file)
            if not isinstance(value, np.ndarray):
                if hasattr(value, 'close'): value.close()
                raise ValueError('Expected a plain .npy array, not an archive')
            note = ''
            if 'depth_file' in row:
                camera = manifest.get('camera'); keys(camera, ('width', 'height', 'encoding', 'intrinsics', 'pixel_offset', 'rectified'))
                if camera['rectified'] is not True:
                    raise ValueError('Only rectified pinhole depth supported')
                w, h = camera['width'], camera['height']
                if type(w) is not int or type(h) is not int or not 0 < w * h <= 4_000_000:
                    raise ValueError('Invalid depth dimensions')
                expected_dtype = {'32FC1': np.dtype('float32'), '16UC1': np.dtype('uint16')}.get(camera['encoding'])
                if value.shape != (h, w) or expected_dtype is None or value.dtype != expected_dtype:
                    raise ValueError('Depth array does not match declared dimensions/encoding')
                value = np.ascontiguousarray(value)
                value = project_depth(value.tobytes(), w, h, value.strides[0], camera['encoding'], False,
                                      camera['intrinsics'], pixel_offset=camera['pixel_offset'])
                note = 'Baseline stride=4; valid optical depth 0.1..6 m; other pixels remain missing'
            else:
                if value.ndim != 2 or value.shape[1] != 3 or value.dtype.kind not in 'fi':
                    raise ValueError('Points require an N x 3 numeric array in sensor metres')
            if len(value) > 100000:
                raise ValueError('Invalid or oversized sensor points')
            total_points += len(value)
            if total_points > 5_000_000:
                raise ValueError('Input exceeds five million raw points')
            value = np.asarray(value, dtype=float)
            if len(value) > 100000 or not np.isfinite(value).all() or (value.size and np.max(np.abs(value)) > 10000):
                raise ValueError('Invalid or oversized sensor points')
            robot = None
            if 'robot_pose' in row:
                rt, rq = pose(row['robot_pose']); robot = np.concatenate((rt, rq))
            frames.append(Frame(number(row['stamp_s'], 'frame timestamp'), value, t, q, robot, note))
    stamps = [f.stamp for f in frames]
    if any(s < 0 for s in stamps) or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('All timestamps must strictly increase in the declared clock')
    if sum(len(f.points) for f in frames) > 5_000_000:
        raise ValueError('Input exceeds five million raw points')
    return frames, contract, list(dict.fromkeys(files)), source
