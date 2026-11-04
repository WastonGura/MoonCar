"""Read frozen RGBD chunks without feeding evaluation truth to the estimator.

``xyz_grid`` already uses the producer's nominal IMU attitude.  ``optical_grid``
reconstructs the same measured pixels in optical axes; these are alternative
representations of one measurement, never independent observations.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Iterator

import numpy as np


def _rotation(value, name):
    value = np.asarray(value)
    if (value.shape != (3, 3) or not np.isfinite(value).all()
            or not np.allclose(value.T @ value, np.eye(3), atol=1e-7, rtol=0.)
            or not math.isclose(float(np.linalg.det(value)), 1., abs_tol=1e-7)):
        raise ValueError(f'{name} must be a finite proper rotation')


def _integer(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise ValueError(f'{name} must be an integer')
    return int(value)


@dataclass(frozen=True)
class SensorFrame:
    stamp_ns: int
    gray: np.ndarray
    xyz_grid: np.ndarray
    depth_raw: np.ndarray
    nominal_R_wb: np.ndarray
    R_bc: np.ndarray
    t_bc: np.ndarray
    K: np.ndarray
    wheel_vx: float | None
    wheel_wz: float | None

    def __post_init__(self):
        if _integer(self.stamp_ns, 'stamp_ns') < 0:
            raise ValueError('stamp_ns must be nonnegative')
        if self.gray.ndim != 2 or self.gray.dtype != np.uint8 or min(self.gray.shape) <= 0:
            raise ValueError('gray must be a nonempty uint8 image')
        if self.xyz_grid.shape != (*self.gray.shape, 3) or self.xyz_grid.dtype.kind != 'f':
            raise ValueError('xyz_grid must be a floating HxWx3 image')
        if np.isinf(self.xyz_grid).any():
            raise ValueError('xyz_grid may contain NaN, never infinity')
        finite = np.isfinite(self.xyz_grid)
        if np.any(finite.any(axis=2) != finite.all(axis=2)):
            raise ValueError('unknown xyz_grid pixels must have three NaN components')
        if self.depth_raw.shape != self.gray.shape or not (
                self.depth_raw.dtype.kind == 'f' or self.depth_raw.dtype == np.dtype('uint16')):
            raise ValueError('depth_raw must be a matching metric float or uint16 millimetre image')
        _rotation(self.nominal_R_wb, 'nominal_R_wb')
        _rotation(self.R_bc, 'R_bc')
        if np.shape(self.t_bc) != (3,) or not np.isfinite(self.t_bc).all():
            raise ValueError('t_bc must be a finite translation')
        if (np.shape(self.K) != (3, 3) or not np.isfinite(self.K).all()
                or min(self.K[0, 0], self.K[1, 1]) <= 0
                or not np.allclose(self.K[2], [0., 0., 1.], atol=1e-9, rtol=0.)
                or abs(self.K[0, 1]) + abs(self.K[1, 0]) > 1e-9):
            raise ValueError('K must describe the original rectified pixel grid')
        if (self.wheel_vx is None) != (self.wheel_wz is None):
            raise ValueError('wheel hints must both be available or both be None')
        if self.wheel_vx is not None and not np.isfinite([self.wheel_vx, self.wheel_wz]).all():
            raise ValueError('available wheel hints must be finite')

    @property
    def stamp(self) -> float:
        return self.stamp_ns / 1e9

    @property
    def yaw(self) -> float:
        return math.atan2(float(self.nominal_R_wb[1, 0]), float(self.nominal_R_wb[0, 0]))

    @property
    def camera_model(self) -> dict:
        return dict(K=self.K, rotation=self.nominal_R_wb @ self.R_bc,
                    translation=self.nominal_R_wb @ self.t_bc)

    @property
    def optical_grid(self) -> np.ndarray:
        """Original depth in optical axes with the producer's exact unknown mask.

        Float depth is metres; recorded uint16 depth is millimetres.  No masked
        rover pixels, rejected returns or missing depth are restored.
        """
        z = self.depth_raw.astype(np.float64)
        if self.depth_raw.dtype == np.dtype('uint16'):
            z *= .001
        valid = np.isfinite(self.xyz_grid).all(axis=2)
        valid &= np.isfinite(z) & (z >= .1) & (z <= 6.)
        v, u = np.indices(z.shape)
        points = np.stack(((u + .5 - self.K[0, 2]) * z / self.K[0, 0],
                           (v + .5 - self.K[1, 2]) * z / self.K[1, 1], z), axis=2)
        points[~valid] = np.nan
        return points


def sha256_file(path) -> str:
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _read_manifest(path):
    path = Path(path)
    if path.is_dir():
        path /= 'manifest.json'
    with path.open(encoding='utf-8-sig') as stream:
        manifest = json.load(stream)
    if not isinstance(manifest, dict) or manifest.get('complete') is not True:
        raise ValueError('a complete frozen sequence manifest is required')
    chunks = manifest.get('chunks')
    if not isinstance(chunks, list) or not chunks:
        raise ValueError('manifest must contain nonempty chunks')
    return path, manifest


def _chunk_path(manifest_path, filename):
    if not isinstance(filename, str) or not filename:
        raise ValueError('chunk filename must be a nonempty string')
    root = manifest_path.parent.resolve()
    path = (root / filename).resolve()
    if not path.is_relative_to(root):
        raise ValueError('chunk filename escapes its sequence directory')
    return path


def describe_manifest(path) -> dict:
    path, manifest = _read_manifest(path)
    chunks = manifest['chunks']
    count = sum(_integer(chunk['count'], 'chunk count') for chunk in chunks)
    first = int(chunks[0]['first_stamp_ns'])
    last = int(chunks[-1]['last_stamp_ns'])
    return dict(path=str(path.resolve()), sha256=sha256_file(path), case=manifest.get('case'),
                experiment=manifest.get('experiment'), count=count, chunks=len(chunks),
                first_stamp_ns=first, last_stamp_ns=last, duration_s=(last - first) / 1e9,
                omission_count=manifest.get('omission_count'),
                source_bindings=dict(manifest.get('source_bindings', {})))


def iter_sensor_frames(manifest_path, *, verify_hashes=False) -> Iterator[SensorFrame]:
    """Yield all legal frames in manifest order, loading only one NPZ chunk.

    Arrays are read-only views of that chunk; retaining frames can retain their
    backing chunk.  Do not collect a whole sequence merely to iterate it.
    Missing raw exposures remain missing; no recorded failed frame is filtered.
    ``verify_hashes`` optionally checks each original NPZ before reading it.
    """
    path, manifest = _read_manifest(manifest_path)
    previous_stamp = None
    previous_shape = None
    previous_K = None
    total = 0
    required = ('gray', 'xyz_grid', 'depth_raw', 'stamp_ns', 'body_rotation',
                'camera_tf_matrix', 'camera_K')
    optional = ('stamp', 'yaw', 'camera_rotation', 'camera_translation', 'wheel', 'wheel_available')
    for chunk_index, chunk in enumerate(manifest['chunks']):
        count = _integer(chunk['count'], 'chunk count')
        if count <= 0 or ('index' in chunk and chunk['index'] != chunk_index):
            raise ValueError('invalid chunk count or ordering')
        if 'first_global_index' in chunk and chunk['first_global_index'] != total:
            raise ValueError('noncontiguous chunk global indices')
        if 'last_global_index' in chunk and chunk['last_global_index'] != total + count - 1:
            raise ValueError('inconsistent chunk global indices')
        source = _chunk_path(path, chunk['frames_file'])
        if verify_hashes and sha256_file(source) != chunk.get('frames_sha256'):
            raise ValueError(f'chunk SHA256 mismatch: {source.name}')
        with np.load(source, allow_pickle=False) as archive:
            if any(name not in archive for name in required):
                raise ValueError(f'missing sensor arrays in {source.name}')
            arrays = {name: archive[name] for name in (*required, *optional) if name in archive}
        if any(value.ndim == 0 or value.shape[0] != count for value in arrays.values()):
            raise ValueError('chunk sensor arrays have inconsistent frame counts')
        if arrays['stamp_ns'].shape != (count,) or arrays['stamp_ns'].dtype.kind not in 'iu':
            raise ValueError('stamp_ns must be a one-dimensional integer array')
        if ('wheel' in arrays) != ('wheel_available' in arrays):
            raise ValueError('wheel and wheel_available must be recorded together')
        for value in arrays.values():
            value.setflags(write=False)
        for index in range(count):
            stamp_ns = _integer(arrays['stamp_ns'][index], 'stamp_ns')
            if previous_stamp is not None and stamp_ns <= previous_stamp:
                raise ValueError('sensor timestamps must strictly increase across chunks')
            if index == 0 and stamp_ns != chunk['first_stamp_ns']:
                raise ValueError('first chunk timestamp differs from manifest')
            if index == count - 1 and stamp_ns != chunk['last_stamp_ns']:
                raise ValueError('last chunk timestamp differs from manifest')
            if 'stamp' in arrays and not math.isclose(float(arrays['stamp'][index]), stamp_ns / 1e9,
                                                      abs_tol=1e-9, rel_tol=0.):
                raise ValueError('float and integer exposure timestamps disagree')
            tf = arrays['camera_tf_matrix'][index]
            if (tf.shape != (4, 4) or not np.isfinite(tf).all()
                    or not np.allclose(tf[3], [0., 0., 0., 1.], atol=1e-9, rtol=0.)):
                raise ValueError('camera_tf_matrix must be a homogeneous transform')
            vx = wz = None
            if 'wheel' in arrays:
                if arrays['wheel_available'].shape != (count,) or arrays['wheel_available'].dtype.kind != 'b':
                    raise ValueError('wheel_available must be a boolean frame array')
                if arrays['wheel'].shape != (count, 2):
                    raise ValueError('wheel must contain exactly vx and wz')
                if arrays['wheel_available'][index]:
                    vx, wz = map(float, arrays['wheel'][index])
            frame = SensorFrame(stamp_ns, arrays['gray'][index], arrays['xyz_grid'][index],
                arrays['depth_raw'][index], arrays['body_rotation'][index], tf[:3, :3], tf[:3, 3],
                arrays['camera_K'][index], vx, wz)
            if previous_shape is not None and (frame.gray.shape != previous_shape
                    or not np.array_equal(frame.K, previous_K)):
                raise ValueError('image shape or calibration changed within the frozen sequence')
            model = frame.camera_model
            for key, expected in (('camera_rotation', model['rotation']),
                                  ('camera_translation', model['translation'])):
                if key in arrays and (arrays[key][index].shape != expected.shape
                        or not np.allclose(arrays[key][index], expected, atol=1e-8, rtol=0.)):
                    raise ValueError(f'{key} conflicts with the exposure body/camera transform')
            if 'yaw' in arrays:
                yaw = float(arrays['yaw'][index])
                error = math.atan2(math.sin(yaw - frame.yaw), math.cos(yaw - frame.yaw))
                if not math.isfinite(error) or abs(error) > 1e-8:
                    raise ValueError('yaw conflicts with nominal_R_wb')
            valid = np.isfinite(frame.xyz_grid).all(axis=2)
            optical = frame.optical_grid
            if not np.isfinite(optical[valid]).all():
                raise ValueError('finite xyz_grid contains invalid metric depth')
            expected = (optical[valid] @ frame.R_bc.T + frame.t_bc) @ frame.nominal_R_wb.T
            if not np.allclose(frame.xyz_grid[valid], expected, atol=1e-7, rtol=1e-7):
                raise ValueError('xyz_grid conflicts with raw depth, calibration or exposure transforms')
            previous_stamp, previous_shape, previous_K = stamp_ns, frame.gray.shape, frame.K
            total += 1
            yield frame
        # Release this iteration's backing arrays before opening the next chunk.
        del arrays
    if 'extracted_frames' in manifest and total != manifest['extracted_frames']:
        raise ValueError('manifest extracted_frames differs from yielded frame count')


def _truth_values(truth):
    stamps, positions = [], []
    for row in truth:
        if not isinstance(row, dict):
            raise ValueError('truth rows must be dictionaries')
        stamp_ns = row.get('stamp_ns')
        if stamp_ns is None:
            stamp = row.get('stamp')
            if type(stamp) not in (int, float) or not math.isfinite(stamp):
                raise ValueError('truth timestamp is missing or invalid')
            stamp_ns = round(stamp * 1e9)
        stamp_ns = _integer(stamp_ns, 'truth stamp_ns')
        xyz = np.asarray(row.get('xyz'), dtype=float)
        if stamp_ns < 0 or xyz.shape != (3,) or not np.isfinite(xyz).all():
            raise ValueError('truth requires a nonnegative timestamp and finite XYZ')
        if stamps and stamp_ns <= stamps[-1]:
            raise ValueError('truth timestamps must strictly increase')
        stamps.append(stamp_ns)
        positions.append(xyz)
    return np.asarray(stamps, dtype=np.int64), np.asarray(positions, dtype=float).reshape(-1, 3)


def load_truth(path) -> list:
    """Read evaluation-only raw observer JSONL; retain all recorded rows."""
    with Path(path).open(encoding='utf-8-sig') as stream:
        rows = [json.loads(line) for line in stream if line.strip()]
    _truth_values(rows)
    return rows


def interpolate_truth(truth, stamps_ns, *, max_gap=.1):
    """Return evaluation XYZ and validity mask, without any extrapolation.

    Exact recorded samples remain valid even beside a recording gap; an
    interpolated sample requires both bracketing rows at most max_gap apart.
    Missing/unbracketed positions are NaN, never a fitted or held pose.
    """
    if type(max_gap) not in (int, float) or not math.isfinite(max_gap) or max_gap <= 0:
        raise ValueError('max_gap must be finite and positive seconds')
    query = np.asarray(stamps_ns)
    if query.ndim != 1 or (query.size and query.dtype.kind not in 'iu'):
        raise ValueError('query stamps_ns must be a one-dimensional integer array')
    query = query.astype(np.int64, copy=False)
    stamps, positions = _truth_values(truth)
    output = np.full((len(query), 3), np.nan)
    valid = np.zeros(len(query), dtype=bool)
    if not len(stamps):
        return output, valid
    right = np.searchsorted(stamps, query)
    for index, (stamp, end) in enumerate(zip(query, right)):
        if end < len(stamps) and stamp == stamps[end]:
            output[index], valid[index] = positions[end], True
        elif 0 < end < len(stamps):
            start = end - 1
            gap = int(stamps[end]) - int(stamps[start])
            if gap <= round(max_gap * 1e9):
                fraction = (int(stamp) - int(stamps[start])) / gap
                output[index] = (1. - fraction) * positions[start] + fraction * positions[end]
                valid[index] = True
    return output, valid
