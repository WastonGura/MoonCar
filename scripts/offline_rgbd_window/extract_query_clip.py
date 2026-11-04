"""Extract one fixed, sensor-only RGBD clip for an offline query comparison.

This is not a ROS node or replay. Image/IMU headers are indexed without loading
their payloads; only clip image payloads and bracketing IMUs are decoded. TF and
CameraInfo metadata may be read outside the clip. No truth or estimated pose is
opened. A failed extraction leaves a complete ledger, never a usable pickle.
"""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
import argparse
import bisect
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import pickle
import sqlite3
import struct
import sys
import time

sys.dont_write_bytecode = True

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'scripts'), str(ROOT / 'ros2_ws/src/mooncar_slam/scripts')]

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer

from rgbd_odometry_node import calibration_signature, depth_grid, gray_image
from terrain_odometry_node import imu_at, outside_rover
from offline_rgbd_window.online import make_sensor_frame

TRIAL = ROOT / 'tmp/ros_acceptance/shadow-throughput-20261022/trial-01'
EXPERIMENT = 'experiment-700e5c41a749'
BAG_BYTES = 287252480
BAG_SHA256 = 'cec74fe2eeed367fc224bed35013e9fe7b43fb649b48f1a09a1ea494b8070bd9'
CLIP = (28_000_000_000, 40_500_000_000)
SCORE_START = 31_346_000_000
INTERVAL = 100_000_000
RGB, DEPTH = '/mooncar/localization_rgb/image', '/mooncar/depth/image'
RI, DI = '/mooncar/localization_rgb/camera_info', '/mooncar/depth/camera_info'
IMU = '/mooncar/imu'
INPUT_SOURCES = (
    'ros2_ws/src/mooncar_slam/scripts/rgbd_odometry_node.py',
    'ros2_ws/src/mooncar_slam/scripts/terrain_odometry_node.py',
    'scripts/offline_rgbd_window/data.py',
    'scripts/offline_rgbd_window/online.py',
)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def write_json(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def cdr_header_stamp(prefix):
    """Read the standard CDR Header stamp; validate against decoded clip rows."""
    if len(prefix) != 12 or prefix[0] != 0 or prefix[1] not in (0, 1):
        raise ValueError('Unsupported CDR encapsulation or truncated Header')
    sec, nanosec = struct.unpack(('<' if prefix[1] else '>') + 'iI', prefix[4:12])
    if sec < 0 or nanosec >= 1_000_000_000:
        raise ValueError('Invalid nonnegative sensor Header stamp')
    return sec * 1_000_000_000 + nanosec


def header_index(connection, topic_ids):
    result = defaultdict(lambda: defaultdict(list))
    placeholders = ','.join('?' for _ in topic_ids)
    query = ('SELECT id,topic_id,timestamp,substr(data,1,12) FROM messages '
             f'WHERE topic_id IN ({placeholders}) ORDER BY timestamp,id')
    for rowid, topic_id, record_ns, prefix in connection.execute(query, topic_ids):
        stamp = cdr_header_stamp(prefix)
        result[topic_id][stamp].append(dict(rowid=rowid, record_ns=record_ns))
    return result


def payload(connection, rowid, message_class, expected_stamp=None):
    row = connection.execute('SELECT data FROM messages WHERE id=?', (rowid,)).fetchone()
    if row is None:
        raise ValueError('Indexed message is missing')
    message = deserialize_message(row[0], message_class)
    if expected_stamp is not None and ns(message) != expected_stamp:
        raise ValueError('CDR Header index disagrees with decoded message')
    return message


def extract(output):
    started, cpu_started = time.perf_counter(), time.process_time()
    output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(2)
    database = TRIAL / 'native-bag/native-bag_0.db3'
    before_stat = database.stat()
    source_sha = {name: sha256(ROOT / name) for name in INPUT_SOURCES}
    source_sha['scripts/offline_rgbd_window/extract_query_clip.py'] = sha256(Path(__file__))
    manifest = dict(schema_version=1, experiment=EXPERIMENT, complete=False,
        data_sufficient=False, clip_ns=list(CLIP), score_start_ns=SCORE_START,
        interval_ns=INTERVAL, keyframe_interval_ns=500_000_000,
        paired_count=0, expected_count=(CLIP[1]-CLIP[0])//INTERVAL+1,
        extracted_frames=0, missing=[], source=dict(bag=str(database),
            shadow=str(TRIAL / 'shadow.jsonl'), result=str(TRIAL / 'result.json')),
        source_sha256=source_sha,
        axes=dict(initial_imu_yaw_rad=0., normalization='Rz(-initial_imu_yaw) times mounted IMU attitude'),
        sensor_contract='All exact recorded RGBD pairs in the fixed clip, including originally replaced exposures. '
            'No odometry, wheel, world geometry, truth, command or baseline pose input. '
            'Same original imu_at(max_gap=.03), CameraInfo, TF, depth and rover mask.',
        replay_scope='Independent local initialization at clip start; not original online full-history reproduction.',
        pickle_contract='sensors.pickle is list[offline_rgbd_window.data.SensorFrame]; present only on strict success.')
    ledger, sensors, problems = [], [], []
    try:
        if before_stat.st_size != BAG_BYTES or sha256(database) != BAG_SHA256:
            raise ValueError('Original sealed bag size or SHA256 differs')
        manifest['source_sha256']['bag'] = BAG_SHA256
        for name in ('shadow.jsonl', 'result.json', 'native-bag/metadata.yaml'):
            manifest['source_sha256'][name] = sha256(TRIAL / name)
        closed = read_json(TRIAL / 'result.json')
        if closed.get('experiment') != EXPERIMENT or closed.get('outcome') != 'route_complete':
            raise ValueError('Fixed recorded experiment is not the completed route')
        if any(closed['cleanup'][key].get('exit_code') != 0
               or closed['cleanup'][key].get('forced', False)
               for key in ('driver', 'shadow', 'observer', 'bag', 'world')):
            raise ValueError('Recorded capture did not close normally')
        native = defaultdict(list)
        with (TRIAL / 'shadow.jsonl').open(encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                if row.get('type') == 'imu_reference':
                    if row.get('initial_imu_yaw_rad') != 0.:
                        raise ValueError('Recorded initial IMU yaw is not the fixed zero reference')
                    manifest['native_imu_reference'] = row
                if row.get('type') == 'exposure' and CLIP[0] <= row['stamp_ns'] <= CLIP[1]:
                    native[row['stamp_ns']].append(row)
        if 'native_imu_reference' not in manifest:
            raise ValueError('Recorded initial IMU reference is absent')
        with sqlite3.connect(database.resolve().as_uri() + '?mode=ro', uri=True) as connection:
            connection.execute('PRAGMA query_only=ON')
            topics = {key: (name, get_message(kind)) for key, name, kind in
                      connection.execute('SELECT id,name,type FROM topics')}
            by_name = {name: key for key, (name, _) in topics.items()}
            for name in (RGB, DEPTH, RI, DI, IMU, '/tf', '/tf_static'):
                if name not in by_name:
                    raise ValueError('Required bag topic absent: ' + name)
            index = header_index(connection, [by_name[RGB], by_name[DEPTH], by_name[IMU]])
            image_index = {name: {stamp: rows for stamp, rows in index[by_name[name]].items()
                                 if CLIP[0] <= stamp <= CLIP[1]} for name in (RGB, DEPTH)}
            expected = set(range(CLIP[0], CLIP[1]+1, INTERVAL))
            actual = set(image_index[RGB]) | set(image_index[DEPTH])
            paired = set(image_index[RGB]) & set(image_index[DEPTH])
            manifest.update(paired_count=len(paired), rgb_count=len(image_index[RGB]),
                            depth_count=len(image_index[DEPTH]),
                            unexpected_stamps_ns=sorted(actual-expected),
                            originally_processed_count=sum(any(r.get('disposition') == 'processed'
                                for r in rows) for rows in native.values()),
                            extraction_selection='All recorded exact pairs; never filtered by native processed mask.')
            buffer = Buffer(cache_time=Duration(seconds=120))
            infos = defaultdict(dict)
            metadata_counts = Counter()
            small_ids = [by_name[name] for name in ('/tf', '/tf_static', RI, DI)]
            placeholders = ','.join('?' for _ in small_ids)
            sql = ('SELECT topic_id,data FROM messages WHERE topic_id IN (' + placeholders +
                   ') ORDER BY timestamp,id')
            for topic_id, blob in connection.execute(sql, small_ids):
                name, cls = topics[topic_id]
                message = deserialize_message(blob, cls)
                metadata_counts[name] += 1
                if name in ('/tf', '/tf_static'):
                    for transform in message.transforms:
                        (buffer.set_transform_static if name == '/tf_static' else buffer.set_transform)(
                            transform, 'fixed_query_clip_extraction')
                else:
                    signature = calibration_signature(message)
                    stamp = ns(message)
                    if stamp in infos[name] and infos[name][stamp][1] != signature:
                        raise ValueError('Conflicting same-stamp CameraInfo: ' + name)
                    infos[name][stamp] = (message, signature)
            info_times = {name: sorted(infos[name]) for name in (RI, DI)}
            imus, imu_metadata = [], []
            for stamp, rows in sorted(index[by_name[IMU]].items()):
                if not CLIP[0]-60_000_000 <= stamp <= CLIP[1]+60_000_000:
                    continue
                if len(rows) != 1:
                    raise ValueError('Duplicate clip IMU timestamp: ' + str(stamp))
                message = payload(connection, rows[0]['rowid'], topics[by_name[IMU]][1], stamp)
                transform = buffer.lookup_transform('base_footprint', message.header.frame_id,
                                                    Time(nanoseconds=stamp))
                q, mount_q = message.orientation, transform.transform.rotation
                mount = Rotation.from_quat([mount_q.x, mount_q.y, mount_q.z, mount_q.w])
                body = Rotation.from_quat([q.x, q.y, q.z, q.w]) * mount.inv()
                normalized = Rotation.from_rotvec([0., 0., -0.]) * body
                rate = mount.apply([message.angular_velocity.x, message.angular_velocity.y,
                                    message.angular_velocity.z])
                imus.append((stamp/1e9, normalized.as_quat(), float(rate[2])))
                imu_metadata.append(dict(stamp_ns=stamp, message_id=rows[0]['rowid']))
            imu_times = [round(row[0]*1e9) for row in imus]
            manifest['decoded_metadata_counts'] = dict(metadata_counts)
            manifest['decoded_imu_count'] = len(imus)
            manifest['imu_bounds_ns'] = [imu_times[0], imu_times[-1]] if imus else None
            for stamp in sorted(expected | actual | set(native)):
                row = dict(type='input', stamp_ns=stamp, planned_stamp_ns=stamp,
                    score=stamp >= SCORE_START, status='missing', sensor_index=None,
                    rgb_message_ids=[r['rowid'] for r in image_index[RGB].get(stamp, [])],
                    depth_message_ids=[r['rowid'] for r in image_index[DEPTH].get(stamp, [])],
                    source_dispositions=[r.get('disposition') for r in native[stamp]],
                    source_planned_stamps_ns=sorted({r['planned_stamp_ns'] for r in native[stamp]
                                                    if r.get('planned_stamp_ns') is not None}),
                    errors=[])
                try:
                    if stamp not in expected:
                        raise ValueError('Unexpected exposure outside the fixed 100ms input grid')
                    if len(row['rgb_message_ids']) != 1 or len(row['depth_message_ids']) != 1:
                        raise ValueError('Missing or duplicated exact RGB/depth pair')
                    if not any(r.get('disposition') == 'queued' for r in native[stamp]):
                        raise ValueError('Recorded pair is absent from native exposure ledger')
                    rgb = payload(connection, row['rgb_message_ids'][0], topics[by_name[RGB]][1], stamp)
                    depth = payload(connection, row['depth_message_ids'][0], topics[by_name[DEPTH]][1], stamp)
                    camera_info = []
                    for name in (RI, DI):
                        at = bisect.bisect_right(info_times[name], stamp)-1
                        if at < 0:
                            raise ValueError('No real prior CameraInfo: ' + name)
                        camera_info.append(infos[name][info_times[name][at]][0])
                    if calibration_signature(camera_info[0]) != calibration_signature(camera_info[1]):
                        raise ValueError('RGB/depth CameraInfo signatures differ')
                    info = camera_info[1]
                    if (rgb.header.frame_id != depth.header.frame_id or depth.header.frame_id != info.header.frame_id
                            or (rgb.width, rgb.height) != (depth.width, depth.height)
                            or (depth.width, depth.height) != (info.width, info.height)):
                        raise ValueError('Registered RGB/depth image frame or shape differs')
                    at = bisect.bisect_left(imu_times, stamp)
                    row['imu_bracket_ns'] = [imu_times[max(0, at-1)], imu_times[min(at, len(imu_times)-1)]] if imus else []
                    attitude = imu_at(imus, stamp/1e9)
                    if attitude is None:
                        raise ValueError('No real IMU bracket satisfying original max_gap=.03')
                    transform = buffer.lookup_transform('base_footprint', depth.header.frame_id,
                                                        Time(nanoseconds=stamp))
                    q, t = transform.transform.rotation, transform.transform.translation
                    K = np.asarray(info.k).reshape(3, 3)
                    optical = depth_grid(depth, (K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
                    sensor = make_sensor_frame(stamp, gray_image(rgb), optical, attitude[0].as_matrix(),
                        Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
                        np.array([t.x, t.y, t.z]), K, outside_rover)
                    row.update(status='extracted', sensor_index=len(sensors),
                               camera_info_stamp_ns=[ns(value) for value in camera_info],
                               camera_tf_stamp_ns=ns(transform))
                    sensors.append(sensor)
                except Exception as error:
                    row['errors'].append(type(error).__name__ + ': ' + str(error))
                    problems.append(dict(stamp_ns=stamp, errors=row['errors']))
                ledger.append(row)
        if (database.stat().st_size, database.stat().st_mtime_ns) != (before_stat.st_size, before_stat.st_mtime_ns):
            raise ValueError('Read-only bag bytes/mtime changed')
        if any(sha256(ROOT / name) != source_sha[name] for name in INPUT_SOURCES):
            raise ValueError('Extraction input helper sources changed')
        if problems or len(sensors) != manifest['expected_count']:
            manifest.update(missing=problems, available_sensor_count=len(sensors), phase='data_insufficient')
        else:
            destination = output / 'sensors.pickle'
            with destination.open('xb') as stream:
                pickle.dump(sensors, stream, protocol=5)
            manifest.update(complete=True, data_sufficient=True, phase='complete',
                extracted_frames=len(sensors), sensor_stamps_ns=[sensor.stamp_ns for sensor in sensors],
                sensors_file=destination.name, sensors_sha256=sha256(destination))
    except Exception as error:
        manifest.update(phase='data_insufficient', error=type(error).__name__ + ': ' + str(error),
                        missing=problems, available_sensor_count=len(sensors))
    with (output / 'ledger.jsonl').open('x', encoding='utf-8') as stream:
        for row in ledger:
            stream.write(json.dumps(row, allow_nan=False) + '\n')
    manifest.update(ledger_sha256=sha256(output / 'ledger.jsonl'),
        elapsed_wall_s=time.perf_counter()-started, process_cpu_s=time.process_time()-cpu_started,
        bag_bytes_mtime_unchanged=(database.stat().st_size, database.stat().st_mtime_ns)
            == (before_stat.st_size, before_stat.st_mtime_ns))
    write_json(output / 'manifest.json', manifest)
    print(json.dumps({key: manifest.get(key) for key in ('phase', 'complete', 'data_sufficient',
        'paired_count', 'expected_count', 'extracted_frames', 'available_sensor_count', 'error')}), flush=True)
    return 0 if manifest['data_sufficient'] else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        default=Path('/evidence/query-cache-offline-20261023/input-01'))
    args = parser.parse_args()
    return extract(args.output)


if __name__ == '__main__':
    raise SystemExit(main())
