"""Prospective negative-case experiment helper, never an autonomous stop policy."""
import hashlib
import json
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
from resampling_dataset import TerrainMap
from swept_resampling import predict
from survey_evaluation import write_json

# Frozen order, independent of later recovery. No zero-angle synthetic probes.
DELTAS=(.02,-.02,.04,-.04,.08,-.08)


def select(quality,cloud,camera,robot,info,pixel_offset,remaining):
    base=math.atan2(robot[1,0],robot[0,0]); candidates=[]
    for delta in DELTAS:
        yaw=math.atan2(math.sin(base+delta),math.cos(base+delta))
        rows,gate=predict(quality,cloud,camera,robot,yaw,info,pixel_offset,remaining,gate_version='v3')
        candidate=dict(delta=delta,yaw=yaw,gate=gate,deficient_cells=len(rows),
            marginal_cells=sum(r['marginal_prediction'] for r in rows))
        candidates.append(candidate)
        if rows and gate['reason']=='no_modelled_marginal_support':return candidate,candidates
    return None,candidates


def select_live(capture,node,seconds):
    import rclpy
    write_json(capture/'checkpoint-request.json',dict(purpose='pre-action negative-case selection'))
    deadline=time.monotonic()+15
    while not (capture/'checkpoint.json').exists():
        if time.monotonic()>deadline:raise TimeoutError('Sensor-only checkpoint unavailable')
        rclpy.spin_once(node,timeout_sec=.05)
    meta=json.loads((capture/'checkpoint.json').read_text());path=capture/'checkpoint.npz'
    if hashlib.sha256(path.read_bytes()).hexdigest()!=meta['sha256']:raise ValueError('Checkpoint hash mismatch')
    terrain=TerrainMap()
    with np.load(path,allow_pickle=False) as data:
        for i,stamp in enumerate(data['stamp']):
            camera=data['estimated_camera'][i]
            terrain.add_frame(data['points'][data['offsets'][i]:data['offsets'][i+1]],float(stamp),
                camera[:3,3],Rotation.from_matrix(camera[:3,:3]).as_quat())
        robot=data['estimated_robot'][-1]
    selected,candidates=select(terrain.quality(True),terrain.cloud(),camera,robot,
        meta['camera_info'],meta['pixel_offset'],max(0,meta['first_stamp']+seconds-meta['last_stamp']))
    # The backend clock keeps advancing during CPU work; never use a stale ROS clock.
    from compare_survey_strategies import request
    now=request()['motion']['diagnostics']['sim_stamp']
    age=now-meta['last_stamp'];remaining=meta['first_stamp']+seconds-now
    executable=selected is not None and 0<=age<=5 and remaining>=8
    result=dict(selected=selected,candidates=candidates,feature_stamp=meta['last_stamp'],
        completed_sim_stamp=now,feature_age_seconds=age,remaining_seconds=remaining,
        executable=executable,checkpoint_sha256=meta['sha256'],
        reason='selected_negative_probe' if executable else 'no_candidate_or_stale_or_low_budget')
    write_json(capture/'selection.json',result)
    return result
