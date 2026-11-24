"""Offline scoring of the predeclared matched observation task; no control inputs.

Run inside the validation container after both worlds have stopped:
python3 /source/reports/camera_mode_comparison_20261104/analyze.py /evidence/camera-mode-comparison-20261104
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, '/source/ros2_ws/src/mooncar_terrain/scripts')
from terrain_map import TerrainMap


def load(path):
    return json.loads(path.read_text())


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def stats(values):
    a = np.asarray(values, dtype=float)
    return dict(count=len(a), mean=float(a.mean()), p95=float(np.quantile(a, .95)),
                maximum=float(a.max())) if len(a) else None


def roi_metrics(quality, box):
    resolution = quality['resolution']
    cells = [c for c in quality['cells'] if
             box[0] <= (c['ix']+.5)*resolution < box[2] and
             box[1] <= (c['iy']+.5)*resolution < box[3]]
    area = (box[2]-box[0])*(box[3]-box[1])
    return dict(roi_area_m2=area, observed_area_m2=len(cells)*resolution**2,
                observed_fraction=len(cells)*resolution**2/area,
                valid_surface_area_m2=sum(c['fit_state']=='valid' for c in cells)*resolution**2,
                quality_area_m2=sum(c['quality'] for c in cells)*resolution**2,
                mean_observed_quality=float(np.mean([c['quality'] for c in cells])) if cells else None)


def score(root, mode):
    case = root/mode/'case'; result = load(case/'result.json')
    autonomy = result['autonomy']; history = autonomy['history']
    session = root/'sessions'/result['experiment']
    delivery = root/'sessions'/autonomy['evidence_folder']
    truth = rows(case/'observer/truth.jsonl')
    alignment = rows(case/'observer/alignment.jsonl')
    states = rows(case/'states.jsonl')
    events = rows(session/'window-localization.jsonl')
    t = np.array([r['stamp'] for r in truth])
    xy = np.array([r['xyz'][:2] for r in truth])
    yaw = np.unwrap([r['yaw'] for r in truth])
    tilt = np.degrees(np.arccos(np.clip(np.cos([r['roll'] for r in truth])*
                       np.cos([r['pitch'] for r in truth]), -1, 1)))
    start = autonomy['started']+history[0]['started_seconds']
    stop = autonomy['started']+autonomy['elapsed_seconds']
    def physical(left, right):
        selected = np.where((t >= left)&(t <= right))[0]
        if not len(selected):
            return None
        points = xy[selected]; headings = yaw[selected]
        return dict(start_sim=left, end_sim=right, duration_sim_s=right-left,
                    truth_samples=len(selected), path_m=float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
                    net_translation_m=float(np.linalg.norm(points[-1]-points[0])),
                    maximum_drift_m=float(np.linalg.norm(points-points[0], axis=1).max()),
                    yaw_change_deg=math.degrees(headings[-1]-headings[0]),
                    maximum_yaw_change_deg=float(np.degrees(abs(headings-headings[0])).max()),
                    max_tilt_deg=float(tilt[selected].max()), start_xy=points[0].tolist(), end_xy=points[-1].tolist())
    def wall_at(sim):
        valid = [(r['state']['motion'].get('diagnostics', {}).get('sim_stamp'), r['wall']) for r in states]
        valid = [(s,w) for s,w in valid if s is not None]
        return min(valid, key=lambda item: abs(item[0]-sim))[1]
    observation_start = autonomy['started']+history[3]['started_seconds']
    observation_end = autonomy['started']+history[5]['started_seconds'] if len(history)>5 else stop
    commands = [r for r in rows(case/'observer/samples.jsonl') if r.get('kind')=='command' and
                r.get('topic')=='/mooncar/cmd_vel_safe' and observation_start <= r['sim'] <= observation_end]
    actions = []
    for action in history:
        left = autonomy['started']+action['started_seconds']; right = action['end_sim']
        a = dict(action, physical=physical(left, right))
        if action['action'][0]=='move' and a['physical']:
            a['goal_distance_m'] = float(np.linalg.norm(np.array(a['physical']['end_xy'])-action['action'][1:]))
        actions.append(a)
    archive = delivery/'observations.npz'
    with np.load(archive, allow_pickle=False) as data:
        stamps = data['stamps'].copy(); offsets = data['offsets'].copy(); quaternions = data['quaternions'].copy()
    maps = {}; boxes = dict(task=[-1.,-4.,6.,4.], side=[2.5,.75,4.5,3.])
    # Reuse production archive replay, voxelization, sampling and surface-fit rules.
    for label, horizon in [('formal_start',start),('equal_40s',start+40),('task_end',stop)]:
        count = int(np.searchsorted(stamps, horizon, side='right'))
        terrain = TerrainMap.load(archive, until=count)
        quality = terrain.quality(with_surface=True)
        maps[label] = dict(requested_sim=horizon, observed_until_sim=float(stamps[count-1]) if count else None,
                           task_ended_before_horizon=stop<horizon, frames=count,
                           points=int(offsets[count]), rois={name:roi_metrics(quality,box) for name,box in boxes.items()})
        for name, values in maps[label]['rois'].items():
            if label != 'formal_start':
                initial = maps['formal_start']['rois'][name]
                values['observed_gain_m2'] = values['observed_area_m2']-initial['observed_area_m2']
                values['valid_surface_gain_m2'] = values['valid_surface_area_m2']-initial['valid_surface_area_m2']
    camera = []
    for action in history[3:5]:
        left = autonomy['started']+action['started_seconds']; right = action['end_sim']
        selected = np.where((stamps>=left)&(stamps<=right))[0]
        azimuths = []
        for index in selected:
            x,y,z,w = quaternions[index]/np.linalg.norm(quaternions[index])
            azimuths.append(math.degrees(math.atan2(2*(y*z-w*x), 2*(x*z+w*y))))
        camera.append(dict(start_sim=left,end_sim=right,new_frames=len(selected),
                           points=int(sum(offsets[i+1]-offsets[i] for i in selected)),
                           optical_azimuth_deg=azimuths))
    plans = {r['stamp_ns']:r for r in events if r['type']=='plan'}
    published = [r for r in events if r['type']=='navigation_pose_published' and
                 start <= r['stamp_ns']/1e9 <= stop]
    latency = [(r['published_wall_ns']-plans[r['stamp_ns']]['received_wall_ns'])/1e9
               for r in published if r['stamp_ns'] in plans]
    health = [r['state'].get('motion',{}).get('diagnostics',{}).get('laser_localization',{}) for r in states]
    run_end = next((r for r in reversed(events) if r['type']=='run_end'),None)
    formal_plans = [r for r in events if r['type']=='plan' and r.get('stamp_ns') is not None and start<=r['stamp_ns']/1e9<=stop]
    gimbal_feedback = [r['state']['motion']['diagnostics'].get('gimbal_position') for r in states
                      if observation_start <= (r['state']['motion'].get('diagnostics',{}).get('sim_stamp') or 0) <= observation_end]
    gimbal_feedback = [v for v in gimbal_feedback if v is not None]
    rss = [r['localization_rss_kib']/1024 for r in rows(case/'resources.jsonl') if 'localization_rss_kib' in r]
    output = dict(mode=mode, experiment=result['experiment'], outcome=result['outcome'],
        completed_actions=sum(bool(a['success']) for a in history), planned_actions=6,
        software=result['software'], preparation_sim_s=start-autonomy['started'],
        preparation_wall_s=wall_at(start)-wall_at(autonomy['started']),
        task_sim_s=stop-start, task_wall_s_approx=wall_at(stop)-wall_at(start),
        collection_wall_s=result['elapsed_wall_s'], formal_task=physical(start,stop),
        observation=physical(observation_start,observation_end),
        observation_safe_commands=dict(count=len(commands), max_abs_linear=max((abs(r['linear']) for r in commands),default=None),
                                       max_abs_angular=max((abs(r['angular']) for r in commands),default=None)),
        observation_gimbal_feedback_range_rad=dict(minimum=np.min(gimbal_feedback,axis=0).tolist(),
            maximum=np.max(gimbal_feedback,axis=0).tolist()) if gimbal_feedback else None,
        actions=actions, camera=camera, maps=maps,
        formal_xy_error_m=stats([r['xy_error_m'] for r in alignment if start<=r['stamp']<=stop]),
        phase_xy_error_m={name:stats([r['xy_error_m'] for r in alignment if left<=r['stamp']<=right])
            for name,left,right in [('approach',start,observation_start),
                                    ('observation',observation_start,observation_end),
                                    ('continuation',observation_end,stop),('equal_40s',start,start+40)]},
        formal_received_to_publish_wall_s=stats(latency),
        formal_publication_gap_sim_s=stats(np.diff(sorted(r['published_sim_ns']/1e9 for r in published))),
        formal_plan_counts=dict(processed=len(formal_plans), output=sum(bool(r.get('output')) for r in formal_plans),
            published=len(published), failures=dict(Counter(r.get('reason') for r in formal_plans if not r.get('output')))),
        localization_rss_mib=stats(rss), primary_run_end=run_end,
        navigation_rejections=dict(Counter(r.get('reason') for r in events if r['type']=='navigation_pose_rejected')),
        max_health_fault_counter=max((r.get('fault_counter',0) or 0 for r in health),default=0),
        pauses=[r for r in history if r.get('kind')=='pause'],
        observer=load(case/'observer/complete.json'),
        snapshot_read_errors=result['snapshot_read_errors'],
        driver_exit=load(root/mode/'driver-exit.json'), backend_exit=load(root/mode/'backend-exit.json'),
        observer_exit=result['observer_exit'], lifecycle_exit=result['lifecycle_exit'],
        archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(), archive=str(archive))
    (root/mode/'analysis.json').write_text(json.dumps(output,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path); args = parser.parse_args()
    result = dict(protocol='/source/docs/camera_mode_comparison_plan_20261104.md',
                  roi_coordinates='map; initial map gauge follows world yaw and origin; no fitted trajectory alignment',
                  arms=[score(args.root,mode) for mode in ('fixed','gimbal')])
    (args.root/'comparison.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps([{k:a[k] for k in ('mode','outcome','task_sim_s','observation','formal_xy_error_m',
                       'formal_received_to_publish_wall_s')} for a in result['arms']],ensure_ascii=False))
