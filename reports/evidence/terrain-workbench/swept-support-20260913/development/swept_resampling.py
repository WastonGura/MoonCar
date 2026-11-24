"""Offline swept-view support hypothesis; observed points only, never world geometry."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial.transform import Rotation
from arrival_resampling import replay
from resampling_dataset import TerrainMap, ROOT
from survey_evaluation import write_csv, write_json


def swept_poses(camera, robot, yaw, angular_step=.25):
    """Shortest world-Z rotation, including the camera's measured lever arm and tilt."""
    camera, robot = np.asarray(camera, float), np.asarray(robot, float)
    if (camera.shape != (4,4) or robot.shape != (4,4) or
            not np.isfinite(camera).all() or not np.isfinite(robot).all() or
            not math.isfinite(yaw) or abs(yaw)>math.pi or
            not math.isfinite(angular_step) or not .01<=angular_step<=.5):
        raise ValueError('Finite camera/base transforms, yaw within pi and bounded step required')
    for pose in (camera,robot):
        if not np.allclose(pose[3],[0,0,0,1]) or not np.allclose(pose[:3,:3].T@pose[:3,:3],np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(pose[:3,:3]),1):
            raise ValueError('Expected rigid transforms')
    start=math.atan2(robot[1,0],robot[0,0])
    delta=math.atan2(math.sin(yaw-start),math.cos(yaw-start))
    poses=[]
    for a in np.linspace(0,delta,max(1,math.ceil(abs(delta)/angular_step))+1):
        rotation=Rotation.from_euler('z',a).as_matrix(); pose=camera.copy()
        pose[:3,:3]=rotation@camera[:3,:3]
        pose[:3,3]=robot[:3,3]+rotation@(camera[:3,3]-robot[:3,3])
        poses.append(pose)
    return poses, abs(delta)


def rays(info, pixel_offset):
    width,height=info['width'],info['height'];fx,fy,cx,cy=[info['k'][i] for i in (0,4,2,5)]
    if (type(width) is not int or type(height) is not int or not 1<=width<=1920 or not 1<=height<=1080 or
            not all(math.isfinite(x) for x in (fx,fy,cx,cy,pixel_offset)) or min(fx,fy)<=0 or abs(pixel_offset)>1):
        raise ValueError('Invalid recorded camera calibration')
    # Same stride and optical-depth limits as the existing point-cloud producer.
    v,u=np.mgrid[0:height:4,0:width:4]
    return np.column_stack(((u.ravel()+pixel_offset-cx)/fx,(v.ravel()+pixel_offset-cy)/fy,np.ones(u.size)))


def support(points, resolution, voxel):
    """Unique hypothesized voxel support; repeated views do not multiply evidence."""
    points=np.asarray(points,float).reshape(-1,3)
    if not len(points):return dict(points=0,spread_m=0.,can_fit=False)
    _,indices=np.unique(np.floor(points/voxel).astype(np.int64),axis=0,return_index=True)
    points=points[indices]
    design=np.column_stack((points[:,:2]-points[:,:2].mean(axis=0),np.ones(len(points))))
    spread=float(np.linalg.svd(design,compute_uv=False)[-1]/math.sqrt(len(points))) if len(points)>=3 else 0.
    return dict(points=len(points),spread_m=spread,can_fit=len(points)>=6 and spread>=resolution*.05)


def predict(quality, cloud, camera, robot, yaw, info, pixel_offset, remaining, voxel=.05):
    if not math.isfinite(remaining) or remaining<0:raise ValueError('Nonnegative remaining time required')
    resolution=quality['resolution']; cloud=np.asarray(cloud,float)
    if (not math.isfinite(resolution) or resolution<=0 or not 0<voxel<=resolution/2 or
            cloud.ndim!=2 or cloud.shape[1]!=3 or not np.isfinite(cloud).all()):
        raise ValueError('Finite observed XYZ and valid map resolutions required')
    poses,angle=swept_poses(camera,robot,yaw); local=rays(info,pixel_offset)
    directions=[local@pose[:3,:3].T for pose in poses]
    keys=np.floor(cloud[:,:2]/resolution).astype(int); rows=[]
    for cell in quality['cells']:
        if cell['fit_state'] not in ('too_few_voxels','insufficient_xy_spread'):continue
        key=np.array([cell['ix'],cell['iy']]); observed=cloud[np.all(keys==key,axis=1)]
        if not len(observed):raise ValueError('Deficient cell has no observed support')
        height=float(np.median(observed[:,2])); hits=[]
        for pose,d in zip(poses,directions):
            t=np.divide(height-pose[2,3],d[:,2],out=np.zeros(len(d)),where=np.abs(d[:,2])>1e-12)
            patch=pose[:3,3]+t[:,None]*d
            selected=(t>=.1)&(t<=6)&np.all(np.floor(patch[:,:2]/resolution).astype(int)==key,axis=1)
            hits.append(patch[selected])
        hold=support(np.vstack([observed,hits[0]]),resolution,voxel)
        swept=support(np.vstack([observed,*hits]),resolution,voxel)
        # ponytail: median-height patches are unoccluded hypotheses, not invented map
        # samples. No slope/occlusion/noise calibration; keep this evaluator offline.
        rows.append(dict(ix=int(key[0]),iy=int(key[1]),decision_state=cell['fit_state'],
            observed_points=len(observed),hold_visible=bool(len(hits[0])),final_visible=bool(len(hits[-1])),
            swept_visible=any(len(hit)>0 for hit in hits),hold_points=hold['points'],swept_points=swept['points'],
            hold_spread_m=hold['spread_m'],swept_spread_m=swept['spread_m'],
            hold_prediction=hold['can_fit'],swept_prediction=swept['can_fit'],
            marginal_prediction=swept['can_fit'] and not hold['can_fit']))
    seconds=angle/.5+3
    reason=('already_aligned' if angle<=.12 else 'insufficient_budget' if remaining<seconds else
            'try_observation' if any(r['marginal_prediction'] for r in rows) else 'no_modelled_marginal_support')
    return rows,dict(version='swept-support-v2',turn=reason=='try_observation',reason=reason,
        estimated_seconds=seconds,remaining_seconds=remaining,sampled_poses=len(poses),angle_rad=angle,
        meaning='Optimistic support hypothesis, not recovery probability or a validated stopping rule')


def snapshot(capture, cut):
    """Read only the requested observation prefix; truth arrays are never accessed."""
    terrain=TerrainMap()
    with np.load(capture/'samples.npz',allow_pickle=False) as data:
        if not 1<=cut<=len(data['stamp']):raise ValueError('Empty or out-of-bounds prefix')
        points,offsets,poses,stamps=(data[k] for k in ('points','offsets','estimated_camera','stamp'))
        for i in range(cut):
            terrain.add_frame(points[offsets[i]:offsets[i+1]],float(stamps[i]),poses[i,:3,3],Rotation.from_matrix(poses[i,:3,:3]).as_quat())
        return terrain,poses[cut-1],data['estimated_robot'][cut-1]


def report(captures, output):
    output.mkdir(parents=True,exist_ok=False)
    plan=json.loads((captures/'plan.json').read_text()); rows=[]; trials=[]; hashes={}
    for trial in plan['trials']:
        folder=captures/trial['folder']; capture=folder/'approach'
        try:
            control=json.loads((folder/'approach-control.json').read_text())
            states,meta=replay(capture,control,trial['observation_yaw'])
            if trial['arm']=='turn' and not control.get('observation_reached'):raise ValueError('Observation heading not reached')
            terrain,camera,robot=snapshot(capture,meta['stage_frames']['decision'])
            manifest=json.loads((capture/'manifest.json').read_text())
            info=json.loads((capture/'camera-info.json').read_text())
            values,gate=predict(terrain.quality(True),terrain.cloud(),camera,robot,trial['observation_yaw'],info,
                manifest['projection_pixel_offset'],meta['decisions']['all_observed']['remaining_seconds'])
            old=meta['decisions']['all_observed']; oldkeys={tuple(k) for k in old['opportunity_cells']}
            for value in values:
                key=(value['ix'],value['iy']); final=states['final'].get(key,{})
                rows.append(dict(folder=trial['folder'],pair=trial['pair'],arm=trial['arm'],**value,
                    initial_deficient=states['baseline'].get(key,{}).get('fit_state') in ('too_few_voxels','insufficient_xy_spread'),
                    v1_prediction=key in oldkeys,final_state=final.get('fit_state','missing'),recovered=final.get('fit_state')=='valid'))
            trials.append(dict(folder=trial['folder'],experiment=control['experiment'],pair=trial['pair'],arm=trial['arm'],
                status='evaluated',gate=gate,old_gate=old,stage_frames=meta['stage_frames'],
                decision_stamp=meta['decision_event_stamp'],last_feature_stamp=meta['stage_last_stamps']['decision'],
                turn_sim_seconds=(meta['turn_event_stamp']-meta['decision_event_stamp']) if meta['turn_event_stamp'] else 0))
            np.savez_compressed(output/(trial['folder']+'-decision.npz'),cloud=terrain.cloud(),camera=camera,robot=robot)
        except (ValueError,FileNotFoundError) as error:
            trials.append(dict(folder=trial['folder'],pair=trial['pair'],arm=trial['arm'],status='failed',error=str(error)))
        for path in (folder/'approach-control.json',capture/'samples.npz',capture/'manifest.json',capture/'camera-info.json'):
            if path.exists():hashes[str(path.relative_to(captures))]=hashlib.sha256(path.read_bytes()).hexdigest()
        print(trials[-1]['folder'],trials[-1]['status'],flush=True)
    groups=[]
    for scope in ('all_decision_deficient','initial_deficient'):
      for arm in ('turn','no_turn'):
        for rule in ('v1_prediction','swept_visible','swept_prediction','marginal_prediction'):
            known=[r for r in rows if r['arm']==arm and r['final_state']!='missing' and (scope=='all_decision_deficient' or r['initial_deficient'])]
            tp=sum(r[rule] and r['recovered'] for r in known); fp=sum(r[rule] and not r['recovered'] for r in known)
            fn=sum(not r[rule] and r['recovered'] for r in known); tn=sum(not r[rule] and not r['recovered'] for r in known)
            groups.append(dict(scope=scope,arm=arm,rule=rule,tp=tp,fp=fp,fn=fn,tn=tn,precision=tp/(tp+fp) if tp+fp else None,
                recall=tp/(tp+fn) if tp+fn else None,meaning='Hold-arm turn prediction is counterfactual; do not score it as executed-turn efficacy'))
    pairs=[]
    for pair in sorted({t['pair'] for t in plan['trials']}):
        arms={arm:{(r['ix'],r['iy']):r for r in rows if r['pair']==pair and r['arm']==arm} for arm in ('turn','no_turn')}
        members={arm:next((t for t in trials if t['pair']==pair and t['arm']==arm and t['status']=='evaluated'),None) for arm in arms}
        if not all(members.values()):pairs.append(dict(pair=pair,status='incomplete'));continue
        for scope in ('all_decision_deficient','initial_deficient'):
            common=arms['turn'].keys() & arms['no_turn'].keys()
            if scope=='initial_deficient':common={k for k in common if all(arms[a][k]['initial_deficient'] for a in arms)}
            known=[k for k in common if all(arms[a][k]['final_state']!='missing' for a in arms)]
            extra=sum(arms['turn'][k]['recovered'] and not arms['no_turn'][k]['recovered'] for k in known)
            holdonly=sum(arms['no_turn'][k]['recovered'] and not arms['turn'][k]['recovered'] for k in known)
            pairs.append(dict(pair=pair,scope=scope,status='evaluated',common_remaining=len(common),missing=len(common)-len(known),
                turn_only=extra,hold_only=holdonly,net_recovered=extra-holdonly,
                would_turn=members['turn']['gate']['turn'],false_skip=not members['turn']['gate']['turn'] and extra>0,
                no_positive_net=members['turn']['gate']['turn'] and extra<=holdonly))
    write_csv(output/'cells.csv',rows);write_csv(output/'metrics.csv',groups);write_json(output/'pairs.json',pairs)
    write_json(output/'summary.json',dict(trials=trials,metrics=groups,pairs=pairs,input_sha256=hashes,
        online_policy_changed=False,accepted_policy=False,
        limits=['Sensor-only median-height ray patches; no occlusion, surface tilt or temporal noise model.',
                'Hypothesized points remain outside observed maps; support is not fit validity or accuracy.',
                'Correlated cell instances, descriptive paired results; no causal ground truth for individual cells.',
                'Both experimental arms are executed regardless of the shadow gate; failures retained.']))
    for path in (Path(__file__),ROOT/'scripts/arrival_resampling.py',ROOT/'scripts/resampling_dataset.py',
                 ROOT/'ros2_ws/src/mooncar_terrain/scripts/terrain_map.py'):
        shutil.copyfile(path,output/path.name)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('captures',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();report(args.captures,args.output)
