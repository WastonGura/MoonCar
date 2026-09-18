"""Evaluation-only synchronized poses and depth samples; never publishes control or truth."""
import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
import numpy as np

from simulation_experiment import transform
from survey_evaluation import write_json, write_csv
from terrain_reference import read_shapes, surface
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/mooncar_terrain/scripts'))
from terrain_geometry import transform_points


def matrix(pose):
    result=np.eye(4)
    result[:3,:3]=transform_points(np.eye(3),[0,0,0],pose[3:]).T
    result[:3,3]=pose[:3]
    return result


def interpolate(samples, stamp, max_gap=.05):
    """Bracket only; no extrapolation. Shortest-arc quaternion interpolation."""
    times=np.array([s[0] for s in samples]); i=int(np.searchsorted(times,stamp))
    if i<len(times) and abs(times[i]-stamp)<1e-8: return matrix(samples[i][1]),0.
    if not 0<i<len(times): raise ValueError('Ground truth does not bracket observation')
    gap=times[i]-times[i-1]
    if gap<=0 or gap>max_gap: raise ValueError('Ground-truth gap exceeds limit')
    f=(stamp-times[i-1])/gap; a=np.array(samples[i-1][1]); b=np.array(samples[i][1])
    qa=a[3:]/np.linalg.norm(a[3:]); qb=b[3:]/np.linalg.norm(b[3:])
    if qa@qb<0: qb=-qb
    angle=math.acos(float(np.clip(qa@qb,-1,1)))
    q=(1-f)*qa+f*qb if angle<1e-6 else (math.sin((1-f)*angle)*qa+math.sin(f*angle)*qb)/math.sin(angle)
    return matrix(np.r_[(1-f)*a[:3]+f*b[:3],q/np.linalg.norm(q)]),float(gap)


def camera_extrinsic(path):
    model=ET.parse(path).getroot().find("model[@name='mooncar']")
    if model is None: raise ValueError('Expected converted MoonCar model')
    link=model.find("link[@name='base_footprint']")
    sensor=link.find("sensor[@name='mooncar_depth_camera']") if link is not None else None
    if sensor is None: raise ValueError('Camera must be fixed into the model-root base_footprint link')
    # Gazebo camera axes: forward X, left Y, up Z; ROS optical: right X, down Y, forward Z.
    optical=np.eye(4); optical[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
    return transform(model)@transform(link)@transform(sensor)@optical,sensor.findtext('gz_frame_id')


def ground_masks(shapes,world):
    height,owner=surface(shapes,world[:,:2])
    footprint=np.array([k>=0 and shapes[k][1]=='plane' for k in owner])
    interior=footprint.copy()
    # Fixed spatial margin, never an error-based outlier filter. Keep full-footprint metrics too.
    for _,kind,size,pose in shapes:
        if kind=='plane': continue
        extent=size if kind!='cylinder' else np.array([size[0],size[0],size[1]])
        half=(np.abs(pose[:3,:3])@extent)[:2]+.25
        interior &= np.any(np.abs(world[:,:2]-pose[:2,3])>half,axis=1)
    return height,footprint,interior


def capture(folder, robot_sdf, world_sdf, seconds, raw_depth=False):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, TransformException
    from sensor_msgs.msg import PointCloud2, CameraInfo, Image
    from sensor_msgs_py.point_cloud2 import read_points_numpy
    if folder.exists(): raise ValueError('Choose a new output folder; recordings are preserved')
    if not 5<=seconds<=300 or not math.isfinite(seconds): raise ValueError('Duration must be 5..300 simulation seconds')
    if not os.environ.get('GZ_PARTITION') or os.environ.get('ROS_DOMAIN_ID')!='89':
        raise ValueError('Explicit experiment partition and ROS domain 89 required')
    extrinsic,optical_frame=camera_extrinsic(robot_sdf)
    folder.mkdir(parents=True)
    shutil.copyfile(Path(__file__),folder/'capture_source.py')
    shutil.copyfile(robot_sdf,folder/'robot.sdf'); shutil.copyfile(world_sdf,folder/'survey_yard.sdf')
    manifest=dict(status='starting',seconds=seconds,partition=os.environ['GZ_PARTITION'],domain=89,
                  raw_depth=raw_depth,
                  max_truth_gap_seconds=.05,observation_interval_seconds=.5,correction_fitted=False,
                  truth_method='Gazebo dynamic model poses; linear translation and shortest-arc quaternion interpolation',
                  camera_method='Converted simulator SDF sensor pose plus explicit Gazebo-to-optical axis convention',
                  camera_extrinsic=extrinsic.tolist(),optical_frame=optical_frame)
    write_json(folder/'manifest.json',manifest)
    truth=deque(maxlen=20000); lock=threading.Lock(); errors=[]; drops=[]; frames=[]; clouds=[]; pending=deque(maxlen=20)
    images=deque(maxlen=64); raw_images=[]; depth_format=None
    gzlog=(folder/'gazebo.log').open('w')
    def receive():
        try:
            with (folder/'truth.jsonl').open('w') as handle:
                for line in process.stdout:
                    if not line.strip(): continue
                    msg=json.loads(line); header=msg['header']['stamp']
                    stamp=int(header.get('sec',0))+int(header.get('nsec',0))/1e9
                    body=next(p for p in msg['pose'] if p.get('name')=='mooncar')
                    pose=[body.get('position',{}).get(k,0) for k in ('x','y','z')]+[body.get('orientation',{}).get(k,0) for k in ('x','y','z','w')]
                    with lock:
                        if truth and stamp<truth[-1][0]: raise ValueError('Gazebo clock reset during capture')
                        if truth and stamp==truth[-1][0]: continue
                        truth.append((stamp,pose))
                    handle.write(json.dumps(dict(stamp=stamp,pose=pose))+'\n')
        except Exception as error: errors.append(str(error))
    rclpy.init(); node=Node('terrain_trajectory_evaluator',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer(node=node); listener=TransformListener(buffer,node)
    last_selected=-math.inf; first=None; wall_started=time.monotonic(); info={}
    def cloud(msg):
        nonlocal last_selected
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if stamp-last_selected<.5: return
        last_selected=stamp
        if len(pending)==pending.maxlen: drops.append(dict(stamp=stamp,reason='pending queue full')); return
        pending.append((msg,time.monotonic()))
    def camera_info(msg):
        value=dict(frame=msg.header.frame_id,width=msg.width,height=msg.height,k=list(msg.k),p=list(msg.p),d=list(msg.d))
        if info and value!=info: errors.append('CameraInfo changed during capture')
        info.update(value)
    node.create_subscription(PointCloud2,'/mooncar/terrain/observations',cloud,qos_profile_sensor_data)
    node.create_subscription(CameraInfo,'/mooncar/depth/camera_info',camera_info,qos_profile_sensor_data)
    if raw_depth: node.create_subscription(Image,'/mooncar/depth/image',lambda msg:images.append(msg),qos_profile_sensor_data)
    parameters=AsyncParameterClient(node,'/mooncar_terrain')
    try:
        if not parameters.wait_for_services(timeout_sec=10): raise RuntimeError('Terrain parameters unavailable')
        future=parameters.get_parameters(['depth_pixel_offset'])
        rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if not future.done() or future.result() is None or future.result().values[0].type!=Parameter.Type.DOUBLE.value:
            raise RuntimeError('Terrain pixel offset is not declared as a double')
        manifest['projection_pixel_offset']=future.result().values[0].double_value
    except Exception as error:
        manifest.update(status='incomplete',error=str(error));write_json(folder/'manifest.json',manifest)
        gzlog.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        raise
    write_json(folder/'manifest.json',manifest)
    for filename in ('terrain_geometry.py','terrain_node.py'):
        shutil.copyfile(ROOT/'ros2_ws/src/mooncar_terrain/scripts'/filename,folder/filename)
    process=subprocess.Popen(['gz','topic','-e','-t','/world/lunar_yard/dynamic_pose/info','--json-output'],stdout=subprocess.PIPE,stderr=gzlog,text=True)
    reader=threading.Thread(target=receive,daemon=True); reader.start()
    try:
        while time.monotonic()-wall_started<1200:
            rclpy.spin_once(node,timeout_sec=.03)
            if errors: raise RuntimeError(errors[0])
            if process.poll() is not None: raise RuntimeError('Gazebo truth stream ended')
            if first is not None and node.get_clock().now().nanoseconds/1e9-first>=seconds: break
            if first is None and time.monotonic()-wall_started>45: raise RuntimeError('No synchronized observations within 45 seconds')
            if not pending: continue
            msg,received=pending[0]; stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            try:
                if msg.header.frame_id!=optical_frame: raise RuntimeError('Observation optical frame differs from SDF sensor')
                with lock: actual,gap=interpolate(list(truth),stamp)
                at=Time.from_msg(msg.header.stamp)
                def lookup(child):
                    value=buffer.lookup_transform('map',child,at).transform
                    p,q=value.translation,value.rotation
                    return matrix([p.x,p.y,p.z,q.x,q.y,q.z,q.w])
                estimated=lookup('base_footprint'); camera=lookup(optical_frame)
                if raw_depth:
                    image=next((im for im in images if im.header.stamp==msg.header.stamp),None)
                    if image is None or not info: raise ValueError('Matching raw depth or CameraInfo unavailable')
                    fmt=dict(width=image.width,height=image.height,step=image.step,encoding=image.encoding,
                             bigendian=bool(image.is_bigendian),frame=image.header.frame_id)
                    if depth_format is not None and fmt!=depth_format: raise RuntimeError('Depth format changed')
                    if fmt['frame']!=optical_frame: raise RuntimeError('Raw depth frame differs from optical frame')
                    depth_format=fmt
            except (ValueError,TransformException) as error:
                if time.monotonic()-received>4:
                    drops.append(dict(stamp=stamp,reason=str(error))); pending.popleft()
                continue
            pending.popleft()
            points=read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True).copy()
            if not len(points): drops.append(dict(stamp=stamp,reason='empty cloud')); continue
            if first is None:
                first=stamp; manifest['status']='recording';write_json(folder/'manifest.json',manifest)
            frames.append(dict(stamp=stamp,gap=gap,truth_robot=actual,estimated_robot=estimated,
                               truth_camera=actual@extrinsic,estimated_camera=camera))
            clouds.append(points)
            if raw_depth: raw_images.append(np.frombuffer(bytes(image.data),dtype=np.uint8).copy())
            # One requested sensor-only prefix for prospective offline-rule probes.
            if (folder/'checkpoint-request.json').exists() and not (folder/'checkpoint.json').exists():
                save_sensor_prefix(folder,frames,clouds,info,manifest['projection_pixel_offset'])
        else: raise RuntimeError('Wall-time limit exceeded')
        manifest['status']='captured'
    except Exception as error:
        manifest.update(status='incomplete',error=str(error))
    finally:
        process.terminate()
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: process.kill();process.wait()
        reader.join(timeout=5);gzlog.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if frames:
            np.savez_compressed(folder/'samples.npz',points=np.concatenate(clouds),offsets=np.r_[0,np.cumsum([len(c) for c in clouds])],
                                **{k:np.array([f[k] for f in frames]) for k in frames[0]})
            if raw_depth:
                np.savez_compressed(folder/'raw-depth.npz',data=np.stack(raw_images),stamps=np.array([f['stamp'] for f in frames]))
                write_json(folder/'depth-format.json',depth_format)
        write_json(folder/'camera-info.json',info);write_json(folder/'drops.json',drops)
        manifest.update(frames=len(frames),dropped_selected_frames=len(drops),wall_seconds=time.monotonic()-wall_started,
                        first_stamp=first,last_stamp=frames[-1]['stamp'] if frames else None)
        write_json(folder/'manifest.json',manifest)
    if manifest['status']!='captured': raise RuntimeError(manifest.get('error','Capture failed'))


def save_sensor_prefix(folder,frames,clouds,info,pixel_offset):
    """Publish data before the completion marker; deliberately exclude truth arrays."""
    path=folder/'checkpoint.npz'
    np.savez_compressed(path,points=np.concatenate(clouds),
        offsets=np.r_[0,np.cumsum([len(c) for c in clouds])],
        **{k:np.array([f[k] for f in frames]) for k in ('stamp','estimated_camera','estimated_robot')})
    write_json(folder/'checkpoint.json',dict(frames=len(frames),last_stamp=frames[-1]['stamp'],
        first_stamp=frames[0]['stamp'],camera_info=info,pixel_offset=pixel_offset,
        sha256=hashlib.sha256(path.read_bytes()).hexdigest()))


def report(folder):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    manifest=json.loads((folder/'manifest.json').read_text())
    shapes=read_shapes(folder/'survey_yard.sdf'); rows=[]
    with np.load(folder/'samples.npz',allow_pickle=False) as data:
        for i,stamp in enumerate(data['stamp']):
            estimated,truth=data['estimated_robot'][i],data['truth_robot'][i]
            ec,tc=data['estimated_camera'][i],data['truth_camera'][i]
            points=data['points'][data['offsets'][i]:data['offsets'][i+1]]
            world=points@tc[:3,:3].T+tc[:3,3]; mapped=points@ec[:3,:3].T+ec[:3,3]
            height,ground,interior=ground_masks(shapes,world)
            yaw=lambda m:math.atan2(m[1,0],m[0,0])
            angle=yaw(estimated)-yaw(truth)
            diff=np.linalg.inv(estimated)@ec-np.array(manifest['camera_extrinsic'])
            row=dict(stamp=float(stamp),elapsed=float(stamp-data['stamp'][0]),truth_gap=float(data['gap'][i]),
                     true_x=float(truth[0,3]),true_y=float(truth[1,3]),true_z=float(truth[2,3]),true_yaw=yaw(truth),
                     estimated_x=float(estimated[0,3]),estimated_y=float(estimated[1,3]),
                     xy_error_m=float(np.linalg.norm(estimated[:2,3]-truth[:2,3])),
                     robot_z_error_m=float(estimated[2,3]-truth[2,3]),
                     camera_z_error_m=float(ec[2,3]-tc[2,3]),
                     yaw_error_deg=math.degrees(math.atan2(math.sin(angle),math.cos(angle))),
                     extrinsic_max_abs_matrix_difference=float(np.abs(diff).max()),
                     points=len(points),ground_points=int(ground.sum()),interior_points=int(interior.sum()))
            for group,mask in (('ground',ground),('interior',interior)):
                for label,cloud in (('true_pose',world),('map_pose',mapped)):
                    dz=cloud[mask,2]-height[mask]
                    row[label+'_'+group+'_bias_m']=float(dz.mean()) if len(dz) else None
                    row[label+'_'+group+'_rmse_m']=float(np.sqrt(np.mean(dz**2))) if len(dz) else None
            rows.append(row)
    write_csv(folder/'frames.csv',rows)
    xy=np.array([r['xy_error_m'] for r in rows]); z=np.array([r['robot_z_error_m'] for r in rows])
    n=sum(r['ground_points'] for r in rows)
    summary=dict(status=manifest['status'],frames=len(rows),dropped_selected_frames=manifest['dropped_selected_frames'],
                 duration=rows[-1]['elapsed'],xy_rmse_m=float(np.sqrt(np.mean(xy**2))),xy_max_m=float(xy.max()),
                 robot_z_max_abs_m=float(np.abs(z).max()),camera_z_max_abs_m=max(abs(r['camera_z_error_m']) for r in rows),
                 yaw_max_abs_deg=max(abs(r['yaw_error_deg']) for r in rows),
                 extrinsic_max_abs_matrix_difference=max(r['extrinsic_max_abs_matrix_difference'] for r in rows),
                 sampled_truth_path_m=sum(math.hypot(b['true_x']-a['true_x'],b['true_y']-a['true_y']) for a,b in zip(rows,rows[1:])),
                 ground_points=n,correction_fitted=False,accepted_global_accuracy=False)
    for group in ('ground','interior'):
        count=sum(r[group+'_points'] for r in rows);summary[group+'_points']=count
        for label in ('true_pose','map_pose'):
            summary[label+'_'+group+'_rmse_m']=math.sqrt(sum(r[group+'_points']*(r[label+'_'+group+'_rmse_m'] or 0)**2 for r in rows)/count) if count else None
            summary[label+'_'+group+'_bias_m']=sum(r[group+'_points']*(r[label+'_'+group+'_bias_m'] or 0) for r in rows)/count if count else None
    summary['interior_spatial_margin_m']=.25
    summary['limitations']=['One controlled trajectory, not global or hardware calibration.',
        'Truth interpolation requires bracketing gap <= 0.05 simulation seconds; rejected samples are retained in drops.json.',
        'Ground subset uses independent world XY top-surface identity; no height-error rejection. Points are temporally correlated.',
        'Interior subset additionally excludes non-plane visual AABBs inflated by 0.25 m; excluded footprint points remain in full-ground statistics.',
        'True-pose reprojection isolates pose contribution but still includes sensor projection/rendering errors; no depth image refit.',
        'Fixed camera extrinsics come from the converted simulator model, not fitted against observed ground.']
    write_json(folder/'summary.json',summary)
    fig,axes=plt.subplots(1,3,figsize=(12,3.8),layout='constrained')
    axes[0].plot([r['true_x'] for r in rows],[r['true_y'] for r in rows],'k-',label='Gazebo truth')
    axes[0].plot([r['estimated_x'] for r in rows],[r['estimated_y'] for r in rows],'k--',label='ROS map pose')
    axes[0].set(xlabel='X (m)',ylabel='Y (m)',aspect='equal',title='Synchronized trajectory');axes[0].legend(fontsize=8)
    x=[r['true_x'] for r in rows]; y=[r['true_y'] for r in rows]
    half=max(.75,(max(x)-min(x))*.6,(max(y)-min(y))*.6)
    axes[0].set(xlim=((max(x)+min(x))/2-half,(max(x)+min(x))/2+half),
                ylim=((max(y)+min(y))/2-half,(max(y)+min(y))/2+half))
    axes[0].legend(fontsize=8,loc='upper left')
    t=[r['elapsed'] for r in rows]
    axes[1].plot(t,xy*100,'k-',label='XY pose difference');axes[1].plot(t,z*100,'k--',label='Z pose difference')
    axes[1].set(xlabel='Simulation seconds',ylabel='Difference (cm)',title='Pose correspondence');axes[1].legend(fontsize=8)
    for label,style in (('true_pose','k-'),('map_pose','k--')):
        axes[2].plot(t,[r[label+'_interior_rmse_m']*1000 if r[label+'_interior_rmse_m'] is not None else np.nan for r in rows],style,label=label)
    axes[2].set(xlabel='Simulation seconds',ylabel='Point height RMSE (mm)',title='Interior plane height',ylim=(0,None));axes[2].legend(fontsize=8)
    fig.savefig(folder/'trajectory-height.png',dpi=180);fig.savefig(folder/'trajectory-height.pdf');plt.close(fig)
    for filename in ('trajectory_evaluation.py','terrain_reference.py','simulation_experiment.py','survey_evaluation.py'):
        shutil.copyfile(ROOT/'scripts'/filename,folder/filename)
    if not (folder/'terrain_geometry.py').exists():
        shutil.copyfile(ROOT/'ros2_ws/src/mooncar_terrain/scripts/terrain_geometry.py',folder/'terrain_geometry.py')
    write_json(folder/'hashes.json',{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir() if p.is_file() and p.name not in ('hashes.json','run.log')})
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('capture'); p.add_argument('folder',type=Path);p.add_argument('--robot-sdf',type=Path,required=True);p.add_argument('--world-sdf',type=Path,required=True);p.add_argument('--seconds',type=float,default=90);p.add_argument('--raw-depth',action='store_true')
    p=sub.add_parser('report');p.add_argument('folder',type=Path)
    args=parser.parse_args()
    if args.command=='capture': capture(args.folder,args.robot_sdf,args.world_sdf,args.seconds,args.raw_depth)
    else: report(args.folder)
