"""Post-hoc frame-subset diagnostics, not independent causal experiments or a planner."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import numpy as np
from scipy.spatial.transform import Rotation
from resampling_dataset import TerrainMap,features,ROOT
from survey_evaluation import write_json,write_csv


def phase_indices(stamps,robots,action_stamp,cut,stationary=False):
    """Estimate motion interval from poses; never label this Nav2's arrival event."""
    if stationary:return cut,cut
    yaw=np.arctan2(robots[:,1,0],robots[:,0,0]);last=robots[-1,:2,3]
    yaw_difference=lambda a,b:np.abs(np.arctan2(np.sin(a-b),np.cos(a-b)))
    moved=(np.linalg.norm(robots[:,:2,3]-robots[cut-1,:2,3],axis=1)>.01)|(yaw_difference(yaw,yaw[cut-1])>.01)
    active=np.flatnonzero((stamps>=action_stamp)&moved)
    if not len(active):return len(stamps),len(stamps)
    outside=(np.linalg.norm(robots[:,:2,3]-last,axis=1)>.03)|(yaw_difference(yaw,yaw[-1])>.03)
    late=np.flatnonzero(outside & (np.arange(len(stamps))>=active[0]))
    settled=max(int(active[0]),int(late[-1])+1 if len(late) else int(active[0]))
    return int(active[0]),settled


def subset_indices(count,cut,begin,settled):
    prefix=list(range(cut))
    return dict(prefix=prefix,waiting=prefix+list(range(cut,begin)),
                transit=prefix+list(range(begin,settled)),settled=prefix+list(range(settled,count)),
                moving_and_settled=prefix+list(range(begin,count)),
                final_frame=prefix+[count-1],full=list(range(count)))


def spread(terrain,key):
    cloud=terrain.cloud();indices=np.floor(cloud[:,:2]/terrain.resolution).astype(int)
    points=cloud[np.all(indices==key,axis=1),:2].astype(float)
    return float(np.linalg.svd(points-points.mean(axis=0),compute_uv=False)[-1]/math.sqrt(len(points))) if len(points)>1 else 0.


def report(plan,output):
    specification=json.loads(plan.read_text());output.mkdir(parents=True,exist_ok=False)
    with Path(specification['predictions']).open(encoding='utf-8-sig') as handle:predictions=list(csv.DictReader(handle))
    rows=[];groups=[];metadata=[];hashes={}
    for trial in specification['trials']:
        root=Path(trial['folder']);condition=trial['condition'];capture=root/condition
        control=json.loads((root/f'{condition}-control.json').read_text())
        if control['status']!='completed':raise ValueError('Attribution requires a completed capture')
        with np.load(capture/'samples.npz',allow_pickle=False) as data:
            points,offsets,poses,robots,stamps=(data[k] for k in ('points','offsets','estimated_camera','estimated_robot','stamp'))
        cutoff=min(stamps[0]+5,control.get('action_started_sim_stamp',math.inf))
        cut=int(np.searchsorted(stamps,cutoff,side='left'))
        if not 0<cut<len(stamps):raise ValueError('Invalid baseline window')
        begin,settled=phase_indices(stamps,robots,control.get('action_started_sim_stamp',cutoff),cut,condition=='stationary')
        maps={};states={};first_valid={}
        for name,indices in subset_indices(len(stamps),cut,begin,settled).items():
            terrain=TerrainMap()
            for i in indices:
                terrain.add_frame(points[offsets[i]:offsets[i+1]],float(stamps[i]),poses[i,:3,3],Rotation.from_matrix(poses[i,:3,:3]).as_quat())
                if name=='full' and i>=cut:
                    for cell in terrain.quality(with_surface=True)['cells']:
                        if cell['fit_state']=='valid':first_valid.setdefault((cell['ix'],cell['iy']),float(stamps[i]))
            maps[name]=terrain;states[name]={(c['ix'],c['iy']):c for c in terrain.quality(with_surface=True)['cells']}
        single_transit_valid=set();single_transit_frames=[]
        for index in range(begin,settled):
            terrain=TerrainMap()
            for i in [*range(cut),index]:
                terrain.add_frame(points[offsets[i]:offsets[i+1]],float(stamps[i]),poses[i,:3,3],Rotation.from_matrix(poses[i,:3,:3]).as_quat())
            valid={(c['ix'],c['iy']) for c in terrain.quality(with_surface=True)['cells'] if c['fit_state']=='valid'}
            single_transit_valid.update(valid);single_transit_frames.append((index,valid))
        baseline=states['prefix'];selected=[r for r in predictions if r['experiment']==control['experiment'] and r['eligible']=='True']
        for prediction in selected:
            key=(int(prediction['ix']),int(prediction['iy']));cell=baseline[key]
            recovered={name:cells.get(key,{}).get('fit_state')=='valid' for name,cells in states.items()}
            assert recovered['full']==(prediction['recovered']=='True'),'Frozen full-replay label changed'
            actual=features(cell,.25,poses[cut-1,:3,3],poses[-1,:3,3],math.atan2(poses[-1,1,2],poses[-1,0,2]))
            rows.append(dict(experiment=control['experiment'],condition=condition,ix=key[0],iy=key[1],
                false_negative=prediction['distance_direction_prediction']=='False' and recovered['full'],
                predicted=prediction['distance_direction_prediction']=='True',candidate_in_fov=prediction['candidate_in_fov']=='True',
                initial_state=cell['fit_state'],initial_points=cell['points'],point_count_proxy=float(prediction['point_count_proxy']),
                final_points=states['full'][key]['points'],initial_xy_spread_m=spread(maps['prefix'],key),
                final_xy_spread_m=spread(maps['full'],key),first_valid_stamp=first_valid.get(key),
                actual_endpoint_prediction_posthoc=actual['distance_direction_prediction'],
                any_single_transit_recovered=key in single_transit_valid,
                **{name+'_recovered':value for name,value in recovered.items()}))
        for cohort in ('all_deficient','false_negatives'):
            subset=[r for r in rows if r['experiment']==control['experiment'] and (cohort=='all_deficient' or r['false_negative'])]
            for name in (*maps,'any_single_transit'):
                groups.append(dict(experiment=control['experiment'],cohort=cohort,subset=name,cells=len(subset),
                                   recovered=sum(r[name+'_recovered'] for r in subset)))
        metadata.append(dict(experiment=control['experiment'],condition=condition,frames=len(stamps),baseline_frames=cut,
            begin_index=begin,settled_index=settled,motion_start_pose_stamp=float(stamps[begin]) if condition!='stationary' and begin<len(stamps) else None,
            settled_pose_stamp=float(stamps[settled]) if settled<len(stamps) else None,
            arrival_definition='post-hoc stable suffix within .03m/.03rad of final estimated pose; not Nav2 event',
            single_transit_false_negative_counts=[dict(frame=i,stamp=float(stamps[i]),recovered=sum(
                (r['ix'],r['iy']) in valid for r in rows if r['experiment']==control['experiment'] and r['false_negative']))
                for i,valid in single_transit_frames],
            estimated_translation_m=float(np.linalg.norm(robots[-1,:2,3]-robots[cut-1,:2,3]))))
        for path in (capture/'samples.npz',root/f'{condition}-control.json'):
            hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    hashes[specification['predictions']]=hashlib.sha256(Path(specification['predictions']).read_bytes()).hexdigest()
    write_csv(output/'cells.csv',rows);write_csv(output/'subsets.csv',groups)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    experiments=[m['experiment'] for m in metadata if m['condition']=='approach']
    names=('waiting','any_single_transit','transit','settled','final_frame','moving_and_settled','full')
    labels=('Wait only','Any single\ntransit (oracle)','All transit','Settled only','Final frame','Transit +\nsettled','Full')
    fig,ax=plt.subplots(figsize=(11,5),layout='constrained')
    for j,experiment in enumerate(experiments):
        selected={g['subset']:g for g in groups if g['experiment']==experiment and g['cohort']=='false_negatives'}
        values=[selected[n]['recovered'] for n in names];width=.8/max(1,len(experiments))
        positions=np.arange(len(names))-.4+width*(j+.5)
        ax.bar(positions,values,width,color='white',edgecolor='black',hatch='//' if j else '',label=f"{experiment[-6:]}: {selected['full']['cells']} missed cells")
        for x,y in zip(positions,values):ax.text(x,y+.2,str(y),ha='center',fontsize=8)
    ax.set_xticks(range(len(names)),labels,fontsize=8);ax.set_ylabel('Recovered initial missed cells')
    ax.set_title('Same-data frame-subset diagnostic; recovery counts overlap');ax.legend(fontsize=8)
    fig.supxlabel('Common baseline. Unequal exposures. Per-cell single-frame oracle is not one common viewpoint or a predictor.',fontsize=8)
    fig.savefig(output/'subsets.png',dpi=180);fig.savefig(output/'subsets.pdf');plt.close(fig)
    write_json(output/'summary.json',dict(trials=metadata,input_sha256=hashes,limits=[
        'Frame-subset ablation of the same observations, not independent controlled trials.',
        'Subset recovery counts overlap; do not add them or call them causal contributions.',
        'Any-single-transit is a per-cell oracle over different single frames, not one shared image or deployable prediction.',
        'Unequal frame counts/exposure; a stable suffix is not a matched-duration control.',
        'Estimated poses and sensor fit labels only; no accuracy or safety acceptance.',
        'Actual endpoint and recovery times are post-hoc diagnostics, never prediction features.']))
    shutil.copyfile(plan,output/'plan.json')
    for path in (Path(__file__),ROOT/'scripts/resampling_dataset.py',ROOT/'ros2_ws/src/mooncar_terrain/scripts/terrain_map.py',ROOT/'ros2_ws/src/mooncar_terrain/scripts/terrain_geometry.py'):
        shutil.copyfile(path,output/path.name)
    print(json.dumps(metadata,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('plan',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();report(a.plan,a.output)
