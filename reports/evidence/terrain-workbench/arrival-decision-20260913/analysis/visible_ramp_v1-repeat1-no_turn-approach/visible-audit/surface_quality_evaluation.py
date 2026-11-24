"""Offline visible-surface / Q diagnostic using synchronized truth; never controls the rover."""
import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import shutil
import numpy as np
from scipy.spatial.transform import Rotation

from depth_projection_evaluation import first_hit
from terrain_reference import read_shapes, reference_grid
from trajectory_evaluation import ROOT
from terrain_map import TerrainMap
from survey_evaluation import write_json, write_csv

CATEGORIES=('ground','sloped_top','level_top','curved_top','steep_side','mixed','no_visible_reference')


def surface_labels(shapes, owners, world):
    """Classify independent ray hits, not the magnitude of measured error."""
    labels=[None]*len(owners)
    for k in np.unique(owners):
        indices=np.flatnonzero(owners==k)
        if k<0:
            for i in indices:labels[i]=(-1,'unknown','no_visible_reference')
            continue
        _,kind,size,pose=shapes[k]
        local=(world[indices]-pose[:3,3])@pose[:3,:3]
        normals=np.zeros_like(local);faces=['surface']*len(indices)
        if kind=='plane':normals[:,2]=1
        elif kind=='ellipsoid':normals=local/size**2
        elif kind=='box':
            axis=np.argmin(np.abs(np.abs(local)-size),axis=1)
            for j,a in enumerate(axis):
                sign=1 if local[j,a]>=0 else -1
                normals[j,a]=sign;faces[j]=f'{a}:{sign}'
        elif kind=='cylinder':
            cap=np.abs(np.abs(local[:,2])-size[1])<1e-6
            normals[~cap,:2]=local[~cap,:2];normals[cap,2]=np.sign(local[cap,2])
            faces=['cap' if c else 'side' for c in cap]
        normals=normals@pose[:3,:3].T
        normals/=np.linalg.norm(normals,axis=1)[:,None]
        for i,n,face in zip(indices,normals,faces):
            if kind=='plane':category='ground'
            elif n[2]<=.5:category='steep_side'
            elif kind=='ellipsoid':category='curved_top'
            elif math.degrees(math.acos(float(np.clip(n[2],-1,1))))>3:category='sloped_top'
            else:category='level_top'
            labels[i]=(int(k),face,category)
    return labels


def compare_cell(ref, observed, ideal, labels):
    category=next(iter(labels))[2] if len(labels)==1 else ('mixed' if labels else 'no_visible_reference')
    paired=bool(observed and observed['valid'] and ideal and ideal['valid'])
    reason=('unobserved' if not observed else 'valid' if observed['valid'] else
            'too_few_voxels' if observed['points']<6 else 'insufficient_xy_spread' if observed['z'] is None else
            'steep_fit' if observed['slope']>=60 else 'nonplanar_or_multiple_heights')
    return dict(ix=ref['ix'],iy=ref['iy'],x=ref['x'],y=ref['y'],category=category,
                top_category=ref['category'],observed=observed is not None,
                quality=observed['quality'] if observed else None,
                valid=bool(observed and observed['valid']),reference_valid=bool(ideal and ideal['valid']),
                paired=paired,single_surface_pair=paired and category in ('ground','sloped_top','level_top','curved_top'),
                surface_count=len(labels),fit_status=reason,
                voxel_points=observed['points'] if observed else 0,
                coverage=observed['coverage'] if observed else None,
                view_directions=observed['view_directions'] if observed else None,
                height_error_m=observed['z']-ideal['z'] if paired else None,
                slope_error_deg=observed['slope']-ideal['slope'] if paired else None,
                highest_height_error_m=observed['z']-ref['z'] if observed and observed['valid'] and ref['z'] is not None else None)


def statistics(rows):
    observed=[r for r in rows if r['observed']];paired=[r for r in rows if r['paired']]
    height=np.array([r['height_error_m'] for r in paired]);slope=np.array([r['slope_error_deg'] for r in paired])
    return dict(cells=len(rows),observed=len(observed),unobserved=len(rows)-len(observed),
                invalid=sum(not r['valid'] for r in observed),paired=len(paired),
                single_surface_pairs=sum(r['single_surface_pair'] for r in rows),
                valid_fraction=sum(r['valid'] for r in observed)/len(observed) if observed else None,
                height_rmse_m=float(np.sqrt(np.mean(height**2))) if len(height) else None,
                slope_mae_deg=float(np.mean(np.abs(slope))) if len(slope) else None)


def analyse(folder, output, origin=(3.5,3.), radius=8.):
    folder,output=Path(folder).resolve(),Path(output).resolve()
    if output.exists() or output==folder or folder in output.parents:raise ValueError('Use a new separate output folder')
    if not all(math.isfinite(v) for v in (*origin,radius)) or radius<=0:raise ValueError('Invalid ROI')
    files=('manifest.json','samples.npz','survey_yard.sdf','terrain_geometry.py','terrain_node.py')
    inputs={n:hashlib.sha256((folder/n).read_bytes()).hexdigest() for n in files}
    frozen=json.loads((folder/'hashes.json').read_text())
    if any(frozen.get(n)!=v for n,v in inputs.items()):raise ValueError('Capture hash mismatch')
    manifest=json.loads((folder/'manifest.json').read_text())
    if manifest['status']!='captured' or manifest.get('projection_pixel_offset')!=.5:
        raise ValueError('Expected completed corrected Gazebo capture')
    shapes=read_shapes(folder/'survey_yard.sdf');seen=defaultdict(set);counts=defaultdict(int)
    observed=TerrainMap(source='evaluation only: measured depth at synchronized true poses')
    ideal=TerrainMap(source='evaluation only: nearest visible analytic surface at identical rays and true poses')
    missing=0;error_squared=0.;matched=0
    with np.load(folder/'samples.npz',allow_pickle=False) as data:
        points,offsets,cameras,stamps=data['points'],data['offsets'],data['truth_camera'],data['stamp']
        for i,(camera,stamp) in enumerate(zip(cameras,stamps)):
            cloud=points[offsets[i]:offsets[i+1]];direction=cloud.astype(float)/cloud[:,2,None]
            depth,owners=first_hit(shapes,camera[:3,3],direction@camera[:3,:3].T)
            valid=owners>=0;missing+=int((~valid).sum());matched+=int(valid.sum())
            error_squared+=float(np.sum((cloud[valid,2]-depth[valid])**2))
            reference=direction[valid]*depth[valid,None]
            hits=reference@camera[:3,:3].T+camera[:3,3]
            labels=surface_labels(shapes,owners[valid],hits)
            for key,label in zip(np.floor(hits[:,:2]/observed.resolution).astype(int),labels):
                seen[tuple(key)].add(label);counts[label[2]]+=1
            quat=Rotation.from_matrix(camera[:3,:3]).as_quat()
            observed.add_frame(cloud,float(stamp),camera[:3,3],quat)
            if len(reference)>=3:ideal.add_frame(reference,float(stamp),camera[:3,3],quat)
            if i%40==0:print(f'Replayed {i+1}/{len(stamps)} synchronized frames',flush=True)
    obs={(c['ix'],c['iy']):c for c in observed.elevation()};ref={(c['ix'],c['iy']):c for c in ideal.elevation()}
    rows=[compare_cell(c,obs.get((c['ix'],c['iy'])),ref.get((c['ix'],c['iy'])),seen[(c['ix'],c['iy'])])
          for c in reference_grid(shapes,origin,radius,observed.resolution)]
    groups=[dict(category=category,**statistics([r for r in rows if category=='all' or r['category']==category]))
            for category in ('all',*CATEGORIES)]
    bins=[]
    for category in ('all',*CATEGORIES):
        for lo,hi in ((0,.2),(.2,.4),(.4,.6),(.6,.8),(.8,1.00001)):
            bins.append(dict(category=category,q_min=lo,q_max=min(1,hi),**statistics([
                r for r in rows if r['quality'] is not None and lo<=r['quality']<hi and (category=='all' or r['category']==category)])))
    output.mkdir(parents=True)
    write_csv(output/'cells.csv',rows);write_csv(output/'groups.csv',groups);write_csv(output/'quality-bins.csv',bins)
    write_json(output/'summary.json',dict(status='diagnostic_complete',accepted_accuracy=False,
        source=str(folder),input_sha256=inputs,origin=list(origin),radius=radius,resolution=observed.resolution,
        frames=len(stamps),observed_points=len(points),ray_matched=matched,ray_missing=missing,
        ray_depth_rmse_m=math.sqrt(error_squared/matched) if matched else None,point_categories=dict(counts),groups=groups,
        high_q=statistics([r for r in rows if r['quality'] is not None and r['quality']>=.8]),
        limits=['Offline true-pose replay isolates geometry; not the online map or full pose accuracy.',
                'Same rays and frame set; measured/reference voxelization and cell membership can differ.',
                'Surface categories from independent expected ray hits; mixed faces retained separately.',
                'Ground/observed Q counts use synchronized evaluator frames, not the mission keyframe schedule.',
                'Missing and invalid fits remain in denominators; errors only for paired valid fits.',
                'Mixed fitted-plane differences are representation diagnostics, never single-surface accuracy.',
                'Q formula unchanged; correlated cells and one trajectory do not calibrate probability or establish causality.',
                'Primitive surfaces differ from rendered triangle meshes. No material or traversability truth.']))
    observed.save(output/'measured-true-pose.npz');ideal.save(output/'visible-reference.npz')
    plot(rows,bins,output)
    for name in ('surface_quality_evaluation.py','depth_projection_evaluation.py','terrain_reference.py','trajectory_evaluation.py','simulation_experiment.py','survey_evaluation.py'):
        shutil.copyfile(ROOT/'scripts'/name,output/name)
    for name in ('terrain_map.py','terrain_geometry.py'):shutil.copyfile(ROOT/'ros2_ws/src/mooncar_terrain/scripts'/name,output/name)
    write_json(output/'hashes.json',{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir() if p.is_file()})
    print(json.dumps(groups,indent=2))


def plot(rows,bins,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from matplotlib.collections import PatchCollection
    fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
    styles={'ground':'o','sloped_top':'s','curved_top':'^','mixed':'x'}
    for category,style in styles.items():
        selected=[b for b in bins if b['category']==category and b['observed']]
        x=[(b['q_min']+b['q_max'])/2 for b in selected]
        axes[0].plot(x,[100*b['valid_fraction'] for b in selected],style,color='black',label=f'{category} (N={sum(b["observed"] for b in selected)})')
        paired=[b for b in selected if b['paired']]
        axes[1].plot([(b['q_min']+b['q_max'])/2 for b in paired],[1000*b['height_rmse_m'] for b in paired],style,color='black',label=f'{category} (N={sum(b["paired"] for b in paired)})')
    for ax in axes:ax.set(xlabel='Sampling Q bin midpoint',xlim=(0,1));ax.legend(fontsize=8)
    axes[0].set(ylabel='Valid observed fits (%)',ylim=(-2,102));axes[1].set_ylabel('Visible-reference height RMSE (mm)')
    fig.suptitle('True-pose replay: Q is not percentage accuracy',fontsize=11)
    fig.savefig(output/'quality-vs-surface.png',dpi=180);fig.savefig(output/'quality-vs-surface.pdf');plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(10,5),layout='constrained')
    for ax,key,title in zip(axes,('highest_height_error_m','height_error_m'),('Highest-surface comparison','Matched visible-surface comparison')):
        valid=[r for r in rows if r[key] is not None]
        collection=PatchCollection([Rectangle((r['ix']*.25,r['iy']*.25),.25,.25) for r in valid],cmap='gray_r')
        collection.set_array(np.array([abs(r[key])*1000 for r in valid]));collection.set_clim(0,100)
        ax.add_collection(collection);ax.set_aspect('equal');ax.set_facecolor('#e5e5e5')
        ax.set(title=title,xlabel='World X (m)',ylabel='World Y (m)',
               xlim=(min(r['x'] for r in rows)-.125,max(r['x'] for r in rows)+.125),
               ylim=(min(r['y'] for r in rows)-.125,max(r['y'] for r in rows)+.125))
        fig.colorbar(collection,ax=ax,label='Abs height difference (mm)',extend='max',shrink=.65)
    fig.supxlabel('Gray background: no comparable value. Same display scale; different reference semantics.',fontsize=8)
    fig.savefig(output/'surface-correspondence.png',dpi=180);fig.savefig(output/'surface-correspondence.pdf');plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--origin',type=float,nargs=2,default=(3.5,3.));p.add_argument('--radius',type=float,default=8.)
    args=p.parse_args();analyse(args.folder,args.output,args.origin,args.radius)
