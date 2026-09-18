"""Evaluate a frozen path-opportunity rule on controlled, independently generated worlds."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
from scipy.spatial.transform import Rotation
from resampling_dataset import extract,ROOT,TerrainMap
from depth_projection_evaluation import first_hit
from terrain_reference import read_shapes
from surface_quality_evaluation import surface_labels,analyse
from trajectory_evaluation import report as trajectory_report
from survey_evaluation import write_json,write_csv


def report(root,output):
    plan=json.loads((root/'plan.json').read_text())
    for name,digest in plan['source_sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Predeclared source changed: '+name)
    output.mkdir(parents=True,exist_ok=False)
    rows=[];groups=[];trials=[]
    for trial in plan['trials']:
        scene=trial['scene'];folder=root/scene
        for condition in plan['conditions']:
            capture=folder/condition;control=json.loads((folder/f'{condition}-control.json').read_text())
            if control.get('status')!='completed':
                trials.append(dict(scene=scene,condition=condition,status='failed',control=control));continue
            saved=json.loads((folder/'plan.json').read_text())
            if saved['start']!=trial['start'] or saved['goal']!=trial['goal'] or saved['scene']!=scene:
                raise ValueError('Captured task differs from plan')
            if (capture/'survey_yard.sdf').read_bytes()!=(root/(scene+'.sdf')).read_bytes():
                # XML declarations differ; compare the parsed model instead of formatting.
                import xml.etree.ElementTree as ET
                if ET.tostring(ET.parse(capture/'survey_yard.sdf').getroot())!=ET.tostring(ET.parse(root/(scene+'.sdf')).getroot()):
                    raise ValueError('World differs from visibility precheck')
            values,meta=extract(capture,control,trial['goal'],'prospective_independent_world',saved['planned_path'])
            # Truth labels are evaluator-only strata, attached AFTER sensor-only prediction.
            seen=defaultdict(set);terrain=TerrainMap();shapes=read_shapes(capture/'survey_yard.sdf')
            with np.load(capture/'samples.npz',allow_pickle=False) as data:
                for i,stamp in enumerate(data['stamp']):
                    cloud=data['points'][data['offsets'][i]:data['offsets'][i+1]]
                    estimated=data['estimated_camera'][i]
                    terrain.add_frame(cloud,float(stamp),estimated[:3,3],Rotation.from_matrix(estimated[:3,:3]).as_quat())
                    if i>=meta['baseline_frames']:continue
                    truth=data['truth_camera'][i];directions=cloud.astype(float)/cloud[:,2,None]
                    depths,owners=first_hit(shapes,truth[:3,3],directions@truth[:3,:3].T);valid=owners>=0
                    hits=(directions[valid]*depths[valid,None])@truth[:3,:3].T+truth[:3,3]
                    labels=surface_labels(shapes,owners[valid],hits)
                    measured=cloud[valid]@estimated[:3,:3].T+estimated[:3,3]
                    for key,label in zip(np.floor(measured[:,:2]/.25).astype(int),labels):seen[tuple(key)].add(label)
                xy=data['truth_robot'][data['stamp']>=control['action_started_sim_stamp'],:2,3]
                a,b=np.asarray(saved['planned_path']);direction=b-a
                closest=a+np.clip((xy-a)@direction/(direction@direction),0,1)[:,None]*direction
                deviation=float(np.linalg.norm(xy-closest,axis=1).max()) if len(xy) else None
                poses=data['truth_robot'][data['stamp']>=control['action_started_sim_stamp']]
                delta=np.arctan2(poses[:,1,0],poses[:,0,0])-np.arctan2(direction[1],direction[0])
                yaw_deviation=float(np.degrees(np.abs(np.arctan2(np.sin(delta),np.cos(delta)))).max()) if len(delta) else None
            for row in values:
                labels=seen[(row['ix'],row['iy'])]
                row.update(scene=scene,baseline_surface=next(iter(labels))[2] if len(labels)==1 else 'mixed' if labels else 'no_reference')
            rows.extend(values)
            target='sloped_top' if 'ramp' in scene else 'curved_top'
            for scope in ('all_initial_deficient','pure_target_initial_deficient'):
                selected=[r for r in values if r['eligible'] and (scope=='all_initial_deficient' or r['baseline_surface']==target)]
                for rule in ('distance_direction_prediction','path_prediction'):
                    known=[r for r in selected if r['recovered'] is not None]
                    tp=sum(r[rule] and r['recovered'] for r in known);fp=sum(r[rule] and not r['recovered'] for r in known)
                    fn=sum(not r[rule] and r['recovered'] for r in known);tn=sum(not r[rule] and not r['recovered'] for r in known)
                    groups.append(dict(scene=scene,condition=condition,experiment=control['experiment'],scope=scope,rule=rule,
                        cells=len(selected),missing=len(selected)-len(known),tp=tp,fp=fp,fn=fn,tn=tn,
                        precision=tp/(tp+fp) if tp+fp else None,recall=tp/(tp+fn) if tp+fn else None))
            trial_output=output/f'{scene}-{condition}';trial_output.mkdir()
            terrain.save(trial_output/'estimated-map.npz')
            trajectory_report(capture)
            analyse(capture,trial_output/'visible-audit',origin=trial['start'][:2],radius=5.)
            if condition=='approach':plot_capture(folder,terrain,trial,trial_output)
            trials.append(dict(scene=scene,condition=condition,**meta,path_deviation_max_m=deviation,
                tangent_yaw_deviation_max_deg=yaw_deviation,
                path_deviation_semantics='Post-action true XY distance to predeclared segment; not a predictor input',
                pure_target_baseline_cells=sum(r['baseline_surface']==target for r in values),
                pure_target_deficient_cells=sum(r['baseline_surface']==target and r['eligible'] for r in values)))
    write_csv(output/'samples.csv',rows);write_csv(output/'rule-check.csv',groups)
    write_json(output/'summary.json',dict(trials=trials,groups=groups,accepted_policy=False,features_use_truth=False,
        fitted_parameters=False,limits=plan['limits']+['Baseline surface strata use independent ray ownership assigned to the measured estimated-pose cell; mixed labels remain separate.',
        'Recovery means a valid fused plane fit, not better height accuracy or safe terrain.',
        'Stationary and approach have fresh baselines; compare within each baseline, not identical-cell randomized effects.']))
    shutil.copyfile(root/'plan.json',output/'plan.json')
    shutil.copyfile(Path(__file__),output/Path(__file__).name)
    plot_rules(groups,output)
    print(json.dumps(trials,indent=2),flush=True)


def plot_capture(folder,terrain,trial,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PatchCollection
    from matplotlib.patches import Rectangle
    fig,axes=plt.subplots(1,2,figsize=(11,4.5),layout='constrained')
    shot=folder/'approach-end.jpg'
    if shot.exists():axes[0].imshow(plt.imread(shot))
    else:axes[0].text(.5,.5,'Scene screenshot unavailable',ha='center')
    axes[0].axis('off');axes[0].set_title('Actual Gazebo capture')
    cells=[c for c in terrain.elevation() if c['valid']]
    patches=PatchCollection([Rectangle((c['ix']*.25,c['iy']*.25),.25,.25) for c in cells],cmap='gray_r',edgecolors='none')
    patches.set_array(np.array([c['z'] for c in cells]));patches.set_clim(-.05,.2)
    axes[1].add_collection(patches);axes[1].set(xlim=(-.5,5),ylim=(-3,3),aspect='equal',xlabel='X (m)',ylabel='Y (m)',title='Depth-derived valid surface heights')
    axes[1].plot(*np.array([trial['start'][:2],trial['goal']]).T,'k--',label='Predeclared segment');axes[1].legend(fontsize=8)
    fig.colorbar(patches,ax=axes[1],label='Estimated surface Z (m)',shrink=.7)
    fig.supxlabel('Blank cells have no valid fitted height. Geometry fixture, not hardware or real soil validation.',fontsize=8)
    fig.savefig(output/'scene-and-map.png',dpi=170);plt.close(fig)


def plot_rules(groups,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    selected=[g for g in groups if g['scope']=='all_initial_deficient' and g['condition']=='approach']
    fig,ax=plt.subplots(figsize=(8,4),layout='constrained')
    for i,g in enumerate(selected):
        bottom=0
        for key,hatch in (('tp',''),('fp','//'),('fn','..')):
            ax.bar(i,g[key],bottom=bottom,color='white',edgecolor='black',hatch=hatch,label={'tp':'Detected recovery','fp':'False opportunity','fn':'Missed recovery'}[key] if i==0 else None);bottom+=g[key]
    ax.set_xticks(range(len(selected)),[g['scene'].replace('visible_','').replace('_v1','')+'\n'+('Endpoint' if g['rule'].startswith('distance') else 'Planned path union') for g in selected])
    ax.set_ylabel('Initial deficient cells');ax.legend(fontsize=8)
    ax.set_title('Frozen rules on two independent geometry fixtures')
    fig.supxlabel('One moving trajectory per world; correlated cells, no probability calibration or efficacy claim.',fontsize=8)
    fig.savefig(output/'rule-check.png',dpi=180);fig.savefig(output/'rule-check.pdf');plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();report(a.root,a.output)
