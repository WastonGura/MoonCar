import sys,json,hashlib,datetime
from pathlib import Path
import numpy as np
sys.path.insert(0,'/source/scripts')
from simulation_experiment import survey_world,validate_task,transform
from trajectory_evaluation import camera_extrinsic
from terrain_reference import read_shapes
from depth_projection_evaluation import first_hit
from surface_quality_evaluation import surface_labels
from collections import Counter,defaultdict
import xml.etree.ElementTree as ET
root=Path('/evidence/visible-path-trials-20260913');root.mkdir(exist_ok=False)
trials=[dict(scene='visible_ramp_v1',start=[0,-.6,.463648],goal=[.8,-.2],seconds=25),dict(scene='visible_curve_v1',start=[0,.6,-.463648],goal=[.8,.2],seconds=25)]
plan=dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),trials=trials,conditions=['stationary','approach'],baseline_seconds=5,path_spacing_m=.1,rule='Union of frozen range+direction rule across planned straight segment; deduplicate cell IDs',endpoint_rule='Same unchanged range+direction rule at planned goal',split='Two prospectively fixed independent world geometries; no fitted parameters or policy tuning',limits=['Straight planned segment is not Nav2 global path; measured deviation is evaluation only','Correlated cells; one pair per world, no efficacy significance claim','World and true poses used only for evaluator visibility and setup checks'],source_sha256={name:hashlib.sha256((Path('/source')/name).read_bytes()).hexdigest() for name in ['scripts/simulation_experiment.py','scripts/resampling_dataset.py','scripts/capture_slope_resampling.py','scripts/autonomous_survey.py','ros2_ws/src/mooncar_terrain/scripts/terrain_map.py']})
(root/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
extrinsic,_=camera_extrinsic(Path('/source/reports/evidence/terrain-workbench/projection-20260913/fixed-motion/robot.sdf'))
info=json.loads(Path('/source/reports/evidence/terrain-workbench/projection-20260913/fixed-motion/camera-info.json').read_text());fx,fy,cx,cy=[info['k'][i] for i in (0,4,2,5)]
v,u=np.mgrid[0:240:4,0:320:4];rays=np.column_stack(((u.ravel()+.5-cx)/fx,(v.ravel()+.5-cy)/fy,np.ones(u.size)))
checks=[]
for trial in trials:
    scene=trial['scene'];world=root/(scene+'.sdf');survey_world(scene).write(world)
    shapes=read_shapes(world)
    for phase,xy in [('start',trial['start'][:2]),('goal',trial['goal'])]:
        validate_task(dict(version=1,region=[-25,-25,25,25],start=[*xy,trial['start'][2]],scene=scene))
        model=ET.fromstring(f'<model><pose>{xy[0]} {xy[1]} 0 0 0 {trial["start"][2]}</pose></model>')
        camera=transform(model)@extrinsic
        distances,owners=first_hit(shapes,camera[:3,3],rays@camera[:3,:3].T)
        valid=(owners>=0)&(distances>=.15)&(distances<=6)
        hits=(rays[valid]*distances[valid,None])@camera[:3,:3].T+camera[:3,3]
        labels=surface_labels(shapes,owners[valid],hits);seen=defaultdict(set)
        for point,label in zip(hits,labels):seen[tuple(np.floor(point[:2]/.25).astype(int))].add(label)
        target='sloped_top' if 'ramp' in scene else 'curved_top'
        pure=sum(len(labels)==1 and next(iter(labels))[2]==target for labels in seen.values())
        checks.append(dict(scene=scene,phase=phase,point_categories=dict(Counter(label[2] for label in labels)),target=target,pure_target_cells= pure,camera_height=float(camera[2,3])))
        assert pure>0,(scene,phase,'Target not visible')
(root/'visibility-precheck.json').write_text(json.dumps(checks,indent=2)+'\n')
print(json.dumps(checks,indent=2))

