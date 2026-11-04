"""Validation-only shipped ramp world and target roster; no estimator edits."""
from pathlib import Path
import hashlib,json,math,time,os
import xml.etree.ElementTree as ET
import numpy as np
import autonomous_survey as survey
import experiment_scenes as scenes
import simulation_experiment as experiment

ROOT=Path('/source');PACKAGE=ROOT/'ros2_ws/src/mooncar_gazebo'
RAMP=PACKAGE/'worlds/mooncar_ramp_v2.sdf'
OUT=Path(os.environ['MOONCAR_VALIDATION_OUTPUT']);OUT.mkdir(parents=True,exist_ok=True)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
from ament_index_python.packages import get_package_share_directory
installed=Path(get_package_share_directory('mooncar_gazebo'))
assets={RAMP}
for node in ET.parse(RAMP).iter():
    if node.tag in ('uri','albedo_map','normal_map','roughness_map'):
        assets.add(scenes._resource(node.text))
for source in assets:
    assert sha(source)==sha(installed/source.relative_to(PACKAGE)),source
original_bounds=scenes.scene_bounds;original_world=experiment.survey_world;original_meta=experiment.scene_metadata
def bounds(scene):return (-8.,-8.,8.,8.) if scene=='survey_v2_native' else original_bounds(scene)
def world(scene='survey'):
    return ET.parse(RAMP) if scene=='survey_v2_native' else original_world(scene)
def metadata(scene,mode='terrain'):
    result=original_meta(scene,mode)
    if scene=='survey_v2_native':
        result.update(world_name='mooncar_ramp_v2',canonical_world='worlds/mooncar_ramp_v2.sdf',
            canonical_world_sha256=sha(RAMP),validation_fixture='shipped_ramp_v2_fixed_targets',
            source_description='Validation-only shipped Terrain V2 ramp, no pillars; original window localization and navigation protections; fixed test goals, not natural choose_view')
    return result
def spawn(x,y,yaw,**kwargs):
    assert (x,y,yaw)==(0.,0.,0.),'Ramp fixture permits only declared flat start'
    low=np.array([-.53,-.46]);high=-low;points=[];area=0.;slopes=[]
    for model in ET.parse(RAMP).findall('./world/model'):
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                mesh=collision.find('geometry/mesh')
                if mesh is None:continue
                path=scenes._resource(mesh.findtext('uri'))
                if path.parent!=PACKAGE/'models/terrain_v2/ramp' or path.name=='surface_pebbles.obj':continue
                vertices,faces=scenes._obj(path)
                matrix=scenes._transform(model)@scenes._transform(link)@scenes._transform(collision)
                scale=np.array(list(map(float,mesh.findtext('scale','1 1 1').split())))
                vertices=(vertices*scale)@matrix[:3,:3].T+matrix[:3,3]
                triangles=vertices[faces]
                for triangle in triangles[np.all(triangles[:,:,:2].max(axis=1)>=low,axis=1)&np.all(triangles[:,:,:2].min(axis=1)<=high,axis=1)]:
                    polygon=np.asarray(scenes._clip(triangle,low,high))
                    if len(polygon)<3:continue
                    piece=abs(float(np.sum(polygon[:,0]*np.roll(polygon[:,1],-1)-polygon[:,1]*np.roll(polygon[:,0],-1))))/2
                    if piece<1e-12:continue
                    normal=np.cross(triangle[1]-triangle[0],triangle[2]-triangle[0])
                    slopes.append(math.degrees(math.atan2(np.linalg.norm(normal[:2]),abs(normal[2]))))
                    area+=piece;points.extend(polygon)
    points=np.asarray(points);assert len(points) and abs(area-np.prod(high-low))<1e-6
    assert max(slopes)<=5 and np.ptp(points[:,2])<=.05
    return dict(initial_z=float(points[:,2].max()),ground_height_range_m=[float(points[:,2].min()),float(points[:,2].max())],
        maximum_ground_slope_deg=max(slopes),setup_only=True,truth_inputs_to_navigation=False,validation_fixture='actual_ramp_mesh_flat_start')

scenes.scene_bounds=experiment.scene_bounds=survey.scene_bounds=bounds
experiment.survey_world=world;experiment.scene_metadata=metadata;experiment.validate_v2_spawn=spawn
ROUTE=[('move',1.,0.),('move',2.5,0.),('move',4.,0.),('turn',math.pi/2),('turn',0.),
       ('move',5.5,0.),('turn',math.pi),('move',4.,0.),('move',2.5,0.),('move',1.,0.)]
def run(self):
    self.wall_started=time.monotonic();self.data['validation_fixture']='Fixed shipped-V2 ramp ascent, midslope turns and descent; production scan/execute/health/permit; choose_view not evaluated'
    self.data['validation_route']=[list(p) for p in ROUTE]
    try:
        if not self.initial_depth_observation():return
        for item in ROUTE:
            if not self.healthy():return
            record=dict(kind='advance' if item[0]=='move' else 'turn',action=list(item),started_seconds=self.now()-self.data['started'],outcome='running',success=False)
            self.data['history'].append(record);self.save()
            decision=dict(estimated_seconds=30.)
            if item[0]=='move':decision.update(x=item[1],y=item[2])
            else:decision['yaw']=item[1]
            success=self.execute(decision,turn=item[0]=='turn')
            record["decision"]=dict(decision)
            record.update(success=bool(success),outcome='succeeded' if success else 'failed',end_sim=self.now());self.save()
            if not success:
                if self.data['active']:self.stop('坡面固定动作未完成，已安全停止','fault')
                return
            until=self.now()+2
            while self.healthy() and self.now()<until:time.sleep(.1)
        self.stop('上坡、坡中转向和下坡固定任务完成','completed')
    except Exception as error:self.stop('坡面验证异常：'+str(error),'fault')
    finally:
        if self.data['active']:self.stop('坡面验证结束','fault')
        try:
            if self.recorder:self.recorder.finish()
        except Exception as error:self.data['evidence_error']=str(error)
        self.data['validation_finished']=True;self.save()
survey.AutonomousSurvey.run=run
proof=dict(world=str(RAMP),world_name='mooncar_ramp_v2',world_sha256=sha(RAMP),assets={str(p.relative_to(PACKAGE)):sha(p) for p in sorted(assets)},
    fixed_route=[list(p) for p in ROUTE],spawn=spawn(0.,0.,0.),bounds=list(bounds('survey_v2_native')),
    policy='Validation backend fixture only. Actual shipped geometry; no truth/known pose input to estimator, no direct velocity command, all normal motion checks retained.')
(OUT/'ramp-fixture.json').write_text(json.dumps(proof,indent=2)+'\n')
