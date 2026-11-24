"""Create only a new bounded test container; preserve all previous campaigns."""
import hashlib,json,subprocess
from pathlib import Path

root=Path(__file__).resolve().parents[4]
p=Path(__file__).resolve().parent
docker=r'C:\Users\jiker\AppData\Local\Programs\DockerDesktop\resources\bin\docker.exe'
name='mooncar-r5-contact-geometry-20261011-02'
def save(n,v):(p/n).write_text(json.dumps(v,indent=2)+'\n')
def inspect(n):return json.loads(subprocess.check_output([docker,'inspect',n],text=True))[0]
save('workbench-before.json',inspect('mooncar-terrain-validation'))
protected={}
for folder in ('scripts','containers','ros2_ws/src','docs','web','firmware'):
    for f in (root/folder).rglob('*'):
        if f.is_file() and not any(x in f.parts for x in ('__pycache__','node_modules')):
            protected[str(f.relative_to(root))]=hashlib.sha256(f.read_bytes()).hexdigest()
save('preservation-before.json',protected)
auth=json.loads((root/'reports/evidence/terrain-v2/r5-contact-geometry-20261011/authorization.json').read_text())
auth.update(current_user_instruction='查明一下原因吧',authorization_basis='Renewed investigation request after discussion of the prepared bounded geometry comparison',new_campaign=True,old_attempts_preserved=1)
save('authorization.json',auth)
save('prediction.json',dict(matrix=['plate L/R','coplanar mesh L/R','mirrored diagonals L/R','restore plate L/R'],
    upper_surface_z_m=0,step_s=.001,no_parameter_search=True,
    hypothesis='Coplanar mesh internal edges alter contacts and residual translation',
    support_requires='Valid input and capture in all rounds, restored plate reproduces plate; mesh representation/diagonal orientation changes contact geometry and displacement',
    limitation='Representation effect does not alone prove erroneous normals are the sole cause or a DART/ODE defect',budget=8))
args=[docker,'run','-d','--name',name,'--cpus','4','--memory','6g','--shm-size','512m',
    '--mount',f'type=bind,source={root},target=/source,readonly',
    '--mount',f'type=bind,source={root}/reports/evidence/terrain-v2/r5-turn-gate-validation-20261011,target=/evidence,readonly',
    '--mount',f'type=bind,source={p},target=/probe',
    '-e','PYTHONPATH=/source/scripts','-e','ROS_DOMAIN_ID=95','-e','ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST',
    '-e','MOONCAR_IMAGE_DDS_RUNTIME=/opt/mooncar/image-dds/rmw-8.4.4',
    'mooncar-terrain:r5-image-dds-20261008','bash','-c','sleep infinity']
subprocess.run(args,check=True)
save('container-start.json',inspect(name))
subprocess.run([docker,'exec',name,'bash','/source/scripts/prepare_r5_contact_geometry.sh'],check=True,timeout=2700)
print('Prepared new campaign without launching Gazebo.',flush=True)
