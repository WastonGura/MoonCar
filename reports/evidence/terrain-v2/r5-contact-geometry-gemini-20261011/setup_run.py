"""Create only a new bounded test container; preserve all previous campaigns."""
import hashlib, json, shutil, subprocess
from pathlib import Path

root = Path(__file__).resolve().parents[4]
p = Path(__file__).resolve().parent
docker = shutil.which('docker') or r'C:\Users\jiker\AppData\Local\Programs\DockerDesktop\resources\bin\docker.exe'
name = 'mooncar-r5-contact-gemini-20261011'

def save(n, v): (p / n).write_text(json.dumps(v, indent=2) + '\n')
def inspect(n): return json.loads(subprocess.check_output([docker, 'inspect', n], text=True))[0]

save('workbench-before.json', inspect('mooncar-terrain-validation'))

# Record git state before running
git_rev = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
git_status = subprocess.check_output(['git', '-C', str(root), 'status', '--porcelain'], text=True)
save('git-before.json', dict(head=git_rev, porcelain=git_status))

protected = {}
for folder in ('scripts', 'containers', 'ros2_ws/src', 'docs', 'web', 'firmware'):
    for f in (root / folder).rglob('*'):
        if f.is_file() and not any(x in f.parts for x in ('__pycache__', 'node_modules')):
            protected[str(f.relative_to(root)).replace('\\', '/')] = hashlib.sha256(f.read_bytes()).hexdigest()
save('preservation-before.json', protected)

auth = {
    "answer": "允许这8次测试诊断",
    "budget": 8,
    "user_delegation_verbatim": "请执行 D:\\Documents\\Projects\\Repositories\\MoonCar-mapping\\docs\\tasks\\2026-10-11-gemini-r5-contact-diagnosis.md。\n按任务书调查网格接缝是否造成转向偏移，保留原始证据，并填写 GEMINI_REPLY.md。完成后提供证据目录绝对路径和提交号，由Codex独立验收。",
    "task_document": "docs/tasks/2026-10-11-gemini-r5-contact-diagnosis.md",
    "task_version": 1,
    "baseline_commit": "7a87fd1",
    "historical_compatibility_field_purpose": "Historical compatibility token required by scripts/run_r5_contact_geometry.py gate; not claimed as user verbatim reply",
    "previous_attempts_preserved": {
        "r5-contact-geometry-20261011": 1,
        "r5-contact-geometry-20261011-02": 3
    },
    "ground_collision_representation_only": True,
    "geometric_screen_failure_is_recorded_and_not_alone_stop": True,
    "no_rerun": True,
    "production_changed": False,
    "current_user_instruction": "请执行 D:\\Documents\\Projects\\Repositories\\MoonCar-mapping\\docs\\tasks\\2026-10-11-gemini-r5-contact-diagnosis.md",
    "authorization_basis": "User delegated 2026-10-11-gemini-r5-contact-diagnosis.md task book specifying 8 bounded diagnostic attempts",
    "new_campaign": True
}
save('authorization.json', auth)

save('prediction.json', dict(
    matrix=['plate L/R', 'coplanar mesh L/R', 'mirrored diagonals L/R', 'restore plate L/R'],
    upper_surface_z_m=0, step_s=.001, no_parameter_search=True,
    hypothesis='Coplanar mesh internal edges alter contacts and residual translation',
    support_requires='Valid input and capture in all rounds, restored plate reproduces plate; mesh representation/diagonal orientation changes contact geometry and displacement',
    limitation='Representation effect does not alone prove erroneous normals are the sole cause or a DART/ODE defect',
    budget=8
))

evidence_dir = root / 'reports/evidence/terrain-v2/r5-turn-gate-validation-20261011'
args = [
    docker, 'run', '-d', '--name', name, '--cpus', '4', '--memory', '6g', '--shm-size', '512m',
    '--mount', f'type=bind,source={root.as_posix()},target=/source,readonly',
    '--mount', f'type=bind,source={evidence_dir.as_posix()},target=/evidence,readonly',
    '--mount', f'type=bind,source={p.as_posix()},target=/probe',
    '-e', 'PYTHONPATH=/source/scripts',
    '-e', 'ROS_DOMAIN_ID=95',
    '-e', 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST',
    '-e', 'MOONCAR_IMAGE_DDS_RUNTIME=/opt/mooncar/image-dds/rmw-8.4.4',
    'mooncar-terrain:r5-image-dds-20261008',
    'bash', '-c', 'sleep infinity'
]
subprocess.run(args, check=True)
save('container-start.json', inspect(name))

subprocess.run([docker, 'exec', name, 'bash', '/source/scripts/prepare_r5_contact_geometry.sh'], check=True, timeout=2700)
print('Prepared new campaign without launching Gazebo.', flush=True)
