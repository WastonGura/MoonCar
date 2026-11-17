"""Fail-closed loader for the project DART repair; legacy worlds are untouched."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

PLUGIN_NAME = 'libmooncar-physics7-dartsim.so'


def resolve_control_profile(world_path, model_file, requested='auto'):
    if requested not in ('auto','legacy','r5_dart_v2'):
        raise RuntimeError('Invalid sim_control_profile: '+requested)
    world=ET.parse(world_path).find('world')
    engine=world.findtext("plugin[@name='gz::sim::systems::Physics']/engine/filename",'')
    compatible=(model_file=='mooncar_r5.urdf.xacro' and world.get('name') in
                ('mooncar_ramp_v2','mooncar_curve_v2','mooncar_survey_v2','mooncar_survey_v3') and Path(engine).name==PLUGIN_NAME)
    if requested=='r5_dart_v2' and not compatible:
        raise RuntimeError('r5_dart_v2 control requires R5 and a project DART v2 world')
    return ('r5_dart_v2' if compatible else 'legacy') if requested=='auto' else requested


def prepare_world(world_path, plugin_root, package_root, runtime_version=None):
    world_path = Path(world_path)
    tree = ET.parse(world_path)
    filename = tree.find("./world/plugin[@name='gz::sim::systems::Physics']/engine/filename")
    if filename is None or filename.text != PLUGIN_NAME:
        return str(world_path)
    if not plugin_root:
        raise RuntimeError('v2 DART requires MOONCAR_DART_PLUGIN_ROOT; no backend fallback is allowed')
    root = Path(plugin_root).resolve()
    library = root/PLUGIN_NAME
    manifest_path = root/'manifest.json'
    if not library.is_file() or not manifest_path.is_file():
        raise RuntimeError('Project DART library or manifest missing: '+str(root))
    manifest = json.loads(manifest_path.read_text())
    if runtime_version is None:
        from ament_index_python.packages import get_package_prefix
        prefix = Path(get_package_prefix('gz_physics_vendor'))
        # Jazzy vendors also list other Gazebo vendors as pkg-config Requires.
        # Sourcing ROS does not export those pkg-config paths automatically.
        pc_paths = sorted((prefix/'opt').glob('*/lib/pkgconfig'))
        env = dict(os.environ, PKG_CONFIG_PATH=':'.join(map(str, pc_paths))+':'+os.environ.get('PKG_CONFIG_PATH',''))
        runtime_version = subprocess.check_output(
            ['pkg-config','--modversion','gz-physics7'],text=True,env=env).strip()
    version = runtime_version
    if manifest['gz_physics_version'] != version:
        raise RuntimeError(f'DART build/runtime version mismatch: {manifest["gz_physics_version"]} / {version}')
    if hashlib.sha256(library.read_bytes()).hexdigest() != manifest['library_sha256']:
        raise RuntimeError('Project DART library checksum mismatch')
    filename.text = str(library)
    # ConstructSdfCollision needs resolved paths so its friction import handles
    # the very same collision mesh. Do not change mesh geometry or surfaces.
    for uri in tree.findall('.//collision/geometry/mesh/uri'):
        if uri.text.startswith('model://mooncar_gazebo/'):
            path = Path(package_root)/uri.text.removeprefix('model://mooncar_gazebo/')
        else:
            path = Path(uri.text)
            if not path.is_absolute():
                path = world_path.parent/path
        if not path.is_file():
            raise RuntimeError('Unresolved DART collision mesh: '+uri.text)
        uri.text = str(path.resolve())
    if tree.findall('.//collision/geometry/mesh/submesh'):
        raise RuntimeError('Project DART contact import does not support submeshes')
    with tempfile.NamedTemporaryFile(prefix='mooncar-v2-',suffix='.sdf',delete=False) as temp:
        tree.write(temp,encoding='utf-8',xml_declaration=True)
        return temp.name
