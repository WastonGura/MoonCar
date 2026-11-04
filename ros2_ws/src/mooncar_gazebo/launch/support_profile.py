"""Opt-in frozen R5 support candidate, independent of adaptive steering release."""
import hashlib
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

PROFILE = 'r5_support_v1'
LIBRARIES = {
    'libode.so.8.0.2': '313dbad175a34e0fd0387792bc922766b5e7727c3883ede5b84322cd055140c7',
    'libmooncar-axle-allocation.so': 'd85a756cc12d950f92ba405c60bd45ba71c989ae4554c2bb05d14792905b069d',
    'libmooncar-turn-slip-control.so': 'b99d96651a8175e6b6eacfa10bd7b1d65eb88d61b56cbf802a036048d7dd726c',
}
DART_SHA = 'cfe9b7c6bf72c204645df81f1be7bc646ffbd342acea52c0684a1ad24848274c'

def verify(root):
    root = Path(root).resolve()
    manifest = json.loads((root/'manifest.json').read_text())
    for name, digest in LIBRARIES.items():
        if manifest['libraries'].get(name) != digest or hashlib.sha256((root/name).read_bytes()).hexdigest() != digest:
            raise RuntimeError('Support candidate runtime checksum mismatch: '+name)
    if hashlib.sha256(Path(manifest['dart_path']).read_bytes()).hexdigest() != DART_SHA:
        raise RuntimeError('Support candidate DART checksum mismatch')
    if not (root/'libode.so.8').is_symlink() or (root/'libode.so.8').resolve() != root/'libode.so.8.0.2':
        raise RuntimeError('Support candidate ODE SONAME mismatch')
    return dict(manifest, libraries=LIBRARIES, dart_sha256=DART_SHA)

def prepare_world(path):
    tree = ET.parse(path)
    world = tree.find('world')
    if world is None or world.find('physics') is None:
        raise RuntimeError('Support candidate requires a world with explicit physics')
    if tuple(map(float, world.findtext('gravity', '').split())) != (0., 0., -1.62):
        raise RuntimeError('Support candidate requires lunar gravity')
    if world.find("model[@name='mooncar']") is not None:
        raise RuntimeError('Support launch must spawn its bound robot description')
    for field, value in [('max_step_size', '0.00025'), ('real_time_update_rate', '4000')]:
        node = world.find('physics/'+field)
        if node is None: node = ET.SubElement(world.find('physics'), field)
        node.text = value
    system = world.find("plugin[@name='gz::sim::systems::Physics']")
    if system is None: raise RuntimeError('Physics system missing')
    engine = system.find('engine')
    if engine is None: engine = ET.SubElement(system, 'engine')
    filename = engine.find('filename')
    if filename is None: filename = ET.SubElement(engine, 'filename')
    filename.text = 'libmooncar-physics7-dartsim.so'
    for uri in tree.findall('.//mesh/uri'):
        if '://' not in uri.text and not Path(uri.text).is_absolute():
            uri.text = str((Path(path).resolve().parent/uri.text).resolve())
    with tempfile.NamedTemporaryFile(prefix='mooncar-support-', suffix='.sdf', delete=False) as output:
        tree.write(output, encoding='utf-8', xml_declaration=True)
        return output.name

def augment_urdf(text, root):
    from turn_guard import augment_urdf as adaptive_model
    robot = ET.fromstring(adaptive_model(text, root))
    extension = robot[-1]
    extension.remove(extension.find("plugin[@name='mooncar::AdaptiveAxleAllocation']"))
    allocation = ET.SubElement(extension, 'plugin', filename=str(Path(root)/'libmooncar-axle-allocation.so'), name='mooncar::AxleAllocation')
    for name, value in [('enabled', 'true'), ('front_factor', '0.96026822178862603'), ('rear_factor', '1.039731778211374')]:
        ET.SubElement(allocation, name).text = value
    return ET.tostring(robot, encoding='unicode')
