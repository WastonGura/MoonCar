"""Setup-only scene geometry; never an observation or a navigation map.

Terrain V2 uses continuous collision meshes, including two fine measured patches.
Spawn checks therefore read the written triangles rather than treating the ground
as one obstacle, or replacing its fine patches with the coarse height grid.
"""
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1] / 'ros2_ws/src/mooncar_gazebo'
V2_WORLD = PACKAGE / 'worlds/mooncar_survey_v2.sdf'
V3_WORLD = PACKAGE / 'worlds/mooncar_survey_v3.sdf'
V2_SCENES = ('survey_v2', 'survey_v2_native')
V3_SCENES = ('survey_v3_native',)
CONTINUOUS_SCENES = (*V2_SCENES,*V3_SCENES)
SCENES = ('survey', *CONTINUOUS_SCENES, 'visible_ramp_v1', 'visible_curve_v1')


def scene_bounds(scene):
    if scene not in SCENES:
        raise ValueError('Unknown experiment scene')
    return (-15., -15., 15., 15.) if scene in CONTINUOUS_SCENES else (-25., -25., 25., 25.)


def scene_metadata(scene, localization_mode='terrain'):
    """Describe the selected physical scene and actual observation backend."""
    bounds = scene_bounds(scene)
    sources = {
        'rgbd': 'RGB 纹理、米制深度与 IMU 在线定位、深度观测采集',
        'rgbd_window': '多关键帧局部地图与滑动窗口联合定位、深度观测采集（不依赖主动回访）',
        'terrain': '3D 深度与 IMU 地形定位、深度观测采集',
        'direct': '二维激光与 IMU 在线定位、深度观测采集',
        'ekf': '二维激光、轮式里程计与 IMU 融合定位、深度观测采集',
        None: '轮式里程计与 IMU 融合、激光在线建图、深度观测采集',
    }
    if localization_mode not in sources:
        raise ValueError('Unknown experiment localization mode')
    description = 'Gazebo 50×50 米仿真'
    if scene in V2_SCENES:
        fixtures = '追加四个定位标柱' if scene == 'survey_v2' else '原生地形，无追加定位标柱'
        description = f'Gazebo Terrain V2 survey 30×30 米（{fixtures}）'
    if scene in V3_SCENES:description='Gazebo Terrain V3 连续地形 30×30 米（无追加定位标柱）'
    world_name='mooncar_survey_v3' if scene in V3_SCENES else 'mooncar_survey_v2' if scene in V2_SCENES else 'lunar_yard'
    result = dict(scene=scene, world_name=world_name,
                  bounds=list(bounds), localization_mode=localization_mode or 'baseline',
                  camera_sensor_profile='rgbd_v1' if localization_mode in ('rgbd','rgbd_window') else 'legacy',
                  source_description=f'{description}；{sources[localization_mode]}；历史深度位姿未做回环修正')
    if scene in V2_SCENES:
        result.update(canonical_world='worlds/mooncar_survey_v2.sdf',
                      canonical_world_sha256=hashlib.sha256(V2_WORLD.read_bytes()).hexdigest(),
                      localization_fixture_profile=('terrain_v2_workbench_markers_v1'
                                                    if scene == 'survey_v2' else 'none'))
    if scene in V3_SCENES:
        result.update(canonical_world='worlds/mooncar_survey_v3.sdf',canonical_world_sha256=hashlib.sha256(V3_WORLD.read_bytes()).hexdigest(),
                      terrain_version='terrain-v3-continuous-v1',localization_fixture_profile='none')
    return result


def _resource(uri):
    prefix = 'model://mooncar_gazebo/'
    if not uri or not uri.startswith(prefix):
        raise ValueError('Unsupported Terrain V2 resource')
    path = (PACKAGE / uri[len(prefix):]).resolve()
    if PACKAGE.resolve() not in path.parents or not path.is_file():
        raise ValueError('Missing Terrain V2 resource: ' + uri)
    return path


def verify_installed_scene_resources(installed_package):
    """The setup geometry and the resources actually loaded by Gazebo must agree."""
    installed_package = Path(installed_package)
    resources = {PACKAGE / 'worlds/lunar_yard.sdf', V2_WORLD,V3_WORLD}
    for world in (V2_WORLD,V3_WORLD):
        for element in ET.parse(world).iter():
            if element.tag in ('uri', 'albedo_map', 'normal_map', 'roughness_map'):
                resources.add(_resource(element.text))
    for version in ('v2','v3'):
        resources.update(PACKAGE / f'models/terrain_{version}/survey' / name
                         for name in ('manifest.json', 'height_grid.npz', 'class_mask.png'))
    for source in sorted(resources):
        installed = installed_package / source.relative_to(PACKAGE)
        if (not installed.is_file() or hashlib.sha256(installed.read_bytes()).digest() !=
                hashlib.sha256(source.read_bytes()).digest()):
            raise ValueError('已构建的 Gazebo 场景资源与当前代码不一致，请先重新构建 ROS 工作区：' +
                             source.relative_to(PACKAGE).as_posix())


def _transform(element):
    pose = element.find('pose')
    if pose is not None and pose.attrib:
        raise ValueError('Unsupported Terrain V2 pose reference')
    x, y, z, r, p, a = map(float, element.findtext('pose', '0 0 0 0 0 0').split())
    cr, sr, cp, sp, ca, sa = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(a), math.sin(a)
    result = np.eye(4)
    result[:3, :3] = [[ca*cp, ca*sp*sr-sa*cr, ca*sp*cr+sa*sr],
                       [sa*cp, sa*sp*sr+ca*cr, sa*sp*cr-ca*sr], [-sp, cp*sr, cp*cr]]
    result[:3, 3] = [x, y, z]
    return result


def _obj(path):
    vertices, faces = [], []
    for line in path.read_text(encoding='utf-8').splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == 'v':
            vertices.append(list(map(float, fields[1:4])))
        elif fields[0] == 'f':
            if len(fields) != 4:
                raise ValueError('Terrain V2 spawn geometry requires triangular faces')
            faces.append([int(value.split('/')[0])-1 for value in fields[1:]])
    vertices, faces = np.asarray(vertices), np.asarray(faces, dtype=int)
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all()
            or faces.ndim != 2 or np.any(faces < 0) or np.any(faces >= len(vertices))):
        raise ValueError('Invalid Terrain V2 collision mesh')
    return vertices, faces


def _component_bounds(vertices, faces):
    """Merged pebble meshes contain many separate rocks, not one giant obstacle."""
    unique, inverse = np.unique(vertices, axis=0, return_inverse=True)
    parent = list(range(len(unique)))
    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index
    for face in inverse[faces]:
        root = find(int(face[0]))
        for index in face[1:]:
            parent[find(int(index))] = root
    groups = {}
    for index, point in enumerate(unique):
        groups.setdefault(find(index), []).append(point)
    return [(np.min(points, axis=0)[:2], np.max(points, axis=0)[:2]) for points in groups.values()]


@lru_cache(maxsize=2)
def _v2_geometry(version='v2'):
    ground, classes, obstacles = [], [], []
    folder=PACKAGE/f'models/terrain_{version}/survey'
    manifest = json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
    ground_files = set(manifest['mesh_sha256']) - {'surface_pebbles.obj'}
    for model in ET.parse(V3_WORLD if version=='v3' else V2_WORLD).findall('./world/model'):
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                mesh = collision.find('geometry/mesh')
                if mesh is None:
                    raise ValueError('Unsupported Terrain V2 collision geometry')
                path = _resource(mesh.findtext('uri'))
                vertices, faces = _obj(path)
                scale = np.asarray(list(map(float, mesh.findtext('scale', '1 1 1').split())))
                matrix = _transform(model) @ _transform(link) @ _transform(collision)
                vertices = (vertices * scale) @ matrix[:3, :3].T + matrix[:3, 3]
                if path.parent == folder and path.name in ground_files:
                    ground.append(vertices[faces])
                    classes.extend([path.stem] * len(faces))
                elif path.name == 'surface_pebbles.obj':
                    obstacles.extend(_component_bounds(vertices, faces))
                else:
                    obstacles.append((vertices.min(axis=0)[:2], vertices.max(axis=0)[:2]))
    triangles = np.concatenate(ground)
    return triangles, np.asarray(classes), obstacles, triangles[:, :, :2].min(axis=1), triangles[:, :, :2].max(axis=1)


def v2_spawn_obstacles(*, include_markers=True,scene='survey_v2'):
    obstacles = [(low.copy(), high.copy()) for low, high in _v2_geometry('v3' if scene in V3_SCENES else 'v2')[2]]
    for marker in v2_marker_metadata() if include_markers else ():
        center, size = np.asarray(marker['position_m']), np.asarray(marker['size_m'])
        obstacles.append((center[:2]-size[:2]/2, center[:2]+size[:2]/2))
    return obstacles


@lru_cache(maxsize=1)
def v2_marker_metadata():
    """Real scan targets: the low V2 relief lies below the unchanged lidar.

    Only the generated workbench world gains these explicitly named fixtures.
    Their known poses are never supplied to localization or the terrain mapper.
    """
    triangles, _, _, minimum, maximum = _v2_geometry()
    result = []
    # Keep both measured CE4 patches clear; these are separate engineered aids.
    for index, (x, y) in enumerate(((-7., -5.), (-7., 4.), (3., 7.), (8., -1.))):
        low, high = np.array([x-.5, y-.5]), np.array([x+.5, y+.5])
        selected = np.flatnonzero(np.all(maximum >= low, axis=1) & np.all(minimum <= high, axis=1))
        points = [point for triangle in triangles[selected] for point in _clip(triangle, low, high)]
        if not points:
            raise ValueError('Localization marker has no Terrain V2 ground')
        ground_min = float(np.asarray(points)[:, 2].min())
        bottom = ground_min-.02
        result.append(dict(name=f'wb_localization_marker_{index}', position_m=[x, y, bottom+.5],
                           size_m=[1., 1., 1.], ground_min_m=ground_min,
                           purpose='physical laser return fixture; no known-pose input to localization'))
    return tuple(result)


def add_v2_localization_markers(tree):
    world = tree.find('world')
    if world is None or world.get('name') != 'mooncar_survey_v2':
        raise ValueError('Localization fixtures require the Terrain V2 survey world')
    for marker in v2_marker_metadata():
        if world.find(f"model[@name='{marker['name']}']") is not None:
            raise ValueError('Duplicate workbench localization fixture')
        model = ET.SubElement(world, 'model', name=marker['name'])
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = ' '.join(map(str, marker['position_m']))+' 0 0 0'
        link = ET.SubElement(model, 'link', name='marker')
        for kind in ('collision', 'visual'):
            element = ET.SubElement(link, kind, name='marker_'+kind)
            shape = ET.SubElement(ET.SubElement(element, 'geometry'), 'box')
            ET.SubElement(shape, 'size').text = ' '.join(map(str, marker['size_m']))
            if kind == 'visual':
                material = ET.SubElement(element, 'material')
                ET.SubElement(material, 'ambient').text = '0.25 0.32 0.38 1'
                ET.SubElement(material, 'diffuse').text = '0.35 0.42 0.48 1'
    return tree


def _clip(triangle, low, high):
    polygon = list(triangle)
    for axis in (0, 1):
        for limit, sign in ((low[axis], 1), (high[axis], -1)):
            clipped = []
            if not polygon:
                return []
            previous = polygon[-1]
            for point in polygon:
                inside = sign * (point[axis] - limit) >= -1e-10
                was_inside = sign * (previous[axis] - limit) >= -1e-10
                if inside != was_inside:
                    fraction = (limit - previous[axis]) / (point[axis] - previous[axis])
                    clipped.append(previous + fraction * (point - previous))
                if inside:
                    clipped.append(point)
                previous = point
            polygon = clipped
    return polygon


def validate_v2_spawn(x, y, yaw, *, include_markers=True,scene='survey_v2'):
    """Require a supported, gently graded regolith start before any process starts.

    These conservative setup limits only select the initial parked position;
    they do not change navigation or localization acceptance thresholds.
    """
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError('起点必须为有限数值')
    hx = .43*abs(math.cos(yaw)) + .36*abs(math.sin(yaw)) + .1
    hy = .43*abs(math.sin(yaw)) + .36*abs(math.cos(yaw)) + .1
    low, high = np.array([x-hx, y-hy]), np.array([x+hx, y+hy])
    if np.any(low < -15) or np.any(high > 15):
        raise ValueError('起点车体超出 Terrain V2 地表边界，请选择周围平地')
    triangles, classes, _, minimum, maximum = _v2_geometry('v3' if scene in V3_SCENES else 'v2')
    for obstacle_low, obstacle_high in v2_spawn_obstacles(include_markers=include_markers,scene=scene):
        if np.all(high >= obstacle_low) and np.all(low <= obstacle_high):
            raise ValueError('起点车体与 Terrain V2 岩石或碎石重叠，请选择周围平地')
    selected = np.flatnonzero(np.all(maximum >= low, axis=1) & np.all(minimum <= high, axis=1))
    points, covered, slopes, ground_classes = [], 0., [], set()
    for index in selected:
        triangle = triangles[index]
        polygon = _clip(triangle, low, high)
        if len(polygon) < 3:
            continue
        polygon = np.asarray(polygon)
        area = abs(float(np.sum(polygon[:, 0]*np.roll(polygon[:, 1], -1) -
                                polygon[:, 1]*np.roll(polygon[:, 0], -1)))) / 2
        if area < 1e-12:
            continue
        normal = np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0])
        slopes.append(math.degrees(math.atan2(float(np.linalg.norm(normal[:2])), abs(float(normal[2])))))
        covered += area
        points.extend(polygon)
        ground_classes.add(str(classes[index]))
    expected = float(np.prod(high-low))
    if not points or abs(covered-expected) > max(1e-6, expected*1e-6):
        raise ValueError('起点车体没有完整连续地表支承，请选择周围平地')
    points = np.asarray(points)
    height_range = float(np.ptp(points[:, 2]))
    design = np.column_stack((points[:, :2]-[x, y], np.ones(len(points))))
    plane = np.linalg.lstsq(design, points[:, 2], rcond=None)[0]
    residual = float(np.max(np.abs(design @ plane-points[:, 2])))
    safe_classes = {'regolith_passable', 'ce4_crater_patch', 'ce4_background_regolith_passable'}
    if (not ground_classes <= safe_classes or max(slopes) > 5. or height_range > .05 or residual > .02):
        raise ValueError('起点车体位于陡坡、起伏或不稳定地表，请选择周围平地')
    return dict(initial_z=float(points[:, 2].max()), ground_height_range_m=[float(points[:, 2].min()), float(points[:, 2].max())],
                maximum_ground_slope_deg=max(slopes), ground_plane_residual_m=residual,
                ground_classes=sorted(ground_classes), clearance_margin_m=.1,
                setup_only=True, truth_inputs_to_navigation=False)
