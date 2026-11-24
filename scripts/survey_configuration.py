"""Read-only comparison of observation/time assumptions and declared/runtime settings."""
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from survey_contract import ROOT, digest, keys, number

FIELDS = {
    'horizontal_fov_deg': ('水平视场（度）', 1.),
    'camera_offset_xy_m': ('相机相对车体XY（米）', .01),
    'linear_speed_m_s': ('移动速度代理／控制器目标（米每秒）', .01),
    'angular_speed_rad_s': ('转动速度代理／配置值（弧度每秒）', .01),
    'footprint_size_m': ('安全包络长宽（米）', .01),
}


def camera_fov(camera):
    keys(camera, ('width', 'height', 'k'))
    w, h, k = camera['width'], camera['height'], camera['k']
    if type(w) is not int or type(h) is not int or min(w, h) <= 0 or not isinstance(k, list) or len(k) != 9:
        raise ValueError('Invalid CameraInfo dimensions/K')
    k = [number(v, 'K') for v in k]
    if min(k[0], k[4]) <= 0 or not 0 <= k[2] <= w or not 0 <= k[5] <= h:
        raise ValueError('Invalid CameraInfo focal lengths/principal point')
    if any(abs(k[i]) > 1e-9 for i in (1, 3, 6, 7)) or abs(k[8]-1) > 1e-9:
        raise ValueError('Only rectified zero-skew pinhole CameraInfo supported')
    # Continuous image edge approximation; rectified pinhole camera only.
    return math.degrees(math.atan((w - k[2]) / k[0]) + math.atan(k[2] / k[0]))


def audit_configuration(config):
    keys(config, ('format', 'expected', 'observed', 'scope'), ('source_files', 'notes'))
    if config['format'] != 'mooncar-survey-configuration-v1':
        raise ValueError('Unsupported configuration format')
    if config['scope'] not in ('source_declarations', 'runtime_snapshot', 'synthetic_fixture'):
        raise ValueError('Configuration requires an explicit evidence scope')
    expected, actual = config['expected'], dict(config['observed'])
    if not isinstance(expected, dict) or not isinstance(config['observed'], dict):
        raise ValueError('Expected/observed must be objects')
    if set(expected) - set(FIELDS) or set(actual) - set(FIELDS) - {'camera_info'}:
        raise ValueError('Unknown configuration quantities')
    if 'camera_info' in actual:
        derived = camera_fov(actual.pop('camera_info'))
        if 'horizontal_fov_deg' in actual and abs(number(actual['horizontal_fov_deg'], 'FOV') - derived) > 1:
            raise ValueError('Observed CameraInfo and declared FOV disagree')
        actual['horizontal_fov_deg'] = derived
    rows = []
    for field, (label, tolerance) in FIELDS.items():
        left, right = expected.get(field), actual.get(field)
        values = []
        for v in (left, right):
            if v is None:
                values.append(None)
            elif field in ('camera_offset_xy_m', 'footprint_size_m'):
                if not isinstance(v, list) or len(v) != 2:
                    raise ValueError(field + ' requires two components in the same declared frame')
                values.append([number(x, field, field == 'footprint_size_m') for x in v])
            else:
                n = number(v, field, True)
                if field == 'horizontal_fov_deg' and n >= 180:
                    raise ValueError('Pinhole FOV must be below 180 degrees')
                values.append(n)
        left, right = values
        difference = (max(abs(a-b) for a, b in zip(left, right)) if isinstance(left, list)
                      else abs(left-right)) if left is not None and right is not None else None
        state = 'unknown' if difference is None else 'difference' if difference > tolerance else 'consistent'
        rows.append(dict(field=field, label=label, expected=left, observed=right,
                         difference=difference, comparison_tolerance=tolerance, state=state))
    source_files = config.get('source_files', [])
    if not isinstance(source_files, list): raise ValueError('source_files must be a list')
    mismatches = []
    for row in source_files:
        keys(row, ('path', 'sha256'))
        path = Path(row['path'])
        if not path.is_file() or digest(path) != row['sha256']: mismatches.append(str(path))
    return dict(format='mooncar-survey-configuration-audit-v1', scope=config['scope'], checks=rows,
                differences=sum(r['state'] == 'difference' for r in rows),
                unknown=sum(r['state'] == 'unknown' for r in rows),
                status='source_changed' if mismatches else 'needs_review' if any(r['state'] != 'consistent' for r in rows) else 'consistent',
                source_mismatches=mismatches, source_files=source_files, notes=config.get('notes', []),
                meaning='只读核对：源码声明不代表实际加载或实测；差异不自动表示缺陷，一致也不代表测量或安全认证。')


def project_configuration(root=ROOT):
    """Extract only recognized current source declarations; never evaluate Xacro/Python."""
    root = Path(root)
    planner = root/'scripts/autonomous_survey.py'
    nav = root/'ros2_ws/src/mooncar_navigation/config/nav2_params.yaml'
    robot = root/'ros2_ws/src/mooncar_description/urdf/mooncar_r5.urdf.xacro'
    p, n, x = (path.read_text(encoding='utf-8') for path in (planner, nav, robot))
    expected, observed, notes = {}, {}, []
    if 'yaw-math.pi/6,yaw+math.pi/6' in p:
        expected['horizontal_fov_deg'] = 60.
    match = re.search(r'cx,cy=x\+([.\d]+)\*math.cos\(yaw\),y\+([.\d]+)\*math.sin\(yaw\)', p)
    if match and match[1] == match[2]: expected['camera_offset_xy_m'] = [float(match[1]), 0.]
    match = re.search(r'seconds=distance/([.\d]+)\+.*?/([.\d]+)\+5', p)
    if match:
        expected['linear_speed_m_s'], expected['angular_speed_rad_s'] = map(float, match.groups())
    # Confirm this is the camera macro argument, not an unrelated visual angle.
    tree = ET.fromstring(x)
    cameras = [node for node in tree.iter() if node.tag.endswith('mooncar_gazebo_sensors')]
    if cameras:
        match = re.fullmatch(r'\$\{([\d.]+) \* pi / 180\}', cameras[0].get('depth_hfov', ''))
        if match: observed['horizontal_fov_deg'] = float(match[1])
    match = re.search(r'^\s*desired_linear_vel:\s*([\d.]+)\s*$', n, re.M)
    if match: observed['linear_speed_m_s'] = float(match[1])
    footprints = re.findall(r'^\s*footprint:\s*"(\[.*\])"\s*$', n, re.M)
    if footprints:
        polygons = [json.loads(v) for v in footprints]
        if polygons.count(polygons[0]) == len(polygons):
            points = polygons[0]
            expected['footprint_size_m'] = [max(v[i] for v in points)-min(v[i] for v in points) for i in (0, 1)]
    notes += ['规划器与R5/导航源码声明的对照，未启动运行；未提供实际TF、实测速度或碰撞扫掠。',
              'footprint只读到导航声明，缺少独立碰撞包络时保持未知，不自动缩小安全余量。',
              '比较容差为报告差异筛查规则，未改运行参数或项目精度门槛。']
    return dict(format='mooncar-survey-configuration-v1', expected=expected, observed=observed,
                scope='source_declarations', notes=notes,
                source_files=[dict(path=str(path.resolve()), sha256=digest(path)) for path in (planner, nav, robot)])
