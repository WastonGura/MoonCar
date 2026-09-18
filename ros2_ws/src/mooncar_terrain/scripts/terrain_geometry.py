"""Geometry-only traversability prototype; thresholds require rover calibration."""
import math

import numpy as np


def project_depth(data, width, height, step, encoding, bigendian, intrinsics, stride=4,
                  pixel_offset=0.0):
    if width <= 0 or height <= 0 or stride < 1:
        raise ValueError('Invalid image dimensions or stride')
    if not math.isfinite(pixel_offset) or abs(pixel_offset) > 1:
        raise ValueError('Pixel offset must be finite and within one pixel')
    if encoding not in ('32FC1', '16UC1'):
        raise ValueError(f'Unsupported depth encoding: {encoding}')
    dtype = np.dtype(('>' if bigendian else '<') + ('f4' if encoding == '32FC1' else 'u2'))
    if step < width * dtype.itemsize or len(data) < step * height:
        raise ValueError('Truncated depth image')
    fx, fy, cx, cy = intrinsics
    if not all(math.isfinite(v) for v in intrinsics) or min(fx, fy) <= 0:
        raise ValueError('Invalid camera intrinsics')
    depth = np.ndarray((height, width), dtype=dtype, buffer=data,
                       strides=(step, dtype.itemsize))[::stride, ::stride].astype(float)
    if encoding == '16UC1':
        depth *= 0.001
    v, u = np.mgrid[0:height:stride, 0:width:stride]
    valid = np.isfinite(depth) & (depth >= 0.1) & (depth <= 6.0)
    z = depth[valid]
    # Gazebo Ogre2 uses half-pixel sample centers; real-camera intrinsics default to zero.
    return np.column_stack(((u[valid] + pixel_offset - cx) * z / fx,
                            (v[valid] + pixel_offset - cy) * z / fy, z))


def transform_points(points, translation, quaternion):
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid transform quaternion')
    x, y, z, w = q / norm
    rotation = np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)],
    ])
    return points @ rotation.T + np.asarray(translation)


def tilt_degrees(quaternion):
    """Angle between an IMU's body-up axis and gravity-up; independent of yaw."""
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid IMU orientation quaternion')
    x, y, z, w = q / norm
    return math.degrees(2 * math.atan2(math.hypot(x, y), math.hypot(z, w)))


def correct_camera_pose(robot_position, robot_quaternion, camera_position,
                        camera_quaternion, imu_quaternion):
    """Add measured roll/pitch to a planar robot/camera TF without using IMU yaw."""
    def unit(value):
        q = np.asarray(value, dtype=float)
        if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
            raise ValueError('Invalid pose or IMU quaternion')
        return q / np.linalg.norm(q)

    def product(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return np.array((aw*bx+ax*bw+ay*bz-az*by,
                         aw*by-ax*bz+ay*bw+az*bx,
                         aw*bz+ax*by-ay*bx+az*bw,
                         aw*bw-ax*bx-ay*by-az*bz))

    robot, camera, imu = map(unit, (robot_quaternion, camera_quaternion, imu_quaternion))
    if tilt_degrees(robot) > .01:
        raise ValueError('Robot TF is not planar; IMU correction would double-count tilt')
    origin, sensor = map(lambda p: np.asarray(p, dtype=float), (robot_position, camera_position))
    if origin.shape != (3,) or sensor.shape != (3,) or not np.isfinite(origin).all() or not np.isfinite(sensor).all():
        raise ValueError('Invalid pose translation')
    x, y, z, w = imu
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    heading = np.array((0., 0., math.sin(yaw/2), math.cos(yaw/2)))
    conjugate = lambda q: q * np.array((-1., -1., -1., 1.))
    tilt = product(conjugate(heading), imu)
    correction = unit(product(product(robot, tilt), conjugate(robot)))
    corrected_position = origin + transform_points([sensor-origin], (0, 0, 0), correction)[0]
    return corrected_position, unit(product(correction, camera))


def classify_cells(points, resolution=0.3, slope_deg=18.0, roughness=0.04,
                   step_height=0.18, drop_height=0.12, min_points=6, return_elevation=False):
    """Return cell center, slope, residual roughness, reason bitmask, and optional elevation.

    Bits: 1 slope, 2 roughness, 4 raised obstacle, 8 visible depression.
    Unobserved/undersampled cells are omitted, never classified as free.
    Heights must be in a gravity-aligned frame with nominal ground z=0.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if resolution <= 0:
        raise ValueError('Resolution must be positive')
    groups = np.floor(points[:, :2] / resolution).astype(int)
    cells = []
    for key in np.unique(groups, axis=0):
        sample = points[(groups == key).all(axis=1)]
        if len(sample) < min_points:
            continue
        xy = sample[:, :2] - sample[:, :2].mean(axis=0)
        design = np.column_stack((xy, np.ones(len(sample))))
        plane, _, rank, singular = np.linalg.lstsq(design, sample[:, 2], rcond=None)
        # A single depth scanline cannot establish a surface normal.
        if rank < 3 or singular[-1] / math.sqrt(len(sample)) < resolution * 0.05:
            continue
        slope = math.degrees(math.atan(np.linalg.norm(plane[:2])))
        residual = float(np.sqrt(np.mean((design @ plane - sample[:, 2])**2)))
        reason = ((1 if slope > slope_deg else 0) |
                  (2 if residual > roughness else 0) |
                  (4 if np.median(sample[:, 2]) > step_height else 0) |
                  (8 if np.median(sample[:, 2]) < -drop_height else 0))
        center = (key + 0.5) * resolution
        if return_elevation:
            cells.append((float(center[0]), float(center[1]), slope, residual, reason, float(plane[2])))
        else:
            cells.append((float(center[0]), float(center[1]), slope, residual, reason))
    return cells


def compute_traversability_cost(slope, roughness, reason=0,
                                slope_mild=5.0, slope_max=18.0,
                                roughness_mild=0.015, roughness_max=0.040,
                                cost_max_passable=95):
    """Compute normalized 2.5D traversability cost in [0, 100] for Nav2 OccupancyGrid.

    - Flat / smooth (slope <= slope_mild and roughness <= roughness_mild): cost = 0
    - Passable slopes / roughness: non-linear continuous cost in [10, cost_max_passable]
    - Lethal hazards (reason != 0 or slope >= slope_max or roughness >= roughness_max): cost = 100
    """
    if reason != 0 or slope >= slope_max or roughness >= roughness_max:
        return 100
    if slope <= slope_mild and roughness <= roughness_mild:
        return 0
    s_norm = max(0.0, (slope - slope_mild) / max(1e-3, slope_max - slope_mild))
    r_norm = max(0.0, (roughness - roughness_mild) / max(1e-3, roughness_max - roughness_mild))
    risk = 0.75 * (s_norm ** 1.5) + 0.25 * r_norm
    cost = int(round(cost_max_passable * min(1.0, risk)))
    return max(1, min(cost_max_passable, cost))

