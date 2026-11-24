"""Slope-safe heading adjustment: intent-separated observation vs travel reorientation.

Implements the architecture documented in docs/slope_heading_alternative_20261104.md
using sensor-derived terrain maps and measured vehicle attitude only (no Gazebo truth).

References:
- Ishigami, Miwa, Nagatani & Yoshida (JFR 2007, DOI 10.1002/rob.20187):
  terramechanics slip model for planetary rover steering on loose soil.
- Ishigami, Nagatani & Yoshida (JFR 2009, DOI 10.1002/rob.20277):
  slope traversal and slip-compensated steering control.
- Rabiee & Biswas (IEEE ICRA 2019, DOI 10.1109/ICRA.2019.8794216):
  friction-based kinematic model for skid-steer wheeled mobile robots.
- Martinez, Morales, Sanchez & Garcia-Cerezo (IEEE/RSJ IROS 2024,
  DOI 10.1109/IROS58592.2024.10801776): ICR-based skid-steer kinematics on slopes.
"""
import heapq
import math
import numpy as np


# Conservative slope/roughness turning envelope aligned with terrain_node.py
# (slope_mild=5.0 deg) and ramp_fixture.py flat-start verification (<= 5.0 deg).
GENTLE_SLOPE_MAX_DEG = 5.0
GENTLE_ROUGHNESS_MAX_M = 0.025
STEEP_KEEPOUT_SLOPE_DEG = 18.0

# Heading compatibility constraints for slope departures and continuous arcs.
ALIGNED_YAW_TOLERANCE_RAD = math.radians(8.0)
MAX_DEPARTURE_HEADING_ERR_RAD = math.radians(35.0)
MAX_SLOPE_STEP_BEND_RAD = math.radians(35.0)
MIN_STAGING_DISTANCE_M = 0.50
MAX_STAGING_DISTANCE_M = 6.50

# Gimbal mechanical limits (matching ExperimentMotion.pan_tilt).
GIMBAL_PAN_LIMIT_RAD = 2.35619
GIMBAL_TILT_MIN_RAD = -0.611
GIMBAL_TILT_MAX_RAD = 0.262
GIMBAL_TILT_DEFAULT_RAD = -0.2618
GIMBAL_SLEW_RATE_RAD_S = 0.35

# Curvature-bounded forward arc parameters: all radii >> half wheel track (~0.19 m)
# so inner wheels maintain forward rolling contact instead of reverse-skidding downhill.
DEFAULT_ARC_RADII_M = (1.1, 1.5, 2.0, 2.5)
ARC_WAYPOINT_SPACING_M = 0.65
ARC_LINEAR_VEL_M_S = 0.12
STAGING_LINEAR_VEL_M_S = 0.13
FLAT_TURN_RATE_RAD_S = 0.30
SWEPT_UNCERTAINTY_BUFFER_M = 0.10


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_rpy(x, y, z, w):
    """Convert unit quaternion (x, y, z, w) to intrinsic ZYX (roll, pitch, yaw) radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def profile_footprint_half_extents(simulation_profile='r5_support_v1'):
    if simulation_profile == 'legacy':
        return 0.43, 0.36, 0.65
    return 0.25, 0.22, 0.45


def within_profile_region(region, x, y, yaw, simulation_profile='r5_support_v1', margin=0.10):
    half_l, half_w, _ = profile_footprint_half_extents(simulation_profile)
    hx = half_l * abs(math.cos(yaw)) + half_w * abs(math.sin(yaw)) + margin
    hy = half_l * abs(math.sin(yaw)) + half_w * abs(math.cos(yaw)) + margin
    return (region[0] + hx <= x <= region[2] - hx
            and region[1] + hy <= y <= region[3] - hy)


def cell_slope_deg(cell):
    """Return measured cell slope in degrees, or None if unavailable/nonfinite."""
    if not isinstance(cell, dict):
        return None
    slope = cell.get('slope')
    if type(slope) in (int, float) and math.isfinite(slope):
        return float(slope)
    gx, gy = cell.get('gx'), cell.get('gy')
    if (type(gx) in (int, float) and type(gy) in (int, float)
            and math.isfinite(gx) and math.isfinite(gy)):
        return math.degrees(math.atan(math.hypot(float(gx), float(gy))))
    return None


def _index_quality_cells(quality):
    if not isinstance(quality, dict):
        return 0.25, {}
    qr = float(quality.get('resolution', 0.25) or 0.25)
    cells = {}
    for cell in quality.get('cells', ()):
        if isinstance(cell, dict) and 'ix' in cell and 'iy' in cell:
            cells[(int(cell['ix']), int(cell['iy']))] = cell
    return qr, cells


def _pose_tilt_deg(pose):
    if not isinstance(pose, dict):
        return None
    tilt = pose.get('tilt_deg')
    if type(tilt) in (int, float) and math.isfinite(tilt):
        return abs(float(tilt))
    roll, pitch = pose.get('roll'), pose.get('pitch')
    if (type(roll) in (int, float) and type(pitch) in (int, float)
            and math.isfinite(roll) and math.isfinite(pitch)):
        cos_tilt = max(-1.0, min(1.0, math.cos(float(roll)) * math.cos(float(pitch))))
        return math.degrees(math.acos(cos_tilt))
    return None


def assess_terrain_at_xy(x, y, quality_cells, qr, footprint_radius_m=0.45,
                         gentle_slope_max_deg=GENTLE_SLOPE_MAX_DEG,
                         gentle_roughness_max_m=GENTLE_ROUGHNESS_MAX_M):
    """Assess measured surface slope, roughness, and validity over a turning disk."""
    rad_cells = max(1, int(math.ceil(footprint_radius_m / qr)))
    cx = int(math.floor(x / qr))
    cy = int(math.floor(y / qr))
    slopes = []
    roughnesses = []
    valid_cells = 0
    observed_cells = 0
    invalid_cells = 0
    for ix in range(cx - rad_cells, cx + rad_cells + 1):
        wx = (ix + 0.5) * qr
        for iy in range(cy - rad_cells, cy + rad_cells + 1):
            wy = (iy + 0.5) * qr
            if math.hypot(wx - x, wy - y) > footprint_radius_m + qr * 0.5:
                continue
            cell = quality_cells.get((ix, iy))
            if cell is None:
                continue
            observed_cells += 1
            fit_state = cell.get('fit_state')
            if fit_state == 'valid' or (fit_state is None and cell.get('valid') is True):
                valid_cells += 1
            elif fit_state is not None:
                invalid_cells += 1
            s = cell_slope_deg(cell)
            if s is not None:
                slopes.append(s)
            r = cell.get('roughness')
            if type(r) in (int, float) and math.isfinite(r):
                roughnesses.append(float(r))
    # Use 75th-percentile slope across the turning footprint to resist single-cell noise
    # while remaining conservative against sloped edges.
    map_slope_deg = float(np.percentile(slopes, 75)) if slopes else None
    max_roughness_m = float(max(roughnesses)) if roughnesses else None
    gentle = (
        map_slope_deg is not None
        and map_slope_deg <= gentle_slope_max_deg
        and valid_cells >= 1
        and (max_roughness_m is None or max_roughness_m <= gentle_roughness_max_m)
    )
    return dict(
        map_slope_deg=round(map_slope_deg, 3) if map_slope_deg is not None else None,
        max_roughness_m=round(max_roughness_m, 4) if max_roughness_m is not None else None,
        valid_cells=valid_cells,
        observed_cells=observed_cells,
        invalid_cells=invalid_cells,
        gentle=gentle,
    )


def assess_local_slope(pose, quality, *, simulation_profile='r5_support_v1',
                       gentle_slope_max_deg=GENTLE_SLOPE_MAX_DEG,
                       gentle_roughness_max_m=GENTLE_ROUGHNESS_MAX_M):
    """Evaluate local slope and whether in-place chassis spinning is verified safe.

    Uses both measured terrain gradients around the rover and measured body tilt
    (roll/pitch from TF/IMU when provided), taking the more conservative (steeper) value.
    """
    _, _, footprint_radius_m = profile_footprint_half_extents(simulation_profile)
    qr, quality_cells = _index_quality_cells(quality)
    terrain = assess_terrain_at_xy(
        pose['x'], pose['y'], quality_cells, qr,
        footprint_radius_m=footprint_radius_m,
        gentle_slope_max_deg=gentle_slope_max_deg,
        gentle_roughness_max_m=gentle_roughness_max_m,
    )
    tilt_deg = _pose_tilt_deg(pose)
    map_slope = terrain['map_slope_deg']
    candidates = [v for v in (map_slope, tilt_deg) if v is not None]
    measured_slope_deg = max(candidates) if candidates else None
    has_slope_evidence = measured_slope_deg is not None
    roughness_exceeded = (
        terrain['max_roughness_m'] is not None
        and terrain['max_roughness_m'] > gentle_roughness_max_m
    )
    slope_exceeded = (
        measured_slope_deg is not None
        and measured_slope_deg > gentle_slope_max_deg
    )
    if slope_exceeded:
        reason = 'measured_slope_exceeds_turn_envelope'
        in_place_turn_safe = False
    elif roughness_exceeded:
        reason = 'measured_roughness_exceeds_turn_envelope'
        in_place_turn_safe = False
    elif not has_slope_evidence:
        reason = 'unverified_local_slope'
        in_place_turn_safe = False
    else:
        reason = 'verified_gentle_turn_envelope'
        in_place_turn_safe = True

    return dict(
        in_place_turn_safe=in_place_turn_safe,
        slope_exceeded=slope_exceeded,
        roughness_exceeded=roughness_exceeded,
        has_slope_evidence=has_slope_evidence,
        slope_deg=round(measured_slope_deg, 3) if measured_slope_deg is not None else None,
        map_slope_deg=map_slope,
        tilt_deg=round(tilt_deg, 3) if tilt_deg is not None else None,
        max_roughness_m=terrain['max_roughness_m'],
        valid_cells=terrain['valid_cells'],
        observed_cells=terrain['observed_cells'],
        threshold_deg=float(gentle_slope_max_deg),
        reason=reason,
    )


def _known_disk_clear(costs, ix, iy, resolution, radius_m):
    height, width = costs.shape
    n = int(math.ceil(radius_m / resolution))
    if ix < n or iy < n or ix + n >= width or iy + n >= height:
        return False
    patch = costs[iy - n:iy + n + 1, ix - n:ix + n + 1]
    yy, xx = np.ogrid[-n:n + 1, -n:n + 1]
    disk = xx * xx + yy * yy <= n * n
    return not np.any((patch[disk] < 0) | (patch[disk] >= 99))


def swept_poses_clear(costs, ox, oy, resolution, region, origin, radius, poses,
                      *, simulation_profile='r5_support_v1',
                      uncertainty_buffer_m=SWEPT_UNCERTAINTY_BUFFER_M,
                      quality_cells=None, quality_resolution=0.25,
                      max_slope_deg=STEEP_KEEPOUT_SLOPE_DEG):
    """Verify full oriented chassis footprint along a continuous sequence of (x, y, yaw).

    Unknown costmap cells (< 0) and lethal/inscribed obstacles (>= 99) always block.
    Region and mission radius bounds are enforced at every pose with uncertainty buffer.
    """
    height, width = costs.shape
    half_l, half_w, _ = profile_footprint_half_extents(simulation_profile)
    hx = half_l + uncertainty_buffer_m
    hy = half_w + uncertainty_buffer_m
    cell_diag = resolution / math.sqrt(2.0)
    max_cost_seen = 0
    max_slope_seen = 0.0

    for x, y, yaw in poses:
        c, s = math.cos(yaw), math.sin(yaw)
        bx = hx * abs(c) + hy * abs(s)
        by = hx * abs(s) + hy * abs(c)
        if (x - bx < region[0] or x + bx > region[2]
                or y - by < region[1] or y + by > region[3]):
            return False, 'swept_region_boundary', max_cost_seen, max_slope_seen
        if math.isfinite(radius) and math.hypot(x - origin[0], y - origin[1]) + math.hypot(hx, hy) > radius:
            return False, 'swept_mission_radius', max_cost_seen, max_slope_seen
        ix0 = math.floor((x - bx - resolution - ox) / resolution)
        ix1 = math.floor((x + bx + resolution - ox) / resolution)
        iy0 = math.floor((y - by - resolution - oy) / resolution)
        iy1 = math.floor((y + by + resolution - oy) / resolution)
        if ix0 < 0 or iy0 < 0 or ix1 >= width or iy1 >= height:
            return False, 'swept_outside_costmap', max_cost_seen, max_slope_seen
        xx, yy = np.meshgrid(
            ox + (np.arange(ix0, ix1 + 1) + 0.5) * resolution,
            oy + (np.arange(iy0, iy1 + 1) + 0.5) * resolution,
        )
        dx, dy = xx - x, yy - y
        mask = (np.abs(c * dx + s * dy) <= hx + cell_diag) & (np.abs(-s * dx + c * dy) <= hy + cell_diag)
        values = costs[iy0:iy1 + 1, ix0:ix1 + 1][mask]
        if not values.size or np.any(values < 0):
            return False, 'swept_unknown_space', max_cost_seen, max_slope_seen
        if np.any(values >= 99):
            return False, 'swept_obstacle_or_inflation', max_cost_seen, max_slope_seen
        max_cost_seen = max(max_cost_seen, int(np.max(values)))
        if quality_cells is not None:
            qkey = (int(math.floor(x / quality_resolution)), int(math.floor(y / quality_resolution)))
            cell = quality_cells.get(qkey)
            s_deg = cell_slope_deg(cell)
            if s_deg is not None:
                max_slope_seen = max(max_slope_seen, s_deg)
                if s_deg >= max_slope_deg:
                    return False, 'swept_steep_keepout', max_cost_seen, max_slope_seen

    return True, 'observed_swept_footprint_clear', max_cost_seen, round(max_slope_seen, 3)


def _heading_compatible_path(path, start_yaw, max_departure_err=MAX_DEPARTURE_HEADING_ERR_RAD,
                             max_bend_rad=MAX_SLOPE_STEP_BEND_RAD, sample_step_m=0.45):
    """Check that a path departs near start_yaw and bends smoothly without hairpin reversals."""
    if len(path) < 2:
        return False, math.pi, 0.0
    sampled = [path[0]]
    acc = 0.0
    for pt in path[1:]:
        acc += math.hypot(pt[0] - sampled[-1][0], pt[1] - sampled[-1][1])
        if acc >= sample_step_m:
            sampled.append(pt)
            acc = 0.0
    if math.hypot(path[-1][0] - sampled[-1][0], path[-1][1] - sampled[-1][1]) > 0.15:
        sampled.append(path[-1])
    if len(sampled) < 2:
        return False, math.pi, 0.0

    prev_heading = start_yaw
    departure_err = None
    max_bend = 0.0
    for idx in range(len(sampled) - 1):
        dx = sampled[idx + 1][0] - sampled[idx][0]
        dy = sampled[idx + 1][1] - sampled[idx][1]
        if math.hypot(dx, dy) < 1e-4:
            continue
        seg_heading = math.atan2(dy, dx)
        bend = abs(wrap(seg_heading - prev_heading))
        if departure_err is None:
            departure_err = bend
            if departure_err > max_departure_err:
                return False, departure_err, bend
        elif bend > max_bend_rad:
            return False, departure_err, bend
        max_bend = max(max_bend, bend)
        prev_heading = seg_heading

    if departure_err is None:
        return False, math.pi, 0.0
    return True, departure_err, max_bend


def _path_to_poses(path, start_yaw, step_m=0.10):
    poses = []
    if not path:
        return poses
    prev_yaw = start_yaw
    for i in range(len(path) - 1):
        x0, y0 = path[i]
        x1, y1 = path[i + 1]
        dist = math.hypot(x1 - x0, y1 - y0)
        if dist < 1e-6:
            continue
        seg_yaw = math.atan2(y1 - y0, x1 - x0)
        n = max(1, int(math.ceil(dist / step_m)))
        for k in range(n):
            t = k / n
            poses.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, seg_yaw))
        prev_yaw = seg_yaw
    poses.append((path[-1][0], path[-1][1], prev_yaw))
    return poses


def find_gentle_turn_zone(pose, target_yaw, grid, quality, region, origin, radius, remaining,
                          *, simulation_profile='r5_support_v1',
                          gentle_slope_max_deg=GENTLE_SLOPE_MAX_DEG,
                          gentle_roughness_max_m=GENTLE_ROUGHNESS_MAX_M,
                          max_distance_m=MAX_STAGING_DISTANCE_M):
    """Search observed passable terrain for a gentle turning zone reachable without steep spin.

    Requires:
    1. Continuous observed path from pose to staging cell.
    2. Departure heading within MAX_DEPARTURE_HEADING_ERR_RAD of pose['yaw'] (no steep spin!).
    3. Gentle measured slope (<= gentle_slope_max_deg) and valid surface over turning footprint.
    4. Swept chassis clearance along transit and full turning clearance at destination.
    5. Fits within remaining simulation time budget.
    """
    info = grid.info
    resolution = float(info.resolution)
    ox, oy = info.origin.position.x, info.origin.position.y
    costs = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)
    height, width = costs.shape
    _, _, clearance_m = profile_footprint_half_extents(simulation_profile)
    qr, quality_cells = _index_quality_cells(quality)

    sx = int(math.floor((pose['x'] - ox) / resolution))
    sy = int(math.floor((pose['y'] - oy) / resolution))
    if not (0 <= sx < width and 0 <= sy < height):
        return None, dict(reason='start_outside_costmap', candidates_evaluated=0)

    # Heading-biased Dijkstra on observed passable cells (8-connected so diagonal
    # paths align smoothly with continuous rover heading without Manhattan stair-stepping).
    distances = {(sx, sy): 0.0}
    parents = {(sx, sy): None}
    queue = [(0.0, sx, sy)]
    neighbors = (
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
    )
    max_search_cost = max_distance_m * 1.8

    while queue:
        dist, x, y = heapq.heappop(queue)
        if dist != distances[(x, y)]:
            continue
        for dx, dy, step_scale in neighbors:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            c = int(costs[ny, nx])
            px = ox + (nx + 0.5) * resolution
            py = oy + (ny + 0.5) * resolution
            d_robot = math.hypot(px - pose['x'], py - pose['y'])
            max_c = 100 if d_robot <= clearance_m else 99
            if not (0 <= c < max_c):
                continue
            if not within_profile_region(region, px, py, pose['yaw'], simulation_profile=simulation_profile):
                continue
            if math.isfinite(radius) and math.hypot(px - origin[0], py - origin[1]) > radius - clearance_m:
                continue
            # Strongly discourage initial steps behind the rover so paths grow forward.
            if d_robot >= 0.25 and d_robot <= 1.2:
                bearing = math.atan2(py - pose['y'], px - pose['x'])
                if abs(wrap(bearing - pose['yaw'])) > math.radians(60.0):
                    continue
            weight = 1.0 + 1.5 * ((max(0, c) / 100.0) ** 2)
            nd = dist + resolution * step_scale * weight
            if nd <= max_search_cost and nd < distances.get((nx, ny), math.inf):
                distances[(nx, ny)] = nd
                parents[(nx, ny)] = (x, y)
                heapq.heappush(queue, (nd, nx, ny))

    stride = max(1, int(round(0.25 / resolution)))
    evaluated = 0
    rejections = {}
    best = None

    def reject(tag):
        rejections[tag] = rejections.get(tag, 0) + 1

    # Inspect reachable gentle cells on a 0.25 m grid.
    sorted_cells = sorted(distances.items(), key=lambda item: item[1])
    for (ix, iy), graph_cost in sorted_cells:
        if ix % stride != 0 or iy % stride != 0:
            continue
        wx = ox + (ix + 0.5) * resolution
        wy = oy + (iy + 0.5) * resolution
        direct_dist = math.hypot(wx - pose['x'], wy - pose['y'])
        if direct_dist < MIN_STAGING_DISTANCE_M or direct_dist > max_distance_m:
            continue
        if int(costs[iy, ix]) > 55:
            continue
        evaluated += 1
        terrain = assess_terrain_at_xy(
            wx, wy, quality_cells, qr,
            footprint_radius_m=clearance_m,
            gentle_slope_max_deg=gentle_slope_max_deg,
            gentle_roughness_max_m=gentle_roughness_max_m,
        )
        if not terrain['gentle']:
            reject('not_gentle_surface')
            continue
        if not _known_disk_clear(costs, ix, iy, resolution, clearance_m):
            reject('insufficient_turning_clearance')
            continue
        if not within_profile_region(region, wx, wy, target_yaw, simulation_profile=simulation_profile):
            reject('target_yaw_outside_region')
            continue

        # Reconstruct path and check departure heading compatibility.
        path = []
        cur = (ix, iy)
        while cur is not None:
            path.append((ox + (cur[0] + 0.5) * resolution, oy + (cur[1] + 0.5) * resolution))
            cur = parents[cur]
        path.reverse()
        path[0] = (pose['x'], pose['y'])

        compat, dep_err, max_bend = _heading_compatible_path(path, pose['yaw'])
        if not compat:
            reject('incompatible_departure_heading')
            continue

        # Verify full swept footprint along the transit path.
        transit_poses = _path_to_poses(path, pose['yaw'], step_m=0.10)
        clear, sweep_reason, max_cost, max_slope = swept_poses_clear(
            costs, ox, oy, resolution, region, origin, radius, transit_poses,
            simulation_profile=simulation_profile,
            quality_cells=quality_cells, quality_resolution=qr,
        )
        if not clear:
            reject(sweep_reason)
            continue

        path_len = sum(
            math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
            for i in range(len(path) - 1)
        )
        arrival_yaw = transit_poses[-1][2] if transit_poses else pose['yaw']
        turn_angle = abs(wrap(target_yaw - arrival_yaw))
        transit_s = path_len / STAGING_LINEAR_VEL_M_S + 3.0
        turn_s = turn_angle / FLAT_TURN_RATE_RAD_S + 3.0
        total_s = transit_s + turn_s
        if total_s > remaining:
            reject('budget_exceeded')
            continue

        # Prefer flatter interior of the gentle zone (margin against 0.25 m arrival tolerance),
        # balanced against transit distance and smooth departure alignment.
        slope_val = terrain['map_slope_deg'] if terrain['map_slope_deg'] is not None else gentle_slope_max_deg
        edge_penalty = 1.5 if slope_val > max(0.0, gentle_slope_max_deg - 1.0) else 0.0
        score = path_len + 0.5 * slope_val + edge_penalty + 0.8 * dep_err
        if best is None or score < best['score']:
            best = dict(
                x=round(wx, 3),
                y=round(wy, 3),
                arrival_yaw=round(arrival_yaw, 4),
                path_length_m=round(path_len, 3),
                departure_heading_err_deg=round(math.degrees(dep_err), 2),
                max_path_bend_deg=round(math.degrees(max_bend), 2),
                staging_slope_deg=terrain['map_slope_deg'],
                staging_roughness_m=terrain['max_roughness_m'],
                staging_valid_cells=terrain['valid_cells'],
                max_transit_cost=max_cost,
                max_transit_slope_deg=max_slope,
                transit_seconds=round(transit_s, 1),
                turn_seconds=round(turn_s, 1),
                estimated_seconds=round(total_s, 1),
                score=score,
                path=[[round(px, 3), round(py, 3)] for px, py in path],
            )
            # Stop early only when we have a clearly flat staging zone close ahead.
            if path_len <= 2.5 and slope_val <= 2.0:
                break

    diag = dict(candidates_evaluated=evaluated, rejections=rejections)
    if best is None:
        diag['reason'] = 'no_reachable_gentle_turn_zone'
    else:
        diag['reason'] = 'gentle_turn_zone_found'
    return best, diag


def plan_smooth_arc_maneuver(pose, target_yaw, grid, quality, region, origin, radius, remaining,
                             *, simulation_profile='r5_support_v1',
                             arc_radii_m=DEFAULT_ARC_RADII_M):
    """Plan a curvature-bounded continuous forward arc maneuver on a slope.

    All wheels keep forward rolling velocity (v > 0, |kappa| = 1/R <= 1/min(R)) to avoid
    pure-spin inner-wheel reversal and downhill breakaway. Swept chassis clearance,
    observed support, region bounds, and time budget are checked along the full arc.
    """
    info = grid.info
    resolution = float(info.resolution)
    ox, oy = info.origin.position.x, info.origin.position.y
    costs = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)
    qr, quality_cells = _index_quality_cells(quality)

    delta_short = wrap(target_yaw - pose['yaw'])
    if abs(delta_short) <= ALIGNED_YAW_TOLERANCE_RAD:
        return None, dict(reason='already_aligned', candidates_evaluated=0)

    # Evaluate primary turn direction first, plus opposite direction if primary is blocked.
    turn_deltas = [delta_short]
    if abs(delta_short) < math.pi - 0.15:
        alt = delta_short - math.copysign(2.0 * math.pi, delta_short)
        if abs(alt) <= math.radians(240.0):
            turn_deltas.append(alt)

    rejections = {}
    evaluated = 0
    best = None

    def reject(tag):
        rejections[tag] = rejections.get(tag, 0) + 1

    for turn_delta in turn_deltas:
        sign = 1.0 if turn_delta >= 0.0 else -1.0
        abs_delta = abs(turn_delta)
        for R in sorted(arc_radii_m):
            if R <= 0.6:
                continue
            evaluated += 1
            arc_len = abs_delta * R
            est_seconds = arc_len / ARC_LINEAR_VEL_M_S + 4.0
            if est_seconds > remaining:
                reject('budget_exceeded')
                continue

            # Sample continuous arc poses every 0.05 m for rigorous swept-footprint check.
            n_sweep = max(8, int(math.ceil(arc_len / 0.05)))
            cx = pose['x'] - sign * R * math.sin(pose['yaw'])
            cy = pose['y'] + sign * R * math.cos(pose['yaw'])
            swept_poses = []
            for k in range(n_sweep + 1):
                frac = k / n_sweep
                yaw_k = wrap(pose['yaw'] + turn_delta * frac)
                xk = cx + sign * R * math.sin(yaw_k)
                yk = cy - sign * R * math.cos(yaw_k)
                swept_poses.append((xk, yk, yaw_k))

            clear, reason, max_cost, max_slope = swept_poses_clear(
                costs, ox, oy, resolution, region, origin, radius, swept_poses,
                simulation_profile=simulation_profile,
                quality_cells=quality_cells, quality_resolution=qr,
            )
            if not clear:
                reject(reason)
                continue

            # Emit intermediate navigation waypoints spaced by ARC_WAYPOINT_SPACING_M
            # so each leg has a small heading offset (< ~30 deg) under SlopeFollowPath.
            n_wp = max(2, int(math.ceil(arc_len / ARC_WAYPOINT_SPACING_M)))
            # Also guarantee angular step <= 30 deg per waypoint so RPP never stalls.
            n_wp = max(n_wp, int(math.ceil(abs_delta / math.radians(30.0))))
            waypoints = []
            for j in range(1, n_wp + 1):
                frac = j / n_wp
                yaw_j = wrap(pose['yaw'] + turn_delta * frac)
                xj = cx + sign * R * math.sin(yaw_j)
                yj = cy - sign * R * math.cos(yaw_j)
                waypoints.append(dict(
                    x=round(xj, 3),
                    y=round(yj, 3),
                    yaw=round(yaw_j, 4),
                    slope_safe=True,
                ))

            # Prefer gentler curvature (larger R) when time and cost allow, balanced against arc length.
            score = arc_len + 0.04 * max_cost + 0.15 * max_slope + 1.2 / R
            if best is None or score < best['score']:
                best = dict(
                    maneuver_kind='curvature_bounded_arc',
                    radius_m=round(R, 3),
                    curvature_rad_m=round(sign / R, 4),
                    arc_length_m=round(arc_len, 3),
                    turn_delta_rad=round(turn_delta, 4),
                    end_pose=dict(
                        x=round(swept_poses[-1][0], 3),
                        y=round(swept_poses[-1][1], 3),
                        yaw=round(swept_poses[-1][2], 4),
                    ),
                    max_swept_cost=max_cost,
                    max_swept_slope_deg=max_slope,
                    estimated_seconds=round(est_seconds, 1),
                    swept_checked=True,
                    waypoints=waypoints,
                    score=score,
                )

    diag = dict(candidates_evaluated=evaluated, rejections=rejections)
    if best is None:
        diag['reason'] = 'no_swept_clear_arc'
    else:
        diag['reason'] = 'smooth_arc_found'
    return best, diag


def plan_heading_adjustment(intent, target_yaw, pose, grid, quality, region, origin, radius, remaining,
                            *, gimbal_available=True,
                            current_gimbal=(0.0, GIMBAL_TILT_DEFAULT_RAD),
                            simulation_profile='r5_support_v1',
                            gentle_slope_max_deg=GENTLE_SLOPE_MAX_DEG,
                            allow_arc_fallback=True):
    """Unified decision engine separating observe_heading vs travel_heading on slopes.

    Follows the exact flowchart in docs/slope_heading_alternative_20261104.md:
    1. observe_heading -> gimbal pan/tilt first (zero chassis slip).
    2. travel_heading (or gimbal out-of-range fallback):
       a) already aligned -> no-op.
       b) verified flat/gentle terrain -> ordinary in-place turn.
       c) sloped/unverified terrain -> gentle-zone staging with slope-safe Nav2 transit.
       d) no reachable gentle zone -> curvature-bounded smooth forward arc (swept-checked).
       e) otherwise -> explicit safe refusal (never forces an unverified steep Spin).
    """
    if intent not in ('observe_heading', 'travel_heading'):
        raise ValueError(f'Unknown heading intent: {intent!r}')
    if type(target_yaw) not in (int, float) or not math.isfinite(target_yaw) or abs(target_yaw) > math.pi + 1e-6:
        raise ValueError('target_yaw must be a finite angle in [-pi, pi]')
    target_yaw = wrap(float(target_yaw))
    radius = math.inf if radius is None else float(radius)

    slope_info = assess_local_slope(
        pose, quality,
        simulation_profile=simulation_profile,
        gentle_slope_max_deg=gentle_slope_max_deg,
    )

    gimbal_note = None
    if intent == 'observe_heading':
        rel_pan = wrap(target_yaw - pose['yaw'])
        if gimbal_available and abs(rel_pan) <= GIMBAL_PAN_LIMIT_RAD:
            # 3D tilt compensation when measured body roll/pitch are available.
            tilt = GIMBAL_TILT_DEFAULT_RAD
            roll, pitch = pose.get('roll'), pose.get('pitch')
            if (type(roll) in (int, float) and type(pitch) in (int, float)
                    and math.isfinite(roll) and math.isfinite(pitch)):
                body_pitch_along_view = float(pitch) * math.cos(rel_pan) - float(roll) * math.sin(rel_pan)
                tilt = max(GIMBAL_TILT_MIN_RAD, min(GIMBAL_TILT_MAX_RAD, GIMBAL_TILT_DEFAULT_RAD - body_pitch_along_view))
            cur_pan = float(current_gimbal[0]) if current_gimbal and len(current_gimbal) >= 1 else 0.0
            cur_tilt = float(current_gimbal[1]) if current_gimbal and len(current_gimbal) >= 2 else GIMBAL_TILT_DEFAULT_RAD
            est_s = math.hypot(rel_pan - cur_pan, tilt - cur_tilt) / GIMBAL_SLEW_RATE_RAD_S + 2.5
            if est_s > remaining:
                return dict(
                    feasible=False,
                    intent=intent,
                    strategy='refused_budget_exceeded',
                    maneuver_kind='none',
                    chassis_motion=False,
                    slope_assessment=slope_info,
                    reason='gimbal_observation_exceeds_budget',
                    estimated_seconds=round(est_s, 1),
                    stages=[],
                )
            return dict(
                feasible=True,
                intent=intent,
                strategy='gimbal_observe',
                maneuver_kind='gimbal_only',
                chassis_motion=False,
                slope_safe_transit=False,
                target_yaw=target_yaw,
                pan=round(rel_pan, 4),
                tilt=round(tilt, 4),
                estimated_seconds=round(est_s, 1),
                slope_assessment=slope_info,
                reason='observe_heading_fulfilled_by_gimbal',
                stages=[dict(
                    stage='gimbal_pan_tilt',
                    kind='gimbal',
                    pan=round(rel_pan, 4),
                    tilt=round(tilt, 4),
                    estimated_seconds=round(est_s, 1),
                )],
            )
        gimbal_note = 'gimbal_unavailable' if not gimbal_available else 'pan_exceeds_gimbal_limit'

    yaw_err = abs(wrap(target_yaw - pose['yaw']))
    if yaw_err <= ALIGNED_YAW_TOLERANCE_RAD:
        return dict(
            feasible=True,
            intent=intent,
            strategy='already_aligned',
            maneuver_kind='none',
            chassis_motion=False,
            slope_safe_transit=False,
            target_yaw=target_yaw,
            estimated_seconds=0.0,
            slope_assessment=slope_info,
            gimbal_fallback_reason=gimbal_note,
            reason='chassis_already_aligned',
            stages=[],
        )

    # Branch 2: Verified gentle/flat terrain -> ordinary in-place turn allowed.
    if slope_info['in_place_turn_safe']:
        if not (within_profile_region(region, pose['x'], pose['y'], pose['yaw'], simulation_profile=simulation_profile)
                and within_profile_region(region, pose['x'], pose['y'], target_yaw, simulation_profile=simulation_profile)):
            return dict(
                feasible=False,
                intent=intent,
                strategy='refused_region_boundary',
                maneuver_kind='none',
                chassis_motion=False,
                slope_assessment=slope_info,
                gimbal_fallback_reason=gimbal_note,
                reason='in_place_turn_violates_region_boundary',
                stages=[],
            )
        est_s = yaw_err / FLAT_TURN_RATE_RAD_S + 3.0
        if est_s > remaining:
            return dict(
                feasible=False,
                intent=intent,
                strategy='refused_budget_exceeded',
                maneuver_kind='none',
                chassis_motion=False,
                slope_assessment=slope_info,
                gimbal_fallback_reason=gimbal_note,
                reason='in_place_turn_exceeds_budget',
                estimated_seconds=round(est_s, 1),
                stages=[],
            )
        return dict(
            feasible=True,
            intent=intent,
            strategy='in_place_turn',
            maneuver_kind='verified_flat_spin',
            chassis_motion=True,
            slope_safe_transit=False,
            target_yaw=target_yaw,
            estimated_seconds=round(est_s, 1),
            slope_assessment=slope_info,
            gimbal_fallback_reason=gimbal_note,
            reason='verified_flat_ground_in_place_turn',
            stages=[dict(
                stage='in_place_spin',
                kind='turn',
                yaw=target_yaw,
                estimated_seconds=round(est_s, 1),
            )],
        )

    # Branch 3: On sloped/unverified terrain -> seek reachable gentle turning zone first.
    staging, staging_diag = find_gentle_turn_zone(
        pose, target_yaw, grid, quality, region, origin, radius, remaining,
        simulation_profile=simulation_profile,
        gentle_slope_max_deg=gentle_slope_max_deg,
    )
    if staging is not None:
        return dict(
            feasible=True,
            intent=intent,
            strategy='gentle_zone_staging',
            maneuver_kind='stage_then_turn',
            chassis_motion=True,
            slope_safe_transit=True,
            target_yaw=target_yaw,
            estimated_seconds=staging['estimated_seconds'],
            slope_assessment=slope_info,
            gimbal_fallback_reason=gimbal_note,
            staging=staging,
            staging_diagnostics=staging_diag,
            reason='slope_in_place_turn_avoided_via_gentle_zone',
            stages=[
                dict(
                    stage='stage_transit',
                    kind='advance',
                    x=staging['x'],
                    y=staging['y'],
                    slope_safe=True,
                    estimated_seconds=staging['transit_seconds'],
                ),
                dict(
                    stage='verify_gentle_settled',
                    kind='verify_slope',
                    max_slope_deg=float(gentle_slope_max_deg),
                    target_xy=[staging['x'], staging['y']],
                ),
                dict(
                    stage='reorient_in_gentle_zone',
                    kind='turn',
                    yaw=target_yaw,
                    estimated_seconds=staging['turn_seconds'],
                ),
            ],
        )

    # Branch 4: No gentle zone available -> evaluate curvature-bounded smooth forward arc.
    arc_diag = dict(reason='arc_fallback_disabled')
    if allow_arc_fallback:
        arc, arc_diag = plan_smooth_arc_maneuver(
            pose, target_yaw, grid, quality, region, origin, radius, remaining,
            simulation_profile=simulation_profile,
        )
        if arc is not None:
            stages = [
                dict(
                    stage=f'arc_leg_{idx + 1}',
                    kind='advance',
                    x=wp['x'],
                    y=wp['y'],
                    yaw=wp['yaw'],
                    slope_safe=True,
                    estimated_seconds=round(arc['estimated_seconds'] / len(arc['waypoints']), 1),
                )
                for idx, wp in enumerate(arc['waypoints'])
            ]
            return dict(
                feasible=True,
                intent=intent,
                strategy='smooth_arc_maneuver',
                maneuver_kind='curvature_bounded_arc',
                chassis_motion=True,
                slope_safe_transit=True,
                target_yaw=target_yaw,
                estimated_seconds=arc['estimated_seconds'],
                slope_assessment=slope_info,
                gimbal_fallback_reason=gimbal_note,
                staging_diagnostics=staging_diag,
                arc=arc,
                arc_diagnostics=arc_diag,
                reason='slope_in_place_turn_avoided_via_smooth_arc',
                stages=stages,
            )

    # Branch 5: Explicit safe refusal — never force an unverified steep-slope Spin.
    return dict(
        feasible=False,
        intent=intent,
        strategy='refused_unsafe_slope_turn',
        maneuver_kind='none',
        chassis_motion=False,
        slope_safe_transit=False,
        target_yaw=target_yaw,
        estimated_seconds=0.0,
        slope_assessment=slope_info,
        gimbal_fallback_reason=gimbal_note,
        staging_diagnostics=staging_diag,
        arc_diagnostics=arc_diag,
        reason='no_gentle_zone_or_clear_arc_on_slope',
        stages=[],
    )
