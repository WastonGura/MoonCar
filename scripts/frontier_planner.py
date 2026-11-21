"""Literature-guided dual-layer exploration planner (local NBV + global frontier tour).

References:
- Yamauchi (CIRA 1997): frontier extraction between known passable and unobserved space.
- Gonzalez-Banos & Latombe (IJRR 2002): safe-region NBV utility vs travel cost.
- Bircher et al. (ICRA 2016): receding-horizon local next-best-view replanning.
- Selin et al. (IEEE RA-L 2019, AEP): hybrid local NBV + global frontier switching.
- Zhou et al. (IEEE RA-L 2021, FUEL) & Cao et al. (RSS 2021, TARE): frontier clustering,
  approximate shortest global tour (greedy + 2-opt), and heading continuity.
"""
import heapq
import math
import numpy as np

from experiment_scenes import scene_bounds
from survey_evaluation import scoped_cells


# Aligned with elevated mooncar_r5.urdf.xacro (optical frame at x~0.183m, z=0.285m, HFOV=73 deg).
R5_CAMERA_FORWARD_M = 0.183
R5_CAMERA_HEIGHT_M = 0.285
R5_CAMERA_HFOV_RAD = math.radians(73.0)
R5_CAMERA_HALF_VFOV_RAD = 0.5061
R5_CAMERA_MAX_RANGE_M = 5.5
R5_CAMERA_MIN_RANGE_M = 0.6
GIMBAL_PAN_LIMIT_RAD = 2.35619
GIMBAL_TILT_DEFAULT_RAD = -0.2618
GIMBAL_RECENTER_THRESHOLD_RAD = 0.175  # ~10 deg
LOCAL_NEW_GAIN_SWITCH_M2 = 1.5
GLOBAL_WAYPOINT_STEP_M = 6.0


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def inside_survey_region(region, x, y):
    xmin, ymin, xmax, ymax = region
    return xmin <= x < xmax and ymin <= y < ymax


def known_chassis_clearance(costs, x, y, resolution, margin_m=0.65):
    height, width = costs.shape
    n = math.ceil(margin_m / resolution)
    if x < n or y < n or x + n >= width or y + n >= height:
        return False
    patch = costs[y - n:y + n + 1, x - n:x + n + 1]
    yy, xx = np.ogrid[-n:n + 1, -n:n + 1]
    disk = xx * xx + yy * yy <= n * n
    return not np.any((patch[disk] < 0) | (patch[disk] >= 99))


def resampling_value(cell, key, resolution, camera):
    if cell.get('fit_state') not in ('too_few_voxels', 'insufficient_xy_spread'):
        return 0.0
    angle = math.atan2(camera[1] - (key[1] + 0.5) * resolution, camera[0] - (key[0] + 0.5) * resolution)
    bearing = int((angle + math.pi) / (math.pi / 4.0)) % 8
    if cell.get('bearing_mask', 255) & (1 << bearing):
        return 0.0
    return 0.5


def run_dijkstra(costs, ox, oy, resolution, pose, origin, radius, region,
                 max_cost_distance=6.0, stride=1):
    """Terrain-weighted Dijkstra over known passable cells, optional coarse stride for global search."""
    height, width = costs.shape
    xmin, ymin, xmax, ymax = region
    sx = int(math.floor((pose['x'] - ox) / resolution))
    sy = int(math.floor((pose['y'] - oy) / resolution))
    if not (0 <= sx < width and 0 <= sy < height):
        return {}, {}
    if stride > 1:
        sx = (sx // stride) * stride
        sy = (sy // stride) * stride
    step_m = resolution * stride
    distances = {(sx, sy): 0.0}
    parents = {(sx, sy): None}
    queue = [(0.0, sx, sy)]
    neighbors = ((stride, 0), (-stride, 0), (0, stride), (0, -stride))
    while queue:
        dist, x, y = heapq.heappop(queue)
        if dist != distances[(x, y)]:
            continue
        for dx, dy in neighbors:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            c = int(costs[ny, nx])
            px = ox + (nx + 0.5) * resolution
            py = oy + (ny + 0.5) * resolution
            if not (xmin + 0.7 <= px <= xmax - 0.7 and ymin + 0.7 <= py <= ymax - 0.7):
                continue
            if math.hypot(px - origin[0], py - origin[1]) > radius - 0.75:
                continue
            d_robot = math.hypot(px - pose['x'], py - pose['y'])
            max_c = 100 if d_robot <= 0.65 else 99
            if not (0 <= c < max_c):
                continue
            if stride > 1:
                mx, my = (x + nx) // 2, (y + ny) // 2
                mc = int(costs[my, mx])
                if not (0 <= mc < max_c):
                    continue
            weight = 1.0 + 2.0 * ((max(0, c) / 100.0) ** 2)
            nd = dist + step_m * weight
            if nd <= max_cost_distance and nd < distances.get((nx, ny), math.inf):
                distances[(nx, ny)] = nd
                parents[(nx, ny)] = (x, y)
                heapq.heappush(queue, (nd, nx, ny))
    return distances, parents


def reconstruct_path(parents, key, ox, oy, resolution):
    path = []
    cur = key
    while cur is not None:
        path.append((ox + (cur[0] + 0.5) * resolution, oy + (cur[1] + 0.5) * resolution))
        cur = parents[cur]
    path.reverse()
    return path


def extract_frontier_clusters(costs, ox, oy, resolution, region, origin, radius,
                              observed_keys=None, quality_resolution=0.25,
                              cluster_radius_m=2.0, min_cells=3):
    """Extract and cluster dual frontiers (costmap unknown OR depth-unobserved cells)."""
    height, width = costs.shape
    xmin, ymin, xmax, ymax = region
    passable = (costs >= 0) & (costs < 99)
    unknown = costs < 0

    # Any passable cell adjacent to an unknown costmap cell is a classic Yamauchi frontier.
    adj_unknown = np.zeros_like(passable)
    adj_unknown[1:, :] |= unknown[:-1, :]
    adj_unknown[:-1, :] |= unknown[1:, :]
    adj_unknown[:, 1:] |= unknown[:, :-1]
    adj_unknown[:, :-1] |= unknown[:, 1:]
    frontier_mask = passable & adj_unknown

    # Also detect depth-unobserved frontiers inside LiDAR-cleared passable space (vectorized).
    if observed_keys is not None:
        stride = max(1, int(round(quality_resolution / resolution)))
        ys = np.arange(0, height, stride)
        xs = np.arange(0, width, stride)
        wy = oy + (ys + 0.5) * resolution
        wx = ox + (xs + 0.5) * resolution
        y_ok = ys[(wy >= ymin) & (wy < ymax)]
        x_ok = xs[(wx >= xmin) & (wx < xmax)]
        if len(y_ok) and len(x_ok):
            sub_pass = passable[np.ix_(y_ok, x_ok)]
            py_idx, px_idx = np.nonzero(sub_pass)
            if len(px_idx):
                gx = ox + (x_ok[px_idx] + 0.5) * resolution
                gy = oy + (y_ok[py_idx] + 0.5) * resolution
                if math.isfinite(radius):
                    in_rad = np.hypot(gx - origin[0], gy - origin[1]) <= radius
                    px_idx, py_idx, gx, gy = px_idx[in_rad], py_idx[in_rad], gx[in_rad], gy[in_rad]
                qxs = np.floor(gx / quality_resolution).astype(np.int64)
                qys = np.floor(gy / quality_resolution).astype(np.int64)
                for idx_i in range(len(qxs)):
                    if (int(qxs[idx_i]), int(qys[idx_i])) not in observed_keys:
                        frontier_mask[int(y_ok[py_idx[idx_i]]), int(x_ok[px_idx[idx_i]])] = True

    fy, fx = np.nonzero(frontier_mask)
    if len(fx) == 0:
        return []

    # Filter to survey region + radius and sub-sample cleanly if very dense.
    points = []
    step = max(1, len(fx) // 600)
    for idx in range(0, len(fx), step):
        wx = ox + (int(fx[idx]) + 0.5) * resolution
        wy = oy + (int(fy[idx]) + 0.5) * resolution
        if inside_survey_region(region, wx, wy) and math.hypot(wx - origin[0], wy - origin[1]) <= radius:
            points.append((wx, wy))
    if not points:
        return []

    clusters = []
    for wx, wy in points:
        merged = False
        for cl in clusters:
            if math.hypot(wx - cl['cx'], wy - cl['cy']) <= cluster_radius_m:
                n = cl['count'] + 1
                cl['cx'] = (cl['cx'] * cl['count'] + wx) / n
                cl['cy'] = (cl['cy'] * cl['count'] + wy) / n
                cl['count'] = n
                merged = True
                break
        if not merged:
            clusters.append({'cx': wx, 'cy': wy, 'count': 1})

    min_count = max(1, min_cells // step)
    clusters = [cl for cl in clusters if cl['count'] >= min_count]
    clusters.sort(key=lambda cl: cl['count'], reverse=True)
    return clusters[:16]


def order_frontier_tour(pose, waypoints):
    """Order frontier viewpoints via greedy nearest-neighbor + 2-opt open TSP (FUEL-inspired)."""
    n = len(waypoints)
    if n <= 1:
        return list(range(n))

    def start_cost(idx):
        wp = waypoints[idx]
        dx, dy = wp['x'] - pose['x'], wp['y'] - pose['y']
        dist = wp.get('path_distance', math.hypot(dx, dy))
        heading_err = abs(wrap(math.atan2(dy, dx) - pose['yaw'])) if math.hypot(dx, dy) > 0.1 else 0.0
        # Slight preference for heading continuity and larger clusters.
        size_bonus = 1.0 + 0.15 * min(wp.get('cluster_count', 1), 20) / 20.0
        return (dist + 0.8 * heading_err) / size_bonus

    def edge_cost(a, b):
        wa, wb = waypoints[a], waypoints[b]
        return math.hypot(wa['x'] - wb['x'], wa['y'] - wb['y'])

    # Greedy nearest-neighbor seed
    remaining = set(range(n))
    first = min(remaining, key=start_cost)
    tour = [first]
    remaining.remove(first)
    while remaining:
        last = tour[-1]
        nxt = min(remaining, key=lambda idx: edge_cost(last, idx))
        tour.append(nxt)
        remaining.remove(nxt)

    # 2-opt refinement on open path (fixed start at pose)
    def total_cost(order):
        cost = start_cost(order[0])
        for i in range(len(order) - 1):
            cost += edge_cost(order[i], order[i + 1])
        return cost

    best_cost = total_cost(tour)
    improved = True
    while improved and n >= 3:
        improved = False
        for i in range(n - 1):
            for j in range(i + 1, n):
                candidate = tour[:i] + list(reversed(tour[i:j + 1])) + tour[j + 1:]
                c = total_cost(candidate)
                if c + 1e-6 < best_cost:
                    tour = candidate
                    best_cost = c
                    improved = True
    return tour


def find_standoff_viewpoint(cluster, costs, ox, oy, resolution, distances, parents,
                            pose, max_cost=55):
    """Find a reachable, chassis-clear standoff viewpoint facing a frontier cluster."""
    cx, cy = cluster['cx'], cluster['cy']
    base_dir = math.atan2(pose['y'] - cy, pose['x'] - cx)
    best = None
    best_dist = math.inf
    # Try radial standoffs around the cluster toward the rover and neighboring sectors.
    for d_ang in (0.0, math.pi / 4, -math.pi / 4, math.pi / 2, -math.pi / 2, 3 * math.pi / 4, -3 * math.pi / 4, math.pi):
        ang = base_dir + d_ang
        for ds in (1.5, 2.2, 3.0, 0.9):
            vx = cx + ds * math.cos(ang)
            vy = cy + ds * math.sin(ang)
            ix = int(math.floor((vx - ox) / resolution))
            iy = int(math.floor((vy - oy) / resolution))
            # Snap to nearest key in distances (handles stride > 1 cleanly).
            key = (ix, iy)
            if key not in distances:
                # Check immediate neighbors for stride-2 grid alignment
                matched = None
                for ox_i in (-1, 0, 1):
                    for oy_i in (-1, 0, 1):
                        cand = (ix + ox_i, iy + oy_i)
                        if cand in distances:
                            matched = cand
                            break
                    if matched is not None:
                        break
                if matched is None:
                    continue
                key = matched
                ix, iy = key
                vx = ox + (ix + 0.5) * resolution
                vy = oy + (iy + 0.5) * resolution
            if costs[iy, ix] > max_cost:
                continue
            if not known_chassis_clearance(costs, ix, iy, resolution):
                continue
            d = distances[key]
            if d < best_dist:
                best_dist = d
                path = reconstruct_path(parents, key, ox, oy, resolution)
                best = dict(
                    x=vx, y=vy,
                    yaw=float(wrap(math.atan2(cy - vy, cx - vx))),
                    path_distance=d,
                    path=path,
                    cluster_cx=cx, cluster_cy=cy,
                    cluster_count=cluster['count'],
                )
    return best


def choose_hybrid_view(grid, quality, pose, origin, radius, history, remaining,
                       *, current_gimbal=(0.0, GIMBAL_TILT_DEFAULT_RAD),
                       gimbal_available=True, depth_observed_visibility=False,
                       region=None, return_diagnostics=False):
    """Dual-layer planner (AEP + FUEL): local NBV with true gimbal decoupling + global frontier tour."""
    radius = math.inf if radius is None else radius
    region = scene_bounds('survey') if region is None else region
    xmin, ymin, xmax, ymax = region
    current_pan = float(current_gimbal[0]) if current_gimbal and len(current_gimbal) >= 1 else 0.0

    def inside_motion_region(x, y):
        return xmin + 0.7 <= x <= xmax - 0.7 and ymin + 0.7 <= y <= ymax - 0.7

    diagnostics = dict(
        planner_version='hybrid-frontier-v4.0',
        reachable_move_positions=0,
        candidate_orientations=0,
        history_rejected=0,
        low_gain_rejected=0,
        time_rejected=0,
        feasible_turns=0,
        feasible_moves=0,
        positive_gain_over_budget=0,
        movement_reservation_applied=False,
        budget_limited=False,
        frontier_cluster_count=0,
        global_frontier_fallback=False,
        global_tour_length=0,
    )

    def result(value):
        return (value, diagnostics) if return_diagnostics else value

    info = grid.info
    resolution = float(info.resolution)
    ox, oy = info.origin.position.x, info.origin.position.y
    costs = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)
    sx = int(math.floor((pose['x'] - ox) / resolution))
    sy = int(math.floor((pose['y'] - oy) / resolution))
    if not (0 <= sx < info.width and 0 <= sy < info.height):
        return result(None)

    qr = quality['resolution']
    cells = {
        key: cell for key, cell in scoped_cells(quality, origin, radius).items()
        if inside_survey_region(region, (key[0] + 0.5) * qr, (key[1] + 0.5) * qr)
    }
    observed = {k: c['quality'] for k, c in cells.items()}
    elevations = {k: c['z'] for k, c in cells.items() if c.get('z') is not None}
    gradients = {
        k: (c['gx'], c['gy']) for k, c in cells.items()
        if c.get('gx') is not None and c.get('gy') is not None
    }
    depth_supported = (
        {k for k, c in cells.items()
         if type(c.get('points')) in (int, float) and math.isfinite(c['points']) and c['points'] > 0}
        if depth_observed_visibility else set()
    )

    def blocks_visibility(cost, key):
        return cost >= 100 or (cost < 0 and (not depth_observed_visibility or key not in depth_supported))

    def is_grazing(key, ray_angle):
        grad = gradients.get(key)
        if grad is None:
            return False
        gx, gy = grad
        s = math.hypot(gx, gy)
        if s < 0.20:
            return False
        return abs(math.cos(ray_angle) * gx + math.sin(ray_angle) * gy) / s < 0.25

    def visibility(x, y, yaw, tilt=GIMBAL_TILT_DEFAULT_RAD):
        visible = set()
        cx = x + R5_CAMERA_FORWARD_M * math.cos(yaw)
        cy = y + R5_CAMERA_FORWARD_M * math.sin(yaw)
        half_hfov = R5_CAMERA_HFOV_RAD / 2.0

        def eligible_cell(key):
            px, py = (key[0] + 0.5) * qr, (key[1] + 0.5) * qr
            return inside_survey_region(region, px, py) and math.hypot(px - origin[0], py - origin[1]) <= radius

        if elevations:
            robot_key = (math.floor(x / qr), math.floor(y / qr))
            z_robot = elevations.get(robot_key, float(pose.get('z', 0.0)) if isinstance(pose, dict) else 0.0)
            cz = z_robot + R5_CAMERA_HEIGHT_M
            s_min = math.tan(tilt - R5_CAMERA_HALF_VFOV_RAD)
            s_max = math.tan(tilt + R5_CAMERA_HALF_VFOV_RAD)
            for angle in np.linspace(yaw - half_hfov, yaw + half_hfov, 13):
                max_slope = s_min
                ca, sa = math.cos(angle), math.sin(angle)
                for r in np.arange(0.35, R5_CAMERA_MAX_RANGE_M + 0.01, 0.35):
                    px, py = cx + r * ca, cy + r * sa
                    if math.hypot(px - origin[0], py - origin[1]) > radius or not inside_survey_region(region, px, py):
                        break
                    ix = int(math.floor((px - ox) / resolution))
                    iy = int(math.floor((py - oy) / resolution))
                    if not (0 <= ix < info.width and 0 <= iy < info.height):
                        break
                    cost = int(costs[iy, ix])
                    key = (math.floor(px / qr), math.floor(py / qr))
                    zt = elevations.get(key, z_robot)
                    slope = (zt - cz) / r
                    if slope < max_slope:
                        if blocks_visibility(cost, key):
                            break
                        continue
                    if slope <= s_max:
                        if r >= R5_CAMERA_MIN_RANGE_M and eligible_cell(key) and not is_grazing(key, angle):
                            visible.add(key)
                        max_slope = slope
                    else:
                        if r >= R5_CAMERA_MIN_RANGE_M and eligible_cell(key) and not is_grazing(key, angle):
                            visible.add(key)
                        break
                    if blocks_visibility(cost, key):
                        break
        else:
            for angle in np.linspace(yaw - half_hfov, yaw + half_hfov, 13):
                ca, sa = math.cos(angle), math.sin(angle)
                for r in np.arange(0.35, R5_CAMERA_MAX_RANGE_M + 0.01, 0.35):
                    px, py = cx + r * ca, cy + r * sa
                    if math.hypot(px - origin[0], py - origin[1]) > radius or not inside_survey_region(region, px, py):
                        break
                    ix = int(math.floor((px - ox) / resolution))
                    iy = int(math.floor((py - oy) / resolution))
                    if not (0 <= ix < info.width and 0 <= iy < info.height):
                        break
                    cost = int(costs[iy, ix])
                    key = (math.floor(px / qr), math.floor(py / qr))
                    if r >= R5_CAMERA_MIN_RANGE_M and eligible_cell(key) and not is_grazing(key, angle):
                        visible.add(key)
                    if blocks_visibility(cost, key):
                        break
        return visible, (cx, cy)

    # 1. Global Frontier Extraction & Tour Ordering (AEP + FUEL)
    clusters = extract_frontier_clusters(
        costs, ox, oy, resolution, region, origin, radius,
        observed_keys=observed, quality_resolution=qr,
    )
    diagnostics['frontier_cluster_count'] = len(clusters)

    global_stride = max(1, int(round(0.2 / resolution)))
    global_distances, global_parents = run_dijkstra(
        costs, ox, oy, resolution, pose, origin, radius, region,
        max_cost_distance=45.0, stride=global_stride,
    )
    frontier_waypoints = []
    for cl in clusters:
        wp = find_standoff_viewpoint(cl, costs, ox, oy, resolution, global_distances, global_parents, pose)
        if wp is not None:
            # Skip already-blocked frontier standoffs
            if any(math.hypot(wp['x'] - h['x'], wp['y'] - h['y']) < 0.7 and h.get('blocked') for h in history[-80:]):
                continue
            frontier_waypoints.append(wp)

    tour_order = order_frontier_tour(pose, frontier_waypoints)
    diagnostics['global_tour_length'] = len(tour_order)
    primary_global_wp = frontier_waypoints[tour_order[0]] if tour_order else None
    global_dir = (
        math.atan2(primary_global_wp['y'] - pose['y'], primary_global_wp['x'] - pose['x'])
        if primary_global_wp is not None and math.hypot(primary_global_wp['x'] - pose['x'], primary_global_wp['y'] - pose['y']) > 0.5
        else None
    )

    # 2. Local NBV Reachability & Candidate Generation (Bircher RH-NBV + AEP Local)
    local_distances, local_parents = run_dijkstra(
        costs, ox, oy, resolution, pose, origin, radius, region,
        max_cost_distance=7.5, stride=1,
    )
    positions = [(pose['x'], pose['y'], 0.0, [(pose['x'], pose['y'])], None)]
    for distance in (1.5, 2.5, 3.8):
        for angle in np.arange(0.0, 2.0 * math.pi, math.pi / 8.0):
            x = pose['x'] + distance * math.cos(angle)
            y = pose['y'] + distance * math.sin(angle)
            ix = int(math.floor((x - ox) / resolution))
            iy = int(math.floor((y - oy) / resolution))
            if (ix, iy) not in local_distances or costs[iy, ix] > 55:
                continue
            if not known_chassis_clearance(costs, ix, iy, resolution):
                continue
            if any(math.hypot(x - p[0], y - p[1]) < 0.35 for p in positions):
                continue
            path = reconstruct_path(local_parents, (ix, iy), ox, oy, resolution)
            positions.append((x, y, local_distances[(ix, iy)], path, None))

    # Inject local standoff viewpoints toward the top 4 tour frontiers if locally reachable.
    for idx in tour_order[:4]:
        wp = frontier_waypoints[idx]
        if wp['path_distance'] <= 7.0 and not any(math.hypot(wp['x'] - p[0], wp['y'] - p[1]) < 0.35 for p in positions):
            positions.append((wp['x'], wp['y'], wp['path_distance'], wp['path'], wp['yaw']))

    diagnostics['reachable_move_positions'] = len(positions) - 1

    # 3. Evaluate Local NBV Candidates with True Chassis-Gimbal Decoupling
    candidates = []
    recenter_penalty_s = (abs(current_pan) / 1.5 + 1.0) if (gimbal_available and abs(current_pan) > GIMBAL_RECENTER_THRESHOLD_RAD) else 0.0

    for x, y, distance, path, hint_yaw in positions:
        if distance and not inside_motion_region(x, y):
            continue
        route_visible = set()
        route_cameras = {}
        if distance:
            step = max(1, round(0.75 / resolution))
            sampled = path[::step]
            if sampled[-1] != path[-1]:
                sampled.append(path[-1])
            for i, (px, py) in enumerate(sampled[:-1]):
                nx, ny = sampled[i + 1]
                # Because the rover re-centers the gimbal to pan=0 before moving,
                # route visibility genuinely aligns with travel heading!
                seen, camera = visibility(px, py, math.atan2(ny - py, nx - px))
                route_visible.update(seen)
                for q in seen:
                    route_cameras.setdefault(q, camera)

        target_yaws = list(np.arange(-math.pi, math.pi, math.pi / 4.0))
        if hint_yaw is not None and not any(abs(wrap(hint_yaw - y_)) < 0.2 for y_ in target_yaws):
            target_yaws.append(hint_yaw)

        for yaw in target_yaws:
            diagnostics['candidate_orientations'] += 1
            if any(
                math.hypot(x - h['x'], y - h['y']) < 0.7
                and (h.get('blocked') or abs(wrap(yaw - h['yaw'])) < 0.6)
                for h in history[-80:]
            ):
                diagnostics['history_rejected'] += 1
                continue

            bearing = math.atan2(y - pose['y'], x - pose['x']) if distance else pose['yaw']
            if not distance:
                pan = float(wrap(yaw - pose['yaw']))
                tilt = GIMBAL_TILT_DEFAULT_RAD
                if gimbal_available and abs(pan) <= GIMBAL_PAN_LIMIT_RAD:
                    seconds = abs(pan - current_pan) / 1.5 + 1.8
                    use_pan_tilt = True
                    requires_chassis_spin = False
                elif gimbal_available:
                    # Skip 180-deg in-place chassis spin when gimbal is available;
                    # rear blind spots are observed during motion or from adjacent stops.
                    continue
                else:
                    seconds = abs(pan) / 0.3 + 2.5
                    use_pan_tilt = False
                    requires_chassis_spin = True
            else:
                turn_to_bearing = abs(wrap(bearing - pose['yaw']))
                arrival_pan = float(wrap(yaw - bearing))
                tilt = GIMBAL_TILT_DEFAULT_RAD
                if gimbal_available and abs(arrival_pan) <= GIMBAL_PAN_LIMIT_RAD:
                    # True chassis-gimbal decoupling: drive forward (camera centered),
                    # then pan ONLY the gimbal at the destination (zero chassis spin!).
                    # Include the round-trip re-centering cost so we only pan sideways
                    # when side gain genuinely outweighs continuous forward exploration.
                    use_pan_tilt = True
                    pan = arrival_pan
                    requires_chassis_spin = False
                    arrival_look_s = (abs(arrival_pan) / 1.5 + 1.2) if abs(arrival_pan) > 0.12 else 0.8
                    seconds = recenter_penalty_s + distance / 0.16 + turn_to_bearing / 0.35 + arrival_look_s + 1.5
                else:
                    use_pan_tilt = False
                    pan = 0.0
                    requires_chassis_spin = abs(arrival_pan) > 0.12
                    arrival_spin_s = (abs(arrival_pan) / 0.3 + 2.0) if requires_chassis_spin else 0.8
                    seconds = recenter_penalty_s + distance / 0.16 + turn_to_bearing / 0.35 + arrival_spin_s + 2.0

            endpoint_visible, camera = visibility(x, y, yaw, tilt=tilt)
            visible = set(endpoint_visible)
            cameras = {q: camera for q in visible}
            visible.update(route_visible)
            for q, route_camera in route_cameras.items():
                cameras.setdefault(q, route_camera)

            new_m2 = sum(q not in observed for q in visible) * qr * qr
            improve_m2 = sum(
                resampling_value(cells[q], q, qr, cameras[q]) for q in visible if q in cells
            ) * qr * qr
            # Prioritize genuine new terrain coverage over minor resampling.
            utility = new_m2 + 0.6 * improve_m2
            if utility < 0.25:
                diagnostics['low_gain_rejected'] += 1
                continue

            kind = 'turn' if distance < 0.1 else ('resample' if improve_m2 > new_m2 else 'advance')
            # Smooth heading continuity + global tour directional bias (FUEL-inspired).
            direction_bias = 1.0
            if distance >= 0.1:
                direction_bias += 0.20 * math.cos(wrap(bearing - pose['yaw']))
                if global_dir is not None:
                    direction_bias += 0.25 * math.cos(wrap(bearing - global_dir))
                if requires_chassis_spin:
                    direction_bias *= 0.75
            score = (utility / seconds) * max(0.4, direction_bias)

            candidate = dict(
                x=round(x, 3), y=round(y, 3), yaw=float(yaw), kind=kind,
                pan=round(pan, 3), tilt=round(tilt, 3),
                use_pan_tilt=use_pan_tilt,
                requires_chassis_spin=requires_chassis_spin,
                expected_new_m2=round(new_m2, 3),
                expected_improvement_m2=round(improve_m2, 3),
                expected_utility_m2=round(utility, 3),
                visible_cells=sorted(visible),
                strategy='hybrid_frontier_v4',
                estimated_seconds=round(seconds, 1),
                score=float(score),
            )
            if distance:
                candidate['enroute_visible_cells'] = len(visible - endpoint_visible)
            if seconds > remaining:
                diagnostics['time_rejected'] += 1
                diagnostics['positive_gain_over_budget'] += 1
                continue
            diagnostics['feasible_turns' if kind == 'turn' else 'feasible_moves'] += 1
            candidates.append(candidate)

    diagnostics['budget_limited'] = not candidates and diagnostics['positive_gain_over_budget'] > 0
    best = max(candidates, key=lambda c: c['score'], default=None)

    # Prefer advancing when stationary gimbal gain is small or immediately after another stationary scan.
    moves = [c for c in candidates if c['kind'] != 'turn']
    if best is not None and best['kind'] == 'turn' and moves:
        last_was_turn = bool(
            history and history[-1].get('kind') == 'turn'
            and math.hypot(pose['x'] - history[-1].get('x', 0), pose['y'] - history[-1].get('y', 0)) < 0.6
        )
        if last_was_turn or best['expected_new_m2'] < 2.2:
            best = max(moves, key=lambda c: c['score'])

    # 4. Global Frontier Switch (Selin AEP): when local new gain is depleted, drive toward the
    #    first reachable cluster on the 2-opt global frontier tour instead of meandering locally!
    local_depleted = (best is None) or (best['expected_new_m2'] < LOCAL_NEW_GAIN_SWITCH_M2)
    if local_depleted and tour_order and not diagnostics['budget_limited']:
        for idx in tour_order:
            wp = frontier_waypoints[idx]
            path = wp['path']
            if not path or len(path) < 2:
                continue
            # Pick a bounded intermediate waypoint up to GLOBAL_WAYPOINT_STEP_M along the safe path.
            target_pt = path[-1]
            acc = 0.0
            for i in range(1, len(path)):
                acc += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
                if acc >= GLOBAL_WAYPOINT_STEP_M:
                    px, py = path[i]
                    ix = int(math.floor((px - ox) / resolution))
                    iy = int(math.floor((py - oy) / resolution))
                    if inside_motion_region(px, py) and known_chassis_clearance(costs, ix, iy, resolution):
                        target_pt = (px, py)
                        break
            gx, gy = target_pt
            if not inside_motion_region(gx, gy):
                continue
            dist_to_target = math.hypot(gx - pose['x'], gy - pose['y'])
            if dist_to_target < 0.6:
                continue
            if any(math.hypot(gx - h['x'], gy - h['y']) < 0.7 and h.get('blocked') for h in history[-80:]):
                continue
            bearing = math.atan2(gy - pose['y'], gx - pose['x'])
            is_final_standoff = math.hypot(gx - wp['x'], gy - wp['y']) < 0.5
            desired_yaw = wp['yaw'] if is_final_standoff else bearing
            arrival_pan = float(wrap(desired_yaw - bearing))
            use_pan_tilt = bool(gimbal_available and abs(arrival_pan) <= GIMBAL_PAN_LIMIT_RAD)
            pan = arrival_pan if use_pan_tilt else 0.0
            seconds = recenter_penalty_s + dist_to_target / 0.16 + abs(wrap(bearing - pose['yaw'])) / 0.35 + 3.0
            if seconds > remaining:
                diagnostics['time_rejected'] += 1
                diagnostics['positive_gain_over_budget'] += 1
                continue
            visible, _ = visibility(gx, gy, desired_yaw)
            new_m2 = sum(q not in observed for q in visible) * qr * qr
            global_candidate = dict(
                x=round(gx, 3), y=round(gy, 3), yaw=float(desired_yaw), kind='advance',
                pan=round(pan, 3), tilt=round(GIMBAL_TILT_DEFAULT_RAD, 3),
                use_pan_tilt=use_pan_tilt,
                requires_chassis_spin=False,
                expected_new_m2=round(new_m2, 3),
                expected_improvement_m2=0.0,
                expected_utility_m2=round(max(new_m2, wp['cluster_count'] * qr * qr * 0.25), 3),
                visible_cells=sorted(visible),
                strategy='hybrid_frontier_v4',
                estimated_seconds=round(seconds, 1),
                score=float(max(new_m2, 1.0) / seconds),
                global_frontier_target=True,
                frontier_cluster=[round(wp['cluster_cx'], 2), round(wp['cluster_cy'], 2)],
            )
            diagnostics['global_frontier_fallback'] = True
            return result(global_candidate)

    diagnostics['budget_limited'] = best is None and diagnostics['positive_gain_over_budget'] > 0
    return result(best)
