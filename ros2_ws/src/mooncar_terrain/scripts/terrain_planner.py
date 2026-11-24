"""Eight-heading terrain search, shared motion/constraints for all ablations.

Research prototype for a skid-steer rover, not a stability certification. Unknown
footprint coverage is blocked. In-place turns are swept and costed explicitly.
"""
from dataclasses import asdict, dataclass
import heapq
import itertools
import math
import time


@dataclass(frozen=True)
class Vehicle:
    length: float = .55
    width: float = .45
    max_pitch: float = 18.
    max_roll: float = 12.
    max_step: float = .12
    max_roughness: float = .04
    min_support: float = .15
    terrain_weight: float = 2.
    uncertainty_weight: float = 2.
    turn_weight: float = .15

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in asdict(self).values()):
            raise ValueError('Vehicle parameters must be finite and positive')
        if self.min_support > 1 or max(self.max_pitch, self.max_roll) >= 60:
            raise ValueError('Invalid terrain limits')


HEADINGS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
MODES = ('geometry', 'fixed', 'direction', 'uncertainty', 'full')


class TerrainPlanner:
    def __init__(self, cells, resolution, vehicle=None):
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('Invalid resolution')
        self.resolution = resolution
        self.vehicle = vehicle or Vehicle()
        self.cells = {}
        for cell in cells:
            key = (cell['ix'], cell['iy'])
            if any(type(v) is not int for v in key) or key in self.cells:
                raise ValueError('Invalid or duplicate cell indices')
            if cell['valid']:
                for field in ('z', 'gx', 'gy', 'roughness', 'support'):
                    if not isinstance(cell[field], (int, float)) or not math.isfinite(cell[field]):
                        raise ValueError('Invalid terrain cell')
                if not 0 <= cell['support'] <= 1 or cell['roughness'] < 0:
                    raise ValueError('Invalid terrain support or roughness')
            self.cells[key] = cell

    def footprint(self, x, y, yaw):
        """Intersect oriented rectangle and grid squares with separating axes."""
        r, v = self.resolution, self.vehicle
        c, s = math.cos(yaw), math.sin(yaw)
        hx, hy = v.length / 2, v.width / 2
        bx, by = abs(c) * hx + abs(s) * hy, abs(s) * hx + abs(c) * hy
        for ix in range(math.floor((x - bx) / r), math.floor((x + bx) / r) + 1):
            for iy in range(math.floor((y - by) / r), math.floor((y + by) / r) + 1):
                dx, dy = (ix + .5) * r - x, (iy + .5) * r - y
                if (abs(dx) <= bx + r / 2 and abs(dy) <= by + r / 2
                        and abs(dx * c + dy * s) <= hx + r / 2 * (abs(c) + abs(s))
                        and abs(-dx * s + dy * c) <= hy + r / 2 * (abs(c) + abs(s))):
                    yield ix, iy

    def assess(self, x, y, yaw):
        v, r = self.vehicle, self.resolution
        covered = set(self.footprint(x, y, yaw))
        pitch, roll, rough, uncertainty, slope = 0., 0., 0., 0., 0.
        for key in covered:
            cell = self.cells.get(key)
            if not cell or not cell['valid'] or cell['support'] < v.min_support:
                return None
            gx, gy = cell['gx'], cell['gy']
            p = math.degrees(math.atan(abs(gx * math.cos(yaw) + gy * math.sin(yaw))))
            b = math.degrees(math.atan(abs(-gx * math.sin(yaw) + gy * math.cos(yaw))))
            if p > v.max_pitch or b > v.max_roll or cell['roughness'] > v.max_roughness:
                return None
            pitch, roll = max(pitch, p), max(roll, b)
            slope = max(slope, math.degrees(math.atan(math.hypot(gx, gy))))
            rough, uncertainty = max(rough, cell['roughness']), max(uncertainty, 1 - cell['support'])
            for dx, dy in ((1, 0), (0, 1)):
                other = self.cells.get((key[0] + dx, key[1] + dy))
                if (key[0] + dx, key[1] + dy) in covered and other and other['valid']:
                    edge_a = cell['z'] + (gx * dx + gy * dy) * r / 2
                    edge_b = other['z'] - (other['gx'] * dx + other['gy'] * dy) * r / 2
                    if abs(edge_a - edge_b) > v.max_step:
                        return None
        return dict(pitch=pitch, roll=roll, roughness=rough, uncertainty=uncertainty, slope=slope)

    def risk(self, assessment, mode):
        v = self.vehicle
        if mode == 'geometry':
            return 0.
        if mode in ('direction', 'full'):
            terrain = (assessment['pitch'] / v.max_pitch) ** 2 + (assessment['roll'] / v.max_roll) ** 2
        else:
            terrain = (assessment['slope'] / max(v.max_pitch, v.max_roll)) ** 2
        terrain += assessment['roughness'] / v.max_roughness
        return v.terrain_weight * terrain + (v.uncertainty_weight * assessment['uncertainty']
                                             if mode in ('uncertainty', 'full') else 0.)

    def motion_cost(self, a, b, mode='full'):
        """Cost a swept in-place turn or straight forward segment; None is blocked."""
        delta = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        distance = math.hypot(b[0] - a[0], b[1] - a[1])
        if distance > 1e-8:
            bearing = math.atan2(b[1] - a[1], b[0] - a[0])
            alignment = math.atan2(math.sin(bearing - a[2]), math.cos(bearing - a[2]))
            if abs(delta) > 1e-6 or abs(alignment) > 1e-6:
                return None
        samples = max(1, math.ceil(distance / (self.resolution / 4)), math.ceil(abs(delta) / (math.pi / 24)))
        maximum = 0.
        for j in range(samples + 1):
            t = j / samples
            value = self.assess(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * delta)
            if value is None:
                return None
            maximum = max(maximum, self.risk(value, mode))
        return (distance + self.vehicle.turn_weight * abs(delta)) * (1 + maximum)

    def plan(self, start, goal, mode='full', timeout=5., max_expansions=100000,
             cancelled=None, goal_yaw=None):
        """Search from the actual start, with checked connectors to the exact goal.

        Motion is an explicit sequence of straight segments and stationary turns.
        A controller that smooths these corners needs a separate swept-path check.
        """
        begin = time.monotonic()
        if mode not in MODES or len(start) != 3 or len(goal) != 2 or not all(math.isfinite(n) for n in (*start, *goal)):
            raise ValueError('Invalid planning request')
        if not math.isfinite(timeout) or not 0 < timeout <= 30 or not 1 <= max_expansions <= 1000000:
            raise ValueError('Invalid search budget')
        if goal_yaw is not None and not math.isfinite(goal_yaw):
            raise ValueError('Invalid goal orientation')
        r = self.resolution
        root = (math.floor(start[0] / r), math.floor(start[1] / r), round(start[2] / (math.pi / 4)) % 8)
        serial = itertools.count()
        queue, best, parents = [], {root: 0.}, {}
        cache = {}
        expansions = 0
        solution = None

        def pose(state):
            return (start[0] + (state[0] - root[0]) * r,
                    start[1] + (state[1] - root[1]) * r, state[2] * math.pi / 4)

        def assessment(state):
            if state not in cache:
                cache[state] = self.assess(*pose(state))
            return cache[state]

        def finish(status, poses=(), cost=None):
            path = []
            previous = None
            for x, y, yaw in poses:
                if previous is not None and (x, y, yaw) == previous:
                    continue
                cell = self.cells[(math.floor(x / r), math.floor(y / r))]
                z = cell['z'] + cell['gx'] * (x - (cell['ix'] + .5) * r) + cell['gy'] * (y - (cell['iy'] + .5) * r)
                path.append(dict(x=x, y=y, yaw=yaw, z=z))
                previous = x, y, yaw
            length = sum(math.hypot(b['x'] - a['x'], b['y'] - a['y']) for a, b in zip(path, path[1:]))
            return dict(status=status, mode=mode, path=path, length=length, cost=cost,
                        compute_ms=(time.monotonic() - begin) * 1000, expansions=expansions,
                        requested_start=list(start), requested_goal=list(goal),
                        endpoint_tolerance=0., requested_goal_yaw=goal_yaw, vehicle=asdict(self.vehicle))

        if self.assess(*start) is None or assessment(root) is None:
            return finish('invalid_start')
        root_cost = self.motion_cost(start, pose(root), mode)
        if root_cost is None:
            return finish('invalid_start')
        if not any(self.assess(goal[0], goal[1], yaw) is not None
                   for yaw in ([goal_yaw] if goal_yaw is not None else [h * math.pi / 4 for h in range(8)])):
            return finish('invalid_goal')
        best[root] = root_cost
        heapq.heappush(queue, (root_cost + math.hypot(goal[0] - start[0], goal[1] - start[1]), next(serial), root, root_cost))
        while queue:
            if cancelled is not None and cancelled():
                return finish('cancelled')
            if time.monotonic() - begin > timeout or expansions >= max_expansions:
                return finish('budget_exceeded')
            bound, _, current, queued_cost = heapq.heappop(queue)
            if solution is not None and bound >= solution[0] - 1e-12:
                return finish('ok', solution[1], solution[0])
            if queued_cost != best[current]:
                continue
            expansions += 1
            a = pose(current)
            # The nearest lattice point may be blocked even when the exact goal is safe.
            # Also connect from adjacent points, retaining every swept-motion check.
            if math.hypot(goal[0] - a[0], goal[1] - a[1]) <= r * math.sqrt(2) + 1e-8:
                bearing = math.atan2(goal[1] - a[1], goal[0] - a[0]) if math.hypot(goal[0] - a[0], goal[1] - a[1]) > 1e-8 else a[2]
                tail = [a, (a[0], a[1], bearing), (goal[0], goal[1], bearing)]
                if goal_yaw is not None:
                    tail.append((goal[0], goal[1], goal_yaw))
                costs = [self.motion_cost(u, v, mode) for u, v in zip(tail, tail[1:])]
                if all(c is not None for c in costs):
                    total = best[current] + sum(costs)
                    if solution is None or total < solution[0]:
                        route = [current]
                        while route[-1] in parents:
                            route.append(parents[route[-1]])
                        solution = total, [tuple(start)] + [pose(s) for s in reversed(route)] + tail[1:]
            dx, dy = HEADINGS[current[2]]
            candidates = [(current[0] + dx, current[1] + dy, current[2]),
                          (current[0], current[1], (current[2] + 1) % 8),
                          (current[0], current[1], (current[2] - 1) % 8)]
            for nxt in candidates:
                end = assessment(nxt)
                if end is None:
                    continue
                a, b = pose(current), pose(nxt)
                increment = self.motion_cost(a, b, mode)
                if increment is not None:
                    cost = best[current] + increment
                    if cost < best.get(nxt, math.inf):
                        best[nxt], parents[nxt] = cost, current
                        heuristic = math.hypot(b[0] - goal[0], b[1] - goal[1])
                        heapq.heappush(queue, (cost + heuristic, next(serial), nxt, cost))
        return finish('ok', solution[1], solution[0]) if solution is not None else finish('no_path')
