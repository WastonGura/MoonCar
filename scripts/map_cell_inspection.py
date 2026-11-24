"""Read-only explanations for a point in the live navigation map."""
import base64
import math


def inspect_cell(grid, evidence, keepout, x, y):
    result = dict(status='unavailable')
    if not grid:
        return result
    ox, oy = grid['origin']
    r = grid['resolution']
    width, height = grid['width'], grid['height']
    col, row = math.floor((x - ox) / r), math.floor((y - oy) / r)
    result.update(stamp=grid['stamp'], resolution=r)
    if not (0 <= col < width and 0 <= row < height):
        return dict(result, status='outside')
    center = [ox + (col + .5) * r, oy + (row + .5) * r]
    costs = base64.b64decode(grid['data'])
    cost = costs[row * width + col]
    status = ('unknown' if cost == 255 else 'hazard' if cost >= 100 else
              'buffer' if cost == 99 else 'restricted' if cost else 'passable')
    result.update(status=status, center=center, cost=-1 if cost == 255 else cost)

    def evidence_at(point):
        if not evidence or evidence['frame'] != 'map':
            return None
        s = evidence['resolution']
        ix, iy = (math.floor(v / s) for v in point)
        item = evidence['cells'].get(f'{ix},{iy}')
        return dict(item, resolution=s, snapshot_stamp=evidence['stamp']) if item else None

    result['terrain'] = evidence_at(center)
    result['keepout'] = in_keepout(keepout, center)
    if status == 'buffer':
        # Look for a lethal grid cell in the configured inflation neighborhood.
        # Proximity is evidence, since the composed grid has no source IDs.
        n = math.ceil(.65 / r)
        nearby = [(math.hypot(c - col, j - row) * r, c, j)
                  for j in range(max(0, row - n), min(height, row + n + 1))
                  for c in range(max(0, col - n), min(width, col + n + 1))
                  if costs[j * width + c] == 100]
        if nearby:
            distance, c, j = min(nearby)
            if distance <= .65:
                source = [ox + (c + .5) * r, oy + (j + .5) * r]
                result['nearby_hazard'] = dict(center=source, distance=distance,
                                                terrain=evidence_at(source), keepout=in_keepout(keepout, source))
    return result


def in_keepout(mask, point):
    if mask is None:
        return None
    origin = mask.info.origin
    q = origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = point[0] - origin.position.x, point[1] - origin.position.y
    r = mask.info.resolution
    col, row = math.floor((c * dx + s * dy) / r), math.floor((-s * dx + c * dy) / r)
    return (0 <= col < mask.info.width and 0 <= row < mask.info.height
            and mask.data[row * mask.info.width + col] == 100)
