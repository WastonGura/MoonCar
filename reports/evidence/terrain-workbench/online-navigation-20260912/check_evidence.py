"""Offline check: python check_evidence.py (standard library only)."""
import base64
import json
import math
from pathlib import Path

root = Path(__file__).parent
def read(name):
    return json.loads((root / name).read_text(encoding='utf-8-sig'))

arrival = read('arrival.json')['motion']
truth = next(p['position'] for p in read('gazebo-arrival.json')['pose'] if p['name'] == 'mooncar')
error = math.hypot(truth['x'] - 9, truth['y'] - 3)
assert arrival['phase'] == 'succeeded' and not arrival['active']
assert error < .25, error
samples = read('motion-samples.json')
assert max(s['path_points'] for s in samples) > 10
assert arrival['map_bounds'][2] > 19
assert arrival['map_bounds'][2] - min(s['bounds'][2] for s in samples) > 3
failure = read('regression-result.json')['motion']
assert failure['phase'] == 'failed' and not failure['active'] and '208' in failure['error']
g = failure['navigation_grid']
x, y = [math.floor((v - o) / g['resolution']) for v, o in zip(failure['goal'], g['origin'])]
cost = base64.b64decode(g['data'])[y * g['width'] + x]
assert 99 <= cost <= 100, cost
print(f'PASS: arrival truth error {error:.3f} m; map expanded; blocked-goal regression retained (cost {cost}).')
