"""Read-only checks against recorded runtime evidence. Uses existing NumPy."""
import base64
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import numpy as np

root=Path(__file__).parent;repo=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(repo/'scripts'));sys.path.insert(0,str(repo/'ros2_ws/src/mooncar_terrain/scripts'))
from terrain_map import TerrainMap
from autonomous_survey import choose_view

def read(name):return json.loads((root/name).read_text(encoding='utf-8-sig'))
mission=read('mission-180s.json');final=read('final-state.json')
assert not mission['active'] and not final['motion']['active']
assert 170<=mission['elapsed_seconds']<=mission['budget_seconds']==180
assert len(mission['history'])==18 and all(h['success'] for h in mission['history'])
assert {h['kind'] for h in mission['history']}=={'advance','turn'}
assert read('after-page-reload.json')['autonomy']['active']
assert (root/'budget-stop-velocity.txt').read_text(encoding='utf-8-sig').count('0.0')==6
terrain=TerrainMap.load(root/'observations-180s.npz');q=terrain.quality()
assert len(terrain.frames)>200
assert len(set(c['quality'] for c in q['cells']))>10
assert all(0<=c['quality']<=1 for c in q['cells'])
g=final['motion']['navigation_grid']
grid=NS(info=NS(width=g['width'],height=g['height'],resolution=g['resolution'],origin=NS(position=NS(x=g['origin'][0],y=g['origin'][1]))),
        data=np.frombuffer(base64.b64decode(g['data']),dtype=np.int8))
args=(grid,q,final['motion']['pose'],mission['origin'],mission['radius'],mission['history'])
short=choose_view(*args,mission['remaining_seconds']);long=choose_view(*args,180)
assert short is None and long is not None, (short,long)
second=read('mission-resample.json')
assert second['phase']=='stopped' and not second['active']
assert any(h['kind']=='resample' and h['success'] and h['actual_quality_gain_m2']>0 for h in second['history'])
assert np.linalg.norm(np.array(second['origin'])-np.array([final['motion']['pose']['x'],final['motion']['pose']['y']]))<.1
assert not read('manual-stop-state.json')['motion']['active']
assert (root/'manual-stop-velocity.txt').read_text(encoding='utf-8-sig').count('0.0')==6
print(json.dumps(dict(result='PASS',actions=len(mission['history']),frames=len(terrain.frames),
                     elapsed=mission['elapsed_seconds'],next_action_seconds=long['estimated_seconds']),ensure_ascii=False))
