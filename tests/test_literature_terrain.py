import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(ROOT/'ros2_ws/src/mooncar_terrain/scripts'))
from compare_terrain_strategies import synthetic_scenes
from literature_terrain import ElevationNeighborhood
from terrain_geometry import classify_cells


def test_recorded_sparse_pebble_does_not_establish_a_whole_steep_cell():
    evidence=json.loads((ROOT/'reports/turn_minimap_20261111/hazard-classification.json').read_text())
    points=evidence['reconstructed_keyframe']['points']
    baseline=classify_cells(points,resolution=.2)
    assert baseline[0][4]&1
    assert ElevationNeighborhood().update(points,0.)==[]


@pytest.mark.parametrize('name,points',list(synthetic_scenes()))
def test_ground_hazards_remain_distinct_from_missing_support(name,points):
    candidate=ElevationNeighborhood()
    candidate.update(points,0.)
    result=candidate.update(points,.1)
    if name=='single_scanline':
        assert result==[]
        return
    half_x=.2 if name in ('raised_25cm','drop_25cm') else .4
    cells=[c for c in result if abs(c[0])<half_x and abs(c[1])<.4]
    if name.startswith('isolated_'):
        cells=[c for c in result if 0<c[0]<.2 and 0<c[1]<.2]
    assert cells
    if name in ('flat','gentle_10deg'):
        assert all(c[4]==0 for c in cells)
    else:
        assert all(c[4]!=0 for c in cells)


def test_new_height_hazard_is_not_averaged_into_old_ground():
    scenes=dict(synthetic_scenes())
    candidate=ElevationNeighborhood()
    for stamp in range(6):
        candidate.update(scenes['flat'],float(stamp))
    result=candidate.update(scenes['isolated_obstacle_25cm'],6.)
    cell=next(c for c in result if 0<c[0]<.2 and 0<c[1]<.2)
    assert cell[4]&4


def test_failed_or_duplicate_updates_leave_the_map_unchanged():
    candidate=ElevationNeighborhood(max_cells=10)
    candidate.update([[.01,.01,0.]],1.)
    before={k:dict(v) for k,v in candidate.cells.items()}
    with pytest.raises(ValueError):
        candidate.update([[.01,.01,.3]],1.)
    with pytest.raises(ValueError):
        candidate.update([[float('inf'),0.,0.]],float('nan'))
    points=np.column_stack((np.arange(20)*.05,np.zeros(20),np.zeros(20)))
    with pytest.raises(ValueError):
        candidate.update(points,2.)
    assert candidate.cells==before
    assert candidate.last_stamp==1.
    assert candidate.update([],7.)==[]
    assert candidate.cells=={}
