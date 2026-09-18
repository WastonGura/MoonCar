from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from report_heading_control import compare


def row(ix,recovered,points='3',surface='sloped_top'):
    return dict(ix=str(ix),iy='0',eligible='True',baseline_surface=surface,
                initial_state='too_few_voxels',initial_points=points,
                initial_spatial_coverage='0.2',initial_view_directions='1',recovered=recovered)


if __name__=='__main__':
    a=[row(0,'False'),row(1,'True'),row(2,''),row(9,'False')]
    b=[row(0,'True'),row(1,'False','4'),row(2,'True'),row(8,'True')]
    r=compare(a,b)
    assert r['common_cells']==3 and r['missing_pairs']==1
    assert r['net_recovered']==0 and r['turn_only']==r['no_turn_only']==1
    exact=compare(a,b,exact=True)
    assert exact['common_cells']==2 and exact['net_recovered']==1 and exact['missing_pairs']==1
    assert compare(a,b,target='curved_top')['common_cells']==0
    assert compare([],[])['turn_rate'] is None
    print('PASS: paired baseline intersection, missing outcomes, matched summaries and target strata')
