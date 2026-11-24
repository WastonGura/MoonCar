import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from surface_quality_evaluation import surface_labels,compare_cell,statistics
from terrain_map import TerrainMap


class SurfaceQualityTests(unittest.TestCase):
    def test_independent_faces_and_rotated_slope(self):
        angle=.2;pose=np.eye(4);pose[:3,:3]=[[math.cos(angle),0,math.sin(angle)],[0,1,0],[-math.sin(angle),0,math.cos(angle)]]
        shapes=[('ramp','box',np.array([1,1,.1]),pose),('rock','ellipsoid',np.ones(3),np.eye(4))]
        points=np.array([[0,0,.1],[1,0,0]])@pose[:3,:3].T
        labels=surface_labels(shapes,np.array([0,0,1,1]),np.vstack((points,[0,0,1],[1,0,0])))
        self.assertEqual([x[2] for x in labels],['sloped_top','steep_side','curved_top','steep_side'])
        self.assertNotEqual(labels[0][1],labels[1][1])

    def test_full_q_can_be_invalid_or_biased_and_denominators_survive(self):
        x,y=np.meshgrid((np.arange(4)+.5)/16,(np.arange(4)+.5)/16)
        points=np.column_stack((x.ravel(),y.ravel(),np.zeros(16)))
        def build(cloud):
            m=TerrainMap()
            for stamp,camera in enumerate(([-1,.1,1],[.1,-1,1],[1,.1,1]),1):
                m.add_frame(cloud-np.array(camera),stamp,camera)
            return m.elevation()[0]
        reference=build(points);biased=build(points+[0,0,.1]);mixed=build(np.vstack((points,points+[0,0,.5])))
        self.assertEqual(biased['quality'],1);self.assertEqual(mixed['quality'],1)
        self.assertTrue(biased['valid']);self.assertFalse(mixed['valid'])
        grid=dict(ix=0,iy=0,x=.125,y=.125,category='ground',z=0.)
        rows=[compare_cell(grid,biased,reference,{(0,'surface','ground')}),
              compare_cell(grid,mixed,reference,{(0,'surface','ground'),(1,'surface','level_top')}),
              compare_cell(grid,None,None,set())]
        self.assertEqual(rows[1]['category'],'mixed');self.assertIsNone(rows[1]['height_error_m'])
        summary=statistics(rows)
        self.assertEqual((summary['observed'],summary['invalid'],summary['paired'],summary['unobserved']),(2,1,1,1))
        self.assertAlmostEqual(summary['height_rmse_m'],.1,places=6)
        self.assertEqual(summary['valid_fraction'],.5)


if __name__=='__main__':unittest.main()
