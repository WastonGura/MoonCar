"""Measured-cell semantics and bounds; all fixtures are synthetic grid messages."""
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from depth_localization_map import MeasuredDepthMap


def update(core,values,*,stamp=1,origin=(0.,0.,0.),width=None,height=1,**kwargs):
    return core.update(stamp_ns=stamp,frame=kwargs.pop('frame','map'),resolution=kwargs.pop('resolution',.2),
        width=width or len(values)//height,height=height,origin=origin,
        quaternion=kwargs.pop('quaternion',[0.,0.,0.,1.]),data=values,**kwargs)


class MapSemanticsTests(unittest.TestCase):
    def test_passable_hazard_unknown_are_not_raycast_or_hole_filled(self):
        core=MeasuredDepthMap(padding_m=0.)
        update(core,[-1,0,1,50,95,100])
        self.assertEqual(core.raster()['data'],[-1,0,0,0,0,100])
        self.assertEqual(len(core.cells),5)

    def test_rolling_unknown_does_not_erase_observations_but_explicit_new_class_does(self):
        core=MeasuredDepthMap(padding_m=0.)
        update(core,[100,0])
        update(core,[-1,100],stamp=2,origin=(.2,0.,0.))
        self.assertEqual(core.raster()['data'],[100,0,100])
        update(core,[0],stamp=3)
        self.assertEqual(core.cells[(0,0)],0)
        self.assertEqual(core.cells[(2,0)],100)

    def test_negative_origins_align_to_world_lattice_and_padding_stays_unknown(self):
        core=MeasuredDepthMap(padding_m=.2)
        update(core,[100],origin=(-.4,-.2,0.))
        data=core.raster()
        self.assertEqual(data['origin'],[-.6000000000000001,-.4,0.])
        self.assertEqual(data['width'],3);self.assertEqual(data['height'],3)
        self.assertEqual(data['data'].count(100),1);self.assertEqual(data['data'].count(-1),8)

    def test_frame_resolution_orientation_and_clock_changes_fail_closed(self):
        for change in ({'frame':'odom'},{'resolution':.1},{'quaternion':[0.,0.,math.sin(.1),math.cos(.1)]},
                       {'origin':(.01,0.,0.)},{'stamp':0}):
            with self.subTest(change=change):
                core=MeasuredDepthMap(padding_m=0.);update(core,[100])
                with self.assertRaises(ValueError):update(core,[0],stamp=change.pop('stamp',2),**change)
                self.assertEqual(core.cells,{(0,0):100})
                self.assertIsNotNone(core.fault)
                with self.assertRaises(ValueError):update(core,[0],stamp=3)

    def test_capacity_rejection_is_atomic_and_does_not_refresh_time(self):
        core=MeasuredDepthMap(max_cells=4,padding_m=0.)
        update(core,[100,0])
        with self.assertRaises(ValueError):update(core,[0,0,0],stamp=2,origin=(.4,0.,0.))
        self.assertEqual(len(core.cells),2);self.assertEqual(core.last_stamp,1)
        # Sparse known keys cannot create an unbounded dense output allocation.
        with self.assertRaises(ValueError):update(core,[0],stamp=2,origin=(100.,0.,0.))
        self.assertEqual(len(core.cells),2)

    def test_illegal_values_and_duplicate_stamps_do_not_update_cells(self):
        core=MeasuredDepthMap(padding_m=0.);update(core,[100])
        for values in ([96],[99],[-2],[101],[0.]):
            with self.subTest(values=values),self.assertRaises(ValueError):update(core,values,stamp=2)
        self.assertFalse(update(core,[0],stamp=1));self.assertEqual(core.cells[(0,0)],100)


class RosMapAdapterTests(unittest.TestCase):
    def test_callback_uses_source_timestamp_and_keeps_unknown_padding(self):
        import rclpy
        from nav_msgs.msg import OccupancyGrid
        from depth_localization_map import make_node
        rclpy.init(domain_id=95);node=make_node()
        try:
            node.publisher=Mock();node.status=Mock()
            msg=OccupancyGrid();msg.header.frame_id='map';msg.header.stamp.sec=7
            msg.info.width=2;msg.info.height=1;msg.info.resolution=.2;msg.info.origin.orientation.w=1.
            msg.data=[95,100]
            node.on_grid(msg)
            output=node.publisher.publish.call_args.args[0]
            self.assertEqual(output.header.stamp.sec,7);self.assertEqual(output.header.frame_id,'map')
            self.assertEqual(output.data.count(0),1);self.assertEqual(output.data.count(100),1)
            self.assertGreater(output.data.count(-1),0)
        finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main(verbosity=2)
