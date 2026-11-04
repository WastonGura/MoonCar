#!/usr/bin/env python3
"""Occupancy from measured traversability; no ray clearing or world geometry.

TerrainNode supplies measured passable costs 0..95, lethal cost 100 and unknown
-1. Unknown never erases a historical observation. A later explicit known value
updates that cell, so a former hazard becomes free only after a passable source
classification. The separate terrain risk overlay retains continuous costs.
"""
import json
import math

import numpy as np


class MeasuredDepthMap:
    """Bounded world-cell accumulator for axis-aligned measured source grids."""
    def __init__(self,max_cells=200000,padding_m=1.):
        if type(max_cells) is not int or max_cells<1 or not math.isfinite(padding_m) or padding_m<0:
            raise ValueError('Invalid map capacity or padding')
        self.max_cells=max_cells;self.padding_m=float(padding_m)
        self.resolution=None;self.cells={};self.bounds=None;self.last_stamp=None;self.fault=None

    def update(self,*,stamp_ns,frame,resolution,width,height,origin,quaternion,data):
        if self.fault:raise ValueError(self.fault)
        if type(stamp_ns) is not int or stamp_ns<0:raise ValueError('Invalid source timestamp')
        if self.last_stamp is not None and stamp_ns<self.last_stamp:
            self.fault='source_clock_moved_backwards';raise ValueError(self.fault)
        if frame!='map':
            if self.resolution is not None:self.fault='source_frame_changed'
            raise ValueError('Measured source frame must be map')
        if (not math.isfinite(resolution) or resolution<=0 or type(width) is not int or type(height) is not int
                or width<1 or height<1 or width*height>self.max_cells):
            raise ValueError('Invalid or oversized source grid')
        if self.resolution is not None and abs(resolution-self.resolution)>1e-9:
            self.fault='source_resolution_changed';raise ValueError(self.fault)
        origin=np.asarray(origin,dtype=float);q=np.asarray(quaternion,dtype=float)
        if origin.shape!=(3,) or q.shape!=(4,) or not np.isfinite(origin).all() or not np.isfinite(q).all():
            raise ValueError('Invalid source origin')
        norm=float(np.linalg.norm(q))
        if norm<1e-9 or np.linalg.norm(q[:3]/norm)>1e-9 or abs(origin[2])>1e-9:
            if self.resolution is not None:self.fault='source_grid_orientation_changed'
            raise ValueError('Only gravity-aligned unrotated map grids are supported')
        scaled=origin[:2]/resolution;indices=np.rint(scaled)
        if np.max(abs(scaled-indices))>1e-5:
            if self.resolution is not None:self.fault='source_grid_alignment_changed'
            raise ValueError('Source origin must lie on the world resolution lattice')
        if len(data)!=width*height:raise ValueError('Source grid data length mismatch')
        values=np.asarray(data)
        if values.ndim!=1 or values.dtype.kind not in 'iu' or np.any(~((values==-1)|((values>=0)&(values<=95))|(values==100))):
            raise ValueError('Source costs must be unknown -1, passable 0..95 or hazard 100')
        if self.last_stamp is not None and stamp_ns==self.last_stamp:return False
        ix,iy=map(int,indices)
        known=np.flatnonzero(values>=0)
        updates={(ix+int(index%width),iy+int(index//width)):(100 if values[index]==100 else 0)
                 for index in known}
        if len(self.cells)+sum(key not in self.cells for key in updates)>self.max_cells:
            raise ValueError('Measured map known-cell capacity reached; start a new experiment')
        # Include the current rolling extent and previous observed cells. Padding
        # remains unknown, including the vehicle footprint unless actually seen.
        keys=self.cells.keys()|updates.keys()
        low_x=min([ix]+[key[0] for key in keys]);low_y=min([iy]+[key[1] for key in keys])
        high_x=max([ix+width]+[key[0]+1 for key in keys]);high_y=max([iy+height]+[key[1]+1 for key in keys])
        padding=math.ceil(self.padding_m/resolution)
        bounds=(low_x-padding,low_y-padding,high_x+padding,high_y+padding)
        if (bounds[2]-bounds[0])*(bounds[3]-bounds[1])>self.max_cells:
            raise ValueError('Measured map raster capacity reached; start a new experiment')
        self.cells.update(updates);self.resolution=float(resolution);self.bounds=bounds;self.last_stamp=stamp_ns
        return True

    def raster(self):
        if self.bounds is None:raise ValueError('No measured source grid')
        x0,y0,x1,y1=self.bounds;grid=np.full((y1-y0,x1-x0),-1,dtype=np.int8)
        for (x,y),cost in self.cells.items():grid[y-y0,x-x0]=cost
        return dict(resolution=self.resolution,width=x1-x0,height=y1-y0,
                    origin=[x0*self.resolution,y0*self.resolution,0.],data=grid.ravel().tolist())


def make_node():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String

    class DepthLocalizationMapNode(Node):
        def __init__(self):
            super().__init__('mooncar_depth_localization_map')
            self.declare_parameter('maximum_cells',200000);self.declare_parameter('padding_m',1.)
            self.core=MeasuredDepthMap(self.get_parameter('maximum_cells').value,self.get_parameter('padding_m').value)
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
            self.publisher=self.create_publisher(OccupancyGrid,'/map',qos)
            self.status=self.create_publisher(String,'/mooncar/depth_localization_map/status',1)
            self.create_subscription(OccupancyGrid,'/mooncar/terrain/traversability_map',self.on_grid,qos)

        def on_grid(self,msg):
            origin=msg.info.origin.position;q=msg.info.origin.orientation
            try:
                changed=self.core.update(stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
                    frame=msg.header.frame_id,resolution=float(msg.info.resolution),width=int(msg.info.width),height=int(msg.info.height),
                    origin=[origin.x,origin.y,origin.z],quaternion=[q.x,q.y,q.z,q.w],data=msg.data)
                if not changed:return
                data=self.core.raster();output=OccupancyGrid();output.header=msg.header
                output.info.map_load_time=msg.header.stamp;output.info.resolution=data['resolution']
                output.info.width=data['width'];output.info.height=data['height']
                output.info.origin.position.x=data['origin'][0];output.info.origin.position.y=data['origin'][1]
                output.info.origin.orientation.w=1.;output.data=data['data']
                self.publisher.publish(output)
                self.status.publish(String(data=json.dumps(dict(healthy=True,known_cells=len(self.core.cells),
                    width=data['width'],height=data['height'],source='measured depth traversability',
                    unknown_cells_filled=False,truth_inputs=False))))
            except (ValueError,TypeError,OverflowError) as error:
                self.status.publish(String(data=json.dumps(dict(healthy=False,reason=str(error),fault=self.core.fault,
                    known_cells=len(self.core.cells),truth_inputs=False))))
                self.get_logger().warning('Measured map update rejected: '+str(error))

    return DepthLocalizationMapNode()


def main():
    import rclpy
    rclpy.init();node=make_node()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
