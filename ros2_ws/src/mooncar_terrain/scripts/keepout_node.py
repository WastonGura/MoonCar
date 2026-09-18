#!/usr/bin/env python3
import copy
import math

from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
import yaml


class KeepoutNode(Node):
    def __init__(self):
        super().__init__('mooncar_keepout')
        self.declare_parameter('zones_file', '')
        with open(self.get_parameter('zones_file').value, encoding='utf-8') as stream:
            self.zones = yaml.safe_load(stream)['zones']
        for zone in self.zones:
            if not all(math.isfinite(zone[k]) for k in ('x', 'y', 'radius')) or zone['radius'] <= 0:
                raise ValueError('Keepout zone needs finite coordinates and a positive radius')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(OccupancyGrid, '/mooncar/terrain/keepout', qos)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)

    def on_map(self, msg):
        mask = OccupancyGrid(header=copy.deepcopy(msg.header), info=copy.deepcopy(msg.info))
        q = msg.info.origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        ox, oy, resolution = msg.info.origin.position.x, msg.info.origin.position.y, msg.info.resolution
        min_x = min_y = 0
        max_x, max_y = msg.info.width, msg.info.height
        for zone in self.zones:
            dx, dy = zone['x']-ox, zone['y']-oy
            local_x, local_y = c*dx+s*dy, -s*dx+c*dy
            min_x = min(min_x, math.floor((local_x-zone['radius'])/resolution))
            min_y = min(min_y, math.floor((local_y-zone['radius'])/resolution))
            max_x = max(max_x, math.ceil((local_x+zone['radius'])/resolution))
            max_y = max(max_y, math.ceil((local_y+zone['radius'])/resolution))
        mask.info.width, mask.info.height = max_x-min_x, max_y-min_y
        mask.info.origin.position.x = ox + resolution*(c*min_x-s*min_y)
        mask.info.origin.position.y = oy + resolution*(s*min_x+c*min_y)
        row, col = np.mgrid[0:mask.info.height, 0:mask.info.width]
        px, py = (col+0.5)*resolution, (row+0.5)*resolution
        x = mask.info.origin.position.x + px*c - py*s
        y = mask.info.origin.position.y + px*s + py*c
        # The prior adds hazards only; it must not clear unknown map cells.
        data = np.full(row.shape, -1, dtype=np.int8)
        for zone in self.zones:
            data[(x-zone['x'])**2+(y-zone['y'])**2 <= zone['radius']**2] = 100
        mask.data = data.ravel().tolist()
        self.publisher.publish(mask)
        self.get_logger().info(f'Published {len(self.zones)} operator-defined keepout zones')


def main():
    rclpy.init()
    node = KeepoutNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
