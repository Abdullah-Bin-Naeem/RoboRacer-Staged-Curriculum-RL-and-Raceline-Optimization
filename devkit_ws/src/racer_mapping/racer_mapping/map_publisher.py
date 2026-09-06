#!/usr/bin/env python3

"""Publish a saved occupancy grid as a latched /map, for visualisation.

nav2's map_server is a lifecycle node and needs nav2_lifecycle_manager to reach
the active state. This does the one thing needed here instead: read the
yaml+pgm pair and publish it once, latched.

The grid is published in `frame_id` (default `world`) rather than `map`. The
mapping run used slam_toolbox with scan matching disabled, so the map->odom
correction was identity and the map frame coincides with the devkit's `world`.
Publishing directly in `world` means no static transform is needed to line the
map up with the car.
"""

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

LATCHED = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def read_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5', 'expected a binary PGM'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        return np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)


class MapPublisher(Node):

    def __init__(self):
        super().__init__('map_publisher')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('frame_id', 'world')

        path = self.get_parameter('map_yaml').value
        if not path:
            raise RuntimeError('map_yaml parameter is required')
        frame = self.get_parameter('frame_id').value

        meta = yaml.safe_load(open(path))
        import os
        img = read_pgm(os.path.join(os.path.dirname(path), meta['image']))
        h, w = img.shape

        # OccupancyGrid: -1 unknown, 0 free, 100 occupied, row 0 at MIN y.
        # PGM row 0 is at MAX y, so flip vertically.
        grid = np.full((h, w), -1, dtype=np.int8)
        grid[img == 0] = 100
        grid[img >= 254] = 0
        grid = np.flipud(grid)

        msg = OccupancyGrid()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(meta['resolution'])
        msg.info.width, msg.info.height = w, h
        msg.info.origin.position.x = float(meta['origin'][0])
        msg.info.origin.position.y = float(meta['origin'][1])
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()

        self.pub = self.create_publisher(OccupancyGrid, '/map', LATCHED)
        self.pub.publish(msg)
        self.get_logger().info(
            f'published {w}x{h} @ {meta["resolution"]} m in frame "{frame}" '
            f'({int((grid==100).sum())} occupied, {int((grid==0).sum())} free)')


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    try:
        rclpy.spin(node)      # stay alive so the latched message is available
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
