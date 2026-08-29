#!/usr/bin/env python3

"""Does the LiDAR scan match the map when placed at the TRUE pose?

    python3 check_scan_alignment.py [n_scans]

Independent of AMCL entirely. It takes the ground-truth pose from /odom, projects
every beam endpoint into map coordinates, and measures how far those endpoints
land from the nearest wall. If the sensor, the extrinsic and the map all agree,
the endpoints sit ON the walls and the error is a few centimetres.

It scores four variants, because the two classic ways this breaks are silent:

    as-is            what the code currently assumes
    scan mirrored    beam i taken as -angle instead of +angle. AutoDRIVE runs in
                     Unity, which is LEFT-handed; if the ray-cast order is not
                     converted, the range array is reversed while the header
                     still advertises angle_min = -135 deg. Everything looks
                     healthy and the scan is a mirror image.
    yaw negated      the same handedness problem in the IMU/pose quaternion
    both             mirrored scan AND negated yaw, which can cancel out

The variant with the SMALLEST endpoint-to-wall error is the true geometry. If
'as-is' does not win, that is the bug -- and note that a wrong variant can still
build a self-consistent map, so map-vs-centerline checks cannot catch it.

Drive the car (or leave it parked somewhere with asymmetric walls) and run this.
A spot where the left and right walls are at different distances is essential:
in a symmetric corridor every variant scores the same.
"""

import math
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from scipy import ndimage
from sensor_msgs.msg import LaserScan
import yaml

NS = '/autodrive/roboracer_1'
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)
MAP = ('/home/theflash/Documents/roboracer/devkit_ws/src/racer_mapping/'
       'maps/track_clean')
LIDAR_X = 0.2733          # lidar offset forward of the rear axle, from the bridge


def read_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        return np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Check(Node):

    def __init__(self, want):
        super().__init__('check_scan_alignment')
        meta = yaml.safe_load(open(MAP + '.yaml'))
        img = read_pgm(MAP + '.pgm')
        self.res = meta['resolution']
        self.ox, self.oy = meta['origin'][0], meta['origin'][1]
        self.h, self.w = img.shape
        # metres from each cell to the nearest occupied cell
        self.dist = ndimage.distance_transform_edt(img != 0) * self.res

        self.pose = None
        self.want = want
        self.scores = {k: [] for k in ('as-is', 'scan mirrored',
                                       'yaw negated', 'both')}
        self.n = 0
        self.create_subscription(Odometry, f'{NS}/odom', self._cb_odom, QOS)
        self.create_subscription(LaserScan, f'{NS}/lidar', self._cb_scan, QOS)

    def _cb_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))

    def _score(self, x, y, yaw, ang, rng):
        """Mean distance from each beam endpoint to the nearest wall."""
        lx = x + LIDAR_X * math.cos(yaw)
        ly = y + LIDAR_X * math.sin(yaw)
        ex = lx + rng * np.cos(yaw + ang)
        ey = ly + rng * np.sin(yaw + ang)
        col = ((ex - self.ox) / self.res).astype(int)
        row = (self.h - 1 - (ey - self.oy) / self.res).astype(int)
        ok = (row >= 0) & (row < self.h) & (col >= 0) & (col < self.w)
        if ok.sum() < 20:
            return None
        return float(self.dist[row[ok], col[ok]].mean())

    def _cb_scan(self, msg):
        if self.pose is None or self.n >= self.want:
            return
        rng = np.asarray(msg.ranges, dtype=float)
        ang = msg.angle_min + np.arange(len(rng)) * msg.angle_increment
        good = np.isfinite(rng) & (rng > msg.range_min) & (rng < msg.range_max * 0.95)
        if good.sum() < 50:
            return
        rng, ang = rng[good], ang[good]
        x, y, yaw = self.pose

        for name, a, yw in (('as-is', ang, yaw),
                            ('scan mirrored', -ang, yaw),
                            ('yaw negated', ang, -yaw),
                            ('both', -ang, -yaw)):
            s = self._score(x, y, yw, a, rng)
            if s is not None:
                self.scores[name].append(s)
        self.n += 1


def main():
    want = int(sys.argv[1]) if len(sys.argv) > 1 else 40
    rclpy.init()
    node = Check(want)
    print(f'collecting {want} scans -- park or drive somewhere the LEFT and RIGHT '
          'walls differ\n')
    while rclpy.ok() and node.n < want:
        rclpy.spin_once(node, timeout_sec=0.5)

    print('  mean distance from beam endpoints to the nearest wall')
    print('  (the true geometry is the one closest to zero)\n')
    rows = []
    for name, vals in node.scores.items():
        if vals:
            rows.append((float(np.mean(vals)), name, len(vals)))
    rows.sort()
    for i, (v, name, n) in enumerate(rows):
        mark = '   <-- BEST' if i == 0 else ''
        print(f'    {name:16s} {v:6.3f} m   (n={n}){mark}')

    if rows:
        best_v, best, _ = rows[0]
        print()
        if best == 'as-is':
            print('  Scan, extrinsic and map agree. Geometry is NOT the problem.')
        else:
            print(f'  *** {best.upper()} fits the map {rows[1][0] / max(best_v,1e-6):.1f}x '
                  'better than what the code assumes. ***')
            if 'mirrored' in best:
                print('  The range array is reversed relative to the header: Unity is')
                print('  left-handed and the ray-cast order was never converted.')
                print('  Fix at the source, in rl_racer/rl_racer/obs.py and anywhere')
                print('  else beam index is turned into an angle -- NOT by editing')
                print('  the map, which is self-consistent with the mirrored scan.')
        if best_v > 0.15:
            print(f'\n  NOTE: even the best variant is {best_v:.3f} m off. Either the')
            print('  pose was moving fast (scan/pose timestamp skew) or the lidar')
            print(f'  extrinsic ({LIDAR_X} m forward) is wrong.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
