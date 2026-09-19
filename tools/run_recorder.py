#!/usr/bin/env python3
"""Record one development run: ground truth, the localizer's estimate, the
follower's own status and the simulator's lap/collision counters.

    python3 run_recorder.py OUT.csv [--seconds N]     # inside the racer container

One row per /odom message (the simulator tick). Columns:

    t                    seconds since the first row (receive time)
    gt_x gt_y gt_yaw     /odom pose (world == map)
    gt_v gt_yaw_rate     /odom twist (body frame)
    est_x est_y est_yaw  TF map -> roboracer_1, latest (what the follower steers on)
    dr_x dr_y dr_yaw     TF odom -> roboracer_1, latest (dead reckoning alone)
    pp_x pp_y pp_yaw     newest map->odom composed with newest odom->base: EXACTLY the
                         pose pure_pursuit steers on (compose_latest). est_* is a chain
                         lookup, whose map->odom is AMCL's post-dated (0.5 s) correction.
    ready                /localization_ready
    laps collisions      simulator counters
    last_lap             /last_lap_time
    pp_<field>           latest /pure_pursuit/status (pure_pursuit.STATUS_FIELDS)

DEVELOPMENT ONLY: reads restricted topics continuously.
"""
import argparse
import csv
import math
import time

import rclpy
import tf2_ros
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32

NS = '/autodrive/roboracer_1'
PP_FIELDS = ('v_est', 'v_enc', 'v_pose', 'v_target', 'u_cmd', 'throttle', 'steering',
             'ld', 'e_lat', 'e_head', 'kappa', 's', 'slip', 'a_imu', 'delay')
COLUMNS = (['t', 'gt_x', 'gt_y', 'gt_yaw', 'gt_v', 'gt_yaw_rate',
            'est_x', 'est_y', 'est_yaw', 'dr_x', 'dr_y', 'dr_yaw', 'pp_x', 'pp_y', 'pp_yaw', 'ready', 'laps', 'collisions', 'last_lap']
           + [f'pp_{f}' for f in PP_FIELDS])


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self, out):
        super().__init__('run_recorder')
        self.f = open(out, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(COLUMNS)
        self.t0 = None
        self.ready = 0
        self.laps = -1
        self.collisions = -1
        self.last_lap = float('nan')
        self.pp = [float('nan')] * len(PP_FIELDS)
        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        q = qos_profile_sensor_data
        self.create_subscription(Odometry, f'{NS}/odom', self._odom, q)
        self.create_subscription(Float32MultiArray, '/pure_pursuit/status', self._pp, 10)
        self.create_subscription(Int32, f'{NS}/lap_count', self._laps, q)
        self.create_subscription(Int32, f'{NS}/collision_count', self._coll, q)
        self.create_subscription(Float32, f'{NS}/last_lap_time', self._last, q)
        self.create_subscription(Bool, '/localization_ready', self._ready, QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    def _pp(self, m):
        self.pp = list(m.data)[:len(PP_FIELDS)] + [float('nan')] * max(0, len(PP_FIELDS) - len(m.data))

    def _laps(self, m):
        self.laps = m.data

    def _coll(self, m):
        self.collisions = m.data

    def _last(self, m):
        self.last_lap = m.data

    def _ready(self, m):
        self.ready = int(m.data)

    def _odom(self, m):
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        p = m.pose.pose
        est = [float('nan')] * 3
        try:
            tf = self.buf.lookup_transform('map', 'roboracer_1', Time())
            est = [tf.transform.translation.x, tf.transform.translation.y, yaw_of(tf.transform.rotation)]
        except tf2_ros.TransformException:
            pass
        dr = [float('nan')] * 3
        try:
            tf = self.buf.lookup_transform('odom', 'roboracer_1', Time())
            dr = [tf.transform.translation.x, tf.transform.translation.y, yaw_of(tf.transform.rotation)]
        except tf2_ros.TransformException:
            pass
        pp = [float('nan')] * 3
        try:
            mo = self.buf.lookup_transform('map', 'odom', Time())
            if dr[0] == dr[0]:
                my = yaw_of(mo.transform.rotation)
                c, sn = math.cos(my), math.sin(my)
                t = mo.transform.translation
                pp = [t.x + c * dr[0] - sn * dr[1], t.y + sn * dr[0] + c * dr[1], my + dr[2]]
        except tf2_ros.TransformException:
            pass
        self.w.writerow(
            [f'{now - self.t0:.3f}', f'{p.position.x:.4f}', f'{p.position.y:.4f}',
             f'{yaw_of(p.orientation):.4f}', f'{m.twist.twist.linear.x:.3f}',
             f'{m.twist.twist.angular.z:.4f}']
            + [f'{v:.4f}' for v in est + dr + pp]
            + [self.ready, self.laps, self.collisions, f'{self.last_lap:.3f}']
            + [f'{v:.4f}' for v in self.pp])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--seconds', type=float, default=0.0)
    a = ap.parse_args()
    rclpy.init()
    node = Recorder(a.out)
    end = time.monotonic() + a.seconds if a.seconds > 0 else None
    try:
        while rclpy.ok() and (end is None or time.monotonic() < end):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.f.close()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
