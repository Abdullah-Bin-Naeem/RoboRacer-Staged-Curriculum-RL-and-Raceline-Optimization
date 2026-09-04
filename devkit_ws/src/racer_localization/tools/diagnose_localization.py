#!/usr/bin/env python3

"""Walk the localization chain link by link and report which one is dead.

    python3 diagnose_localization.py [seconds]

Run it while the car is MOVING (manual drive, teleop, or the RL policy). It does
not publish anything, so it is safe to run alongside race.launch.py.

The chain, in the order it has to work:

    sim moves  ->  /ips + /odom change            (is the car actually driving?)
      ->  encoders change                          (does the bridge see the wheels?)
      ->  odom->roboracer_1 moves                  (is dead_reckoning integrating?)
      ->  map->odom changes                        (is AMCL correcting?)
      ->  /amcl_pose publishes                     (is the filter updating at all?)

The first link that is FLAT is the bug. Everything downstream of it is a
symptom, not a cause.

It also measures scan-vs-TF timestamp skew, because AMCL drops any scan it
cannot transform: if the scan stamp lands AHEAD of the newest odom->base
transform, the lookup is an extrapolation into the future and the scan is
discarded -- which freezes the filter while every individual topic still looks
perfectly healthy.
"""

import math
import os
import sys

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Imu, JointState, LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32

NS = '/autodrive/roboracer_1'
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Track:
    """First/last value of a scalar or 2-vector, plus a message count.

    Also accumulates PATH length (sum of |step|), which is the number that
    distinguishes "drove 14 m in a circle" from "drove 14 m in a line" -- the
    net displacement alone cannot tell those apart, and the difference is
    exactly where a broken motion model hides.
    """

    def __init__(self):
        self.n = 0
        self.first = None
        self.last = None
        self.path = 0.0
        self.back = 0            # steps that went backwards (scalars only)

    def add(self, v):
        self.n += 1
        if self.first is None:
            self.first = v
        elif isinstance(v, tuple):
            self.path += math.hypot(v[0] - self.last[0], v[1] - self.last[1])
        else:
            d = v - self.last
            self.path += abs(d)
            if d < -1e-9:
                self.back += 1
        self.last = v

    def span(self):
        if self.first is None or self.n < 2:
            return 0.0
        if isinstance(self.first, tuple):
            return math.hypot(self.last[0] - self.first[0],
                              self.last[1] - self.first[1])
        return abs(self.last - self.first)


class Diag(Node):

    def __init__(self):
        super().__init__('diagnose_localization')
        self.ips = Track()
        self.truth = Track()
        self.enc_l = Track()
        self.enc_r = Track()
        self.imu = Track()
        self.scan = Track()
        self.amcl = Track()
        self.skew = []          # scan stamp - newest odom->base stamp

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Instantaneous encoder speed vs true speed. A CONSTANT ratio is a unit
        # or scale error; a ratio near 1 at steady speed with excursions during
        # acceleration is wheel slip. The path totals cannot tell those apart.
        self._enc_prev = {}
        self.enc_speed = None
        self._truth_prev = None
        self.ratio = []
        # map->base straight from TF, which is what pure_pursuit now steers on.
        # /amcl_pose can be clean while this is not: AMCL publishes the POSE
        # directly, but computes the TRANSFORM as pose o (odom->base)^-1 looked
        # up AT THE SCAN TIMESTAMP. If that lookup is mistimed, the transform is
        # corrupted while the topic stays fine.
        self.tf_err = []
        self.amcl_err = []
        self.amcl_last = None
        # Collisions and cross-track error answer the question the pose error
        # cannot: did the FILTER lose the car, or did the CONTROLLER drive it
        # into a wall? They need opposite fixes.
        self.collisions0 = None
        self.collisions = None
        self.xtrack = []
        self.px = self.py = None
        # Track-scoped: raceline/<track>/. RACER_TRACK picks the track, the same
        # variable racer_common.frames reads; DIAG_PATH_CSV overrides outright.
        _track = os.environ.get('RACER_TRACK', 'porto')
        for _csv in (os.environ.get('DIAG_PATH_CSV'),
                     os.path.expanduser(
                         f'~/Documents/roboracer/raceline/{_track}/centerline_speed.csv')):
            if _csv and os.path.exists(_csv):
                d = np.loadtxt(_csv, delimiter=',')
                self.px, self.py = d[:, 1], d[:, 2]
                break
        self.create_subscription(Int32, f'{NS}/collision_count',
                                 self._cb_col, QOS)

        self.create_subscription(Point, f'{NS}/ips',
                                 lambda m: self.ips.add((m.x, m.y)), QOS)
        self.create_subscription(Odometry, f'{NS}/odom', self._cb_truth, QOS)
        self.create_subscription(JointState, f'{NS}/left_encoder',
                                 lambda m: self._cb_enc('l', m), QOS)
        self.create_subscription(JointState, f'{NS}/right_encoder',
                                 lambda m: self._cb_enc('r', m), QOS)
        self.create_subscription(Imu, f'{NS}/imu',
                                 lambda m: self.imu.add(m.orientation.z), QOS)
        self.create_subscription(LaserScan, f'{NS}/lidar', self._cb_scan, QOS)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._cb_amcl, QOS)

    def _cb_enc(self, side, msg):
        if not msg.position:
            return
        (self.enc_l if side == 'l' else self.enc_r).add(msg.position[0])
        ang, t = float(msg.position[0]), stamp_s(msg.header)
        prev = self._enc_prev.get(side)
        self._enc_prev[side] = (ang, t)
        if prev is None:
            return
        dt = t - prev[1]
        if 1e-4 < dt <= 0.5:
            self.enc_speed = abs(ang - prev[0]) / dt * 0.0581

    def _cb_truth(self, msg):
        p = msg.pose.pose.position
        self.truth.add((p.x, p.y))
        t = stamp_s(msg.header)
        if self._truth_prev is not None:
            dt = t - self._truth_prev[2]
            if 1e-4 < dt <= 0.5:
                v = math.hypot(p.x - self._truth_prev[0],
                               p.y - self._truth_prev[1]) / dt
                # Only compare when genuinely moving; at a standstill the ratio
                # is noise over noise.
                if v > 0.3 and self.enc_speed is not None:
                    self.ratio.append((self.enc_speed / v, v))
        self._truth_prev = (p.x, p.y, t)

        # How far is the CAR from the path it is supposed to be following?
        if self.px is not None:
            self.xtrack.append(float(np.min(np.hypot(self.px - p.x,
                                                     self.py - p.y))))

        # Which pose source is actually closer to the truth right now?
        if self.amcl_last is not None:
            self.amcl_err.append(math.hypot(self.amcl_last[0] - p.x,
                                            self.amcl_last[1] - p.y))
        try:
            tr = self.tf_buffer.lookup_transform('map', 'roboracer_1',
                                                 rclpy.time.Time())
            self.tf_err.append(math.hypot(tr.transform.translation.x - p.x,
                                          tr.transform.translation.y - p.y))
        except Exception:                                   # noqa: BLE001
            pass

    def _cb_col(self, msg):
        if self.collisions0 is None:
            self.collisions0 = int(msg.data)
        self.collisions = int(msg.data)

    def _cb_amcl(self, msg):
        p = msg.pose.pose.position
        self.amcl.add((p.x, p.y))
        self.amcl_last = (p.x, p.y)

    def _cb_scan(self, msg):
        self.scan.add(stamp_s(msg.header))
        # How far ahead of the newest odom->base transform is this scan? AMCL
        # must look up that transform AT the scan's stamp; a positive skew is an
        # extrapolation into the future, which fails and drops the scan.
        try:
            tr = self.tf_buffer.lookup_transform('odom', 'roboracer_1', rclpy.time.Time())
            self.skew.append(stamp_s(msg.header) - stamp_s(tr.header))
        except Exception:                                   # noqa: BLE001
            pass

    def sample_tf(self, store):
        for name, (parent, child) in (('map->odom', ('map', 'odom')),
                                      ('odom->base', ('odom', 'roboracer_1'))):
            try:
                tr = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                t = tr.transform.translation
                store.setdefault(name, Track()).add((t.x, t.y))
            except Exception:                               # noqa: BLE001
                store.setdefault(name, Track())


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = Diag()
    tf_tracks = {}

    print(f'sampling for {secs:.0f} s -- DRIVE THE CAR NOW\n')
    end = node.get_clock().now().nanoseconds * 1e-9 + secs
    while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        node.sample_tf(tf_tracks)

    def row(label, tr, unit, moved_thresh):
        if tr.n == 0:
            return f'  {label:22s} NO DATA                                <-- dead link'
        hz = tr.n / secs
        span = tr.span()
        flag = '' if span > moved_thresh else '   <-- FLAT'
        return (f'  {label:22s} {hz:6.1f} Hz  net {span:8.3f}  path {tr.path:8.3f} '
                f'{unit}{flag}')

    print('link                     rate      net displacement / path length')
    print('-' * 78)
    print(row('/ips (truth)', node.ips, 'm', 0.05))
    print(row('/odom (truth)', node.truth, 'm', 0.05))
    print(row('/left_encoder', node.enc_l, 'rad', 0.5))
    print(row('/right_encoder', node.enc_r, 'rad', 0.5))
    print(row('/imu (quat z)', node.imu, '', 0.001))
    print(row('/lidar', node.scan, 's', 0.0))
    print(row('TF odom->base', tf_tracks.get('odom->base', Track()), 'm', 0.05))
    print(row('TF map->odom', tf_tracks.get('map->odom', Track()), 'm', 0.001))
    print(row('/amcl_pose', node.amcl, 'm', 0.05))

    # ---- the reverse test -------------------------------------------------
    # dead_reckoning assumes a NEGATIVE encoder delta means the car is backing
    # up. If the simulator reports magnitude only, reversing integrates FORWARD
    # and odometry runs away. Every previous validation of this path drove
    # forward only (config.py sets throttle_min 0.0 -- the RL policy cannot
    # reverse), so a sign bug here would never have been exercised until now.
    wheel_r = 0.0581
    enc_path = 0.5 * (node.enc_l.path + node.enc_r.path) * wheel_r
    odom = tf_tracks.get('odom->base', Track())
    print('\n--- motion model cross-check ---')
    print(f'  wheel path from encoders : {enc_path:8.3f} m')
    print(f'  dead reckoning path      : {odom.path:8.3f} m  (should match above)')
    print(f'  dead reckoning net       : {odom.span():8.3f} m')
    print(f'  TRUTH path               : {node.truth.path:8.3f} m')
    print(f'  TRUTH net                : {node.truth.span():8.3f} m')
    print(f'  encoder steps that went BACKWARDS: left {node.enc_l.back}, '
          f'right {node.enc_r.back}  (nonzero = encoders are signed)')
    if node.truth.path > 0.5:
        err = abs(odom.path - node.truth.path) / node.truth.path
        print(f'  path-length error        : {err * 100:6.1f} %')

    if len(node.ratio) > 20:
        rs = sorted(r for r, _ in node.ratio)
        n = len(rs)
        q = lambda f: rs[min(n - 1, int(f * n))]
        slow = [r for r, v in node.ratio if v < 1.0]
        print(f'\n  encoder speed / true speed   (n={n}, only while v > 0.3 m/s)')
        print(f'    p10 {q(0.10):5.2f}   median {q(0.50):5.2f}   '
              f'p90 {q(0.90):5.2f}   max {rs[-1]:5.2f}')
        if len(slow) >= 15:
            slow.sort()
            print(f'    while v < 1.0 m/s: median {slow[len(slow) // 2]:5.2f} '
                  f'(n={len(slow)})')
        med = q(0.50)
        if med < 1.15 and q(0.90) > 1.5:
            print('    -> near 1 at steady speed with a heavy tail: WHEEL SLIP.')
            print('       Drive with continuous throttle (RL policy) rather than')
            print('       keyboard on/off, or AMCL cannot be tuned meaningfully.')
        elif med > 1.5:
            print(f'    -> ratio is ~{med:.2f} across the WHOLE speed range, not just')
            print('       under acceleration. That is a SCALE error, not slip:')
            print(f'       the effective rolling radius is {0.0581 / med:.5f} m,')
            print(f'       not 0.0581 m.')

    # ---- did the controller leave the line, or did the filter lose it? -----
    if node.xtrack or node.collisions is not None:
        print('\n--- control vs localization ---')
        if node.collisions is not None and node.collisions0 is not None:
            hits = node.collisions - node.collisions0
            print(f'  collisions during window : {hits}'
                  + ('   <-- the car HIT something' if hits else ''))
        if node.xtrack:
            x = np.array(node.xtrack)
            print(f'  TRUE cross-track error   : mean {x.mean():.3f}  '
                  f'p95 {np.percentile(x, 95):.3f}  max {x.max():.3f} m')
            ap = np.mean(node.amcl_err) if node.amcl_err else float('nan')
            if x.mean() > 0.30 and ap < 0.20:
                print('  -> the car is off the LINE while the pose is GOOD:')
                print('     a controller problem (lookahead / speed), not localization.')
            elif ap > 0.30 and x.mean() > 0.30:
                print('  -> pose is bad AND the car is off the line: the filter lost it')
                print('     and the follower chased the bad estimate.')

    # ---- which pose source should the follower steer on? -------------------
    if node.tf_err or node.amcl_err:
        print('\n--- pose error vs ground truth (the number that matters) ---')
        for label, errs in (('/amcl_pose  (topic)', node.amcl_err),
                            ('TF map->base       ', node.tf_err)):
            if not errs:
                print(f'  {label}  no samples')
                continue
            a = np.array(errs)
            print(f'  {label}  n={len(a):4d}  mean {a.mean():7.3f}  '
                  f'p95 {np.percentile(a, 95):7.3f}  max {a.max():7.3f} m')
        if node.tf_err and node.amcl_err:
            tf_m, ap_m = np.mean(node.tf_err), np.mean(node.amcl_err)
            if tf_m > 2 * ap_m + 0.05:
                print('  -> TF is much WORSE than the topic: map->odom is corrupted.')
                print('     Revert pure_pursuit to use_tf_pose:=false until the')
                print('     scan/TF timestamp skew below is fixed.')
            elif ap_m > 2 * tf_m + 0.05:
                print('  -> TF is much BETTER: keep use_tf_pose:=true.')
            else:
                print('  -> comparable; choose TF for its higher rate and smoothness.')

    if node.skew:
        s = sorted(node.skew)
        n = len(s)
        ahead = sum(1 for v in s if v > 0) / n
        print('\nscan stamp minus newest odom->base stamp '
              f'(n={n}):')
        print(f'  median {s[n // 2] * 1e3:+7.1f} ms   '
              f'min {s[0] * 1e3:+7.1f} ms   max {s[-1] * 1e3:+7.1f} ms')
        print(f'  {ahead * 100:.0f}% of scans are stamped AHEAD of the newest transform')
        if ahead > 0.5:
            print('  ^ AMCL must extrapolate into the future for these and will')
            print('    DROP them. This freezes the filter even though every topic')
            print('    above looks healthy.')
    else:
        print('\nno scan/TF skew samples -- odom->base was never available')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
