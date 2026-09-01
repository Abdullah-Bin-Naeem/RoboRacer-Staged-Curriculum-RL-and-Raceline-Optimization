#!/usr/bin/env python3

"""Log ground truth against the localization estimate, one row per sample.

    python3 log_localization.py [out.csv] [--rate HZ] [--seconds N]

`localization_error` prints hypot(dx, dy) every 5 s. That is enough to know
something is wrong and not enough to know what: it cannot show WHICH WAY the
estimate is wrong, whether the error is growing or oscillating, or what the car
was doing when it grew. This writes every sample to a CSV so the drift can
actually be plotted and correlated.

WHAT EACH GROUP OF COLUMNS IS FOR
---------------------------------
true_*              /ips position + /imu heading. The reference.
est_*               TF map -> roboracer_1. The pose the follower drives on when
                    use_tf_pose is true, i.e. the one that matters.
err_along/cross     the error rotated into the CAR'S OWN frame, signed:
                      along > 0  estimate is AHEAD of the car
                      cross > 0  estimate is to the car's LEFT
                    A magnitude cannot distinguish "half a metre down the track"
                    from "half a metre into the wall"; this can.
err_yaw_deg         signed heading error, + = estimate rotated CCW of truth.

m2o_yaw_deg         THE INVARIANT, and the first column to look at.
                    dead_reckoning takes heading straight from /imu's absolute
                    quaternion, which in the bridge is the SAME variable
                    published as /odom's ground-truth orientation. So `odom` is
                    yaw-locked to `world`. The map was built against that same
                    ground truth (measured: track_sm aligns to the world-frame
                    map with 0.024 m RMS and 0.19 deg residual rotation). So the
                    CORRECT map->odom is a PURE TRANSLATION and this column must
                    stay at zero. Every degree in it is the scan matcher
                    rotating the estimate away from a heading that was exact.

o2b_*               dead reckoning alone. Encoder overread shows up here as
                    position error that grows with distance and never rotates.

speed, yaw_rate     what the car was doing. yaw_rate is the one to correlate
                    against: dead_reckoning stamps odom->base at now+20 ms while
                    carrying an IMU heading that is up to one 18 Hz tick (55 ms)
                    stale, and slam_toolbox looks that transform up AT THE SCAN
                    STAMP to build map->odom. So a heading error proportional to
                    yaw_rate gets baked in on every update -- and on a loop
                    driven in one direction those errors share a sign instead of
                    cancelling. If d(m2o_yaw)/dt tracks yaw_rate, that is the
                    mechanism. If m2o_yaw wanders with no relation to yaw_rate,
                    it is not.

Read it back with anything; the quick look is:

    python3 -c "
    import csv,sys
    r=list(csv.DictReader(open('out.csv')))
    print(r[0]['t'], r[-1]['t'], r[-1]['err_dist'], r[-1]['m2o_yaw_deg'])"

DEVELOPMENT ONLY -- /ips and /odom are restricted, and this reads them
continuously, which is the pattern that is never race-legal. See
racer_common/restricted.py.
"""

import argparse
import csv
import math
import os
import sys

import rclpy
import tf2_ros
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import Imu

try:
    from racer_common import restricted
except ImportError:                                     # tools run outside the ws
    restricted = None

NS = '/autodrive/roboracer_1'
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)

COLUMNS = [
    't', 'dist_m',
    'true_x', 'true_y', 'true_yaw_deg',
    'est_x', 'est_y', 'est_yaw_deg',
    'err_along', 'err_cross', 'err_dist', 'err_yaw_deg',
    'm2o_x', 'm2o_y', 'm2o_yaw_deg',
    'o2b_x', 'o2b_y', 'o2b_yaw_deg',
    'speed', 'yaw_rate',
]


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Logger(Node):

    def __init__(self, path, rate, seconds):
        super().__init__('log_localization')
        self.path = path
        self.seconds = seconds

        self.ips = None              # (x, y)   ground truth position
        self.yaw = None              # rad      ground truth heading
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.dist = 0.0
        self._prev = None
        self.t0 = None
        self.rows = 0
        self.peak = 0.0
        self.peak_yaw = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # /ips is position only; heading has to come from /imu. /odom carries
        # both, and its twist is the cheapest source of speed and yaw rate.
        self.create_subscription(Point, f'{NS}/ips', self._cb_ips, QOS)
        self.create_subscription(Imu, f'{NS}/imu', self._cb_imu, QOS)
        self.create_subscription(Odometry, f'{NS}/odom', self._cb_odom, QOS)

        self.fh = open(path, 'w', newline='')
        self.csv = csv.writer(self.fh)
        self.csv.writerow(COLUMNS)

        if restricted is not None:
            restricted.warn(self, f'{NS}/ips', 'continuous logging against the estimate')
            restricted.warn(self, f'{NS}/odom', 'continuous logging against the estimate')
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'logging to {os.path.abspath(path)} at {rate:g} Hz'
            + (f' for {seconds:g} s' if seconds else ' until Ctrl-C'))

    # ---- inputs ----------------------------------------------------------

    def _cb_ips(self, msg):
        self.ips = (msg.x, msg.y)

    def _cb_imu(self, msg):
        self.yaw = yaw_from_quat(msg.orientation)

    def _cb_odom(self, msg):
        # Twist only. Position comes from /ips so the reference is the same
        # signal localization_bootstrap seeds from.
        self.speed = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z

    def _lookup(self, parent, child):
        try:
            t = self.tf_buffer.lookup_transform(parent, child, Time())
        except Exception:                                # noqa: BLE001
            return None
        v = t.transform.translation
        return (v.x, v.y, yaw_from_quat(t.transform.rotation))

    # ---- sampling --------------------------------------------------------

    def _tick(self):
        if self.ips is None or self.yaw is None:
            return
        est = self._lookup('map', 'roboracer_1')
        if est is None:
            if self.rows == 0:
                self.get_logger().info('waiting for TF map -> roboracer_1 ...',
                                       throttle_duration_sec=5.0)
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0

        tx, ty = self.ips
        if self._prev is not None:
            self.dist += math.hypot(tx - self._prev[0], ty - self._prev[1])
        self._prev = (tx, ty)

        # Error rotated into the car's own frame, so the sign means something.
        ex, ey = est[0] - tx, est[1] - ty
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        along = ex * c + ey * s
        cross = -ex * s + ey * c
        dyaw = math.degrees(wrap(est[2] - self.yaw))
        dist_err = math.hypot(ex, ey)
        self.peak = max(self.peak, dist_err)
        self.peak_yaw = max(self.peak_yaw, abs(dyaw))

        m2o = self._lookup('map', 'odom') or (float('nan'),) * 3
        o2b = self._lookup('odom', 'roboracer_1') or (float('nan'),) * 3

        self.csv.writerow([
            f'{t:.3f}', f'{self.dist:.3f}',
            f'{tx:.4f}', f'{ty:.4f}', f'{math.degrees(self.yaw):.3f}',
            f'{est[0]:.4f}', f'{est[1]:.4f}', f'{math.degrees(est[2]):.3f}',
            f'{along:.4f}', f'{cross:.4f}', f'{dist_err:.4f}', f'{dyaw:.3f}',
            f'{m2o[0]:.4f}', f'{m2o[1]:.4f}', f'{math.degrees(m2o[2]):.3f}',
            f'{o2b[0]:.4f}', f'{o2b[1]:.4f}', f'{math.degrees(o2b[2]):.3f}',
            f'{self.speed:.3f}', f'{self.yaw_rate:.4f}',
        ])
        self.rows += 1
        if self.rows % 100 == 0:
            self.fh.flush()         # so the file is useful before Ctrl-C
            self.get_logger().info(
                f'{t:6.1f} s  {self.dist:6.1f} m  err {dist_err:5.3f} m '
                f'(along {along:+.3f} cross {cross:+.3f}) '
                f'yaw {dyaw:+6.2f} deg  map->odom yaw {math.degrees(m2o[2]):+6.2f} deg')

        if self.seconds and t >= self.seconds:
            raise KeyboardInterrupt

    # ---- teardown --------------------------------------------------------

    def close(self):
        try:
            self.fh.flush()
            self.fh.close()
        except Exception:                                # noqa: BLE001
            pass
        print(f'\n  wrote {self.rows} rows to {os.path.abspath(self.path)}')
        if self.rows:
            print(f'  peak position error {self.peak:.3f} m, '
                  f'peak heading error {self.peak_yaw:.2f} deg')
            print('  first column to read is m2o_yaw_deg: it must be ~0. If it '
                  'is not,\n  the scan matcher is rotating a heading that was '
                  'already exact.')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('out', nargs='?', default='localization_log.csv')
    ap.add_argument('--rate', type=float, default=20.0,
                    help='samples per second (default 20)')
    ap.add_argument('--seconds', type=float, default=0.0,
                    help='stop after this long; 0 = until Ctrl-C')
    args = ap.parse_args()

    rclpy.init()
    node = Logger(args.out, args.rate, args.seconds)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
