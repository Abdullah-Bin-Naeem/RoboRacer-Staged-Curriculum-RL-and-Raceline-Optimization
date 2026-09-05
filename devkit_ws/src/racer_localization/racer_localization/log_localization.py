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

pp_*                the follower's own view, from /pure_pursuit/status: fused
                    speed estimate, encoder speed, target, commanded wheel speed,
                    throttle, steering, lookahead, lateral and heading error to
                    the path, path kappa and s, and the slip it asked for.
                    `speed` is ground truth, so pp_v_est - speed validates the
                    estimator and pp_v_enc / speed is the encoder's lie, per
                    sample. raceline/analyze_run.py turns all of it into a
                    verdict. NaN when the follower is not running.

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

import numpy as np
import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time
from scipy import ndimage
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float32MultiArray

try:
    from racer_common import restricted
    from racer_common.frames import MAPS_DIR
except ImportError:                                     # run outside the ws
    restricted = None
    MAPS_DIR = os.path.join(os.path.expanduser('~'),
                            'Documents/roboracer/devkit_ws/src/'
                            'racer_mapping/maps')

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
    # ---- added for the "why is it rotated?" question -------------------
    'enc_dist',        # cumulative encoder arc length; vs dist_m = wheel slip
    'dr_err',          # dead reckoning alone, against truth
    'fit_true',        # scan-to-wall fit with the scan placed at the TRUE pose
    'fit_est',         # ... and at the ESTIMATED pose
    'fit_ratio',       # fit_est / fit_true -- THE decisive column, see below
]

# Follower status, in the order pure_pursuit.STATUS_FIELDS publishes it.
PP_FIELDS = ('v_est', 'v_enc', 'v_pose', 'v_target', 'u_cmd', 'throttle', 'steering',
             'ld', 'e_lat', 'e_head', 'kappa', 's', 'slip', 'a_imu', 'delay')
COLUMNS += ['pp_' + f for f in PP_FIELDS]

# The lidar sits this far forward of the rear axle. From the bridge's own TF
# broadcast, and identical to the static transform chassis.launch.py publishes.
LIDAR_X = 0.2733


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        return np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Logger(Node):

    def __init__(self, path=None, rate=None, seconds=None):
        super().__init__('log_localization')

        # Two ways in, because this is both a launched node and a hand-run
        # script. Command-line arguments win when given; otherwise the ROS
        # parameters do, which is how race.launch.py drives it.
        p = self.declare_parameter
        p('out', 'localization_log.csv')
        p('rate', 20.0)
        p('seconds', 0.0)
        # Grid to score the scan against. Default is the one slam_toolbox
        # localizes against, so the fit columns answer a question about the
        # map actually in use rather than a different one.
        # Grid for the scan-fit metric. Empty = the track's AMCL grid, resolved
        # through racer_common.frames (track:= or RACER_TRACK); a base path
        # without extension overrides.
        p('map', '')
        p('track', '')
        p('wheel_radius', 0.0581)       # MEASURED; see dead_reckoning.py
        g = lambda n: self.get_parameter(n).value   # noqa: E731
        path = path if path is not None else str(g('out'))
        rate = rate if rate is not None else float(g('rate'))
        seconds = seconds if seconds is not None else float(g('seconds'))
        self.wheel_r = float(g('wheel_radius'))

        self.path = path
        self.seconds = seconds

        # ---- the map, as a distance-to-nearest-wall field --------------
        # Scoring a scan is then one array lookup per beam: project each
        # endpoint into map cells and read off how far it landed from a wall.
        base = str(g('map'))
        if not base:
            from racer_common import frames
            base = frames.map_yaml(str(g('track')) or None)[:-len('.yaml')]
        meta = yaml.safe_load(open(base + '.yaml'))
        img = read_pgm(base + '.pgm')
        self.res = meta['resolution']
        self.ox, self.oy = meta['origin'][0], meta['origin'][1]
        self.mh, self.mw = img.shape
        self.dist_field = ndimage.distance_transform_edt(img != 0) * self.res

        self.ips = None              # (x, y)   ground truth position
        self.yaw = None              # rad      ground truth heading
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.dist = 0.0
        self._prev = None
        self.t0 = None
        self.scan = None             # latest LaserScan, for the fit columns
        self.enc_dist = 0.0          # cumulative encoder arc length
        self._enc = {}               # side -> last cumulative wheel angle
        self._ds = {}                # side -> metres awaiting accumulation
        self._odom0 = None           # truth at the first sample, to zero DR
        self.pp = None               # latest /pure_pursuit/status payload
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
        self.create_subscription(LaserScan, f'{NS}/lidar', self._cb_scan, QOS)
        self.create_subscription(JointState, f'{NS}/left_encoder',
                                 lambda m: self._cb_enc('l', m), QOS)
        self.create_subscription(JointState, f'{NS}/right_encoder',
                                 lambda m: self._cb_enc('r', m), QOS)
        # Not restricted: the follower's own state. Absent when it is not running.
        self.create_subscription(Float32MultiArray, '/pure_pursuit/status', self._cb_pp, QOS)

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
        # signal localization_bootstrap seeds from. The stamp is kept: /ips has
        # no header, and the bridge stamps every topic of one frame together.
        self.speed = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z
        self.truth_t = Time.from_msg(msg.header.stamp)

    def _cb_scan(self, msg):
        self.scan = msg

    def _cb_pp(self, msg):
        self.pp = list(msg.data)

    def _cb_enc(self, side, msg):
        """Cumulative wheel ANGLE in radians -- see dead_reckoning.py.

        Arc length is dangle * r regardless of timing, so nothing here needs
        dt. Compared against the true distance travelled, this is a direct
        readout of wheel slip: enc_dist / dist_m > 1 means the encoders are
        overreading, which is exactly the error the scan matcher has to reject.
        """
        if not msg.position:
            return
        ang = float(msg.position[0])
        prev = self._enc.get(side)
        self._enc[side] = ang
        if prev is None:
            return
        d = ang - prev
        if abs(d) > 300.0:          # counter reset, not travel
            return
        self._ds[side] = self._ds.get(side, 0.0) + d * self.wheel_r

    def _consume_enc(self):
        if not self._ds:
            return
        self.enc_dist += float(np.mean(list(self._ds.values())))
        for k in self._ds:
            self._ds[k] = 0.0

    def _fit(self, x, y, yaw):
        """Mean distance from this scan's beam endpoints to the nearest wall.

        Place the live scan at pose (x, y, yaw) and ask how well it agrees with
        the map. Small = the scan lies on the walls.

        Computing it at BOTH the true pose and the estimated pose is what makes
        the localization failure attributable, and it is the one question the
        other columns cannot answer:

            fit_est > fit_true   the matcher is failing to find a pose that
                                 exists. A search/tuning problem.
            fit_est < fit_true   the matcher found a genuinely BETTER fit than
                                 reality. The map disagrees with the world, so
                                 no amount of tuning helps -- the map or the
                                 lidar extrinsic is wrong.
        """
        s = self.scan
        if s is None:
            return float('nan')
        rng = np.asarray(s.ranges, dtype=float)
        ang = s.angle_min + np.arange(len(rng)) * s.angle_increment
        ok = np.isfinite(rng) & (rng > s.range_min) & (rng < s.range_max * 0.95)
        if ok.sum() < 50:
            return float('nan')
        rng, ang = rng[ok], ang[ok]
        lx = x + LIDAR_X * math.cos(yaw)
        ly = y + LIDAR_X * math.sin(yaw)
        ex = lx + rng * np.cos(yaw + ang)
        ey = ly + rng * np.sin(yaw + ang)
        # floor to the cell index FIRST, then flip the row. Writing this as
        # int(mh - 1 - v) floors after the subtraction, which is the same as
        # rounding v UP, and lands one row (5 cm) high on 99.8 % of points.
        col = np.floor((ex - self.ox) / self.res).astype(int)
        row = (self.mh - 1) - np.floor((ey - self.oy) / self.res).astype(int)
        inside = ((row >= 0) & (row < self.mh) & (col >= 0) & (col < self.mw))
        if inside.sum() < 20:
            return float('nan')
        return float(self.dist_field[row[inside], col[inside]].mean())

    def _lookup(self, parent, child, at=None):
        """Transform at time `at`, latest when None or when `at` is not covered."""
        try:
            try:
                t = self.tf_buffer.lookup_transform(parent, child, at if at is not None else Time())
            except Exception:                            # noqa: BLE001
                if at is None:
                    raise
                t = self.tf_buffer.lookup_transform(parent, child, Time())
        except Exception:                                # noqa: BLE001
            return None
        v = t.transform.translation
        return (v.x, v.y, yaw_from_quat(t.transform.rotation))

    def _estimate(self):
        """map -> roboracer_1 AT THE TRUTH'S STAMP, so the two are compared at
        the same instant. The odometry leg is looked up at that stamp; the
        map -> odom correction is taken latest, because AMCL post-dates it by
        0.5 s and an exact-time lookup would return a correction half a second
        old. Composed by hand for that reason. Before this the estimate was
        the latest transform against a truth one bridge frame old, which read
        as a 35-40 ms x speed lead on every run; it was partly real (see
        dead_reckoning extrapolate_pos) and partly this comparison."""
        m2o = self._lookup('map', 'odom')
        o2b = self._lookup('odom', 'roboracer_1', getattr(self, 'truth_t', None))
        if m2o is None or o2b is None:
            return self._lookup('map', 'roboracer_1'), m2o, o2b
        c, s = math.cos(m2o[2]), math.sin(m2o[2])
        est = (m2o[0] + c * o2b[0] - s * o2b[1],
               m2o[1] + s * o2b[0] + c * o2b[1],
               m2o[2] + o2b[2])
        return est, m2o, o2b

    # ---- sampling --------------------------------------------------------

    def _tick(self):
        if self.ips is None or self.yaw is None:
            return
        est, m2o, o2b = self._estimate()
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

        m2o = m2o or (float('nan'),) * 3
        o2b = o2b or (float('nan'),) * 3

        self._consume_enc()

        # Dead reckoning on its own. The odom frame's origin is wherever
        # dead_reckoning started, so anchor it to the first truth sample and
        # measure how far the two have diverged since.
        if self._odom0 is None and not math.isnan(o2b[0]):
            self._odom0 = (tx - o2b[0], ty - o2b[1])
        if self._odom0 is None or math.isnan(o2b[0]):
            dr_err = float('nan')
        else:
            dr_err = math.hypot(self._odom0[0] + o2b[0] - tx,
                                self._odom0[1] + o2b[1] - ty)

        fit_t = self._fit(tx, ty, self.yaw)
        fit_e = self._fit(est[0], est[1], est[2])
        ratio = (fit_e / fit_t) if (fit_t and fit_t == fit_t and fit_t > 1e-6
                                    and fit_e == fit_e) else float('nan')

        self.csv.writerow([
            f'{t:.3f}', f'{self.dist:.3f}',
            f'{tx:.4f}', f'{ty:.4f}', f'{math.degrees(self.yaw):.3f}',
            f'{est[0]:.4f}', f'{est[1]:.4f}', f'{math.degrees(est[2]):.3f}',
            f'{along:.4f}', f'{cross:.4f}', f'{dist_err:.4f}', f'{dyaw:.3f}',
            f'{m2o[0]:.4f}', f'{m2o[1]:.4f}', f'{math.degrees(m2o[2]):.3f}',
            f'{o2b[0]:.4f}', f'{o2b[1]:.4f}', f'{math.degrees(o2b[2]):.3f}',
            f'{self.speed:.3f}', f'{self.yaw_rate:.4f}',
            f'{self.enc_dist:.3f}', f'{dr_err:.4f}',
            f'{fit_t:.4f}', f'{fit_e:.4f}', f'{ratio:.4f}',
        ] + [f'{v:.4f}' for v in (self.pp if self.pp is not None and len(self.pp) == len(PP_FIELDS)
                                  else [float('nan')] * len(PP_FIELDS))])
        self.rows += 1
        if self.rows % 100 == 0:
            self.fh.flush()         # so the file is useful before Ctrl-C
            slip = (self.enc_dist / self.dist) if self.dist > 1.0 else float('nan')
            self.get_logger().info(
                f'{t:6.1f} s  {self.dist:6.1f} m  err {dist_err:5.3f} m '
                f'(along {along:+.3f} cross {cross:+.3f}) '
                f'yaw {dyaw:+6.2f} deg  m->o yaw {math.degrees(m2o[2]):+6.2f} deg | '
                f'fit true {fit_t:.3f} est {fit_e:.3f} (x{ratio:.2f})  '
                f'enc/true {slip:.3f}  dr_err {dr_err:.2f} m')

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
    # Launched by ros2 launch/run, argv carries --ros-args and its friends, and
    # argparse would try to read `__node:=log_localization` as the output path.
    # In that case take everything from ROS parameters instead.
    if '--ros-args' in sys.argv:
        cli = (None, None, None)
    else:
        ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
        ap.add_argument('out', nargs='?', default='localization_log.csv')
        ap.add_argument('--rate', type=float, default=20.0,
                        help='samples per second (default 20)')
        ap.add_argument('--seconds', type=float, default=0.0,
                        help='stop after this long; 0 = until Ctrl-C')
        a = ap.parse_args()
        cli = (a.out, a.rate, a.seconds)

    rclpy.init()
    node = Logger(*cli)
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
