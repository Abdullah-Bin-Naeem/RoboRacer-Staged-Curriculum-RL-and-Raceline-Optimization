#!/usr/bin/env python3
"""Localization drift: ground truth against the localizer's estimate.

    tools/localization_drift.sh [SECONDS] [NAME]      # record in the container, then plot

or the two halves by hand:

    python3 localization_drift.py record OUT.csv [--seconds N]   # inside the racer container
    python3 localization_drift.py plot OUT.csv                   # anywhere with numpy + matplotlib

RECORD (rclpy + tf2 only, runs in the racer image)
    actual     /autodrive/roboracer_1/odom, frame 'world'. The map was built in
               the simulator's own coordinates, so world == map.
    estimate   TF map -> roboracer_1: AMCL's map->odom composed with dead
               reckoning's odom->base, the pose the follower steers on
               (use_tf_pose:=true).
    Each ground-truth sample is held DELAY_S and the estimate is then looked up
    AT ITS STAMP, not "latest": AMCL publishes map->odom only after it has
    processed a scan, so a lookup on arrival would compare against a pose from
    an earlier tick and report the car's motion as error (~0.2 m at 8 m/s and
    25 ms). Samples the buffer cannot interpolate (before localization is up)
    are skipped and counted.

PLOT
    position_vs_time.png, position_vs_distance.png
        x and y, actual and estimated overlaid, and the position error.
    orientation_vs_time.png, orientation_vs_distance.png
        yaw, actual and estimated overlaid, and the heading error.
    Shaded spans are where /localization_ready was false (seeding, recovery
    after a wall reset). Distance is the ground-truth arc length; a jump over
    JUMP_M in one sample is a simulator reset and adds no distance.
    Prints the two summary numbers, position error and heading error, over the
    ready samples.

Reads /odom continuously: a development measurement, never part of a race run.
"""
import argparse
import csv
import math
import sys

NS = '/autodrive/roboracer_1'
ODOM_TOPIC = f'{NS}/odom'
READY_TOPIC = '/localization_ready'
MAP_FRAME = 'map'
BASE_FRAME = 'roboracer_1'
DELAY_S = 0.5
JUMP_M = 1.0
COLUMNS = ['t', 'gt_x', 'gt_y', 'gt_yaw', 'est_x', 'est_y', 'est_yaw', 'ready']


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---- record -----------------------------------------------------------------

def record(args):
    import collections

    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    import tf2_ros

    class Recorder(Node):
        def __init__(self):
            super().__init__('localization_drift')
            # roboracer_stack on the competition branches, racer_common on multi-track.
            try:
                from roboracer_stack.common import restricted
            except ImportError:
                try:
                    from racer_common import restricted
                except ImportError:
                    restricted = None
            if restricted is not None:
                restricted.warn(self, ODOM_TOPIC, 'localization_drift ground truth')
            else:
                self.get_logger().error(f'reading {ODOM_TOPIC}: not race-legal')
            self.buf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
            self.tfl = tf2_ros.TransformListener(self.buf, self)
            # Reliability and durability must match the bridge; depth is ours.
            self.create_subscription(Odometry, ODOM_TOPIC, self._odom, QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST, depth=50))
            self.create_subscription(Bool, READY_TOPIC, self._ready, QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST, depth=1))
            self.ready = False
            self.pending = collections.deque()
            self.f = open(args.out, 'w', newline='')
            self.w = csv.writer(self.f)
            self.w.writerow(COLUMNS)
            self.n_ok = 0
            self.n_skip = 0
            self.t0 = None
            self.create_timer(0.05, self._drain)
            self.create_timer(5.0, self._report)
            self.get_logger().info(f'recording {ODOM_TOPIC} vs TF {MAP_FRAME}->{BASE_FRAME} into {args.out}')

        def _ready(self, msg):
            self.ready = bool(msg.data)

        def _odom(self, msg):
            p = msg.pose.pose
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.t0 is None:
                self.t0 = t
            self.pending.append((t, Time.from_msg(msg.header.stamp), p.position.x, p.position.y,
                                 yaw_of(p.orientation), self.ready))

        def _drain(self, final=False):
            if not self.pending:
                return
            newest = self.pending[-1][0]
            while self.pending and (final or newest - self.pending[0][0] >= DELAY_S):
                t, stamp, gx, gy, gyaw, ready = self.pending.popleft()
                try:
                    tf = self.buf.lookup_transform(MAP_FRAME, BASE_FRAME, stamp)
                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                        tf2_ros.ConnectivityException):
                    self.n_skip += 1
                    continue
                tr = tf.transform
                self.w.writerow([f'{t:.4f}', f'{gx:.4f}', f'{gy:.4f}', f'{gyaw:.5f}',
                                 f'{tr.translation.x:.4f}', f'{tr.translation.y:.4f}',
                                 f'{yaw_of(tr.rotation):.5f}', int(ready)])
                self.n_ok += 1
            self.f.flush()

        def _report(self):
            self.get_logger().info(f'{self.n_ok} samples, {self.n_skip} without an estimate, '
                                   f'ready={self.ready}')

        def close(self):
            self._drain(final=True)
            self.f.close()
            self.get_logger().info(f'wrote {self.n_ok} samples ({self.n_skip} skipped) to {args.out}')

    rclpy.init()
    node = Recorder()
    try:
        if args.seconds > 0:
            import time
            end = time.monotonic() + args.seconds
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ---- plot -------------------------------------------------------------------

def _spans(x, mask):
    """(start, end) of each run of True in mask, in x units."""
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = x[i]
        if not m and start is not None:
            out.append((start, x[i]))
            start = None
    if start is not None:
        out.append((start, x[-1]))
    return out


def _stats(v):
    import numpy as np
    v = np.abs(v)
    return (f'mean {v.mean():.3f}  rms {math.sqrt((v * v).mean()):.3f}  '
            f'p90 {np.percentile(v, 90):.3f}  max {v.max():.3f}')


def plot(args):
    import os

    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    d = np.genfromtxt(args.csv, delimiter=',', names=True)
    if d.size < 2:
        sys.exit(f'{args.csv}: fewer than two samples')
    t = d['t'] - d['t'][0]
    ready = d['ready'] > 0.5
    step = np.hypot(np.diff(d['gt_x']), np.diff(d['gt_y']))
    step[step > JUMP_M] = 0.0
    dist = np.concatenate([[0.0], np.cumsum(step)])

    pos_err = np.hypot(d['est_x'] - d['gt_x'], d['est_y'] - d['gt_y'])
    yaw_err = np.array([wrap(e - g) for e, g in zip(d['est_yaw'], d['gt_yaw'])])
    gt_yaw = np.degrees(np.unwrap(d['gt_yaw']))
    # Estimated yaw drawn as ground truth plus the wrapped error, so both lines
    # unwrap together instead of drifting apart by 360 deg at a branch cut.
    est_yaw = gt_yaw + np.degrees(yaw_err)
    yaw_err_deg = np.degrees(yaw_err)

    out_dir = args.out_dir or os.path.splitext(args.csv)[0] + '_drift'
    os.makedirs(out_dir, exist_ok=True)
    title = os.path.basename(args.csv)

    def figure(x, xlabel, rows, name):
        fig, axes = plt.subplots(len(rows), 1, sharex=True, figsize=(12, 2.8 * len(rows)))
        for ax, (ylabel, series) in zip(axes, rows):
            for y, label, style in series:
                ax.plot(x, y, style, label=label, lw=1.0)
            for a, b in _spans(x, ~ready):
                ax.axvspan(a, b, color='0.85', lw=0)
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)
            if len(series) > 1:
                ax.legend(loc='best')
        axes[-1].set_xlabel(xlabel)
        axes[0].set_title(f'{title}  (grey: localization not ready)')
        fig.tight_layout()
        path = os.path.join(out_dir, name)
        fig.savefig(path, dpi=120)
        plt.close(fig)
        print(f'  {path}')

    pos_rows = [
        ('x [m]', [(d['gt_x'], 'actual', 'k-'), (d['est_x'], 'estimated', 'C1-')]),
        ('y [m]', [(d['gt_y'], 'actual', 'k-'), (d['est_y'], 'estimated', 'C1-')]),
        ('position error [m]', [(pos_err, 'error', 'C3-')]),
    ]
    yaw_rows = [
        ('yaw [deg]', [(gt_yaw, 'actual', 'k-'), (est_yaw, 'estimated', 'C1-')]),
        ('heading error [deg]', [(yaw_err_deg, 'error', 'C3-')]),
    ]
    print('figures:')
    figure(t, 'time [s]', pos_rows, 'position_vs_time.png')
    figure(dist, 'distance [m]', pos_rows, 'position_vs_distance.png')
    figure(t, 'time [s]', yaw_rows, 'orientation_vs_time.png')
    figure(dist, 'distance [m]', yaw_rows, 'orientation_vs_distance.png')

    n = int(ready.sum())
    print(f'\n{len(t)} samples over {t[-1]:.1f} s and {dist[-1]:.1f} m; {n} with localization ready')
    if n == 0:
        print('no ready samples: nothing to summarise')
        return
    print(f'position error [m]   {_stats(pos_err[ready])}')
    print(f'heading error [deg]  {_stats(yaw_err_deg[ready])}')

    # Where the position error points. Along-track error that grows with speed
    # is a time offset between the two sources' stamps, not drift: estimate
    # the offset and show what remains once it is removed.
    ex, ey = d['est_x'] - d['gt_x'], d['est_y'] - d['gt_y']
    h = d['gt_yaw']
    along = ex * np.cos(h) + ey * np.sin(h)
    cross = -ex * np.sin(h) + ey * np.cos(h)
    print(f'  along-track [m]    {_stats(along[ready])}   (signed mean {along[ready].mean():+.3f}, + = estimate ahead)')
    print(f'  cross-track [m]    {_stats(cross[ready])}   (signed mean {cross[ready].mean():+.3f}, + = estimate left)')
    lags = np.arange(-0.10, 0.1001, 0.005)
    errs = [np.hypot(np.interp(t + lag, t, d['est_x']) - d['gt_x'],
                     np.interp(t + lag, t, d['est_y']) - d['gt_y'])[ready].mean() for lag in lags]
    best = int(np.argmin(errs))
    print(f'  stamp offset       estimate matches best shifted {lags[best] * 1000:+.0f} ms: '
          f'mean position error {errs[best]:.3f} m instead of {pos_err[ready].mean():.3f}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('record', help='log ground truth and estimate (in the racer container)')
    r.add_argument('out')
    r.add_argument('--seconds', type=float, default=0.0, help='stop after N s (0: until Ctrl+C)')
    p = sub.add_parser('plot', help='figures and summary from a recording')
    p.add_argument('csv')
    p.add_argument('--out-dir', default='', help='default: <csv without .csv>_drift/')
    args = ap.parse_args()
    record(args) if args.cmd == 'record' else plot(args)


if __name__ == '__main__':
    main()
