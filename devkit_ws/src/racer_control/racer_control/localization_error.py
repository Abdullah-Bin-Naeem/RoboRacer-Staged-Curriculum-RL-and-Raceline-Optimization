#!/usr/bin/env python3

"""Measure how far AMCL's pose is from the truth.

The whole point of running the race-legal stack in the simulator is that ground
truth is still available to check it against. This node subscribes to both and
reports the error, so localization quality is a number rather than an impression.

    estimated: /amcl_pose                       (lidar + map + dead reckoning)
    truth:     /autodrive/roboracer_1/odom      (RESTRICTED at race time)

Development only -- it consumes a restricted topic by design.

WHAT IS MEASURED, AND WHAT IS NOT
---------------------------------
Sampling starts only after /localization_ready is latched, plus `grace_s`.
Everything before that is the bootstrap: the filter has not converged, the seed
may still be being retried, and the car may be standing still. Including it made
the lifetime mean and max meaningless -- the peak-error warning fired on almost
every run because of the startup transient, which trained you to ignore it.

Two statistics are reported, because they answer different questions:

    window  the last `window` samples -- what localization is doing RIGHT NOW,
            which is what you watch while tuning
    session everything since sampling opened -- the honest headline number,
            and the one that must stay under the raceline's wall margin

Set require_ready:=false to sample from the first message (the old behaviour).

Publishes the running error on ~/error (Float32, metres) so it can be plotted.
"""

import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32

NS = '/autodrive/roboracer_1'
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)
LATCHED = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)

# Clearance between the car body and the nearest wall at the tightest point of
# the line being driven -- error above this puts the car into a barrier.
# MEASURED from track_clean.pgm with the 0.27 m car width subtracted:
#     centerline_full.csv      0.367 m
#     raceline_scipy_a6/a8     0.089 m
#     raceline_tum.csv        -0.064 m   (line passes THROUGH a wall)
# Default is the optimized line, the tighter and more useful gate. Override with
#     -p wall_margin_m:=0.367
# when running the centerline.
DEFAULT_WALL_MARGIN_M = 0.089


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class _Running:
    """Bounded-memory summary: count, mean, max, and a p95 over a reservoir."""

    def __init__(self, reservoir=20000):
        self.n = 0
        self.total = 0.0
        self.peak = 0.0
        self._sample = deque(maxlen=reservoir)

    def add(self, v):
        self.n += 1
        self.total += v
        self.peak = max(self.peak, v)
        self._sample.append(v)

    @property
    def mean(self):
        return self.total / self.n if self.n else float('nan')

    @property
    def p95(self):
        return float(np.percentile(self._sample, 95)) if self._sample else float('nan')


class LocalizationError(Node):

    def __init__(self):
        super().__init__('localization_error')

        p = self.declare_parameter
        p('estimate_topic', '/amcl_pose')
        p('truth_topic', f'{NS}/odom')
        p('report_period', 5.0)
        # Ignore everything before localization declares itself converged.
        p('require_ready', True)
        p('grace_s', 1.0)          # extra settling after /localization_ready
        p('window', 200)           # samples in the rolling "right now" figure
        p('wall_margin_m', DEFAULT_WALL_MARGIN_M)

        g = lambda n: self.get_parameter(n).value
        est_topic = g('estimate_topic')
        truth_topic = g('truth_topic')
        self.require_ready = bool(g('require_ready'))
        self.grace_s = float(g('grace_s'))
        self.margin = float(g('wall_margin_m'))

        self.truth = None            # (x, y, yaw)
        self.estimate = None
        self.pos = _Running()
        self.yaw = _Running()
        self.win_pos = deque(maxlen=int(g('window')))
        self.win_yaw = deque(maxlen=int(g('window')))
        self.ready_at = None if self.require_ready else 0.0
        self.skipped = 0

        self.create_subscription(Odometry, truth_topic, self._cb_truth, QOS)
        self.create_subscription(PoseWithCovarianceStamped, est_topic, self._cb_est, QOS)
        if self.require_ready:
            self.create_subscription(Bool, '/localization_ready', self._cb_ready, LATCHED)
        self.pub = self.create_publisher(Float32, '~/error', QOS)
        self.create_timer(float(g('report_period')), self._report)

        self.get_logger().warn(
            f'DEVELOPMENT ONLY: comparing {est_topic} against {truth_topic}, '
            'which is RESTRICTED during racing')
        if self.require_ready:
            self.get_logger().info(
                f'sampling starts {self.grace_s:.1f} s after /localization_ready; '
                'bootstrap is excluded (require_ready:=false to include it)')

    def _cb_ready(self, msg):
        if msg.data and self.ready_at is None:
            self.ready_at = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(
                f'/localization_ready latched -- sampling opens in {self.grace_s:.1f} s '
                f'({self.skipped} bootstrap samples discarded)')

    def _open(self):
        if self.ready_at is None:
            return False
        return self.get_clock().now().nanoseconds * 1e-9 - self.ready_at >= self.grace_s

    def _cb_truth(self, msg):
        p = msg.pose.pose.position
        self.truth = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))

    def _cb_est(self, msg):
        p = msg.pose.pose.position
        self.estimate = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))
        if self.truth is None:
            return
        d = math.hypot(self.estimate[0] - self.truth[0],
                       self.estimate[1] - self.truth[1])
        a = abs(wrap(self.estimate[2] - self.truth[2]))

        # /error is published regardless, so a plot shows the convergence too.
        m = Float32()
        m.data = float(d)
        self.pub.publish(m)

        if not self._open():
            self.skipped += 1
            return
        self.pos.add(d)
        self.yaw.add(a)
        self.win_pos.append(d)
        self.win_yaw.append(a)

    def _report(self):
        if not self.pos.n:
            if self.estimate is None:
                self.get_logger().info('waiting for /amcl_pose ...')
            elif self.truth is None:
                self.get_logger().info('waiting for ground truth ...')
            elif not self._open():
                self.get_logger().info(
                    f'holding: waiting for /localization_ready ({self.skipped} '
                    'samples discarded so far)')
            return
        wp = np.array(self.win_pos)
        wa = np.degrees(np.array(self.win_yaw))
        self.get_logger().info(
            f'now[{len(wp):3d}] {wp.mean():.3f} m / {wa.mean():.2f} deg  |  '
            f'session[n={self.pos.n}] mean {self.pos.mean:.3f} '
            f'p95 {self.pos.p95:.3f} max {self.pos.peak:.3f} m, '
            f'heading mean {math.degrees(self.yaw.mean):.2f} '
            f'max {math.degrees(self.yaw.peak):.2f} deg')

    def summary(self):
        if not self.pos.n:
            print(f'\nno samples collected ({self.skipped} discarded before '
                  '/localization_ready)')
            return
        print('\n' + '=' * 62)
        print(f'  localization error over {self.pos.n} updates '
              f'({self.skipped} bootstrap samples excluded)')
        print(f'    position  mean {self.pos.mean:.3f}  p95 {self.pos.p95:.3f}  '
              f'max {self.pos.peak:.3f}  m')
        print(f'    heading   mean {math.degrees(self.yaw.mean):.2f}  '
              f'p95 {math.degrees(self.yaw.p95):.2f}  '
              f'max {math.degrees(self.yaw.peak):.2f}  deg')
        print('=' * 62)
        if self.pos.p95 > self.margin:
            print(f'  WARNING: p95 error exceeds the wall margin '
                  f'({self.margin:.3f} m) of the line being driven.')
            print('           Expect contact at the tightest point of the lap.')
        elif self.pos.peak > self.margin:
            print(f'  NOTE: peak error exceeded {self.margin:.3f} m but p95 did '
                  'not -- occasional excursions, not a systematic offset.')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
