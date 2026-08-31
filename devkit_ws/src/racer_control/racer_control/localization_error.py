#!/usr/bin/env python3

"""Measure how far the localization estimate is from the truth.

The whole point of running the race-legal stack in the simulator is that ground
truth is still available to check it against. This node subscribes to both and
reports the error, so localization quality is a number rather than an impression.

    estimated: /amcl_pose, or the map->base TRANSFORM  (lidar + map + dead reckoning)
    truth:     /autodrive/roboracer_1/odom             (RESTRICTED at race time)

Development only -- it consumes a restricted topic by design.

TWO WAYS TO READ THE ESTIMATE
-----------------------------
    use_tf:=false  (default)  subscribe to `estimate_topic`, i.e. /amcl_pose
    use_tf:=true              look up map -> base_frame from TF at `tf_rate`

Both are legitimate; they measure different things.

The TOPIC path measures the localizer in isolation -- AMCL's /amcl_pose or
slam_toolbox's /pose (same message type, so estimate_topic:=/pose is all the
swap needs). That is the corrected pose at scan time, at the localizer's own
update rate.

The TF path measures map -> base, which is the localizer's correction COMPOSED
with dead_reckoning's 200 Hz odometry. That is what pure_pursuit actually
consumes when use_tf_pose is true, so it is the pose the car really drives on,
sampled between scans as well as at them. If the question is "will I hit a
wall", this is the honest one.

WHAT IS MEASURED, AND WHAT IS NOT
---------------------------------
Sampling starts only after /localization_ready is latched, plus `grace_s`.
Everything before that is the bootstrap: the filter has not converged, the seed
may still be being retried, and the car may be standing still. Including it made
the lifetime mean and max meaningless -- the peak-error warning fired on almost
every run because of the startup transient, which trained you to ignore it.

Set require_ready:=false to sample from the first message. Do that whenever
localization_bootstrap is NOT running -- nothing else latches
/localization_ready, so the node would otherwise hold forever.

Two statistics are reported, because they answer different questions:

    window  the last `window` samples -- what localization is doing RIGHT NOW,
            which is what you watch while tuning
    session everything since sampling opened -- the honest headline number,
            and the one that must stay under the raceline's wall margin

Publishes the running error on ~/error (Float32, metres) so it can be plotted.
"""

import math
from collections import deque

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Float32

NS = '/autodrive/roboracer_1'
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)
LATCHED = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)

# The raceline's tightest point sits this far from a wall, so sustained error
# above it will put the car into one.
WALL_MARGIN_M = 0.134


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
        # Read the estimate from TF instead of a topic -- see the module
        # docstring. Required for any localizer that does not publish
        # /amcl_pose, which is all of them except AMCL.
        p('use_tf', False)
        p('map_frame', 'map')
        p('base_frame', 'roboracer_1')
        p('tf_rate', 20.0)         # sampling rate for the TF path

        g = lambda n: self.get_parameter(n).value
        est_topic = g('estimate_topic')
        truth_topic = g('truth_topic')
        self.require_ready = bool(g('require_ready'))
        self.grace_s = float(g('grace_s'))
        self.use_tf = bool(g('use_tf'))
        self.map_frame = str(g('map_frame'))
        self.base_frame = str(g('base_frame'))

        self.truth = None            # (x, y, yaw)
        self.estimate = None
        self.pos = _Running()
        self.yaw = _Running()
        self.win_pos = deque(maxlen=int(g('window')))
        self.win_yaw = deque(maxlen=int(g('window')))
        self.ready_at = None if self.require_ready else 0.0
        self.skipped = 0
        self._tf_warned = False

        self.create_subscription(Odometry, truth_topic, self._cb_truth, QOS)

        if self.use_tf:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.create_timer(1.0 / float(g('tf_rate')), self._tf_tick)
            self.source = f'TF {self.map_frame} -> {self.base_frame}'
        else:
            self.create_subscription(
                PoseWithCovarianceStamped, est_topic, self._cb_est, QOS)
            self.source = est_topic

        if self.require_ready:
            self.create_subscription(Bool, '/localization_ready', self._cb_ready, LATCHED)
        self.pub = self.create_publisher(Float32, '~/error', QOS)
        self.create_timer(float(g('report_period')), self._report)

        self.get_logger().warn(
            f'DEVELOPMENT ONLY: comparing {self.source} against {truth_topic}, '
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

    def _tf_tick(self):
        """Sample map->base from TF. Time() means 'latest available'.

        Asking for the latest rather than `now` avoids racing the publishers:
        map->odom updates at the scan rate while odom->base updates far faster,
        so a lookup at `now` is an extrapolation past the newest map->odom.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            if not self._tf_warned:
                self.get_logger().info(f'waiting for {self.source}: {exc}')
                self._tf_warned = True
            return
        self._tf_warned = False
        t = tf.transform.translation
        self._accumulate(t.x, t.y, yaw_from_quat(tf.transform.rotation))

    def _cb_est(self, msg):
        p = msg.pose.pose.position
        self._accumulate(p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))

    def _accumulate(self, x, y, yaw):
        self.estimate = (x, y, yaw)
        if self.truth is None:
            return
        d = math.hypot(x - self.truth[0], y - self.truth[1])
        a = abs(wrap(yaw - self.truth[2]))

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
                self.get_logger().info(f'waiting for {self.source} ...')
            elif self.truth is None:
                self.get_logger().info('waiting for ground truth ...')
            elif not self._open():
                self.get_logger().info(
                    f'holding: waiting for /localization_ready ({self.skipped} '
                    'samples discarded so far). Pass require_ready:=false when '
                    'localization_bootstrap is not running.')
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
        print(f'    source    {self.source}')
        print(f'    position  mean {self.pos.mean:.3f}  p95 {self.pos.p95:.3f}  '
              f'max {self.pos.peak:.3f}  m')
        print(f'    heading   mean {math.degrees(self.yaw.mean):.2f}  '
              f'p95 {math.degrees(self.yaw.p95):.2f}  '
              f'max {math.degrees(self.yaw.peak):.2f}  deg')
        print('=' * 62)
        if self.pos.p95 > WALL_MARGIN_M:
            print(f'  WARNING: p95 error exceeds the raceline wall margin '
                  f'({WALL_MARGIN_M:.3f} m).')
            print('           Expect contact at the tightest point of the lap.')
        elif self.pos.peak > WALL_MARGIN_M:
            print(f'  NOTE: peak error exceeded {WALL_MARGIN_M:.3f} m but p95 did '
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
