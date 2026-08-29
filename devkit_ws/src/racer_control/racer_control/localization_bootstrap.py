#!/usr/bin/env python3

"""Get AMCL a starting pose, then hand over to the follower.

Two modes:

  mode:=truth   (default)  Read /ips and /imu ONCE, seed AMCL with that pose,
                           and hand over as soon as it settles. Ground truth is
                           RESTRICTED at race time, so this is a development
                           shortcut -- but only for the first instant. After the
                           seed, tracking is lidar + map + dead reckoning alone,
                           which is the part actually worth testing.

  mode:=global             No initial pose at all. Scatter particles across the
                           map, creep forward on lidar alone, and wait for the
                           cloud to collapse. This is how a real race starts, and
                           it is slower and less reliable -- one stretch of
                           corridor looks much like another, so it has to reach a
                           corner before the ambiguity breaks.

Either way it latches /localization_ready when done, which the follower waits on
instead of a fixed timer.

The seed assumes map coordinates match the simulator's world frame. They do
here: the SLAM run had scan matching disabled, so slam_toolbox's map->world
correction was identity.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Empty

NS = '/autodrive/roboracer_1'
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)
LATCHED = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class LocalizationBootstrap(Node):

    def __init__(self):
        super().__init__('localization_bootstrap')

        p = self.declare_parameter
        p('throttle', 0.08)              # gentle -- this is a search, not a lap
        p('steer_gain', 0.6)             # wall-centring proportional gain
        p('pos_std_target', 0.15)        # [m]   converged when below this
        p('yaw_std_target', 0.09)        # [rad] ~5 degrees
        p('straight_distance_m', 3.0)    # creep straight this far, then seek a corner
        p('timeout_s', 60.0)
        p('mode', 'truth')               # 'truth' seeds from /ips; 'global' searches
        p('settle_s', 2.0)               # after seeding, let AMCL absorb a few scans
        p('wheel_radius', 0.0590)

        g = lambda n: self.get_parameter(n).value
        self.throttle = g('throttle')
        self.steer_gain = g('steer_gain')
        self.pos_target = g('pos_std_target')
        self.yaw_target = g('yaw_std_target')
        self.straight_m = g('straight_distance_m')
        self.timeout = g('timeout_s')
        self.wheel_r = g('wheel_radius')
        self.mode = str(g('mode')).lower()
        self.settle_s = g('settle_s')
        self.truth_pos = None
        self.truth_quat = None
        self.seeded_at = None

        self.scan = None
        self.ready = False
        self.distance = 0.0
        self._enc = {}
        self._rate = {}
        self.speed = 0.0
        self.pos_std = None
        self.yaw_std = None
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self._last = None

        self.create_subscription(LaserScan, f'{NS}/lidar', self._cb_scan, QOS)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._cb_pose, QOS)
        self.create_subscription(JointState, f'{NS}/left_encoder',
                                 lambda m: self._cb_enc('l', m), QOS)
        self.create_subscription(JointState, f'{NS}/right_encoder',
                                 lambda m: self._cb_enc('r', m), QOS)
        if self.mode == 'truth':
            self.create_subscription(Point, f'{NS}/ips', self._cb_ips, QOS)
            self.create_subscription(Imu, f'{NS}/imu', self._cb_imu, QOS)

        self.pub_t = self.create_publisher(Float32, f'{NS}/throttle_command', QOS)
        self.pub_s = self.create_publisher(Float32, f'{NS}/steering_command', QOS)
        self.pub_ready = self.create_publisher(Bool, '/localization_ready', LATCHED)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped,
                                              '/initialpose', QOS)

        self._announce(False)
        if self.mode == 'truth':
            self.get_logger().warn(
                'mode=truth: seeding the initial pose from /ips + /imu, which are '
                'RESTRICTED at race time. Tracking afterwards is race-legal.')
        else:
            self._scatter()

        self.create_timer(0.05, self._tick)
        self.create_timer(2.0, self._report)

    # ---- setup -----------------------------------------------------------

    def _scatter(self):
        cli = self.create_client(Empty, '/reinitialize_global_localization')
        if not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                'no /reinitialize_global_localization service; falling back to '
                "AMCL's configured initial pose")
            return
        cli.call_async(Empty.Request())
        self.get_logger().info('scattered particles across the map -- no initial pose needed')

    def _announce(self, value):
        m = Bool()
        m.data = bool(value)
        self.pub_ready.publish(m)

    # ---- callbacks -------------------------------------------------------

    def _cb_scan(self, msg):
        self.scan = msg

    def _cb_ips(self, msg):
        self.truth_pos = (msg.x, msg.y)

    def _cb_imu(self, msg):
        q = msg.orientation
        self.truth_quat = (q.x, q.y, q.z, q.w)

    def _seed(self):
        """Publish the true pose to /initialpose, once."""
        x, y = self.truth_pos
        qx, qy, qz, qw = self.truth_quat
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        (m.pose.pose.orientation.x, m.pose.pose.orientation.y,
         m.pose.pose.orientation.z, m.pose.pose.orientation.w) = qx, qy, qz, qw
        # Tight but not zero: the seed is good, the map is not perfect.
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05 ** 2
        m.pose.covariance[35] = math.radians(3.0) ** 2
        self.pub_init.publish(m)
        self.seeded_at = self.get_clock().now().nanoseconds * 1e-9
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        self.get_logger().info(
            f'seeded AMCL at x={x:+.3f} y={y:+.3f} yaw={math.degrees(yaw):+.1f} deg')

    def _cb_pose(self, msg):
        c = msg.pose.covariance
        self.pos_std = math.sqrt(max(c[0], 0.0) + max(c[7], 0.0))
        self.yaw_std = math.sqrt(max(c[35], 0.0))

    def _cb_enc(self, side, msg):
        if not msg.position:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ang = float(msg.position[0])
        prev = self._enc.get(side)
        self._enc[side] = (ang, t)
        if prev is None:
            return
        dt = t - prev[1]
        if dt <= 1e-4 or dt > 0.5:
            return
        self._rate[side] = (ang - prev[0]) / dt * self.wheel_r
        rates = [v for v in self._rate.values() if v is not None]
        if rates:
            self.speed = float(np.mean(rates))

    # ---- behaviour -------------------------------------------------------

    def _beam(self, angle):
        """Range at `angle` radians, ignoring inf/nan."""
        s = self.scan
        i = int(round((angle - s.angle_min) / s.angle_increment))
        if not 0 <= i < len(s.ranges):
            return float('inf')
        window = [r for r in s.ranges[max(0, i - 4):i + 5]
                  if math.isfinite(r) and r > s.range_min]
        return min(window) if window else float('inf')

    def _drive(self):
        """Creep forward, centred between the walls. Lidar only."""
        left = self._beam(math.radians(90))
        right = self._beam(math.radians(-90))
        front = self._beam(0.0)

        steer = 0.0
        if math.isfinite(left) and math.isfinite(right):
            steer = self.steer_gain * (right - left) / max(left + right, 0.1)

        # Past the straight phase, bias the steering so the car finds a corner --
        # corridors are ambiguous, corners are not.
        if self.distance > self.straight_m:
            steer += 0.25

        throttle = self.throttle
        if math.isfinite(front) and front < 1.0:
            throttle *= max(0.0, (front - 0.4) / 0.6)   # ease off near a wall

        self._send(throttle, float(np.clip(steer, -1.0, 1.0)))

    def _send(self, throttle, steering):
        t, s = Float32(), Float32()
        t.data, s.data = float(throttle), float(steering)
        self.pub_t.publish(t)
        self.pub_s.publish(s)

    def _converged(self):
        return (self.pos_std is not None
                and self.pos_std <= self.pos_target
                and self.yaw_std <= self.yaw_target)

    def _tick(self):
        if self.ready:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last is not None:
            self.distance += abs(self.speed) * (now - self._last)
        self._last = now

        if self.mode == 'truth':
            if self.seeded_at is None:
                if self.truth_pos is None or self.truth_quat is None:
                    return          # still waiting for the first /ips and /imu
                self._seed()
                return
            if now - self.seeded_at >= self.settle_s:
                std = f'{self.pos_std:.3f} m' if self.pos_std is not None else 'unknown'
                self._finish(f'seeded and settled ({std}); tracking is now '
                             'lidar + map + dead reckoning only')
            return

        if self._converged():
            self._finish(f'converged after {self.distance:.2f} m '
                         f'(pos std {self.pos_std:.3f} m, '
                         f'yaw std {math.degrees(self.yaw_std):.1f} deg)')
            return

        if now - self.t0 > self.timeout:
            self._finish(f'TIMEOUT after {self.timeout:.0f} s -- handing over with '
                         f'pos std {self.pos_std if self.pos_std else float("nan"):.3f} m. '
                         'The pose may be wrong; check the scan against the map.')
            return

        if self.scan is not None:
            self._drive()   # global mode only -- truth mode returns above

    def _finish(self, message):
        self.ready = True
        self._send(0.0, 0.0)
        self._announce(True)
        self.get_logger().info(message)
        self.get_logger().info('/localization_ready = true, follower may take over')

    def _report(self):
        if self.ready:
            return
        if self.mode == 'truth' and self.seeded_at is None:
            self.get_logger().info(
                f'waiting for ground truth: ips={"ok" if self.truth_pos else "--"} '
                f'imu={"ok" if self.truth_quat else "--"}')
            return
        if self.pos_std is None:
            self.get_logger().info('waiting for /amcl_pose ...')
            return
        self.get_logger().info(
            f'searching: {self.distance:.2f} m driven, pos std {self.pos_std:.3f} m '
            f'(target {self.pos_target}), yaw std {math.degrees(self.yaw_std):.1f} deg '
            f'(target {math.degrees(self.yaw_target):.1f})')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationBootstrap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # The context may already be gone on Ctrl-C; stopping is best-effort.
        try:
            node._send(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
