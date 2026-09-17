#!/usr/bin/env python3

"""Race-legal odometry from wheel encoders + IMU.

AMCL cannot run without an odometry input -- its particle propagation step IS a
motion model driven by odometry deltas. The devkit's own /odom is ground truth
and RESTRICTED at race time, so this node rebuilds an equivalent from the two
permitted sources:

    distance  <- wheel encoders  (cumulative wheel angle, radians)
                 or the tire observer on the wheel speed (distance_source)
    heading   <- IMU orientation (absolute quaternion, so yaw does NOT drift)

Publishes `odom -> base_frame` on /tf plus a nav_msgs/Odometry. Position drifts,
because the sim's encoders report the commanded wheel speed rather than the car
(see common/tire_model.py). Correcting that drift is AMCL's job.

Because IMU yaw is absolute and world-referenced, the `odom` frame here comes out
aligned with the simulator's world orientation; only its origin differs.

ONE STEP PER SIMULATOR FRAME
----------------------------
The bridge publishes every topic from one handler per WebSocket frame, stamped
on receipt, encoders first and IMU after. So a frame is: left encoder, right
encoder, IMU, lidar, within a millisecond or two. Travel is banked once per
frame, only when EVERY live encoder has reported it:

  * banking on the first encoder alone moved the car by mean(dl, 0) = half a
    step, while the extrapolation clock had already advanced to the new frame,
    so the transform sampled in between lagged by ~ds/2 (0.22 m at 8 m/s).
  * an encoder that stops publishing is dropped from the mean after
    encoder_stale_s rather than contributing 0 forever, which halved distance.

The step moves along the MEAN heading of the interval, IMU yaw at the previous
frame and at this one. The newest heading alone rotates every chord of a corner
the same way, a systematic drift AMCL's zero-mean noise cannot represent:
simulated on raceline_a7.0 at 18 Hz, 0.126 m worst per lap against 0.039 m.
The IMU for a frame lands just after its encoders, so a complete frame waits up
to imu_wait_s in the timer for it before being banked on an extrapolated yaw.

DISTANCE NEVER TOUCHES A RATE x A DIFFERENT dt
----------------------------------------------
Encoder angle is CUMULATIVE, so arc length is `dangle * r` regardless of how
long the interval was or whether frames were dropped. Holding a rate from one
interval across another (Jensen) inflated distance by ~(sigma/mu)^2, ~2% at
15% stamp jitter. With distance_source=tire the observer's speed is integrated
over the SAME stamp interval the wheel speed was measured on, and those
intervals telescope, so the sum is unbiased.
"""

import math
from collections import deque

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from roboracer_stack.common.tire_model import TireSpeedObserver
from sensor_msgs.msg import Imu, JointState

NS = '/autodrive/roboracer_1'
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)
SIDES = ('l', 'r')


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class DeadReckoning(Node):

    def __init__(self):
        super().__init__('dead_reckoning')

        p = self.declare_parameter
        # MEASURED 0.0581, not the published 0.0590: path-integration gave
        # 0.05815 and steady-state speed matching 0.05813. The spec value
        # overreads distance by 1.55%, a SYSTEMATIC bias. Measured on the
        # practice track; the wheel is the same car on the compete track.
        p('wheel_radius', 0.0581)
        p('odom_frame', 'odom')
        p('base_frame', 'roboracer_1')  # devkit's name, so `lidar` still parents to it
        # 200 Hz, not 50. MEASURED: scans land uniformly in the gap between odom
        # transforms, so at 50 Hz (20 ms) 96% of them are stamped AHEAD of the
        # newest one. AMCL must look up odom->base AT the scan timestamp to
        # build map->odom. 200 Hz cuts the worst case to 5 ms. Pure resampling
        # of the integrated pose: the rate does not change the distance.
        p('publish_rate', 200.0)
        p('publish_tf', True)
        # Post-date the TF stamp so a consumer asking for "now" lands INSIDE the
        # transform window rather than extrapolating past its end.
        p('transform_tolerance', 0.02)
        # Encoders overread under slip. <1.0 trims the systematic part; AMCL
        # handles what is left.
        p('distance_scale', 1.0)
        # 'encoder': arc length from the cumulative wheel angle.
        # 'tire':    the tire observer's car speed over the same interval, which
        #            takes the wheelspin at launch and the under-read while
        #            braking out of the distance. See common/tire_model.py.
        p('distance_source', 'encoder')
        p('tire_rise_slope', 3.0)                   # as pure_pursuit.yaml
        p('v_slip_den', 4.0)
        # Reset detector, in wheel radians. Cumulative angle survives dropped
        # frames, so the ONLY delta worth rejecting is a discontinuity -- the
        # sim resetting the counter. At the 24 m/s top speed (413 rad/s):
        #     one 18 Hz tick ~23 rad, a 0.5 s stall ~206 rad,
        #     a full 44 m compete lap ~757 rad.
        # A wall respawn does NOT reset the counter; bootstrap's respawn watch
        # handles that from the IMU heading jump.
        p('max_wheel_step', 300.0)
        # An encoder silent for this long (while the other reports) is dead and
        # leaves the mean. Well above one frame at the slowest measured 12.8 Hz.
        p('encoder_stale_s', 0.25)
        # How long a complete encoder frame waits for its IMU message before it
        # is banked on the extrapolated heading instead.
        p('imu_wait_s', 0.02)
        # Frames the reported/extrapolation speed is averaged over (encoder
        # source). One frame carries the full receipt-stamp jitter.
        p('speed_window_frames', 3)
        # How hard the de-jittered frame clock follows the raw stamps (0..1).
        p('frame_clock_gain', 0.1)
        # Carry heading and position forward to each transform's stamp with the
        # IMU yaw rate and the speed. Otherwise the transform covering scan k
        # carries frame k-1: measured 8-14 deg map->odom swings at the S-curve
        # and a 35-40 ms x speed lead of the estimate. Capped at 0.1 s.
        p('extrapolate_yaw', True)
        p('extrapolate_pos', True)

        g = lambda n: self.get_parameter(n).value
        self.wheel_r = float(g('wheel_radius'))
        self.odom_frame, self.base_frame = g('odom_frame'), g('base_frame')
        self.publish_tf = bool(g('publish_tf'))
        self.scale = float(g('distance_scale'))
        self.tf_tol = float(g('transform_tolerance'))
        self.max_step = float(g('max_wheel_step'))
        self.stale_s = float(g('encoder_stale_s'))
        self.imu_wait = float(g('imu_wait_s'))
        self.extrapolate_yaw = bool(g('extrapolate_yaw'))
        self.extrapolate_pos = bool(g('extrapolate_pos'))
        self.source = str(g('distance_source')).lower()
        if self.source not in ('encoder', 'tire'):
            raise RuntimeError(f"distance_source must be 'encoder' or 'tire', not {self.source!r}")
        self.observer = TireSpeedObserver(g('tire_rise_slope'), g('v_slip_den'))

        # ---- integrated state, valid at self._frame_t ----
        self.x = self.y = 0.0
        self.speed = 0.0
        self._frame_t = None            # stamp of the last banked frame
        self._frame_yaw = None          # IMU yaw (unwrapped) at that stamp
        self._frame_th = None           # its de-jittered time, see _frame_clock
        self._fc_t = self._fc_raw = None
        self._fc_gaps = deque(maxlen=16)
        self.fc_gain = float(g('frame_clock_gain'))
        self._odo = 0.0                 # cumulative distance, for the speed window
        self._odo_hist = deque(maxlen=max(2, int(g('speed_window_frames')) + 1))

        # ---- encoders ----
        self._enc = {}                  # side -> (angle, stamp) of its newest message
        self._pend = {}                 # side -> metres not yet banked
        self._fresh = {}                # side -> reported since the last bank
        self._pend_t = None             # newest stamp among the pending messages
        self._dead_warned = set()
        self._resets = 0

        # ---- IMU: (stamp, unwrapped yaw, yaw rate) ----
        self._imu = deque(maxlen=64)
        self.yaw_rate = 0.0

        self.create_subscription(JointState, f'{NS}/left_encoder',
                                 lambda m: self._cb_enc('l', m), QOS)
        self.create_subscription(JointState, f'{NS}/right_encoder',
                                 lambda m: self._cb_enc('r', m), QOS)
        self.create_subscription(Imu, f'{NS}/imu', self._cb_imu, QOS)

        self.pub = self.create_publisher(Odometry, 'odom', QOS)
        self.tfb = tf2_ros.TransformBroadcaster(self)
        self.create_timer(1.0 / g('publish_rate'), self._tick)

        self.get_logger().info(
            f'dead reckoning: {self.odom_frame} -> {self.base_frame}, '
            f'distance from {self.source}, wheel r={self.wheel_r} m, scale={self.scale}, '
            f'{g("publish_rate"):.0f} Hz, tf +{self.tf_tol * 1e3:.0f} ms, '
            f'extrapolate yaw={self.extrapolate_yaw} pos={self.extrapolate_pos}')

    # ---- inputs ------------------------------------------------------------

    def _cb_enc(self, side, msg):
        """position is cumulative wheel angle in RADIANS (measured, not ticks)."""
        if not msg.position:
            return
        self._on_encoder(side, float(msg.position[0]), stamp_s(msg.header))

    def _on_encoder(self, side, ang, t):
        prev = self._enc.get(side)
        self._enc[side] = (ang, t)
        if prev is None:
            return
        if any(o != side and self._enc[o][1] > prev[1] + self.stale_s for o in self._enc):
            # Back from silence while the other side kept reporting: its delta
            # spans frames the other side has already banked alone. Resync
            # instead of adding them twice. (A stall of BOTH sides is not
            # covered by anyone, so that delta is kept -- cumulative angle.)
            return
        dang = ang - prev[0]
        if abs(dang) > self.max_step:
            # Counter discontinuity, not travel. _enc is already resynced, so
            # the next delta is measured from the new origin.
            self._resets += 1
            self.get_logger().warn(
                f'{side} encoder jumped {dang:.1f} rad (> {self.max_step:.0f}); '
                'treating as a counter reset, not travel.')
            return
        self._pend[side] = self._pend.get(side, 0.0) + dang * self.wheel_r
        self._fresh[side] = True
        self._pend_t = t if self._pend_t is None else max(self._pend_t, t)

    def _cb_imu(self, msg):
        self._on_imu(stamp_s(msg.header), yaw_from_quat(msg.orientation),
                     msg.angular_velocity.z)

    def _on_imu(self, t, yaw, rate):
        if self._imu:
            t0, y0, _ = self._imu[-1]
            if t <= t0:
                return                  # duplicate or out of order
            yaw = y0 + wrap(yaw - wrap(y0))
        self._imu.append((t, yaw, rate))
        self.yaw_rate = rate

    # ---- heading -----------------------------------------------------------

    def _yaw_at(self, t, extrapolate=True):
        """Unwrapped IMU heading at time t: interpolated between samples, or the
        newest carried forward by its rate (capped at 0.1 s so a stalled IMU
        cannot spin the estimate). None before the first IMU message."""
        h = self._imu
        if not h:
            return None
        tn, yn, rn = h[-1]
        if t >= tn:
            return yn + (rn * min(t - tn, 0.1) if extrapolate else 0.0)
        for i in range(len(h) - 1, 0, -1):
            ta, ya, _ = h[i - 1]
            tb, yb, _ = h[i]
            if ta <= t:
                return ya + (yb - ya) * (t - ta) / (tb - ta)
        return h[0][1]

    # ---- integration -------------------------------------------------------

    def _frame_complete(self):
        """Every live encoder has reported since the last bank."""
        if self._pend_t is None:
            return False
        live = [s for s in self._enc if self._pend_t - self._enc[s][1] <= self.stale_s]
        for s in self._enc:
            if s not in live and s not in self._dead_warned:
                self._dead_warned.add(s)
                self.get_logger().warn(
                    f'{s} encoder silent for > {self.stale_s:.2f} s; distance from the '
                    'other side alone until it returns')
            elif s in live:
                self._dead_warned.discard(s)
        return bool(live) and all(self._fresh.get(s, False) for s in live)

    def _maybe_bank(self, now):
        if not self._frame_complete():
            return
        t = self._pend_t
        imu_in = bool(self._imu) and self._imu[-1][0] >= t
        if not imu_in and now - t < self.imu_wait:
            return                      # the frame's IMU is a millisecond behind
        self._bank(t)

    def _bank(self, t):
        live = [s for s in self._enc if t - self._enc[s][1] <= self.stale_s]
        ds_enc = sum(self._pend.get(s, 0.0) for s in live) / max(len(live), 1)
        for s in list(self._pend):
            self._pend[s] = 0.0
            self._fresh[s] = False
        self._pend_t = None

        yaw1 = self._yaw_at(t)
        if self._frame_t is None or self._frame_yaw is None or yaw1 is None:
            # First frame (or no IMU yet): establish the origin, bank nothing.
            # The car is parked; pre-roll travel is dropped, not guessed.
            self._frame_t, self._frame_yaw = t, yaw1
            self._frame_th = self._frame_clock(t)
            self._odo_hist.append((self._frame_th, self._odo))
            return

        th = self._frame_clock(t)
        dt = th - self._frame_th
        self._frame_th = th
        if self.source == 'tire' and 0.0 < dt < 0.5:
            v0 = self.observer.v
            v1 = self.observer.step(ds_enc / dt, dt)
            ds = 0.5 * (v0 + v1) * dt
        else:
            ds = ds_enc
        ds *= self.scale

        h = 0.5 * (self._frame_yaw + yaw1)          # unwrapped, so a plain mean
        self.x += ds * math.cos(h)
        self.y += ds * math.sin(h)
        self._odo += ds
        self._frame_t, self._frame_yaw = t, yaw1

        self._odo_hist.append((th, self._odo))
        if self.source == 'tire':
            self.speed = self.observer.v * self.scale
        else:
            t0, o0 = self._odo_hist[0]
            if th - t0 > 1e-3:
                self.speed = (self._odo - o0) / (th - t0)

    def _frame_clock(self, t):
        """De-jittered frame time, for RATES only (speed, the tire observer).

        Stamps are receipt times: WebSocket + Unity jitter of ~15 % of a frame.
        A wheel speed of (distance / stamp interval) carries all of it, and the
        tire curve is nonlinear, so a noisy u biases the observer: offline, a
        matched tire model ended a lap 0.03 m out on clean stamps and 0.99 m
        out on jittered ones. So the frame clock advances by the median period
        and follows the raw stamp with a small gain; a real irregularity (a
        stall, a dropped frame) is more than half a period off and snaps.
        Position extrapolation keeps the raw stamp, which shares the TF clock.
        """
        if self._fc_raw is not None:
            gap = t - self._fc_raw
            if gap > 0.0:
                self._fc_gaps.append(gap)
        self._fc_raw = t
        if self._fc_t is None or not self._fc_gaps:
            self._fc_t = t
            return t
        period = sorted(self._fc_gaps)[len(self._fc_gaps) // 2]
        err = t - (self._fc_t + period)
        if abs(err) > 0.5 * period:
            self._fc_t = t
        else:
            self._fc_t += period + self.fc_gain * err
        return self._fc_t

    def _pose_at(self, t):
        """(x, y, yaw) at time t: the banked frame carried forward by the speed
        and yaw rate (see extrapolate_*), capped at 0.1 s so a stalled sensor
        cannot run the estimate away."""
        yaw = self._yaw_at(t, extrapolate=self.extrapolate_yaw)
        if not self.extrapolate_pos or self._frame_t is None:
            return self.x, self.y, yaw
        dt = max(0.0, min(t - self._frame_t, 0.1))
        ym = self._yaw_at(self._frame_t + 0.5 * dt)
        return (self.x + self.speed * dt * math.cos(ym),
                self.y + self.speed * dt * math.sin(ym), yaw)

    # ---- output ------------------------------------------------------------

    def _tick(self):
        now = self.get_clock().now()
        self._step(now.nanoseconds * 1e-9)

    def _step(self, t):
        """Bank a complete frame if one is waiting, then publish at time t."""
        self._maybe_bank(t)
        if not self._imu or self._frame_t is None:
            return

        t_tf = t + max(self.tf_tol, 0.0)
        x_tf, y_tf, yaw_tf = self._pose_at(t_tf)
        x_od, y_od, yaw_od = self._pose_at(t)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = rclpy.time.Time(seconds=t_tf).to_msg()
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x_tf
            tf.transform.translation.y = y_tf
            tf.transform.rotation.z = math.sin(0.5 * yaw_tf)
            tf.transform.rotation.w = math.cos(0.5 * yaw_tf)
            self.tfb.sendTransform(tf)

        od = Odometry()
        od.header.stamp = rclpy.time.Time(seconds=t).to_msg()
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y = x_od, y_od
        od.pose.pose.orientation.z = math.sin(0.5 * yaw_od)
        od.pose.pose.orientation.w = math.cos(0.5 * yaw_od)
        od.twist.twist.linear.x = self.speed
        od.twist.twist.angular.z = self.yaw_rate
        # Informational only: nav2 AMCL reads odometry from TF, never this topic.
        od.pose.covariance[0] = od.pose.covariance[7] = 0.05
        od.pose.covariance[35] = 0.01
        od.twist.covariance[0] = 0.02
        od.twist.covariance[35] = 0.01
        self.pub.publish(od)


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
