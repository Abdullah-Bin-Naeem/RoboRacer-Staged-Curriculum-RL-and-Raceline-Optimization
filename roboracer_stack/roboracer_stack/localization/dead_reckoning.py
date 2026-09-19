#!/usr/bin/env python3

"""Race-legal odometry from wheel encoders + IMU.

AMCL cannot run without an odometry input -- its particle propagation step IS a
motion model driven by odometry deltas. The devkit's own /odom is ground truth
and RESTRICTED at race time, so this node rebuilds an equivalent from the two
permitted sources:

    distance  <- wheel encoders  (cumulative wheel angle, radians)
    heading   <- IMU orientation (absolute quaternion, so yaw does NOT drift)

Publishes `odom -> base_frame` on /tf plus a nav_msgs/Odometry. Position drifts,
because wheel slip makes the encoders overread (measured at up to ~3x under hard
acceleration in the RL work). Correcting that drift is exactly AMCL's job -- but
it is also why the motion-model noise in the AMCL config has to be generous.

Because IMU yaw is absolute and world-referenced, the `odom` frame here comes out
aligned with the simulator's world orientation; only its origin differs.

DISTANCE NEVER TOUCHES dt
-------------------------
Encoder angle is CUMULATIVE, so arc length is `dangle * r` regardless of how
long the interval was or whether frames were dropped. Integrating instead as
`(dangle/dt_encoder) * dt_timer` -- a rate multiplied straight back into a
different interval -- is not an identity, and it was biasing distance two ways:

  * Jensen. The speed from interval k is held across interval k+1, and for
    jittery intervals E[dt_{k+1}/dt_k] = mu * E[1/dt] > 1, inflating distance by
    roughly (sigma/mu)^2. The bridge stamps every message with
    `get_clock().now()` at socket receipt, so dt carries the full WebSocket +
    Unity frame jitter; at 18 Hz nominal, 15% jitter is ~2% systematic OVERREAD
    -- larger than the 1.55% wheel-radius bias corrected below, and systematic,
    which is exactly what AMCL's zero-mean alpha noise cannot represent.

  * Dropped travel. The old dt sanity guard returned AFTER `self._enc[side]` had
    already been overwritten, so a stale or duplicate frame permanently lost that
    angle delta -- biasing position SHORT.

So the angle delta is now accumulated directly, and dt survives only where it is
harmless: computing the reported `twist.linear.x`, which nothing integrates.
"""

import math
from collections import deque

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from roboracer_stack.common.tire_model import TireSpeedObserver
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32

NS = '/autodrive/roboracer_1'
QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DeadReckoning(Node):

    def __init__(self):
        super().__init__('dead_reckoning')

        p = self.declare_parameter
        # MEASURED 0.0581, not the published 0.0590: path-integration gave
        # 0.05815 and steady-state speed matching 0.05813. The spec value
        # overreads distance by 1.55%, which is ~0.43 m of phantom travel per
        # 27.7 m lap -- a SYSTEMATIC bias, which is exactly what AMCL's
        # zero-mean alpha noise cannot represent. See rl_racer/rl_racer/config.py.
        p('wheel_radius', 0.0581)
        p('odom_frame', 'odom')
        p('base_frame', 'roboracer_1')  # devkit's name, so `lidar` still parents to it
        # 200 Hz, not 50. MEASURED: scans land uniformly in the gap between odom
        # transforms, so at 50 Hz (20 ms) 96% of them are stamped AHEAD of the
        # newest one. AMCL must look up odom->base AT the scan timestamp to
        # build map->odom; a mistimed lookup corrupts that transform while the
        # published /amcl_pose stays clean. 200 Hz cuts the worst case to 5 ms.
        #
        # This is pure RESAMPLING of the integrated pose -- the timer rate no
        # longer affects how far the car thinks it has travelled.
        p('publish_rate', 200.0)
        p('publish_tf', True)
        # Post-date the TF stamp so a consumer asking for "now" always lands
        # INSIDE the transform window rather than extrapolating past its end.
        # nav2's own amcl does the same for map->odom, with 0.5 s. The cost is
        # that the pose reported at stamp t is really the pose from t-tolerance;
        # 20 ms is small next to the bridge's own WebSocket latency, and far
        # smaller than the error a dropped scan causes.
        p('transform_tolerance', 0.02)
        # Encoders overread under slip. <1.0 trims the systematic part; AMCL
        # handles what is left.
        # 'encoder': arc length from the cumulative wheel angle, which is what
        # this sim's encoders report -- i.e. the THROTTLE, not the car. Every
        # slip goes straight into the position: measured encoder/true distance
        # 1.02 over a lap, and up to 1.01 m of ALONG-TRACK error carried into
        # the hairpin-1 braking point on a run started from the spawn (runs 3,
        # 13, 14), which is what the follower's warmup cap exists to mask.
        # 'tire': run the sim's own longitudinal model on the measured wheel
        # speed and integrate the CAR's speed instead, over the same stamp
        # interval the wheel speed was measured on so the sum stays unbiased.
        # Ported from the iros_compete_usman branch (common/tire_model.py).
        # Default stays 'encoder': 'tire' is an A/B, not a silent switch.
        # 'slip': the encoder arc length MINUS the part of it that is wheelspin,
        #     ds = dang * r  -  k * max(u_cmd - v_car, 0) * dt   while |yaw rate| < gate
        # u_cmd is our own throttle command as a wheel speed (legal: it is our
        # output), v_car the tire observer on the encoder rate. 'tire' is the
        # k = 1 end of this and over-corrects (straight +5.3 % -> -3.0 %, A03).
        # k replayed through THIS function on six logs, 45 Hz and 20 Hz alike
        # (A03, S01, S03, mtb15, edit_2, truth): the straight-zeroing k is
        # 0.22-0.33, and 0.27 takes the straight over-read from +5.3..+6.8 % to
        # within +-1 %, corners and whole lap within +-1.3 %. Ungated, corners
        # go to -1..-3.5 %: through a corner the car carries lateral slip the
        # longitudinal model does not see, so the gate leaves them on the
        # encoder, within 1 % there already. u is the COMMAND, not the encoder
        # rate: fitted on the encoder rate k scattered 0.64-1.2 between runs.
        p('distance_source', 'encoder')
        p('slip_k', 0.27)
        # The BRAKING side, added after M02 (2026-09-18): with the wheel slower
        # than the car the encoder UNDER-reads, -6.3..-9.1 % through the hairpin-1
        # braking zone (s 12.5-17.5) on all eight logs. The straight's over-read
        # used to cancel it by luck; with that removed the estimate fell 0.2-0.7 m
        # BEHIND the car into hairpin 1, the car braked late, reached the apex up
        # to +0.77 m/s fast and ran wide (both M02 contacts). The observer's speed
        # is right under braking, so the term adds max(v_car - u_cmd, 0) * dt back;
        # fitted per run 1.03-1.20 on the 45 Hz logs (1.03-1.48 at 20 Hz).
        p('slip_k_brake', 1.1)
        p('slip_yaw_gate', 0.5)                     # rad/s
        p('u_per_throttle', 25.25)                  # as pure_pursuit.yaml
        # The observer's input rate spans at least this long: a per-sample rate
        # at 45 Hz scatters 2x and runs the observer low (pure_pursuit's
        # enc_rate_window_s, same finding).
        p('slip_rate_window_s', 0.05)
        # A STOPPED WHEEL GETS NO SLIP CORRECTION. The braking term adds
        # max(v_car - u_cmd, 0) * dt back, with v_car the tire observer; after
        # a wall contact the simulator teleports the car and zeroes its
        # velocity, the follower is held and the command drops to 0, the
        # wheel reads 0 -- and the observer, which only knows the wheel, decays
        # from 6 m/s over about a second. That decay was integrated as travel:
        # 3.3 m of phantom motion along the heading while the car sat still
        # (lv_fast_19_1, 2026-09-19, 14.35-15.45 s), which dragged the
        # localizer's estimate 2.2 m off and cost the first recovery seed.
        # Below this wheel speed the slip model has nothing to say (it was
        # fitted on a rolling wheel inside the follower's slip band; a locked
        # wheel at speed is never commanded), so the encoder is taken as is.
        p('slip_wheel_min_m_s', 0.3)
        p('tire_rise_slope', 3.0)                   # as pure_pursuit.yaml
        p('v_slip_den', 4.0)
        p('distance_scale', 1.0)
        # Reset detector, in wheel radians. Cumulative angle survives dropped
        # frames, so the ONLY delta worth rejecting is a discontinuity -- i.e.
        # the sim resetting the counter. Arithmetic at the measured 24 m/s top
        # speed and r=0.0581 (wheel rate 413 rad/s):
        #     one 18 Hz tick        ~23 rad
        #     a 0.5 s stall        ~206 rad
        #     a full 27.7 m lap    ~477 rad   <- what a reset-to-zero looks like
        # 300 sits above any plausible gap and below a lap's accumulation.
        # A reset also invalidates AMCL's pose, so re-seed the filter too.
        p('max_wheel_step', 300.0)
        # Extrapolate the IMU heading to each transform's stamp with the IMU's own
        # yaw rate. Otherwise the heading in odom->base lags the scan by up to one
        # 18 Hz tick: the (post-dated) transform that covers scan k was published
        # before tick k's messages arrived, so it carries heading k-1. At the
        # S-curve's 2.7 rad/s that is 8 deg, and AMCL absorbs it by rotating
        # map->odom -- measured as +-8..14 deg swings of m2o_yaw with 0.5-1.1 m of
        # cross error at s ~ 20 m on every run, i.e. the wall hits there. Over the
        # <= 75 ms involved a measured rate is good to ~0.01 rad.
        p('extrapolate_yaw', True)
        # The same for POSITION. The transform that covers scan k carries the
        # integrated position of frame k-1 as well, so AMCL pairs each scan with
        # odometry one frame old and pushes map->odom forward by a frame of
        # travel wherever the walls constrain the along-track direction.
        # Measured (runs 16-22, 13-18 Hz alike): the estimate LEADS the true
        # position by 35-40 ms x speed, 0.30 m at 6.5 m/s, and shifting the
        # truth by 40 ms takes the along error from 0.21 to 0.08 m rms with no
        # residual offset. Carrying the position forward by the wheel speed to
        # each stamp removes the mismatch. Capped at 0.1 s like the heading.
        p('extrapolate_pos', True)

        g = lambda n: self.get_parameter(n).value
        self.wheel_r = g('wheel_radius')
        self.odom_frame, self.base_frame = g('odom_frame'), g('base_frame')
        self.publish_tf = g('publish_tf')
        self.scale = g('distance_scale')
        self.dist_src = str(g('distance_source')).lower()
        self._tire = TireSpeedObserver(rise_slope=float(g('tire_rise_slope')),
                                       v_slip_den=float(g('v_slip_den')))
        if self.dist_src not in ('encoder', 'tire', 'slip'):
            raise RuntimeError(f"distance_source must be encoder, tire or slip, not {self.dist_src!r}")
        self.slip_k = float(g('slip_k'))
        self.slip_k_brake = float(g('slip_k_brake'))
        self.slip_gate = float(g('slip_yaw_gate'))
        self.u_per_thr = float(g('u_per_throttle'))
        self.slip_win = float(g('slip_rate_window_s'))
        self.slip_wheel_min = float(g('slip_wheel_min_m_s'))
        self._obs = {s: TireSpeedObserver(rise_slope=float(g('tire_rise_slope')),
                                          v_slip_den=float(g('v_slip_den'))) for s in ('l', 'r')}
        self._hist = {'l': deque(maxlen=16), 'r': deque(maxlen=16)}
        self._u_cmd = 0.0
        self.tf_tol = float(g('transform_tolerance'))
        self.max_step = float(g('max_wheel_step'))
        self.extrapolate_yaw = bool(g('extrapolate_yaw'))
        self.extrapolate_pos = bool(g('extrapolate_pos'))
        self._imu_t = None              # stamp of the heading in self.yaw
        self._enc_t = None              # stamp of the newest encoder message banked into x, y

        self.x = self.y = 0.0
        self.yaw = None                 # from IMU; None until the first message
        self.speed = 0.0
        self.yaw_rate = 0.0
        self._enc = {}                  # side -> (angle, stamp)
        self._ds = {}                   # side -> metres of arc awaiting integration
        self._rate = {}                 # side -> m/s   (reporting only)
        self._last = None
        self._resets = 0

        self.create_subscription(JointState, f'{NS}/left_encoder',
                                 lambda m: self._cb_enc('l', m), QOS)
        self.create_subscription(JointState, f'{NS}/right_encoder',
                                 lambda m: self._cb_enc('r', m), QOS)
        self.create_subscription(Imu, f'{NS}/imu', self._cb_imu, QOS)
        if self.dist_src == 'slip':
            self.create_subscription(Float32, f'{NS}/throttle_command', self._cb_thr, QOS)

        self.pub = self.create_publisher(Odometry, 'odom', QOS)
        self.tfb = tf2_ros.TransformBroadcaster(self)
        self.create_timer(1.0 / g('publish_rate'), self._tick)

        self.get_logger().info(
            f'dead reckoning: {self.odom_frame} -> {self.base_frame}, '
            f'wheel r={self.wheel_r} m, scale={self.scale}, '
            f'{g("publish_rate"):.0f} Hz, tf +{self.tf_tol * 1e3:.0f} ms, '
            f'extrapolate yaw={self.extrapolate_yaw} pos={self.extrapolate_pos}, '
            f'distance from {self.dist_src}'
            + (f' (k={self.slip_k}, k_brake={self.slip_k_brake}, yaw gate {self.slip_gate} rad/s)'
               if self.dist_src == 'slip' else ''))

    def _cb_enc(self, side, msg):
        """position is cumulative wheel angle in RADIANS (measured, not ticks)."""
        if not msg.position:
            return
        ang = float(msg.position[0])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        prev = self._enc.get(side)
        self._enc[side] = (ang, t)
        if prev is None:
            self._ds.setdefault(side, 0.0)
            return

        dang = ang - prev[0]

        # ---- distance: cumulative, so dt is irrelevant and must stay out ----
        if abs(dang) > self.max_step:
            # Counter discontinuity, not travel. Skip it; _enc is already
            # resynced above, so the next delta is measured from the new origin.
            self._resets += 1
            self.get_logger().warn(
                f'{side} encoder jumped {dang:.1f} rad (> {self.max_step:.0f}); '
                f'treating as a reset, not travel. AMCL needs re-seeding.')
            return
        dt = t - prev[1]
        if self.dist_src == 'slip':
            self._ds[side] = self._ds.get(side, 0.0) + dang * self.wheel_r - self._slip_ds(side, ang, t, dt)
        elif self.dist_src != 'tire':
            self._ds[side] = self._ds.get(side, 0.0) + dang * self.wheel_r
        elif 1e-4 < dt <= 0.5:
            # The observer's CAR speed over the same interval the wheel speed was
            # measured on. Those intervals telescope, so the sum is unbiased;
            # holding a rate across a different interval inflates it instead.
            self._ds[side] = self._ds.get(side, 0.0) + self._tire.step(
                abs(dang) / dt * self.wheel_r, dt) * dt * (1.0 if dang >= 0.0 else -1.0)
        self._enc_t = t if self._enc_t is None else max(self._enc_t, t)

        # ---- speed: reported in twist.linear.x, never integrated ----
        if dt <= 1e-4 or dt > 0.5:
            return          # stale or duplicate stamp; a bad dt gives garbage
        self._rate[side] = dang / dt * self.wheel_r
        rates = [v for v in self._rate.values() if v is not None]
        if rates:
            self.speed = float(np.mean(rates)) * self.scale

    def _slip_ds(self, side, ang, t, dt):
        """Metres of wheelspin in this encoder interval (see distance_source)."""
        h = self._hist[side]
        h.append((ang, t))
        if not (1e-4 < dt <= 0.5):
            return 0.0
        # rate over the newest span >= slip_rate_window_s
        a0, t0 = h[0]
        for a_old, t_old in reversed(h):
            if t - t_old >= self.slip_win:
                a0, t0 = a_old, t_old
                break
        if t - t0 <= 1e-4:
            return 0.0
        u_wheel = abs(ang - a0) / (t - t0) * self.wheel_r
        v_car = self._obs[side].step(u_wheel, dt)
        if abs(self.yaw_rate) >= self.slip_gate or u_wheel < self.slip_wheel_min:
            return 0.0
        return (self.slip_k * max(self._u_cmd - v_car, 0.0)
                - self.slip_k_brake * max(v_car - self._u_cmd, 0.0)) * dt

    def _cb_thr(self, msg):
        self._u_cmd = max(0.0, float(msg.data)) * self.u_per_thr

    def _cb_imu(self, msg):
        self.yaw = yaw_from_quat(msg.orientation)
        self.yaw_rate = msg.angular_velocity.z
        self._imu_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _yaw_at(self, t):
        """Heading at time t: the last IMU heading carried forward by its rate.

        Capped at 0.1 s so a stalled IMU cannot spin the estimate.
        """
        if not self.extrapolate_yaw or self._imu_t is None:
            return self.yaw
        return self.yaw + self.yaw_rate * max(0.0, min(t - self._imu_t, 0.1))

    def _pos_at(self, t):
        """Position at time t: the integrated position carried forward by the
        wheel speed along the heading, see extrapolate_pos. Capped at 0.1 s so
        a stalled encoder cannot run the estimate away."""
        if not self.extrapolate_pos or self._enc_t is None:
            return self.x, self.y
        dt = max(0.0, min(t - self._enc_t, 0.1))
        yaw = self._yaw_at(self._enc_t + 0.5 * dt)
        return (self.x + self.speed * dt * math.cos(yaw),
                self.y + self.speed * dt * math.sin(yaw))

    def _consume_ds(self):
        """Metres travelled since the last tick, averaged over the wheels.

        Both encoders are published inside the same bridge handler, so they
        normally land together. If a tick happens to fall between them the
        update is split across two ticks -- mean(dl, 0) then mean(0, dr) -- which
        still sums to the correct mean(dl, dr); only the yaw used differs, by one
        5 ms tick.
        """
        if not self._ds:
            return 0.0
        ds = float(np.mean(list(self._ds.values())))
        for side in self._ds:
            self._ds[side] = 0.0
        return ds * self.scale

    def _tick(self):
        if self.yaw is None:
            return
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self._last is None:
            self._last = t
            self._consume_ds()      # drop pre-roll travel rather than banking it
            return
        self._last = t

        # Integrate encoder ARC LENGTH along the IMU heading. No bicycle model
        # needed: the IMU already gives the true heading each step. No dt either
        # -- the distance was measured, not inferred from a rate.
        ds = self._consume_ds()
        self.x += ds * math.cos(self.yaw)
        self.y += ds * math.sin(self.yaw)

        stamp = now.to_msg()
        # TF is post-dated; the Odometry message keeps the true stamp, since
        # nothing looks that up by time.
        tf_stamp = (now + rclpy.duration.Duration(
            seconds=self.tf_tol)).to_msg() if self.tf_tol > 0.0 else stamp
        # Heading AT the stamp each message carries, not the heading of the last
        # IMU tick: see extrapolate_yaw. The position integration above keeps the
        # measured heading, because the encoder distance it moves was measured
        # in the same bridge tick as that heading.
        t_tf = t + (self.tf_tol if self.tf_tol > 0.0 else 0.0)
        half_tf = self._yaw_at(t_tf) / 2.0
        half_od = self._yaw_at(t) / 2.0
        x_tf, y_tf = self._pos_at(t_tf)
        x_od, y_od = self._pos_at(t)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = tf_stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x_tf
            tf.transform.translation.y = y_tf
            tf.transform.rotation.z, tf.transform.rotation.w = math.sin(half_tf), math.cos(half_tf)
            self.tfb.sendTransform(tf)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y = x_od, y_od
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(half_od), math.cos(half_od)
        od.twist.twist.linear.x = self.speed
        od.twist.twist.angular.z = self.yaw_rate
        # Position is dead-reckoned and drifts; heading comes from an absolute
        # sensor. Say so, so AMCL weights them sensibly.
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
