#!/usr/bin/env python3

"""Get AMCL a starting pose, CONFIRM it took, then hand over to the follower.

Two modes:

  mode:=truth   (default)  Read /ips and /imu ONCE, seed AMCL with that pose,
                           verify AMCL actually adopted it, and hand over. Ground
                           truth is RESTRICTED at race time, so this is a
                           development shortcut -- but only for the first instant.
                           After the seed, tracking is lidar + map + dead
                           reckoning alone, which is the part actually worth
                           testing.

  mode:=global             No initial pose at all. Scatter particles across the
                           map, creep forward on lidar alone, and wait for the
                           cloud to collapse. This is how a real race starts, and
                           it is slower and less reliable -- one stretch of
                           corridor looks much like another, so it has to reach a
                           corner before the ambiguity breaks.

Either way it latches /localization_ready when done, which the follower waits on
instead of a fixed timer.

WHY THIS IS A HANDSHAKE AND NOT A PUBLISH
-----------------------------------------
AMCL is a nav2 LIFECYCLE node. It starts UNCONFIGURED and only reaches `active`
once nav2_lifecycle_manager has transitioned it, which takes a second or two.
Anything sent to /initialpose before that is silently DISCARDED -- amcl logs
"Received initial pose request, but AMCL is not yet in the active state" and
returns. The publisher here is VOLATILE, so there is no late-joiner redelivery
either: the message is simply gone.

Every node in localization.launch.py starts at the same instant, so whether the
seed lands used to be a pure race between the bridge's first /ips message and
nav2's lifecycle transition -- and the bridge usually won. When the seed was
lost, AMCL fell back to its `initial_pose` parameter and this node declared
/localization_ready anyway, handing the follower a pose that had been wrong
since startup.

So the sequence is now:

    1. poll /<amcl_node>/get_state until AMCL reports PRIMARY_STATE_ACTIVE
    2. publish the seed to /initialpose
    3. nudge AMCL into running an update (it is stationary in truth mode, and
       AMCL only resamples once it believes it has moved)
    4. compare the next /amcl_pose against what was sent; re-seed if it differs
    5. only then latch /localization_ready

If the seed cannot be confirmed, this node REFUSES to hand over rather than
letting the follower drive on a pose nobody checked. Set
require_convergence:=false to restore the old hand-over-anyway behaviour.

The seed assumes map coordinates match the simulator's world frame. They do
here: the SLAM run had scan matching disabled, so slam_toolbox's map->world
correction was identity.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
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


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat_xyzw(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


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
        p('wheel_radius', 0.0581)        # MEASURED; see dead_reckoning.py
        # ---- seed handshake (see the module docstring) ----
        p('amcl_node', 'amcl')           # lifecycle node to query for readiness
        p('amcl_wait_s', 30.0)           # give up if it never becomes active
        p('seed_tolerance_m', 0.30)      # how far AMCL may land from the seed
        p('seed_tolerance_deg', 15.0)
        p('max_seed_attempts', 5)
        # Refuse to latch /localization_ready unless the pose was actually
        # confirmed. False restores the old "hand over regardless" behaviour.
        p('require_convergence', True)

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
        self.amcl_wait = float(g('amcl_wait_s'))
        self.tol_m = float(g('seed_tolerance_m'))
        self.tol_yaw = math.radians(float(g('seed_tolerance_deg')))
        self.max_attempts = int(g('max_seed_attempts'))
        self.require_convergence = bool(g('require_convergence'))
        self.truth_pos = None
        self.truth_quat = None
        self.seeded_at = None

        self.scan = None
        self.ready = False               # /localization_ready has been latched
        self.finished = False            # this node is done, one way or the other
        self.distance = 0.0
        self._enc = {}
        self._rate = {}
        self.speed = 0.0
        self.pos_std = None
        self.yaw_std = None
        self.est = None                  # (x, y, yaw) from the latest /amcl_pose
        self.est_t = None                # when that estimate arrived
        self.seed_pose = None            # (x, y, yaw) of the last seed sent
        self.attempts = 0
        # Error of the latest /amcl_pose against the truth AT THAT INSTANT.
        # Verification must not compare against the seed: the seed is a snapshot
        # from settle_s ago, so if the car is moving (an RL policy driving, say)
        # the "disagreement" is just distance travelled -- 3.7 m/s x 2 s = 7.5 m
        # of pure motion, which fails every tolerance no matter how well AMCL
        # is actually tracking.
        self.est_err = None
        self.est_dyaw = None
        self.speed_est = 0.0
        self.amcl_active = False
        self.scattered = False
        self._state_pending = False
        self._state_last = 0.0
        self._nudge_last = 0.0
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

        amcl = str(g('amcl_node')).strip('/')
        self._state_cli = self.create_client(GetState, f'/{amcl}/get_state')
        # Forces a filter update without motion. Best effort: if this build of
        # amcl does not expose it, verification just waits for a real scan
        # update instead.
        self._nomotion_cli = self.create_client(Empty, '/request_nomotion_update')

        self._announce(False)
        if self.mode == 'truth':
            self.get_logger().warn(
                'mode=truth: seeding the initial pose from /ips + /imu, which are '
                'RESTRICTED at race time. Tracking afterwards is race-legal.')
        self.get_logger().info(f'waiting for {amcl} to reach the active state '
                               'before sending anything')

        self.create_timer(0.05, self._tick)
        self.create_timer(2.0, self._report)

    # ---- setup -----------------------------------------------------------

    def _poll_amcl_state(self, now):
        """Ask the lifecycle node whether it is active yet. Non-blocking."""
        if self._state_pending or now - self._state_last < 0.5:
            return
        if not self._state_cli.service_is_ready():
            return
        self._state_last = now
        self._state_pending = True
        self._state_cli.call_async(GetState.Request()).add_done_callback(
            self._on_state)

    def _on_state(self, future):
        self._state_pending = False
        try:
            res = future.result()
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().debug(f'get_state failed: {exc}')
            return
        if res is not None and res.current_state.id == State.PRIMARY_STATE_ACTIVE:
            self.amcl_active = True
            self.get_logger().info(
                'AMCL is ACTIVE -- an initial pose will now be accepted')

    def _scatter(self):
        cli = self.create_client(Empty, '/reinitialize_global_localization')
        if not cli.wait_for_service(timeout_sec=5.0):
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
        """Publish the true pose to /initialpose and remember what was sent."""
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
        yaw = yaw_from_quat_xyzw((qx, qy, qz, qw))
        self.seed_pose = (x, y, yaw)
        self.seeded_at = self.get_clock().now().nanoseconds * 1e-9
        self.attempts += 1
        self.get_logger().info(
            f'seed {self.attempts}/{self.max_attempts}: x={x:+.3f} y={y:+.3f} '
            f'yaw={math.degrees(yaw):+.1f} deg -- awaiting confirmation')

    def _cb_pose(self, msg):
        c = msg.pose.covariance
        self.pos_std = math.sqrt(max(c[0], 0.0) + max(c[7], 0.0))
        self.yaw_std = math.sqrt(max(c[35], 0.0))
        q = msg.pose.pose.orientation
        self.est = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                    yaw_from_quat_xyzw((q.x, q.y, q.z, q.w)))
        self.est_t = self.get_clock().now().nanoseconds * 1e-9
        # Score it against truth NOW, while both are current. Valid whether the
        # car is parked or lapping at racing speed.
        if self.truth_pos is not None and self.truth_quat is not None:
            self.est_err = math.hypot(self.est[0] - self.truth_pos[0],
                                      self.est[1] - self.truth_pos[1])
            self.est_dyaw = abs(wrap(self.est[2]
                                     - yaw_from_quat_xyzw(self.truth_quat)))

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

    def _nudge(self, now):
        """Ask AMCL to run an update even though the car has not moved.

        In truth mode nothing drives, and AMCL only resamples (and therefore
        only republishes /amcl_pose) once the odometry says it has travelled
        update_min_d. Without this the confirmation step would wait forever.
        Best effort -- skipped silently if the service is not offered.
        """
        if now - self._nudge_last < 0.4 or not self._nomotion_cli.service_is_ready():
            return
        self._nudge_last = now
        self._nomotion_cli.call_async(Empty.Request())

    def _converged(self):
        return (self.pos_std is not None
                and self.pos_std <= self.pos_target
                and self.yaw_std <= self.yaw_target)

    def _tick(self):
        if self.finished:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last is not None:
            self.distance += abs(self.speed) * (now - self._last)
        self._last = now

        # Nothing may be sent to AMCL until it is genuinely active -- see the
        # module docstring. This is the whole point of the handshake.
        if not self.amcl_active:
            self._poll_amcl_state(now)
            if not self.amcl_active and now - self.t0 > self.amcl_wait:
                self._fail(f'AMCL never reached the active state within '
                           f'{self.amcl_wait:.0f} s. Is nav2_lifecycle_manager '
                           'running, and did map_server come up?')
            return

        if self.mode == 'truth':
            self._tick_truth(now)
        else:
            self._tick_global(now)

    def _tick_truth(self, now):
        if self.truth_pos is None or self.truth_quat is None:
            return                      # still waiting for the first /ips and /imu
        if self.seeded_at is None:
            self._seed()
            return

        if abs(self.speed) < 0.3:
            self._nudge(now)     # parked: AMCL needs a shove to run an update
        if now - self.seeded_at < self.settle_s:
            return

        fresh = (self.est is not None and self.est_t is not None
                 and self.est_t >= self.seeded_at and self.est_err is not None)
        if fresh:
            gap, dyaw = self.est_err, self.est_dyaw
            if gap <= self.tol_m and dyaw <= self.tol_yaw:
                std = f'{self.pos_std:.3f} m' if self.pos_std is not None else 'unknown'
                moving = ' while driving' if abs(self.speed) > 0.3 else ''
                self._finish(
                    f'seed CONFIRMED on attempt {self.attempts}{moving}: AMCL is '
                    f'{gap * 100:.1f} cm / {math.degrees(dyaw):.1f} deg from truth '
                    f'(pos std {std}). Tracking is now lidar + map + dead '
                    'reckoning only.')
                return
            reason = (f'AMCL is {gap:.2f} m / {math.degrees(dyaw):.1f} deg from '
                      f'TRUTH, tolerance {self.tol_m:.2f} m / '
                      f'{math.degrees(self.tol_yaw):.1f} deg '
                      f'(speed {abs(self.speed):.1f} m/s)')
        else:
            reason = 'no /amcl_pose arrived after the seed'

        if self.attempts < self.max_attempts:
            self.get_logger().warn(f'seed {self.attempts} not adopted -- {reason}; re-seeding')
            self._seed()
        else:
            self._fail(f'seed REJECTED after {self.attempts} attempts -- {reason}')

    def _tick_global(self, now):
        if not self.scattered:
            self._scatter()
            self.scattered = True
            return

        if self._converged():
            self._finish(f'converged after {self.distance:.2f} m '
                         f'(pos std {self.pos_std:.3f} m, '
                         f'yaw std {math.degrees(self.yaw_std):.1f} deg)')
            return

        if now - self.t0 > self.timeout:
            std = f'{self.pos_std:.3f}' if self.pos_std is not None else 'nan'
            self._fail(f'TIMEOUT after {self.timeout:.0f} s -- pos std {std} m never '
                       f'reached {self.pos_target} m. The pose is NOT localized; '
                       'check the scan against the map.')
            return

        if self.scan is not None:
            self._drive()

    def _finish(self, message):
        self.finished = True
        self.ready = True
        self._send(0.0, 0.0)
        self._announce(True)
        self.get_logger().info(message)
        self.get_logger().info('/localization_ready = true, follower may take over')

    def _fail(self, message):
        """Give up. By default do NOT let the follower drive on this pose."""
        self.finished = True
        self._send(0.0, 0.0)
        self.get_logger().error(message)
        if self.require_convergence:
            self.get_logger().error(
                'NOT latching /localization_ready -- the follower stays parked on '
                'purpose. Fix the cause, or pass require_convergence:=false to '
                'hand over anyway.')
        else:
            self.ready = True
            self._announce(True)
            self.get_logger().warn(
                'require_convergence:=false -- handing over an UNCONFIRMED pose')

    def _report(self):
        if self.finished:
            return
        if not self.amcl_active:
            self.get_logger().info(
                f'waiting for AMCL to become active '
                f'({self.get_clock().now().nanoseconds * 1e-9 - self.t0:.0f} s '
                f'of {self.amcl_wait:.0f} s)')
            return
        if self.mode == 'truth' and self.seeded_at is None:
            self.get_logger().info(
                f'waiting for ground truth: ips={"ok" if self.truth_pos else "--"} '
                f'imu={"ok" if self.truth_quat else "--"}')
            return
        if self.mode == 'truth':
            self.get_logger().info(
                f'seed {self.attempts}/{self.max_attempts} sent, awaiting a fresh '
                '/amcl_pose to confirm it')
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
