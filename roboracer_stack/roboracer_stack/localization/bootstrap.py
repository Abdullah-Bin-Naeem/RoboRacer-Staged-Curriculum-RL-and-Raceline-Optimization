#!/usr/bin/env python3

"""Get the localizer a starting pose, CONFIRM it took, then hand over.

Serves BOTH localizers. Everything AMCL-shaped is a parameter, so slam_toolbox
reuses this handshake rather than getting a second copy of it -- see
"THE TWO LOCALIZERS" below.

Three modes:

  mode:=truth   (default)  Read /ips and /imu ONCE, seed the localizer with that
                           pose, verify it actually adopted it, release /ips,
                           and hand over. RACE-LEGAL: the first lap is a warmup
                           and the timer starts after it, so a single read
                           before the car moves sits inside the permitted
                           window -- see common/restricted.py, THE WARMUP
                           WINDOW. The subscription is destroyed by
                           _release_truth() as soon as the seed resolves, so
                           the claim is enforced rather than asserted. After
                           that, tracking is lidar + map + dead reckoning
                           alone.

  mode:=global             No initial pose at all. Scatter particles across the
                           map, creep forward on lidar alone, and wait for the
                           cloud to collapse. Reads no ground truth at any
                           point, but it is slower and less reliable -- one
                           stretch of corridor looks much like another, so it
                           has to reach a corner before the ambiguity breaks.
                           AMCL ONLY: slam_toolbox has no global search.

  mode:=spawn              Seed from the MEASURED spawn constant (spawn_x/y/yaw,
                           defaulting to common/frames SPAWN_*) plus the IMU's
                           absolute heading, confirm the localizer adopted it,
                           and hand over. Reads NO restricted topic at any
                           point: the position is a constant measured offline
                           and the IMU is a permitted sensor. The practice
                           spawn is fixed to 2 mm, so this is as exact as
                           truth. Works for both localizers. THE RACE DEFAULT.

Either way it latches /localization_ready when done, which the follower waits on
instead of a fixed timer.

WHICH MODE TO RACE
------------------
spawn. It is exact on a track whose spawn has been measured, it works for both
localizers, and nothing in it can be mistaken for ground-truth use by a steward
reading the ROS graph. truth is the organizer-confirmed alternative (restricted
topics may be read in the warm-up lap); keep it for the day the spawn constant
is stale. global exists to prove the stack can start with no prior at all.

WHY THIS IS A HANDSHAKE AND NOT A PUBLISH
-----------------------------------------
AMCL is a nav2 LIFECYCLE node. It starts UNCONFIGURED and only reaches `active`
once nav2_lifecycle_manager has transitioned it, which takes a second or two.
Anything sent to /initialpose before that is silently DISCARDED -- amcl logs
"Received initial pose request, but AMCL is not yet in the active state" and
returns. The publisher here is VOLATILE, so there is no late-joiner redelivery
either: the message is simply gone.

Every node in amcl.launch.py starts at the same instant, so whether the
seed lands used to be a pure race between the bridge's first /ips message and
nav2's lifecycle transition -- and the bridge usually won. When the seed was
lost, AMCL fell back to its `initial_pose` parameter and this node declared
/localization_ready anyway, handing the follower a pose that had been wrong
since startup.

So the sequence is now:

    1. wait for the localizer to come up -- get_state for AMCL, the appearance
       of map->odom for slam_toolbox (see THE TWO LOCALIZERS below)
    2. publish the seed to /initialpose
    3. nudge AMCL into running an update (it is stationary in truth mode, and
       AMCL only resamples once it believes it has moved)
    4. compare the next estimate against what was sent; re-seed if it differs
    5. score the lidar scan against the map AT that estimate (see below)
    6. only then latch /localization_ready

If the seed cannot be confirmed, this node REFUSES to hand over rather than
letting the follower drive on a pose nobody checked. Set
require_convergence:=false to restore the old hand-over-anyway behaviour.

CHECKING THE SEED AGAINST THE WORLD, NOT AGAINST ITSELF
-------------------------------------------------------
In spawn mode there is no truth, so step 4 compares AMCL with the seed it was
just given. The particles start inside 5 cm / 3 deg of that seed and resampling
cannot carry them far, so a stale SPAWN_* or a map that does not line up with
the simulator's frame used to pass step 4 anyway. Step 5 asks the one
independent witness there is: at the confirmed pose, what fraction of lidar
beams end within scan_match_tol_m of a wall on the map? A right pose scores
near 1; a pose 0.3 m or a few degrees out scores far lower (scan_match_min).

AFTER HAND-OVER: THE RESPAWN WATCH
----------------------------------
A wall contact respawns the car at the last checkpoint with its velocity
zeroed. The encoders do not reset, but the IMU heading jumps by more than the
yaw rate can explain -- the signature pure_pursuit already uses to reset its
speed estimate. Nothing told AMCL, whose particles stayed at the wall. So this
node stays subscribed after hand-over and, on that signature, re-seeds AMCL on
the racing line behind the last estimate, at the nearest stretch whose heading
matches the new IMU heading, with a covariance stretched along the track over
that stretch. Race-legal: IMU, the shipped raceline, AMCL's own last estimate.
A respawn that keeps the heading (mid-straight) is not detectable this way.

THE TWO LOCALIZERS
------------------
Only AMCL ships on this branch; the slam_toolbox column is kept because the
handshake is written against these parameters, not against AMCL.

                      AMCL                        slam_toolbox
  ready_check         lifecycle (get_state)       tf (map->odom appears)
  estimate_topic      /amcl_pose                  /pose
  verify_via_tf       false                       true
  nudge_service       /request_nomotion_update    none
  global_service      /reinitialize_global_...    NONE -- see below

slam_toolbox is not a lifecycle node, so there is no state to poll. What it does
instead is publish map->odom only once it has processed its FIRST scan -- which
is also the exact moment its localizePoseCallback stops early-returning on
`processor_type_ != PROCESS_LOCALIZATION`. So the transform appearing IS the
handshake, and ready_check:=tf waits for precisely the right thing.

There is NO global mode for slam_toolbox. Its only entry points are
START_AT_FIRST_NODE / START_AT_GIVEN_POSE / LOCALIZE_AT_POSE
(DeserializePoseGraph.srv) plus /initialpose -- every one of them takes a pose.
It is a single-hypothesis scan matcher, not a particle filter, so there is
nowhere to put the competing hypotheses a map-wide search produces. With
global_service:='' this node says so and falls back to the localizer's
configured map_start_pose.

THAT FALLBACK IS WHY THE SEED IS LOGGED VERBATIM. mode:=spawn seeds from
roboracer_stack.common.frames.SPAWN_* directly, and a slam run with no seed at
all starts on the same constant. If that constant is wrong the error is frozen
for the whole run: slam_toolbox seeds with a +-0.5 m correlative search
(correlation_search_space_dimension: 1.0) and cannot recover a larger one. So
every truth-mode seed prints a paste-ready SPAWN_X/Y/YAW block -- run once in
dev, paste into frames.py, and spawn mode inherits a measured spawn instead of
a guess.

The seed assumes map coordinates match the simulator's world frame. They do
here: the SLAM run had scan matching disabled, so slam_toolbox's map->world
correction was identity.
"""

import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time
from roboracer_stack.common import restricted
from roboracer_stack.common.frames import NS as COMMON_NS
from roboracer_stack.common.frames import (DEFAULT_RACELINE, LIDAR_XYZ, SPAWN_X,
                                           SPAWN_Y, SPAWN_YAW)
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


def near_wall_mask(data, width, height, tol_cells):
    """Boolean grid, row = y index as in OccupancyGrid: True within tol_cells
    (a square neighbourhood) of an occupied cell. Box sum by 2-D cumsum."""
    occ = np.asarray(data, dtype=np.int16).reshape(height, width) >= 65
    k = max(0, int(tol_cells))
    c = np.pad(occ.astype(np.int32), ((k + 1, k), (k + 1, k))).cumsum(0).cumsum(1)
    n = 2 * k + 1
    box = c[n:, n:] - c[:-n, n:] - c[n:, :-n] + c[:-n, :-n]
    return box > 0


def scan_match_score(mask, resolution, origin_xy, scan, pose, lidar_x,
                     max_range, stride=4):
    """Fraction of valid beams, cast from `pose` (x, y, yaw of the base frame),
    that end on `mask`. None if fewer than 20 beams are usable. Angles follow
    REP-103 (angle_min + i * increment, counter-clockwise), as AMCL assumes."""
    x, y, yaw = pose
    r = np.asarray(scan.ranges, dtype=float)
    a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    r, a = r[::stride], a[::stride]
    ok = np.isfinite(r) & (r > scan.range_min) & (r < min(scan.range_max, max_range))
    if ok.sum() < 20:
        return None
    lx, ly = x + lidar_x * math.cos(yaw), y + lidar_x * math.sin(yaw)
    ex = lx + r[ok] * np.cos(yaw + a[ok])
    ey = ly + r[ok] * np.sin(yaw + a[ok])
    col = np.floor((ex - origin_xy[0]) / resolution).astype(int)
    row = np.floor((ey - origin_xy[1]) / resolution).astype(int)
    h, w = mask.shape
    inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    hits = int(mask[row[inside], col[inside]].sum())
    return hits / float(ok.sum())


def respawn_seed(xs, ys, psis, step, est_xy, yaw, back_m, max_dyaw):
    """Where on the racing line did the car respawn?

    Walks back from the point nearest the last estimate for up to back_m and
    takes the FIRST contiguous run of points whose heading is within max_dyaw
    of the new IMU heading. Returns (x, y, psi, run_length_m) at the point of
    that run whose heading matches best -- on a straight every point matches
    about equally and the run length carries the uncertainty; through a bend
    the best match is sharp -- or None if no point behind matches.
    """
    n = len(xs)
    i0 = int(np.argmin(np.hypot(xs - est_xy[0], ys - est_xy[1])))
    run = []
    for k in range(int(back_m / step) + 1):
        i = (i0 - k) % n
        if abs(wrap(psis[i] - yaw)) <= max_dyaw:
            run.append(i)
        elif run:
            break
    if not run:
        return None
    m = min(run, key=lambda i: abs(wrap(psis[i] - yaw)))
    return float(xs[m]), float(ys[m]), float(psis[m]), len(run) * step


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
        p('mode', 'spawn')               # 'spawn' seeds from the constant below,
        #                                  'truth' from /ips, 'global' searches
        # ---- spawn mode: the measured spawn, position only -------------------
        # Defaults from common/frames; the launch files pass initial_x/y/yaw
        # through so there is one source. The heading comes from the IMU
        # (absolute, world-referenced, a permitted sensor) unless
        # spawn_use_imu_yaw is off, in which case spawn_yaw is used verbatim.
        p('spawn_x', float(SPAWN_X))
        p('spawn_y', float(SPAWN_Y))
        p('spawn_yaw', float(SPAWN_YAW))
        p('spawn_use_imu_yaw', True)
        p('settle_s', 2.0)               # after seeding, let the filter absorb a few scans
        p('wheel_radius', 0.0581)        # MEASURED; see dead_reckoning.py
        # ---- seed handshake (see the module docstring) ----
        p('localizer_wait_s', 30.0)      # give up if it never comes up
        # ---- everything that differs between the two localizers -------------
        # Parameters, not branches, so adding a third localizer means a row in
        # race.launch.py's LOCALIZERS table and nothing here.
        p('estimate_topic', '/amcl_pose')     # slam_toolbox publishes /pose
        p('ready_check', 'lifecycle')         # 'lifecycle' | 'tf'; see _ready_yet
        p('localizer_node', 'amcl')           # polled when ready_check=lifecycle
        p('verify_via_tf', False)             # confirm on TF map->base, not a topic
        p('map_frame', 'map')
        p('odom_frame', 'odom')
        p('base_frame', 'roboracer_1')
        # Empty string disables. slam_toolbox offers neither.
        p('nudge_service', '/request_nomotion_update')
        p('global_service', '/reinitialize_global_localization')
        p('seed_tolerance_m', 0.30)      # how far AMCL may land from the seed
        p('seed_tolerance_deg', 15.0)
        p('max_seed_attempts', 5)
        # Refuse to latch /localization_ready unless the pose was actually
        # confirmed. False restores the old "hand over regardless" behaviour.
        p('require_convergence', True)
        # ---- scan against the map at the confirmed pose (docstring) ----------
        # Calibrated offline on track_clean by ray-casting the map (LOCALIZER.md).
        # 0 disables.
        p('scan_match_min', 0.5)
        p('scan_match_tol_m', 0.10)
        p('scan_match_max_range', 6.0)   # far beams amplify a small yaw error
        p('map_topic', '/map')
        # ---- respawn watch (docstring) --------------------------------------
        p('respawn_watch', True)
        p('respawn_yaw_jump', 0.35)      # rad beyond rate * dt; as pure_pursuit
        p('respawn_back_m', 6.0)         # how far behind to look for the checkpoint
        p('respawn_max_dyaw_deg', 15.0)
        p('respawn_std_cross', 0.30)
        p('path_csv', DEFAULT_RACELINE)

        g = lambda n: self.get_parameter(n).value
        self.throttle = g('throttle')
        self.steer_gain = g('steer_gain')
        self.pos_target = g('pos_std_target')
        self.yaw_target = g('yaw_std_target')
        self.straight_m = g('straight_distance_m')
        self.timeout = g('timeout_s')
        self.wheel_r = g('wheel_radius')
        self.mode = str(g('mode')).lower()
        if self.mode not in ('spawn', 'truth', 'global'):
            raise RuntimeError(f"mode must be 'spawn', 'truth' or 'global', not {self.mode!r}")
        self.spawn = (float(g('spawn_x')), float(g('spawn_y')), float(g('spawn_yaw')))
        self.spawn_imu_yaw = bool(g('spawn_use_imu_yaw'))
        self.settle_s = g('settle_s')
        self.localizer_wait = float(g('localizer_wait_s'))
        self.est_topic = str(g('estimate_topic'))
        self.ready_check = str(g('ready_check')).lower()
        self.verify_via_tf = bool(g('verify_via_tf'))
        self.map_frame = str(g('map_frame'))
        self.odom_frame = str(g('odom_frame'))
        self.base_frame = str(g('base_frame'))
        self.nudge_srv = str(g('nudge_service')).strip()
        self.global_srv = str(g('global_service')).strip()
        self.tol_m = float(g('seed_tolerance_m'))
        self.tol_yaw = math.radians(float(g('seed_tolerance_deg')))
        self.max_attempts = int(g('max_seed_attempts'))
        self.require_convergence = bool(g('require_convergence'))
        self.scan_min = float(g('scan_match_min'))
        self.scan_tol = float(g('scan_match_tol_m'))
        self.scan_range = float(g('scan_match_max_range'))
        self.map_mask = None             # near_wall_mask of the latest /map
        self.map_res = None
        self.map_origin = None
        self._scan_warned = False
        self.respawn_watch = bool(g('respawn_watch'))
        self.respawn_jump = float(g('respawn_yaw_jump'))
        self.respawn_back = float(g('respawn_back_m'))
        self.respawn_dyaw = math.radians(float(g('respawn_max_dyaw_deg')))
        self.respawn_std_cross = float(g('respawn_std_cross'))
        self._imu_prev = None            # (stamp, yaw, rate) for the respawn watch
        self._respawn_last = -1e9
        self.respawns = 0
        self.line = None                 # (x, y, psi, step) of the raceline
        if self.respawn_watch:
            try:
                d = np.loadtxt(str(g('path_csv')), delimiter=',')
                self.line = (d[:, 1], d[:, 2], d[:, 3], float(d[1, 0] - d[0, 0]))
            except Exception as exc:                    # noqa: BLE001
                self.get_logger().warn(
                    f'respawn watch OFF: cannot read the raceline ({exc})')
        self.truth_pos = None            # ground truth as read; truth mode only
        self.truth_quat = None
        # What _seed() publishes: /ips + /imu in truth mode, the spawn constant
        # + /imu in spawn mode. Kept apart from truth_* so that spawn mode never
        # holds a ground-truth value at all.
        self.src_pos = None
        self.src_quat = None
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
        self.est = None                  # (x, y, yaw) from the latest estimate
        self.est_t = None                # when that estimate arrived
        self.seed_pose = None            # (x, y, yaw) of the last seed sent
        self.attempts = 0
        # Error of the latest estimate against truth AT THAT INSTANT, never
        # against the seed. See _record() for why that distinction matters.
        self.est_err = None
        self.est_dyaw = None
        self.speed_est = 0.0
        self.localizer_up = False
        self.scattered = False
        self._state_pending = False
        self._state_last = 0.0
        self._nudge_last = 0.0
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self._last = None

        self.create_subscription(LaserScan, f'{NS}/lidar', self._cb_scan, QOS)
        self.create_subscription(PoseWithCovarianceStamped, self.est_topic,
                                 self._cb_pose, QOS)
        self.create_subscription(JointState, f'{NS}/left_encoder',
                                 lambda m: self._cb_enc('l', m), QOS)
        self.create_subscription(JointState, f'{NS}/right_encoder',
                                 lambda m: self._cb_enc('r', m), QOS)
        # Kept as a handle, not discarded: this subscription is destroyed the
        # moment the seed resolves, so ground truth is genuinely read only
        # inside the warmup window. /imu is a legal sensor and stays.
        self._ips_sub = None
        if self.mode == 'truth':
            self._ips_sub = self.create_subscription(
                Point, f'{NS}/ips', self._cb_ips, QOS)
        elif self.mode == 'spawn':
            # No /ips subscription exists in this mode, not even a dormant one.
            self.src_pos = (self.spawn[0], self.spawn[1])
            if not self.spawn_imu_yaw:
                half = 0.5 * self.spawn[2]
                self.src_quat = (0.0, 0.0, math.sin(half), math.cos(half))
        # The seed heading (truth, spawn) and the respawn watch (every mode).
        self.create_subscription(Imu, f'{NS}/imu', self._cb_imu, QOS)
        if self.scan_min > 0.0:
            self.create_subscription(OccupancyGrid, str(g('map_topic')),
                                     self._cb_map, LATCHED)

        # TF is needed to notice slam_toolbox coming up (ready_check=tf) and to
        # read the estimate the follower actually drives on (verify_via_tf).
        self.tf_buffer = self.tf_listener = None
        if self.ready_check == 'tf' or self.verify_via_tf:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            if self.verify_via_tf:
                self.create_timer(0.05, self._tf_estimate)

        self.pub_t = self.create_publisher(Float32, f'{NS}/throttle_command', QOS)
        self.pub_s = self.create_publisher(Float32, f'{NS}/steering_command', QOS)
        self.pub_ready = self.create_publisher(Bool, '/localization_ready', LATCHED)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped,
                                              '/initialpose', QOS)

        self.localizer_name = str(g('localizer_node')).strip('/')
        self._state_cli = (
            self.create_client(GetState, f'/{self.localizer_name}/get_state')
            if self.ready_check == 'lifecycle' else None)
        # Forces a filter update without motion. Best effort: if this localizer
        # does not expose it, verification just waits for a real scan update
        # instead -- which is what slam_toolbox does, since a forced
        # relocalization republishes /pose on the very next scan anyway.
        self._nomotion_cli = (self.create_client(Empty, self.nudge_srv)
                              if self.nudge_srv else None)

        self._announce(False)
        if self.mode == 'truth':
            restricted.seed(self, f'{COMMON_NS}/ips', 'initial pose seed')
            self.get_logger().info(
                'mode=truth: seeding the initial pose from /ips + /imu before the '
                'car moves, then releasing /ips. Tracking is lidar + map + dead '
                'reckoning from there on.')
        else:
            restricted.banner(self, [f'{NS}/lidar', f'{NS}/imu', f'{NS}/left_encoder',
                                     f'{NS}/right_encoder', self.est_topic])
            if self.mode == 'spawn':
                x, y, yaw = self.spawn
                heading = 'heading from /imu' if self.spawn_imu_yaw else 'heading from spawn_yaw'
                self.get_logger().info(
                    f'mode=spawn: seeding the initial pose from the measured spawn '
                    f'x={x:+.3f} y={y:+.3f} yaw={math.degrees(yaw):+.1f} deg ({heading}). '
                    'No restricted topic is read at any point.')
        if self.ready_check == 'lifecycle':
            self.get_logger().info(
                f'waiting for {self.localizer_name} to reach the active state '
                'before sending anything')
        else:
            self.get_logger().info(
                f'waiting for TF {self.map_frame} -> {self.odom_frame}; it appears '
                'only once the localizer has processed its first scan, which is '
                'also when it starts accepting /initialpose')

        self.create_timer(0.05, self._tick)
        self.create_timer(2.0, self._report)

    # ---- setup -----------------------------------------------------------

    def _ready_yet(self, now):
        """Has the localizer reached the point where it will accept a seed?

        Two mechanisms, because the two localizers answer it differently:

        lifecycle   AMCL is a nav2 lifecycle node; poll get_state until ACTIVE.
        tf          slam_toolbox is a plain node with no state to poll. It
                    publishes map->odom only after processing its first scan,
                    and that is the same instant localizePoseCallback stops
                    early-returning. So the transform existing IS the handshake.
        """
        if self.ready_check == 'tf':
            try:
                if self.tf_buffer.can_transform(
                        self.map_frame, self.odom_frame, Time()):
                    self.localizer_up = True
                    self.get_logger().info(
                        f'{self.map_frame} -> {self.odom_frame} is live -- the '
                        'localizer has processed a scan and will now accept '
                        '/initialpose')
            except Exception:                           # noqa: BLE001
                pass
            return
        self._poll_lifecycle(now)

    def _poll_lifecycle(self, now):
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
            self.localizer_up = True
            self.get_logger().info(
                f'{self.localizer_name} is ACTIVE -- an initial pose will now '
                'be accepted')

    def _scatter(self):
        """Ask the localizer to search the map with no prior. AMCL only."""
        cli = self.create_client(Empty, self.global_srv)
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f'no {self.global_srv} service; falling back to the '
                "localizer's configured initial pose")
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
        self.src_pos = self.truth_pos

    def _cb_imu(self, msg):
        q = msg.orientation
        quat = (q.x, q.y, q.z, q.w)
        if self.mode == 'truth':
            self.src_quat = self.truth_quat = quat
        elif self.mode == 'spawn' and self.spawn_imu_yaw:
            self.src_quat = quat
        self._watch_respawn(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                            yaw_from_quat_xyzw(quat), msg.angular_velocity.z)

    def _cb_map(self, msg):
        info = msg.info
        self.map_res = float(info.resolution)
        self.map_origin = (info.origin.position.x, info.origin.position.y)
        self.map_mask = near_wall_mask(msg.data, info.width, info.height,
                                       math.ceil(self.scan_tol / self.map_res))

    def _scan_check(self, pose):
        """(ok, text) for the scan scored against the map at `pose`.

        Passes, loudly, when there is nothing to score with: AMCL cannot be
        active without the map, so a missing map means this node's own
        subscription is at fault, and that must not park the car.
        """
        if self.scan_min <= 0.0:
            return True, 'scan check off'
        score = None
        if self.map_mask is not None and self.scan is not None:
            score = scan_match_score(self.map_mask, self.map_res, self.map_origin,
                                     self.scan, pose, float(LIDAR_XYZ[0]),
                                     self.scan_range)
        if score is None:
            if not self._scan_warned:
                self._scan_warned = True
                self.get_logger().warn(
                    'scan-vs-map check SKIPPED: no map or no usable scan yet')
            return True, 'scan check skipped'
        text = (f'{score * 100:.0f}% of beams on a wall within '
                f'{self.scan_tol * 100:.0f} cm (min {self.scan_min * 100:.0f}%)')
        return score >= self.scan_min, text

    def _watch_respawn(self, t, yaw, rate):
        """After hand-over: re-seed AMCL when the heading jumps (docstring)."""
        prev, self._imu_prev = self._imu_prev, (t, yaw, rate)
        if (not self.respawn_watch or not self.ready or prev is None
                or self.line is None or self.est is None):
            return
        dt = t - prev[0]
        if not 0.0 < dt <= 0.3:
            return          # over a stall the car may really have turned that far
        jump = wrap(yaw - prev[1] - prev[2] * dt)
        if abs(jump) <= self.respawn_jump or t - self._respawn_last < 1.0:
            return
        self._respawn_last = t
        self.respawns += 1
        xs, ys, psis, step = self.line
        seed = respawn_seed(xs, ys, psis, step, self.est[:2], yaw,
                            self.respawn_back, self.respawn_dyaw)
        if seed is None:
            self.get_logger().error(
                f'RESPAWN {self.respawns} (heading jumped {math.degrees(jump):+.0f} deg) '
                f'but no raceline point within {self.respawn_back:.0f} m behind the '
                f'last estimate heads {math.degrees(yaw):+.0f} deg. AMCL NOT re-seeded.')
            return
        sx, sy, psi, run = seed
        # Uniform over the matching stretch along the track: std = L / sqrt(12).
        along = max(0.5, run / math.sqrt(12.0))
        c, s = math.cos(psi), math.sin(psi)
        va, vc = along ** 2, self.respawn_std_cross ** 2
        cov = (c * c * va + s * s * vc, c * s * (va - vc), s * s * va + c * c * vc)
        self._publish_initialpose(sx, sy, yaw, cov, math.radians(5.0))
        self.get_logger().warn(
            f'RESPAWN {self.respawns}: heading jumped {math.degrees(jump):+.0f} deg. '
            f'Re-seeded AMCL at x={sx:+.2f} y={sy:+.2f} yaw={math.degrees(yaw):+.0f} deg, '
            f'{run:.1f} m of matching line, along std {along:.2f} m.')

    def _seed(self):
        """Publish the seed pose to /initialpose and remember what was sent."""
        x, y = self.src_pos
        yaw = yaw_from_quat_xyzw(self.src_quat)
        # Tight but not zero: the seed is good, the map is not perfect.
        self._publish_initialpose(x, y, yaw, (0.05 ** 2, 0.0, 0.05 ** 2),
                                  math.radians(3.0))
        self.seed_pose = (x, y, yaw)
        self.seeded_at = self.get_clock().now().nanoseconds * 1e-9
        self.attempts += 1
        self.get_logger().info(
            f'seed {self.attempts}/{self.max_attempts}: x={x:+.3f} y={y:+.3f} '
            f'yaw={math.degrees(yaw):+.1f} deg -- awaiting confirmation')
        if self.attempts == 1 and self.mode == 'truth':
            self._report_spawn(x, y, yaw)

    def _publish_initialpose(self, x, y, yaw, cov_xy, std_yaw):
        """cov_xy = (xx, xy, yy); nav2 AMCL reads the full 2x2 block."""
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.map_frame
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z = math.sin(0.5 * yaw)
        m.pose.pose.orientation.w = math.cos(0.5 * yaw)
        m.pose.covariance[0], m.pose.covariance[1] = cov_xy[0], cov_xy[1]
        m.pose.covariance[6], m.pose.covariance[7] = cov_xy[1], cov_xy[2]
        m.pose.covariance[35] = std_yaw ** 2
        self.pub_init.publish(m)

    def _report_spawn(self, x, y, yaw):
        """Print the MEASURED spawn, ready to paste into frames.py.

        SPAWN_* is no longer the primary path -- the warmup seed is, and it is
        exact. It is still the FALLBACK, used whenever this node is off
        (bootstrap:=false), in global mode, or when /ips never arrives. A
        fallback that is 0.9 m out is not a fallback: slam_toolbox's +-0.5 m
        seed search cannot recover it. So the measurement is printed whenever it
        disagrees with the constant, and the constant is worth keeping honest.

        Only printed when the car is actually parked: seeding while moving
        measures where the car IS, not where it STARTS.
        """
        gap = math.hypot(x - float(SPAWN_X), y - float(SPAWN_Y))
        dyaw = abs(wrap(yaw - float(SPAWN_YAW)))
        if abs(self.speed) > 0.1:
            self.get_logger().info(
                f'not reporting a spawn constant: the car is moving at '
                f'{abs(self.speed):.1f} m/s, so this pose is not the spawn')
            return
        if gap <= 0.10:
            self.get_logger().info(
                f'MEASURED SPAWN x={x:+.3f} y={y:+.3f} '
                f'yaw={math.degrees(yaw):+.1f} deg -- within {gap * 100:.0f} cm '
                'of roboracer_stack.common.frames.SPAWN_*, so the fallback is sound too. '
                'Nothing to change.')
            return
        reach = ('OUTSIDE slam_toolbox\'s +-0.5 m seed search, so any run that '
                 'falls back to it CANNOT recover and the offset is frozen for '
                 'the whole run' if gap > 0.5 else
                 'inside the +-0.5 m seed search, so slam could still recover it')
        self.get_logger().warn(
            f'MEASURED SPAWN is {gap:.3f} m / {math.degrees(dyaw):.1f} deg from '
            f'roboracer_stack.common.frames.SPAWN_* -- {reach}. This run is seeded and '
            'therefore fine; the FALLBACK is what is wrong.\n'
            f'    Paste into common/frames.py so bootstrap:=false and '
            f'bootstrap_mode:=global start somewhere real:\n'
            f"        SPAWN_X = '{x:.3f}'\n"
            f"        SPAWN_Y = '{y:.3f}'\n"
            f"        SPAWN_YAW = '{yaw:.4f}'")

    def _cb_pose(self, msg):
        c = msg.pose.covariance
        self.pos_std = math.sqrt(max(c[0], 0.0) + max(c[7], 0.0))
        self.yaw_std = math.sqrt(max(c[35], 0.0))
        if self.verify_via_tf:
            return          # covariance is still worth having; the pose is not
        q = msg.pose.pose.orientation
        self._record(msg.pose.pose.position.x, msg.pose.pose.position.y,
                     yaw_from_quat_xyzw((q.x, q.y, q.z, q.w)))

    def _tf_estimate(self):
        """Read the estimate from TF map->base instead of a pose topic.

        This is the transform pure_pursuit consumes when use_tf_pose is true --
        the localizer's correction COMPOSED with dead reckoning -- so confirming
        the seed against it checks the pose the car will really drive on. It is
        also the only option for a localizer that publishes no pose topic.

        Freshness is wall-clock arrival, exactly as for the topic path: TF is
        continuous, so `settle_s` is what actually separates before from after.
        A forced relocalization is reprocessed on the very next scan (~55 ms at
        the measured 18 Hz), far inside the default 2 s settle.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except Exception:                               # noqa: BLE001
            return
        t, q = tf.transform.translation, tf.transform.rotation
        self._record(t.x, t.y, yaw_from_quat_xyzw((q.x, q.y, q.z, q.w)))

    def _record(self, x, y, yaw):
        """Store the latest estimate and score it against truth AT THIS INSTANT.

        Scoring must not compare against the seed: the seed is a snapshot from
        settle_s ago, so if the car is moving the "disagreement" is just
        distance travelled -- 3.7 m/s x 2 s = 7.5 m of pure motion, which fails
        every tolerance no matter how well the localizer is tracking.
        """
        self.est = (x, y, yaw)
        self.est_t = self.get_clock().now().nanoseconds * 1e-9
        ref = None
        if self.mode == 'truth':
            if self.truth_pos is not None and self.truth_quat is not None:
                ref = (*self.truth_pos, yaw_from_quat_xyzw(self.truth_quat))
        elif self.seed_pose is not None and abs(self.speed) < 0.3:
            # No truth to score against in spawn mode: the seed itself is the
            # reference, which is only meaningful while the car is parked --
            # the normal case, since the follower waits on this node.
            ref = self.seed_pose
        if ref is not None:
            self.est_err = math.hypot(x - ref[0], y - ref[1])
            self.est_dyaw = abs(wrap(yaw - ref[2]))
        else:
            self.est_err = self.est_dyaw = None

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
        if self._nomotion_cli is None:
            return
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

        # Nothing may be sent until the localizer is genuinely up -- see the
        # module docstring. This is the whole point of the handshake.
        if not self.localizer_up:
            self._ready_yet(now)
            if not self.localizer_up and now - self.t0 > self.localizer_wait:
                if self.ready_check == 'tf':
                    self._fail(
                        f'no TF {self.map_frame} -> {self.odom_frame} within '
                        f'{self.localizer_wait:.0f} s. Is the localizer running, '
                        'and is it receiving scans? slam_toolbox publishes this '
                        'only after processing its first scan.')
                else:
                    self._fail(
                        f'{self.localizer_name} never reached the active state '
                        f'within {self.localizer_wait:.0f} s. Is '
                        'nav2_lifecycle_manager running, and did map_server '
                        'come up?')
            return

        if self.mode == 'global':
            self._tick_global(now)
        else:
            self._tick_seeded(now)

    def _tick_seeded(self, now):
        """truth and spawn modes: seed, confirm the localizer adopted it, hand over."""
        if self.src_pos is None or self.src_quat is None:
            return                      # still waiting for /ips and/or /imu
        if self.seeded_at is None:
            self._seed()
            return

        if abs(self.speed) < 0.3:
            self._nudge(now)     # parked: AMCL needs a shove to run an update
        if now - self.seeded_at < self.settle_s:
            return

        ref_name = 'truth' if self.mode == 'truth' else 'the seed'
        fresh = (self.est is not None and self.est_t is not None
                 and self.est_t >= self.seeded_at)
        std = f'{self.pos_std:.3f} m' if self.pos_std is not None else 'unknown'
        moving = ' while driving' if abs(self.speed) > 0.3 else ''
        if fresh and self.est_err is not None:
            gap, dyaw = self.est_err, self.est_dyaw
            scan_ok, scan_text = self._scan_check(self.est)
            if gap <= self.tol_m and dyaw <= self.tol_yaw and scan_ok:
                self._finish(
                    f'{scan_text}. seed CONFIRMED on attempt {self.attempts}{moving}: the '
                    f'estimate is {gap * 100:.1f} cm / {math.degrees(dyaw):.1f} deg from '
                    f'{ref_name} (pos std {std}). Tracking is now lidar + map + dead '
                    'reckoning only.')
                return
            if gap <= self.tol_m and dyaw <= self.tol_yaw:
                reason = (f'the estimate agrees with {ref_name}, but the SCAN DOES NOT '
                          f'MATCH THE MAP there ({scan_text}): a wrong SPAWN_* or a '
                          'map that does not line up with the simulator')
            else:
                reason = (f'the estimate is {gap:.2f} m / {math.degrees(dyaw):.1f} deg from '
                          f'{ref_name.upper()}, tolerance {self.tol_m:.2f} m / '
                          f'{math.degrees(self.tol_yaw):.1f} deg '
                          f'(speed {abs(self.speed):.1f} m/s)')
        elif (fresh and self.mode == 'spawn' and self._converged()
              and self._scan_check(self.est)[0]):
            # Moving, so the seed is no longer a valid reference (see _record);
            # the filter's own covariance and the scan are what is left.
            self._finish(
                f'{self._scan_check(self.est)[1]}. seed CONFIRMED on attempt {self.attempts}{moving}: the filter '
                f'converged (pos std {std}, yaw std {math.degrees(self.yaw_std):.1f} deg). '
                'Tracking is now lidar + map + dead reckoning only.')
            return
        elif fresh and self.mode == 'spawn' and self._converged():
            reason = (f'the filter converged but the SCAN DOES NOT MATCH THE MAP '
                      f'({self._scan_check(self.est)[1]})')
        elif fresh:
            reason = (f'the filter has not converged while moving (pos std {std})'
                      if self.mode == 'spawn' else f'no {ref_name} reference to score against')
        else:
            source = (f'TF {self.map_frame} -> {self.base_frame}'
                      if self.verify_via_tf else self.est_topic)
            reason = f'no fresh {source} arrived after the seed'

        if self.attempts < self.max_attempts:
            self.get_logger().warn(f'seed {self.attempts} not adopted -- {reason}; re-seeding')
            self._seed()
        else:
            self._fail(f'seed REJECTED after {self.attempts} attempts -- {reason}')

    def _tick_global(self, now):
        if not self.global_srv:
            # slam_toolbox has no map-wide search: it is a single-hypothesis
            # scan matcher, so there is nowhere to hold the competing
            # hypotheses one produces. Every entry point it offers takes a
            # pose, so map_start_pose is all that is left.
            self._finish(
                'this localizer has NO global search, so mode:=global leaves it '
                'on its configured map_start_pose, UNVERIFIED: if that is more '
                'than ~0.5 m from the true spawn it cannot be recovered and the '
                'offset is frozen for the run. You almost certainly want '
                'bootstrap_mode:=spawn instead (the measured spawn constant, no '
                'restricted topic) or bootstrap_mode:=truth (one /ips read '
                'inside the warmup window, then released).')
            return

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

    def _release_truth(self):
        """Destroy the /ips subscription. Idempotent.

        This is what makes the warmup-window claim true rather than a promise:
        after the seed resolves, one way or the other, nothing in this node can
        read ground truth again. /imu stays -- it is a legal sensor, and
        dead_reckoning reads it continuously anyway.
        """
        if self._ips_sub is None:
            return
        self.destroy_subscription(self._ips_sub)
        self._ips_sub = None
        restricted.released(self, [f'{COMMON_NS}/ips'])

    def _finish(self, message):
        self.finished = True
        self.ready = True
        self._send(0.0, 0.0)
        self._release_truth()
        self._announce(True)
        self.get_logger().info(message)
        self.get_logger().info('/localization_ready = true, follower may take over')

    def _fail(self, message):
        """Give up. By default do NOT let the follower drive on this pose."""
        self.finished = True
        self._send(0.0, 0.0)
        self._release_truth()
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
        if not self.localizer_up:
            waited = self.get_clock().now().nanoseconds * 1e-9 - self.t0
            what = (f'TF {self.map_frame} -> {self.odom_frame}'
                    if self.ready_check == 'tf'
                    else f'{self.localizer_name} to become active')
            self.get_logger().info(
                f'waiting for {what} ({waited:.0f} s of '
                f'{self.localizer_wait:.0f} s)')
            return
        if self.mode != 'global' and self.seeded_at is None:
            where = '/ips' if self.mode == 'truth' else 'spawn constant'
            self.get_logger().info(
                f'waiting for the seed inputs: position={"ok" if self.src_pos else "--"} '
                f'({where}) heading={"ok" if self.src_quat else "--"} (/imu)')
            return
        if self.mode != 'global':
            source = (f'TF {self.map_frame} -> {self.base_frame}'
                      if self.verify_via_tf else self.est_topic)
            self.get_logger().info(
                f'seed {self.attempts}/{self.max_attempts} sent, awaiting a fresh '
                f'{source} to confirm it')
            return
        if self.pos_std is None:
            self.get_logger().info(f'waiting for {self.est_topic} ...')
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
