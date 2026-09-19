#!/usr/bin/env python3

"""Get the localizer a starting pose, CONFIRM it took, then hand over.

Serves BOTH localizers. Everything AMCL-shaped is a parameter, so slam_toolbox
reuses this handshake rather than getting a second copy of it -- see
"THE TWO LOCALIZERS" below.

Two modes:

  mode:=truth   (default)  Read /ips and /imu ONCE, seed the localizer with that
                           pose, verify it actually adopted it, release /ips,
                           and hand over. RACE-LEGAL: the first lap is a warmup
                           and the timer starts after it, so a single read
                           before the car moves sits inside the permitted
                           window -- see racer_common/restricted.py, THE WARMUP
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

Either way it latches /localization_ready when done, which the follower waits on
instead of a fixed timer.

WHICH MODE TO RACE
------------------
truth. It is legal, it is exact, and it is the only one slam_toolbox can use at
all. global exists to prove the stack can start with no prior, and as AMCL's
fallback if the warmup rule ever changes.

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

    1. wait for the localizer to come up -- get_state for AMCL, the appearance
       of map->odom for slam_toolbox (see THE TWO LOCALIZERS below)
    2. publish the seed to /initialpose
    3. nudge AMCL into running an update (it is stationary in truth mode, and
       AMCL only resamples once it believes it has moved)
    4. compare the next estimate against what was sent; re-seed if it differs
    5. only then latch /localization_ready

If the seed cannot be confirmed, this node REFUSES to hand over rather than
letting the follower drive on a pose nobody checked. Set
require_convergence:=false to restore the old hand-over-anyway behaviour.

THE TWO LOCALIZERS
------------------
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

THAT FALLBACK IS WHY THE SEED IS LOGGED VERBATIM. mode:=race forbids /ips, so a
slam race run starts on racer_common.frames.SPAWN_* and nothing else. If that
constant is wrong the error is frozen for the whole run: slam_toolbox seeds with
a +-0.5 m correlative search (correlation_search_space_dimension: 1.0) and
cannot recover a larger one. So every truth-mode seed prints a paste-ready
SPAWN_X/Y/YAW block -- run once in dev, paste into frames.py, and race mode
inherits a measured spawn instead of a guess.

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
from nav_msgs.msg import OccupancyGrid
from lifecycle_msgs.srv import GetState
from racer_common import frames, restricted
from racer_common.frames import NS as COMMON_NS
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time
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



# How far "ahead" of the car a checkpoint may sit and still count as behind it.
# The simulator always resets BACKWARD to a checkpoint already passed, but the
# pose at the contact is projected onto the centreline to order it, and a car
# cutting inside a tight corner projects SHORT of the progress it has really
# made. Measured over the six contacts logged on IROS 2026 (runs 14, 19, 20)
# the projection error runs -1.04 to +1.50 m, and every value in 1.05-1.35
# picks the checkpoint the simulator actually used for all six; below 1.05 the
# two hairpin-1 contacts pick the checkpoint 3 m too far back, above 1.35 the
# contact after hairpin 1 picks the one ahead. 1.20 is the middle of that
# window. tools/check_recovery_prior.py is the check. Picking wrong is not
# fatal: the seed then fails its tolerance and the global search takes over.
BACK_SLOP_M = 1.20


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

def pick_back(s_last, cs, lap, slop=BACK_SLOP_M):
    """Arc length from a checkpoint to the car along the lap, or None.

    None means the centreline was unusable (a lap length of 0), which is worth
    saying rather than dividing by: doing that killed this node on the first
    reset of every run until 2026-09-17 (IROS run 20, 9 resets, none
    recovered).
    """
    if not lap > 0.0:
        return None
    return (s_last - cs + slop) % lap - slop


class LocalizationBootstrap(Node):

    def __init__(self):
        super().__init__('localization_bootstrap')

        p = self.declare_parameter
        p('track', '')                   # which track's registered spawn to check against; '' = RACER_TRACK
        p('throttle', 0.08)              # gentle -- this is a search, not a lap
        p('steer_gain', 0.6)             # wall-centring proportional gain
        p('gap_gain', 1.0)               # recovery creep: steer per rad toward the most open beam
        p('pos_std_target', 0.15)        # [m]   converged when below this
        p('yaw_std_target', 0.09)        # [rad] ~5 degrees
        p('straight_distance_m', 3.0)    # creep straight this far, then seek a corner
        p('timeout_s', 60.0)
        p('mode', 'truth')               # 'truth' seeds from /ips; 'global' searches
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
        # ---- scan-vs-map confirmation (ported from iros_compete_usman) --------
        # Steps above confirm the estimate against the SEED and against AMCL's own
        # covariance, and both can agree on a pose that is simply wrong: a stale
        # SPAWN_* or a map that does not line up with the simulator's frame passes
        # them. This asks the one independent witness there is -- at the confirmed
        # pose, what fraction of lidar beams end within scan_match_tol_m of a wall
        # on the map? A right pose scores near 1; a pose 0.3 m or a few degrees out
        # scores far lower. 0 disables. Calibrated on track_clean, see LOCALIZER.md
        # on the iros_compete_usman branch.
        p('scan_match_min', 0.5)
        p('scan_match_tol_m', 0.10)
        p('scan_match_max_range', 6.0)   # far beams amplify a small yaw error
        p('map_topic', '/map')
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
        # ---- recovery after a wall contact (see _tick_recover) ----
        # The simulator resets a hit car to the last checkpoint and zeroes its
        # velocity; the rules add 10 s and say localization "will have to be
        # robust against this re-setting action". Without this, dead reckoning
        # keeps integrating from the old pose, AMCL's particles no longer explain
        # the scan, and the follower drives blind: runs 14 and 24 ended that way.
        # Legal inputs only: encoders (speed collapse), IMU (heading step), lidar
        # (creep), our own TF, /initialpose and AMCL's services.
        p('recover', True)
        p('recover_settle_s', 1.0)          # settle after a recovery seed (initial seed uses settle_s)
        p('recover_tolerance_m', 0.40)      # a checkpoint prior is good to ~0.1 m; AMCL may land near it
        p('recover_max_attempts', 3)        # then the global search
        p('recover_global_timeout_s', 15.0)
        p('recover_creep_s', 1.0)           # roll gently after confirmation so AMCL tightens on motion
        p('reset_yaw_step_deg', 10.0)       # an IMU heading step this size in one tick, after the
        # yaw-rate term, is a reset. 20 missed a 12 deg reset at 45 Hz (IROS run 14):
        # the residual after subtracting rate*dt is under 2 deg at any tick, the encoder
        # did NOT collapse through either reset in that run (it reads the command), so
        # the heading step is the only signature that fires there.
        # Reset signature 3, the one that needs neither the heading nor the
        # encoder: a teleport moves EVERY beam. Between two consecutive scans
        # the median |range change| is 0.02-0.06 m while driving (45 Hz, up to
        # 9 m/s on the straight, 3 deg/tick of yaw in the hairpins) and 0.5-0.7 m
        # through a reset (lv_margin_2, lv_fast_19_1). lv_margin_2's first reset
        # (2026-09-19) fired NEITHER of the two signatures above -- the checkpoint
        # heading was 8.8 deg from the car's, under the 10 deg step, and the
        # encoder kept reading the command -- so the follower drove on with a
        # 0.6 m error and hit again a second later. 0 disables.
        p('reset_scan_jump_m', 0.30)
        p('reset_speed_from', 1.0)          # encoder speed collapsing from >= this ...
        p('reset_speed_to', 0.3)            # ... to <= this between two samples is a reset
        p('recover_use_checkpoints', True)  # False forces the no-data tier, to test it where truth exists
        p('recover_local_back_m', 0.9)      # no-data tier: seed this far back along the last pose (mean of 9 resets)
        p('recover_local_std_m', 1.0)       # ... with this much position uncertainty (resets spread 0.15-2.8 m)
        p('recover_std_ok_m', 0.30)         # a recovery seed counts as adopted only if AMCL's own pos std is under this
        # Two registered checkpoints can sit a metre apart and a contact between
        # them is ambiguous to the centimetre: the simulator's trigger fires when
        # the BODY enters it, so (0.52, 3.89) reset to the checkpoint 0.3 m ahead
        # (run 19) and (0.53, 3.98) to the one 1.1 m behind (lv_L750_warm). When
        # the prior is the wrong one of the pair the localizer moves off it with a
        # clean fit; that used to fail the seed tolerance three times and end in
        # the global search. Within this distance of the prior, a scan-confirmed,
        # converged estimate is accepted where it settled instead.
        p('recover_offprior_m', 3.0)

        g = lambda n: self.get_parameter(n).value
        self.throttle = g('throttle')
        self.steer_gain = g('steer_gain')
        self.gap_gain = float(g('gap_gain'))
        self.pos_target = g('pos_std_target')
        self.yaw_target = g('yaw_std_target')
        self.straight_m = g('straight_distance_m')
        self.timeout = g('timeout_s')
        self.wheel_r = g('wheel_radius')
        self.mode = str(g('mode')).lower()
        self.settle_s = g('settle_s')
        self.localizer_wait = float(g('localizer_wait_s'))
        self.est_topic = str(g('estimate_topic'))
        self.ready_check = str(g('ready_check')).lower()
        self.scan_min = float(g('scan_match_min'))
        self.scan_tol = float(g('scan_match_tol_m'))
        self.scan_range = float(g('scan_match_max_range'))
        self.map_mask = self.map_res = self.map_origin = None
        self._scan_warned = False
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
        self.recover = bool(g('recover'))
        self.recover_settle = float(g('recover_settle_s'))
        self.recover_tol = float(g('recover_tolerance_m'))
        self.recover_attempts = int(g('recover_max_attempts'))
        self.recover_global_timeout = float(g('recover_global_timeout_s'))
        self.recover_creep = float(g('recover_creep_s'))
        self.reset_yaw_step = math.radians(float(g('reset_yaw_step_deg')))
        self.reset_v_from, self.reset_v_to = float(g('reset_speed_from')), float(g('reset_speed_to'))
        self.reset_scan_jump = float(g('reset_scan_jump_m'))
        self.recover_use_cps = bool(g('recover_use_checkpoints'))
        self.recover_back = float(g('recover_local_back_m'))
        self.recover_local_std = float(g('recover_local_std_m'))
        self.recover_std_ok = float(g('recover_std_ok_m'))
        self.recover_offprior = float(g('recover_offprior_m'))
        self.checkpoints = frames.checkpoints(str(g('track')) or None)
        # The centreline orders the lap: "the checkpoint behind the car" is the
        # one with the largest arc length not exceeding the car's, wrapping at
        # the lap. A heading test alone is ambiguous where lanes run parallel
        # (from the spawn, the middle-lane checkpoint is nearer than the right
        # one and the car is "ahead" of both).
        self._cl = self._load_centreline(str(g('track')) or None)
        self._cp_s = [self._s_of(cx, cy) for cx, cy, _ in self.checkpoints] if self._cl is not None else None
        self.seed_std_m, self.seed_std_deg = 0.05, 3.0      # the initial seed; recovery widens them
        self.rec_state = 'off'           # off | watch | flagged | seed | global | creep
        self.last_good = None            # (x, y, yaw) last estimate recorded while trusted
        self._reset_flag = None          # wall time of the last detected reset
        self._rec_t0 = None              # when this recovery began
        self._creep_t0 = None
        self.recoveries = 0
        self._imu_yaw = None
        self._imu_t = None
        self._speed_prev = 0.0
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
        if self.mode == 'truth' or self.recover:
            # /imu is a legal sensor: the truth seed's heading, and the heading
            # step that marks a reset.
            self.create_subscription(Imu, f'{NS}/imu', self._cb_imu, QOS)
            if self.scan_min > 0.0:
                self.create_subscription(OccupancyGrid, str(g('map_topic')), self._cb_map, LATCHED)

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
        prev, self.scan = self.scan, msg
        if prev is None or self.reset_scan_jump <= 0.0 or len(prev.ranges) != len(msg.ranges):
            return
        a = np.asarray(prev.ranges, dtype=float)
        b = np.asarray(msg.ranges, dtype=float)
        ok = np.isfinite(a) & np.isfinite(b) & (a > msg.range_min) & (b > msg.range_min)
        if ok.sum() < 100:
            return
        jump = float(np.median(np.abs(a[ok] - b[ok])))
        if jump > self.reset_scan_jump:
            self._flag_reset(f'scan jumped {jump:.2f} m at the median beam in one tick')

    def _cb_ips(self, msg):
        self.truth_pos = (msg.x, msg.y)

    def _cb_imu(self, msg):
        q = msg.orientation
        self.truth_quat = (q.x, q.y, q.z, q.w)
        # Reset signature 1: a heading step the measured yaw rate cannot explain
        # (3.2 rad/s of steering is 10 deg in a 55 ms tick; a reset is 20-80).
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        yaw = yaw_from_quat_xyzw(self.truth_quat)
        if self._imu_yaw is not None and self._imu_t is not None:
            step = wrap(yaw - self._imu_yaw) - float(msg.angular_velocity.z) * max(0.0, min(t - self._imu_t, 0.3))
            if abs(step) > self.reset_yaw_step:
                self._flag_reset(f'heading stepped {math.degrees(step):+.0f} deg in one tick')
        self._imu_yaw, self._imu_t = yaw, t

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
                                     self.scan, pose, float(frames.LIDAR_XYZ[0]),
                                     self.scan_range)
        if score is None:
            if not self._scan_warned:
                self._scan_warned = True
                self.get_logger().warn('scan-vs-map check SKIPPED: no map or no usable scan yet')
            return True, 'scan check skipped'
        text = (f'{score * 100:.0f}% of beams on a wall within '
                f'{self.scan_tol * 100:.0f} cm (min {self.scan_min * 100:.0f}%)')
        return score >= self.scan_min, text

    def _seed(self):
        """Publish the true pose to /initialpose and remember what was sent."""
        x, y = self.truth_pos
        # A PLANAR quaternion built from the yaw. Copying the IMU's quaternion
        # verbatim failed intermittently: nav2 validates the norm to 1e-4 and the
        # bridge forwards the simulator's rounded components, so AMCL rejected
        # four re-seeds in a row as "malformed" (2026-09-11). Yaw is all it needs.
        yaw = yaw_from_quat_xyzw(self.truth_quat)
        qx, qy, qz, qw = 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        (m.pose.pose.orientation.x, m.pose.pose.orientation.y,
         m.pose.pose.orientation.z, m.pose.pose.orientation.w) = qx, qy, qz, qw
        # Tight but not zero: the seed is good, the map is not perfect. Recovery
        # widens these to the prior's uncertainty.
        m.pose.covariance[0] = m.pose.covariance[7] = self.seed_std_m ** 2
        m.pose.covariance[35] = math.radians(self.seed_std_deg) ** 2
        self.pub_init.publish(m)
        self.seed_pose = (x, y, yaw)
        self.seeded_at = self.get_clock().now().nanoseconds * 1e-9
        self.attempts += 1
        self.get_logger().info(
            f'seed {self.attempts}/{self.max_attempts}: x={x:+.3f} y={y:+.3f} '
            f'yaw={math.degrees(yaw):+.1f} deg -- awaiting confirmation')
        if self.attempts == 1:
            self._report_spawn(x, y, yaw)

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
        # The registered spawn of the track in use, not the module constants
        # (those are computed at import for RACER_TRACK and would be Porto's
        # on a track:= run).
        from racer_common import frames
        SPAWN_X, SPAWN_Y, SPAWN_YAW = frames.spawn(str(self.get_parameter('track').value) or None)
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
                'of racer_common.frames.SPAWN_*, so the fallback is sound too. '
                'Nothing to change.')
            return
        reach = ('OUTSIDE slam_toolbox\'s +-0.5 m seed search, so any run that '
                 'falls back to it CANNOT recover and the offset is frozen for '
                 'the whole run' if gap > 0.5 else
                 'inside the +-0.5 m seed search, so slam could still recover it')
        self.get_logger().warn(
            f'MEASURED SPAWN is {gap:.3f} m / {math.degrees(dyaw):.1f} deg from '
            f'racer_common.frames.SPAWN_* -- {reach}. This run is seeded and '
            'therefore fine; the FALLBACK is what is wrong.\n'
            f'    Paste into racer_common/frames.py so bootstrap:=false and '
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
        if self.truth_pos is not None and self.truth_quat is not None:
            self.est_err = math.hypot(x - self.truth_pos[0], y - self.truth_pos[1])
            self.est_dyaw = abs(wrap(yaw - yaw_from_quat_xyzw(self.truth_quat)))
        # 'creep' too: the estimate was confirmed a moment ago, and a car reset
        # again mid-creep lands at a checkpoint AHEAD of where the creep began.
        # Frozen at the confirmation instead, every re-seed went back to the first
        # checkpoint while the car sat 3, 6, 9 m further on (M01, 2026-09-18:
        # seven resets, then the global search timed out).
        if self.rec_state in ('watch', 'creep'):
            self.last_good = (x, y, yaw)         # frozen the instant a reset is flagged

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
            prev, self.speed = self._speed_prev, float(np.mean(rates))
            self._speed_prev = self.speed
            # Reset signature 2: the simulator zeroes the velocity in one step.
            if prev >= self.reset_v_from and abs(self.speed) <= self.reset_v_to:
                self._flag_reset(f'encoder speed collapsed {prev:.1f} -> {abs(self.speed):.1f} m/s in one sample')

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

        # Steering is + = LEFT in this sim (+0.7 at hairpin 1 gives +2.3 rad/s yaw)
        # and the scan is REP-103 (+90 deg = left), so steer toward the side with
        # MORE room: left - right. It was right - left, which steers toward the
        # nearer wall and grows any offset -- M01's creep curved left from the
        # first metre and M02's hit the wall within 1 m, five times over.
        steer = 0.0
        if math.isfinite(left) and math.isfinite(right):
            steer = self.steer_gain * (left - right) / max(left + right, 0.1)
        if self.rec_state == 'creep':
            # A reset checkpoint on a bend points the car at the outside wall
            # (s 22.2 on IROS 2026: 0.94 m at +30 deg, 3.8 m at -30), where
            # side-beam centring alone drives straight on. Head for the most
            # open direction ahead as well.
            angles = [math.radians(a) for a in range(-60, 61, 10)]
            ranges = [min(self._beam(a), 4.0) for a in angles]
            best = angles[int(np.argmax(ranges))]
            steer += self.gap_gain * best

        # Past the straight phase, bias the steering so the car finds a corner --
        # corridors are ambiguous, corners are not.
        # Not in a recovery creep: the pose is already confirmed there, and on a
        # long straight the bias walked the car into the left wall about 3 m in
        # (M01 at recover_creep_s 3.0, 2 m/s).
        if self.distance > self.straight_m and self.rec_state != 'creep':
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
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.finished:
            if self.recover and self.rec_state != 'off':
                self._tick_recover(now)
            return
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
            scan_ok, scan_text = self._scan_check(self.est) if self.est is not None else (True, 'no estimate')
            if gap <= self.tol_m and dyaw <= self.tol_yaw and scan_ok and self._recover_ok():
                std = f'{self.pos_std:.3f} m' if self.pos_std is not None else 'unknown'
                moving = ' while driving' if abs(self.speed) > 0.3 else ''
                self._finish(
                    f'{scan_text}. seed CONFIRMED on attempt {self.attempts}{moving}: the '
                    f'estimate is {gap * 100:.1f} cm / {math.degrees(dyaw):.1f} deg from truth '
                    f'(pos std {std}). Tracking is now lidar + map + dead '
                    'reckoning only.')
                return
            if (self.rec_state == 'seed' and gap > self.tol_m and gap <= self.recover_offprior
                    and dyaw <= self.tol_yaw and scan_ok and self._recover_ok()):
                self._finish(
                    f'{scan_text}. recovery seed CONVERGED OFF THE PRIOR on attempt {self.attempts}: '
                    f'the localizer settled {gap:.2f} m from the checkpoint prior (pos std '
                    f'{self.pos_std:.3f} m) -- an adjacent checkpoint; accepted where it settled')
                return
            agrees = gap <= self.tol_m and dyaw <= self.tol_yaw
            failed = [n for n, ok in (('position', agrees), (scan_text, scan_ok),
                                      (f'pos std {self.pos_std} > {self.recover_std_ok}', self._recover_ok()))
                      if not ok]
            reason = (f'the estimate is {gap:.2f} m / {math.degrees(dyaw):.1f} deg from '
                      f'the seed, tolerance {self.tol_m:.2f} m / '
                      f'{math.degrees(self.tol_yaw):.1f} deg '
                      f'(speed {abs(self.speed):.1f} m/s); failed: {", ".join(failed)}')
            if (self.rec_state == 'seed' and agrees and self.est is not None
                    and self.attempts >= self.max_attempts):
                # The estimate sits on the checkpoint, only the scan score or the
                # spread is short -- typically a car reset facing a wall. The
                # global fallback cannot converge on this track (std 4-5 m for 15 s,
                # M03 recovery #3, 2026-09-18), so a pose that agrees with the
                # prior on every attempt is the better bet; creep tightens it.
                self.get_logger().warn(f'recovery: accepting the seed on agreement -- {reason}')
                self._finish(f'seed ACCEPTED on agreement after {self.attempts} attempts: '
                             f'{gap * 100:.1f} cm / {math.degrees(dyaw):.1f} deg from the prior')
                return
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
                'bootstrap_mode:=truth instead -- it is race-legal (one /ips '
                'read inside the warmup window, then released) and it is the '
                'only way this localizer starts on the true pose.')
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

    # ---- recovery after a wall contact -----------------------------------
    def _load_centreline(self, track):
        import os
        path = os.path.join(frames.raceline_dir(track), 'centerline_full.csv')
        if not os.path.exists(path):
            return None
        try:
            c = np.genfromtxt(path, delimiter=',', comments='#')
            s, x, y = c[:, 0], c[:, 1], c[:, 2]
        except Exception:                                # noqa: BLE001
            return None
        # The extractor does not orient the ring: on IROS 2026 the file's s
        # DECREASES along the lap (spawn at s 44.4, hairpin 1 at 25), on ICRA
        # and Porto it increases. "Behind the car" below is (s_last - cs), so
        # a reversed file picks the checkpoint AHEAD. Orient against the spawn
        # heading: if the tangent at the spawn opposes it, run s the other way.
        try:
            sx, sy, syaw = (float(v) for v in frames.spawn(track))
            j = int(np.argmin((x - sx) ** 2 + (y - sy) ** 2))
            k = (j + 3) % len(s)
            tangent = math.atan2(y[k] - y[j], x[k] - x[j])
            if math.cos(tangent - syaw) < 0.0:
                # Reverse the VALUES and the ORDER together. Mirroring the
                # values alone leaves s descending, so s[-1] is 0 and every
                # lap length taken from it is 0: that made the first reset on
                # IROS 2026 kill this node with a ZeroDivisionError (run 20,
                # 9 resets, none recovered). _lap_length below no longer
                # depends on the ordering either.
                s = (s[-1] - s)[::-1]
                x, y = x[::-1], y[::-1]
                self.get_logger().info('centreline runs against the lap; arc length reversed for checkpoint ordering')
        except Exception:                                # noqa: BLE001
            pass
        return s, x, y

    def _s_of(self, x, y):
        s, cx, cy = self._cl
        return float(s[int(np.argmin((cx - x) ** 2 + (cy - y) ** 2))])

    def _lap_length(self):
        """Lap length from the centreline, whichever way its s runs."""
        return float(np.max(self._cl[0])) if self._cl is not None else 0.0

    def _flag_reset(self, why):
        if not self.recover or self.rec_state in ('off', 'flagged', 'seed', 'global'):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self._reset_flag = now
        self.rec_state = 'flagged'           # freezes last_good; _tick_recover takes it from here
        self.get_logger().warn(f'RESET detected: {why}')

    def _recovery_prior(self):
        """Where the car most likely is now: the known checkpoint behind the
        last trusted pose, else that pose moved back along the track. Heading
        always from the IMU, which is absolute and legal."""
        yaw = yaw_from_quat_xyzw(self.truth_quat) if self.truth_quat else (self.last_good[2] if self.last_good else 0.0)
        lg = self.last_good
        if lg is not None:
            best = None
            if not self.recover_use_cps:
                pass                                              # forced to the no-data tier
            elif self._cl is not None and self._cp_s:
                # Arc length behind the last pose along the centreline, wrapping.
                # Up to 1 m "ahead" is allowed: a car cutting inside a hairpin
                # projects onto the centreline short of where it is along the
                # arc (IROS run 14: the contact projected 0.8 m short of the
                # checkpoint the simulator reset it to, and the prior would
                # have been the one 3.3 m further back).
                lap = self._lap_length(); s_last = self._s_of(lg[0], lg[1])
                for (cx, cy, _), cs in zip(self.checkpoints, self._cp_s):
                    back = pick_back(s_last, cs, lap)
                    if back is None:
                        continue                      # unusable centreline
                    if back <= 25.0 and (best is None or back < best[0]):
                        best = (back, cx, cy)
            else:
                for cx, cy, cyaw in self.checkpoints:            # no centreline: heading test
                    dx, dy = lg[0] - cx, lg[1] - cy
                    dist = math.hypot(dx, dy)
                    ahead = dx * math.cos(cyaw) + dy * math.sin(cyaw)
                    if 0.0 <= ahead <= 25.0 and dist <= 25.0 and (best is None or dist < best[0]):
                        best = (dist, cx, cy)
            if best is not None:
                return best[1], best[2], yaw, 0.15, 5.0, self.recover_tol, f'checkpoint {best[0]:.1f} m behind the last pose along the track'
            # No data for this stretch: the reset is somewhere 0.15-2.8 m behind
            # the contact. Seed the mean, spread the particles over the range, and
            # accept wherever AMCL converges within it (tolerance 3 m, plus the
            # pos-std check in _recover_ok) instead of demanding it land on the seed.
            x = lg[0] - self.recover_back * math.cos(lg[2])
            y = lg[1] - self.recover_back * math.sin(lg[2])
            return x, y, yaw, self.recover_local_std, 15.0, 3.0, f'no known checkpoint behind the last pose; {self.recover_back:.1f} m back along it, wide'
        return None

    def _recover_ok(self):
        """Extra acceptance for a RECOVERY seed: AMCL must also have converged."""
        if self.rec_state != 'seed':
            return True
        return self.pos_std is not None and self.pos_std <= self.recover_std_ok

    def _tick_recover(self, now):
        if self.rec_state == 'watch':
            return
        if self.rec_state == 'flagged':
            prior = self._recovery_prior()
            self._rec_t0 = now
            self.recoveries += 1
            self._announce(False)
            self._send(0.0, 0.0)
            if prior is None:
                self.get_logger().error('reset before any trusted pose; searching the whole map')
                self._recover_global('no prior available')
                return
            x, y, yaw, std_m, std_deg, tol_m, why = prior
            self.truth_pos, self.truth_quat = (x, y), (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
            self.seed_std_m, self.seed_std_deg = std_m, std_deg
            self.tol_m, self.settle_s, self.max_attempts = tol_m, self.recover_settle, self.recover_attempts
            self.seeded_at, self.attempts, self.est_err, self.est_dyaw = None, 0, None, None
            self.rec_state = 'seed'
            self.get_logger().warn(
                f'recovery #{self.recoveries}: follower held; prior ({x:+.2f}, {y:+.2f}, '
                f'{math.degrees(yaw):+.0f} deg) -- {why}')
            return
        if self.rec_state == 'seed':
            # Parked while the seed is confirmed, and SAID so every tick: the
            # bridge keeps the last command, and a single zero at the flag lost
            # a race against the follower's final control tick (v2_L875_w28:
            # 0.17 throttle held, 4.2 m/s into the hairpin-2 wall).
            self._send(0.0, 0.0)
            self._tick_truth(now)             # seed -> confirm -> _finish/_fail, intercepted above
            return
        if self.rec_state == 'global':
            self._tick_global(now)
            return
        if self.rec_state == 'creep':
            if self._reset_flag is not None and self._reset_flag > self._creep_t0:
                self.rec_state = 'flagged'    # hit again while creeping: start over
                return
            if now - self._creep_t0 < self.recover_creep and self.scan is not None:
                self._drive()
                return
            self.rec_state = 'watch'
            self._announce(True)
            self.get_logger().info(
                f'recovery #{self.recoveries} complete in {now - self._rec_t0:.1f} s; follower resumes')

    def _recover_confirmed(self, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f'recovery: {message}')
        self.rec_state, self._creep_t0, self.distance = 'creep', now, 0.0
        if self.est is not None:
            self.last_good = self.est
            # Where the car actually was after the reset: a checkpoint. On a new
            # track this is how the table gets filled -- paste into frames.TRACKS.
            x, y, yaw = self.est
            self.get_logger().info(
                f"CHECKPOINT CANDIDATE (converged after reset): ('{x:.3f}', '{y:.3f}', '{yaw:.3f}')")

    def _recover_global(self, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().warn(f'recovery: prior not adopted ({message}); searching the whole map')
        if self.truth_pos is not None and self.last_good is not None:
            self.get_logger().warn(
                f'UNMATCHED RESET near ({self.last_good[0]:+.2f}, {self.last_good[1]:+.2f}): if the car '
                'is at a checkpoint not in frames.TRACKS, add it from this run\'s ground truth')
        self.rec_state, self.t0, self.scattered, self.distance = 'global', now, False, 0.0
        self.timeout = self.recover_global_timeout

    def _recover_giveup(self, message):
        self.get_logger().error(
            f'recovery: global search failed too ({message}); resuming on the current estimate, '
            'which AMCL may still correct while driving. Parked forever loses the race for certain.')
        self.rec_state = 'watch'
        self._announce(True)

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
        if self.rec_state in ('seed', 'global'):
            self._recover_confirmed(message)
            return
        self.finished = True
        self.ready = True
        self._send(0.0, 0.0)
        self._release_truth()
        self._announce(True)
        self.get_logger().info(message)
        self.get_logger().info('/localization_ready = true, follower may take over')
        if self.recover:
            self.last_good = self.seed_pose or self.est
            self.rec_state = 'watch'
            self.get_logger().info(
                f'recovery armed: watching encoders + IMU for a reset; '
                f'{len(self.checkpoints)} checkpoint(s) known for this track')

    def _fail(self, message):
        """Give up. By default do NOT let the follower drive on this pose."""
        if self.rec_state == 'seed':
            self._recover_global(message)
            return
        if self.rec_state == 'global':
            self._recover_giveup(message)
            return
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
            if self.rec_state in ('seed', 'global', 'creep'):
                self.get_logger().info(f'recovery #{self.recoveries}: {self.rec_state}, attempt {self.attempts}')
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
        if self.mode == 'truth' and self.seeded_at is None:
            self.get_logger().info(
                f'waiting for ground truth: ips={"ok" if self.truth_pos else "--"} '
                f'imu={"ok" if self.truth_quat else "--"}')
            return
        if self.mode == 'truth':
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
