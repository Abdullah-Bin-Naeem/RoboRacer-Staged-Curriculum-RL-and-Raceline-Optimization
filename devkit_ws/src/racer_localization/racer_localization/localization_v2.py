#!/usr/bin/env python3

"""Localizer C: segmented scan-to-map localization. Publishes map -> odom.

    ros2 launch racer_localization v2.launch.py
    ros2 launch racer_bringup race.launch.py localizer:=v2

The third interchangeable localizer beside AMCL and slam_toolbox, behind the
same frames, the same bootstrap and the same logger. `odom -> roboracer_1`
stays dead_reckoning's. This node owns exactly one thing: the map -> odom
correction, which on this stack is a pure TRANSLATION (dead_reckoning carries
the IMU's absolute heading, and the map is world-aligned; log_localization's
m2o_yaw_deg column is the invariant).

DIVIDE AND RULE
---------------
Measured 2026-09-19 (raceline/FINDINGS.md): AMCL's cross-track error is 1.6 cm,
at the floor the geometry allows, and its along-track error is 0.32-0.51 m
p90 -- built on the long straight, where the scan carries ZERO along-track
information (centreline s 26.7-35.9: information 0.0-2.6 along against
370-450 across), and made worse by the filter itself: AMCL jumps map->odom by
up to 0.46 m in ONE sample there, against a 0.09-0.13 m wall margin. Dead
reckoning over that straight over-reads about 2 %, so some of the drift is real
and unavoidable. So the track is divided by what the LiDAR can observe
(tools/segment_track.py, per track, from the map) and each part gets a rule:

    STRAIGHT_BLIND   nothing forced: the information matrix already makes the
                     along correction vanish where the scan cannot see along,
                     and shadow run 1 measured that forcing the gain to 0 there
                     is WORSE (along p90 0.429 against 0.316). What this mode
                     carries is the tightest along rate limit, and the guard.
    APPROACH         the first metres after a blind stretch: whatever the
                     odometry drifted comes out here, rate-limited so it is a
                     slope rather than a step, and gone before the braking point.
    CORNER           full correction, the tightest rate limit and the strictest
                     gates: the wall margin is smallest here.
    TRANSIT          full correction, normal gains.

The table is a lookup by nearest centreline point (direction-free), and it is
now a LABEL, not a controller: both of its levers were measured to be stale
proxies for what the live information matrix already knows. Forcing the along
gain to 0 on the blind straight was worse than not (0.429 vs 0.316 p90), and a
per-mode rate limit throttled the correction at hairpin 1 while the table still
said "blind" and the information said otherwise -- that crashed race run 1.
What the table earns its place with is analyze_run.py's per-mode error
breakdown, which is how both bugs were found. `blind_along_info` is the real
floor: below it the along correction is dropped whatever anything else says.

THE ESTIMATE (v2_filter.V2Filter, ROS-free; the replay tool runs the same code)
    predict   P += R(theta) diag(q_along, q_cross) R(theta)^T * ds
    measure   scan_matcher.match(prior, scan) -> (dx, dy), H (2x2 information)
    clamp     each axis of the innovation against gate_sigma*sigma + gate_floor,
              in the car frame -- clamped, never dropped, or a large error locks
              the correction out (it did: cross 0.06 -> 0.43 m, shadow run 1)
    fuse      Omega = P^-1 + G H G,  c += Omega^-1 (G H G) delta
              G = the mode's gain in the car frame; a rank-one H on the straight
              updates cross-track only, by construction
    limit     the PUBLISHED correction moves toward c at the mode's rate [m/s];
              the remainder is carried, never dropped

STARTUP. On the first scan the node seeds itself from `initial_x`/`initial_y`
(empty = the track's registered spawn) and starts publishing map->odom, exactly
as slam_toolbox starts on map_start_pose. It must: localization_bootstrap's
ready_check:=tf waits for map->odom to APPEAR before it sends the real seed, so
a localizer that publishes nothing until seeded deadlocks against it -- the
first `localizer:=v2` run sat in "waiting for TF map -> odom" for the full 30 s
timeout and the car never moved. map->odom existing therefore does NOT mean the
pose is confirmed; /localization_ready is the bootstrap's to latch, and the
follower waits on that.

INTERFACES (AMCL-shaped, so localization_bootstrap needs no change)
    /initialpose                       PoseWithCovarianceStamped   seed
    /request_nomotion_update           std_srvs/Empty              match now
    /reinitialize_global_localization  std_srvs/Empty              2-D search at the IMU yaw
    /map                               OccupancyGrid, latched      the grid (no map_server here)
    ~/pose                             PoseWithCovarianceStamped   estimate + posterior cov
    ~/status                           Float32MultiArray           v2_status.STATUS_FIELDS
    /tf  map -> odom                   unless shadow:=true

shadow:=true runs everything but the TF broadcast and /map, so this node can
be measured against AMCL on identical live data while AMCL owns the tree.

Race-legal: lidar against a pre-built map, odometry from dead_reckoning's TF.
No restricted topic is read here, in any mode.
"""

import math
import os
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import Empty

from racer_common import frames
from racer_localization.scan_matcher import LikelihoodField, scan_to_points
from racer_localization.v2_filter import Segments, V2Filter
from racer_localization.v2_status import MODES, REJECT_CODES, STATUS_FIELDS

QOS = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)
# /map is latched: a late RViz or bootstrap still gets it.
QOS_MAP = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LocalizationV2(Node):

    def __init__(self):
        super().__init__('localization_v2')
        p = self.declare_parameter
        p('track', '')
        p('map_yaml', '')                      # '' = the track's track_clean.yaml
        p('segments_csv', '')                  # '' = raceline/<track>/segments.csv
        p('scan_topic', f'{frames.NS}/lidar')
        p('map_frame', frames.MAP)
        p('odom_frame', frames.ODOM)
        p('base_frame', frames.BASE)
        p('lidar_x', float(frames.LIDAR_XYZ[0]))
        p('lidar_y', float(frames.LIDAR_XYZ[1]))
        # Fallback prior, exactly slam_toolbox's map_start_pose. Empty = the
        # track's registered spawn. This is what breaks the handshake deadlock:
        # localization_bootstrap's ready_check:=tf waits for map->odom BEFORE it
        # sends the seed, so a localizer that publishes nothing until it is
        # seeded waits for the bootstrap while the bootstrap waits for it. The
        # first run of localizer:=v2 sat there for the full 30 s timeout.
        p('initial_x', '')
        p('initial_y', '')
        p('shadow', False)                     # publish pose/status, never TF or /map
        p('publish_rate', 100.0)               # map->odom republish [Hz]
        p('transform_tolerance', 0.10)         # post-date like dead_reckoning (0.02) and AMCL (0.5)
        # ---- matcher ---- (see config/localization_v2.yaml for the measurements)
        p('beam_stride', 3)
        p('max_use_m', 9.5)
        p('sigma', 0.12)
        p('sigma_start', 0.40)
        p('max_iters', 8)
        p('inlier_m', 0.25)
        p('min_inlier_frac', 0.55)
        p('max_resid_m', 0.16)
        p('max_step_m', 1.00)
        p('yaw_dof', False)
        p('min_clearance_m', 0.18)
        # ---- fusion ----
        p('q_along', 0.06 ** 2)      # the measured ~2 % DR over-read; see the yaml
        p('q_cross', 0.005 ** 2)
        p('p_init', 0.30 ** 2)
        p('blind_along_info', 5.0)
        p('gain_along', [1.0, 1.0, 1.0, 1.0])   # measured: forcing 0 on the blind straight is worse
        p('gain_cross', [1.0, 1.0, 1.0, 1.0])
        p('rate_m_s', [1.00, 1.00, 1.00, 1.00])  # uniform, measured; see the yaml
        p('rate_cross_m_s', [0.20, 0.20, 0.20, 0.20])   # the cross axis is slower; see the yaml
        p('rate_cross_gap_m', 0.06)                     # ...except when relocalising
        p('mode_hysteresis_m', 0.30)
        p('gate_sigma', 4.0)                   # innovation clamp: gate_sigma*sigma + gate_floor, per axis
        p('gate_floor_m', 0.12)
        p('p_floor_along_m', 0.03)             # the filter never claims better than this
        p('p_floor_cross_m', 0.025)
        # True = estimate the scale from corner fixes. False now FREEZES it at
        # scale_init and STILL APPLIES it; it used to disable the compensation
        # altogether, so scale_init had no effect in that mode (2.77 % and
        # 3.50 % scored identically, both = none, gt_dist along p90 0.333
        # against 0.173 with it). Frozen-at-measured is the useful safe mode
        # and offline it matches the estimator (0.173 vs 0.174).
        p('est_scale', True)                   # estimate the odometry scale error; see v2_filter
        p('q_scale', 2e-7)
        p('p_scale_init', 0.03 ** 2)
        p('scale_max', 0.030)   # the top of the measured DR over-read; it binds, see v2_filter
        p('scale_init', 0.025)     # measured DR over-read; the estimator starts here
        p('k_accel', 0.0096)       # accel-dependent slip; replayed 2026-09-19, see yaml
        p('odom_accel_max', 0.0)   # wheelspin cap [m/s^2], OFF: replay rejected it, see v2_filter
        p('odom_speed_window_s', 0.10)
        p('odom_spin_margin', 0.4)
        p('beam_sigma_m', 0.03)                # per-beam endpoint noise: puts `info` in 1/m^2
        p('info_scale', 1.0)
        p('status_every', 1)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        track = str(g('track')) or None
        map_yaml = str(g('map_yaml')) or frames.map_yaml(track)
        seg_csv = str(g('segments_csv')) or os.path.join(frames.raceline_dir(track), 'segments.csv')

        self.map_frame, self.odom_frame, self.base_frame = (str(g('map_frame')), str(g('odom_frame')),
                                                            str(g('base_frame')))
        self.lidar_xy = (float(g('lidar_x')), float(g('lidar_y')))
        self.shadow = bool(g('shadow'))
        self.tf_tol = float(g('transform_tolerance'))
        self.stride = int(g('beam_stride'))
        self.max_use = float(g('max_use_m'))
        self.status_every = max(1, int(g('status_every')))

        t0 = time.time()
        self.field = LikelihoodField(map_yaml)
        segments = Segments(seg_csv)
        self.filt = V2Filter(
            self.field, segments,
            matcher_kwargs=dict(sigma=float(g('sigma')), sigma_start=float(g('sigma_start')),
                                max_iters=int(g('max_iters')), inlier_m=float(g('inlier_m')),
                                min_inlier_frac=float(g('min_inlier_frac')),
                                max_resid_m=float(g('max_resid_m')), max_step_m=float(g('max_step_m')),
                                yaw_dof=bool(g('yaw_dof'))),
            q_along=float(g('q_along')), q_cross=float(g('q_cross')), p_init=float(g('p_init')),
            blind_along_info=float(g('blind_along_info')),
            gain_along=[float(v) for v in g('gain_along')], gain_cross=[float(v) for v in g('gain_cross')],
            rate_m_s=[float(v) for v in g('rate_m_s')],
            rate_cross_m_s=[float(v) for v in g('rate_cross_m_s')],
            rate_cross_gap_m=float(g('rate_cross_gap_m')),
            mode_hysteresis_m=float(g('mode_hysteresis_m')),
            beam_sigma_m=float(g('beam_sigma_m')), info_scale=float(g('info_scale')),
            min_clearance_m=float(g('min_clearance_m')),
            gate_sigma=float(g('gate_sigma')), gate_floor_m=float(g('gate_floor_m')),
            p_floor_along_m=float(g('p_floor_along_m')), p_floor_cross_m=float(g('p_floor_cross_m')),
            est_scale=bool(g('est_scale')), q_scale=float(g('q_scale')),
            p_scale_init=float(g('p_scale_init')), scale_max=float(g('scale_max')),
            scale_init=float(g('scale_init')), k_accel=float(g('k_accel')),
            odom_accel_max=float(g('odom_accel_max')), odom_speed_window_s=float(g('odom_speed_window_s')),
            odom_spin_margin=float(g('odom_spin_margin')))
        self.get_logger().info(
            f'map {os.path.basename(map_yaml)} {self.field.w}x{self.field.h} @ {self.field.res} m, '
            f'field built in {time.time() - t0:.2f} s; stride {self.stride}, sigma {g("sigma")}'
            + ('; SHADOW: no TF, no /map' if self.shadow else ''))
        if len(segments):
            self.get_logger().info(f'segments: {len(segments)} points from {seg_csv}: {segments.counts()}')
        else:
            self.get_logger().warn(f'no segment table at {seg_csv!r}: every point is TRANSIT and only '
                                   'the live observability guard limits the along gain. Run '
                                   'tools/segment_track.py --track <track>.')

        sp = frames.spawn(track)
        ix, iy = str(g('initial_x')).strip(), str(g('initial_y')).strip()
        self.initial_xy = (float(ix or sp[0]), float(iy or sp[1]))
        self._n = 0
        self._last_stamp = None
        self._pending_global = False
        self._last_o = None

        self.tfb = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '~/pose', QOS)
        self.pub_status = self.create_publisher(Float32MultiArray, '~/status', QOS)
        # No nav2 map_server in this stack, and localization_bootstrap confirms
        # a seed by scoring the scan against /map (near_wall_mask), so the grid
        # is published from here -- once, latched.
        if not self.shadow:
            self.pub_map = self.create_publisher(OccupancyGrid, '/map', QOS_MAP)
            self.pub_map.publish(self._grid_msg())
        self.create_subscription(LaserScan, str(g('scan_topic')), self._cb_scan, QOS)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._cb_initialpose, QOS)
        # The bootstrap's latch. Low = the follower is held (startup, or a
        # recovery after a wall contact): the rate limiter steps aside so the
        # estimate the bootstrap confirms is the filter's, not a lagged copy.
        self.create_subscription(Bool, '/localization_ready', self._cb_ready, QOS_MAP)
        self.create_service(Empty, '/request_nomotion_update', self._srv_nudge)
        self.create_service(Empty, '/reinitialize_global_localization', self._srv_global)
        self.create_timer(1.0 / float(g('publish_rate')), self._tick_tf)

    def _grid_msg(self):
        """The PGM as nav_msgs/OccupancyGrid, row 0 at the BOTTOM as ROS wants
        it (the PGM has it at the top): 100 occupied, 0 free, -1 unknown."""
        f = self.field
        m = OccupancyGrid()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = f.res
        m.info.width, m.info.height = f.w, f.h
        m.info.origin.position.x, m.info.origin.position.y = f.ox, f.oy
        m.info.origin.orientation.w = 1.0
        data = np.full((f.h, f.w), -1, dtype=np.int8)
        data[f.occupied] = 100
        data[f.free] = 0
        m.data = data[::-1].reshape(-1).tolist()
        return m

    # ---- odometry (dead_reckoning's TF) ----------------------------------
    def _odom_at(self, stamp):
        """odom -> base at `stamp`, latest if the stamp is not covered."""
        for at in (stamp, Time()):
            try:
                t = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, at)
                v = t.transform.translation
                return v.x, v.y, yaw_from_quat(t.transform.rotation)
            except Exception:                            # noqa: BLE001
                continue
        return None

    # ---- seed / services --------------------------------------------------
    def _cb_initialpose(self, msg):
        o = self._odom_at(Time.from_msg(msg.header.stamp))
        if o is None:
            self.get_logger().warn('initialpose: no odom->base yet, seed dropped')
            return
        cov = msg.pose.covariance
        std = math.sqrt(max(cov[0], cov[7], 1e-4))
        self.filt.seed((msg.pose.pose.position.x, msg.pose.pose.position.y), o, std_m=std)
        self.get_logger().info(
            f'seeded: map->odom ({self.filt.c[0]:+.3f}, {self.filt.c[1]:+.3f}) std {std:.2f} m; the '
            f'seed yaw {math.degrees(yaw_from_quat(msg.pose.pose.orientation)):+.1f} deg is ignored '
            '(odom carries the IMU heading)')

    def _cb_ready(self, msg):
        self.filt.held = not bool(msg.data)

    def _srv_nudge(self, req, res):
        return res                                       # every scan is already a match

    def _srv_global(self, req, res):
        self._pending_global = True
        return res

    # ---- the scan ---------------------------------------------------------
    def _cb_scan(self, msg):
        t_start = time.perf_counter()
        stamp = Time.from_msg(msg.header.stamp)
        o = self._odom_at(stamp)
        if o is None:
            self._publish_status(None, 'no_odom', t_start)
            return
        pts = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                             range_min=msg.range_min, range_max=msg.range_max,
                             lidar_xy=self.lidar_xy, max_use_m=self.max_use, stride=self.stride)
        if self.filt.c is None:
            # Start on the fallback prior so map->odom exists and the bootstrap
            # can hand us a real seed. p_init is wide, so the first accepted
            # match dominates it; /localization_ready is still the bootstrap's
            # to latch, so the follower does not drive on this.
            self.filt.seed(self.initial_xy, o, std_m=math.sqrt(self.filt.p_init))
            self.get_logger().info(
                f'starting on the fallback prior ({self.initial_xy[0]:.3f}, {self.initial_xy[1]:.3f}) '
                f'+-{math.sqrt(self.filt.p_init):.2f} m -- publishing map->odom so the bootstrap '
                'can seed. NOT a confirmed pose.')
        if self._pending_global:
            self._pending_global = False
            t = time.perf_counter()
            r = self.filt.global_search(pts, o)
            ms = (time.perf_counter() - t) * 1e3
            if self.filt.c is None or not r.ok:
                self.get_logger().error(f'global search failed ({r.reason}) in {ms:.0f} ms')
            else:
                pose = self.filt.pose(o)
                self.get_logger().warn(f'global search: base at ({pose[0]:.2f}, {pose[1]:.2f}), '
                                       f'inliers {r.inlier_frac:.2f}, {ms:.0f} ms')

        dt = 0.0
        if self._last_stamp is not None:
            dt = max(0.0, (stamp - self._last_stamp).nanoseconds * 1e-9)
        self._last_stamp = stamp
        self._last_o = o

        out = self.filt.step(o, dt, pts, t_start=t_start)
        self._n += 1
        self._publish_status(out, None, t_start)
        if self.filt.pub is not None:
            self._publish_pose(stamp, o)

    # ---- outputs ----------------------------------------------------------
    def _publish_pose(self, stamp, o):
        x, y, yaw = self.filt.pose(o)
        m = PoseWithCovarianceStamped()
        m.header.stamp = stamp.to_msg()
        m.header.frame_id = self.map_frame
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        P = self.filt.P
        cov = m.pose.covariance
        cov[0], cov[1], cov[6], cov[7] = float(P[0, 0]), float(P[0, 1]), float(P[1, 0]), float(P[1, 1])
        cov[35] = math.radians(1.0) ** 2      # the IMU's heading: not estimated here
        self.pub_pose.publish(m)

    def _tick_tf(self):
        if self.filt.pub is None or self.shadow:
            return
        tf = TransformStamped()
        now = self.get_clock().now()
        tf.header.stamp = (now + Duration(seconds=self.tf_tol)).to_msg() if self.tf_tol > 0 else now.to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x = float(self.filt.pub[0])
        tf.transform.translation.y = float(self.filt.pub[1])
        tf.transform.rotation.w = 1.0          # pure translation, by construction
        self.tfb.sendTransform(tf)

    def _publish_status(self, out, reject, t_start):
        if self._n % self.status_every:
            return
        if out is None:                       # no odometry yet: an empty status with the reason
            out = dict.fromkeys(STATUS_FIELDS, 0.0)
            out['mode'] = float(self.filt.mode)
            out['resid_m'] = -1.0
            out['reject'] = float(REJECT_CODES.index(reject))
            out['compute_ms'] = (time.perf_counter() - t_start) * 1e3
        msg = Float32MultiArray()
        msg.data = self.filt.status_list(out)
        self.pub_status.publish(msg)
        if self._n % 200 == 0 and self._n:
            self.get_logger().info(
                f'{MODES[int(out["mode"])]:14s} info al {out["along_info"]:6.1f} cr {out["cross_info"]:6.1f} | '
                f'meas ({out["dx_meas"]:+.3f},{out["dy_meas"]:+.3f}) appl ({out["dx_appl"]:+.3f},'
                f'{out["dy_appl"]:+.3f}) pending {out["pending_m"]:.3f} | inl {out["inlier_frac"]:.2f} '
                f'res {out["resid_m"]:.3f} yaw_hint {out["yaw_hint_deg"]:+.2f} | '
                f'sig al {out["sig_along"]:.3f} cr {out["sig_cross"]:.3f} | {out["compute_ms"]:.1f} ms '
                f'{REJECT_CODES[int(out["reject"])]}')


def main():
    rclpy.init()
    node = LocalizationV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # A second SIGINT (launch sends one, the terminal another) lands inside
        # destroy_node and prints a traceback that reads like a crash. It is a
        # shutdown, so swallow it.
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
