#!/usr/bin/env python3

"""localization_v2's estimator, ROS-free: predict, mode, guard, fuse, rate-limit.

The node (localization_v2.py) is the ROS glue around this class; the offline
harness (tools/replay_localization_v2.py) drives the SAME class over a recorded
run. One implementation, so a number tuned offline is the number that races.

State: c = (cx, cy), the map->odom translation, with covariance P. The
rotation of map->odom is identity by construction (see the node docstring).
`pub` is the rate-limited version of c that actually goes out on /tf.

    step(o, dt, pts) -> dict in v2_status.STATUS_FIELDS order (+ 'ok')

with o = (x, y, yaw) of odom->base at the scan stamp, dt seconds since the
previous scan, pts = scan endpoints in the base frame (scan_matcher.scan_to_points).
"""

import math

import numpy as np

from racer_localization.scan_matcher import ScanMatcher
from racer_localization.v2_status import MODE_ID, MODES, REJECT_CODES, STATUS_FIELDS


def rot(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


class Segments:
    """raceline/<track>/segments.csv: a mode per centreline point, looked up by
    nearest point (direction-free). Written by tools/segment_track.py. Absent:
    every point is TRANSIT and the live guard alone limits the along gain."""

    def __init__(self, path=None, rows=None):
        self.xy = None
        self.mode = None
        if rows is None and path:
            try:
                rows = [l.split(',') for l in open(path) if l.strip() and not l.startswith('#')]
            except OSError:
                rows = None
        if rows:
            self.xy = np.array([[float(r[1]), float(r[2])] for r in rows])
            self.mode = np.array([MODE_ID.get(r[6].strip(), MODE_ID['TRANSIT']) for r in rows])

    def __len__(self):
        return 0 if self.mode is None else len(self.mode)

    def counts(self):
        return {m: int((self.mode == i).sum()) for i, m in enumerate(MODES)} if len(self) else {}

    def lookup(self, x, y):
        if self.xy is None:
            return MODE_ID['TRANSIT']
        i = int(np.argmin((self.xy[:, 0] - x) ** 2 + (self.xy[:, 1] - y) ** 2))
        return int(self.mode[i])


class V2Filter:

    def __init__(self, field, segments, *, matcher_kwargs=None, q_along=0.06 ** 2,
                 q_cross=0.005 ** 2, p_init=0.30 ** 2, blind_along_info=5.0,
                 gain_along=(1.0, 1.0, 1.0, 1.0), gain_cross=(1.0, 1.0, 1.0, 1.0),
                 rate_m_s=(1.00, 1.00, 1.00, 1.00), rate_cross_m_s=None,
                 rate_cross_gap_m=0.06, mode_hysteresis_m=0.30,
                 beam_sigma_m=0.03, info_scale=1.0, min_clearance_m=0.18,
                 gate_sigma=4.0, gate_floor_m=0.12,
                 p_floor_along_m=0.03, p_floor_cross_m=0.025,
                 est_scale=True, q_scale=2e-7, p_scale_init=0.03 ** 2, scale_max=0.030,
                 scale_init=0.025, k_accel=0.0, accel_tau_s=0.25, accel_max=12.0,
                 odom_accel_max=0.0, odom_speed_window_s=0.10, odom_spin_margin=0.4):
        self.field = field
        self.matcher = ScanMatcher(field, **(matcher_kwargs or {}))
        self.segments = segments
        self.q_along, self.q_cross = float(q_along), float(q_cross)
        self.p_init = float(p_init)
        self.blind_info = float(blind_along_info)
        self.gain_along = [float(v) for v in gain_along]
        self.gain_cross = [float(v) for v in gain_cross]
        self.rate = [float(v) for v in rate_m_s]
        # None = the same limit on both axes: decoupling alone, no retune.
        self.rate_cross = ([float(v) for v in rate_cross_m_s]
                           if rate_cross_m_s is not None else list(self.rate))
        self.rate_cross_gap = float(rate_cross_gap_m)
        self.hyst_m = float(mode_hysteresis_m)
        # UNITS. The matcher's `info` is sum(w g.g) with |g| ~ 1: a count of
        # beams pinning each axis, dimensionless. P is in m^2. Adding a count to
        # 1/m^2 under-weighs the measurement by 1/sigma_beam^2 -- the first
        # synthetic replay took 2 % of a correction the matcher had right, for
        # exactly that reason. sigma_beam is the endpoint noise per beam:
        # fit_true (endpoint-to-wall at the TRUE pose) measures 0.010-0.019 m
        # on the real runs, plus the 0.025 m cell; 0.03 m. info_scale is a
        # further trust multiplier on top, 1 = the field is the truth.
        self.beam_sigma = float(beam_sigma_m)
        self.info_scale = float(info_scale) / (self.beam_sigma ** 2)
        self.min_clear = float(min_clearance_m)
        # Per-axis innovation clamp and covariance floor; see the fuse step.
        self.gate_sigma = float(gate_sigma)
        self.gate_floor = float(gate_floor_m)
        self.p_floor_along = float(p_floor_along_m)
        self.p_floor_cross = float(p_floor_cross_m)
        # ODOMETRY SCALE ERROR, the third state. Dead reckoning over-reads 2.0-3.4 %
        # on this stack (measured across six runs), and the blind straight is
        # ~16 m, so it arrives at hairpin 1 with 0.3-0.5 m of along error that no
        # scan can see. It IS observable, though, every time a corner pins the
        # along axis: the correction accepted there, over the distance travelled
        # since the last pin, measures the scale. So estimate it and spend it on
        # the straight, where it is the whole error. q_scale is small on purpose
        # -- the scale is a slowly varying property of the tyre model, not a
        # per-scan quantity.
        #
        # scale_max BINDS, and that is deliberate. The estimator always runs to
        # its bound (it reaches +2.99 % at a 3.0 % cap, +5.98 % at a 6 % one), so
        # it is absorbing more than odometry scale -- some of the along error
        # grows with distance for other reasons, and possibly the /ips reference
        # itself lags at speed. So the bound is set from PHYSICS, not from what
        # flatters the metric: dead reckoning measured 1.025-1.034 of true
        # distance over six runs, so 3.0 % is the top of the plausible range.
        # Bounded there it is a along-track bias compensator with a principled
        # ceiling rather than a scale identifier, and it is no worse than the
        # unbounded version (lv_slow_dist: 0.165 m at 3 % against 0.204 at 6 %).
        self.est_scale = bool(est_scale)
        self.q_scale = float(q_scale)
        self.p_scale_init = float(p_scale_init)
        self.scale_max = float(scale_max)
        # START FROM WHAT IS ALREADY MEASURED, not from zero. The estimator is
        # only observable at corners, so from k=0 it needs ~100 m to converge --
        # and the crashes happen at 26 m, on lap 1. Dead reckoning over-read
        # 2.0-3.4 % on every logged run (o2b vs truth, six runs), so 2.2 % is a
        # far better prior than 0, and the filter refines it from there.
        self.k_scale = float(np.clip(scale_init, -self.scale_max, self.scale_max))
        # SLIP IS NOT A CONSTANT -- it is linear in longitudinal acceleration,
        # and that is why the scalar above always ran to whatever bound it was
        # given. Throttle commands a WHEEL speed, so the encoder reads the
        # wheel; the wheel leads the ground by the slip ratio, and slip ratio
        # is what generates tyre force, so it is proportional to a. Measured on
        # 13059 noise-immune chords over gt_dist + lv_race_2 + lv_slow_dist:
        #
        #     k(%) = 0.961 * a + 2.766      R2 0.486
        #     braking a<-3: -1.1 %   steady |a|<1: +2.1 %   accel a>+3: +6.6 %
        #
        # An 8-point swing, consistent to +-0.5 pt across all three runs. It is
        # also exactly backwards for a single scalar: the along axis is
        # observable at the CORNERS, where the car is braking and k is ~-1 %,
        # and the error has to be spent on the STRAIGHT, where the car is at
        # full throttle and k is ~+6.6 %. The estimator learned the corner
        # value, applied it to the straight, and saturated trying to split the
        # difference. k_accel carries the known slope so the estimated state
        # only has to carry the steady-state intercept (drag balance, ~2.8 %).
        # k_accel=0 restores the old constant-scale behaviour exactly.
        self.k_accel = float(k_accel)
        self.accel_tau = float(accel_tau_s)
        self.accel_max = float(accel_max)
        self._v_prev = None
        self._accel = 0.0
        # WHEELSPIN CAP. The car cannot accelerate faster than the tire's peak
        # longitudinal force allows (VEHICLE_MODEL: 4.55 m/s^2 robust, ~5 at
        # the peak), but the encoder reads the WHEEL, which the throttle spins
        # up as fast as it likes. On v2_L850h725 lap 2 (2026-09-19) the launch
        # out of hairpin 1 read 20-30 % more distance than the car covered for
        # 0.25 s -- 0.24 m of along error in a stretch where the scan cannot
        # see along -- and the car hit the wall at s 27.4 with the estimate
        # 0.39 m ahead. So the odometry-implied speed, smoothed over
        # odom_speed_window_s (a per-scan rate scatters +-15 %), may not rise
        # faster than odom_accel_max: the excess is wheelspin and is not
        # integrated. Braking and steady speed are untouched. 0 disables.
        #
        # OFF (0) BY DEFAULT: replayed over seven recorded runs it fixed the
        # run it was built on (v2_L875h725 along p90 0.220 -> 0.178) and
        # WRECKED four others (gt_dist 0.173 -> 1.65, lv_slow_dist 0.153 ->
        # 1.59, lv_race_2 0.157 -> 0.74) at 5, 6 and 7 m/s^2, with and without
        # the noise margin. The windowed wheel speed is not a clean enough
        # signal to gate on; the per-scan increments scatter 2x at 45 Hz
        # (pure_pursuit's enc_rate_window_s finding). Kept so the idea is
        # measured rather than remembered; do not enable without a replay
        # that passes on every run.
        self.odom_accel_max = float(odom_accel_max)
        self.odom_win = float(odom_speed_window_s)
        # Only an excess beyond this margin [m/s] is treated as spin: the
        # windowed wheel speed still scatters a few percent, and clipping that
        # scatter under-reads the whole lap (replayed: p90 along 0.17 -> 1.7 m
        # on gt_dist with no margin).
        self.odom_spin_margin = float(odom_spin_margin)
        self._ds_hist = []             # (ds, dt) of recent scans, for the windowed speed
        self._v_lim = None             # the ramp-limited car speed
        self.spin_m = 0.0              # metres of odometry discarded as wheelspin (for the log)

        # While /localization_ready is low the follower is not driving, so
        # the published correction may jump: the rate limit exists to protect
        # the steering, and during a recovery it only delays the pose the
        # bootstrap is trying to confirm (lv_fast_19_1: 1.85 m owed, paid at
        # 1 m/s). The node sets this from the bootstrap's latch.
        self.held = False
        self.c = None                  # None until seeded
        # 3x3: [cx, cy, k_scale]
        self.P = np.diag([self.p_init, self.p_init, self.p_scale_init])
        self.pub = None                # the rate-limited published correction
        self.mode = MODE_ID['TRANSIT']
        self._cand = self.mode
        self._travel = 0.0
        self._last_o = None
        self.mode_changes = []         # (x, y, from, to) for the log

    # ---- seeding ----------------------------------------------------------
    def seed(self, map_pose_xy, o, std_m=0.30):
        """map->base is `map_pose_xy` while odom->base is `o`: set c to match."""
        self.c = np.array([map_pose_xy[0] - o[0], map_pose_xy[1] - o[1]], dtype=float)
        self.pub = self.c.copy()       # a seed is applied at once, not ramped
        # A re-seed says nothing about the odometry scale, so k and its variance
        # survive it: after a wall reset the car is elsewhere, the tyres are not.
        self.P = np.diag([float(std_m) ** 2, float(std_m) ** 2, self.P[2, 2]])

    def global_search(self, pts, o):
        x, y, r = self.matcher.global_search(pts, o[2])
        if x is None:
            return r
        self.seed((x, y), o, std_m=0.10)
        return r

    def pose(self, o):
        """map->base implied by the PUBLISHED correction, or None."""
        if self.pub is None:
            return None
        return o[0] + self.pub[0], o[1] + self.pub[1], o[2]

    def sigmas(self, yaw):
        u = np.array([math.cos(yaw), math.sin(yaw)])
        v = np.array([-u[1], u[0]])
        Pc = self.P[:2, :2]
        return (math.sqrt(max(float(u @ Pc @ u), 0.0)),
                math.sqrt(max(float(v @ Pc @ v), 0.0)))

    # ---- one scan -----------------------------------------------------------
    def step(self, o, dt, pts, t_start=None):
        import time
        t0 = time.perf_counter() if t_start is None else t_start
        out = dict.fromkeys(STATUS_FIELDS, 0.0)
        out['mode'] = float(self.mode)
        out['resid_m'] = -1.0
        out['ok'] = False
        if self.c is None:
            out['reject'] = float(REJECT_CODES.index('not_seeded'))
            out['compute_ms'] = (time.perf_counter() - t0) * 1e3
            return out

        # -- predict: the prior grows along the heading with distance travelled
        ds = 0.0 if self._last_o is None else math.hypot(o[0] - self._last_o[0], o[1] - self._last_o[1])
        ds_raw = ds
        if self.odom_accel_max > 0.0 and dt > 1e-4:
            self._ds_hist.append((ds, dt))
            while len(self._ds_hist) > 1 and sum(h[1] for h in self._ds_hist) > self.odom_win:
                self._ds_hist.pop(0)
            v_win = sum(h[0] for h in self._ds_hist) / max(sum(h[1] for h in self._ds_hist), 1e-4)
            if self._v_lim is None:
                self._v_lim = v_win
            self._v_lim = min(v_win, self._v_lim + self.odom_accel_max * dt)
            if v_win > self._v_lim + self.odom_spin_margin:
                ds = ds * (self._v_lim + self.odom_spin_margin) / v_win
                self.spin_m += ds_raw - ds
        R = rot(o[2])
        u = np.array([math.cos(o[2]), math.sin(o[2])])
        # Odometry advanced ds*(1+k) while the car moved ds, so map->odom must
        # give back k*ds along the heading. F carries that dependence into the
        # covariance, which is what makes k observable from position fixes.
        # Longitudinal acceleration from the odometry itself (race-legal: the
        # same encoder stream dead_reckoning already integrates), low-passed
        # because a per-sample difference at 22 ms is mostly noise.
        if dt > 1e-4:
            v_now = ds / dt
            if self._v_prev is not None:
                a_raw = float(np.clip((v_now - self._v_prev) / dt, -self.accel_max, self.accel_max))
                w = min(1.0, dt / max(self.accel_tau, 1e-3))
                self._accel += w * (a_raw - self._accel)
            self._v_prev = v_now
        # APPLY the scale correction always; ESTIMATE it only when asked. These
        # used to be one flag, so est_scale:=false silently turned off slip
        # compensation altogether instead of freezing it at the measured value
        # (scale_init had no effect at all in that mode -- 2.77 % and 3.50 %
        # scored identically). Frozen-at-measured is the useful safe mode: the
        # compensation without the estimator's dynamics.
        k_eff = self.k_scale + self.k_accel * self._accel
        if ds_raw > 0.0:
            # odom advanced ds_raw; the car moved ds*(1-k_eff): give the rest back
            self.c = self.c - (k_eff * ds + (ds_raw - ds)) * u
        F = np.eye(3)
        if self.est_scale:
            F[0, 2] = -ds * u[0]
            F[1, 2] = -ds * u[1]
        Q = np.zeros((3, 3))
        Q[:2, :2] = R @ np.diag([self.q_along, self.q_cross]) @ R.T * ds
        Q[2, 2] = self.q_scale * ds if self.est_scale else 0.0
        self.P = F @ self.P @ F.T + Q
        self._last_o = o

        # -- mode: the table, with hysteresis in metres of travel
        pred = (o[0] + self.c[0], o[1] + self.c[1], o[2])
        cand = self.segments.lookup(pred[0], pred[1])
        if cand != self._cand:
            self._cand, self._travel = cand, 0.0
        else:
            self._travel += ds
        if cand != self.mode and self._travel >= self.hyst_m:
            self.mode_changes.append((pred[0], pred[1], self.mode, cand))
            self.mode = cand
        out['mode'] = float(self.mode)

        # -- measure
        r = self.matcher.match(pred, pts)
        along_info, cross_info = r.along_cross(o[2])
        out.update(along_info=along_info, cross_info=cross_info, dx_meas=r.dx, dy_meas=r.dy,
                   inlier_frac=r.inlier_frac, resid_m=r.resid if math.isfinite(r.resid) else -1.0,
                   yaw_hint_deg=math.degrees(r.yaw_hint), iters=float(r.iters))
        reject = 'ok'
        if not r.ok:
            reject = 'matcher'
        elif self.field.clearance(pred[0] + r.dx, pred[1] + r.dy) < self.min_clear:
            reject = 'clearance'

        if reject == 'ok':
            g_al = self.gain_along[self.mode]
            if along_info < self.blind_info and g_al > 0.0:
                g_al = 0.0                         # the guard: this scan says blind
                reject = 'guard'                   # along dropped, cross fused

            # PER-AXIS PLAUSIBILITY, in the car frame, against what the filter
            # itself claims to know. A fixed whole-step gate cannot do this job:
            # on the real shadow run (2026-09-19) dead reckoning drifted ~0.3 m
            # ALONG down the blind straight, the matcher asked for it back with
            # inliers 1.00 and residual 0.007, `max_step_m` 0.35 rejected the
            # whole match on the along component -- which the gain was about to
            # discard anyway -- and the CROSS correction died with it. The error
            # then grew, so the next step was bigger, so it was rejected harder:
            # cross went 0.06 -> 0.43 m through hairpin 1 while every scan fitted
            # perfectly. Clamp each axis instead of dropping the measurement, so
            # a large innovation slows the correction down but can never lock it
            # out. gate_floor covers the honest error the covariance cannot see
            # (map, extrinsic, timing).
            u = np.array([math.cos(o[2]), math.sin(o[2])])
            v = np.array([-u[1], u[0]])
            delta = np.array([r.dx, r.dy])
            d_al, d_cr = float(delta @ u), float(delta @ v)
            s_al, s_cr = self.sigmas(o[2])
            lim_al = self.gate_sigma * s_al + self.gate_floor
            lim_cr = self.gate_sigma * s_cr + self.gate_floor
            c_al = max(-lim_al, min(lim_al, d_al))
            c_cr = max(-lim_cr, min(lim_cr, d_cr))
            out['clamp_m'] = math.hypot(d_al - c_al, d_cr - c_cr)
            delta = c_al * u + c_cr * v

            # G is the gain in the car frame, rotated to the map. With gains of
            # 0/1 it is a projection and G H G is the information with the
            # dropped axis removed from rows and columns; (G H G) delta uses only
            # the kept component of the measurement.
            G = R @ np.diag([g_al, self.gain_cross[self.mode]]) @ R.T
            H = G @ (r.info * self.info_scale) @ G
            H = 0.5 * (H + H.T)
            H3 = np.zeros((3, 3))
            H3[:2, :2] = H
            try:
                P_post = np.linalg.inv(np.linalg.inv(self.P) + H3)
                # The STATE update is projected onto the gain subspace too. P
                # carries along/cross coupling in the map frame after turns, so
                # P_post (G H G) delta has an along component on a cross-only
                # measurement -- Kalman-consistent, and exactly the along-track
                # drift on the blind straight the rule forbids (synthetic
                # replay: 0.92 m of along walk there with gain_along = 0).
                dx = P_post @ np.concatenate([H @ delta, [0.0]])
                # Only the POSITION part is projected onto the gain subspace; the
                # scale correction is scalar and rides the cross-covariance, so a
                # cross-only fix does not move it.
                self.c = self.c + G @ dx[:2]
                if self.est_scale:
                    self.k_scale = float(np.clip(self.k_scale + dx[2], -self.scale_max, self.scale_max))
                # FLOOR THE COVARIANCE, in the car frame. The measurement model
                # treats beams as independent, so ~130 of them at 3 cm drive
                # sigma_cross to 2 mm -- the shadow run's posterior -- and the
                # filter then believes itself past any real error in the map,
                # the extrinsic or the timing, which makes the gate above
                # nonsense and the estimate deaf. Nothing here claims better
                # than the cell size.
                P_post = 0.5 * (P_post + P_post.T)
                P_car = R.T @ P_post[:2, :2] @ R
                P_car[0, 0] = max(P_car[0, 0], self.p_floor_along ** 2)
                P_car[1, 1] = max(P_car[1, 1], self.p_floor_cross ** 2)
                P_post[:2, :2] = R @ P_car @ R.T
                self.P = P_post
                out['ok'] = True
            except np.linalg.LinAlgError:
                reject = 'matcher'

        # -- rate-limit the published correction toward c; carry the rest
        if self.pub is None:
            self.pub = self.c.copy()
        # PER AXIS, in the car frame, so the two axes can carry DIFFERENT rates.
        # Scaling the whole vector by step_max/|gap| forced one rate on both,
        # and the along gap (0.1-0.3 m off the blind straight) always decided
        # it. Note what the measurement did NOT show: decoupling ALONE, at
        # equal rates, is slightly WORSE on cross (p90 0.030 -> 0.032), because
        # the shared limit was incidentally smoothing cross. The win is the
        # separate, slower cross rate that decoupling makes possible -- see
        # rate_cross_m_s in the yaml for the nine-run table.
        gap = self.c - self.pub
        n = float(np.linalg.norm(gap))
        u = np.array([math.cos(o[2]), math.sin(o[2])])
        v = np.array([-u[1], u[0]])
        appl = np.zeros(2)
        if n > 1e-9:
            if self.held:
                appl = gap
            elif dt > 0:
                lim_al = self.rate[self.mode] * dt
                g_al = float(gap @ u)
                g_cr = float(gap @ v)
                # The slow cross rate is for STEADY-STATE smoothness, where the
                # cross gap is a couple of centimetres. It must not also throttle
                # a genuine relocalisation: after a wall reset the gap is tens of
                # centimetres and 0.20 m/s would take a second to close it, which
                # is what made the two runs containing resets (lv_L750,
                # lv_fast_1) worse when the slow rate was applied unconditionally.
                # Above rate_cross_gap_m the along rate applies instead.
                lim_cr = (self.rate[self.mode] if abs(g_cr) > self.rate_cross_gap
                          else self.rate_cross[self.mode]) * dt
                appl = (max(-lim_al, min(lim_al, g_al)) * u
                        + max(-lim_cr, min(lim_cr, g_cr)) * v)
            else:
                appl = gap
            self.pub = self.pub + appl
        sa, sc = self.sigmas(o[2])
        out['k_scale'] = self.k_scale
        out['spin_m'] = self.spin_m
        out.update(dx_appl=float(appl[0]), dy_appl=float(appl[1]),
                   reject=float(REJECT_CODES.index(reject)),
                   pending_m=max(n - float(np.linalg.norm(appl)), 0.0),
                   sig_along=sa, sig_cross=sc,
                   compute_ms=(time.perf_counter() - t0) * 1e3)
        return out

    def status_list(self, out):
        return [float(out[k]) for k in STATUS_FIELDS]
