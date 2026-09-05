#!/usr/bin/env python3
"""Physics-based raceline and velocity profile for the AutoDRIVE RoboRacer.

    map (track_clean.pgm) -> centerline -> minimum-curvature line -> velocity
    profile under the SIMULATOR'S tire and drag model -> CSVs for pure_pursuit

Every number about the car comes from raceline/VEHICLE_MODEL.md, which was
derived from the simulator's source, not guessed. Run from the venv:

    python optimize_raceline.py                      # centerline (if missing), all methods, ladder export
    python optimize_raceline.py --method tum         # one solver
    python optimize_raceline.py --safety 0.10        # tighter wall margin
    python optimize_raceline.py --score a.csv b.csv  # re-score existing lines under this physics
    python optimize_raceline.py --regen-centerline   # rebuild the centerline from the map

What this replaces, and why (review of 2026-09-03, see VEHICLE_MODEL.md §7):

  * The notebook's TUM method passed [x, y, w_LEFT, w_RIGHT] as the reference
    track. tph documents the columns as [x, y, w_tr_right, w_tr_left]; the
    direction of tph's normal only fixes the sign of alpha, not which wall is on
    the right. The exported TUM line violated the true corridor at 30 of 276
    points and had −0.023 m of body clearance against the map.
  * The notebook's scipy method linearised around the current iterate but
    applied the result as an offset from the fixed centerline, so iterations
    1..7 formed a period-3 cycle and never improved on iteration 2. Here the
    step is applied from the iterate, along the iterate's normals, with the
    corridor re-measured from the map at every pass, so the loop converges.
  * The scorer used a_lat 6.0 and a_long 5.0 with quadratic drag 0.05. The sim
    gives a robust 4.90 lateral, 4.55 longitudinal, and LINEAR drag 0.273/s.
  * Exported width columns were the centerline's, not the line's. They are now
    ray-cast from the map at the line.
  * Periodic splines were fitted without repeating the first point, which
    drops the last point. Harmless at 400 points, fixed anyway.

Column layout of every CSV is unchanged: s, x, y, psi, kappa, w_right, w_left[, v].
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
import os
from pathlib import Path

import math
import numpy as np
import trajectory_planning_helpers as tph
import yaml
from scipy import interpolate, ndimage
from scipy.optimize import lsq_linear

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
MAPS_DIR = REPO / "devkit_ws/src/racer_mapping/maps"
DEFAULT_TRACK = os.environ.get("RACER_TRACK", "porto")
# Per track: the grid lives in maps/<track>/ and the lines in raceline/<track>/.
# --margin-zones is per track too (they are s-ranges on THAT centreline);
# the argument that built each track's ladder is recorded in
# racer_common.frames.TRACKS so it can be reproduced.
def map_base(track):
    # track_clean is what AMCL localizes against: the world as the LiDAR sees
    # it. When that has openings the LiDAR sees through but the car cannot
    # drive through (a duct's open end), the GEOMETRY must use a copy with
    # them sealed, track_solid, or the centreline shortcuts through them.
    solid = MAPS_DIR / track / "track_solid"
    return solid if solid.with_suffix(".pgm").exists() else MAPS_DIR / track / "track_clean"


# --------------------------------------------------------------------------- #
# Physics, from the simulator source (VEHICLE_MODEL.md §2, §3)
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class SimPhysics:
    g: float = 9.81
    mass: float = 3.906           # sprung 3.47 + 4 x 0.109
    mu_x_peak: float = 0.72       # forward extremum 0.9 x stiffness 0.8, at slip 0.15
    mu_x_asym: float = 0.464      # forward asymptote 0.58 x 0.8, for slip >= 0.25
    mu_y_peak: float = 1.00       # sideways extremum at tan(alpha) = 0.01
    mu_y_asym: float = 0.50       # sideways asymptote for tan(alpha) >= 0.10
    drag_lin: float = 0.273       # Rigidbody.drag: a = -0.273 v   (LINEAR)
    wheelbase: float = 0.324
    width: float = 0.27
    max_steer: float = 0.5236     # rad
    steer_rate: float = 3.2       # rad/s, controller rate limit
    u_per_throttle: float = 25.25  # wheel surface speed per unit throttle, m/s

    @property
    def a_lat_robust(self) -> float:      # 4.905: held at ANY slip angle
        return self.mu_y_asym * self.g

    @property
    def a_long_robust0(self) -> float:    # 4.55: held at ANY slip ratio, before drag
        return self.mu_x_asym * self.g

    @property
    def a_long_profile(self) -> float:
        # What the velocity profile plans for longitudinally. The tire's robust
        # floor is the 4.55 asymptote (a_long_robust0), but the slip-band
        # controller reaches the peak of the curve, not the asymptote, and the
        # car was measured braking and accelerating at ~5.5 m/s^2 while tracking
        # (it beats a 4.55 profile on every corner exit, runs 22-29). 5.0 is
        # between the two: a measured, still-conservative plan. Validated on the
        # 7.0 line, run 38: 6.50 s best, clean, wall margins unchanged, against
        # 6.58 at 4.55. Lateral stays at the asymptote (a_lat_robust) because
        # there the peak is only held for half a degree of slip (VEHICLE_MODEL
        # §3.2) and the controller cannot sit in it.
        return 5.0

    def a_accel(self, v):                 # forward, wheels allowed to spin
        return self.a_long_robust0 - self.drag_lin * np.asarray(v, float)

    def a_brake_locked(self, v):          # throttle exactly 0 -> wheels locked
        return self.a_long_robust0 + self.drag_lin * np.asarray(v, float)

    @property
    def kappa_car(self) -> float:         # 1.78 1/m at full lock, bicycle model
        return float(np.tan(self.max_steer) / self.wheelbase)


PHYS = SimPhysics()


@dataclass
class ProfileLimits:
    """What the velocity profile is allowed to ask for. Defaults are the robust
    (asymptote) limits with a small lateral margin; see VEHICLE_MODEL.md §5.1."""
    a_lat: float = 4.5                    # 92 % of the 4.90 asymptote
    a_long: float = PHYS.a_long_profile   # planned long accel; measured, see PHYS.a_long_profile
    v_max: float = 8.0                    # pure_pursuit.yaml clips here anyway
    dyn_model_exp: float = 2.0            # friction ellipse; conservative

    def ggv(self) -> np.ndarray:
        # tph interpolates rows by speed; the last row must sit at or above v_max.
        return np.array([[0.0, self.a_long, self.a_lat],
                         [self.v_max + 1.0, self.a_long, self.a_lat]])

    def ax_max_machines(self) -> np.ndarray:
        # The motor never limits acceleration (VEHICLE_MODEL.md §3.1); the tire does.
        return np.array([[0.0, self.a_long], [self.v_max + 1.0, self.a_long]])

    def drag_coeff(self, phys: SimPhysics = PHYS, v_lo: float = 2.0) -> float:
        """tph models drag as c·v²/m; the sim's is 0.273·v (linear). Least-squares
        fit of c over [v_lo, v_max], the speeds the profile actually visits.
        Error stays under ±0.4 m/s² across that range. Folding the linear drag
        into the ggv/machine tables instead is exact on straights but tph then
        applies no drag at all where the friction ellipse binds, which over-allows
        corner-exit acceleration by 0.273 v; and putting it only on the machine
        side under-estimates braking by twice that. The fit is the smaller error."""
        v = np.linspace(v_lo, self.v_max, 200)
        return float(phys.drag_lin * phys.mass * np.sum(v ** 3) / np.sum(v ** 4))


# --------------------------------------------------------------------------- #
# Map
# --------------------------------------------------------------------------- #
class TrackMap:
    """Occupancy grid + distance transform. Free = 254/255, wall = 0, unknown = 205."""

    def __init__(self, base: Path):
        meta = yaml.safe_load(open(base.with_suffix(".yaml")))
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        with open(base.with_suffix(".pgm"), "rb") as f:
            assert f.readline().strip() == b"P5", "not a binary PGM"
            line = f.readline()
            while line.startswith(b"#"):
                line = f.readline()
            self.W, self.H = map(int, line.split())
            f.readline()
            self.img = np.frombuffer(f.read(self.W * self.H), dtype=np.uint8).reshape(self.H, self.W)
        self.free = self.img >= 254
        # metres from each free cell to the nearest non-free cell; 0 outside free space,
        # plus WHICH cell that is, so widths() can tell which side it lies on
        self.dist = ndimage.distance_transform_edt(self.free) * self.res
        # every non-free cell centre, for per-side nearest-wall queries in widths()
        rr, cc = np.nonzero(~self.free)
        wx, wy = self.px2world(rr, cc)
        from scipy.spatial import cKDTree
        self._walls = cKDTree(np.c_[wx, wy])

    # PGM row 0 is the top of the image = max world y
    def px2world(self, r, c):
        return self.ox + (c + 0.5) * self.res, self.oy + (self.H - r - 0.5) * self.res

    def rc(self, x, y):
        c = np.clip(np.round((np.asarray(x) - self.ox) / self.res - 0.5).astype(int), 0, self.W - 1)
        r = np.clip(np.round(self.H - (np.asarray(y) - self.oy) / self.res - 0.5).astype(int), 0, self.H - 1)
        return r, c

    def clearance(self, x, y):
        """Distance from each point to the nearest wall/unknown cell (0 if not in free space)."""
        r, c = self.rc(x, y)
        return self.dist[r, c]

    def widths(self, x, y, psi, max_r=3.0):
        """Ray-cast from each point along +-normal until leaving free space.
        Returns (w_right, w_left) in metres, measured at THIS line."""
        x, y, psi = map(np.asarray, (x, y, psi))
        step = self.res * 0.25
        out = []
        for sign in (-1.0, +1.0):                    # right first, then left
            ang = psi + sign * np.pi / 2.0
            ca, sa = np.cos(ang), np.sin(ang)
            d = np.full(len(x), max_r)
            alive = np.ones(len(x), bool)
            for k in range(1, int(max_r / step) + 1):
                r = k * step
                hit = alive & (self.clearance(x + ca * r, y + sa * r) <= 0.0)
                d[hit] = r - step / 2.0
                alive &= ~hit
                if not alive.any():
                    break
            out.append(d)
        w_right, w_left = out
        # A ray along the normal can miss a wall that lies AHEAD of it, such as
        # the free end of a duct on a hairpin approach, and then the solver's
        # corridor, the follower's chord bound and any margin zone all believe
        # in room that is not there. So each side's width is also capped by the
        # nearest wall cell on THAT side within max_r. (A single-nearest-cell
        # rule is not enough: the other side's wall can be marginally closer.)
        # Guard, not a fix for a seen failure: the ICRA 2026 spot that prompted
        # it turned out to be the line running along a side wall at the design
        # margin, which the normal ray had seen all along.
        cpsi, spsi = np.cos(psi), np.sin(psi)
        for i, hits in enumerate(self._walls.query_ball_point(np.c_[x, y], max_r)):
            if not hits:
                continue
            pts = self._walls.data[hits]
            vx, vy = pts[:, 0] - x[i], pts[:, 1] - y[i]
            dist = np.hypot(vx, vy)
            left = (cpsi[i] * vy - spsi[i] * vx) > 0.0
            if left.any():
                w_left[i] = min(w_left[i], dist[left].min())
            if (~left).any():
                w_right[i] = min(w_right[i], dist[~left].min())
        return w_right, w_left


# --------------------------------------------------------------------------- #
# Geometry helpers
# --------------------------------------------------------------------------- #
def heading(x, y):
    """Closed-loop central-difference heading. No one-sided seam like np.gradient."""
    return np.arctan2(np.roll(y, -1) - np.roll(y, 1), np.roll(x, -1) - np.roll(x, 1))


def resample_closed(x, y, n, smooth=0.0):
    """Periodic spline through the loop (first point repeated, so scipy does not
    drop the last one), re-sampled at n points uniform in ARC LENGTH."""
    xc, yc = np.append(x, x[0]), np.append(y, y[0])
    tck, _ = interpolate.splprep([xc, yc], s=smooth, per=True)
    uf = np.linspace(0.0, 1.0, 40 * n, endpoint=False)
    xf, yf = interpolate.splev(uf, tck)
    seg = np.hypot(np.diff(np.append(xf, xf[0])), np.diff(np.append(yf, yf[0])))
    s = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
    u_new = np.interp(np.linspace(0.0, seg.sum(), n, endpoint=False), s, uf)
    xn, yn = interpolate.splev(u_new, tck)
    return np.asarray(xn), np.asarray(yn)


def spline_geometry(x, y):
    """tph closed splines: element lengths and analytic curvature at each knot."""
    closed = np.vstack([np.column_stack([x, y]), [x[0], y[0]]])
    cx_, cy_, A, normvec = tph.calc_splines.calc_splines(path=closed)
    el = tph.calc_spline_lengths.calc_spline_lengths(coeffs_x=cx_, coeffs_y=cy_)
    _, kappa = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=cx_, coeffs_y=cy_, ind_spls=np.arange(len(cx_)),
        t_spls=np.zeros(len(cx_)), calc_curv=True)
    return dict(coeffs_x=cx_, coeffs_y=cy_, A=A, normvec=normvec, el=el, kappa=kappa)


def velocity_profile(kappa, el, lim: ProfileLimits, phys: SimPhysics = PHYS, mu=None):
    """mu: optional per-point scaling of the grip limits (tph applies it to both
    ggv columns). Used for per-corner lateral limits, see --lat-zones."""
    vx = tph.calc_vel_profile.calc_vel_profile(
        ax_max_machines=lim.ax_max_machines(), kappa=kappa, el_lengths=el,
        closed=True, drag_coeff=lim.drag_coeff(phys), m_veh=phys.mass, ggv=lim.ggv(),
        dyn_model_exp=lim.dyn_model_exp, mu=mu, v_max=lim.v_max)
    vx_cl = np.append(vx, vx[0])                      # one more sample than segments
    ax = tph.calc_ax_profile.calc_ax_profile(vx_profile=vx_cl, el_lengths=el, eq_length_output=False)
    t = tph.calc_t_profile.calc_t_profile(vx_profile=vx_cl, ax_profile=ax, el_lengths=el)
    return vx, ax, float(t[-1])


def apply_v_zones(vx, el, s, zones, lim: ProfileLimits, phys: SimPhysics = PHYS):
    """Per-zone speed ceiling, then re-impose longitudinal feasibility.

    --lat-zones lowers GRIP in a corner; this lowers the SPEED CEILING over an
    s-range, which is the only thing that binds on a straight (there kappa is
    ~0, so no lateral limit is active and the profile simply runs to v_max).

    Capping alone would leave a step in the profile that no car can follow, so
    a forward/backward sweep re-imposes the tire's longitudinal limit against
    the sim's linear drag: braking is helped by drag, accelerating is hindered
    by it. The loop is closed, so it is swept a few times to wrap.
    """
    if not zones:
        return vx
    # A per-point CEILING, not a cap: inside a zone the zone's value applies,
    # outside it the global --v-max does. The caller must have solved the base
    # profile at the highest ceiling in play, since this can only ever reduce
    # a speed -- solve at 7 and no zone can raise the straight to 8.
    v = np.asarray(vx, float).copy()
    ceiling = np.full(len(v), float(lim.v_max))
    for s0, s1, vc in zones:
        ceiling[(s >= s0) & (s <= s1)] = vc
    v = np.minimum(v, ceiling)
    n = len(v)
    for _ in range(3):
        for i in range(n):                                  # forward: acceleration
            j = (i + 1) % n
            a = max(lim.a_long - phys.drag_lin * v[i], 0.1)
            v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2.0 * a * el[i]))
        for i in range(n - 1, -1, -1):                      # backward: braking
            j = (i + 1) % n
            a = lim.a_long + phys.drag_lin * v[j]
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2.0 * a * el[i]))
    return v


def steering_rate_required(kappa, el, vx, phys: SimPhysics = PHYS):
    """rad/s the steering must move to follow the path at the profile speed."""
    L = phys.wheelbase
    dk = (np.roll(kappa, -1) - np.roll(kappa, 1)) / (el + np.roll(el, 1))
    return np.abs(L / (1.0 + (kappa * L) ** 2) * dk) * vx


def score_line(x, y, tm: TrackMap, lim: ProfileLimits, phys: SimPhysics = PHYS, label=""):
    """One scorer for every line: same splines, same physics."""
    g = spline_geometry(x, y)
    vx, ax, t = velocity_profile(g["kappa"], g["el"], lim, phys)
    body = tm.clearance(x, y) - phys.width / 2.0
    sr = steering_rate_required(g["kappa"], g["el"], vx, phys)
    return dict(label=label, x=np.asarray(x), y=np.asarray(y), kappa=g["kappa"], el=g["el"],
                vx=vx, ax=ax, t=t, length=float(g["el"].sum()),
                kmax=float(np.abs(g["kappa"]).max()), body_margin=float(body.min()),
                steer_rate_max=float(sr.max()), n=len(x))


def feasible(r, safety, phys: SimPhysics = PHYS, tol=0.02):
    return (r["kmax"] <= phys.kappa_car and r["body_margin"] >= safety - tol
            and r["steer_rate_max"] <= phys.steer_rate)


def select_best(hist, safety, phys: SimPhysics = PHYS, tol_frac=0.005):
    """Fastest feasible iterate, with ties broken toward smoothness.

    Lap-time differences under half a percent are below what the physics
    resolves (measured λ sweep: 7.522 to 7.531 s for the same line). Among the
    iterates inside that band, the one needing the least steering rate is the
    one pure pursuit can actually track."""
    ok = [r for r in hist if feasible(r, safety, phys)]
    if not ok:
        return None
    t0 = min(r["t"] for r in ok)
    return min((r for r in ok if r["t"] <= t0 * (1.0 + tol_frac)), key=lambda r: r["steer_rate_max"])


# --------------------------------------------------------------------------- #
# Centerline from the map (ported from raceline_experiments.ipynb)
# --------------------------------------------------------------------------- #
def extract_centerline(tm: TrackMap, n_points=400, smooth=0.5):
    from skimage.morphology import skeletonize

    lab, n = ndimage.label(tm.free)
    sizes = ndimage.sum(tm.free, lab, range(1, n + 1))
    surface = lab == (int(np.argmax(sizes)) + 1)
    skel = skeletonize(surface)

    K = np.ones((3, 3)); K[1, 1] = 0
    pruned = skel.copy()
    for _ in range(1000):                              # delete endpoints until only cycles remain
        ends = pruned & (ndimage.convolve(pruned.astype(np.uint8), K, mode="constant") == 1)
        if not ends.any():
            break
        pruned[ends] = False
    llab, ln = ndimage.label(pruned, structure=np.ones((3, 3)))
    lsz = ndimage.sum(pruned, llab, range(1, ln + 1))
    loop = llab == (int(np.argmax(lsz)) + 1)

    order = _order_loop(loop)
    raw = np.array([tm.px2world(r, c) for r, c in order])
    return resample_closed(raw[:, 0], raw[:, 1], n_points, smooth=smooth)


def _order_loop(loop):
    """Order the pixels of a (pruned) skeleton ring into one closed walk.

    Not a greedy neighbour walk: a skeleton keeps small thick spots after
    pruning, and a greedy walk that picks the first free neighbour strands
    itself at one of them and stops, after which resample_closed joins the
    dead end back to the start THROUGH WALLS. On the ICRA 2026 switchback that
    dropped the whole left third of the lap and cut across the duct. Instead:
    BFS from a start pixel to its antipode (the farthest pixel along the
    ring), take that shortest path as one half, delete its interior, and BFS
    again for the other half. Shortest paths never wander into thick spots or
    leftover spurs, and the union is the ring. Pixels off both halves are the
    artifacts, reported but harmless.
    """
    from collections import deque
    pts = [tuple(p) for p in np.argwhere(loop)]
    on = set(pts)
    nbrs = lambda p: [(p[0] + dr, p[1] + dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1)
                      if (dr or dc) and (p[0] + dr, p[1] + dc) in on]

    def bfs(start, blocked=frozenset()):
        parent = {start: None}; q = deque([start])
        while q:
            cur = q.popleft()
            for nb in nbrs(cur):
                if nb not in parent and nb not in blocked:
                    parent[nb] = cur; q.append(nb)
        return parent

    start = pts[0]
    par = bfs(start)
    far = max(par, key=lambda p: _bfs_depth(par, p))
    half1 = _chain(par, far)                         # start -> far
    par2 = bfs(start, blocked=frozenset(half1[1:-1]))
    if far not in par2:
        print("WARNING: skeleton is not a ring; falling back to one half", file=sys.stderr)
        return half1
    half2 = _chain(par2, far)                        # start -> far the other way round
    order = half1 + half2[::-1][1:-1]
    missed = len(on) - len(set(order))
    if missed:
        print(f"note: {missed} skeleton pixels off the ring (thick spots/spurs), ignored", file=sys.stderr)
    return order


def _chain(parent, node):
    out = []
    while node is not None:
        out.append(node); node = parent[node]
    return out[::-1]


def _bfs_depth(parent, node):
    d = 0
    while parent[node] is not None:
        node = parent[node]; d += 1
    return d


def export_centerline(cx, cy, tm: TrackMap, out_dir: Path, lim: ProfileLimits, phys: SimPhysics = PHYS):
    g = spline_geometry(cx, cy)
    psi = heading(cx, cy)
    wr, wl = tm.widths(cx, cy, psi)
    s = np.concatenate([[0.0], np.cumsum(g["el"])[:-1]])
    np.savetxt(out_dir / "centerline_full.csv", np.column_stack([s, cx, cy, psi, g["kappa"], wr, wl]),
               delimiter=",", fmt="%.5f", header="s_m,x_m,y_m,psi_rad,kappa_radpm,w_right_m,w_left_m", comments="# ")
    half = phys.width / 2.0
    np.savetxt(out_dir / "centerline_widths.csv",
               np.column_stack([cx, cy, np.maximum(wr - half, 0.0), np.maximum(wl - half, 0.0)]),
               delimiter=",", fmt="%.4f", header="x_m,y_m,w_tr_right_m,w_tr_left_m", comments="# ")
    return wr, wl


# --------------------------------------------------------------------------- #
# Solvers. All three re-measure the corridor from the MAP at every pass, so the
# constraint is always about the line being solved, never a stale centerline.
# --------------------------------------------------------------------------- #
def apply_margin_zones(x, y, wr, wl, zones):
    """Extra one-sided margin in s-ranges along THIS line: zones = [(s0, s1, side, extra)].

    Measured need: the car exits the S-entry right-hander 0.1-0.2 m wide to the
    LEFT, which is the inside of the following left-hander where the line ran
    0.33 m from the wall; two wall touches at s ~ 20 m with 0.02 m to spare.
    Shrinking the corridor on that side for s in [17, 21.5] moves the line
    right there and costs almost nothing elsewhere."""
    if not zones:
        return wr, wl
    seg = np.hypot(np.roll(x, -1) - x, np.roll(y, -1) - y)
    s = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
    wr, wl = wr.copy(), wl.copy()
    for s0, s1, side, extra in zones:
        m = (s >= s0) & (s <= s1)
        if side.upper().startswith('L'):
            wl[m] -= extra
        else:
            wr[m] -= extra
    return wr, wl


def _ref_track(tm, x, y, safety, phys, step_max=None, zones=None):
    """[x, y, w_RIGHT, w_LEFT] in tph's column order, widths measured at this line.

    step_max is a trust region: the QP linearises curvature around this line, and
    on a track whose radii are about 1 m a 0.9 m move makes that linearisation
    meaningless (the first unconstrained pass gave |k|max 2.7 and 8.7 rad/s of
    steering). Capping each pass at step_max makes the loop converge, the same
    way tph's own iqp_handler shrinks the corridor around the previous solution."""
    psi = heading(x, y)
    wr, wl = tm.widths(x, y, psi)
    wr, wl = apply_margin_zones(x, y, wr, wl, zones)
    half = phys.width / 2.0
    wr_c, wl_c = wr - safety, wl - safety
    if step_max is not None:
        wr_c, wl_c = np.minimum(wr_c, half + step_max), np.minimum(wl_c, half + step_max)
    floor = half + 0.005                              # keep the QP feasible where the track is narrow
    return np.column_stack([x, y, np.maximum(wr_c, floor), np.maximum(wl_c, floor)])


def _tph_loop(tm, cx, cy, lim, safety, solver, label, iters, stepsize, tol, phys, verbose, step_max,
              shrink=0.8, step_min=0.02, zones=None):
    x, y = np.asarray(cx, float), np.asarray(cy, float)
    hist = []
    for it in range(iters):
        # Trust region shrinks geometrically: big moves first, then refinement.
        # With a fixed 0.15 m cap the loop wandered at |alpha| ~ 0.13 for 15 passes.
        ref = _ref_track(tm, x, y, safety, phys, max(step_max * shrink ** it, step_min), zones)
        closed = np.vstack([ref[:, :2], ref[0, :2]])
        _, _, A, normvec = tph.calc_splines.calc_splines(path=closed)
        alpha, extra = solver(ref, normvec, A)
        rl, *_ = tph.create_raceline.create_raceline(
            refline=ref[:, :2], normvectors=normvec, alpha=alpha, stepsize_interp=stepsize)
        x, y = rl[:, 0], rl[:, 1]
        r = score_line(x, y, tm, lim, phys, label)
        r.update(iter=it, alpha_max=float(np.abs(alpha).max()), extra=extra)
        hist.append(r)
        if verbose:
            print(f"  {label:9s} iter {it:2d}: |alpha|max {r['alpha_max']:.3f}  |k|max {r['kmax']:.3f}  "
                  f"len {r['length']:.2f}  lap {r['t']:.3f} s  margin {r['body_margin']:+.3f}  "
                  f"steer {r['steer_rate_max']:.2f} rad/s")
        if r["alpha_max"] < tol:
            break
    return select_best(hist, safety, phys), hist


def solve_tum(tm, cx, cy, lim, safety, iters=15, stepsize=0.10, tol=0.01, step_max=0.15,
              kappa_bound=None, phys=PHYS, verbose=True, zones=None):
    """TUM minimum curvature QP, iterated with the corridor re-measured each pass
    and each pass limited to step_max metres of lateral movement."""
    kb = kappa_bound if kappa_bound is not None else 0.85 * phys.kappa_car

    def solver(ref, normvec, A):
        alpha, err = tph.opt_min_curv.opt_min_curv(
            reftrack=ref, normvectors=normvec, A=A, kappa_bound=kb, w_veh=phys.width,
            print_debug=False, plot_debug=False, closed=True)
        return alpha, float(err)
    return _tph_loop(tm, cx, cy, lim, safety, solver, "tum", iters, stepsize, tol, phys, verbose, step_max,
                     zones=zones)


def solve_shortest(tm, cx, cy, lim, safety, iters=8, stepsize=0.10, tol=0.01, step_max=0.15,
                   phys=PHYS, verbose=True, zones=None):
    """TUM shortest path, for comparison: shortest is not fastest on a tight track."""
    def solver(ref, normvec, A):
        alpha = tph.opt_shortest_path.opt_shortest_path(
            reftrack=ref, normvectors=normvec, w_veh=phys.width, print_debug=False)
        return alpha, None
    return _tph_loop(tm, cx, cy, lim, safety, solver, "shortest", iters, stepsize, tol, phys, verbose, step_max,
                     zones=zones)


def solve_scipy(tm, cx, cy, lim, safety, lam=2.0, iters=20, tol=0.005, phys=PHYS, verbose=True, zones=None):
    """Hand-rolled minimum curvature as bound-constrained least squares.

    Curvature is proportional to the second difference of the points for uniform
    spacing, which is linear in the lateral offsets alpha. Each pass solves for
    the offsets FROM THE CURRENT LINE along ITS normals, applies them, and
    resamples to uniform spacing without smoothing, so the corridor measured at
    the start of the pass still holds at the end. lam damps the step."""
    N = len(cx)
    i = np.arange(N); im, ip = (i - 1) % N, (i + 1) % N
    S = np.zeros((N, N)); S[i, im], S[i, i], S[i, ip] = 1.0, -2.0, 1.0
    margin = phys.width / 2.0 + safety
    x, y = np.asarray(cx, float), np.asarray(cy, float)
    hist = []
    for it in range(iters):
        psi = heading(x, y)
        nx, ny = -np.sin(psi), np.cos(psi)             # LEFT normal: alpha > 0 moves left
        wr, wl = tm.widths(x, y, psi)
        wr, wl = apply_margin_zones(x, y, wr, wl, zones)
        lo, hi = -(wr - margin), (wl - margin)
        bad = lo > hi; mid = (lo + hi) / 2.0
        lo, hi = np.where(bad, mid - 1e-3, lo), np.where(bad, mid + 1e-3, hi)

        A = np.zeros((2 * N, N))
        A[i, im] += nx[im]; A[i, i] += -2.0 * nx[i]; A[i, ip] += nx[ip]
        A[N + i, im] += ny[im]; A[N + i, i] += -2.0 * ny[i]; A[N + i, ip] += ny[ip]
        b = np.concatenate([x[im] - 2.0 * x[i] + x[ip], y[im] - 2.0 * y[i] + y[ip]])
        alpha = lsq_linear(np.vstack([A, np.sqrt(lam) * S]), np.concatenate([-b, np.zeros(N)]),
                           bounds=(lo, hi), max_iter=400).x

        x, y = resample_closed(x + alpha * nx, y + alpha * ny, N, smooth=0.0)
        r = score_line(x, y, tm, lim, phys, "scipy")
        r.update(iter=it, alpha_max=float(np.abs(alpha).max()))
        hist.append(r)
        if verbose:
            print(f"  scipy     iter {it:2d}: |alpha|max {r['alpha_max']:.3f}  |k|max {r['kmax']:.3f}  "
                  f"len {r['length']:.2f}  lap {r['t']:.3f} s  margin {r['body_margin']:+.3f}  "
                  f"steer {r['steer_rate_max']:.2f} rad/s")
        if r["alpha_max"] < tol:
            break
    return select_best(hist, safety, phys), hist


def refine_time(tm, win, lim, safety, zones=None, n_knots=40, iters=60, eps=0.01, phys=PHYS, verbose=True):
    """Minimum-TIME refinement of a line, by direct lap-time descent.

    Minimum curvature is the right proxy for a grip-limited car; ours is also
    acceleration-limited on every exit (4.55 m/s^2 minus drag), and for that
    car later apexes and straighter exits are worth time that curvature does
    not see. TUM's own answer is a full min-time optimal-control problem; this
    is the cheap version, and on Porto it found nothing: 40 knots, 60 steps,
    28 minutes, lap unchanged to the millisecond. The tph profile is piecewise
    (forward/backward passes), so finite-difference gradients are noisy, and a
    28 m lap with four corners leaves little for apex placement to win. Kept
    for longer tracks; not worth running here. The lateral offset from the current line is a
    periodic cubic spline through n_knots values, the objective is the lap time
    of the resulting line under the same physics the profile uses (plus a
    quadratic penalty for leaving the corridor, which is measured from the map
    at the start line and includes the margin zones), and L-BFGS-B walks it with
    finite-difference gradients: ~40 profile evaluations per step, a few
    minutes in total. Offsets stay small (< 0.3 m), so the start line's
    normals are an adequate frame; the result is re-scored against the map.
    """
    from scipy.interpolate import CubicSpline
    from scipy.optimize import minimize

    x0, y0 = win["x"], win["y"]
    n = len(x0)
    psi = heading(x0, y0)
    nx, ny = -np.sin(psi), np.cos(psi)
    wr, wl = tm.widths(x0, y0, psi)
    wr, wl = apply_margin_zones(x0, y0, wr, wl, zones)
    margin = phys.width / 2.0 + safety
    lo, hi = -(wr - margin), (wl - margin)                 # allowed offset per point, + = left
    # The start line is feasible by definition: widths re-measured at a finer
    # ray-cast differ by millimetres from the corridor it was solved in, and
    # without this the penalty charged the start 0.46 s (measured) and the
    # descent spent its budget on millimetres instead of time.
    lo, hi = np.minimum(lo, 0.0), np.maximum(hi, 0.0)

    s = np.concatenate([[0.0], np.cumsum(win["el"])[:-1]]); L = float(win["el"].sum())
    sk = np.linspace(0.0, L, n_knots, endpoint=False)
    half = L / n_knots / 2.0
    # knot bounds: the tightest corridor within half a knot spacing, so the
    # spline between knots stays inside as long as it does not overshoot much
    klo = np.array([lo[(np.abs(((s - c + L / 2) % L) - L / 2)) <= half].max() for c in sk])
    khi = np.array([hi[(np.abs(((s - c + L / 2) % L) - L / 2)) <= half].min() for c in sk])
    klo, khi = np.minimum(klo, 0.0), np.maximum(khi, 0.0)

    def line(k):
        cs = CubicSpline(np.append(sk, L), np.append(k, k[0]), bc_type="periodic")
        a = cs(s)
        return x0 + a * nx, y0 + a * ny, a

    def objective(k):
        x, y, a = line(k)
        g = spline_geometry(x, y)
        _, _, t = velocity_profile(g["kappa"], g["el"], lim, phys)
        viol = np.maximum(0.0, lo - a) + np.maximum(0.0, a - hi)
        return t + 1e3 * float(np.sum(viol ** 2))

    k0 = np.zeros(n_knots)
    t0 = objective(k0)
    if verbose:
        print(f"  refine: start {t0:.3f} s, {n_knots} knots, corridor half-width {np.median(hi - lo) / 2:.2f} m")
    res = minimize(objective, k0, method="L-BFGS-B", bounds=list(zip(klo, khi)),
                   options=dict(maxiter=iters, eps=eps, ftol=1e-9, gtol=1e-6))
    x, y, a = line(res.x)
    r = score_line(x, y, tm, lim, phys, win["label"] + "+time")
    if verbose:
        print(f"  refine: {res.nit} iterations, {res.nfev} evals -> {r['t']:.3f} s "
              f"({r['t'] - t0:+.3f}), |offset| max {np.abs(a).max():.2f} m, body margin {r['body_margin']:+.3f}, "
              f"steer {r['steer_rate_max']:.2f} rad/s, kmax {r['kmax']:.3f}")
    return r


SOLVERS = {"tum": solve_tum, "scipy": solve_scipy, "shortest": solve_shortest}


# --------------------------------------------------------------------------- #
# Export and reporting
# --------------------------------------------------------------------------- #
def export_csv(path: Path, r, tm: TrackMap):
    x, y = r["x"], r["y"]
    psi = heading(x, y)
    wr, wl = tm.widths(x, y, psi)                      # widths at THIS line, not the centerline's
    s = np.concatenate([[0.0], np.cumsum(r["el"])[:-1]])
    np.savetxt(path, np.column_stack([s, x, y, psi, r["kappa"], wr, wl, r["vx"]]),
               delimiter=",", fmt="%.5f",
               header="s_m,x_m,y_m,psi_rad,kappa_radpm,w_right_m,w_left_m,v_mps", comments="# ")
    return path


def print_table(rows, safety):
    print(f"\n{'line':<18}{'pts':>5}{'length':>9}{'|k|max':>8}{'lap':>9}{'v min':>7}{'v max':>7}"
          f"{'body margin':>13}{'steer rad/s':>13}   ok")
    for r in rows:
        ok = "yes" if feasible(r, safety) else "NO"
        print(f"{r['label']:<18}{r['n']:5d}{r['length']:8.2f}m{r['kmax']:8.3f}{r['t']:8.3f}s"
              f"{r['vx'].min():7.2f}{r['vx'].max():7.2f}{r['body_margin']:+13.3f}{r['steer_rate_max']:13.2f}   {ok}")


def load_xy(path):
    D = np.loadtxt(path, delimiter=",")
    return D[:, 1], D[:, 2]


# --------------------------------------------------------------------------- #
def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--track", default=DEFAULT_TRACK,
                   help="track name; picks maps/<track>/ and raceline/<track>/")
    p.add_argument("--map", type=Path, default=None, help="map base path, no extension; default: the track's")
    p.add_argument("--out", type=Path, default=None, help="output directory; default: raceline/<track>/")
    p.add_argument("--n-points", type=int, default=400)
    p.add_argument("--safety", type=float, default=0.15,
                   help="body-to-wall margin [m] on top of half the car width; a wall touch is a respawn")
    p.add_argument("--a-lat", type=float, default=4.5, help="lateral limit for scoring and the default export")
    p.add_argument("--a-long", type=float, default=PHYS.a_long_profile,
                   help="tire longitudinal limit before drag; 4.55 is held at any slip (wheels may spin at "
                        "corner exit), ~3.5 keeps slip under 10 %% for cleaner encoders")
    p.add_argument("--v-max", type=float, default=8.0)
    p.add_argument("--ladder", default="4.0,4.5,4.9", help="a_lat rungs exported for the winning geometry")
    p.add_argument("--margin-zones", default="",
                   help="extra one-sided wall margin in s-ranges of the line, 's0:s1:L|R:extra[,...]'; e.g. "
                        "'17:21.5:L:0.15' keeps 0.30 m instead of 0.15 from the LEFT wall on the S-exit "
                        "approach, where the car arrives wide out of the previous right-hander (two touches).")
    p.add_argument("--v-zones", default="",
                   help="per-zone SPEED ceiling s0:s1:v_max,... (the 'v' lines). Overrides "
                        "--v-max inside the range; use it to let only the real straights run "
                        "fast while corner approaches stay capped")
    p.add_argument("--lat-zones", default="",
                   help="per-segment lateral limits on top of each rung, 's0:s1:a_lat[,...]' in metres "
                        "along the exported line; e.g. '12.5:17:6.0' holds the S entry at 6.0 while the "
                        "rest of the lap runs the rung. Files get a 'z' suffix. Measured: at 6.5 the S "
                        "entry ran wide at full lock while the other corners held.")
    p.add_argument("--method", choices=["auto", *SOLVERS], default="auto")
    p.add_argument("--iters", type=int, default=15)
    p.add_argument("--objective", choices=["curvature", "time"], default="curvature",
                   help="'time' refines the winning line by lap-time descent (refine_time); files get a 't' prefix")
    p.add_argument("--refine-iters", type=int, default=60)
    p.add_argument("--lam", type=float, default=2.0, help="scipy step damping")
    p.add_argument("--step-max", type=float, default=0.15, help="tum/shortest trust region on the first pass [m]")
    p.add_argument("--regen-centerline", action="store_true")
    p.add_argument("--score", nargs="*", metavar="CSV", help="only re-score these lines and exit")
    p.add_argument("-q", "--quiet", action="store_true")
    a = p.parse_args(argv)
    if a.map is None:
        a.map = map_base(a.track)
    if a.out is None:
        a.out = HERE / a.track

    tm = TrackMap(a.map)
    lim = ProfileLimits(a_lat=a.a_lat, a_long=a.a_long, v_max=a.v_max)
    print(f"physics: a_lat {lim.a_lat} (asymptote {PHYS.a_lat_robust:.2f}), tire a_long {lim.a_long:.2f}, "
          f"drag 0.273 v fitted as {lim.drag_coeff():.3f} v²/m, v_max {lim.v_max}, "
          f"kappa_car {PHYS.kappa_car:.2f}, safety {a.safety} m")

    if a.score:
        rows = [score_line(*load_xy(f), tm, lim, PHYS, Path(f).stem[:18]) for f in a.score]
        print_table(rows, a.safety)
        return 0

    a.out.mkdir(exist_ok=True)
    cl_path = a.out / "centerline_full.csv"
    if cl_path.exists() and not a.regen_centerline:
        cx, cy = load_xy(cl_path)
        print(f"centerline: loaded {len(cx)} points from {cl_path.name}")
    else:
        cx, cy = extract_centerline(tm, a.n_points)
        export_centerline(cx, cy, tm, a.out, lim)
        print(f"centerline: extracted {len(cx)} points from the map -> centerline_full.csv, centerline_widths.csv")

    mzones = []
    for z in [z for z in a.margin_zones.split(",") if z]:
        s0, s1, side, extra = z.split(":")
        mzones.append((float(s0), float(s1), side, float(extra)))
    base = score_line(cx, cy, tm, lim, PHYS, "centerline")
    rows = [base]
    # 'shortest' is selectable but not run by default: on this track it hugs the
    # walls with |k|max 4-6 and needs 7 rad/s of steering, 3 s slower than min-curvature.
    methods = ["tum", "scipy"] if a.method == "auto" else [a.method]
    for m in methods:
        print(f"\n{m}:")
        kw = dict(iters=a.iters, verbose=not a.quiet, zones=mzones)
        if m == "scipy":
            kw["lam"] = a.lam
        else:
            kw["step_max"] = a.step_max
        best, _ = SOLVERS[m](tm, cx, cy, lim, a.safety, **kw)
        if best is None:
            print(f"  {m}: no feasible iterate")
            continue
        best["label"] = m
        rows.append(best)
        export_csv(a.out / f"raceline_{m}.csv", best, tm)

    print_table(rows, a.safety)
    cands = [r for r in rows[1:] if feasible(r, a.safety)]
    if not cands:
        print("\nno feasible raceline; loosen --safety or check the map")
        return 1
    win = min(cands, key=lambda r: r["t"])
    print(f"\nwinner: {win['label']}  {win['t']:.3f} s  ({win['t'] - base['t']:+.3f} s vs centerline)")
    prefix = "raceline_"
    if a.objective == "time":
        print("\nminimum-time refinement:")
        ref = refine_time(tm, win, lim, a.safety, zones=mzones, iters=a.refine_iters)
        if feasible(ref, a.safety) and ref["t"] < win["t"]:
            win = ref
            prefix = "raceline_t_"
            export_csv(a.out / "raceline_t.csv", win, tm)
        else:
            print("  refinement not feasible or not faster; keeping the curvature line")

    # Same geometry, a ladder of lateral limits: the grip limit is measured on the
    # car by stepping up until it runs wide, exactly as make_speed_variants did.
    print(f"\n{'a_lat':>6}{'lap':>9}{'v max':>7}   file")
    zones = [tuple(float(v) for v in z.split(":")) for z in a.lat_zones.split(",") if z]
    vzones = [tuple(float(v) for v in z.split(":")) for z in a.v_zones.split(",") if z]
    s_win = np.concatenate([[0.0], np.cumsum(win["el"])[:-1]])
    for rung in [float(v) for v in a.ladder.split(",")]:
        lr = ProfileLimits(a_lat=rung, a_long=a.a_long, v_max=a.v_max)
        mu = None
        if zones:
            mu = np.ones(len(s_win))
            for s0, s1, a_zone in zones:
                mu[(s_win >= s0) & (s_win <= s1)] = min(a_zone, rung) / rung
        if vzones:
            # Solve at the highest ceiling any zone asks for, then impose the
            # per-region ceilings and re-establish longitudinal feasibility.
            top = ProfileLimits(a_lat=rung, a_long=a.a_long,
                                v_max=max(lr.v_max, max(z[2] for z in vzones)))
            vx, _, _ = velocity_profile(win["kappa"], win["el"], top, mu=mu)
            vx = apply_v_zones(vx, win["el"], s_win, vzones, lr)
            ax = np.gradient(vx ** 2) / (2.0 * np.maximum(win["el"], 1e-6))
            t = float(np.sum(2.0 * win["el"] / (vx + np.roll(vx, -1))))
        else:
            vx, ax, t = velocity_profile(win["kappa"], win["el"], lr, mu=mu)
        rr = dict(win, vx=vx, ax=ax, t=t)
        suffix = ('z' if zones else '') + ('v' if vzones else '')
        out = export_csv(a.out / f"{prefix}a{rung:.1f}{suffix}.csv", rr, tm)
        print(f"{rung:6.1f}{t:8.3f}s{vx.max():7.2f}   {out.name}"
              + (f"   lat {a.lat_zones}" if zones else "") + (f"   v {a.v_zones}" if vzones else ""))

    # Baseline with a speed column, so the centerline can be raced under the same physics.
    export_csv(a.out / "centerline_speed.csv", base, tm)
    print("\nwrote centerline_speed.csv (baseline under the same physics)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
