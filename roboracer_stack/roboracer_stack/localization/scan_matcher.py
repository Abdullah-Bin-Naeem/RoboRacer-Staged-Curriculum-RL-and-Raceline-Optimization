#!/usr/bin/env python3

"""Constrained scan-to-map matching against the occupancy grid. No ROS.

The measurement half of localizer C (`localization_v2`). Kept ROS-free so it
runs offline: `tools/segment_track.py` raycasts the map along the centreline
and scores this module without a simulator, `tools/replay_localization_v2.py`
replays a logged run through it.

WHY THIS AND NOT A PARTICLE FILTER
----------------------------------
Measured 2026-09-19 on runs_docker/{m16,base,tb10_on_amcl}.csv (iros2026,
45 Hz, slip dead reckoning), AMCL's error is not isotropic and never was:

    cross-track   |mean| 0.016 m, p90 0.03-0.045     at the geometric floor
    along-track   |mean| 0.13-0.23, p90 0.32-0.51     +0.22..+0.34 on the straight

and the map itself says why. Raycast along the centreline, the Gauss-Newton
information of a likelihood-field fit on the long straight (centreline s
26.7-35.9) is 0.0-2.6 along-track against 370-450 cross-track: parallel walls
and a 270 deg FOV that sees nothing fore or aft. NO algorithm can observe
along-track there. What a particle filter does on an unobservable axis is
random-walk: on that stretch AMCL's map->odom correction moved 3.8-10.9 m
cumulatively per run, in steps up to 0.19 m, against a 0.09-0.13 m wall margin.
Dead reckoning over the same stretch is exact (1.0000 vs truth).

So this module does not sample. It solves, and it hands back the 2x2
INFORMATION MATRIX of the translation so the caller can add each direction of
the correction with the weight the geometry actually gives it. On the straight
that is rank one: cross-track corrected, along-track carried by odometry. The
"track segment" the caller switches on is a lookup for predictability and
logging; the eigenvalues of `info` are the ground truth of it, per scan.

WHAT IT SOLVES FOR
------------------
Translation only. `dead_reckoning` integrates the IMU's ABSOLUTE quaternion,
so `odom` is already world-aligned and map->odom is a pure translation on this
stack (log_localization's docstring: "m2o_yaw_deg -- THE INVARIANT"). AMCL
estimates that yaw anyway and wandered it 88-587 deg cumulatively per run;
corr(err_yaw, m2o_yaw) = 0.85, and removing it takes the follower's heading
error from 0.81 to 0.43 deg std. `yaw_dof=True` keeps a heavily-prior-weighted
third DOF ONLY to report `yaw_hint`, so that invariant is measured from runs
rather than trusted.

THE COST FUNCTION
-----------------
Likelihood field, the same model as AMCL's `likelihood_field`, evaluated
instead of sampled: d(p) is the distance from a beam endpoint to the nearest
occupied cell (the map's Euclidean distance transform) and each beam weighs
w = exp(-d^2 / 2 sigma^2). Gauss-Newton on sum(w d^2) converges in 3-5
iterations from a dead-reckoned prior. The weight also makes it robust: a beam
on something the map does not have stops pulling instead of dragging.

GRID CONVENTION
---------------
`col = floor((x - ox) / res)`, `row = (h - 1) - floor((y - oy) / res)`: PGM
row 0 is the TOP. Identical to log_localization._fit and
tools/check_scan_alignment.py, the file that established that the as-is
geometry (unmirrored scan, unnegated yaw) is the correct one.
"""

import math
import os

import numpy as np

try:
    from scipy import ndimage
except ImportError:                     # the host venvs lack scipy; the image has it
    ndimage = None


def read_pgm(path):
    """Binary P5 PGM as a (h, w) uint8 array. Same reader as check_scan_alignment."""
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError(f'{path}: expected a binary P5 PGM, got {magic!r}')
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()                    # maxval
        return np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)


def read_map_yaml(path):
    """map_server yaml -> dict. PyYAML when present; the files are six flat keys
    (`origin` may be a flow list or a block list), so a hand parser covers the
    host venvs that lack it."""
    try:
        import yaml
        return yaml.safe_load(open(path))
    except ImportError:
        out, key = {}, None
        for raw in open(path):
            line = raw.split('#', 1)[0].rstrip()
            if not line.strip():
                continue
            if line.lstrip().startswith('- ') and key:
                out.setdefault(key, []).append(float(line.split('-', 1)[1]))
                continue
            k, _, v = line.partition(':')
            key, v = k.strip(), v.strip()
            if not v:
                out[key] = []
            elif v.startswith('['):
                out[key] = [float(t) for t in v.strip('[]').split(',')]
            else:
                try:
                    out[key] = float(v) if ('.' in v or 'e' in v) else int(v)
                except ValueError:
                    out[key] = v
        return out


def _edt1d_rows(f):
    """Exact squared 1-D distance transform of every row (Felzenszwalb &
    Huttenlocher 2004), lower envelope of parabolas. O(n) per row, no
    temporaries beyond the row: the earlier brute-force fallback paired every
    cell against every obstacle cell and, for the INSIDE half of the signed
    field (5 943 wall cells against 211 000 free ones), allocated ~7 GB per
    process -- three of them at once took the machine down on 2026-09-19."""
    INF = float('inf')
    out = np.empty_like(f)
    n = f.shape[1]
    v = np.zeros(n, dtype=np.int64)
    z = np.empty(n + 1)
    for r in range(f.shape[0]):
        fr = f[r]
        k = 0
        v[0] = 0
        z[0], z[1] = -INF, INF
        for q in range(1, n):
            s = ((fr[q] + q * q) - (fr[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
            while s <= z[k]:
                k -= 1
                s = ((fr[q] + q * q) - (fr[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
            k += 1
            v[k] = q
            z[k], z[k + 1] = s, INF
        k = 0
        o = out[r]
        for q in range(n):
            while z[k + 1] < q:
                k += 1
            o[q] = (q - v[k]) ** 2 + fr[v[k]]
    return out


def distance_transform(free):
    """Cells to the nearest False cell of `free`, exact. scipy when available
    (the image), else the separable transform above (the host venvs)."""
    if ndimage is not None:
        return ndimage.distance_transform_edt(free)
    if free.all():
        return np.full(free.shape, float(max(free.shape)))
    f = np.where(free, 1e12, 0.0)
    f = _edt1d_rows(f)
    f = _edt1d_rows(np.ascontiguousarray(f.T)).T
    return np.sqrt(f)


def _cache_path(pgm):
    """~/.cache/racer_localization/<name>-<mtime>-<size>.npz for the finished
    signed field. Computed once per map file; the key changes with the file."""
    st = os.stat(pgm)
    d = os.path.join(os.path.expanduser('~'), '.cache', 'racer_localization')
    return os.path.join(d, f'{os.path.basename(pgm)}-{int(st.st_mtime)}-{st.st_size}.sdf.npz')


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class LikelihoodField:
    """The map's distance transform plus its gradient, bilinearly sampled.

    Built once at startup; iros2026 is 241 x 901 cells at 0.025 m.
    """

    def __init__(self, map_yaml):
        meta = read_map_yaml(map_yaml)
        pgm = meta['image']
        if not os.path.isabs(pgm):
            pgm = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), pgm)
        img = read_pgm(pgm)
        if meta.get('negate', 0):
            img = 255 - img

        self.path = map_yaml
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        self.h, self.w = img.shape

        # map_saver greyscale: 0 occupied, 205 unknown, 254 free.
        occupied = img == 0
        if not occupied.any():
            raise ValueError(f'{pgm}: no occupied cells')
        self.occupied = occupied
        self.free = img >= 250
        # SIGNED distance to the wall FACE: positive in free space, negative
        # inside a wall, zero on the face. Two things made the plain EDT wrong
        # here. Its zero is at occupied-cell CENTRES, so a beam ending on a face
        # reads res/2 and is pulled half a cell into the wall -- side to side
        # that cancels, but the end wall ahead pulls the estimate FORWARD
        # (+0.008 +- 0.018 m along-track at the true pose on synthetic scans).
        # And its V-shaped bottom flattens the finite-difference gradient to
        # ~0.5 exactly where converged endpoints sit, which halved every
        # information value and let the observability guard veto observable
        # sections. The signed field has a unit-slope gradient straight through
        # the face. The cost uses d^2, so it is unchanged in form.
        cache = _cache_path(pgm)
        sdf = None
        try:
            if os.path.exists(cache):
                sdf = np.load(cache)['sdf']
                if sdf.shape != img.shape:
                    sdf = None
        except Exception:                                    # noqa: BLE001
            sdf = None
        if sdf is None:
            outside = distance_transform(~occupied) - 0.5    # free cells: to the nearest face
            inside = distance_transform(occupied) - 0.5      # occupied cells: to the nearest face
            sdf = (np.where(occupied, -inside, outside) * self.res).astype(np.float32)
            try:
                os.makedirs(os.path.dirname(cache), exist_ok=True)
                np.savez_compressed(cache, sdf=sdf)
            except Exception:                                # noqa: BLE001
                pass
        self.dist = sdf

        # Gradient in MAP axes: row grows as y falls, hence the sign on gy.
        grow, gcol = np.gradient(self.dist)
        self.gx = (gcol / self.res).astype(np.float32)
        self.gy = (-grow / self.res).astype(np.float32)

    def to_grid(self, x, y):
        """Map metres -> fractional (row, col)."""
        col = (np.asarray(x, dtype=np.float64) - self.ox) / self.res
        row = (self.h - 1) - (np.asarray(y, dtype=np.float64) - self.oy) / self.res
        return row, col

    def sample(self, x, y):
        """Bilinear (d, dd/dx, dd/dy, in_bounds) at map points. Out-of-bounds
        points read 0 with zero gradient and are masked, never clamped -- a
        clamped endpoint would pull the solution toward the map edge."""
        row, col = self.to_grid(x, y)
        r0 = np.floor(row).astype(np.int32)
        c0 = np.floor(col).astype(np.int32)
        ok = (r0 >= 0) & (r0 < self.h - 1) & (c0 >= 0) & (c0 < self.w - 1)
        r0 = np.clip(r0, 0, self.h - 2)
        c0 = np.clip(c0, 0, self.w - 2)
        fr = (row - r0).astype(np.float32)
        fc = (col - c0).astype(np.float32)
        w00 = (1 - fr) * (1 - fc); w01 = (1 - fr) * fc
        w10 = fr * (1 - fc); w11 = fr * fc

        def bil(a):
            return (a[r0, c0] * w00 + a[r0, c0 + 1] * w01 +
                    a[r0 + 1, c0] * w10 + a[r0 + 1, c0 + 1] * w11)

        d = np.where(ok, bil(self.dist), 0.0)
        gx = np.where(ok, bil(self.gx), 0.0)
        gy = np.where(ok, bil(self.gy), 0.0)
        return d, gx, gy, ok

    def clearance(self, x, y):
        """Nearest-cell distance from one map point to the nearest wall."""
        row, col = self.to_grid(float(x), float(y))
        r, c = int(round(float(row))), int(round(float(col)))
        if not (0 <= r < self.h and 0 <= c < self.w):
            return 0.0
        return float(self.dist[r, c])

    def raycast(self, x, y, yaw, angles, range_max=10.0, step=None):
        """Ranges a lidar at (x, y, yaw) would return, by marching the grid.
        For the offline bench and replay; the node never calls it."""
        step = step or self.res * 0.5
        n = len(angles)
        rng = np.full(n, float(range_max))
        hit = np.zeros(n, dtype=bool)
        ca = np.cos(yaw + angles); sa = np.sin(yaw + angles)
        for d in np.arange(step, range_max, step):
            px = x + ca * d; py = y + sa * d
            col = ((px - self.ox) / self.res).astype(int)
            row = ((self.h - 1) - (py - self.oy) / self.res).astype(int)
            inb = (row >= 0) & (row < self.h) & (col >= 0) & (col < self.w)
            o = np.zeros(n, dtype=bool)
            o[inb] = self.occupied[row[inb], col[inb]]
            new = o & ~hit
            rng[new] = d
            hit |= new
            if hit.all():
                break
        return rng, hit


class MatchResult:
    """One scan-to-map solve and how much to believe it.

    `info` is the 2x2 Gauss-Newton information matrix of the TRANSLATION in map
    axes. Its eigenvectors are the observable directions and its eigenvalues how
    strongly each was pinned; `along_cross(yaw)` projects it onto the car.
    """

    __slots__ = ('dx', 'dy', 'info', 'inlier_frac', 'resid', 'beams', 'iters',
                 'ok', 'reason', 'yaw_hint', 'score')

    def __init__(self, dx=0.0, dy=0.0, info=None, inlier_frac=0.0,
                 resid=float('inf'), beams=0, iters=0, ok=False, reason='',
                 yaw_hint=0.0, score=0.0):
        self.dx, self.dy = float(dx), float(dy)
        self.info = np.zeros((2, 2)) if info is None else np.asarray(info, dtype=float)
        self.inlier_frac = float(inlier_frac)
        self.resid = float(resid)
        self.beams = int(beams)
        self.iters = int(iters)
        self.ok = bool(ok)
        self.reason = reason
        self.yaw_hint = float(yaw_hint)   # what a 3-DOF solve wanted; diagnostic only
        self.score = float(score)         # mean likelihood weight, 0..1

    def along_cross(self, yaw):
        """(along_info, cross_info): the information projected on the car's
        heading and its normal. Zero along means "do not touch along-track"."""
        u = np.array([math.cos(yaw), math.sin(yaw)])
        v = np.array([-u[1], u[0]])
        return float(u @ self.info @ u), float(v @ self.info @ v)

    def __repr__(self):
        ev = np.linalg.eigvalsh(self.info)
        return (f'<Match ok={self.ok} d=({self.dx:+.3f},{self.dy:+.3f}) '
                f'resid={self.resid:.3f} inl={self.inlier_frac:.2f} '
                f'lam={ev[1]:.3g}/{ev[0]:.3g}{" " + self.reason if self.reason else ""}>')


class ScanMatcher:
    """Gauss-Newton scan-to-map registration on a likelihood field.

    A refiner, not a searcher: it takes the dead-reckoned prior and reports
    failure rather than hunting, because every logged failure on this stack
    (L1, L3, run 24) was a small along-track error at a corner turn-in, not a
    lost car. `global_search` is the separate, explicit path for that.
    """

    def __init__(self, field, *, sigma=0.12, sigma_start=0.40, max_iters=8,
                 step_tol=0.002, inlier_m=0.25, min_beams=60,
                 min_inlier_frac=0.55, max_resid_m=0.16, max_step_m=0.35,
                 yaw_dof=False, yaw_prior_info=4.0e4):
        self.field = field
        self.sigma = float(sigma)               # likelihood-field width [m]
        # Coarse-to-fine. At sigma 0.12 a beam 0.30 m off weighs exp(-3.1) =
        # 0.04, so a prior that far out barely pulls and the solve stalls (the
        # bench showed it: inlier and step rejections on PERFECT scans). The
        # first iterations run wide and each one halves the width down to
        # `sigma`, so a 0.3 m error is inside the basin from the start.
        self.sigma_start = max(float(sigma_start), self.sigma)
        self.max_iters = int(max_iters)
        self.step_tol = float(step_tol)
        self.inlier_m = float(inlier_m)
        self.min_beams = int(min_beams)
        self.min_inlier_frac = float(min_inlier_frac)
        self.max_resid_m = float(max_resid_m)
        self.max_step_m = float(max_step_m)
        self.yaw_dof = bool(yaw_dof)
        # Prior that pins yaw when yaw_dof is on: ~0.3 deg 1-sigma over a
        # 360-beam scan. The third DOF is a check, not a free parameter.
        self.yaw_prior_info = float(yaw_prior_info)

    def _endpoints(self, px, py, tx, ty, rx, ry, dyaw):
        if dyaw:
            cy, sy = math.cos(dyaw), math.sin(dyaw)
            return px + tx + rx * cy - ry * sy, py + ty + rx * sy + ry * cy
        return px + tx + rx, py + ty + ry

    def match(self, pred, pts_body):
        """Refine `pred` = (x, y, yaw) of the BASE frame against the map.

        `pts_body` is (N, 2) beam endpoints in the base frame, the lidar
        extrinsic already applied (`scan_to_points`).
        """
        px, py, pyaw = float(pred[0]), float(pred[1]), float(pred[2])
        n = len(pts_body)
        if n < self.min_beams:
            return MatchResult(reason=f'only {n} beams')

        c, s = math.cos(pyaw), math.sin(pyaw)
        rx = pts_body[:, 0] * c - pts_body[:, 1] * s
        ry = pts_body[:, 0] * s + pts_body[:, 1] * c

        tx = ty = dyaw = 0.0
        info = np.zeros((2, 2))
        iters = 0
        k = 3 if self.yaw_dof else 2
        sig = self.sigma_start

        for iters in range(1, self.max_iters + 1):
            ex, ey = self._endpoints(px, py, tx, ty, rx, ry, dyaw)
            d, gx, gy, ok = self.field.sample(ex, ey)
            if ok.sum() < self.min_beams:
                return MatchResult(reason=f'{int(ok.sum())} beams inside the map', iters=iters)

            w = np.exp(-d * d / (2.0 * sig * sig)) * ok
            # Jacobian of d wrt (tx, ty, dyaw); dyaw rotates the cloud about the base.
            jt = gx * (-(ey - py - ty)) + gy * (ex - px - tx)
            wgx, wgy, wjt = gx * w, gy * w, jt * w
            H = np.array([[wgx @ gx, wgx @ gy, wgx @ jt],
                          [wgx @ gy, wgy @ gy, wgy @ jt],
                          [wgx @ jt, wgy @ jt, wjt @ jt + self.yaw_prior_info]])
            g = np.array([wgx @ d, wgy @ d, wjt @ d])
            info = H[:2, :2].copy()

            A, b = H[:k, :k], g[:k]
            # Levenberg damping scaled to the strongest axis: on a straight the
            # weak axis is singular and an undamped solve would slide the
            # estimate down the corridor on sensor noise alone.
            lam = 1e-3 * max(0.5 * float(np.trace(A[:2, :2])), 1.0)
            try:
                step = -np.linalg.solve(A + lam * np.eye(k), b)
            except np.linalg.LinAlgError:
                return MatchResult(info=info, reason='singular normal equations', iters=iters)
            if not np.all(np.isfinite(step)):
                return MatchResult(info=info, reason='non-finite step', iters=iters)

            tx += float(step[0]); ty += float(step[1])
            if k == 3:
                dyaw = wrap(dyaw + float(step[2]))
            # Converged only once the kernel is at its final width; a small
            # step under a wide kernel just means the wide kernel is flat.
            if sig <= self.sigma and math.hypot(float(step[0]), float(step[1])) < self.step_tol:
                break
            sig = max(self.sigma, sig * 0.5)

        # Quality, on the converged solution and at the FINAL width, so the
        # score means the same thing whatever the prior error was.
        two_sig2 = 2.0 * self.sigma * self.sigma
        ex, ey = self._endpoints(px, py, tx, ty, rx, ry, dyaw)
        d, _, _, ok = self.field.sample(ex, ey)
        used = int(ok.sum())
        if used < self.min_beams:
            return MatchResult(info=info, reason='left the map', iters=iters)
        dm = np.abs(d[ok])                   # the field is signed; quality is about magnitude
        inl = dm < self.inlier_m
        inlier = float(inl.mean())
        resid = float(dm[inl].mean()) if inl.any() else float('inf')
        score = float(np.exp(-dm * dm / two_sig2).mean())

        r = MatchResult(dx=tx, dy=ty, info=info, inlier_frac=inlier, resid=resid,
                        beams=used, iters=iters, yaw_hint=dyaw, score=score)
        # The step gate is a GROSS bound only -- solver divergence, a match
        # that walked out of the corridor. Plausibility against what the filter
        # knows is V2Filter's job, per axis, because a step that is large only
        # along an unobservable axis is not evidence of a bad match: rejecting
        # the whole measurement on it locked the cross correction out for a
        # whole corner on the first shadow run. It still judges the OBSERVABLE
        # part, so an unconstrained slide down a corridor does not trip it.
        ev, evec = np.linalg.eigh(info)
        if ev[1] > 0.0 and ev[0] / ev[1] < 0.05:
            step_m = abs(float(np.array([tx, ty]) @ evec[:, 1]))
        else:
            step_m = math.hypot(tx, ty)
        # ...unless the fit is clean: inliers 0.95+ at half the residual bound
        # is not divergence, it is a wrong prior. lv_L750_warm (2026-09-19):
        # the recovery seed was 1.07 m from the car, every scan asked for the
        # 1.05 m step with inliers 1.00 and residual 0.01, and this gate
        # refused it for 2.3 s until the creep moved the car. The filter's
        # per-axis clamp still paces how much of it lands per scan.
        clean = inlier >= 0.95 and resid <= 0.5 * self.max_resid_m
        if step_m > self.max_step_m and not (clean and step_m <= 3.0 * self.max_step_m):
            r.reason = f'step {step_m:.2f} m > {self.max_step_m:.2f}'
        elif inlier < self.min_inlier_frac:
            r.reason = f'inliers {inlier:.2f} < {self.min_inlier_frac:.2f}'
        elif not resid <= self.max_resid_m:
            r.reason = f'residual {resid:.3f} m > {self.max_resid_m:.3f}'
        else:
            r.ok = True
        return r

    def global_search(self, pts_body, yaw, *, step_m=0.20, clearance_m=0.25,
                      bounds=None):
        """Exhaustive 2-D search over free space at a KNOWN yaw, then refine.

        Race-legal relocalization with no prior. Two-dimensional, not AMCL's
        three: the IMU quaternion is absolute, so only the position is
        unknown. iros2026's free space is a few hundred candidates at 0.20 m.
        Returns (x, y, MatchResult); x, y are None on failure.
        """
        f = self.field
        rows, cols = np.nonzero(f.free & (f.dist > clearance_m))
        if len(rows) == 0:
            return None, None, MatchResult(reason='no free space above clearance')
        xs = f.ox + (cols + 0.5) * f.res
        ys = f.oy + ((f.h - 1) - rows + 0.5) * f.res
        if bounds is not None:
            x0, y0, x1, y1 = bounds
            keep = (xs >= x0) & (xs <= x1) & (ys >= y0) & (ys <= y1)
            xs, ys = xs[keep], ys[keep]
            if len(xs) == 0:
                return None, None, MatchResult(reason='bounds exclude all free space')
        key = np.stack([np.round(xs / step_m), np.round(ys / step_m)], axis=1)
        _, idx = np.unique(key, axis=0, return_index=True)
        xs, ys = xs[idx], ys[idx]

        c, s = math.cos(yaw), math.sin(yaw)
        rx = pts_body[:, 0] * c - pts_body[:, 1] * s
        ry = pts_body[:, 0] * s + pts_body[:, 1] * c
        take = max(1, len(rx) // 120)               # coarse sweep on ~120 beams
        rx_s, ry_s = rx[::take], ry[::take]
        two_sig2 = 2.0 * (3.0 * self.sigma) ** 2    # broad kernel for the sweep

        best, best_i = -1.0, -1
        for i in range(len(xs)):
            d, _, _, ok = f.sample(xs[i] + rx_s, ys[i] + ry_s)
            sc = float((np.exp(-d * d / two_sig2) * ok).mean())
            if sc > best:
                best, best_i = sc, i
        if best_i < 0:
            return None, None, MatchResult(reason='global sweep found nothing')
        x, y = float(xs[best_i]), float(ys[best_i])
        r = self.match((x, y, yaw), pts_body)
        if not r.ok:
            return None, None, r
        return x + r.dx, y + r.dy, r


def scan_to_points(ranges, angle_min, angle_increment, *, range_min=0.06,
                   range_max=10.0, lidar_xy=(0.2733, 0.0), max_use_m=None,
                   stride=1):
    """LaserScan arrays -> (N, 2) endpoints in the BASE frame.

    Beams at or beyond range_max hit nothing and are dropped, as are non-finite
    ones. `max_use_m` also drops far beams: an endpoint's position error grows
    with range x heading error, so beyond ~8 m a beam adds noise faster than
    information (the bootstrap caps its own check at scan_match_max_range for
    the same reason). `stride` subsamples: 3 gives ~360 of 1081, the beam count
    of the accepted L3 AMCL config.
    """
    rng = np.asarray(ranges, dtype=np.float64)
    if stride > 1:
        rng = rng[::stride]
    ang = angle_min + np.arange(len(rng)) * (angle_increment * stride)
    lim = range_max * 0.99 if max_use_m is None else min(max_use_m, range_max * 0.99)
    good = np.isfinite(rng) & (rng > range_min) & (rng < lim)
    rng, ang = rng[good], ang[good]
    return np.stack([lidar_xy[0] + rng * np.cos(ang),
                     lidar_xy[1] + rng * np.sin(ang)], axis=1)
