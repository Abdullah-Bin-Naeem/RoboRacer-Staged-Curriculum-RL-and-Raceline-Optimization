#!/usr/bin/env python3

"""Divide the track by what the LiDAR can observe, and bench the matcher on it.

    python3 segment_track.py --track iros2026                # table + segments.csv
    python3 segment_track.py --track iros2026 --bench        # ... and the matcher bench

No simulator, no ROS. The occupancy grid is raycast from every Nth centreline
point (270 deg, 1081 beams, 10 m -- the devkit's lidar) and the scan is scored
with scan_matcher's Gauss-Newton information matrix, projected on the car's
heading (ALONG) and its normal (CROSS). Each point is then classified:

    STRAIGHT_BLIND   along_info below --blind (default 20; the blind straight
                     reads 0-1, real corners 40-125) and |kappa| below
                     --straight-kappa. Too little in the scan constrains the
                     car's position along the corridor; dead reckoning carries
                     it here. Deliberately conservative: the node's live guard
                     (blind_along_info, 5) is the hard floor, this table is
                     where "a dozen beams through a gap" is not trusted.
    APPROACH         the first --approach-m metres after a BLIND stretch, in
                     the lap direction: where whatever the odometry drifted
                     on the straight gets taken out, and it has to be gone
                     before the braking point, smoothly.
    CORNER           |kappa| >= --corner-kappa. Wall margin is smallest here;
                     the localizer trades speed of correction for smoothness.
    TRANSIT          everything else.

The result is written per centreline POINT (mode column beside s, x, y), in
the file's own order, so the node looks a mode up by nearest point and never
has to know which way the file's s runs. Only the APPROACH rule is
directional, and that is resolved here, the way localization_bootstrap does
it: the tangent at the spawn is compared with the spawn heading (frames.py).

Measured on iros2026 (2026-09-19): s 26.7-35.9 is blind (along_info 0.0-2.6
against cross 370-450); hairpin 1's end wall pins it only over the last ~2 m
(s 25.8 -> 24.9: 22 -> 344). That is the number the spec's "switch at 9.9 m"
had to become.

--bench perturbs the prior at each sampled point (+-0.30 m along and cross)
and reports what the matcher recovers, per mode. The gate: every non-blind
mode accepts >= 98 % of the perturbations and recovers to < 0.05 m p90; the
BLIND mode's along residual is printed, not gated -- it is EXPECTED to stay
near 0.30, that is what blind means. Run it after touching the matcher, the
map, the centreline, or the node's beam cuts (--stride, --max-use-m must
match config/localization_v2.yaml).
"""

import argparse
import csv
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))                        # racer_localization
sys.path.insert(0, os.path.join(HERE, '..', '..', 'racer_common'))  # racer_common
from racer_localization.scan_matcher import (LikelihoodField, ScanMatcher,   # noqa: E402
                                             rebase_repo_path, scan_to_points)

MODES = ('STRAIGHT_BLIND', 'APPROACH', 'CORNER', 'TRANSIT')
# The devkit lidar, from autodrive_bridge.py.
ANGLE_MIN, ANGLE_MAX, N_BEAMS, RANGE_MAX = -2.35619, 2.35619, 1081, 10.0
LIDAR_XY = (0.2733, 0.0)


def load_centreline(path):
    c = np.loadtxt(path, delimiter=',', comments='#')
    return c[:, 0], c[:, 1], c[:, 2], c[:, 3], c[:, 4]      # s, x, y, psi, kappa


def lap_direction(x, y, spawn):
    """+1 if the file's order runs with the lap, -1 against it. Same test as
    localization_bootstrap._load_centreline: the tangent at the point nearest
    the spawn against the spawn heading."""
    sx, sy, syaw = spawn
    j = int(np.argmin((x - sx) ** 2 + (y - sy) ** 2))
    k = (j + 3) % len(x)
    tangent = math.atan2(y[k] - y[j], x[k] - x[j])
    return 1 if math.cos(tangent - syaw) >= 0.0 else -1


def observe(field, matcher, x, y, yaw, angles, stride, max_use_m):
    """Information of a perfect scan taken at (x, y, yaw): (along, cross,
    beams, max range, MatchResult of a zero-error solve)."""
    lx = x + LIDAR_XY[0] * math.cos(yaw) - LIDAR_XY[1] * math.sin(yaw)
    ly = y + LIDAR_XY[0] * math.sin(yaw) + LIDAR_XY[1] * math.cos(yaw)
    rng, hit = field.raycast(lx, ly, yaw, angles, RANGE_MAX)
    rng = np.where(hit, rng, RANGE_MAX)
    pts = scan_to_points(rng, ANGLE_MIN, (ANGLE_MAX - ANGLE_MIN) / (N_BEAMS - 1),
                         range_max=RANGE_MAX, lidar_xy=LIDAR_XY, stride=stride,
                         max_use_m=max_use_m)
    r = matcher.match((x, y, yaw), pts)
    a, c = r.along_cross(yaw)
    return a, c, len(pts), float(rng[hit].max()) if hit.any() else 0.0, r, pts


def classify(along, kappa, direction, ds, *, blind, straight_kappa, corner_kappa,
             approach_m, min_run_m):
    n = len(along)
    mode = np.full(n, 3, dtype=int)                                   # TRANSIT
    mode[np.abs(kappa) >= corner_kappa] = 2                           # CORNER
    is_blind = (along < blind) & (np.abs(kappa) < straight_kappa)
    mode[is_blind] = 0                                                # STRAIGHT_BLIND
    # Smooth: a run shorter than min_run_m takes its neighbours' mode.
    mode = _merge_short_runs(mode, ds, min_run_m)
    # APPROACH: the approach_m after each blind run, walking in the lap
    # direction, over TRANSIT *and* CORNER. On iros2026 the blind straight runs
    # straight into hairpin 1 with nothing in between, so the first metres of
    # that corner ARE the approach: the carried error comes out there or not
    # at all, and it needs its own rate limit rather than the corner's.
    order = range(n) if direction > 0 else range(n - 1, -1, -1)
    budget = 0.0
    for i in order:
        if mode[i] == 0:
            budget = approach_m
            continue
        if budget > 0.0:
            mode[i] = 1                                               # APPROACH
        budget -= ds[i]
    return mode


def _merge_short_runs(mode, ds, min_run_m):
    mode = mode.copy()
    n = len(mode)
    i = 0
    while i < n:
        j = i
        while j + 1 < n and mode[j + 1] == mode[i]:
            j += 1
        if ds[i:j + 1].sum() < min_run_m and (i > 0 or j + 1 < n):
            fill = mode[i - 1] if i > 0 else mode[j + 1]
            mode[i:j + 1] = fill
        i = j + 1
    return mode


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--track', default=os.environ.get('RACER_TRACK', 'iros2026'))
    ap.add_argument('--map', help='map yaml (default: the track\'s track_clean.yaml)')
    ap.add_argument('--centreline', help='default: raceline/<track>/centerline_full.csv')
    ap.add_argument('--out', help='default: raceline/<track>/segments.csv')
    ap.add_argument('--every', type=float, default=0.5, help='sample spacing along s [m]')
    ap.add_argument('--stride', type=int, default=3, help='beam subsampling (3 -> ~360 beams)')
    # MUST match the node's beam cuts (config/localization_v2.yaml beam_stride,
    # max_use_m): built with 9.9 m beams the table called the top of the long
    # straight TRANSIT (it could see the far wall) while the node, cutting at
    # 9.5, was blind there -- the guard then fought the table on every scan.
    ap.add_argument('--max-use-m', type=float, default=9.5, help="drop beams beyond this, as the node does")
    ap.add_argument('--sigma', type=float, default=0.12)
    # 20, not the guard's 5: the table is the CONSERVATIVE side. A dozen beams
    # through the middle wall's duct gap gave the straight there along_info
    # 12-25, and the matcher fitted them to 6 mm at a pose 0.10 m down the
    # corridor -- a perfect fit at the wrong place, carried 10 m into hairpin 1.
    # Real corners read 40-125. The live guard (5) stays as the hard floor.
    ap.add_argument('--blind', type=float, default=20.0, help='along_info below this = blind')
    ap.add_argument('--straight-kappa', type=float, default=0.30)
    ap.add_argument('--corner-kappa', type=float, default=0.50)
    ap.add_argument('--approach-m', type=float, default=3.0)
    ap.add_argument('--min-run-m', type=float, default=1.0)
    ap.add_argument('--bench', action='store_true', help='perturb the prior and report recovery')
    ap.add_argument('--perturb', type=float, default=0.30, help='bench prior error [m]')
    ap.add_argument('--no-write', action='store_true')
    a = ap.parse_args()

    from racer_common import frames

    here = rebase_repo_path

    map_yaml = a.map or here(frames.map_yaml(a.track))
    cl_path = a.centreline or here(os.path.join(frames.raceline_dir(a.track), 'centerline_full.csv'))
    out = a.out or here(os.path.join(frames.raceline_dir(a.track), 'segments.csv'))
    spawn = tuple(float(v) for v in frames.spawn(a.track))

    t0 = time.time()
    field = LikelihoodField(map_yaml)
    matcher = ScanMatcher(field, sigma=a.sigma)
    print(f'map {field.w}x{field.h} @ {field.res} m, {int(field.occupied.sum())} occupied cells, '
          f'distance transform {time.time() - t0:.1f} s')

    s, x, y, psi, kappa = load_centreline(cl_path)
    direction = lap_direction(x, y, spawn)
    print(f'centreline {cl_path}: {len(s)} points, {s.max():.1f} m, s runs '
          f'{"WITH" if direction > 0 else "AGAINST"} the lap (spawn test, as the bootstrap does)')

    # Sample every `every` metres of s.
    ds_all = np.abs(np.diff(s, append=s[0] + (s[-1] if direction > 0 else 0)))
    ds_all = np.where(ds_all > 5.0, 0.0, ds_all)
    idx = [0]
    acc = 0.0
    for i in range(1, len(s)):
        acc += ds_all[i - 1]
        if acc >= a.every:
            idx.append(i); acc = 0.0
    idx = np.array(idx)

    angles = np.linspace(ANGLE_MIN, ANGLE_MAX, N_BEAMS)
    along = np.zeros(len(idx)); cross = np.zeros(len(idx))
    beams = np.zeros(len(idx), int); maxr = np.zeros(len(idx))
    t1 = time.time()
    scans = []
    for k, i in enumerate(idx):
        al, cr, nb, mr, r, pts = observe(field, matcher, x[i], y[i], psi[i], angles, a.stride, a.max_use_m)
        along[k], cross[k], beams[k], maxr[k] = al, cr, nb, mr
        scans.append(pts)
    print(f'{len(idx)} poses raycast + matched in {time.time() - t1:.1f} s')

    ds_s = np.full(len(idx), a.every)          # samples are `every` metres apart by construction
    mode = classify(along, kappa[idx], direction, ds_s, blind=a.blind,
                    straight_kappa=a.straight_kappa, corner_kappa=a.corner_kappa,
                    approach_m=a.approach_m, min_run_m=a.min_run_m)

    # ---- the table ---------------------------------------------------------
    # info is sum(w g.g) with |g| ~ 1: effectively "beams pinning this axis".
    # sigma = per-beam noise / sqrt(info); 0.03 m per beam (lidar noise plus
    # the 0.025 m cell) is the scale that puts the columns in metres.
    BEAM_SIGMA = 0.03
    print()
    print(f'{"s":>6} {"kappa":>6} {"along":>8} {"cross":>8} {"sig_al":>7} {"sig_cr":>7} '
          f'{"beams":>5} {"maxr":>5}  mode        (sigmas in m at {BEAM_SIGMA} m/beam)')
    for k, i in enumerate(idx):
        sa = BEAM_SIGMA / math.sqrt(along[k]) if along[k] > 1e-9 else float('inf')
        sc = BEAM_SIGMA / math.sqrt(cross[k]) if cross[k] > 1e-9 else float('inf')
        print(f'{s[i]:6.1f} {kappa[i]:6.2f} {along[k]:8.1f} {cross[k]:8.1f} '
              f'{min(sa, 99.99):7.3f} {min(sc, 99.99):7.3f} {beams[k]:5d} {maxr[k]:5.2f}  {MODES[mode[k]]}')

    # Summary runs, in the file's s order.
    print('\nsegments (file s):')
    k = 0
    while k < len(idx):
        j = k
        while j + 1 < len(idx) and mode[j + 1] == mode[k]:
            j += 1
        print(f'  s {s[idx[k]]:5.1f} - {s[idx[j]]:5.1f}  {MODES[mode[k]]:14s} '
              f'along {along[k:j + 1].min():6.1f}..{along[k:j + 1].max():6.1f}')
        k = j + 1

    # ---- write -------------------------------------------------------------
    if not a.no_write:
        # Every centreline point gets the mode of its nearest sampled point, so
        # the node's nearest-point lookup is over the full-resolution file.
        near = np.abs(s[:, None] - s[idx][None, :]).argmin(1)
        with open(out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['# s_m', 'x_m', 'y_m', 'kappa_radpm', 'along_info', 'cross_info', 'mode'])
            for i in range(len(s)):
                k = near[i]
                w.writerow([f'{s[i]:.5f}', f'{x[i]:.5f}', f'{y[i]:.5f}', f'{kappa[i]:.5f}',
                            f'{along[k]:.2f}', f'{cross[k]:.2f}', MODES[mode[k]]])
        print(f'\nwrote {out}  ({len(s)} points; direction {"with" if direction > 0 else "against"} the lap; '
              f'thresholds blind<{a.blind} corner>={a.corner_kappa} approach {a.approach_m} m)')

    # ---- bench -------------------------------------------------------------
    if a.bench:
        print('\nbench: prior perturbed by +-%.2f m along and cross, then matched' % a.perturb)
        rng = np.random.default_rng(0)
        res = {m: {'along': [], 'cross': [], 'ok': 0, 'n': 0, 'ms': [], 'why': {}} for m in MODES}
        for k, i in enumerate(idx):
            th = psi[i]
            u = np.array([math.cos(th), math.sin(th)]); v = np.array([-u[1], u[0]])
            ang = rng.uniform(0, 2 * math.pi)              # a random one, ON the circle
            for sa_, sc_ in ((a.perturb, 0.0), (-a.perturb, 0.0), (0.0, a.perturb), (0.0, -a.perturb),
                             (a.perturb * math.cos(ang), a.perturb * math.sin(ang))):
                off = sa_ * u + sc_ * v
                t = time.perf_counter()
                r = matcher.match((x[i] + off[0], y[i] + off[1], th), scans[k])
                ms = (time.perf_counter() - t) * 1e3
                rem = off + np.array([r.dx, r.dy])           # error left after the solve
                m = MODES[mode[k]]
                res[m]['n'] += 1; res[m]['ms'].append(ms)
                if r.ok:
                    res[m]['ok'] += 1
                    res[m]['along'].append(float(rem @ u)); res[m]['cross'].append(float(rem @ v))
                else:
                    why = r.reason.split(' ')[0]
                    res[m]['why'][why] = res[m]['why'].get(why, 0) + 1
        print(f'{"mode":14s} {"n":>4} {"ok":>4} {"|along| p90":>12} {"|cross| p90":>12} {"ms p90":>7}  rejections')
        gate_ok = True
        TOL = 0.05
        for m in MODES:
            d = res[m]
            if d['n'] == 0:
                continue
            al = np.abs(d['along']) if d['along'] else np.array([np.nan])
            cr = np.abs(d['cross']) if d['cross'] else np.array([np.nan])
            p90a, p90c = np.nanpercentile(al, 90), np.nanpercentile(cr, 90)
            why = ', '.join(f'{k} x{v}' for k, v in sorted(d['why'].items())) or '-'
            print(f'{m:14s} {d["n"]:4d} {d["ok"]:4d} {p90a:12.3f} {p90c:12.3f} '
                  f'{np.percentile(d["ms"], 90):7.2f}  {why}')
            if m != 'STRAIGHT_BLIND' and (np.isnan(p90a) or p90a > TOL or p90c > TOL):
                gate_ok = False
            if m != 'STRAIGHT_BLIND' and d['ok'] < 0.98 * d['n']:
                gate_ok = False
        blind = mode == 0
        if blind.any():
            sig = BEAM_SIGMA / np.sqrt(np.maximum(along[blind], 1e-9))
            print(f'\nSTRAIGHT_BLIND sigma_along: min {sig.min():.3f} m, median {np.median(sig):.3f} m '
                  f'-- the matcher must NOT claim along-track here; that correction is the odometry\'s.')
        print('\nGATE', 'PASS' if gate_ok else 'FAIL',
              f'(non-blind modes: >= 98 % of perturbations accepted, recovered to < {TOL} m p90)')
        return 0 if gate_ok else 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
