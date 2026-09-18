#!/usr/bin/env python3
"""Minimum-LAP-TIME trajectory, as one nonlinear program.

    python raceline/opt_mintime.py raceline/iros2026/raceline_tum_iqp_h7.0_a7.0b.csv \
        -o raceline/iros2026/raceline_mintime.csv --a-lat 7.0 --a-long 5.0 --a-brake 5.5 \
        --v-max 8.5 --kappa-max 1.25

Why this exists
---------------
`optimize_raceline.py` and `centreline_editor/make_raceline.py` are TWO steps:
a QP finds the least-curvature line that fits the corridor, then a velocity
profile is fitted to that fixed geometry. The first step has no notion of time,
so it cannot trade a little extra curvature somewhere for a better exit onto a
straight. This solves the line and the speed TOGETHER, minimising lap time.

On IROS 2026 that trade is exactly what the car appears to want: through the
hairpin exits it burns 6.65-6.90 m/s^2 of lateral on a line whose geometry only
asks 2.45-5.21, i.e. it is fighting the shape rather than accelerating.

Formulation
-----------
Decision variables, one per path point (closed loop, 435 points here):
    n_i   lateral offset from the REFERENCE line along its normal, + left
    v_i   speed at that point
Objective:  sum over i of  2 ds_i / (v_i + v_{i+1})     (trapezoid in time)
Curvature is Menger curvature through the three moved points, which is exact
for a circle and differentiable, so the geometry and the speed see the same
corridor. Constraints per point:
    n in [-(w_r - c), +(w_l - c)],  c = half width + margin, clipped so the
        REFERENCE line itself is always feasible (some points have no spare)
    friction ellipse   (a_x/a_x_lim)^2 + (a_y/a_lat)^2 <= 1
    a_y = v^2 kappa,   a_x from the speed change between points
    longitudinal budgets separate for drive and brake, drag on the correct side
    |kappa| <= kappa_max        the car cannot produce more (measured 1.28 p99.9)
The reference line is a normal input, so this REFINES a good line rather than
starting from a centreline; n = 0 is always feasible and is the initial guess,
which means the solver can only improve on what you already race.

Only standard library + numpy + casadi. Run from .venv-rl.
"""
import argparse
import os
import sys

import numpy as np

try:
    import casadi as ca
except ImportError:                                            # pragma: no cover
    sys.exit('casadi is required: run this from .venv-rl')

G = 9.81
DRAG = 0.273          # 1/s, Unity Rigidbody drag (VEHICLE_MODEL.md)


def load_line(path):
    D = np.loadtxt(path, delimiter=',')
    if D.ndim != 2 or D.shape[1] < 7:
        sys.exit(f'{path}: not a raceline CSV (need at least 7 columns)')
    return dict(s=D[:, 0], x=D[:, 1], y=D[:, 2], psi=D[:, 3], kappa=D[:, 4],
                wr=D[:, 5], wl=D[:, 6], v=D[:, 7] if D.shape[1] > 7 else None)



def margin_from_run(run_csv, L, pct, floor, smooth_m, ds):
    """Per-point wall margin = the lateral error the car really shows there.

    A uniform margin assumes the car wanders equally everywhere, and it does
    not: on IROS 2026 it holds its line to 0.069 m (p95) down the main straight
    and to 0.278 m through hairpin 1, a factor of four. Feeding the measured
    profile in lets the optimiser spend corridor where the car is accurate and
    keeps it back where the car is not.
    """
    import csv as _csv
    rows = list(_csv.DictReader(open(run_csv)))
    col = lambda k: np.array([float(r[k]) if r[k] not in ('', 'nan') else np.nan for r in rows])
    x, y, v = col('true_x'), col('true_y'), col('speed')
    rd = col('ready') if 'ready' in rows[0] else np.ones(len(x))
    keep = np.r_[True, (np.diff(x) != 0) | (np.diff(y) != 0)]
    x, y, v, rd = x[keep], y[keep], v[keep], rd[keep]
    m = (v > 0.5) & (rd > 0.5)
    x, y = x[m], y[m]
    j = ((x[:, None] - L['x'][None, :]) ** 2 + (y[:, None] - L['y'][None, :]) ** 2).argmin(1)
    e = np.abs((x - L['x'][j]) * (-np.sin(L['psi'][j])) + (y - L['y'][j]) * np.cos(L['psi'][j]))
    n_pts = len(L['s'])
    out = np.full(n_pts, floor)
    half = max(1, int(round(0.5 * smooth_m / max(ds, 1e-6))))
    for i in range(n_pts):
        idx = [(i + k) % n_pts for k in range(-half, half + 1)]
        sel = np.isin(j, idx)
        if sel.sum() >= 20:
            out[i] = max(floor, float(np.percentile(e[sel], pct)))
    # one more pass so the corridor bound does not step
    k = np.ones(2 * half + 1) / (2 * half + 1)
    return np.convolve(np.r_[out[-half:], out, out[:half]], k, mode='same')[half:half + n_pts]


def menger(px, py, i0, i1, i2):
    """Signed curvature of the circle through three points, + left."""
    ax, ay = px[i1] - px[i0], py[i1] - py[i0]
    bx, by = px[i2] - px[i1], py[i2] - py[i1]
    cx, cy = px[i2] - px[i0], py[i2] - py[i0]
    cross = ax * by - ay * bx
    la = ca.sqrt(ax ** 2 + ay ** 2 + 1e-9)
    lb = ca.sqrt(bx ** 2 + by ** 2 + 1e-9)
    lc = ca.sqrt(cx ** 2 + cy ** 2 + 1e-9)
    return 2.0 * cross / (la * lb * lc)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('line', help='reference raceline CSV (the line you race now)')
    p.add_argument('-o', '--out', required=True)
    p.add_argument('--a-lat', type=float, default=7.0)
    p.add_argument('--a-long', type=float, default=5.0)
    p.add_argument('--a-brake', type=float, default=5.5)
    p.add_argument('--v-max', type=float, default=8.5)
    p.add_argument('--v-min', type=float, default=1.0)
    p.add_argument('--kappa-max', type=float, default=1.25,
                   help='the car produced 1.28 at p99.9 over 63 laps; above that it runs wide')
    p.add_argument('--steer-rate', type=float, default=3.2,
                   help='rad/s the steering may be asked to slew, |L/(1+(kL)^2) dk/ds| v. The actuator limit is '
                        '3.2. Leaving it unconstrained is what broke the first min-time line on IROS 2026: it '
                        'needed 4.47 rad/s at s 21.6 and the car washed WIDE there on ten of twelve contacts, '
                        'because a rate-saturated steering command cannot rotate the car fast enough to hold the '
                        'geometry. The raced min-curvature line only ever needs 1.00, so a limit well below 3.2 '
                        'also leaves the controller slew to correct with, which the line itself does not use.\n'
                        'The NLP measures curvature as a circle through three adjacent points, which is right '
                        'for the curvature itself (1.25 against the spline 1.26) but smooths its DERIVATIVE, '
                        'and the rate depends on the derivative: 2.91 by that measure against 5.04 by spline on '
                        'the same line. So the limit is auto-calibrated -- solve, measure the rate the way the '
                        'profiler does, tighten the internal limit by the ratio, repeat. See --rate-iters.')
    p.add_argument('--wheelbase', type=float, default=0.3240)
    p.add_argument('--margin', type=float, default=0.20,
                   help='UNIFORM wall margin per side on top of half the car width. Prefer --margin-from: a single '
                        'number cannot be right everywhere. Measured over 63 laps on IROS 2026 the car tracks its '
                        'line to 0.088 m at p99 down the main straight and to 0.369 m through hairpin 1, so 0.20 is '
                        'simultaneously 0.11 m too generous on the straights and 0.17 m too thin in the hairpin -- '
                        'which is exactly why the apex has always been the tight spot, and why the first min-time '
                        'line spent the corridor at the hairpin-1 EXIT (line clearance 0.660 -> 0.369 m) and put a '
                        'body corner into the wall there on its first clean lap.')
    p.add_argument('--margin-from', default='',
                   help='a log_localization run CSV. The margin at each point becomes the lateral error the car '
                        'ACTUALLY shows there, so the optimiser keeps room where the car wanders and is free to use '
                        'the corridor where it does not.')
    p.add_argument('--margin-pct', type=float, default=99.0, help='percentile of |e_lat| used by --margin-from')
    p.add_argument('--margin-floor', type=float, default=0.10)
    p.add_argument('--margin-buffer', type=float, default=0.10,
                   help='added on top of the measured percentile. This is the knob that matters: a min-time '
                        'solver spends every centimetre of corridor it is given, because clearance it does not '
                        'use is time it does not get, so the margin IS the design rather than a safety detail. '
                        'Measured on IROS 2026, the exchange rate is roughly 0.5 s of lap for 0.1 m of clearance '
                        'across the lap. Pick it from what a contact costs (10 s under the rules), not from what '
                        'makes the predicted lap look good.')
    p.add_argument('--margin-smooth', type=float, default=1.5, help='m, window the margin profile is smoothed over')
    p.add_argument('--margin-zones', default='',
                   help='s0:s1:m[,s0:s1:m...]: FLOOR on the margin inside each s-range, and inside a zone the wall '
                        'bound is not clipped at n=0, so a reference line already inside the margin is pushed out. '
                        'Placed from analyze_run contacts: on IROS 2026 the car exits the s 36-37.5 left-hander '
                        '0.13-0.22 m wide into a right wall 0.39 m off the line at s 39, and three variants hit there.')
    p.add_argument('--half-width', type=float, default=0.135)
    p.add_argument('--rate-iters', type=int, default=4,
                   help='re-solve this many times, each time tightening the internal steering-rate limit by the '
                        'ratio between the TRUE (spline) rate of the solution and the target. 1 disables it.')
    p.add_argument('--max-iter', type=int, default=3000)
    a = p.parse_args()

    L = load_line(a.line)
    n_pts = len(L['s'])
    ds_ref = float(np.median(np.diff(L['s'])))
    lap_ref = float(L['s'][-1] + ds_ref)
    # unit normals, + left, from the exported atan2 heading
    nx, ny = -np.sin(L['psi']), np.cos(L['psi'])

    if a.margin_from:
        marg = margin_from_run(a.margin_from, L, a.margin_pct, a.margin_floor, a.margin_smooth, ds_ref) + a.margin_buffer
        print(f'margin from {os.path.basename(a.margin_from)} at p{a.margin_pct:g}: '
              f'{marg.min():.3f}..{marg.max():.3f} m, mean {marg.mean():.3f} (uniform would be {a.margin:.3f})')
    else:
        marg = np.full(n_pts, a.margin)
    inzone = np.zeros(n_pts, bool)
    for spec in [z for z in a.margin_zones.split(',') if z]:
        s0, s1, m = (float(v) for v in spec.split(':'))
        z = ((L['s'] >= s0) & (L['s'] <= s1)) if s0 <= s1 else ((L['s'] >= s0) | (L['s'] <= s1))
        marg[z] = np.maximum(marg[z], m)
        inzone |= z
        print(f'margin zone s {s0:g}-{s1:g}: floor {m:.3f} m on {int(z.sum())} points')
    c = a.half_width + marg
    lo = np.minimum(0.0, -(L['wr'] - c))        # clipped so n = 0 is always feasible
    hi = np.maximum(0.0, (L['wl'] - c))
    if inzone.any():
        # inside a zone the reference may sit within the margin: let the bound go
        # positive (forces the line left) but never past the other wall's bound
        lo_z = -(L['wr'] - c)
        lo[inzone] = np.minimum(lo_z[inzone], hi[inzone] - 1e-3)
        hi_z = (L['wl'] - c)
        hi[inzone] = np.maximum(hi_z[inzone], lo[inzone] + 1e-3)
        f = inzone & (lo > 1e-6)
        print(f'  zone forces the line LEFT on {int(f.sum())} points, up to {lo[f].max() if f.any() else 0:.3f} m')
    print(f'reference {os.path.basename(a.line)}: {n_pts} points, {lap_ref:.2f} m, '
          f'|kappa|max {np.abs(L["kappa"]).max():.2f}')
    print(f'corridor after {c.mean():.3f} m of body+margin (mean): can move left on {100*np.mean(hi>1e-6):.0f} % of points, '
          f'right on {100*np.mean(lo<-1e-6):.0f} %')

    opti = ca.Opti()
    n = opti.variable(n_pts)
    v = opti.variable(n_pts)
    rate_lim = opti.parameter()          # tightened between passes, see --rate-iters
    px = L['x'] + nx * n
    py = L['y'] + ny * n

    ip = [(i + 1) % n_pts for i in range(n_pts)]
    im = [(i - 1) % n_pts for i in range(n_pts)]

    seg = [ca.sqrt((px[ip[i]] - px[i]) ** 2 + (py[ip[i]] - py[i]) ** 2 + 1e-9) for i in range(n_pts)]
    kap = [menger(px, py, im[i], i, ip[i]) for i in range(n_pts)]

    t_lap = sum(2.0 * seg[i] / (v[i] + v[ip[i]]) for i in range(n_pts))
    opti.minimize(t_lap)

    for i in range(n_pts):
        j = ip[i]
        opti.subject_to(opti.bounded(lo[i], n[i], hi[i]))
        opti.subject_to(opti.bounded(a.v_min, v[i], a.v_max))
        opti.subject_to(opti.bounded(-a.kappa_max, kap[i], a.kappa_max))
        # longitudinal acceleration implied by the speed change over this segment
        ax = (v[j] ** 2 - v[i] ** 2) / (2.0 * seg[i])
        opti.subject_to(ax <= a.a_long - DRAG * v[i])          # drive, drag hinders
        opti.subject_to(-ax <= a.a_brake + DRAG * v[j])        # brake, drag helps
        ay = v[i] ** 2 * kap[i]
        # Friction ellipse. a_long is used for the x half-axis in both directions:
        # a_brake is the larger budget, so this is the conservative choice and
        # keeps the constraint smooth (no branch on the sign of ax).
        opti.subject_to((ax / a.a_long) ** 2 + (ay / a.a_lat) ** 2 <= 1.0)
        # Steering slew the GEOMETRY demands at this speed, same expression as
        # optimize_raceline.steering_rate_required. Two-sided rather than abs()
        # so the constraint stays smooth for IPOPT.
        if a.steer_rate > 0.0:
            dk = (kap[ip[i]] - kap[im[i]]) / (seg[i] + seg[im[i]])
            rate = a.wheelbase / (1.0 + (kap[i] * a.wheelbase) ** 2) * dk * v[i]
            opti.subject_to(opti.bounded(-rate_lim, rate, rate_lim))

    opti.set_initial(n, 0.0)
    opti.set_initial(v, L['v'] if L['v'] is not None else 0.5 * a.v_max)
    opti.solver('ipopt', {'print_time': False},
                {'max_iter': a.max_iter, 'print_level': 0, 'sb': 'yes',
                 'tol': 1e-6, 'acceptable_tol': 1e-4})

    def true_rate(n_vec, v_vec):
        """Steering slew the profiler will report: spline curvature, not the NLP's."""
        try:
            from optimize_raceline import spline_geometry, steering_rate_required
        except ImportError:
            return None
        gx, gy = L['x'] + nx * n_vec, L['y'] + ny * n_vec
        g = spline_geometry(gx, gy)
        return float(np.max(np.abs(steering_rate_required(g['kappa'], g['el'], v_vec))))

    lim = a.steer_rate
    n_o = v_o = None
    for it in range(max(1, a.rate_iters)):
        opti.set_value(rate_lim, lim)
        try:
            sol = opti.solve()
            n_i, v_i = np.array(sol.value(n)).ravel(), np.array(sol.value(v)).ravel()
            status = 'solved'
        except RuntimeError:
            n_i, v_i = np.array(opti.debug.value(n)).ravel(), np.array(opti.debug.value(v)).ravel()
            status = 'did NOT converge, using last iterate'
        tr = true_rate(n_i, v_i)
        keep = tr is None or tr <= a.steer_rate * 1.02
        print(f'  pass {it+1}: internal limit {lim:.2f} -> IPOPT {status}, TRUE rate '
              f'{"n/a" if tr is None else f"{tr:.2f}"} rad/s (target {a.steer_rate:.2f})'
              + ('  ACCEPTED' if keep else ''))
        n_o, v_o = n_i, v_i
        if keep or tr is None or a.rate_iters == 1:
            break
        lim *= max(0.35, (a.steer_rate / tr) ** 0.9)      # tighten toward the target
    if n_o is None:
        sys.exit('no solution')

    X = L['x'] + nx * n_o
    Y = L['y'] + ny * n_o
    seg_o = np.hypot(np.roll(X, -1) - X, np.roll(Y, -1) - Y)
    s_o = np.concatenate([[0.0], np.cumsum(seg_o)[:-1]])
    psi_o = np.arctan2(np.gradient(Y), np.gradient(X))
    dx, dy = np.gradient(X), np.gradient(Y)
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    kap_o = (dx * ddy - dy * ddx) / np.power(dx ** 2 + dy ** 2, 1.5)
    t_o = float(np.sum(seg_o / np.maximum(0.5 * (v_o + np.roll(v_o, -1)), 0.1)))
    t_in = float(np.sum(np.diff(np.append(L['s'], lap_ref)) / L['v'])) if L['v'] is not None else float('nan')
    print(f'lap: reference {t_in:.3f} s  ->  min-time {t_o:.3f} s   ({t_o - t_in:+.3f} s)')
    print(f'     length {lap_ref:.2f} -> {seg_o.sum():.2f} m,  |kappa|max {np.abs(kap_o).max():.2f}, '
          f'v {v_o.min():.2f}..{v_o.max():.2f},  offset |n| mean {np.abs(n_o).mean():.3f} max {np.abs(n_o).max():.3f} m')

    # widths carried over from the reference, shifted by the offset we moved
    wr_o = L['wr'] + n_o
    wl_o = L['wl'] - n_o
    out = np.column_stack([s_o, X, Y, psi_o, kap_o, wr_o, wl_o, v_o])
    np.savetxt(a.out, out, delimiter=',', fmt='%.6f')
    print(f'wrote {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
