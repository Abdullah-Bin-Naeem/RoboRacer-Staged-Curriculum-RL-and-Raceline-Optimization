#!/usr/bin/env python3

"""Score a log_localization CSV: how good is the localizer, and does it matter.

    python3 analyze_localization.py runs/baseline.csv
    python3 analyze_localization.py runs/*.csv --compare runs/compare.html
    python3 analyze_localization.py runs/baseline.csv --path raceline_a6.5.csv

Writes <run>.html (charts) and <run>.json (numbers, for --compare to rank)
beside the CSV, and prints the headline to stdout.

Standard library plus numpy, which is what the devkit image has. Reuses the
loaders from raceline/analyze_run.py on the main branch; everything specific to
localization is here. analyze_run.py answers "how did the CAR do"; this answers
"how good was the POSE it drove on", which is a different question with a
different set of failure modes.

WHAT IS BEING COMPARED
----------------------
    truth      /ips position + /imu heading -- the simulator's own state
    estimate   TF map->odom composed with odom->roboracer_1, i.e. AMCL's
               correction applied to dead reckoning. That composition is
               literally what pure_pursuit reads, so it is the pose the car
               really drives on -- not /amcl_pose, which is lower-rate and
               steps discontinuously.

Both are already in the CSV; this script does not talk to ROS.

THE ERROR FLOOR -- READ THIS BEFORE BELIEVING A SMALL NUMBER
------------------------------------------------------------
The estimate lives in `map`, the truth lives in `world`, and comparing them at
all assumes the two frames coincide. They do, to about 0.024 m RMS and 0.19 deg
(the SLAM map was built against ground-truth odometry). So roughly 0.03 m of
whatever this reports is frame alignment rather than the localizer, and a
result at that magnitude is the floor, not a bug. FLOOR_M below.

THE FOUR QUESTIONS, IN THE ORDER THEY SHOULD BE ASKED
-----------------------------------------------------
1. Does the error matter?      |err_cross| against the wall margin at that
                               point of the lap. Everything else is academic
                               if the answer is no.
2. Is AMCL doing anything?     err_dist against dr_err -- dead reckoning
                               alone, on the SAME trajectory at the SAME
                               instants, which is already a column in every
                               run. A paired comparison, no second run needed.
3. Is it rotating the pose?    m2o_yaw_deg must be zero. dead_reckoning takes
                               heading from the IMU's absolute quaternion, so
                               `odom` is yaw-locked to `world`; the map is
                               aligned to `world` too. The only correction a
                               correct localizer can make is a TRANSLATION.
                               Every degree here is the scan matcher rotating
                               away from a heading that was already exact.
4. Whose fault is the error?   fit_ratio = fit_est / fit_true, the scan scored
                               against the map at the estimated pose versus at
                               the true one.
                                 > 1  a better pose existed and was not found
                                      -- tune AMCL
                                 ~ 1  both poses explain the scan equally well
                                      -- the map cannot tell them apart, and
                                      no filter tuning will help
"""

import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import svg_report as R                                            # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RACELINE_DIRS = [
    os.path.join(HERE, os.pardir, 'raceline'),                    # in the repo
    '/home/autodrive_devkit/raceline',                            # bench mount
    '/home/autodrive_devkit/install/roboracer_stack/share/roboracer_stack/raceline',
]

# map<->world alignment residual; see the module docstring.
FLOOR_M = 0.03
# Half the car's body width. Derived, not guessed: the tightest point of
# raceline_a7.0 has w_right = 0.28125 m, and localization_error.WALL_MARGIN_M
# records the body-to-wall clearance there as 0.134 m. 0.28125 - 0.134 = 0.1475.
CAR_HALF_W = 0.1475
# The occupancy grid's cell size (maps/track_clean.yaml). fit_true / fit_est are
# distances measured against that grid, so a difference smaller than a cell is
# below the map's own resolving power and the ratio between them means nothing
# however large it looks.
MAP_RES_M = 0.05
# Nominal lookahead, used only if the follower did not publish its own. The
# configured range is 0.8-2.2 m.
LOOKAHEAD_NOMINAL_M = 1.3


# --------------------------------------------------------------------------
# Loaders. Ported from raceline/analyze_run.py on main -- same CSV, same
# sample-and-hold problem, so the same fix applies.
# --------------------------------------------------------------------------

def load_log(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        sys.exit(f'{path}: empty')
    cols = {}
    for k in rows[0]:
        try:
            cols[k] = np.array([float(r[k]) if r[k] not in ('', 'nan', 'NaN')
                                else np.nan for r in rows])
        except ValueError:
            pass                                       # non-numeric column
    for need in ('t', 'true_x', 'true_y', 'err_dist'):
        if need not in cols:
            sys.exit(f'{path}: no {need} column -- is this a log_localization CSV?')
    # The logger samples at 20 Hz off an ~18 Hz bridge, so consecutive rows
    # repeat the same truth. Keeping them would weight held samples double and
    # make every rate look like the logger's, not the simulator's.
    keep = np.r_[True, (np.diff(cols['true_x']) != 0) | (np.diff(cols['true_y']) != 0)]
    out = {k: v[keep] for k, v in cols.items()}
    out['_n_raw'] = len(rows)
    return out


def load_line(path):
    D = np.loadtxt(path, delimiter=',')
    if D.ndim != 2 or D.shape[1] < 7:
        return None
    return dict(name=os.path.basename(path), s=D[:, 0], x=D[:, 1], y=D[:, 2],
                psi=D[:, 3], kappa=D[:, 4],
                # Distance from the line to the track edge on each side. The
                # wall-margin section is the only thing that reads these, and
                # analyze_run.py on main drops them.
                w_right=D[:, 5], w_left=D[:, 6],
                v=D[:, 7] if D.shape[1] > 7 else None,
                L=float(D[-1, 0] + (D[1, 0] - D[0, 0])))


def project(x, y, line):
    """Nearest point on the line: signed lateral offset (+ left), kappa, s, index."""
    j = ((x[:, None] - line['x'][None, :]) ** 2
         + (y[:, None] - line['y'][None, :]) ** 2).argmin(1)
    e = ((x - line['x'][j]) * (-np.sin(line['psi'][j]))
         + (y - line['y'][j]) * np.cos(line['psi'][j]))
    return e, line['kappa'][j], line['s'][j], j


def pick_line(log, mov, explicit):
    if explicit:
        line = load_line(explicit)
        if line is None:
            sys.exit(f'{explicit}: not a raceline CSV')
        return line
    cands = []
    for d in RACELINE_DIRS:
        cands += sorted(glob.glob(os.path.join(d, '*.csv')))
    scored, seen = [], set()
    for f in cands:
        if os.path.basename(f) in seen:
            continue
        seen.add(os.path.basename(f))
        line = load_line(f)
        if line is None:
            continue
        e, _, _, _ = project(log['true_x'][mov], log['true_y'][mov], line)
        scored.append((float(np.median(np.abs(e))), line))
    if not scored:
        sys.exit('no raceline CSV found; pass --path')
    scored.sort(key=lambda r: r[0])
    best_med, best = scored[0]
    # The three shipped lines are the SAME geometry with different velocity
    # profiles, so on geometry alone the match is a tie and which one wins is
    # arbitrary. That is harmless for the error statistics, which only use x/y
    # and the widths -- but the lap-time column and the wall margins come from
    # the line, so say so rather than pick silently.
    if len(scored) > 1 and scored[1][0] < 1.5 * max(best_med, 1e-6):
        print(f'NOTE: line auto-detection is ambiguous -- {best["name"]} '
              f'({best_med:.3f} m) vs {scored[1][1]["name"]} ({scored[1][0]:.3f} m). '
              f'The shipped lines share one geometry, so this only matters for the '
              f'lap-time and wall-margin columns; pass --path to pin it.')
    if best_med > 0.5:
        print(f'NOTE: closest line is {best["name"]} at {best_med:.2f} m median '
              'offset -- the run may not have followed a shipped line')
    return best


def pct(a, q):
    a = np.asarray(a)
    a = a[np.isfinite(a)]
    return float(np.percentile(a, q)) if a.size else float('nan')


def nanmean(a):
    a = np.asarray(a)
    a = a[np.isfinite(a)]
    return float(a.mean()) if a.size else float('nan')


def nanmax(a):
    a = np.asarray(a)
    a = a[np.isfinite(a)]
    return float(a.max()) if a.size else float('nan')


def laps_from_s(s, t, L):
    wraps = np.where((s[:-1] > 0.8 * L) & (s[1:] < 0.2 * L))[0]
    return wraps + 1, t[wraps + 1]


def pearson(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    m = np.isfinite(a) & np.isfinite(b)
    if m.sum() < 10:
        return float('nan')
    a, b = a[m], b[m]
    if a.std() < 1e-12 or b.std() < 1e-12:
        return float('nan')
    return float(np.corrcoef(a, b)[0, 1])


def bin_stats(x, y, edges):
    """Mean, p90 and count of y in each bin of x. NaN where a bin is empty."""
    idx = np.digitize(x, edges) - 1
    mean, p90, cnt = [], [], []
    for b in range(len(edges) - 1):
        m = (idx == b) & np.isfinite(y)
        cnt.append(int(m.sum()))
        mean.append(nanmean(y[m]) if m.sum() else float('nan'))
        p90.append(pct(y[m], 90) if m.sum() else float('nan'))
    return np.array(mean), np.array(p90), np.array(cnt)


# --------------------------------------------------------------------------
# The analysis
# --------------------------------------------------------------------------

def find_converged(t, err, thresh, hold_s):
    """First time the error drops below `thresh` and STAYS around there.

    Not "first time it dips below": AMCL's particle cloud crosses any threshold
    on its way down long before it has settled, and scoring the bootstrap as
    part of the run is what used to make the peak-error warning fire on every
    run.

    "Stays" is the MEDIAN of the next hold_s, not its maximum. Requiring the
    maximum was the obvious rule and it is wrong: a run with one genuinely bad
    corner re-crosses the threshold once a lap forever, so it never converges
    at all and the whole run gets scored as bootstrap. The median tolerates a
    recurring excursion while still rejecting a filter that has not settled --
    which is the actual distinction being drawn.
    """
    n = len(t)
    for i in range(n):
        if not np.isfinite(err[i]) or err[i] >= thresh:
            continue
        j = i
        while j < n and t[j] - t[i] < hold_s:
            j += 1
        w = err[i:j]
        w = w[np.isfinite(w)]
        if w.size and float(np.median(w)) < thresh:
            return i
    return None


def analyze(path, args):
    log = load_log(path)
    name = os.path.splitext(os.path.basename(path))[0]
    t, v = log['t'], log['speed']
    mov = v > args.min_speed
    if mov.sum() < 30:
        print(f'{path}: fewer than 30 moving samples, nothing to say')
        return None
    line = pick_line(log, mov, args.path)

    err = log['err_dist']
    # --- the analysis window ------------------------------------------------
    i_conv = find_converged(t, np.where(mov, err, np.nan),
                            args.converge_m, args.hold_s)
    if args.from_s is not None:
        i0 = int(np.searchsorted(t, args.from_s))
    else:
        i0 = i_conv if i_conv is not None else 0
    win = np.zeros(len(t), bool)
    win[i0:] = True
    win &= mov
    if win.sum() < 30:
        print(f'{path}: only {win.sum()} samples after convergence; '
              'falling back to all moving samples')
        win = mov
        i0 = 0

    conv_t = float(t[i_conv]) if i_conv is not None else float('nan')
    conv_d = float(log['dist_m'][i_conv]) if i_conv is not None else float('nan')

    # --- geometry -----------------------------------------------------------
    tx, ty = log['true_x'][win], log['true_y'][win]
    e_true, kap, s_true, j_true = project(tx, ty, line)
    tw, vw = t[win], v[win]
    ed, ec, ea = log['err_dist'][win], log['err_cross'][win], log['err_along'][win]
    eyaw = log['err_yaw_deg'][win]
    dr = log['dr_err'][win] if 'dr_err' in log else np.full(win.sum(), np.nan)
    m2o = log['m2o_yaw_deg'][win] if 'm2o_yaw_deg' in log else np.full(win.sum(), np.nan)
    yrate = log['yaw_rate'][win] if 'yaw_rate' in log else np.full(win.sum(), np.nan)
    fitr = log['fit_ratio'][win] if 'fit_ratio' in log else np.full(win.sum(), np.nan)

    # --- laps ---------------------------------------------------------------
    lap_i, lap_t = laps_from_s(s_true, tw, line['L'])
    lap_times = np.diff(lap_t)

    tick_hz = len(t) / max(t[-1] - t[0], 1e-6)

    # --- wall margin --------------------------------------------------------
    # err_cross > 0 means the estimate sits to the car's LEFT. The controller
    # then believes it is further left than it is and steers right, so the
    # margin the error eats into is the one on the RIGHT.
    w_r = line['w_right'][j_true] - CAR_HALF_W
    w_l = line['w_left'][j_true] - CAR_HALF_W
    margin = np.where(ec > 0, w_r, w_l)
    margin = np.maximum(margin, 1e-3)
    consumed = np.abs(ec) / margin
    frac_half = float(np.mean(consumed > 0.5) * 100.0)
    frac_all = float(np.mean(consumed > 1.0) * 100.0)
    tight = float(np.min(np.minimum(w_r, w_l)))

    st = {
        'name': name, 'file': os.path.abspath(path), 'line': line['name'],
        'duration_s': float(t[-1] - t[0]), 'distance_m': float(log['dist_m'][-1]),
        'samples': int(win.sum()), 'tick_hz': float(tick_hz),
        'laps': int(len(lap_times)),
        'lap_best_s': float(lap_times.min()) if len(lap_times) else float('nan'),
        'lap_mean_s': float(lap_times.mean()) if len(lap_times) else float('nan'),
        'converge_t_s': conv_t, 'converge_dist_m': conv_d,
        'pos_mean': nanmean(ed), 'pos_p50': pct(ed, 50), 'pos_p90': pct(ed, 90),
        'pos_p95': pct(ed, 95), 'pos_max': nanmax(ed),
        'cross_absmean': nanmean(np.abs(ec)), 'cross_p95': pct(np.abs(ec), 95),
        'cross_bias': nanmean(ec),
        'along_absmean': nanmean(np.abs(ea)), 'along_bias': nanmean(ea),
        'yaw_absmean': nanmean(np.abs(eyaw)), 'yaw_p95': pct(np.abs(eyaw), 95),
        'yaw_max': nanmax(np.abs(eyaw)), 'yaw_bias': nanmean(eyaw),
        'dr_mean': nanmean(dr), 'dr_max': nanmax(dr),
        'm2o_yaw_mean': nanmean(m2o), 'm2o_yaw_std': float(np.nanstd(m2o))
        if np.isfinite(m2o).any() else float('nan'),
        'm2o_yaw_absmax': nanmax(np.abs(m2o)),
        'fit_ratio_med': pct(fitr, 50), 'fit_ratio_p90': pct(fitr, 90),
        'margin_tightest_m': tight,
        'margin_half_pct': frac_half, 'margin_over_pct': frac_all,
        'r_speed': pearson(vw, ed), 'r_kappa': pearson(np.abs(kap), ed),
        'r_yawrate': pearson(np.abs(yrate), ed),
        'floor_m': FLOOR_M,
    }
    # The mechanism test for a rotating correction: does the RATE of change of
    # m2o_yaw track the car's turn rate? If it does, the cause is a timestamp,
    # not a filter parameter.
    # Two different mechanisms leave two different fingerprints, and testing
    # only one of them reports "not the timing" when it was.
    #   m2o_yaw itself proportional to yaw_rate  -> a stale heading, or a lever
    #        arm: the error appears while turning and vanishes on the straights.
    #   d(m2o_yaw)/dt proportional to yaw_rate   -> the error is being
    #        INTEGRATED on every update, which on a loop driven one way round
    #        never cancels.
    if np.isfinite(m2o).sum() > 30:
        st['r_m2o_yawrate'] = pearson(yrate, m2o)
        st['r_m2odot_yawrate'] = pearson(yrate, np.gradient(m2o, tw))
    else:
        st['r_m2o_yawrate'] = float('nan')
        st['r_m2odot_yawrate'] = float('nan')
    # Heading error is not a cosmetic number. pure_pursuit steers at a point one
    # lookahead ahead, so a heading error rotates that aim point sideways by
    # ld * sin(err_yaw) -- and THAT lands in the same units, and against the same
    # wall margin, as cross-track error. A degree of heading at a 2 m lookahead
    # is 3.5 cm of aim; five degrees is 17 cm, which is more than the margin.
    ld = log['pp_ld'][win] if 'pp_ld' in log else np.full(win.sum(), np.nan)
    if not np.isfinite(ld).any():
        ld = np.full(win.sum(), LOOKAHEAD_NOMINAL_M)
    aim = np.abs(np.sin(np.radians(eyaw))) * ld
    st['aim_absmean'] = nanmean(aim)
    st['aim_p95'] = pct(aim, 95)
    st['aim_max'] = nanmax(aim)
    st['ld_mean'] = nanmean(ld)
    # Total lateral displacement the controller can be misled by: the estimate
    # being sideways, plus the estimate being rotated, at the aim point.
    st['lat_total_p95'] = pct(np.abs(ec) + aim, 95)

    st['fit_true_med'] = pct(log['fit_true'][win], 50) if 'fit_true' in log else float('nan')
    st['fit_est_med'] = pct(log['fit_est'][win], 50) if 'fit_est' in log else float('nan')
    st['fit_gap_med'] = (st['fit_est_med'] - st['fit_true_med']
                         if np.isfinite(st['fit_est_med']) else float('nan'))

    st['amcl_gain'] = (st['dr_mean'] / st['pos_mean']
                       if st['pos_mean'] > 1e-9 and np.isfinite(st['dr_mean'])
                       else float('nan'))

    _report(st, log, line, win, i0, tw, vw, ed, ec, ea, eyaw, dr, m2o, yrate,
            fitr, s_true, kap, consumed, margin, lap_i, lap_times, path, args)
    # Not written to the JSON -- only carried in memory for --compare, which
    # needs the samples themselves to overlay distributions rather than the
    # summary of them.
    st['_err'] = ed
    return st


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------

def _verdicts(st):
    """The decision tree, as (level, text). Order is the order to read them."""
    out = []
    floor = st['floor_m']

    # 1. does it matter
    if st['margin_over_pct'] > 1.0:
        out.append(('bad', f"Cross-track error exceeds the available wall margin "
                           f"{st['margin_over_pct']:.1f} % of the lap. At those points the "
                           f"controller is steering the car into a wall and only the "
                           f"tyre model is saving it. This is worth fixing."))
    elif st['margin_half_pct'] > 10.0:
        out.append(('warn', f"Cross-track error eats more than half the wall margin "
                            f"{st['margin_half_pct']:.1f} % of the lap (tightest point "
                            f"{st['margin_tightest_m']:.3f} m). There is no room left for a "
                            f"faster line."))
    else:
        out.append(('good', f"Cross-track error stays inside half the wall margin "
                            f"{100 - st['margin_half_pct']:.1f} % of the lap. Localization is "
                            f"not what is limiting the lap time -- look at the "
                            f"raceline and the controller instead."))

    # 1b. the heading half of "does it matter"
    if np.isfinite(st['aim_p95']):
        tot = st['lat_total_p95']
        if tot > st['margin_tightest_m']:
            out.append(('bad', f"Heading error moves the pure-pursuit aim point sideways "
                               f"by {st['aim_p95']:.3f} m at p95 (mean lookahead "
                               f"{st['ld_mean']:.2f} m). Added to the cross-track error that "
                               f"is {tot:.3f} m of lateral displacement the controller can "
                               f"be misled by, against a {st['margin_tightest_m']:.3f} m "
                               f"margin. The heading is the half of this that matters, not "
                               f"the position."))
        elif st['aim_p95'] > st['cross_p95'] * 2:
            out.append(('warn', f"Cross-track error is small ({st['cross_p95']:.3f} m at p95) "
                                f"but heading error is not: {st['yaw_p95']:.2f} deg swings the "
                                f"aim point {st['aim_p95']:.3f} m at a {st['ld_mean']:.2f} m "
                                f"lookahead, {st['aim_p95'] / max(st['cross_p95'], 1e-9):.1f}x "
                                f"more. Any work here belongs on the heading, not the "
                                f"position."))
        else:
            out.append(('good', f"Heading error moves the aim point only "
                                f"{st['aim_p95']:.3f} m at p95, so it is not a hidden "
                                f"contributor."))

    # 2. is the localizer contributing
    g = st['amcl_gain']
    if np.isfinite(g):
        if g < 1.2:
            out.append(('bad', f"The localizer is barely beating dead reckoning "
                               f"({st['pos_mean']:.3f} m vs {st['dr_mean']:.3f} m, "
                               f"x{g:.2f}). Either it is not updating at all or it is "
                               f"being ignored. Run tools/diagnose_localization.py while "
                               f"the car is moving: the usual cause is scan-vs-TF "
                               f"timestamp skew, which makes AMCL silently discard every "
                               f"scan while every individual topic still looks healthy."))
        else:
            out.append(('good', f"The localizer is earning its keep: {st['pos_mean']:.3f} m "
                                f"against {st['dr_mean']:.3f} m for dead reckoning alone "
                                f"on the same trajectory (x{g:.1f} better)."))

    # 3. the rotation invariant
    a = st['m2o_yaw_absmax']
    if np.isfinite(a):
        if a > 1.0:
            txt = (f"map->odom is rotating by up to {a:.2f} deg (std "
                   f"{st['m2o_yaw_std']:.2f}). It must be zero: heading comes from the "
                   f"IMU's absolute quaternion, so odom is already yaw-locked to the "
                   f"world, and the only correct correction is a translation.")
            rd = st['r_m2odot_yawrate']
            rv = st['r_m2o_yawrate']
            if np.isfinite(rd) and abs(rd) > 0.3:
                txt += (f" Its RATE OF CHANGE tracks the car's turn rate "
                        f"(r = {rd:+.2f}), so the error is being integrated on every "
                        f"update -- and on a loop driven one way round those errors "
                        f"share a sign instead of cancelling. That points at "
                        f"dead_reckoning post-dating its TF by 20 ms while carrying "
                        f"an IMU heading up to one 55 ms tick stale. It is a timing "
                        f"bug; no AMCL parameter will fix it.")
            elif np.isfinite(rv) and abs(rv) > 0.3:
                txt += (f" It is proportional to the turn rate itself "
                        f"(r = {rv:+.2f}) rather than accumulating -- the estimate is "
                        f"rotated while cornering and recovers on the straights. That "
                        f"is a stale heading or an uncompensated lever arm, not the "
                        f"filter.")
            else:
                txt += (f" It tracks neither the turn rate (r = {rv:+.2f}) nor its "
                        f"integral (r = {rd:+.2f}), so this is the scan matcher "
                        f"itself, not the odometry timing.")
            out.append(('bad', txt))
        elif a > 0.4:
            out.append(('warn', f"map->odom rotates by up to {a:.2f} deg. Small, but it "
                                f"should be zero -- see the invariant section."))
        else:
            out.append(('good', f"map->odom stays a pure translation (max "
                                f"{a:.2f} deg). The heading path is healthy."))

    # 4. whose fault
    fr = st['fit_ratio_med']
    gap = st.get('fit_gap_med', float('nan'))
    if np.isfinite(fr) and np.isfinite(gap) and abs(gap) < MAP_RES_M / 2:
        out.append(('info', f"fit_ratio is {fr:.2f}, but the scan sits {st['fit_true_med'] * 100:.1f} cm "
                            f"from the walls at the true pose and {st['fit_est_med'] * 100:.1f} cm at the "
                            f"estimated one -- a difference of {gap * 100:.1f} cm, well inside the map's "
                            f"own {MAP_RES_M * 100:.0f} cm cell. A ratio between two sub-cell numbers "
                            f"looks alarming and resolves nothing: the map cannot tell these two poses "
                            f"apart, so this test is BLIND here rather than accusing. Do not tune AMCL "
                            f"on this number."))
    elif np.isfinite(fr):
        if fr > 1.05:
            out.append(('warn', f"The scan fits the map {(fr - 1) * 100:.0f} % worse at the "
                                f"estimated pose than at the true one (median fit_ratio "
                                f"{fr:.3f}). A better pose was available and the filter did "
                                f"not find it, so this IS tunable: more beams, more "
                                f"particles, a smaller update_min_d."))
        else:
            out.append(('info', f"The scan fits about as well at the estimated pose as at "
                                f"the true one (median fit_ratio {fr:.3f}). The map cannot "
                                f"distinguish the two, so filter tuning will not close this "
                                f"gap -- it is map quality or locally ambiguous geometry."))

    # 5. the floor
    if st['pos_mean'] < floor * 1.5:
        out.append(('info', f"Mean error {st['pos_mean']:.3f} m is at the "
                            f"{floor:.2f} m map-to-world alignment floor. Most of what is "
                            f"left is the frame alignment, not the localizer, and chasing "
                            f"it further means remapping rather than tuning."))
    return out


def _report(st, log, line, win, i0, tw, vw, ed, ec, ea, eyaw, dr, m2o, yrate,
            fitr, s_true, kap, consumed, margin, lap_i, lap_times, path, args):
    out_html = os.path.splitext(path)[0] + '.html'
    out_json = os.path.splitext(path)[0] + '.json'
    verdicts = _verdicts(st)

    # ---- terminal ----------------------------------------------------------
    W = 78
    print(f"\n{'=' * W}\n{st['name']}   line: {st['line']}   "
          f"{st['samples']} samples\n{'=' * W}")
    print(f"run       {st['duration_s']:.1f} s   {st['distance_m']:.1f} m   "
          f"sim tick {st['tick_hz']:.1f} Hz   laps {st['laps']}"
          + (f"   best {st['lap_best_s']:.2f} s" if st['laps'] else ''))
    if np.isfinite(st['converge_t_s']):
        print(f"converged {st['converge_t_s']:.1f} s / {st['converge_dist_m']:.1f} m "
              f"(below {args.converge_m} m and held {args.hold_s} s); "
              f"everything before that is excluded")
    else:
        print(f"converged NEVER below {args.converge_m} m -- scoring the whole run")
    print(f"\nposition  mean {st['pos_mean']:.3f}  p50 {st['pos_p50']:.3f}  "
          f"p90 {st['pos_p90']:.3f}  p95 {st['pos_p95']:.3f}  "
          f"max {st['pos_max']:.3f}  m   (floor {FLOOR_M:.2f})")
    print(f"  cross   |mean| {st['cross_absmean']:.3f}  p95 {st['cross_p95']:.3f}  "
          f"bias {st['cross_bias']:+.3f} m   <- the one that hits walls")
    print(f"  along   |mean| {st['along_absmean']:.3f}  bias {st['along_bias']:+.3f} m")
    print(f"heading   |mean| {st['yaw_absmean']:.2f}  p95 {st['yaw_p95']:.2f}  "
          f"max {st['yaw_max']:.2f}  bias {st['yaw_bias']:+.2f}  deg")
    print(f"  -> aim  heading x lookahead ({st['ld_mean']:.2f} m): |mean| "
          f"{st['aim_absmean']:.3f}  p95 {st['aim_p95']:.3f} m   "
          f"cross+aim p95 {st['lat_total_p95']:.3f} m")
    print(f"\nwall margin  tightest {st['margin_tightest_m']:.3f} m   "
          f">half eaten {st['margin_half_pct']:.1f} %   "
          f"exceeded {st['margin_over_pct']:.1f} % of the lap")
    print(f"dead reckoning alone  mean {st['dr_mean']:.3f}  max {st['dr_max']:.3f} m"
          + (f"   -> localizer is x{st['amcl_gain']:.1f} better"
             if np.isfinite(st['amcl_gain']) else ''))
    print(f"m2o yaw (must be 0)   mean {st['m2o_yaw_mean']:+.3f}  "
          f"std {st['m2o_yaw_std']:.3f}  |max| {st['m2o_yaw_absmax']:.3f} deg")
    print(f"fit_ratio             median {st['fit_ratio_med']:.3f}  "
          f"p90 {st['fit_ratio_p90']:.3f}   (>1 = a better pose existed)")
    print(f"  scan-to-wall          true {st['fit_true_med'] * 100:.1f} cm  "
          f"est {st['fit_est_med'] * 100:.1f} cm  gap {st['fit_gap_med'] * 100:+.1f} cm "
          f"(map cell {MAP_RES_M * 100:.0f} cm -- a gap under half a cell means the "
          f"test cannot resolve)")
    print(f"correlation of error with   speed r={st['r_speed']:+.2f}   "
          f"|kappa| r={st['r_kappa']:+.2f}   |yaw_rate| r={st['r_yawrate']:+.2f}")
    print('\nread:')
    for lvl, txt in verdicts:
        tag = {'good': 'OK     ', 'warn': 'CHECK  ',
               'bad': 'PROBLEM', 'info': 'NOTE   '}[lvl]
        print(f'  {tag}  ' + _wrap(txt, W - 12))

    # ---- json --------------------------------------------------------------
    with open(out_json, 'w') as f:
        json.dump({k: (None if isinstance(v, float) and not np.isfinite(v) else v)
                   for k, v in st.items() if not k.startswith('_')},
                  f, indent=2, sort_keys=True)

    # ---- html --------------------------------------------------------------
    rep = R.Report(
        f'Localization report — {st["name"]}',
        f'{st["line"]} · {st["duration_s"]:.0f} s · {st["distance_m"]:.0f} m · '
        f'{st["laps"]} laps · sim tick {st["tick_hz"]:.1f} Hz')

    rep.section('Headline', 'headline')
    rep.p('Truth is the simulator\'s <code>/ips</code> position and '
          '<code>/imu</code> heading. The estimate is TF <code>map→odom</code> '
          'composed with <code>odom→roboracer_1</code> — AMCL\'s correction '
          'applied on top of dead reckoning, which is exactly the pose '
          '<code>pure_pursuit</code> reads. Samples before the filter converged, '
          'and samples with the car stationary, are excluded.', 'lead')
    rep.tiles([
        ('mean error', f'{st["pos_mean"]:.3f}<span class="tn"> m</span>',
         f'floor is {FLOOR_M:.2f} m'),
        ('p95 error', f'{st["pos_p95"]:.3f}<span class="tn"> m</span>',
         f'max {st["pos_max"]:.3f} m'),
        ('cross-track |mean|', f'{st["cross_absmean"]:.3f}<span class="tn"> m</span>',
         f'bias {st["cross_bias"]:+.3f} m'),
        ('heading |mean|', f'{st["yaw_absmean"]:.2f}<span class="tn"> °</span>',
         f'max {st["yaw_max"]:.2f}°'),
        ('aim error p95', f'{st["aim_p95"]:.3f}<span class="tn"> m</span>',
         f'heading × {st["ld_mean"]:.2f} m lookahead'),
        ('vs dead reckoning',
         (f'×{st["amcl_gain"]:.1f}' if np.isfinite(st['amcl_gain']) else '—'),
         f'DR alone {st["dr_mean"]:.3f} m'),
        ('converged',
         (f'{st["converge_t_s"]:.1f}<span class="tn"> s</span>'
          if np.isfinite(st['converge_t_s']) else 'never'),
         (f'after {st["converge_dist_m"]:.1f} m'
          if np.isfinite(st['converge_dist_m']) else
          f'never held below {args.converge_m} m')),
    ])

    rep.section('Verdict', 'verdict')
    for lvl, txt in verdicts:
        rep.verdict(lvl, R.esc(txt))

    # --- 1. does it matter --------------------------------------------------
    rep.section('1 · Does the error matter?', 'matters')
    rep.p('Cross-track error is the component that puts the car into a wall: if '
          'the controller believes it is 15 cm to the left of where it really is, '
          'it steers 15 cm right. Each sample is scored against the margin '
          f'actually available at that point of the lap — the raceline\'s own '
          f'<code>w_left</code>/<code>w_right</code> minus the {CAR_HALF_W:.4f} m '
          'car half-width — on the side the error pushes toward.')
    rep.tiles([
        ('tightest margin', f'{st["margin_tightest_m"]:.3f}<span class="tn"> m</span>',
         'on this line'),
        ('>half the margin', f'{st["margin_half_pct"]:.1f}<span class="tn"> %</span>',
         'of the lap'),
        ('margin exceeded', f'{st["margin_over_pct"]:.1f}<span class="tn"> %</span>',
         'of the lap'),
    ])
    rep.figure(
        R.cdf_chart(ed, 'position error [m]',
                    rules=[('tightest margin', st['margin_tightest_m']),
                           ('alignment floor', FLOOR_M)]),
        'Share of the run below a given error.',
        'Read a percentile straight off the curve; the marked dots are p50, '
        'p90, p95 and p99.')

    # --- 2. where ------------------------------------------------------------
    rep.section('2 · Where on the track', 'where')
    nb = args.bins
    edges = np.linspace(0.0, line['L'], nb + 1)
    m_mean, m_p90, m_cnt = bin_stats(s_true, ed, edges)
    c_mean, _, _ = bin_stats(s_true, ec, edges)
    labels = [f'{e:.1f} m' for e in edges[:-1]]
    notes = [f'{c} samples, p90 {R._fmt(p)}' for c, p in zip(m_cnt, m_p90)]

    rep.figure(
        R.track_map(line['x'], line['y'],
                    _line_err_by_point(line, s_true, ed, edges, m_mean),
                    log['true_x'][win], log['true_y'][win],
                    log['est_x'][win], log['est_y'][win]),
        'The racing line, shaded by mean error where the car passed.',
        'Darker is worse. The grey line is where the car actually was; the '
        'dashed orange line is where it thought it was. Where the two separate '
        'visibly, the localizer was wrong by more than the track is wide at '
        'this zoom.')
    rep.figure(
        R.bar_chart(labels, list(m_mean), xlabel='distance around the lap [m]',
                    ylabel='mean error [m]', notes=notes,
                    rules=[('tightest margin', st['margin_tightest_m'])]),
        'Mean position error by position around the lap.',
        'Both height and shade carry the magnitude, so the worst bins are '
        'findable without reading every label.')
    rep.figure(
        R.diverging_bar(labels, list(c_mean),
                        xlabel='distance around the lap [m]',
                        ylabel='mean cross-track error [m]'),
        'Signed cross-track error by position around the lap.',
        'Blue means the estimate sits left of the car, red means right. A bin '
        'that is consistently one colour is a systematic offset — a map error '
        'at that corner. Bins that alternate are filter noise.')

    worst = np.argsort(np.where(np.isfinite(m_mean), -m_mean, -np.inf))[:8]
    rep.table(
        ['from [m]', 'to [m]', 'mean [m]', 'p90 [m]', 'cross [m]', 'samples'],
        [[f'{edges[i]:.1f}', f'{edges[i + 1]:.1f}', R._fmt(m_mean[i]),
          R._fmt(m_p90[i]), R._fmt(c_mean[i]), int(m_cnt[i])]
         for i in worst if np.isfinite(m_mean[i])])

    # --- 3. over time --------------------------------------------------------
    rep.section('3 · Over time', 'time')
    rep.figure(
        R.line_chart(list(tw),
                     [('localizer', list(ed), 'series1'),
                      ('|cross-track|', list(np.abs(ec)), 'series2'),
                      ('dead reckoning alone', list(dr), 'series3')],
                     xlabel='time [s]', ylabel='error [m]',
                     rules=[('tightest margin', st['margin_tightest_m'])]),
        'Error against time.',
        'Dead reckoning is the same drive with the correction removed — a '
        'paired counterfactual, not a second run. The gap between blue and '
        'green is what the localizer is buying you; if they overlap, it is '
        'buying nothing.')
    if st['laps']:
        rep.table(['lap', 'time [s]', 'mean [m]', 'p90 [m]', '|cross| [m]',
                   '|yaw| [deg]'],
                  _lap_rows(lap_i, lap_times, ed, ec, eyaw))

    # --- 4. the invariant ----------------------------------------------------
    rep.section('4 · The rotation invariant', 'invariant')
    rep.p('<code>dead_reckoning</code> takes heading from the IMU\'s absolute '
          'quaternion rather than by integrating a turn rate, so the '
          '<code>odom</code> frame is yaw-locked to the world and cannot drift '
          'in heading. The map was built against that same ground truth. So the '
          'only correction a correct localizer can ever apply is a '
          '<b>pure translation</b>, and this line must sit at zero. Every degree '
          'in it is the scan matcher rotating away from a heading that was '
          'already exact.')
    rep.figure(
        R.line_chart(list(tw), [('map→odom yaw', list(m2o), 'series1')],
                     xlabel='time [s]', ylabel='map→odom yaw [deg]',
                     rules=[('correct value', 0.0)]),
        'The correction\'s rotation over the run. It should be a flat line at zero.',
        f'|max| {st["m2o_yaw_absmax"]:.3f}°, std {st["m2o_yaw_std"]:.3f}°. '
        f'Correlation against the car\'s turn rate: the value itself '
        f'r = {st["r_m2o_yawrate"]:+.2f} (a stale heading or a lever arm), its rate '
        f'of change r = {st["r_m2odot_yawrate"]:+.2f} (an error integrated on every '
        f'update). Either one points at the odometry timing rather than at an '
        f'AMCL parameter.')

    # --- 5. what drives the error -------------------------------------------
    rep.section('5 · What the error tracks', 'drivers')
    rep.p(f'Pearson correlation of position error against each candidate: '
          f'speed <b>r = {st["r_speed"]:+.2f}</b>, cornering sharpness '
          f'<b>r = {st["r_kappa"]:+.2f}</b>, turn rate '
          f'<b>r = {st["r_yawrate"]:+.2f}</b>. A correlation near zero means '
          'that variable is not the mechanism, whatever the bins look like.')
    for arr, lab, unit in ((vw, 'speed', 'm/s'),
                           (np.abs(kap), 'cornering sharpness |κ|', '1/m'),
                           (np.abs(yrate), 'turn rate |ψ̇|', 'rad/s')):
        fin = arr[np.isfinite(arr)]
        if fin.size < 30:
            continue
        eg = np.linspace(float(fin.min()), float(fin.max()), 13)
        bm, bp, bc = bin_stats(arr, ed, eg)
        rep.figure(
            R.bar_chart([f'{e:.2g}' for e in eg[:-1]], list(bm),
                        xlabel=f'{lab} [{unit}]', ylabel='mean error [m]',
                        notes=[f'{c} samples, p90 {R._fmt(p)}'
                               for c, p in zip(bc, bp)]),
            f'Position error by {lab}.',
            'Empty bins are omitted rather than drawn as zero.')

    # --- 6. whose fault ------------------------------------------------------
    rep.section('6 · Whose fault is it', 'fault')
    if np.isfinite(fitr).sum() > 30:
        rep.p('Place the live LiDAR scan at the true pose and measure how far its '
              'beam endpoints land from the nearest wall in the map '
              '(<code>fit_true</code>). Do it again at the estimated pose '
              '(<code>fit_est</code>). The ratio separates the two causes that '
              'look identical from the error alone: <b>above 1</b>, a better pose '
              'existed and the filter did not find it, which is tunable. '
              '<b>At 1</b>, the scan fits the wrong pose just as well as the '
              'right one — the map cannot tell them apart and no amount of '
              'filter tuning will help.')
        f_mean, _, f_cnt = bin_stats(s_true, fitr, edges)
        rep.figure(R.hist_chart(fitr, bins=40, xlabel='fit_ratio'),
                   'Distribution of fit_ratio.',
                   f'Median {st["fit_ratio_med"]:.3f}, p90 '
                   f'{st["fit_ratio_p90"]:.3f}. Mass sitting right of 1.0 is '
                   f'the tunable part.')
        rep.figure(
            R.bar_chart(labels, list(f_mean),
                        xlabel='distance around the lap [m]',
                        ylabel='mean fit_ratio',
                        notes=[f'{c} samples' for c in f_cnt]),
            'fit_ratio by position around the lap.',
            'Cross-reference against chart 2: a bin with high error AND '
            'fit_ratio near 1 is a map problem at that corner, not a filter '
            'problem.')
    else:
        rep.verdict('info',
                    'No fit_ratio data in this run — log_localization needs '
                    'scipy to build the distance field. <code>scripts/bench.sh '
                    'up</code> installs it; without it the two causes in §6 '
                    'cannot be told apart.')

    rep.section('Method and caveats', 'method')
    rep.raw(
        '<ul class="muted" style="max-width:74ch;line-height:1.7">'
        f'<li>The estimate lives in <code>map</code> and truth in '
        f'<code>world</code>; comparing them assumes the frames coincide, which '
        f'holds to about {FLOOR_M:.2f} m. Results at that magnitude are the '
        f'floor, not a defect.</li>'
        f'<li>Sampled at the logger\'s rate, then de-duplicated: the bridge '
        f'publishes at about {st["tick_hz"]:.0f} Hz and the logger holds the '
        f'last truth between frames, so held rows are dropped rather than '
        f'counted twice.</li>'
        f'<li>Only samples above {args.min_speed} m/s and after convergence '
        f'({args.converge_m} m held for {args.hold_s} s) are scored. A stationary '
        f'car and an unconverged filter both flatter the average.</li>'
        f'<li>Recorded with <code>mode:=dev</code>, which adds the instrument '
        f'nodes but leaves the control path identical to <code>mode:=race</code> '
        f'— the car steers on the estimate either way.</li>'
        f'<li><b>This will not match the number <code>localization_error</code> '
        f'prints</b>, and the difference is not a bug in either. That node '
        f'samples the <i>latest</i> <code>map→base</code> transform against the '
        f'<i>latest</i> ground truth, on its own timer, with no time alignment; '
        f'the two are typically one bridge frame apart, and at speed a 40 ms '
        f'offset is 0.3 m of pure bookkeeping. <code>log_localization</code> '
        f'instead looks the odometry leg up <i>at the truth\'s own timestamp</i> '
        f'so both poses describe the same instant — see its <code>_estimate()</code> '
        f'docstring, which records exactly this correction. Treat the aligned '
        f'number here as the honest one and the node\'s as a live upper bound.</li>'
        '</ul>')

    rep.write(out_html)
    print(f'\nwrote {out_html}\n      {out_json}')


def _line_err_by_point(line, s_true, ed, edges, binned):
    """Spread the per-bin mean error back over the line's own points, so the
    track map can be drawn at full geometric resolution."""
    idx = np.clip(np.digitize(line['s'], edges) - 1, 0, len(binned) - 1)
    return list(binned[idx])


def _lap_rows(lap_i, lap_times, ed, ec, eyaw):
    rows = []
    for n in range(len(lap_times)):
        a, b = lap_i[n], lap_i[n + 1]
        rows.append([n + 1, f'{lap_times[n]:.2f}', R._fmt(nanmean(ed[a:b])),
                     R._fmt(pct(ed[a:b], 90)),
                     R._fmt(nanmean(np.abs(ec[a:b]))),
                     R._fmt(nanmean(np.abs(eyaw[a:b])), 2)])
    return rows


def _wrap(text, width):
    words, lines, cur = text.split(), [], ''
    for w in words:
        if len(cur) + len(w) + 1 > width:
            lines.append(cur)
            cur = w
        else:
            cur = f'{cur} {w}'.strip()
    lines.append(cur)
    return ('\n' + ' ' * 11).join(lines)


# --------------------------------------------------------------------------
# Comparison across runs
# --------------------------------------------------------------------------

RANK_COLS = [
    ('name', 'variant', None),
    ('pos_mean', 'mean [m]', 3),
    ('cross_p95', 'p95 cross [m]', 3),
    ('pos_p95', 'p95 [m]', 3),
    ('cross_absmean', '|cross| [m]', 3),
    ('yaw_absmean', '|yaw| [deg]', 2),
    ('aim_p95', 'aim p95 [m]', 3),
    ('margin_half_pct', '>half margin [%]', 1),
    ('m2o_yaw_absmax', 'm2o yaw [deg]', 2),
    ('fit_ratio_med', 'fit_ratio', 3),
    ('amcl_gain', 'vs DR', 1),
    ('converge_t_s', 'converged [s]', 1),
    ('lap_best_s', 'best lap [s]', 2),
]


def compare(stats, out_html):
    stats = [s for s in stats if s]
    if len(stats) < 2:
        print('compare: need at least two scored runs')
        return
    # Rank on p95 cross-track error: the tail is what hits walls, and the
    # cross component is the part of it that does.
    stats.sort(key=lambda s: (s['cross_p95'] if np.isfinite(s['cross_p95'])
                              else np.inf))
    base = stats[0]
    rep = R.Report('Localization A/B',
                   f'{len(stats)} runs · ranked by p95 cross-track error · '
                   f'best: {base["name"]}')
    rep.section('Ranking', 'ranking')
    rep.p('Ranked on <b>p95 cross-track error</b> rather than on the mean: the '
          'mean is dominated by the long easy straights, and it is the tail that '
          'puts the car into a wall. <code>vs DR</code> is how many times better '
          'than dead reckoning alone on that same run.')
    rep.table([c[1] for c in RANK_COLS],
              [[s.get(k) if nd is None else R._fmt(s.get(k), nd)
                for k, _, nd in RANK_COLS] for s in stats])

    rep.section('Distribution', 'dist')
    rep.p('Each variant\'s full error distribution, not just its summary. A '
          'variant that wins on the mean but crosses to the right of the others '
          'in the tail is the wrong choice: the tail is what hits walls.')
    if any('_err' in s for s in stats):
        rep.figure(
            R.multi_cdf([(s['name'], s.get('_err', [])) for s in stats],
                        'position error [m]', best=0,
                        rules=[('tightest margin', base['margin_tightest_m']),
                               ('alignment floor', FLOOR_M)]),
            'Error distribution, every variant on one axis.',
            'The winner is drawn in colour and the rest in grey — nine hues '
            'would be unreadable, and the question here is whether the winner '
            'leads across the whole distribution or only at the mean. Hover any '
            'grey curve to name it.')
    rows = [(s['name'], s) for s in stats]
    rep.table(['variant', 'mean [m]', 'p50', 'p90', 'p95', 'max',
               'samples', 'laps'],
              [[n, R._fmt(s['pos_mean']), R._fmt(s['pos_p50']),
                R._fmt(s['pos_p90']), R._fmt(s['pos_p95']),
                R._fmt(s['pos_max']), s['samples'], s['laps']]
               for n, s in rows])

    rep.section('Read', 'read')
    b = stats[0]
    w = stats[-1]
    spread = ((w['cross_p95'] - b['cross_p95']) / max(b['cross_p95'], 1e-9)) * 100
    if spread < 15:
        rep.verdict('info', R.esc(
            f'Best and worst differ by only {spread:.0f} % in p95 cross-track '
            f'error ({b["name"]} {b["cross_p95"]:.3f} m vs {w["name"]} '
            f'{w["cross_p95"]:.3f} m). None of these parameters is the '
            f'constraint — the error is coming from somewhere else, most '
            f'likely the map or the odometry timing.'))
    else:
        rep.verdict('good', R.esc(
            f'{b["name"]} is {spread:.0f} % better than {w["name"]} on p95 '
            f'cross-track error. That is a real spread; the parameter it '
            f'changes is worth adopting.'))
    if all(s['pos_mean'] < FLOOR_M * 1.5 for s in stats):
        rep.verdict('info', R.esc(
            f'Every variant sits at the {FLOOR_M:.2f} m alignment floor. There '
            f'is nothing left to win by tuning the filter.'))
    rep.write(out_html)
    print(f'\nwrote {out_html}')


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='+', help='log_localization CSV files')
    ap.add_argument('--path', help='raceline CSV the run followed; '
                                   'auto-detected when omitted')
    ap.add_argument('--min-speed', type=float, default=0.8,
                    help='ignore samples slower than this [m/s]')
    ap.add_argument('--converge-m', type=float, default=0.10,
                    help='error the filter must get below to count as converged')
    ap.add_argument('--hold-s', type=float, default=3.0,
                    help='and stay below it for this long')
    ap.add_argument('--from-s', type=float, default=None,
                    help='override: score from this timestamp onward')
    ap.add_argument('--bins', type=int, default=40,
                    help='bins around the lap for the spatial breakdown')
    ap.add_argument('--compare', metavar='OUT.html',
                    help='also write a ranked comparison of every log given')
    a = ap.parse_args()

    stats = [analyze(p, a) for p in a.logs]
    if a.compare:
        compare(stats, a.compare)


if __name__ == '__main__':
    main()
