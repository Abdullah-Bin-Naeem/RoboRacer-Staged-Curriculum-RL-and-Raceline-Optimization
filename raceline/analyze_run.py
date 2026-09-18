#!/usr/bin/env python3
"""Read a log_localization CSV and say how the run went: lap times, tracking,
weave, localization, encoder slip, and (when the follower's status columns are
present) how the speed estimator and the slip controller behaved.

    python3 raceline/analyze_run.py run.csv
    python3 raceline/analyze_run.py run.csv --path raceline/porto/raceline_a6.5.csv
    python3 raceline/analyze_run.py run_a.csv run_b.csv          # several, one after another

Without --path the line is auto-detected: every CSV in raceline/<track>/ with a
geometry is tried and the one the TRUE trajectory sits closest to wins.

What the numbers mean
---------------------
tracking (truth)      lateral offset of the true car from the line. Total error,
                      controller plus localizer.
tracking (estimate)   the same for the pose the controller steered on. This is
                      the controller's own error; the difference to truth is the
                      localizer's.
corner bias           signed offset toward the inside of the turn, averaged over
                      |kappa| > 0.4. Positive = cutting inside, which is what a
                      too-long lookahead does. Negative = running wide, which is
                      the tire, not the controller.
weave                 on straights, how often the lateral error changes sign per
                      second. Pure pursuit oscillating at its natural frequency
                      shows 1.5-2 /s here; 0.2-0.5 /s is drift, not weave.
v_est error           fused speed estimate minus the true speed (the log's
                      `speed` column is ground truth). This validates the
                      estimator that replaced the encoder.
encoder ratio         wheel speed over true speed, by phase. 1.00 is honest;
                      the sim makes it the throttle echo (VEHICLE_MODEL.md §3.1).
slip_cmd              (u_cmd - v_est)/v_est, what the slip controller asked for.
                      It should never leave [-slip_brake, +slip_accel].

Only standard library + numpy; runs with the system python3.
"""
import argparse
import csv
import glob
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def load_log(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        sys.exit(f"{path}: empty")
    cols = {}
    for k in rows[0]:
        try:
            cols[k] = np.array([float(r[k]) if r[k] not in ('', 'nan') else np.nan for r in rows])
        except ValueError:
            pass                                             # non-numeric column
    # drop sample-and-hold repeats: the logger runs at 20 Hz on an 18 Hz source
    keep = np.r_[True, (np.diff(cols['true_x']) != 0) | (np.diff(cols['true_y']) != 0)]
    return {k: v[keep] for k, v in cols.items()}


def load_line(path):
    try:
        D = np.loadtxt(path, delimiter=',')
    except ValueError:
        return None          # not a line: a run log (header row) or anything else
    if D.ndim != 2 or D.shape[1] < 7:
        return None
    return dict(name=os.path.basename(path), s=D[:, 0], x=D[:, 1], y=D[:, 2], psi=D[:, 3], kappa=D[:, 4],
                v=D[:, 7] if D.shape[1] > 7 else None, L=float(D[-1, 0] + (D[1, 0] - D[0, 0])))


def project(x, y, line):
    """Nearest path point: signed lateral offset (+ left of path), kappa and s there."""
    j = ((x[:, None] - line['x'][None, :]) ** 2 + (y[:, None] - line['y'][None, :]) ** 2).argmin(1)
    e = (x - line['x'][j]) * (-np.sin(line['psi'][j])) + (y - line['y'][j]) * np.cos(line['psi'][j])
    return e, line['kappa'][j], line['s'][j]


def pick_line(log, mov, explicit):
    if explicit:
        return load_line(explicit)
    best, best_med, runner_up = None, np.inf, None
    # Lines live in raceline/<track>/; the flat glob is kept for anything
    # left at the top level.
    for f in sorted(glob.glob(os.path.join(HERE, '*.csv'))
                    + glob.glob(os.path.join(HERE, '*', '*.csv'))):
        line = load_line(f)
        if line is None:
            continue
        e, _, _ = project(log['true_x'][mov], log['true_y'][mov], line)
        med = np.median(np.abs(e))
        if med < best_med:
            if best is not None:
                runner_up = (best['name'], best_med)
            best, best_med = line, med
        elif runner_up is None or med < runner_up[1]:
            runner_up = (line['name'], med)
    if best is None or best_med > 0.5:
        sys.exit("no line in raceline/ matches this run (closest median offset "
                 f"{best_med:.2f} m); pass --path")
    if runner_up is not None and runner_up[1] < 1.5 * best_med:
        print(f"NOTE: line auto-detection is ambiguous ({best['name']} {best_med:.3f} m vs "
              f"{runner_up[0]} {runner_up[1]:.3f} m); pass --path to be sure")
    return best


def sign_changes_per_s(x, t):
    if len(x) < 10:
        return float('nan')
    span = t.max() - t.min()
    return float(np.sum(np.diff(np.sign(x)) != 0) / span) if span > 0 else float('nan')


def laps_from_s(s, t, L):
    """Lap times from the projected s coordinate wrapping around."""
    wraps = np.where((s[:-1] > 0.8 * L) & (s[1:] < 0.2 * L))[0]
    tw = t[wraps + 1]
    return np.diff(tw)


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float('nan')


def analyze(path, explicit_line, min_speed):
    log = load_log(path)
    t, v = log['t'], log['speed']
    mov = v > min_speed
    if mov.sum() < 30:
        print(f"{path}: fewer than 30 moving samples, nothing to say")
        return
    line = pick_line(log, mov, explicit_line)

    print(f"\n{'=' * 78}\n{path}   line: {line['name']}\n{'=' * 78}")
    e_t, kap, s_t = project(log['true_x'][mov], log['true_y'][mov], line)
    e_e, _, _ = project(log['est_x'][mov], log['est_y'][mov], line)
    tm, vm = t[mov], v[mov]

    # ---- run -------------------------------------------------------------
    laps = laps_from_s(s_t, tm, line['L'])
    # Effective simulator tick rate: the logger samples at 20 Hz and holds, so
    # count distinct ground-truth poses. This is the bridge round trip's clock
    # (VEHICLE_MODEL.md §3.6) and it is machine- and settings-dependent:
    # measured 17.3-17.5 Hz after a fresh boot, decaying to 12.8 Hz within a
    # long session (not the GPU driver mode: a restart with the same setting
    # gave 17.5 Hz), and the laps moved with it.
    uniq = len(t)                 # load_log already dropped repeated poses
    n_raw = log.get('_n_raw', uniq)
    tick_hz = uniq / max(t[-1] - t[0], 1e-6)
    print(f"duration {t[-1] - t[0]:.1f} s   distance {log['dist_m'][-1]:.1f} m   "
          f"moving {mov.mean() * 100:.0f} %   v mean {vm.mean():.2f}  max {vm.max():.2f} m/s   "
          f"sim tick {tick_hz:.1f} Hz")
    if len(laps):
        print(f"laps: {len(laps)} full   times " + '  '.join(f"{x:.2f}" for x in laps)
              + f"   best {laps.min():.2f} s" + (f"   (profile predicts {np.sum(np.diff(np.append(line['s'], line['L'])) / line['v']):.2f} s)" if line['v'] is not None else ''))
    else:
        print("laps: none completed")

    # ---- localization ----------------------------------------------------
    ed, ec = log['err_dist'][mov], log['err_cross'][mov]
    enc_ratio = log['enc_dist'][-1] / max(log['dist_m'][-1], 1e-6)
    # ---- resets and recovery --------------------------------------------
    yaw_r = np.deg2rad(log['true_yaw_deg'])
    step = np.abs(np.angle(np.exp(1j * np.diff(yaw_r))))
    jump = np.hypot(np.diff(log['true_x']), np.diff(log['true_y']))
    resets = [j for j in np.flatnonzero((step > np.radians(10)) & (jump > 0.3))]   # 10 deg: run 14's first reset stepped 12
    resets = [j for n, j in enumerate(resets) if n == 0 or j - resets[n - 1] > 5]
    if resets:
        rdy = log['ready'] if 'ready' in log else None
        rows = []
        for j in resets:
            rec = float('nan'); err_after = float('nan')
            if rdy is not None:
                lost = np.flatnonzero((rdy[j:] < 0.5))
                if len(lost):
                    back = np.flatnonzero(rdy[j + lost[0]:] > 0.5)
                    if len(back):
                        k = j + lost[0] + back[0]; rec = t[k] - t[j]
                        m = (t >= t[k] + 1.0) & (t <= t[k] + 3.0)
                        if m.any(): err_after = float(np.nanmean(ed[m[:len(ed)]] if len(m) == len(ed) else np.nan))
            rows.append((t[j] - t[0], rec, err_after))
        print(f"\nresets: {len(resets)}  at +" + ", ".join(f"{r[0]:.0f}s" for r in rows))
        cps = []
        for j in resets:
            k = min(j + 2, len(t) - 1)                       # a couple of samples after the jump: the reset pose
            cps.append((log['true_x'][k], log['true_y'][k], yaw_r[k]))
        uniq = []
        for c in cps:
            if all(np.hypot(c[0] - u[0], c[1] - u[1]) > 0.5 for u in uniq): uniq.append(c)
        print("reset poses (registry form, from ground truth): " + ", ".join(f"('{c[0]:.3f}', '{c[1]:.3f}', '{c[2]:.3f}')" for c in uniq))
        if rdy is not None:
            recs = [r[1] for r in rows if np.isfinite(r[1])]
            print(f"recovery: {len(recs)}/{len(rows)} re-confirmed, time to ready "
                  + (f"median {np.median(recs):.1f} s max {max(recs):.1f} s" if recs else "--")
                  + "; localization error 1-3 s after: " + ", ".join(f"{r[2]:.2f}" if np.isfinite(r[2]) else "--" for r in rows) + " m")
        else:
            print("recovery: no 'ready' column in this log (older logger); rerun to measure recovery times")

    print(f"\nlocalization: error mean {ed.mean():.3f}  p90 {pct(ed, 90):.3f}  max {ed.max():.3f} m   "
          f"cross mean {np.abs(ec).mean():.3f} m   m2o yaw std {np.nanstd(log['m2o_yaw_deg'][mov]):.2f} deg   "
          f"encoder/true distance {enc_ratio:.3f}")
    if 'err_along' in log:
        ea = log['err_along'][mov]
        print(f"localization along/cross (+ = ahead / left): along mean {ea.mean():+.3f}  |p90| {pct(np.abs(ea), 90):.3f}  "
              f"max {np.abs(ea).max():.3f} m   cross |p90| {pct(np.abs(ec), 90):.3f} m")

    # ---- localization_v2, when it ran (as the localizer or as v2_shadow) ----
    # Same truth, same decomposition, one file: err_* is the localizer that
    # owned map->odom, v2_* is localization_v2. The per-mode split is the
    # design's own claim under test: nothing along-track on STRAIGHT_BLIND.
    if 'v2_err_along' in log and np.isfinite(log['v2_err_along'][mov]).any():
        V2_MODES = ('STRAIGHT_BLIND', 'APPROACH', 'CORNER', 'TRANSIT')
        V2_REJ = ('ok', 'matcher', 'clearance', 'no_odom', 'not_seeded', 'guard')
        va, vc, vd = (log['v2_err_along'][mov], log['v2_err_cross'][mov], log['v2_err_dist'][mov])
        fin = np.isfinite(va)
        va, vc, vd = va[fin], vc[fin], vd[fin]
        print(f"\nlocalization_v2:   error mean {vd.mean():.3f}  p90 {pct(vd, 90):.3f}  max {vd.max():.3f} m   "
              f"along mean {va.mean():+.3f} |p90| {pct(np.abs(va), 90):.3f} max {np.abs(va).max():.3f}   "
              f"cross |p90| {pct(np.abs(vc), 90):.3f} m")
        if 'v2_mode' in log:
            md = log['v2_mode'][mov][fin]
            ea_all = log['err_along'][mov][fin] if 'err_along' in log else None
            for i, name in enumerate(V2_MODES):
                k = md == i
                if k.sum() < 5:
                    continue
                line_ = (f"  {name:14s} n {k.sum():5d}   v2 along |p90| {pct(np.abs(va[k]), 90):.3f} mean {va[k].mean():+.3f}"
                         f"   cross |p90| {pct(np.abs(vc[k]), 90):.3f}")
                if ea_all is not None:
                    line_ += f"   | owner along |p90| {pct(np.abs(ea_all[k]), 90):.3f} mean {ea_all[k].mean():+.3f}"
                print(line_)
            # How far each correction WALKED along-track where nothing observes
            # along-track. The owner's is m2o; v2's is the applied step.
            blind = np.flatnonzero(mov)[fin][md == 0]
            if len(blind) > 5 and 'm2o_x' in log:
                yaw = np.deg2rad(log['true_yaw_deg'])
                dmx = np.diff(log['m2o_x']); dmy = np.diff(log['m2o_y'])
                j = blind[blind < len(dmx)]
                own = np.abs(dmx[j] * np.cos(yaw[j]) + dmy[j] * np.sin(yaw[j])).sum()
                v2w = np.abs(log['v2_dx_appl'][j] * np.cos(yaw[j]) + log['v2_dy_appl'][j] * np.sin(yaw[j])).sum()
                print(f"  along-track walk of the correction on STRAIGHT_BLIND: owner {own:.2f} m   v2 {v2w:.2f} m")
        if 'v2_reject' in log:
            rj = log['v2_reject'][mov][fin].astype(int)
            counts = {V2_REJ[i]: int((rj == i).sum()) for i in range(len(V2_REJ)) if (rj == i).any()}
            print(f"  v2 scans: {counts}   compute ms p90 {pct(log['v2_compute_ms'][mov][fin], 90):.2f}   "
                  f"yaw_hint std {np.nanstd(log['v2_yaw_hint_deg'][mov][fin]):.2f} deg (the invariant; ~0)")

    # ---- tracking --------------------------------------------------------
    corner, straight = np.abs(kap) > 0.4, np.abs(kap) < 0.15
    print(f"\ntracking (truth):     |e| mean {np.abs(e_t).mean():.3f}  p90 {pct(np.abs(e_t), 90):.3f}  max {np.abs(e_t).max():.3f} m")
    print(f"tracking (estimate):  |e| mean {np.abs(e_e).mean():.3f}  p90 {pct(np.abs(e_e), 90):.3f}  max {np.abs(e_e).max():.3f} m")
    if corner.any():
        ins_t, ins_e = (e_t * np.sign(kap))[corner], (e_e * np.sign(kap))[corner]
        print(f"corner bias (+ inside): truth {ins_t.mean():+.3f}   estimate {ins_e.mean():+.3f}   "
              f"(n={corner.sum()})")
    print("straights by speed:   " + "   ".join(
        f"{lo:.0f}-{hi:.0f} m/s: |e| {np.abs(e_e[m]).mean():.3f} weave {sign_changes_per_s(e_e[m], tm[m]):.1f}/s (n={m.sum()})"
        for lo, hi in ((0, 2.5), (2.5, 4.5), (4.5, 9)) if (m := straight & (vm >= lo) & (vm < hi)).sum() > 10))

    # ---- controller status columns, if the follower published them ---------
    if 'pp_v_est' not in log:
        print("\n(no pp_* columns: run with the follower publishing ~/status and the logger recording it)")
        return
    ve, venc, vt, u, thr, st, ld, slip = (log[k][mov] for k in
                                          ('pp_v_est', 'pp_v_enc', 'pp_v_target', 'pp_u_cmd', 'pp_throttle',
                                           'pp_steering', 'pp_ld', 'pp_slip'))
    ok = np.isfinite(ve)
    if ok.sum() < 10:
        print("\npp_* columns present but empty (follower not running or not publishing)")
        return
    dv = ve[ok] - vm[ok]
    acc = np.gradient(vm, tm)
    accel, brake, cruise = acc > 1.0, acc < -1.0, np.abs(acc) <= 1.0
    ratio = venc / np.maximum(vm, 0.3)
    print(f"\nspeed estimate:  v_est - v_true mean {dv.mean():+.3f}  p90 |.| {pct(np.abs(dv), 90):.3f}  max {np.abs(dv).max():.3f} m/s")
    if 'pp_a_imu' in log:
        # How much speed the raw IMU x-axis integrates to, against the true change,
        # over stretches of hard acceleration and braking. 1.0 = the integral is honest.
        a_raw = log['pp_a_imu'][mov]; dt_log = np.gradient(tm)
        for name, m in (('accel', accel & ok & np.isfinite(a_raw)), ('brake', brake & ok & np.isfinite(a_raw))):
            if m.sum() > 20:
                gain = np.sum(a_raw[m] * dt_log[m]) / max(np.sum(acc[m] * dt_log[m]), 1e-6)
                print(f"IMU integral gain while {name}: {gain:.2f}  (1.0 = honest; with the pose blend the estimate absorbs the rest)")
    print(f"encoder/true speed: accel {np.nanmean(ratio[accel & ok]):.2f}   brake {np.nanmean(ratio[brake & ok]):.2f}   cruise {np.nanmean(ratio[cruise & ok]):.2f}")
    print(f"slip_cmd: p50 {pct(slip[ok], 50):+.3f}  p90 {pct(slip[ok], 90):+.3f}  min {np.nanmin(slip[ok]):+.3f}  max {np.nanmax(slip[ok]):+.3f}")
    # Command delay: the wheel follows 25.25 * throttle with a lag. Find the lag
    # that best explains the measured wheel speed, and the slip the wheel really saw.
    # 5 ms grid: a 50 ms grid read 150 ms on the 17.3 Hz runs where the trough
    # is really 165-180 ms, i.e. THREE sim frames (VEHICLE_MODEL.md 3.6), and
    # every controller constant was referenced to that coarse reading.
    wheel_cmd = 25.25 * thr
    lags = np.arange(0.0, 0.351, 0.005)
    rms = np.array([np.sqrt(np.nanmean((venc[ok] - np.interp(tm[ok] - lag, tm[ok], wheel_cmd[ok])) ** 2)) for lag in lags])
    best = lags[int(np.argmin(rms))]
    trough = lags[rms <= rms.min() * 1.02]      # lags within 2 % of the best fit
    seen = (venc - vm) / np.maximum(vm, 4.0)
    if 'pp_delay' in log and np.isfinite(log['pp_delay'][mov]).any():
        dl = log['pp_delay'][mov]; dl = dl[np.isfinite(dl)]
        print(f"follower's own delay estimate: {dl[-1] * 1000:.0f} ms at the end (range {dl.min() * 1000:.0f}-{dl.max() * 1000:.0f})")
    print(f"command delay: wheel speed matches 25.25*throttle best at {best * 1000:.0f} ms lag "
          f"(trough {trough[0] * 1000:.0f}-{trough[-1] * 1000:.0f} ms; rms {rms.min():.2f} vs {rms[0]:.2f} m/s at 0; "
          f"3 sim frames = {3000.0 / tick_hz:.0f} ms)   wheel slip actually seen: accel median "
          f"{np.nanmedian(seen[accel & ok]):+.3f}  brake median {np.nanmedian(seen[brake & ok]):+.3f}")
    print(f"target tracking: v_true - v_target mean {np.nanmean(vm[ok] - vt[ok]):+.3f}  p90 |.| {pct(np.abs(vm[ok] - vt[ok]), 90):.3f} m/s")
    print(f"throttle: mean {np.nanmean(thr[ok]):.3f}  at 0: {np.mean(thr[ok] <= 1e-3) * 100:.1f} %  saturated: {np.mean(thr[ok] >= 0.999) * 100:.1f} %   "
          f"steering |.| p90 {pct(np.abs(st[ok]), 90):.2f}  at lock: {np.mean(np.abs(st[ok]) >= 0.999) * 100:.1f} %   "
          f"lookahead {np.nanmin(ld[ok]):.2f}-{np.nanmax(ld[ok]):.2f} m")

    # ---- verdicts --------------------------------------------------------
    print("\nread:")
    fast = straight & (vm > 4.0)
    if fast.sum() > 10:
        w = sign_changes_per_s(e_e[fast], tm[fast])
        print(f"  {'WEAVE on fast straights' if w > 1.2 else 'no weave'} ({w:.1f} sign changes/s at v>4)"
              + ("  -> lengthen lookahead_max" if w > 1.2 else ""))
    if corner.any():
        b = ins_e.mean()
        print("  corners: " + ("cutting inside -> shorten lookahead in corners (lower lookahead_k or lookahead_min)" if b > 0.05
                               else "running wide -> lateral grip, lower a_lat rung" if b < -0.05 else "on the line"))
    print(f"  speed estimator {'OK' if pct(np.abs(dv), 90) < 0.3 else 'POOR'} (p90 error {pct(np.abs(dv), 90):.2f} m/s)")
    rolling = ok & (ve > 0.5)                 # the launch floor is outside the band by design
    if rolling.any():
        print(f"  slip command {'stayed in band' if np.nanmax(np.abs(slip[rolling])) <= 0.16 else 'LEFT the band'} "
              f"while rolling (max |slip| {np.nanmax(np.abs(slip[rolling])):.2f})")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--path', help='raceline CSV the run followed; auto-detected when omitted')
    ap.add_argument('--min-speed', type=float, default=0.8, help='ignore samples slower than this [m/s]')
    a = ap.parse_args()
    for p in a.logs:
        analyze(p, a.path, a.min_speed)


if __name__ == '__main__':
    main()
