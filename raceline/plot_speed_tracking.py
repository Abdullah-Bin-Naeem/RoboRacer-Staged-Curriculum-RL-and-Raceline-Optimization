#!/usr/bin/env python3
"""Where the lap time goes against the planned speed profile, as numbers and charts.

    python3 raceline/plot_speed_tracking.py icra_run23_hardened.csv \
        --path raceline/icra2026/raceline_a7.0zv_hard_l6.0_corners_h.csv

Isolates the CLEAN laps in a session (a long log usually ends in crash/reset
junk), resamples each onto a common s grid, and decomposes the gap to the
profile's predicted lap time into the two halves that have different owners:

    v_plan  -> v_target   the FOLLOWER's own limiting: the v_max clip and the
                          steer_a_lat_max curvature cap rewriting the plan.
                          Owned by whoever sets those limits.
    v_target -> v_true    TRACKING: the car not delivering the speed it was
                          asked for. Owned by the longitudinal controller.

Both are reported in seconds per lap, per track section, so it is clear which
one is worth working on and where.

Charts written to --out (default figures/):
    <run>_speed_profile.png    planned / commanded / achieved against s
    <run>_time_loss.png        cumulative seconds lost, split into the two halves
    <run>_slip_band.png        slip command against the band, and where it pins
    <run>_band_preview.png     the band width now vs at the moment the throttle
                               lands -- the corner-exit mismatch

Standard library + numpy + matplotlib.
"""
import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_run as ar                                     # noqa: E402

G = 9.81


def project_chunked(x, y, line, chunk=2000):
    """ar.project, in slices, so a 40k-sample log does not build a 40k x 540 array."""
    e, k, s = [], [], []
    for i in range(0, len(x), chunk):
        ei, ki, si = ar.project(x[i:i + chunk], y[i:i + chunk], line)
        e.append(ei), k.append(ki), s.append(si)
    return np.concatenate(e), np.concatenate(k), np.concatenate(s)


def lap_bounds(s, t, L):
    """(start, end, lap_time) for every complete lap, as indices into the arrays."""
    w = np.where((s[:-1] > 0.8 * L) & (s[1:] < 0.2 * L))[0] + 1
    return [(w[i], w[i + 1], float(t[w[i + 1]] - t[w[i]])) for i in range(len(w) - 1)]


def resample(s_lap, y_lap, grid):
    """y(s) on the common grid. s from projection is noisy, so sort and dedupe."""
    o = np.argsort(s_lap)
    ss, yy = s_lap[o], y_lap[o]
    keep = np.r_[True, np.diff(ss) > 1e-9]
    ss, yy = ss[keep], yy[keep]
    if len(ss) < 4:
        return np.full_like(grid, np.nan)
    return np.interp(grid, ss, yy, left=np.nan, right=np.nan)


def clean_laps(laps, tol, lo, hi):
    """Laps whose time sits in the tight cluster: the ones actually driven clean."""
    times = np.array([l[2] for l in laps])
    if lo is None or hi is None:
        med = np.median(times[(times > 1.0) & (times < 3.0 * np.median(times))])
        lo, hi = (med - tol, med + tol) if lo is None else (lo, hi)
    return [l for l in laps if lo <= l[2] <= hi], lo, hi


def sections(grid, kappa_grid, L):
    """Split the lap into corner / straight runs from the line's own curvature."""
    corner = np.abs(kappa_grid) > 0.4
    edges = np.r_[0, np.where(np.diff(corner.astype(int)) != 0)[0] + 1, len(grid)]
    out = []
    for a, b in zip(edges[:-1], edges[1:]):
        if grid[b - 1] - grid[a] < 1.5:            # ignore slivers
            continue
        out.append((grid[a], grid[b - 1], 'corner' if corner[a] else 'straight', a, b))
    return out


def analyze(path, line_path, out_dir, tol, lo, hi, ds, cmd_delay, slip_circle, cap, lead_min,
            slip_accel, slip_brake):
    log = ar.load_log(path)
    line = ar.load_line(line_path) if line_path else ar.pick_line(
        log, log['speed'] > 0.8, None)
    if line['v'] is None:
        sys.exit(f"{line['name']}: no v_mps column, nothing to compare against")
    L = line['L']

    e_t, kap_true, s_t = project_chunked(log['true_x'], log['true_y'], line)
    laps = lap_bounds(s_t, log['t'], L)
    if not laps:
        sys.exit(f"{path}: no complete laps")
    keep, lo, hi = clean_laps(laps, tol, lo, hi)
    if not keep:
        sys.exit(f"{path}: no laps inside [{lo:.2f}, {hi:.2f}] s of {len(laps)} complete")

    grid = np.arange(0.0, L, ds)
    v_plan = np.interp(grid, line['s'], line['v'])
    kap_line = np.interp(grid, line['s'], line['kappa'])

    have_pp = 'pp_v_target' in log and np.isfinite(log['pp_v_target']).any()
    stack = {k: [] for k in ('v_true', 'v_target', 'slip', 'thr', 'elat', 'band')}
    for a, b, _ in keep:
        sl = slice(a, b)
        stack['v_true'].append(resample(s_t[sl], log['speed'][sl], grid))
        stack['elat'].append(resample(s_t[sl], np.abs(e_t[sl]), grid))
        if have_pp:
            stack['v_target'].append(resample(s_t[sl], log['pp_v_target'][sl], grid))
            stack['slip'].append(resample(s_t[sl], log['pp_slip'][sl], grid))
            stack['thr'].append(resample(s_t[sl], log['pp_throttle'][sl], grid))
            # The band the controller really used here: it shrinks with lateral
            # load, so a fixed threshold reads "never pinned" in exactly the
            # corners where it is tightest. a_lat as _throttle_slip computes it,
            # from the speed the follower steered on and the path curvature.
            if slip_circle > 0.0:
                a_lat_s = log['pp_v_est'][sl] ** 2 * np.abs(log['pp_kappa'][sl])
                band_s = np.maximum(slip_circle * np.sqrt(
                    np.clip(1.0 - (a_lat_s / cap) ** 2, 0.0, None)), 0.02)
            else:
                # Circle off: the band is the FIXED pair, and which half applies
                # depends on the sign of the slip being commanded. Scoring a
                # fixed band against the circle formula with slip_circle = 0
                # gives max(0, 0.02) = 0.02 and reports ~90 % pinned everywhere,
                # which is an artefact, not a saturated controller.
                sgn = np.sign(log['pp_slip'][sl])
                band_s = np.where(sgn < 0, slip_brake, slip_accel)
            stack['band'].append(resample(s_t[sl], band_s, grid))
    M = {k: np.array(v) for k, v in stack.items() if v}

    v_true = np.nanmedian(M['v_true'], 0)
    v_lo, v_hi = np.nanpercentile(M['v_true'], 10, 0), np.nanpercentile(M['v_true'], 90, 0)
    v_targ = np.nanmedian(M['v_target'], 0) if have_pp else None

    # ---- the decomposition ------------------------------------------------
    # ds/v integrated is the time to drive the lap at that speed. Guard the
    # slow crawl out of a wrap where v -> 0 makes 1/v explode.
    def lap_time_of(v):
        vv = np.clip(np.nan_to_num(v, nan=np.nan), 0.5, None)
        return float(np.nansum(ds / vv))

    t_plan = lap_time_of(v_plan)
    t_true = lap_time_of(v_true)
    times = np.array([l[2] for l in keep])

    print(f"\n{'=' * 78}\n{os.path.basename(path)}   line: {line['name']}\n{'=' * 78}")
    print(f"{len(keep)} clean laps of {len(laps)} complete   window [{lo:.2f}, {hi:.2f}] s")
    print(f"measured   best {times.min():.2f}   median {np.median(times):.2f}   mean {times.mean():.3f} s")
    print(f"profile predicts {t_plan:.2f} s   integrated from the achieved speed {t_true:.2f} s")
    print(f"GAP to the profile: {np.median(times) - t_plan:+.2f} s per lap")

    loss_track = np.zeros_like(grid)
    if have_pp:
        # ALIGN v_target before comparing it to the plan. The follower samples
        # the profile a lead of v * cmd_delay + lead_min AHEAD of the car,
        # because the throttle it sets does not land until the car has moved
        # that far (pure_pursuit.py, "land the wheel on the profile's speed at
        # the profile's position"). So the v_target logged at s is the plan's
        # value for s + lead, and comparing the two at the same s scores that
        # deliberate lead as a loss on every braking zone and as a GAIN on every
        # acceleration -- which is what drove loss_limit negative over s 0-20.
        # Shifting it back by the lead leaves only what the follower genuinely
        # rewrote: the v_max / v_min clip and the slow-loop derate.
        lead = np.clip(v_true, 0.0, None) * cmd_delay + lead_min
        aligned = np.interp((grid - lead) % L, grid, v_targ, period=L)
        vt = np.clip(aligned, 0.5, None)
        vp = np.clip(v_plan, 0.5, None)
        vv = np.clip(v_true, 0.5, None)
        loss_limit = ds / vt - ds / vp        # plan -> commanded: the clip only
        loss_track = ds / vv - ds / vt        # commanded -> achieved: delivery
        clipped = aligned < v_plan - 0.05
        print(f"\n  follower clip  (v_plan -> v_target, lead-aligned): {np.nansum(loss_limit):+.3f} s"
              f"   over {np.nanmean(clipped) * 100:.0f} % of the lap")
        print(f"  delivery       (v_target -> v_true):               {np.nansum(loss_track):+.3f} s"
              f"   <-- the controller's half")
        d = v_true - aligned
        print(f"  speed deficit vs the aligned target: median {np.nanmedian(d):+.3f}  "
              f"mean {np.nanmean(d):+.3f} m/s   under it on {np.nanmean(d < -0.05) * 100:.0f} % of the lap")
        if clipped.any():
            print(f"  clip bites at s {grid[clipped].min():.1f}-{grid[clipped].max():.1f} m "
                  f"(plan asks up to {np.nanmax(v_plan[clipped]):.2f}, follower allows {np.nanmax(aligned[clipped]):.2f} m/s)")
    else:
        loss_limit = np.zeros_like(grid)
        print("\n(no pp_v_target column: cannot split the clip from delivery)")

    # ---- where ------------------------------------------------------------
    print(f"\n{'section':<22}{'len':>7}{'limit s':>10}{'track s':>10}{'deficit':>10}{'slip pin':>10}")
    secs = sections(grid, kap_line, L)
    rows = []
    for s0, s1, kind, a, b in secs:
        lt, lk = np.nansum(loss_limit[a:b]), np.nansum(loss_track[a:b])
        dfc = np.nanmedian((v_true - v_targ)[a:b]) if have_pp else np.nan
        pin = (np.nanmean(np.abs(M['slip'][:, a:b]) >= 0.95 * M['band'][:, a:b]) * 100
               if have_pp and 'slip' in M else np.nan)
        rows.append((s0, s1, kind, lt, lk, dfc, pin))
        print(f"{f'{kind} {s0:.0f}-{s1:.0f} m':<22}{s1 - s0:>6.1f}m{lt:>10.3f}{lk:>10.3f}"
              f"{dfc:>10.2f}{pin:>9.0f}%")
    if have_pp:
        worst = sorted(rows, key=lambda r: -r[4])[:3]
        print("\nworst tracking sections: " + ", ".join(
            f"{r[2]} {r[0]:.0f}-{r[1]:.0f} m ({r[4]:+.3f} s, slip pinned {r[6]:.0f} %)" for r in worst))

    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.join(out_dir, os.path.splitext(os.path.basename(path))[0])
    made = []

    # ---- 1: the profile ---------------------------------------------------
    fig, (ax, axk) = plt.subplots(2, 1, figsize=(13, 7), sharex=True,
                                  gridspec_kw=dict(height_ratios=[3, 1]))
    ax.fill_between(grid, v_lo, v_hi, color='0.85', label='achieved p10-p90 across laps')
    ax.plot(grid, v_plan, color='#1a7f37', lw=2.0, label='planned (raceline v_mps)')
    if have_pp:
        ax.plot(grid, v_targ, color='#0b6bcb', lw=1.4, ls='--', label='commanded (v_target)')
    ax.plot(grid, v_true, color='#c0392b', lw=1.6, label='achieved (median lap)')
    ax.set_ylabel('speed [m/s]')
    ax.set_title(f'{os.path.basename(path)} -- {len(keep)} clean laps, median {np.median(times):.2f} s '
                 f'vs {t_plan:.2f} s planned')
    ax.legend(loc='lower right', fontsize=8), ax.grid(alpha=.3)
    axk.fill_between(grid, 0, np.abs(kap_line), color='#6c3483', alpha=.5)
    axk.set_ylabel('|kappa|'), axk.set_xlabel('s along lap [m]'), axk.grid(alpha=.3)
    fig.tight_layout(), fig.savefig(f'{stem}_speed_profile.png', dpi=130), plt.close(fig)
    made.append(f'{stem}_speed_profile.png')

    # ---- 2: where the time goes -------------------------------------------
    fig, ax = plt.subplots(figsize=(13, 5))
    ax.plot(grid, np.nancumsum(loss_track), color='#c0392b', lw=2,
            label=f'delivery (v_target -> v_true)  {np.nansum(loss_track):+.2f} s')
    if have_pp:
        ax.plot(grid, np.nancumsum(loss_limit), color='#0b6bcb', lw=2,
                label=f'follower clip, lead-aligned (v_plan -> v_target)  {np.nansum(loss_limit):+.2f} s')
        ax.plot(grid, np.nancumsum(loss_limit + loss_track), color='0.2', lw=2.2, ls='--',
                label=f'total  {np.nansum(loss_limit + loss_track):+.2f} s')
    for s0, s1, kind, *_ in secs:
        if kind == 'corner':
            ax.axvspan(s0, s1, color='#6c3483', alpha=.10)
    ax.axhline(0, color='k', lw=.8)
    ax.set_xlabel('s along lap [m]'), ax.set_ylabel('cumulative time lost [s]')
    ax.set_title('Where the lap time goes against the plan (shaded = corners)')
    ax.legend(loc='upper left', fontsize=9), ax.grid(alpha=.3)
    fig.tight_layout(), fig.savefig(f'{stem}_time_loss.png', dpi=130), plt.close(fig)
    made.append(f'{stem}_time_loss.png')

    if have_pp and 'slip' in M:
        # ---- 3: the band --------------------------------------------------
        sl_med = np.nanmedian(M['slip'], 0)
        sl_lo, sl_hi = np.nanpercentile(M['slip'], 10, 0), np.nanpercentile(M['slip'], 90, 0)
        pin = np.nanmean(np.abs(M['slip']) >= 0.95 * M['band'], 0) * 100
        band_med = np.nanmedian(M['band'], 0)
        fig, (ax, axp) = plt.subplots(2, 1, figsize=(13, 7), sharex=True,
                                      gridspec_kw=dict(height_ratios=[2, 1]))
        ax.fill_between(grid, sl_lo, sl_hi, color='0.85', label='slip cmd p10-p90')
        ax.plot(grid, sl_med, color='#c0392b', lw=1.5, label='slip cmd (median lap)')
        ax.plot(grid, band_med, color='#0b6bcb', lw=1.6, label='friction-circle band')
        ax.plot(grid, -band_med, color='#0b6bcb', lw=1.6)
        ax.axhline(0, color='k', lw=.8)
        ax.set_ylabel('commanded slip'), ax.legend(loc='upper right', fontsize=8), ax.grid(alpha=.3)
        ax.set_title('Slip command against the friction-circle band -- pinning = the band is the limit')
        axp.fill_between(grid, 0, pin, color='#c0392b', alpha=.6)
        axp.set_ylabel('% of laps pinned\nat the band'), axp.set_xlabel('s along lap [m]'), axp.grid(alpha=.3)
        fig.tight_layout(), fig.savefig(f'{stem}_slip_band.png', dpi=130), plt.close(fig)
        made.append(f'{stem}_slip_band.png')

        # ---- 4: the preview mismatch --------------------------------------
        # The band WIDTH is evaluated from the curvature under the car now, but
        # the throttle it sets does not land for cmd_delay seconds, by which
        # time the car has moved v * cmd_delay along the path. On a corner exit
        # kappa is falling, so the band is scaled down by a corner the car has
        # already left.
        a_lat_now = v_true ** 2 * np.abs(kap_line)
        s_land = (grid + v_true * cmd_delay) % L
        a_lat_land = v_true ** 2 * np.abs(np.interp(s_land, grid, np.abs(kap_line)))
        f_now = np.sqrt(np.clip(1 - (a_lat_now / cap) ** 2, 0, None))
        f_land = np.sqrt(np.clip(1 - (a_lat_land / cap) ** 2, 0, None))
        band_now = np.maximum(slip_circle * f_now, 0.02)
        band_land = np.maximum(slip_circle * f_land, 0.02)
        gain = band_land - band_now
        fig, ax = plt.subplots(figsize=(13, 5))
        ax.plot(grid, band_now, color='#c0392b', lw=1.8, label='band width now (what the code uses)')
        ax.plot(grid, band_land, color='#1a7f37', lw=1.8,
                label=f'band width at landing (+{cmd_delay * 1000:.0f} ms downstream)')
        ax.fill_between(grid, band_now, band_land, where=band_land > band_now,
                        color='#1a7f37', alpha=.25, label='band the exit is denied')
        ax.set_xlabel('s along lap [m]'), ax.set_ylabel('slip band half-width')
        ax.set_title(f'Friction-circle band: evaluated under the car vs where the throttle lands  '
                     f'(mean gain on exits {gain[gain > 0].mean():+.3f})')
        ax.legend(loc='lower right', fontsize=9), ax.grid(alpha=.3)
        fig.tight_layout(), fig.savefig(f'{stem}_band_preview.png', dpi=130), plt.close(fig)
        made.append(f'{stem}_band_preview.png')
        print(f"\nband preview: widening to the landing point would give a mean "
              f"{gain[gain > 0].mean():+.3f} of extra slip over {np.mean(gain > 0.005) * 100:.0f} % of the lap")

    print("\nwrote:\n  " + "\n  ".join(made))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--path', help='raceline CSV the run followed; auto-detected when omitted')
    ap.add_argument('--out', default=os.path.join(HERE, '..', 'figures'), help='directory for the PNGs')
    ap.add_argument('--tol', type=float, default=0.35, help='clean-lap window, +- this many s around the median')
    ap.add_argument('--lap-lo', type=float, default=None, help='explicit clean-lap window, low [s]')
    ap.add_argument('--lap-hi', type=float, default=None, help='explicit clean-lap window, high [s]')
    ap.add_argument('--ds', type=float, default=0.05, help='s grid spacing [m]')
    ap.add_argument('--cmd-delay', type=float, default=0.15, help='throttle round trip, for the band preview [s]')
    ap.add_argument('--slip-accel', type=float, default=0.16, help="the run's slip_accel (used when slip_circle is 0)")
    ap.add_argument('--slip-brake', type=float, default=0.08, help="the run's slip_brake (used when slip_circle is 0)")
    ap.add_argument('--lead-min', type=float, default=0.10, help="the run's lead_min_m")
    ap.add_argument('--slip-circle', type=float, default=0.12, help='the run\'s slip_circle')
    ap.add_argument('--a-lat-cap', type=float, default=7.0, help="the run's steer_a_lat_max")
    a = ap.parse_args()
    for p in a.logs:
        analyze(p, a.path, a.out, a.tol, a.lap_lo, a.lap_hi, a.ds, a.cmd_delay,
                a.slip_circle, a.a_lat_cap, a.lead_min, a.slip_accel, a.slip_brake)


if __name__ == '__main__':
    main()
