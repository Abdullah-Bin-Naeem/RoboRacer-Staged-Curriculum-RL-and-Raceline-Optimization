#!/usr/bin/env python3
"""Overlay two or more runs against the SAME planned profile.

    python3 raceline/compare_runs.py --path <line.csv> \
        base.csv:"slip_circle 0.12" test.csv:"slip_circle 0.16"

Each argument is <log.csv> or <log.csv>:<label>. Produces one chart with the
plan and every run's achieved speed on it, one with each run's cumulative time
lost against the plan, and a per-section table -- so an A/B is read off a
single picture instead of two runs of numbers.

Time here is measured against the PLAN at each s, which is the honest total:
it does not try to split out the follower's clip (see plot_speed_tracking.py
for that split and for why v_target must be lead-aligned first).
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
import analyze_run as ar                                            # noqa: E402
import plot_speed_tracking as pst                                   # noqa: E402

COLORS = ['#c0392b', '#0b6bcb', '#1a7f37', '#b7791f', '#6c3483']


def load(path, line, grid, ds, tol, lo, hi):
    log = ar.load_log(path)
    L = line['L']
    e, kap, s = pst.project_chunked(log['true_x'], log['true_y'], line)
    laps = pst.lap_bounds(s, log['t'], L)
    keep, lo2, hi2 = pst.clean_laps(laps, tol, lo, hi)
    if not keep:
        sys.exit(f'{path}: no clean laps in [{lo2:.2f}, {hi2:.2f}]')
    V, S = [], []
    for a, b, _ in keep:
        sl = slice(a, b)
        V.append(pst.resample(s[sl], log['speed'][sl], grid))
        if 'pp_slip' in log:
            S.append(pst.resample(s[sl], log['pp_slip'][sl], grid))
    times = np.array([l[2] for l in keep])
    coll = int(log['collisions'][-1] - log['collisions'][0]) if 'collisions' in log else -1
    return dict(name=os.path.basename(path), v=np.nanmedian(np.array(V), 0),
                lo=np.nanpercentile(np.array(V), 10, 0),
                hi=np.nanpercentile(np.array(V), 90, 0),
                times=times, n=len(keep), coll=coll,
                slip=np.nanmedian(np.array(S), 0) if S else None)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('runs', nargs='+', help='log.csv or log.csv:label')
    ap.add_argument('--path', required=True)
    ap.add_argument('--out', default=os.path.join(HERE, '..', 'figures'))
    ap.add_argument('--name', default='compare')
    ap.add_argument('--ds', type=float, default=0.05)
    ap.add_argument('--tol', type=float, default=0.35)
    ap.add_argument('--lap-lo', type=float, default=None)
    ap.add_argument('--lap-hi', type=float, default=None)
    a = ap.parse_args()

    line = ar.load_line(a.path)
    if line['v'] is None:
        sys.exit(f'{a.path}: no v_mps column')
    L = line['L']
    grid = np.arange(0.0, L, a.ds)
    v_plan = np.interp(grid, line['s'], line['v'])
    kap = np.interp(grid, line['s'], line['kappa'])
    t_plan = float(np.sum(a.ds / np.clip(v_plan, 0.5, None)))

    runs, labels = [], []
    for spec in a.runs:
        path, _, label = spec.partition(':')
        r = load(path, line, grid, a.ds, a.tol, a.lap_lo, a.lap_hi)
        runs.append(r)
        labels.append(label or r['name'])

    print(f"\nplan: {os.path.basename(a.path)}   predicts {t_plan:.3f} s\n")
    print(f"{'run':<26}{'laps':>6}{'best':>8}{'median':>9}{'mean':>8}{'coll':>6}{'gap':>8}")
    for r, lab in zip(runs, labels):
        print(f"{lab:<26}{r['n']:>6}{r['times'].min():>8.2f}{np.median(r['times']):>9.2f}"
              f"{r['times'].mean():>8.2f}{r['coll']:>6}{np.median(r['times']) - t_plan:>+8.2f}")

    secs = pst.sections(grid, kap, L)
    print(f"\nseconds lost to the plan, by section")
    print(f"{'section':<22}" + ''.join(f'{lab[:14]:>16}' for lab in labels))
    for s0, s1, kind, i0, i1 in secs:
        row = f"{f'{kind} {s0:.0f}-{s1:.0f} m':<22}"
        for r in runs:
            loss = np.nansum(a.ds / np.clip(r['v'][i0:i1], 0.5, None)
                             - a.ds / np.clip(v_plan[i0:i1], 0.5, None))
            row += f'{loss:>16.3f}'
        print(row)

    os.makedirs(a.out, exist_ok=True)
    stem = os.path.join(a.out, a.name)

    fig, (ax, axk) = plt.subplots(2, 1, figsize=(13, 7), sharex=True,
                                  gridspec_kw=dict(height_ratios=[3, 1]))
    ax.plot(grid, v_plan, color='k', lw=2.4, label=f'planned  ({t_plan:.2f} s)')
    for r, lab, c in zip(runs, labels, COLORS):
        ax.fill_between(grid, r['lo'], r['hi'], color=c, alpha=.12)
        ax.plot(grid, r['v'], color=c, lw=1.7,
                label=f"{lab}  ({np.median(r['times']):.2f} s, {r['n']} laps, {r['coll']} coll)")
    ax.set_ylabel('speed [m/s]'), ax.grid(alpha=.3)
    ax.set_title('Achieved speed against the planned profile')
    ax.legend(loc='lower right', fontsize=8)
    axk.fill_between(grid, 0, np.abs(kap), color='#6c3483', alpha=.5)
    axk.set_ylabel('|kappa|'), axk.set_xlabel('s along lap [m]'), axk.grid(alpha=.3)
    fig.tight_layout(), fig.savefig(f'{stem}_speed.png', dpi=130), plt.close(fig)

    fig, ax = plt.subplots(figsize=(13, 5))
    for r, lab, c in zip(runs, labels, COLORS):
        loss = a.ds / np.clip(r['v'], 0.5, None) - a.ds / np.clip(v_plan, 0.5, None)
        ax.plot(grid, np.nancumsum(loss), color=c, lw=2,
                label=f'{lab}   total {np.nansum(loss):+.2f} s')
    for s0, s1, kind, *_ in secs:
        if kind == 'corner':
            ax.axvspan(s0, s1, color='#6c3483', alpha=.10)
    ax.axhline(0, color='k', lw=.8)
    ax.set_xlabel('s along lap [m]'), ax.set_ylabel('cumulative time lost vs plan [s]')
    ax.set_title('Where each run loses time against the plan (shaded = corners)')
    ax.legend(loc='upper left', fontsize=9), ax.grid(alpha=.3)
    fig.tight_layout(), fig.savefig(f'{stem}_timeloss.png', dpi=130), plt.close(fig)
    print(f"\nwrote:\n  {stem}_speed.png\n  {stem}_timeloss.png")


if __name__ == '__main__':
    main()
