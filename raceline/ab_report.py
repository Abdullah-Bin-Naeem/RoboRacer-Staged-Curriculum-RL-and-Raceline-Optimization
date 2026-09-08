#!/usr/bin/env python3
"""A/B report for two follower runs: the numbers and the figures, in one pass.

    python3 raceline/ab_report.py base.csv:"baseline" test.csv:"E0 delay_ref 0" \
        --path raceline/icra2026/raceline_a7.0.csv --name E0

Writes figures/<name>_ab.png (five panels) and prints a markdown report ready to
paste into docs/plans/CONTROLLER_TUNING_PLAN.md §8.

Why this exists rather than compare_runs.py: the decision gate in §8.5 is four
tests -- collisions, median, per-section time, corner clearance -- plus the
audit's fifth (pp_delay parity between the arms). compare_runs.py draws the
speed overlay and nothing else, so those five were being eyeballed. Every number
printed here is one of the five, and the effective-lookahead panel exists because
a run's lookahead is NOT its lookahead_max (see the plan, §5.1 audit box).
"""
import argparse, math, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
COL = ('#1f77b4', '#d62728')


def load(path):
    import csv
    rows = list(csv.DictReader(open(path)))
    g = lambda k: np.array([float(r[k]) if r[k] not in ('', 'nan') else np.nan
                            for r in rows])
    d = {k: g(k) for k in rows[0] if k != ''}
    return d


def laps(d, lo, hi):
    """Lap times from the s wraparound, out-lap discarded. Returns (times, edges)."""
    s, t = d['pp_s'], d['t']
    w = [i for i in range(1, len(s)) if s[i - 1] - s[i] > 30]
    out = [(t[w[i]] - t[w[i - 1]], w[i - 1], w[i]) for i in range(1, len(w))]
    clean = [x for x in out if lo <= x[0] <= hi]
    return out, clean


def resample(d, a, b, grid, L, key):
    s, v = d['pp_s'][a:b], d[key][a:b]
    m = ~np.isnan(v)
    s, v = s[m], v[m]
    if len(s) < 10:
        return None
    o = np.argsort(s)
    return np.interp(grid, s[o], v[o])


def stats(d, lo, hi, L, grid):
    allx, clean = laps(d, lo, hi)
    lt = np.array([x[0] for x in allx])
    e = np.abs(d['pp_e_lat']); e = e[~np.isnan(e)]
    dly = d['pp_delay'][~np.isnan(d['pp_delay'])]
    ld = d['pp_ld'][~np.isnan(d['pp_ld'])]
    v = d['pp_v_est'][~np.isnan(d['pp_ld'])]
    coll = d['collisions']
    prof = {k: np.nanmean([r for r in
            (resample(d, a, b, grid, L, k) for _, a, b in clean) if r is not None], 0)
            for k in ('pp_v_est', 'pp_v_target', 'pp_e_lat', 'pp_ld')}
    exc = d['pp_s'][np.abs(d['pp_e_lat']) > 0.25]
    return dict(
        n=len(lt), best=lt.min(), med=np.median(lt), mean=lt.mean(), worst=lt.max(),
        n_clean=len(clean), lt=lt,
        coll=int(coll[-1] - coll[0]),
        e_mean=e.mean(), e_p90=np.percentile(e, 90), e_max=e.max(),
        e_worst_s=float(d['pp_s'][np.nanargmax(np.abs(d['pp_e_lat']))]),
        n_exc=int(len(exc)), exc_s=exc[~np.isnan(exc)],
        dly_med=np.median(dly), dly_p90=np.percentile(dly, 90),
        ld_med=np.median(ld), ld_fast=np.median(ld[v > 6.0]) if (v > 6.0).any() else np.nan,
        ld_max=ld.max(), prof=prof)


def section_loss(prof_v, v_plan, grid, ds, secs):
    """Seconds lost to the plan per section, from the achieved speed."""
    out = {}
    for lo, hi, nm in secs:
        m = (grid >= lo) & (grid < hi)
        vv = np.clip(prof_v[m], 0.5, None)
        vp = np.clip(v_plan[m], 0.5, None)
        out[nm] = float(np.sum(ds / vv - ds / vp))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs=2, help='base.csv:label test.csv:label')
    ap.add_argument('--path', required=True)
    ap.add_argument('--name', default='ab')
    ap.add_argument('--out', default=os.path.join(HERE, '..', 'figures'))
    ap.add_argument('--ds', type=float, default=0.05)
    ap.add_argument('--lap-lo', type=float, default=11.4)
    ap.add_argument('--lap-hi', type=float, default=12.4)
    a = ap.parse_args()

    import csv
    pr = list(csv.DictReader(open(a.path)))
    ps = np.array([float(r['# s_m']) for r in pr])
    pv = np.array([float(r['v_mps']) for r in pr])
    L = ps[-1]
    grid = np.arange(0, L, a.ds)
    v_plan = np.interp(grid, ps, pv)
    # The plan's lap time by trapezoid on the ORIGINAL points: sum(ds / v_mean).
    # Not sum(ds/v) on the resampled grid -- that is a harmonic average and reads
    # 11.416 against the true 11.440 on a7.0, which would quietly move every
    # "gap vs plan" by 0.024 s.
    t_plan = float(sum((ps[i] - ps[i - 1]) / (0.5 * (pv[i] + pv[i - 1]))
                       for i in range(1, len(ps)))
                   + (ps[1] - ps[0]) / (0.5 * (pv[0] + pv[-1])))
    # Grid-integrated plan, used ONLY as the reference for the section
    # decomposition below so that both arms are scored the same way.
    t_plan_grid = float(np.sum(a.ds / np.clip(v_plan, 0.5, None)))

    names, S = [], []
    for spec in a.runs:
        f, _, lab = spec.partition(':')
        names.append(lab or os.path.basename(f))
        S.append(stats(load(f), a.lap_lo, a.lap_hi, L, grid))
    B, T = S
    bn, tn = names

    secs = [(0, 21, 'straight 0–21'), (21, 24, 'corner 21–24 (T2)'),
            (24, 37, 'straight 24–37'), (37, 41, 'corner 37–41'),
            (41, 43, 'straight 41–43'), (43, 46, 'corner 43–46'),
            (46, L, 'straight 46–end')]
    lb = section_loss(B['prof']['pp_v_est'], v_plan, grid, a.ds, secs)
    lt_ = section_loss(T['prof']['pp_v_est'], v_plan, grid, a.ds, secs)

    # ---------------- report ----------------
    P = print
    P(f"\n## A/B: {bn}  →  {tn}\n")
    P(f"Plan on `{os.path.basename(a.path)}` = **{t_plan:.3f} s**.\n")
    P("### Lap time\n")
    P("| | laps | clean | best | median | mean | worst | coll | gap vs plan |")
    P("|---|---|---|---|---|---|---|---|---|")
    for nm, s in ((bn, B), (tn, T)):
        P(f"| {nm} | {s['n']} | {s['n_clean']} | {s['best']:.3f} | {s['med']:.3f} | "
          f"{s['mean']:.3f} | {s['worst']:.3f} | {s['coll']} | {s['med']-t_plan:+.3f} |")
    P(f"| **Δ** | | | **{T['best']-B['best']:+.3f}** | **{T['med']-B['med']:+.3f}** | "
      f"**{T['mean']-B['mean']:+.3f}** | | **{T['coll']-B['coll']:+d}** | |")

    P("\n### Lateral\n")
    P("| | \\|e\\| mean | \\|e\\| p90 | \\|e\\| max | s of worst | ticks \\|e\\|>0.25 |")
    P("|---|---|---|---|---|---|")
    for nm, s in ((bn, B), (tn, T)):
        P(f"| {nm} | {s['e_mean']:.3f} | {s['e_p90']:.3f} | {s['e_max']:.3f} | "
          f"{s['e_worst_s']:.1f} | {s['n_exc']} |")

    P("\n### Effective lookahead and loop delay — the audit checks\n")
    P("| | `pp_delay` med | p90 | `pp_ld` med | `pp_ld` at v>6 | `pp_ld` max |")
    P("|---|---|---|---|---|---|")
    for nm, s in ((bn, B), (tn, T)):
        P(f"| {nm} | {s['dly_med']:.3f} | {s['dly_p90']:.3f} | {s['ld_med']:.3f} | "
          f"{s['ld_fast']:.3f} | {s['ld_max']:.3f} |")
    dd = abs(T['dly_med'] - B['dly_med'])
    P(f"\n`pp_delay` median differs by **{dd:.3f} s** — "
      f"{'OK, arms comparable' if dd <= 0.01 else '**> 0.01 s: the arms are NOT directly comparable** (§8.5)'}.")

    P("\n### Section-level time loss vs plan\n")
    P(f"> Decomposition of the *grid-integrated* lap ({t_plan_grid:.3f} s plan reference) using the")
    P("> mean speed profile over clean laps. Both arms are scored identically, so the **Δ column is")
    P("> the result**; the totals do not equal the measured median gap above (Jensen: integrating a")
    P("> mean speed is not the mean of integrated times) and should not be quoted as if they did.\n")
    P(f"| Section | {bn} | {tn} | Δ |")
    P("|---|---|---|---|")
    for _, _, nm in secs:
        P(f"| {nm} | {lb[nm]:+.3f} | {lt_[nm]:+.3f} | {lt_[nm]-lb[nm]:+.3f} |")
    P(f"| **total** | **{sum(lb.values()):+.3f}** | **{sum(lt_.values()):+.3f}** | "
      f"**{sum(lt_.values())-sum(lb.values()):+.3f}** |")

    P("\n### Decision gate (§8.5)\n")
    worst = max(lt_[n] - lb[n] for _, _, n in secs)
    checks = [("collisions in run = 0", T['coll'] == 0),
              ("median improves by >= 0.03 s, or holds with better |e| p90",
               (B['med'] - T['med']) >= 0.03 or
               (T['med'] <= B['med'] + 0.005 and T['e_p90'] < B['e_p90'])),
              ("no section worse by > 0.03 s", worst <= 0.03),
              ("pp_delay median within 0.01 s of the other arm", dd <= 0.01)]
    for label, ok in checks:
        P(f"- [{'x' if ok else ' '}] {label}")
    P(f"\n**{'ACCEPT' if all(o for _, o in checks) else 'REJECT'}** on the gate as written.")

    # ---------------- figures ----------------
    fig, ax = plt.subplots(5, 1, figsize=(13, 17), constrained_layout=True)
    fig.suptitle(f'{bn}  vs  {tn}', fontsize=15, fontweight='bold')

    ax[0].plot(grid, v_plan, 'k--', lw=1.2, label=f'plan ({t_plan:.3f} s)')
    for c, nm, s in zip(COL, (bn, tn), (B, T)):
        ax[0].plot(grid, s['prof']['pp_v_est'], c, lw=1.4, label=f'{nm} achieved')
        ax[0].plot(grid, s['prof']['pp_v_target'], c, lw=0.9, ls=':', alpha=.75,
                   label=f'{nm} v_target')
    ax[0].set_ylabel('speed [m/s]'); ax[0].set_title('Speed, mean of clean laps')
    ax[0].legend(ncol=3, fontsize=8); ax[0].grid(alpha=.3)

    for c, nm, s in zip(COL, (bn, tn), (B, T)):
        ax[1].plot(grid, s['prof']['pp_ld'], c, lw=1.4, label=f'{nm}  (med at v>6: {s["ld_fast"]:.2f} m)')
    ax[1].axhline(1.54, color='gray', ls=':', lw=1)
    ax[1].annotate('0.70 × lookahead_max = 1.54 m (the floored d_scale)', (1, 1.57), fontsize=8, color='gray')
    ax[1].axhline(2.20, color='green', ls=':', lw=1)
    ax[1].annotate('yaml lookahead_max = 2.20 m', (1, 2.23), fontsize=8, color='green')
    ax[1].set_ylabel('Ld [m]'); ax[1].set_title('Effective lookahead — is the controller running the configured value?')
    ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)

    for c, nm, s in zip(COL, (bn, tn), (B, T)):
        ax[2].plot(grid, np.abs(s['prof']['pp_e_lat']), c, lw=1.4, label=nm)
    ax[2].axhline(0.25, color='r', ls=':', lw=1)
    ax[2].set_ylabel('|e_lat| [m]'); ax[2].set_title('Path error, mean of clean laps (red = 0.25 m excursion line)')
    ax[2].legend(fontsize=8); ax[2].grid(alpha=.3)

    bins = np.arange(0, L + 2, 2)
    for c, nm, s in zip(COL, (bn, tn), (B, T)):
        ax[3].hist(s['exc_s'], bins=bins, alpha=.6, color=c, label=f'{nm}  ({s["n_exc"]} ticks)')
    ax[3].set_ylabel('ticks |e| > 0.25 m'); ax[3].set_xlabel('s [m]')
    ax[3].set_title('Where the car leaves the line — local or global?')
    ax[3].legend(fontsize=8); ax[3].grid(alpha=.3)

    x = np.arange(len(secs)); w = 0.38
    ax[4].bar(x - w/2, [lb[n] for _, _, n in secs], w, color=COL[0], label=bn)
    ax[4].bar(x + w/2, [lt_[n] for _, _, n in secs], w, color=COL[1], label=tn)
    ax[4].set_xticks(x); ax[4].set_xticklabels([n for _, _, n in secs], rotation=20, ha='right', fontsize=8)
    ax[4].set_ylabel('s lost to plan'); ax[4].set_title('Section-level time loss')
    ax[4].legend(fontsize=8); ax[4].grid(alpha=.3, axis='y')

    os.makedirs(a.out, exist_ok=True)
    p = os.path.join(a.out, f'{a.name}_ab.png')
    fig.savefig(p, dpi=110)
    P(f"\nFigure: `{os.path.relpath(p)}`")


if __name__ == '__main__':
    main()
