#!/usr/bin/env python3

"""Replay a recorded run through localization_v2, offline, and score it.

    python3 replay_localization_v2.py RUN.npz [--track iros2026] [--params localization_v2.yaml]
                                      [--set rate_m_s=0.3,0.6,0.25,0.5] [--csv out.csv]

RUN.npz comes from log_localization with scan_dump:=RUN.npz (race.launch.py
passes it through in dev mode): every scan, its stamp, odom->base at that
stamp, the true pose, and the map->odom the live localizer (AMCL) had at the
time. This tool runs v2_filter.V2Filter -- the SAME class the node runs -- over
those scans, seeded from the first true pose, and prints, for v2 and for the
live localizer on identical data:

    err_along / err_cross / err_dist   mean, p90, max        (moving only)
    per-mode breakdown                 where the error lives
    correction walk on the blind straight   the number that exposed AMCL
    rejections by reason, compute time

A tuning iteration is this command: seconds, not a two-minute simulator run,
and reproducible. --set overrides any V2Filter/ScanMatcher parameter from the
command line so a sweep is a shell loop.

The predictions this has to meet (plan, 2026-09-19), on the AMCL-line runs:
    v2 err_along p90 < 0.10 m       (AMCL 0.32-0.51)
    v2 err_cross p90 <= 0.05 m      (unchanged)
    v2 no worse than the live localizer on either axis
    worst per-sample correction step <= 0.05 m (AMCL's reaches 0.46)
"""

import argparse
import csv
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'racer_common'))
from racer_localization.scan_matcher import (LikelihoodField, rebase_repo_path,
                                             read_map_yaml, scan_to_points)  # noqa: E402
from racer_localization.v2_filter import Segments, V2Filter                              # noqa: E402
from racer_localization.v2_status import MODES, REJECT_CODES, STATUS_FIELDS              # noqa: E402

LIDAR_XY = (0.2733, 0.0)
MATCHER_KEYS = ('sigma', 'sigma_start', 'max_iters', 'inlier_m', 'min_inlier_frac',
                'max_resid_m', 'max_step_m', 'yaw_dof')
FILTER_KEYS = ('q_along', 'q_cross', 'p_init', 'blind_along_info', 'gain_along', 'gain_cross',
               'rate_m_s', 'mode_hysteresis_m', 'beam_sigma_m', 'info_scale', 'min_clearance_m',
               'gate_sigma', 'gate_floor_m', 'p_floor_along_m', 'p_floor_cross_m',
               'est_scale', 'q_scale', 'p_scale_init', 'scale_max', 'scale_init',
               'k_accel', 'accel_tau_s', 'accel_max', 'rate_cross_m_s', 'rate_cross_gap_m',
               'odom_accel_max', 'odom_speed_window_s', 'odom_spin_margin')




def load_params(path):
    """config/localization_v2.yaml -> flat dict (hand parser when PyYAML is absent)."""
    try:
        import yaml
        return yaml.safe_load(open(path))['localization_v2']['ros__parameters']
    except ImportError:
        out = {}
        for raw in open(path):
            line = raw.split('#', 1)[0].strip()
            if ':' not in line or line.endswith(':'):
                continue
            k, v = (t.strip() for t in line.split(':', 1))
            if v.startswith('['):
                out[k] = [float(t) for t in v.strip('[]').split(',')]
            elif v in ('true', 'false'):
                out[k] = v == 'true'
            else:
                try:
                    out[k] = float(v) if ('.' in v or 'e' in v) else int(v)
                except ValueError:
                    out[k] = v
        return out


def parse_set(items, params):
    for it in items or []:
        k, _, v = it.partition('=')
        if ',' in v:
            params[k] = [float(t) for t in v.split(',')]
        elif v in ('true', 'false'):
            params[k] = v == 'true'
        else:
            params[k] = float(v)
    return params


def stats(v):
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    if len(v) == 0:
        return 'n/a'
    return f'|mean| {np.abs(v).mean():.3f}  mean {v.mean():+.3f}  p90 {np.percentile(np.abs(v), 90):.3f}  max {np.abs(v).max():.3f}'


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('npz')
    ap.add_argument('--track', default=os.environ.get('RACER_TRACK', 'iros2026'))
    ap.add_argument('--map', help='map yaml; default: the npz\'s, else the track\'s')
    ap.add_argument('--params', help='default: config/localization_v2.yaml')
    ap.add_argument('--segments', help='default: raceline/<track>/segments.csv')
    ap.add_argument('--set', action='append', metavar='KEY=VALUE', help='override a parameter')
    ap.add_argument('--stride', type=int, help='beam stride (default from params: beam_stride)')
    ap.add_argument('--seed-std', type=float, default=0.10, help='seed uncertainty from the first true pose')
    ap.add_argument('--min-speed', type=float, default=0.5, help='rows below this speed are not scored')
    ap.add_argument('--csv', help='write per-scan rows (true, v2, live, status) here')
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()

    from racer_common import frames
    d = np.load(a.npz, allow_pickle=False)
    map_yaml = a.map or (str(d['map_yaml']) if 'map_yaml' in d else '') or frames.map_yaml(a.track)
    map_yaml = rebase_repo_path(map_yaml)
    seg_csv = a.segments or rebase_repo_path(os.path.join(frames.raceline_dir(a.track), 'segments.csv'))
    params = load_params(a.params or os.path.join(HERE, '..', 'config', 'localization_v2.yaml'))
    params = parse_set(a.set, params)
    stride = a.stride or int(params.get('beam_stride', 3))

    t0 = time.time()
    field = LikelihoodField(map_yaml)
    segments = Segments(seg_csv)
    filt = V2Filter(field, segments,
                    matcher_kwargs={k: params[k] for k in MATCHER_KEYS if k in params},
                    **{k: params[k] for k in FILTER_KEYS if k in params})
    print(f'map {os.path.basename(map_yaml)} ({time.time() - t0:.1f} s); segments {len(segments)} '
          f'{segments.counts()}; params {os.path.basename(a.params) if a.params else "package default"}'
          + (f' + {a.set}' if a.set else ''))

    ranges, stamps, o2b, true, m2o, speed = (d['ranges'], d['stamp'], d['o2b'], d['true'],
                                              d['m2o'], d['speed'])
    amin, ainc = float(d['angle_min']), float(d['angle_increment'])
    rmin, rmax = float(d['range_min']), float(d['range_max'])
    n = len(ranges)
    print(f'{n} scans, {stamps[-1] - stamps[0]:.1f} s, {float(np.nanmedian(1.0 / np.diff(stamps))):.1f} Hz')

    rows = []
    v2_err = {'along': [], 'cross': [], 'dist': []}
    live_err = {'along': [], 'cross': [], 'dist': []}
    per_mode = {m: {'along': [], 'cross': []} for m in MODES}
    walk = {m: [0.0, 0.0] for m in MODES}          # [along, cross] |steps| of v2's published correction
    live_walk = {m: [0.0, 0.0] for m in MODES}
    rejects = {}
    compute = []
    prev_pub = None
    steps_al, steps_cr = [], []
    prev_live = None
    seeded = False
    for i in range(n):
        o = o2b[i]
        if not np.all(np.isfinite(o)):
            continue
        tx, ty, tyaw = true[i]
        if not seeded:
            filt.seed((tx, ty), o, std_m=a.seed_std)
            seeded = True
            prev_stamp = stamps[i]
        pts = scan_to_points(ranges[i], amin, ainc, range_min=rmin, range_max=rmax,
                             lidar_xy=LIDAR_XY, max_use_m=float(params.get('max_use_m', 9.5)), stride=stride)
        dt = max(0.0, float(stamps[i] - prev_stamp))
        prev_stamp = stamps[i]
        out = filt.step(o, dt, pts)
        compute.append(out['compute_ms'])
        rej = REJECT_CODES[int(out['reject'])]
        rejects[rej] = rejects.get(rej, 0) + 1
        mode = MODES[int(out['mode'])]

        x, y, _ = filt.pose(o)
        c, s = math.cos(tyaw), math.sin(tyaw)
        ex, ey = x - tx, y - ty
        al, cr = ex * c + ey * s, -ex * s + ey * c
        moving = speed[i] > a.min_speed
        lx = ly = float('nan')
        if np.all(np.isfinite(m2o[i])):
            cm, sm = math.cos(m2o[i][2]), math.sin(m2o[i][2])
            lx = m2o[i][0] + cm * o[0] - sm * o[1]
            ly = m2o[i][1] + sm * o[0] + cm * o[1]
            lex, ley = lx - tx, ly - ty
            lal, lcr = lex * c + ley * s, -lex * s + ley * c
        else:
            lal = lcr = float('nan')
        if moving:
            v2_err['along'].append(al); v2_err['cross'].append(cr); v2_err['dist'].append(math.hypot(ex, ey))
            live_err['along'].append(lal); live_err['cross'].append(lcr)
            live_err['dist'].append(math.hypot(lx - tx, ly - ty) if np.isfinite(lx) else float('nan'))
            per_mode[mode]['along'].append(al); per_mode[mode]['cross'].append(cr)
            if prev_pub is not None:
                dp = filt.pub - prev_pub
                sa_, sc_ = dp[0] * c + dp[1] * s, -dp[0] * s + dp[1] * c
                walk[mode][0] += abs(sa_); walk[mode][1] += abs(sc_)
                steps_al.append(sa_); steps_cr.append(sc_)
            if prev_live is not None and np.isfinite(lx) and np.all(np.isfinite(prev_live)):
                dl = m2o[i][:2] - prev_live[:2]
                live_walk[mode][0] += abs(dl[0] * c + dl[1] * s); live_walk[mode][1] += abs(-dl[0] * s + dl[1] * c)
        prev_pub = filt.pub.copy()
        prev_live = m2o[i]
        if a.csv:
            rows.append([stamps[i], tx, ty, tyaw, x, y, al, cr, lx, ly, lal, lcr, speed[i]]
                        + [out[k] for k in STATUS_FIELDS])

    print()
    print('              ' + ' ' * 10 + 'v2 (offline, this code)' + ' ' * 26 + '| live localizer (from the log)')
    for k in ('along', 'cross', 'dist'):
        print(f'  err_{k:6s}  {stats(v2_err[k]):56s} | {stats(live_err[k])}')
    print()
    print('  per mode (v2):')
    for m in MODES:
        if per_mode[m]['along']:
            print(f'    {m:14s} n {len(per_mode[m]["along"]):5d}  along {stats(per_mode[m]["along"])}   '
                  f'cross p90 {np.percentile(np.abs(per_mode[m]["cross"]), 90):.3f}')
    print()
    print(f'  per-sample correction step (what a wall margin feels): |along| p90 '
          f'{np.percentile(np.abs(steps_al), 90) * 1000:.1f} mm max {np.max(np.abs(steps_al)) * 1000:.1f} mm   '
          f'|cross| p90 {np.percentile(np.abs(steps_cr), 90) * 1000:.1f} mm max {np.max(np.abs(steps_cr)) * 1000:.1f} mm')
    print()
    print('  correction walk, sum |steps| in the car frame (v2 | live):')
    for m in MODES:
        if per_mode[m]['along']:
            print(f'    {m:14s} along {walk[m][0]:6.2f} m  cross {walk[m][1]:6.2f} m | '
                  f'along {live_walk[m][0]:6.2f} m  cross {live_walk[m][1]:6.2f} m')
    print()
    print(f'  rejections: {rejects}')
    print(f'  compute ms: median {np.median(compute):.2f}  p90 {np.percentile(compute, 90):.2f}  max {max(compute):.2f}')
    print(f'  mode changes: {len(filt.mode_changes)}   final odometry scale estimate: {filt.k_scale*100:+.2f} %')

    if a.csv:
        with open(a.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['stamp', 'true_x', 'true_y', 'true_yaw', 'v2_x', 'v2_y', 'v2_err_along', 'v2_err_cross',
                        'live_x', 'live_y', 'live_err_along', 'live_err_cross', 'speed']
                       + ['v2_' + k for k in STATUS_FIELDS])
            w.writerows(rows)
        print(f'  wrote {len(rows)} rows to {a.csv}')

    # THE GATE. Beat the incumbent on accuracy, and stay smooth. The earlier
    # gate ("no along-track walk on the blind straight") was built on a botched
    # measurement and on the wrong metric: it penalised tracking a real drift
    # smoothly, which is the correct behaviour. What threatens a 0.09-0.13 m
    # wall margin is the per-SAMPLE jump, and AMCL's reaches 0.46 m.
    def p90(v):
        v = np.asarray([x for x in v if np.isfinite(x)], dtype=float)
        return float(np.percentile(np.abs(v), 90)) if len(v) else float('nan')
    v_al, v_cr = p90(v2_err['along']), p90(v2_err['cross'])
    l_al, l_cr = p90(live_err['along']), p90(live_err['cross'])
    step = max((max(abs(a) for a in st) if st else 0.0) for st in (steps_al, steps_cr))
    have_live = np.isfinite(l_al)
    ok = (step <= 0.05) and (not have_live or (v_al <= l_al and v_cr <= l_cr))
    print(f'\nGATE {"PASS" if ok else "FAIL"}: '
          f'along p90 {v_al:.3f} vs live {l_al:.3f}   cross p90 {v_cr:.3f} vs live {l_cr:.3f}   '
          f'worst per-sample step {step * 1000:.1f} mm (<= 50)')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
