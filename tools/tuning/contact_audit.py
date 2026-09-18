#!/usr/bin/env python3
"""Classify every wall contact in every experiment, and rank the error sources.

    python3 tools/tuning/contact_audit.py experiments/iros2026 [--csv out.csv]

For each run.csv: every respawn (truth jump the speed cannot explain, or a slow
heading jump) is a contact. The 2.0 s before it are read and the contact is put in
exactly one class, checked in this order:

  stall      follower status frozen (pp_s unchanged >= 0.5 s while the car moves)
  loc_along  |along-track localization error| >= 0.30 m at any point in the window
  loc_cross  |cross-track localization error| >= 0.20 m
  speed_est  |v_est - v| >= 0.50 m/s in the window (speed estimate wrong -> braking wrong)
  understeer steering >= 0.6 and achieved curvature < 0.75 x path curvature for >= 0.3 s
  late_turn  heading error <= -15 deg (car rotating behind the path) without the above
  wall_spot  |e_lat| at contact <= 0.30 m and none of the above (the line is too close to a wall)
  other      everything else

It also prints, per run, the lateral error budget on clean driving: truth |e_lat| p90
split into localization (truth - estimate) and control (estimate vs line).
"""
import csv
import glob
import json
import math
import os
import sys

import numpy as np


def load(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        return None
    out = {}
    for k in rows[0]:
        vals = []
        for r in rows:
            try:
                vals.append(float(r[k]))
            except (TypeError, ValueError):
                vals.append(np.nan)
        out[k] = np.array(vals)
    return out


def contacts(d):
    x, y, t, v = d['true_x'], d['true_y'], d['t'], np.nan_to_num(d['speed'])
    yaw = np.radians(d['true_yaw_deg'])
    step = np.hypot(np.diff(x), np.diff(y))
    dt = np.maximum(np.diff(t), 1e-3)
    dyaw = np.abs(np.angle(np.exp(1j * np.diff(yaw))))
    # a respawn zeroes the velocity AND moves the car: both, in one sample
    jump = (step > 0.15) & (np.abs(v[1:]) < 0.8) & (np.abs(v[:-1]) > 1.2)
    idx, last = [], -1e9
    for i in np.flatnonzero(jump) + 1:
        if t[i] - last > 3.0:            # rehits during recovery are not new causes
            idx.append(int(i))
        last = t[i]
    return idx


def classify(d, k):
    t = d['t']
    w = (t >= t[k] - 2.0) & (t < t[k])
    w &= d['ready'] > 0.5 if 'ready' in d else w
    if w.sum() < 5:
        return 'other', dict(s=float('nan'), v=float('nan'), along=float('nan'), cross=float('nan'), vest=float('nan'),
                             steer=float('nan'), head=float('nan'), elat=float('nan'), frozen=0.0, under_s=0.0)
    s = d['pp_s'][w]
    v = d['speed'][w]
    feat = dict(
        s=float(np.nanmedian(d['pp_s'][k - 3:k])) if k >= 3 else float('nan'),
        v=float(np.nanmax(v)),
        along=float(np.nanmax(np.abs(d['err_along'][w]))),
        cross=float(np.nanmax(np.abs(d['err_cross'][w]))),
        vest=float(np.nanmax(np.abs(d['pp_v_est'][w] - v))),
        steer=float(np.nanmax(np.abs(d['pp_steering'][w]))),
        head=float(np.nanmin(np.degrees(d['pp_e_head'][w]))),
        elat=float(d['pp_e_lat'][k - 1]) if k >= 1 else float('nan'),
    )
    # stall: status frozen while the truth moves
    tw, sw = t[w], s
    frozen = 0.0
    for i in range(1, len(sw)):
        if sw[i] == sw[i - 1] and v[i] > 0.5:
            frozen += tw[i] - tw[i - 1]
    k_act = np.abs(d['yaw_rate'][w]) / np.maximum(np.abs(v), 0.3)
    k_path = np.abs(d['pp_kappa'][w])
    under = (np.abs(d['pp_steering'][w]) >= 0.6) & (k_act < 0.75 * k_path) & (k_path > 0.5)
    under_s = float(np.sum(np.diff(tw, prepend=tw[0])[under]))
    feat.update(frozen=frozen, under_s=under_s)
    if frozen >= 0.5:
        c = 'stall'
    elif feat['along'] >= 0.30:
        c = 'loc_along'
    elif feat['cross'] >= 0.20:
        c = 'loc_cross'
    elif feat['vest'] >= 0.50:
        c = 'speed_est'
    elif under_s >= 0.3:
        c = 'understeer'
    elif feat['head'] <= -15:
        c = 'late_turn'
    elif abs(feat['elat']) <= 0.30:
        c = 'wall_spot'
    else:
        c = 'other'
    return c, feat


def where(x, y):
    if y < -14.3:
        return 'hairpin1'
    if y > 3.3 and x < 2.6:
        return 'hairpin2'
    if 3.0 <= x <= 4.4 and 1.4 <= y <= 2.9:
        return 's40_bend'
    if 2.3 <= x <= 3.4 and -4.2 <= y <= -1.6:
        return 'chevron_lane'
    if 3.0 <= x <= 4.4 and -10.6 <= y <= -8.0:
        return 'c2_exit'
    if x < 1.2:
        return 'main_straight'
    return f'({x:.1f},{y:.1f})'


def main():
    root = sys.argv[1]
    rows = []
    budget = []
    for dpath in sorted(glob.glob(os.path.join(root, '[ETV]*'))):
        runf = os.path.join(dpath, 'run.csv')
        if not os.path.exists(runf):
            continue
        d = load(runf)
        if d is None or 'pp_s' not in d:
            continue
        cfg = json.load(open(os.path.join(dpath, 'config.json')))
        tag = cfg.get('verdict_short', '')
        invalid = any(w in tag for w in ('INVALID', 'CONFOUNDED'))
        name = os.path.basename(dpath)
        for k in contacts(d)[:1]:          # first contact only: later ones start from a respawn with broken localization
            c, f = classify(d, k)
            rows.append(dict(run=name, invalid=invalid, cls=c, place=where(d['true_x'][k - 1], d['true_y'][k - 1]), **f))
        # lateral budget on ready, moving, pre-first-contact driving
        ks = contacts(d)
        stop = ks[0] - 20 if ks else len(d['t'])
        m = np.zeros(len(d['t']), bool)
        m[:max(stop, 0)] = True
        m &= (d['ready'] > 0.5) & (d['speed'] > 1.0) & np.isfinite(d['pp_e_lat'])
        if m.sum() > 200:
            budget.append((name, invalid, float(np.nanpercentile(np.abs(d['err_cross'][m]), 90)),
                           float(np.nanpercentile(np.abs(d['err_along'][m]), 90)),
                           float(np.nanpercentile(np.abs(d['pp_e_lat'][m]), 90)),
                           float(np.nanpercentile(np.abs(d['pp_v_est'][m] - d['speed'][m]), 90))))
    print('\nFIRST CONTACT OF EACH RUN (later contacts start from a respawn and are not independent)')
    print(f"{'run':34s} {'class':11s} {'place':14s} {'v':>5s} {'along':>6s} {'cross':>6s} {'vest':>5s} {'steer':>5s} {'head':>5s} {'e_lat':>6s}")
    for r in rows:
        print(f"{r['run'][:34]:34s} {r['cls']:11s} {r['place']:14s} {r['v']:5.2f} {r['along']:6.2f} {r['cross']:6.2f} {r['vest']:5.2f} "
              f"{r['steer']:5.2f} {r['head']:5.0f} {r['elat']:+6.2f}{'  (invalid run)' if r['invalid'] else ''}")
    valid = [r for r in rows if not r['invalid']]
    print(f'\nTOTALS over valid runs ({len(valid)} contacts):')
    from collections import Counter
    for (c, n) in Counter(r['cls'] for r in valid).most_common():
        places = Counter(r['place'] for r in valid if r['cls'] == c)
        print(f'  {c:11s} {n:3d}  ({n / max(len(valid), 1) * 100:.0f} %)   at: ' + ', '.join(f'{p} {q}' for p, q in places.most_common()))
    print('\nBY PLACE:')
    for (p, n) in Counter(r['place'] for r in valid).most_common():
        print(f'  {p:14s} {n:3d}   classes: ' + ', '.join(f'{c} {q}' for c, q in Counter(r["cls"] for r in valid if r["place"] == p).most_common()))
    print('\nERROR BUDGET on clean driving (p90): localization cross / along, controller |e_lat| (on the estimate), speed estimate')
    for b in budget:
        print(f"  {b[0][:34]:34s} loc_cross {b[2]:.2f}  loc_along {b[3]:.2f}  ctrl_e_lat {b[4]:.2f}  v_est {b[5]:.2f}{'  (invalid)' if b[1] else ''}")


if __name__ == '__main__':
    main()
