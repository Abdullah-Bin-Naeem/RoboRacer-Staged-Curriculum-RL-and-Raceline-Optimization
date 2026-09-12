#!/usr/bin/env python3
"""Reproduce HZ_ANALYSIS.md from logs/: the cap sweep of 2026-09-12.

    python3 tools/analyze_caps.py            # all sections
    python3 tools/analyze_caps.py --caps 40,80
    python3 tools/analyze_caps.py --line roboracer_stack/raceline/raceline_a7.0.csv \\
        --csv 'logs/run_default_cap{}.csv' --logs 40=racer_<stamp>.log,45=... --caps 40,45

Inputs: logs/run_cap<hz>.csv (log_localization, 20 Hz), the stack logs mapped
in LOGS below (lap times), the raceline the runs followed, and the map. Needs
numpy and scipy. Launch lap excluded everywhere (t > 25 s from the first row).
"""
import csv
import re
import sys

import numpy as np

CAPS = [20, 40, 45, 50, 60, 70, 80]
LOGS = {20: 'racer_20260912_002127.log', 40: 'racer_20260912_001751.log', 45: 'racer_20260911_235859.log',
        50: 'racer_20260912_000256.log', 60: 'racer_20260912_000643.log', 70: 'racer_20260912_001019.log',
        80: 'racer_20260912_001408.log'}
LINE = 'roboracer_stack/raceline/raceline_a7.0_rec_6.36.csv'
MAP_PGM, MAP_YAML = 'roboracer_stack/maps/track_clean.pgm', 'roboracer_stack/maps/track_clean.yaml'
FOOT_L, FOOT_W = 0.50, 0.28          # assumed footprint, centred on the IPS point
T_SKIP = 25.0                        # launch lap

CSV_PATTERN = 'logs/run_cap{}.csv'
if '--caps' in sys.argv:
    CAPS = [int(c) for c in sys.argv[sys.argv.index('--caps') + 1].split(',')]
if '--line' in sys.argv:                     # another raceline (the runs must have followed it)
    LINE = sys.argv[sys.argv.index('--line') + 1]
if '--csv' in sys.argv:                      # e.g. 'logs/run_default_cap{}.csv'
    CSV_PATTERN = sys.argv[sys.argv.index('--csv') + 1]
if '--logs' in sys.argv:                     # e.g. 40=racer_...log,45=racer_...log
    LOGS = {int(k): v for k, v in (kv.split('=') for kv in sys.argv[sys.argv.index('--logs') + 1].split(','))}

raw = np.loadtxt(LINE, delimiter=',', comments='#')
s_l, x_l, y_l, psi_l, k_l, wr_l, wl_l, v_l = raw.T[:8]


def nearest(x, y):
    return ((x_l[None, :] - x[:, None]) ** 2 + (y_l[None, :] - y[:, None]) ** 2).argmin(axis=1)


def corners(thr=0.45):
    on = np.abs(k_l) >= thr
    segs, i = [], 0
    while i < len(on):
        if on[i]:
            j = i
            while j + 1 < len(on) and on[j + 1]:
                j += 1
            segs.append((s_l[i], s_l[j], np.sign(k_l[i:j + 1].mean()), np.abs(k_l[i:j + 1]).max(), v_l[i:j + 1].min()))
            i = j + 1
        else:
            i += 1
    return segs


def load(cap):
    rows = list(csv.DictReader(open(CSV_PATTERN.format(cap))))
    col = lambda k: np.array([float(r[k]) if r[k] not in ('', 'nan') else np.nan for r in rows])
    d = {k: col(k) for k in ('t', 'true_x', 'true_y', 'true_yaw_deg', 'speed', 'pp_v_est', 'pp_v_enc', 'pp_v_pose',
                             'pp_v_target', 'pp_u_cmd', 'pp_throttle', 'pp_steering', 'pp_e_lat', 'pp_slip',
                             'pp_a_imu', 'pp_delay')}
    d['t'] = d['t'] - d['t'][0]
    d['mov'] = (d['speed'] > 1.0) & (d['t'] > T_SKIP)
    idx = nearest(d['true_x'], d['true_y'])
    dx, dy = d['true_x'] - x_l[idx], d['true_y'] - y_l[idx]
    d['e_true'] = -dx * np.sin(psi_l[idx]) + dy * np.cos(psi_l[idx])      # left positive
    d['inside'] = d['e_true'] * np.sign(k_l[idx])                          # + cutting inside
    d['idx'], d['s'], d['vplan'], d['kap'] = idx, s_l[idx], v_l[idx], k_l[idx]
    d['wr'], d['wl'] = wr_l[idx], wl_l[idx]
    return d


def laps(cap):
    txt = open('logs/' + LOGS[cap]).read()
    v = [float(x) for x in re.findall(r'lap \d+: ([0-9.]+) s', txt)][1:]   # drop the launch lap
    return v, len(re.findall(r'respawn', txt))


def dist_field():
    from scipy import ndimage
    with open(MAP_PGM, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split()); f.readline()
        img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    y = dict(l.split(':', 1) for l in open(MAP_YAML) if ':' in l)
    res = float(y['resolution']); ox, oy = [float(v) for v in y['origin'].strip(' []\n').split(',')[:2]]
    occ = img < int(255 * (1 - float(y['occupied_thresh'])))
    dist = ndimage.distance_transform_edt(~occ) * res

    def at(x, yy):
        col = ((x - ox) / res).astype(int); row = (h - 1 - (yy - oy) / res).astype(int)
        ok = (row >= 0) & (row < h) & (col >= 0) & (col < w)
        out = np.full(len(x), np.nan); out[ok] = dist[row[ok], col[ok]]
        return out
    return at


def main():
    q = lambda a, p: np.nanpercentile(a, p)
    segs = corners()
    print('corners (|kappa| >= 0.45): ' + '  '.join(
        f"C{n + 1}[s {a:.1f}-{b:.1f} {'L' if sg > 0 else 'R'} k{km:.2f} vmin {vm:.2f}]" for n, (a, b, sg, km, vm) in enumerate(segs)))
    D = {cap: load(cap) for cap in CAPS}
    at = dist_field()
    pts = [(a, b) for a in (-FOOT_L / 2, 0, FOOT_L / 2) for b in (-FOOT_W / 2, 0, FOOT_W / 2)]

    print('\n== laps, tracking, clearance, speed ==')
    print(f"{'cap':>3s} {'laps':>4s} {'best':>5s} {'mean':>6s} {'sd':>5s} {'resp':>4s} {'|e_true| p95':>12s} "
          f"{'body min':>8s} {'body p5':>7s} {'at s':>5s} {'straight v-plan':>15s} {'accel':>6s} {'max v':>6s}")
    for cap in CAPS:
        d, m = D[cap], D[cap]['mov']
        lv, resp = laps(cap)
        x, yy, yaw = d['true_x'][m], d['true_y'][m], np.radians(d['true_yaw_deg'][m])
        body = np.full(m.sum(), np.inf)
        for a, b in pts:
            body = np.minimum(body, at(x + a * np.cos(yaw) - b * np.sin(yaw), yy + a * np.sin(yaw) + b * np.cos(yaw)))
        st = np.abs(d['kap']) < 0.15
        acc = np.gradient(d['speed'], d['t'])
        ma = m & st & (acc > 0.5)
        print(f"{cap:3d} {len(lv):4d} {min(lv):5.2f} {np.mean(lv):6.3f} {np.std(lv):5.3f} {resp:4d} {q(np.abs(d['e_true'][m]), 95):12.3f} "
              f"{np.nanmin(body):8.3f} {q(body, 5):7.3f} {d['s'][m][np.nanargmin(body)]:5.1f} "
              f"{np.nanmedian((d['speed'] - d['vplan'])[m & st]):15.3f} {np.nanmedian(acc[ma]):6.2f} {np.nanmax(d['speed'][m]):6.2f}")
    print(f"(the line itself passes {np.nanmin(at(x_l, y_l)):.3f} m from the nearest wall at s {s_l[np.nanargmin(at(x_l, y_l))]:.1f})")

    print('\n== per corner: mean INSIDE offset (+ cutting / - wide), widest excursion, speed - plan ==')
    print(f"{'cap':>3s} " + ' '.join(f"{'C' + str(n + 1):>26s}" for n in range(len(segs))))
    for cap in CAPS:
        d, m = D[cap], D[cap]['mov']; out = []
        for (a, b, sg, km, vm) in segs:
            mm = m & (d['s'] >= a) & (d['s'] <= b)
            out.append(f"{d['inside'][mm].mean():+.3f} min {d['inside'][mm].min():+.3f} v {(d['speed'] - d['vplan'])[mm].mean():+.2f}" if mm.any() else 'nan')
        print(f"{cap:3d} " + ' '.join(f"{o:>26s}" for o in out))

    print('\n== adaptive delay (pp_delay): median per successive 20 s window ==')
    for cap in CAPS:
        d = D[cap]; mv = d['speed'] > 1.0
        w = [np.nanmedian(d['pp_delay'][mv & (d['t'] >= a) & (d['t'] < a + 20)]) for a in range(0, int(d['t'].max()), 20)]
        print(f"{cap:3d}: " + ' '.join(f"{v:.3f}" for v in w))

    print('\n== straights, accelerating (true accel > 0.5): estimator bias and slip-band use ==')
    print(f"{'cap':>3s} {'n':>5s} {'v_est-true':>10s} {'v_enc-true':>10s} {'thr':>6s} {'u_cmd-v_est':>11s} {'band edge':>9s} {'at edge %':>9s} {'v_pose valid %':>14s}")
    for cap in CAPS:
        d, m = D[cap], D[cap]['mov']; st = np.abs(d['kap']) < 0.15
        acc = np.gradient(d['speed'], d['t']); ma = m & st & (acc > 0.5)
        band = (d['pp_u_cmd'] - d['pp_v_est'])[ma]; edge = 0.08 * d['pp_v_est'][ma]
        print(f"{cap:3d} {ma.sum():5d} {np.nanmedian((d['pp_v_est'] - d['speed'])[ma]):10.3f} {np.nanmedian((d['pp_v_enc'] - d['speed'])[ma]):10.3f} "
              f"{np.nanmedian(d['pp_throttle'][ma]):6.3f} {np.nanmedian(band):11.3f} {np.nanmedian(edge):9.3f} {np.mean(band >= edge - 0.05) * 100:9.0f} "
              f"{np.mean(~np.isnan(d['pp_v_pose'])) * 100:14.0f}")


if __name__ == '__main__':
    main()
