#!/usr/bin/env python3
"""Re-profile a raceline's speed on its own geometry, and price variants.

    python3 tools/tuning/profile.py raceline/iros2026/raceline_tum_iqp.csv --a-lat 7.0 --a-acc 4.5 --a-brake 4.5
    python3 tools/tuning/profile.py LINE --a-lat 7.0 --lat-zones 4:7.5:6.0,27:31:6.0 --a-acc 6 --a-brake 5 \
        --v-max 8 --v-zones 33:43.3:9 -o raceline/iros2026/raceline_tum_iqp_X.csv
    python3 tools/tuning/profile.py LINE --sweep            # the lap-time table for the tuning plan

Model (raceline/VEHICLE_MODEL.md section 3): lateral limit a_lat per point (zones override);
longitudinal budgets on a friction ellipse (exponent 2) against the lateral use;
acceleration additionally capped by the tire's robust force minus linear drag,
(min(a_acc, 7.06) - 0.273 v) * ellipse, the budget gross of drag as
make_raceline.py plans it (agrees with the exported tum_iqp lines to ~0.15 s); braking a_brake * ellipse (drag helps, not credited).
Forward/backward passes on the closed loop, three sweeps so the seam converges.
Geometry (x, y, psi, kappa, widths) is copied unchanged; only v_mps is rewritten.
"""
import argparse

import numpy as np

DRAG, TIRE_PEAK = 0.273, 7.06


def zones(spec):
    out = []
    for z in (spec or '').split(','):
        if z:
            a, b, v = (float(f) for f in z.split(':'))
            out.append((a, b, v))
    return out


def solve(s, k, L, a_lat, a_acc, a_brake, v_max, lat_zones=(), v_zones=(), hairpin=None):
    n = len(s)
    alat = np.full(n, a_lat)
    if hairpin:                                             # (kappa threshold, a_lat, padding m)
        kth, ah, pad = hairpin
        for i in np.flatnonzero(np.abs(k) > kth):
            alat[np.abs(((s - s[i]) + L / 2) % L - L / 2) <= pad] = np.minimum(alat[np.abs(((s - s[i]) + L / 2) % L - L / 2) <= pad], ah)
    for a, b, v in lat_zones:
        alat[(s >= a) & (s <= b)] = v
    vcap = np.full(n, v_max)
    for a, b, v in v_zones:
        vcap[(s >= a) & (s <= b)] = v
    v = np.minimum(np.sqrt(alat / np.maximum(np.abs(k), 1e-4)), vcap)
    ds = np.diff(np.r_[s, L])
    for _ in range(3):
        for i in range(2 * n):                              # forward: acceleration
            a, b = i % n, (i + 1) % n
            ell = np.sqrt(max(0.0, 1.0 - (v[a] ** 2 * abs(k[a]) / alat[a]) ** 2))
            acc = (min(a_acc, TIRE_PEAK) - DRAG * v[a]) * ell      # budget is gross of drag, as make_raceline.py plans it
            v[b] = min(v[b], np.sqrt(v[a] ** 2 + 2.0 * max(acc, 0.0) * ds[a]))
        for i in range(2 * n, 0, -1):                       # backward: braking
            a, b = (i - 1) % n, i % n
            ell = np.sqrt(max(0.0, 1.0 - (v[b] ** 2 * abs(k[b]) / alat[b]) ** 2))
            v[a] = min(v[a], np.sqrt(v[b] ** 2 + 2.0 * a_brake * ell * ds[a]))
    return v, float(np.sum(ds / np.maximum(v, 0.1)))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('line')
    p.add_argument('--a-lat', type=float, default=7.0)
    p.add_argument('--lat-zones', default='')
    p.add_argument('--a-acc', type=float, default=4.5)
    p.add_argument('--a-brake', type=float, default=4.5)
    p.add_argument('--v-max', type=float, default=8.0)
    p.add_argument('--v-zones', default='')
    p.add_argument('--hairpin', default='', help="'KAPPA:A_LAT[:PAD_M]': cap a_lat where |kappa| > KAPPA (+-PAD, default 1.0 m); "
                   "position-free, so it follows the corners of any geometry")
    p.add_argument('-o', '--out')
    p.add_argument('--sweep', action='store_true')
    a = p.parse_args()
    data = np.loadtxt(a.line, delimiter=',', comments='#')
    s, k = data[:, 0], data[:, 4]
    L = float(s[-1] + (s[1] - s[0]))
    if a.sweep:
        hp = '4:7.5:{0},27:31:{0}'                           # both hairpins (C1, C4) on this geometry
        straight = '31.5:43.3:{0},0:2:{0}'
        rows = [
            ('tum_iqp as exported (reference)', None),
            ('everything 7.0, acc 4.5 / brake 4.5, 8 m/s', dict(a_lat=7.0, a_acc=4.5, a_brake=4.5, v_max=8)),
            ('hairpins 6.0, rest 7.0, acc 4.5 / brake 4.5 (= a7.0z class)', dict(a_lat=7.0, lz=hp.format(6.0), a_acc=4.5, a_brake=4.5, v_max=8)),
            ('hairpins 6.0, rest 7.0, acc 5.5 / brake 5.0', dict(a_lat=7.0, lz=hp.format(6.0), a_acc=5.5, a_brake=5.0, v_max=8)),
            ('hairpins 6.0, rest 7.0, acc 6.0 / brake 5.0', dict(a_lat=7.0, lz=hp.format(6.0), a_acc=6.0, a_brake=5.0, v_max=8)),
            ('hairpins 6.0, rest 7.0, acc 6.0 / brake 5.0, straight 9', dict(a_lat=7.0, lz=hp.format(6.0), a_acc=6.0, a_brake=5.0, v_max=8, vz=straight.format(9.0))),
            ('hairpins 6.5, rest 7.0, acc 6.0 / brake 5.0, straight 9', dict(a_lat=7.0, lz=hp.format(6.5), a_acc=6.0, a_brake=5.0, v_max=8, vz=straight.format(9.0))),
            ('hairpins 6.5, rest 7.0, acc 6.0 / brake 5.5, straight 9', dict(a_lat=7.0, lz=hp.format(6.5), a_acc=6.0, a_brake=5.5, v_max=8, vz=straight.format(9.0))),
            ('hairpins 7.0, rest 7.0, acc 6.0 / brake 5.0, straight 9', dict(a_lat=7.0, a_acc=6.0, a_brake=5.0, v_max=8, vz=straight.format(9.0))),
            ('hairpins 5.5, rest 7.0, acc 6.0 / brake 5.0, straight 9', dict(a_lat=7.0, lz=hp.format(5.5), a_acc=6.0, a_brake=5.0, v_max=8, vz=straight.format(9.0))),
        ]
        print(f"{'variant':62s} profile lap   hairpin apex v")
        for name, c in rows:
            if c is None:
                v = data[:, 7]
                T = float(np.sum(np.diff(np.r_[s, L]) / np.maximum(v, 0.1)))
            else:
                v, T = solve(s, k, L, c['a_lat'], c['a_acc'], c['a_brake'], c['v_max'],
                             zones(c.get('lz')), zones(c.get('vz')))
            ia = np.argmax(np.abs(k) * (s < 10))
            print(f'{name:62s} {T:7.2f} s     {v[ia]:.2f} m/s')
        return
    hp = None
    if a.hairpin:
        f = [float(x) for x in a.hairpin.split(':')]
        hp = (f[0], f[1], f[2] if len(f) > 2 else 1.0)
    v, T = solve(s, k, L, a.a_lat, a.a_acc, a.a_brake, a.v_max, zones(a.lat_zones), zones(a.v_zones), hp)
    print(f'profile lap {T:.3f} s, v {v.min():.2f}-{v.max():.2f}, a_lat max {np.max(v ** 2 * np.abs(k)):.2f}')
    if a.out:
        out = data.copy()
        out[:, 7] = v
        hdr = (f's_m,x_m,y_m,psi_rad,kappa_radpm,w_right_m,w_left_m,v_mps\n'
               f'profile.py from {a.line}: a_lat {a.a_lat} zones [{a.lat_zones}] acc {a.a_acc} brake {a.a_brake} '
               f'v_max {a.v_max} v_zones [{a.v_zones}] hairpin [{a.hairpin}] -> {T:.3f} s')
        np.savetxt(a.out, out, delimiter=',', fmt='%.5f', header=hdr, comments='# ')
        print('wrote', a.out)


if __name__ == '__main__':
    main()
