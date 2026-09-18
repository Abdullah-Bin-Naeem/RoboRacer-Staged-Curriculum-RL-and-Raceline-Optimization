#!/usr/bin/env python3
"""Re-solve a raceline's speed profile under the COMBINED friction limit.

make_speed_variants.py solves its base profile through tph with a friction
ellipse, then `enforce_long` re-imposes the accel/brake budgets as flat limits
with no ellipse at all, and tph itself drops drag where the ellipse binds. The
result asks for full longitudinal acceleration while the tire is still turning
at its lateral limit: on rl_mt_b0.15_L65_a7.0b.csv the hairpin-1 exit plans
5.7 m/s^2 lateral with 5.8 m/s^2 gross longitudinal, 121 % of the ellipse, and
37 of 435 points exceed it. The car cannot do that on an all-wheel-driven
chassis -- drive slip on the fronts spends their lateral grip -- and runs wide
out of hairpin 1 (M02-M05, 2026-09-18).

This keeps the geometry and every column but v_mps, and re-solves the speeds:

    lateral cap   v_i <= sqrt(a_lat / |kappa_i|),  v_i <= v_max
    forward       v_{i+1}^2 <= v_i^2 + 2 ds (a_long * r_i - DRAG * v_i)
    backward      v_i^2     <= v_{i+1}^2 + 2 ds (a_brake * r_i + DRAG * v_i)
    r_i = (1 - (v_i^2 |kappa_i| / a_lat)^p)^(1/p)       (p = 2: the ellipse)

a_long / a_brake are GROSS tire budgets (drag is added back separately), the
same meaning as make_speed_variants' --a-long / --a-brake. The passes run
around the closed lap until nothing changes. Only ever lowers speeds.

    python3 enforce_friction_ellipse.py iros2026/rl_mt_b0.15_L65_a7.0b.csv \\
        --a-lat 7.0 --a-long 6.5 --a-brake 5.5 --v-max 8.5 -o iros2026/OUT.csv
"""
import argparse
import math

import numpy as np

DRAG_LIN = 0.273


def load(path):
    header = [l for l in open(path) if l.startswith('#')]
    a = np.loadtxt(path, delimiter=',', comments='#')
    return header, a


def parse_lat_zones(spec):
    """'s0:s1:a,...' -> list of (s0, s1, a_lat). Empty string -> []."""
    out = []
    for z in (spec or '').split(','):
        z = z.strip()
        if not z:
            continue
        s0, s1, a = (float(x) for x in z.split(':'))
        out.append((s0, s1, a))
    return out


def a_lat_on_s(s, a_lat, lat_zones):
    alat = np.full(len(s), float(a_lat))
    for s0, s1, a in lat_zones:
        alat[(s >= s0) & (s <= s1)] = a
    return alat


def solve(s, kappa, a_lat, a_long, a_brake, v_max, p=2.0, v_seed=None, iters=50,
          lat_zones=()):
    n = len(s)
    lap = s[-1] + (s[-1] - s[-2])
    ds = np.diff(np.append(s, lap))                       # ds[i]: i -> i+1 (wraps)
    k = np.abs(kappa)
    alat = a_lat_on_s(s, a_lat, lat_zones)
    v_cap = np.minimum(v_max, np.sqrt(alat / np.maximum(k, 1e-6)))
    v = v_cap.copy() if v_seed is None else np.minimum(v_seed, v_cap)

    def room(vi, ki, ai):
        u = min(1.0, (vi * vi * ki) / ai)
        return (1.0 - u ** p) ** (1.0 / p)

    for _ in range(iters):
        before = v.copy()
        # Each step is limited by the MORE loaded of its two ends: the lateral
        # load at the far end depends on the speed being solved for, so iterate.
        for _pass in range(2):                            # forward, wrapping twice
            for i in range(n):
                j = (i + 1) % n
                vj = v[j]
                for _ in range(4):
                    a = a_long * min(room(v[i], k[i], alat[i]),
                                     room(vj, k[j], alat[j])) - DRAG_LIN * v[i]
                    vj = min(v[j], math.sqrt(max(v[i] ** 2 + 2.0 * ds[i] * a, 0.0)))
                v[j] = vj
        for _pass in range(2):                            # backward
            for i in range(n - 1, -1, -1):
                j = (i + 1) % n
                vi = v[i]
                for _ in range(4):
                    a = a_brake * min(room(vi, k[i], alat[i]),
                                      room(v[j], k[j], alat[j])) + DRAG_LIN * v[j]
                    vi = min(v[i], math.sqrt(v[j] ** 2 + 2.0 * ds[i] * a))
                v[i] = vi
        if np.max(np.abs(v - before)) < 1e-6:
            break
    t = float(np.sum(2.0 * ds / (v + np.roll(v, -1))))
    return v, t


def usage(s, kappa, v, a_lat, a_long, a_brake):
    ds = np.diff(np.append(s, s[-1] + s[-1] - s[-2]))
    vn, kn = np.roll(v, -1), np.roll(np.abs(kappa), -1)
    ax = (vn ** 2 - v ** 2) / (2.0 * ds) + DRAG_LIN * v      # per step, gross
    ay = np.maximum(v ** 2 * np.abs(kappa), vn ** 2 * kn)    # more loaded end
    return np.sqrt(np.where(ax > 0, ax / a_long, ax / a_brake) ** 2 + (ay / a_lat) ** 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('csv')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--a-lat', type=float, default=7.0)
    ap.add_argument('--a-long', type=float, default=6.5)
    ap.add_argument('--a-brake', type=float, default=5.5)
    ap.add_argument('--v-max', type=float, default=8.5)
    ap.add_argument('--p', type=float, default=2.0, help='ellipse exponent (2 = ellipse)')
    ap.add_argument('--lat-zones', default='',
                    help='s0:s1:a_lat,... overrides --a-lat inside those s-ranges')
    a = ap.parse_args()

    header, arr = load(a.csv)
    s, kappa, v_in = arr[:, 0], arr[:, 4], arr[:, 7]
    zones = parse_lat_zones(a.lat_zones)
    lap_in = float(np.sum(np.diff(np.append(s, s[-1] + s[-1] - s[-2])) * 2.0 / (v_in + np.roll(v_in, -1))))
    v, t = solve(s, kappa, a.a_lat, a.a_long, a.a_brake, a.v_max, a.p, lat_zones=zones)
    u_in = usage(s, kappa, v_in, a.a_lat, a.a_long, a.a_brake)
    u_out = usage(s, kappa, v, a.a_lat, a.a_long, a.a_brake)
    out = arr.copy()
    out[:, 7] = v
    note = (f'# ellipse a_lat {a.a_lat} lat_zones [{a.lat_zones}] '
            f'a_long {a.a_long} a_brake {a.a_brake} v_max {a.v_max} -> {t:.3f} s\n')
    with open(a.out, 'w') as f:
        if header:
            f.writelines(header)
        else:
            f.write('# s_m,x_m,y_m,psi_rad,kappa_radpm,w_right_m,w_left_m,v_mps\n')
        f.write(note)
        np.savetxt(f, out, delimiter=',', fmt='%.5f')
    print(f'{a.csv}: lap {lap_in:.3f} s, ellipse usage max {u_in.max():.2f} ({np.sum(u_in > 1.02)} pts > 1.02)')
    print(f'{a.out}: lap {t:.3f} s, ellipse usage max {u_out.max():.2f} ({np.sum(u_out > 1.02)} pts > 1.02), '
          f'v min {v.min():.2f} max {v.max():.2f}, max drop {np.max(v_in - v):.2f} m/s at s {s[np.argmax(v_in - v)]:.2f}')
    if zones:
        for s0, s1, az in zones:
            m = (s >= s0) & (s <= s1)
            if not np.any(m):
                continue
            i = np.argmax(np.abs(kappa[m]))
            ss, kk, vv = s[m][i], kappa[m][i], v[m][i]
            print(f'  zone {s0:.1f}-{s1:.1f} a_lat {az}: apex s {ss:.2f} k {kk:.2f} v {vv:.2f}')


if __name__ == '__main__':
    main()
