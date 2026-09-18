#!/usr/bin/env python3
"""Summarise raceline CSVs: profile lap time, lateral/longitudinal demand, steering-rate
feasibility, wall room, and the corner table of the first file.

    python3 tools/tuning/line_report.py raceline/iros2026/raceline_tum_iqp.csv [more.csv ...]
"""
import sys
import numpy as np

L, G = 0.324, 9.81


def load(f):
    return np.loadtxt(f, delimiter=',', comments='#')


def summary(f):
    s, x, y, psi, k, wr, wl, v = load(f).T
    ds = np.diff(np.r_[s, s[-1] + (s[1] - s[0])])
    alat = v ** 2 * np.abs(k)
    ax = np.gradient(v ** 2 / 2, s)
    srate = np.abs(np.gradient(k, s)) * v * L / (1 + (k * L) ** 2)
    print(f"{f.split('/')[-1]}: {len(s)} pts, lap {s[-1] + ds[-1]:.2f} m, profile {np.sum(ds / np.maximum(v, 0.1)):.2f} s, "
          f"v {v.min():.2f}-{v.max():.2f}, a_lat max {alat.max():.2f} p90 {np.percentile(alat, 90):.2f}, "
          f"a_x +{ax.max():.2f}/{ax.min():.2f}, steer rate max {srate.max():.2f} rad/s (limit 3.2), "
          f"min room R {wr.min():.2f} L {wl.min():.2f}")


def corners(f, thr=0.35):
    s, x, y, psi, k, wr, wl, v = load(f).T
    n = len(s)
    print(f"\ncorners on {f.split('/')[-1]} (|kappa| > {thr}):")
    m = np.abs(k) > thr
    i = 0
    while i < n:
        if not m[i]:
            i += 1
            continue
        j = i
        while j < n and m[j]:
            j += 1
        ia = i + int(np.argmax(np.abs(k[i:j])))
        left = k[ia] > 0
        inner, outer = (wl[ia], wr[ia]) if left else (wr[ia], wl[ia])
        print(f"  s {s[i]:5.1f}-{s[j - 1]:5.1f} {'L' if left else 'R'}  kappa {abs(k[ia]):.2f} (r {1 / abs(k[ia]):.2f} m, "
              f"steer {np.degrees(np.arctan(abs(k[ia]) * L)):.0f}/30 deg)  apex v {v[ia]:.2f}  a_lat {v[ia] ** 2 * abs(k[ia]):.2f}  "
              f"entry v {v[max(i - 15, 0)]:.2f}  room inner {inner:.2f} outer {outer:.2f}  at ({x[ia]:.2f}, {y[ia]:.2f})")
        i = j
    print("fast sections (v > 6):")
    m = v > 6
    i = 0
    while i < n:
        if not m[i]:
            i += 1
            continue
        j = i
        while j < n and m[j]:
            j += 1
        print(f"  s {s[i]:5.1f}-{s[j - 1]:5.1f}  v max {v[i:j].max():.2f}  min room {min(wr[i:j].min(), wl[i:j].min()):.2f} m")
        i = j


if __name__ == '__main__':
    for f in sys.argv[1:]:
        summary(f)
    corners(sys.argv[1])
