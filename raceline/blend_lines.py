#!/usr/bin/env python3
"""Blend two racelines of the same track: B inside chosen s-zones, A elsewhere.

For a fast line (A) that is too close to a wall in a few places, take the safer
line's (B) geometry just there, with cosine ramps so curvature stays smooth, then
re-fit the spline and re-score it with optimize_raceline's own tools. The speed
column is a placeholder (the pipeline's plain profile); run
enforce_friction_ellipse.py on the output for the speeds that are raced.

    python3 blend_lines.py iros2026/rl_mt_b0.05.csv iros2026/rl_mt_b0.15.csv \\
        --zones 25.0:28.0,32.3:34.2,37.3:40.0 --ramp 1.5 -o iros2026/OUT.csv

Zones are in A's arc length. Measured reason for the IROS 2026 zones: see
experiments/iros2026 M06 (b0.05 hit the right wall at s 26.8 with 0.19 m of
tracking error; its right-side slack is 0.01-0.11 m at s 26-27, 33, 38-39).
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from optimize_raceline import (DEFAULT_TRACK, PHYS, ProfileLimits, TrackMap,  # noqa: E402
                               map_base, score_line)


def load(path):
    return np.loadtxt(path, delimiter=',', comments='#')


def weight(s, zones, ramp, lap):
    """1 inside each zone, a cosine ramp to 0 over `ramp` metres outside it."""
    w = np.zeros_like(s)
    for s0, s1 in zones:
        inside = ((s - s0) % lap) <= ((s1 - s0) % lap)
        edge = np.minimum((s0 - s) % lap, (s - s1) % lap)
        ramp_w = np.where(edge < ramp, 0.5 * (1.0 + np.cos(np.pi * edge / ramp)), 0.0)
        w = np.maximum(w, np.where(inside, 1.0, ramp_w))
    return w


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('a'); ap.add_argument('b')
    ap.add_argument('--zones', required=True, help='s0:s1,... in A arc length')
    ap.add_argument('--ramp', type=float, default=1.5)
    ap.add_argument('--weight', type=float, default=1.0, help='how far toward B inside the zones (1 = all the way)')
    ap.add_argument('--track', default=DEFAULT_TRACK)
    ap.add_argument('-o', '--out', required=True)
    a = ap.parse_args()

    A, B = load(a.a), load(a.b)
    sa, xa, ya = A[:, 0], A[:, 1], A[:, 2]
    lap = sa[-1] + np.hypot(xa[0] - xa[-1], ya[0] - ya[-1])
    zones = [tuple(float(v) for v in z.split(':')) for z in a.zones.split(',') if z]
    w = a.weight * weight(sa, zones, a.ramp, lap)
    # Blend the optimiser's OWN columns rather than re-fitting a spline: a re-fit
    # of rl_mt_b0.05 raised total |dkappa| 8.9 -> 12.3 and cost the speed profile
    # 0.37 s. Both lines are smooth and within ~0.1 m of each other, so heading and
    # curvature blend linearly to first order. B is matched to A by nearest point.
    j = np.argmin((xa[:, None] - B[None, :, 1]) ** 2 + (ya[:, None] - B[None, :, 2]) ** 2, axis=1)
    out = A.copy()
    for c in (1, 2, 4, 5, 6):                                  # x, y, kappa, w_right, w_left
        out[:, c] = (1.0 - w) * A[:, c] + w * B[j, c]
    dpsi = np.angle(np.exp(1j * (B[j, 3] - A[:, 3])))          # wrap-safe heading blend
    out[:, 3] = A[:, 3] + w * dpsi
    ds = np.hypot(np.diff(out[:, 1]), np.diff(out[:, 2]))
    out[:, 0] = np.concatenate([[0.0], np.cumsum(ds)])
    moved = np.hypot(out[:, 1] - xa, out[:, 2] - ya)
    np.savetxt(a.out, out, delimiter=',', fmt='%.5f',
               header='s_m,x_m,y_m,psi_rad,kappa_radpm,w_right_m,w_left_m,v_mps', comments='# ')
    tm = TrackMap(map_base(a.track))
    r = score_line(out[:, 1], out[:, 2], tm, ProfileLimits(a_long=6.5, v_max=9.0), PHYS, 'blend')
    ra = score_line(xa, ya, tm, ProfileLimits(a_long=6.5, v_max=9.0), PHYS, 'A')
    print(f'A     : length {ra["length"]:.2f} m, body margin {ra["body_margin"]:+.3f} m')
    print(f'blend : length {r["length"]:.2f} m, body margin {r["body_margin"]:+.3f} m, '
          f'max sideways move {moved.max():.3f} m, |dkappa| sum {np.sum(np.abs(np.diff(out[:, 4]))):.1f} '
          f'(A {np.sum(np.abs(np.diff(A[:, 4]))):.1f}) -> {a.out}')


if __name__ == '__main__':
    main()
