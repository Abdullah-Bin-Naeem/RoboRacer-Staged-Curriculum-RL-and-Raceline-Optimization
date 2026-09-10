#!/usr/bin/env python3
"""Make a line that clips a chosen wall on purpose, to test recovery.

    python tools/make_crash_line.py raceline/icra2026/raceline_a7.0zv_hard_l6.0_corners_h.csv \
        --s 21.0:23.5 --side near --depth 0.05 --out raceline/icra2026/crash_t2.csv

Over the s window the geometry is shifted toward the chosen wall until the car's
body sits `depth` metres INTO it, with 1 m cosine ramps in and out; everything
else, including the speed column, is the source line's. The car then contacts
that wall on every lap at a known place and speed, the simulator resets it to
the last checkpoint, and the bootstrap's recovery has something repeatable to
be measured against (analyze_run.py: resets / recovery lines).

--side near  the wall the line is already closer to in the window (default)
--side left / right   force a side
Body half-width is 0.135 m, as in the pipeline.
"""
import argparse
import numpy as np

HALF = 0.135
p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
p.add_argument('csv')
p.add_argument('--s', required=True, help='s0:s1 window [m]')
p.add_argument('--side', default='near', choices=['near', 'left', 'right'])
p.add_argument('--depth', type=float, default=0.05, help='how far the body goes into the wall [m]')
p.add_argument('--ramp', type=float, default=1.0, help='blend length each side [m]')
p.add_argument('--out', required=True)
a = p.parse_args()
s0, s1 = (float(v) for v in a.s.split(':'))
lines = open(a.csv).read().splitlines()
head = [l for l in lines if l.startswith('#')]
L = np.genfromtxt([l for l in lines if not l.startswith('#')], delimiter=',')
s, x, y, psi, kappa, w_r, w_l, v = (L[:, i] for i in range(8))
win = (s >= s0) & (s <= s1)
if a.side == 'near':
    side = 'left' if w_l[win].mean() < w_r[win].mean() else 'right'
else:
    side = a.side
sign = 1.0 if side == 'left' else -1.0                       # +lateral = left of the line
w = w_l if side == 'left' else w_r
# how far to move so the body is `depth` into that wall, per point in the window
shift = np.zeros_like(s)
shift[win] = (w[win] - HALF + a.depth)
# cosine ramps: 0 at the window edges +- ramp, full inside
def ramp(u):
    return 0.5 - 0.5 * np.cos(np.pi * np.clip(u, 0.0, 1.0))
blend = np.zeros_like(s)
blend[win] = np.minimum(ramp((s[win] - s0) / a.ramp), ramp((s1 - s[win]) / a.ramp))
shift *= blend
nx, ny = x + sign * shift * -np.sin(psi), y + sign * shift * np.cos(psi)
new_wl, new_wr = w_l - sign * shift, w_r + sign * shift
with open(a.out, 'w') as f:
    for h in head:
        f.write(h + '\n')
    f.write(f'# CRASH TEST LINE: {a.csv} shifted into the {side} wall over s {s0}-{s1} by up to '
            f'{shift.max():.2f} m (body {a.depth} m into the wall). For recovery testing only.\n')
    for row in zip(s, nx, ny, psi, kappa, new_wr, new_wl, v):
        f.write(','.join(f'{q:.5f}' for q in row) + '\n')
body = np.minimum(new_wl, new_wr) - HALF
print(f"wrote {a.out}: {side} wall, shift max {shift.max():.2f} m over s {s0}-{s1}; body-to-wall in the window min {body[win].min():+.2f} m "
      f"(negative = contact), elsewhere min {body[~win].min():.2f} m; speed there {v[win].min():.1f}-{v[win].max():.1f} m/s (unchanged)")
