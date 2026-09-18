#!/usr/bin/env python3

"""Synthesise a scan-dump .npz from the map and the centreline, no simulator.

    python3 synth_v2_dataset.py OUT.npz [--track iros2026] [--laps 2] [--hz 45]
                                [--speed 6] [--dr-scale 1.02] [--dr-noise 0.003]

A car drives the centreline at `--speed` m/s. Each tick gets a raycast scan
(270 deg, 1081 beams, 10 m, 1 cm range noise) and a dead-reckoned odom->base
pose with a deliberate along-track SCALE error (`--dr-scale`, 1.02 = the raw
encoder's measured 1.018-1.024) plus per-step noise. The IMU heading is exact,
as it is on the car (absolute quaternion). The live map->odom column is NaN:
there is no AMCL here.

The point: tools/replay_localization_v2.py on this file shows, with a known
odometry error and no sensor surprises, whether V2Filter does the two things
the design says -- carries dead reckoning across the blind straight without
touching along-track, and takes the accumulated drift out in the approach,
rate-limited. The regression test for the estimator, as segment_track.py
--bench is for the matcher. It is not a substitute for a recorded run: the
real scan has the duct gaps, the real odometry has the slip model's residual.
"""

import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'racer_common'))
from racer_localization.scan_matcher import LikelihoodField, rebase_repo_path  # noqa: E402

ANGLE_MIN, ANGLE_MAX, N_BEAMS, RANGE_MAX, RANGE_MIN = -2.35619, 2.35619, 1081, 10.0, 0.06
LIDAR_X = 0.2733


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('out')
    ap.add_argument('--track', default=os.environ.get('RACER_TRACK', 'iros2026'))
    ap.add_argument('--laps', type=float, default=2.0)
    ap.add_argument('--hz', type=float, default=45.0)
    ap.add_argument('--speed', type=float, default=6.0)
    ap.add_argument('--dr-scale', type=float, default=1.02, help='dead-reckoned distance / true')
    ap.add_argument('--dr-noise', type=float, default=0.003, help='per-step along-track noise [m]')
    ap.add_argument('--range-noise', type=float, default=0.01)
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()

    from racer_common import frames

    here = rebase_repo_path

    map_yaml = here(frames.map_yaml(a.track))
    cl = np.loadtxt(here(os.path.join(frames.raceline_dir(a.track), 'centerline_full.csv')),
                    delimiter=',', comments='#')
    s, cx, cy, psi = cl[:, 0], cl[:, 1], cl[:, 2], cl[:, 3]
    spawn = tuple(float(v) for v in frames.spawn(a.track))

    # Orient the ring along the lap (same test as the bootstrap), start at the spawn.
    j = int(np.argmin((cx - spawn[0]) ** 2 + (cy - spawn[1]) ** 2))
    k = (j + 3) % len(s)
    if math.cos(math.atan2(cy[k] - cy[j], cx[k] - cx[j]) - spawn[2]) < 0.0:
        cx, cy = cx[::-1], cy[::-1]
        j = len(s) - 1 - j
    cx, cy = np.roll(cx, -j), np.roll(cy, -j)
    seg = np.hypot(np.diff(cx, append=cx[0]), np.diff(cy, append=cy[0]))
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    lap = cum[-1]
    xs = np.concatenate([cx, cx[:1]]); ys = np.concatenate([cy, cy[:1]])

    field = LikelihoodField(map_yaml)
    rng = np.random.default_rng(a.seed)
    angles = np.linspace(ANGLE_MIN, ANGLE_MAX, N_BEAMS)
    dt = 1.0 / a.hz
    n = int(a.laps * lap / a.speed / dt)
    print(f'{a.track}: lap {lap:.1f} m, {n} ticks at {a.hz:g} Hz, {a.speed:g} m/s, dr scale {a.dr_scale}')

    ranges = np.zeros((n, N_BEAMS), dtype=np.float32)
    true = np.zeros((n, 3)); o2b = np.zeros((n, 3)); stamps = np.zeros(n); speed = np.full(n, a.speed)
    dr_x, dr_y = 0.0, 0.0                      # odom frame starts at the spawn's true position
    for i in range(n):
        d = (i * dt * a.speed) % lap
        x = float(np.interp(d, cum, xs)); y = float(np.interp(d, cum, ys))
        d2 = (d + 0.05) % lap
        yaw = math.atan2(float(np.interp(d2, cum, ys)) - y, float(np.interp(d2, cum, xs)) - x)
        lx, ly = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
        r, hit = field.raycast(lx, ly, yaw, angles, RANGE_MAX)
        r = np.where(hit, r + rng.normal(0.0, a.range_noise, N_BEAMS), RANGE_MAX)
        ranges[i] = np.clip(r, RANGE_MIN, RANGE_MAX)
        true[i] = (x, y, yaw)
        stamps[i] = i * dt
        if i > 0:
            step = a.speed * dt * a.dr_scale + rng.normal(0.0, a.dr_noise)
            dr_x += step * math.cos(yaw); dr_y += step * math.sin(yaw)
        else:
            dr_x, dr_y = x, y
        o2b[i] = (dr_x, dr_y, yaw)             # heading exact: the IMU quaternion is absolute
        if i % 500 == 0:
            print(f'  {i}/{n}', end='\r', flush=True)
    print()
    np.savez_compressed(a.out, t=stamps, stamp=stamps, ranges=ranges, o2b=o2b, true=true,
                        m2o=np.full((n, 3), np.nan), speed=speed, ready=np.ones(n, dtype=np.int8),
                        angle_min=ANGLE_MIN, angle_increment=(ANGLE_MAX - ANGLE_MIN) / (N_BEAMS - 1),
                        range_min=RANGE_MIN, range_max=RANGE_MAX, map_yaml=os.path.abspath(map_yaml))
    dr_err = math.hypot(o2b[-1, 0] - true[-1, 0], o2b[-1, 1] - true[-1, 1])
    print(f'wrote {a.out}: dead reckoning alone ends {dr_err:.2f} m off after {a.laps:g} laps')


if __name__ == '__main__':
    main()
