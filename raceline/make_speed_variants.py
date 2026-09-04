#!/usr/bin/env python3
"""Re-profile an existing raceline's GEOMETRY at a ladder of lateral limits.

The line's geometry comes from minimising curvature and does not depend on grip;
only the velocity profile does. So rather than re-solving, this reuses the
geometry and recomputes speeds at a ladder of limits under the simulator's own
physics (optimize_raceline.SimPhysics, from VEHICLE_MODEL.md).

optimize_raceline.py already exports this ladder for the line it produces. Use
this for any OTHER line: a hand-edited CSV, a logged RL trace, an old export.

    python make_speed_variants.py raceline_tum.csv                 # -> raceline_tum_a4.0.csv ...
    python make_speed_variants.py some_line.csv --ladder 3.5,4.0,4.5

Point the follower at each in turn and note where the car first runs wide. That
is the real usable a_lat, measured instead of guessed. Measured so far (see
VEHICLE_MODEL.md §3.2): p90 4.74 m/s² while tracking, the 6.0 rung washed wide
at 5.5, and the tire's asymptote, 4.90, is the physical ceiling.
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from optimize_raceline import (DEFAULT_MAP, PHYS, ProfileLimits, TrackMap,  # noqa: E402
                               export_csv, load_xy, score_line, velocity_profile)


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("csv", type=Path, help="line to re-profile (s,x,y,... layout; only x,y are used)")
    p.add_argument("--ladder", default="4.0,4.5,4.9", help="a_lat rungs [m/s^2]")
    p.add_argument("--a-long", type=float, default=PHYS.a_long_robust0)
    p.add_argument("--v-max", type=float, default=8.0)
    p.add_argument("--map", type=Path, default=DEFAULT_MAP)
    a = p.parse_args()

    tm = TrackMap(a.map)
    x, y = load_xy(a.csv)
    base = score_line(x, y, tm, ProfileLimits(a_long=a.a_long, v_max=a.v_max), PHYS, a.csv.stem)
    print(f"{a.csv.name}: {base['n']} points, {base['length']:.2f} m, |k|max {base['kmax']:.3f}, "
          f"body margin {base['body_margin']:+.3f} m, steering {base['steer_rate_max']:.2f} rad/s "
          f"(limit {PHYS.steer_rate})")
    if base["body_margin"] < 0.0:
        print("WARNING: this geometry puts the car body inside a wall; the profile will not save it")

    print(f"\n{'a_lat':>6}{'lap':>9}{'v max':>7}   file")
    for rung in [float(v) for v in a.ladder.split(",")]:
        lim = ProfileLimits(a_lat=rung, a_long=a.a_long, v_max=a.v_max)
        vx, ax, t = velocity_profile(base["kappa"], base["el"], lim)
        out = export_csv(a.csv.with_name(f"{a.csv.stem}_a{rung:.1f}.csv"), dict(base, vx=vx, ax=ax, t=t), tm)
        print(f"{rung:6.1f}{t:8.3f}s{vx.max():7.2f}   {out.name}")

    print("\nGeometry is identical across all of them; only the speed profile changes.")
    print("Run them in order. The first one the car cannot hold gives you the real grip limit.")


if __name__ == "__main__":
    main()
