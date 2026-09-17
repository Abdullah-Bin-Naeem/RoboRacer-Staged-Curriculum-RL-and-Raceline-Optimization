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
import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from optimize_raceline import (DEFAULT_TRACK, PHYS, ProfileLimits, TrackMap,  # noqa: E402
                               enforce_long, export_csv, load_xy, map_base, score_line, velocity_profile)


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("csv", type=Path, help="line to re-profile (s,x,y,... layout; only x,y are used)")
    p.add_argument("--ladder", default="4.0,4.5,4.9", help="a_lat rungs [m/s^2]")
    p.add_argument("--a-long", type=float, default=PHYS.a_long_robust0)
    p.add_argument("--v-max", type=float, default=10.0,
                   help="speed ceiling; measured 2026-09: the car is still accelerating at 9.5 m/s, the straight is the limit")
    p.add_argument("--a-brake", type=float, default=None,
                   help="braking budget [m/s^2] when it differs from --a-long; measured 2026-09 with throttle 0 (wheel lock): "
                        "5.5 + 0.3 v, so 5.5 is conservative at every speed")
    p.add_argument("--track", default=DEFAULT_TRACK,
                   help="track whose map to score against (maps/<track>/track_solid if present, else track_clean)")
    p.add_argument("--map", type=Path, default=None, help="map base path, no extension; overrides --track")
    p.add_argument("--lat-zones", default="",
                   help="per-segment lateral limit on top of each rung, 's0:s1:a_lat[,...]' in metres along "
                        "the line (same as optimize_raceline); e.g. '3:9:5.0,27:33:5.0' holds both hairpins "
                        "at 5.0 while the rest runs the rung. Files get a 'z' suffix.")
    a = p.parse_args()
    zones = [tuple(float(v) for v in z.split(":")) for z in a.lat_zones.split(",") if z]

    tm = TrackMap(a.map if a.map is not None else map_base(a.track))
    x, y = load_xy(a.csv)
    base = score_line(x, y, tm, ProfileLimits(a_long=a.a_long, v_max=a.v_max), PHYS, a.csv.stem)
    print(f"{a.csv.name}: {base['n']} points, {base['length']:.2f} m, |k|max {base['kmax']:.3f}, "
          f"body margin {base['body_margin']:+.3f} m, steering {base['steer_rate_max']:.2f} rad/s "
          f"(limit {PHYS.steer_rate})")
    if base["body_margin"] < 0.0:
        print("WARNING: this geometry puts the car body inside a wall; the profile will not save it")

    print(f"\n{'a_lat':>6}{'lap':>9}{'v max':>7}   file")
    for rung in [float(v) for v in a.ladder.split(",")]:
        # with a separate brake budget the base profile is solved at the LARGER of the two (a sweep can only
        # lower speeds), then the accel side is brought back down to --a-long by enforce_long
        a_base = max(a.a_long, a.a_brake) if a.a_brake is not None else a.a_long
        lim = ProfileLimits(a_lat=rung, a_long=a_base, v_max=a.v_max)
        mu = None
        if zones:
            s_line = np.concatenate([[0.0], np.cumsum(base["el"])[:-1]])
            mu = np.ones(len(s_line))
            for s0, s1, a_zone in zones:
                # s0 > s1 wraps past the start line (40:2 covers s 40..lap and 0..2); the
                # plain and-mask was empty there, so the hairpin-2 zone never applied (runs 12-16)
                inz = ((s_line >= s0) & (s_line <= s1)) if s0 <= s1 else ((s_line >= s0) | (s_line <= s1))
                mu[inz] = a_zone / rung
        vx, ax, t = velocity_profile(base["kappa"], base["el"], lim, mu=mu)
        if a.a_brake is not None and abs(a.a_brake - a.a_long) > 1e-9:
            # same as optimize_raceline: backward sweep with the separate brake budget, then re-derive ax and t
            vx = enforce_long(vx, base["el"], a.a_long, a.a_brake)
            ax = np.gradient(vx ** 2) / (2.0 * np.maximum(base["el"], 1e-6))
            t = float(np.sum(2.0 * base["el"] / (vx + np.roll(vx, -1))))
        suffix = ("z" if zones else "") + ("b" if a.a_brake is not None else "")
        out = export_csv(a.csv.with_name(f"{a.csv.stem}_a{rung:.1f}{suffix}.csv"), dict(base, vx=vx, ax=ax, t=t), tm)
        print(f"{rung:6.1f}{t:8.3f}s{vx.max():7.2f}   {out.name}" + (f"   lat {a.lat_zones}" if zones else "") + (f"   brake {a.a_brake}" if a.a_brake is not None else ""))

    print("\nGeometry is identical across all of them; only the speed profile changes.")
    print("Run them in order. The first one the car cannot hold gives you the real grip limit.")


if __name__ == "__main__":
    main()
