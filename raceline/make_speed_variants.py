#!/usr/bin/env python3

"""Re-profile an existing raceline at several aggression levels.

The line's GEOMETRY comes from minimising curvature and does not depend on grip
at all -- only the velocity profile does. So rather than re-solving, this reuses
the geometry and recomputes speeds at a ladder of limits.

Point the follower at each in turn and note where the car first runs wide. That
is your real a_lat, measured through lap testing instead of guessed.

    python make_speed_variants.py                    # default ladder
    python make_speed_variants.py raceline_tum.csv   # a different base line
"""

import sys
from pathlib import Path

import numpy as np
import trajectory_planning_helpers as tph
import yaml
from scipy import ndimage

HERE = Path(__file__).parent
MAP = Path.home() / "Documents/roboracer/devkit_ws/src/racer_mapping/maps/track_clean"

MASS = 3.906
DRAG_COEFF = 0.05
DYN_MODEL_EXP = 2.0          # traction ellipse
VEHICLE_WIDTH = 0.27

# (a_lat, a_long, v_max) -- climb until the car cannot hold the line.
LADDER = [
    (6.0, 5.0, 8.0),
    (8.0, 6.5, 10.0),
    (10.0, 8.0, 12.0),
    (12.0, 9.5, 14.0),
    (15.0, 11.0, 16.0),
]

base_name = sys.argv[1] if len(sys.argv) > 1 else "raceline_scipy.csv"
src = HERE / base_name

# --- map, for the clearance check -------------------------------------------
meta = yaml.safe_load(open(MAP.with_suffix(".yaml")))
res_m, (ox, oy, _) = meta["resolution"], meta["origin"]
with open(MAP.with_suffix(".pgm"), "rb") as f:
    f.readline()
    Wm, Hm = map(int, f.readline().split())
    f.readline()
    img = np.frombuffer(f.read(Wm * Hm), dtype=np.uint8).reshape(Hm, Wm)
DIST = ndimage.distance_transform_edt(img >= 254) * res_m


def margin_of(x, y):
    """Worst clearance minus half the car width. Negative means it hits a wall."""
    c = np.clip(((x - ox) / res_m - 0.5).round().astype(int), 0, Wm - 1)
    r = np.clip((Hm - (y - oy) / res_m - 0.5).round().astype(int), 0, Hm - 1)
    return float(DIST[r, c].min() - VEHICLE_WIDTH / 2)


# --- geometry, reused unchanged ---------------------------------------------
D = np.loadtxt(src, delimiter=",")
x, y, w_right, w_left = D[:, 1], D[:, 2], D[:, 5], D[:, 6]

closed = np.vstack([np.column_stack([x, y]), [x[0], y[0]]])
coeffs_x, coeffs_y, _, _ = tph.calc_splines.calc_splines(path=closed)
el = tph.calc_spline_lengths.calc_spline_lengths(coeffs_x=coeffs_x, coeffs_y=coeffs_y)
_, kappa = tph.calc_head_curv_an.calc_head_curv_an(
    coeffs_x=coeffs_x, coeffs_y=coeffs_y, ind_spls=np.arange(len(coeffs_x)),
    t_spls=np.zeros(len(coeffs_x)), calc_curv=True)

s = np.concatenate([[0.0], np.cumsum(el)[:-1]])
psi = np.arctan2(np.gradient(y), np.gradient(x))

print(f"base: {src.name}  {len(x)} points, {el.sum():.2f} m, "
      f"|kappa|max {np.abs(kappa).max():.3f}, margin {margin_of(x, y):+.3f} m")
print(f"\n{'a_lat':>6}{'a_long':>7}{'v_max':>7}{'v peak':>8}{'lap':>9}{'gain':>8}   file")

t_ref = None
for a_lat, a_long, v_max in LADDER:
    ggv = np.array([[0.0, a_long, a_lat], [v_max, a_long, a_lat]])
    axm = np.array([[0.0, a_long], [v_max, a_long]])

    vx = tph.calc_vel_profile.calc_vel_profile(
        ax_max_machines=axm, kappa=kappa, el_lengths=el, closed=True,
        drag_coeff=DRAG_COEFF, m_veh=MASS, ggv=ggv,
        dyn_model_exp=DYN_MODEL_EXP, mu=None, v_max=v_max)

    # calc_ax/t_profile want one more speed sample than segments
    vx_open = np.append(vx, vx[0])
    ax = tph.calc_ax_profile.calc_ax_profile(
        vx_profile=vx_open, el_lengths=el, eq_length_output=False)
    t = float(tph.calc_t_profile.calc_t_profile(
        vx_profile=vx_open, ax_profile=ax, el_lengths=el)[-1])
    if t_ref is None:
        t_ref = t

    out = HERE / f"{src.stem}_a{a_lat:g}.csv"
    np.savetxt(out, np.column_stack([s, x, y, psi, kappa, w_right, w_left, vx]),
               delimiter=",", fmt="%.5f",
               header="s_m,x_m,y_m,psi_rad,kappa_radpm,w_right_m,w_left_m,v_mps",
               comments="# ")
    print(f"{a_lat:6.1f}{a_long:7.1f}{v_max:7.1f}{vx.max():8.2f}{t:8.3f}s"
          f"{t - t_ref:+8.3f}   {out.name}")

print("\nGeometry is identical across all of them -- only the speed profile changes.")
print("Run them in order. The first one the car cannot hold gives you the real")
print("grip limit; back off one step and use that.")
