# IROS 2026 — what to run, what worked, what did not

Branch: **`adil_mt`** (off Abdullah’s `multi-track`, **do not merge these race knobs onto `multi-track` without a review**).  
Headline result: **M15, 56 clean timed laps, first-30 mean 8.963 s** (best 8.92, worst 9.03).

## Race config (copy this)

| item | value |
|---|---|
| raceline | `raceline/iros2026/rl_mt_b05b15w05_ell_L65_B50_v9.0.csv` |
| what that line is | Abdullah mintime **b0.05** geometry, **b0.15 blended in** at the two tight right-wall zones (s 25–28 and s 37–40, half-weight), speeds re-solved with `enforce_friction_ellipse.py` at a_lat 7.0 / a_long 6.5 / **a_brake 5.0** / v_max 9.0 (paper ~8.80 s) |
| dead reckoning | `distance_source:=slip` (k=0.27, k_brake=1.1) |
| `v_max` | **9.0** |
| `enc_rate_window_s` | **0.10** (this was the missing piece vs M08) |
| `exit_guard_*` | **0** (off) |
| `target_lead_s` | **0** |
| `brake_cap_margin` | **0** / unset |
| `drag_ff` | **0** |
| loop | 45 Hz, `warmup_v_max 2.0` for 21 m, hybrid LQR as S03 |
| sim | **fresh AutoDRIVE**, then **Reset + Connect by hand**. Never publish `/autodrive/reset_command`. Stop with `experiments/iros2026/scripts/teardown.sh`. |

Launch (inside `rr_bridge`):

```bash
/root/Documents/roboracer/experiments/iros2026/scripts/run_mt.sh \
  RUN_NAME rl_mt_b05b15w05_ell_L65_B50_v9.0.csv slip v_max:=9.0
```

`run_mt.sh` already defaults `ENC_WIN=0.10` and exit-guard off.

## Why this is faster than S03 / M01

- S03 (our tum_iqp, a_long 6.0): **9.436 s** mean, 43 clean.
- M01 (Abdullah b0.15 + a_long 6.0 + slip DR): **9.314 s** mean, 41 clean.
- M06 (tighter b0.05 + ellipse + v_max 9): first **sub-9** laps (8.93–8.98) but hit the right wall at s 26.8 (line too close).
- Blend + ellipse + v_max 9 + **enc 0.10** (M15): **8.96 s** mean, 56 clean.

Slip DR takes the long-straight encoder over-read down so AMCL does not sit 0.5 m **ahead** at hairpin 1. Enc window **0.10** (not 0.05) is what made that stable at 45 Hz. The blend keeps M06’s pace without M06’s s26 wall.

## What did **not** work (do not re-enable for pace)

| attempt | why it failed |
|---|---|
| `v_max 8.5` (M09) | Clean laps **9.10–9.35**. Global cap eats the straight. |
| `target_lead_s 0.08` (M10) | ~0.1 s slower, still cut hp1 inside (pose ahead, not late brake). |
| Hairpin a_lat 6.5 (M11) | **9.14+** and still hit C4 exit. |
| `exit_guard` 0.08/0.20 (M12) | Lowers `v_target` but slip **accel_ff** still drives; also costs ~0.1 s on every lap. Does nothing for **inside** cuts. |
| Throttle `u ≤ v` while wide (M13) | First hit was hp1 **inside** (`e_lat +0.37`). Guard is outward-only. |
| `drag_ff` (M03, M07) | Fed drive into braking / no lap-time gain. Leave off. |
| `steer_a_lat_max 7.5` (M05) | Raised the throttle ellipse too; hp1 exit contact. |
| Programmatic `/autodrive/reset_command` | Forbidden. Always ask for a hand Reset+Connect. |
| Judging a mean on < ~25 laps | M08 looked sub-9 for two laps then fell apart. |

## Residual (honest)

- Not every lap is < 9.00. M15 first 30: four laps at **9.01–9.03**. Mean/median are under 9.
- After a long stint, hp1 can still take an **inside** cut when along-track loc spikes (~0.5–0.7 m). M14 did that on lap 29; M15 on lap 57. Next real fix is localization on that straight, not another speed knob.
- Use a **cold simulator**. Loop delay walking to 0.12 s is OK; 90 min sessions are not.

## Code that landed on this branch (beyond Abdullah’s lines)

- `distance_source:=slip` in `dead_reckoning.py` (k and k_brake).
- `enforce_friction_ellipse.py` — re-solve speeds so you do not ask for full a_long at a_lat peak (optional `--lat-zones`).
- `blend_lines.py` — b0.05/b0.15 mix for the two tight walls.
- `run_mt.sh` + `teardown.sh` — launch/stop without stray `pure_pursuit` nodes.
- `pure_pursuit.py`: recover-warmup after a wall (2 m/s for 8 m) and an exit-guard **throttle clamp** that is **off** in `run_mt.sh`. Harmless if guard stays 0.
- Launch tunables: `enc_rate_window_s`, `brake_cap_margin` (keep 0 for race).

## Per-run reports

Index: `experiments/iros2026/EXPERIMENTS.md`  
M01–M08 under `experiments/iros2026/M0*_*/report.md`  
M09–M15: `M09_mt_b05b15w05_B50_v85` … `M15_enc010_repeat`.
