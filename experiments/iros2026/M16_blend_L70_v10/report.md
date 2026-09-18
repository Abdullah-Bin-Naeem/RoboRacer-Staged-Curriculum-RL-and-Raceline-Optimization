# M16 — same blend geometry, a_long 7.0 / v_max 10

- **date** 2026-09-19  **line** `rl_mt_b05b15w05_ell_L70_B55_v10.csv` (paper **8.652 s**)
- **config** M15 stack + `v_max:=10.0`  **fresh sim**, hybrid LQR, enc 0.10, slip DR, no target_lead / brake_cap / drag_ff / exit-guard
- **hypothesis** Extra straight cap and a_long 7.0 close part of M15’s +0.12 s straight lag.

## Result

**21 consecutive clean timed laps**, then hp1 **exit** contact. Mean **8.852 s** (M15 8.963). Almost all laps 8.79–8.89; two outliers 8.99 / 9.04.

| window | n | contacts | best | median | mean | worst | < 8.85 | ≥ 8.95 |
|---|---|---|---|---|---|---|---|---|
| timed until first RESET | **21** | 0 then 1 | **8.79** | **8.84** | **8.852** | 9.04 | 12 | 2 |

Times:  
8.79 8.84 8.89 8.82 8.85 8.84 8.80 8.82 8.84 9.04 8.83 8.81 8.79 8.86 8.86 8.83 8.85 8.99 8.80 8.86 8.88

First RESET t≈207.3 s, **s≈19.8 (hp1 exit)**. Last 0.5 s: `e_lat` +0.27 inside → **−0.53 wide**, heading error **−0.48 rad**, steer 0.50–0.80, loc at the wall **fine** (along 0.05, cross 0.03). Turn-in along-track was **+0.75 m** (typical on this run mean +0.18, p90 0.57, max 0.64) — early inside, then swing out. Same family as M14/M15 hp1, now on the **exit** after using 9.8 m/s on the straight.

Sector vs paper (first 20 timed): straight still **+0.14 s**, brake **−0.09 s** (overspeed), hp1 **+0.09 s**. Peak on the straight **9.79** (M15 9.05).

## Verdict

**Accepted as the new pace stack, not yet the race stack.** −0.11 s vs M15, 21 clean, not 30, not a consistent 8.7. Do **not** throw this line away. Next: same line + `controller_mode:=kinematic_mpc` (delay bicycle shoot) to hold hp1 heading; M15 remains the conservative fallback.
