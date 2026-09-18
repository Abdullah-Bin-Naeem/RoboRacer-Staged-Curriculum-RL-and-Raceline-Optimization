# IROS 2026 — what to run, what worked, what did not

Branch: **`adil_mt`** (off Abdullah’s `multi-track`, **do not merge these race knobs onto `multi-track` without a review**).

**Headline (pace):** M22 / M16 line, `v_max` 10 — after one contact, **30 consecutive timed laps mean 8.852 s** (best 8.77, worst 8.94, **all < 9.0**). That is **−0.11 s vs the M15 config currently on `origin/adil_mt` (first-30 mean 8.963).**

**Fallback (most clean laps):** M15 / M20, `v_max` 9 — 56 / 30+ clean, mean **8.96**.

## Race config — pace (copy this)

| item | value |
|---|---|
| raceline | `raceline/iros2026/rl_mt_b05b15w05_ell_L70_B55_v10.csv` |
| what that line is | Same **b0.05 / b0.15 blend** as M15 (tight walls s 25–28 and 37–40). Speeds re-solved with `enforce_friction_ellipse.py` at a_lat **7.0** / a_long **7.0** / a_brake **5.5** / **v_max 10.0** (paper **8.652 s**). M15 was a_long 6.5 / a_brake 5.0 / v_max 9.0 (paper ~8.80). |
| dead reckoning | `distance_source:=slip` (k=0.27, k_brake=1.1) |
| `v_max` | **10.0** (must match the CSV cap; 9.0 clips the straight and you get M15) |
| `enc_rate_window_s` | **0.10** |
| `exit_guard_*` | **0** (off) |
| `target_lead_s` | **0** |
| `brake_cap_margin` | **0** |
| `drag_ff` | **0** |
| `controller_mode` | **hybrid_lqr** (do **not** use `kinematic_mpc` — M17 out-lap cascade) |
| loop | 45 Hz, `warmup_v_max 2.0` for 21 m |
| sim | **Quit AutoDRIVE and reopen** (or a real Reset with the car **still**). Connect by hand. Never publish `/autodrive/reset_command`. If the car moves on Connect with no `pure_pursuit`, Unity still has last throttle — Disconnect, wait, Reset, Connect. Stop with `teardown.sh` (that restarts `rr_bridge`). |

Launch (inside `rr_bridge`):

```bash
/root/Documents/roboracer/experiments/iros2026/scripts/run_mt.sh \
  RUN_NAME rl_mt_b05b15w05_ell_L70_B55_v10.csv slip v_max:=10.0
```

`run_mt.sh` already defaults `ENC_WIN=0.10` and exit-guard off.

## Conservative fallback (M15)

```bash
/root/Documents/roboracer/experiments/iros2026/scripts/run_mt.sh \
  RUN_NAME rl_mt_b05b15w05_ell_L65_B50_v9.0.csv slip v_max:=9.0
```

M20 reproduced this on a cold sim: first-30 mean **8.964**, 33 clean then a contact.

## How the 8.8 was achieved (M16 → M22)

Controller is **the same as M15** (hybrid LQR, slip DR, enc 0.10, 45 Hz). The only pace change is the **speed profile**:

1. Keep M15 **geometry** (blend), do not go back to raw b0.05 (M06 right-wall at s 26).
2. Re-solve ellipse at **a_long 7.0** (tire peak) and **v_max 10** so the 16 m straight can keep pulling after 9 m/s. M15 peaked 9.05; M16/M22 peak ~9.8.
3. **Fresh sim, car still at Connect**, **one** `pure_pursuit`. Launching v10 into a dirty sim or a rolling car is how M17–M19 / M21 died on lap 1–2.

M16 (first time): 21 clean, mean **8.852**, then hp1 exit.  
M22 (repeat, cold, car still): 9 timed 8.79–8.91, **one contact** at `(+5.07, +0.12)`, recovery, then **laps 13–42: 30 consecutive, mean 8.852, every lap < 9**. Best **8.77**.

## What did **not** work (do not re-enable for pace)

| attempt | why it failed |
|---|---|
| `kinematic_mpc` (M17) | Out-lap hp1 at **2 m/s warmup**. Leave unused. |
| `v_max 9.5` (M21) | Still hit as soon as warmup released. Not a middle ground. |
| `v_max 8.5` (M09) | Clean laps **9.10–9.35**. |
| `target_lead_s 0.08` (M10) | ~0.1 s slower; still cut hp1 inside. |
| Hairpin a_lat 6.5 (M11) | **9.14+**. |
| `exit_guard` (M12/M13) | Costs ~0.1 s; does not stop **inside** cuts. |
| `drag_ff` (M03, M07) | Leave off. |
| Dual `pure_pursuit` / dirty sim | Car moves on Connect with no racer; seed fails; first-lap walls. `teardown.sh` restarts the container. |
| Programmatic `/autodrive/reset_command` | Forbidden. |

## Residual (honest)

- M22 is **not** 30 clean from the first timed lap. There was **one contact after 9 laps** (`+5.07, +0.12`), then a long 8.8 streak. That corner is the next fix; do not slow the whole line for it.
- hp1 can still go inside-then-wide when along-track is **ahead** ~0.4–0.7 m on the straight (M16 lap 22, M19 lap 2).
- M15 remains the “need 50 clean” stack.

## Per-run reports

Index: `experiments/iros2026/EXPERIMENTS.md`  
M15: `M15_enc010_repeat` (remote race config until this commit)  
M16: `M16_blend_L70_v10`  M20: `M20_m15_cold`  M22: `M22_m16_repeat`
