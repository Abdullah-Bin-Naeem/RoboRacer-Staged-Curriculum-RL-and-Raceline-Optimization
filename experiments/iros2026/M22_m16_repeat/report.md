# M22 — M16 repeat (cold, car still)

- **date** 2026-09-19  **line** `rl_mt_b05b15w05_ell_L70_B55_v10.csv`  **v_max 10**  hybrid LQR  enc 0.10  slip DR
- **sim** container restarted, AutoDRIVE reopened, car **still** at Connect (odom 0, zero throttle publishers)

## Result

Warmup lap 75.96 s, no wall. First 9 timed: **8.79–8.91** (mean **8.846**). Then **one contact** at `(+5.07, +0.12)`. Recovered (15.74 / 11.40). Then **30 consecutive timed (laps 13–42) mean 8.852**, median 8.85, best **8.77**, worst 8.94. **0 laps ≥ 9.00.**

Times (laps 13–42):  
8.88 8.80 8.90 8.87 8.85 8.88 8.78 8.84 8.84 8.88  
8.84 8.88 8.80 8.89 8.90 8.88 8.83 8.83 8.83 8.89  
8.80 8.84 8.80 8.85 8.87 8.86 8.83 8.79 8.94 8.88

Later contacts at the same `(+5.07, +0.12)` spot and at spawn. The 8.8 pace is real; that corner is the remaining contact.

## vs remote M15

M15 first-30 mean **8.963**. This window **8.852** (−0.11 s). Same controller. Faster profile only.
