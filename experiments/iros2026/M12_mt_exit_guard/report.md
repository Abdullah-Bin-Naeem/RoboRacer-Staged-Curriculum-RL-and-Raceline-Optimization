# M12 — M08 line + exit_guard 0.08/0.20 + recover warmup

- **date** 2026-09-18  **line** `rl_mt_b05b15w05_ell_L65_B50_v9.0.csv`
- **overrides** `v_max:=9.0 target_lead_s:=0.0 brake_cap_margin:=0.2 exit_guard_from:=0.08 exit_guard_full:=0.20` + post-reset 2 m/s for 8 m
- **hypothesis** Exit-guard stops C4/hp1 exit accel-wide; recover warmup stops the cascade.

## Result

Timed before contact: **9.07, 9.12, 9.07, 9.03, 9.07**. First stall **t = 67.05 s, s = 19.77 (hp1 EXIT)**.

On that lap the estimate was **+0.58…+0.65 m along-track** on the straight (clean laps ±0.24). Turn-in was early (`e_lat` +0.25 inside at apex), then it swung outside (−0.10 → **−0.50**). Loc at the wall was fine. Throttle **rose 0.09 → 0.14** while already 16–40 cm out: IMU +1.8…+2.8 m/s². `exit_guard` lowered `v_target` toward `v`, but slip `accel_ff` still commanded wheel speed **above** the car.

Then 5 recoveries; recover-warmup did release after 8 m, then the next wall.

## Verdict

**Rejected as 30-lap.** Guard vs slip feedforward is a controller bug, not a profile miss. Next: clamp `u ≤ v` (and drop plan accel) whenever outward > `exit_guard_from`; keep the M08 v9.0 line so clean laps can still go sub-9.
