# M14 — M08 line, enc 0.10, no exit-guard

- **date** 2026-09-18  **line** `rl_mt_b05b15w05_ell_L65_B50_v9.0.csv`
- **overrides** M01-style: `distance_source:=slip v_max:=9.0 enc_rate_window_s:=0.10 exit_guard 0` (no target_lead, no brake_cap)
- **hypothesis** Enc window 0.10 restores M01 along-track quality on the M08 sub-9 line.

## Result

28 consecutive timed laps, then hp1 contact ~3.5 s after lap 29.

| n | best | median | mean | std | worst | < 9.0 | = 9.00 | > 9.0 |
|---|---|---|---|---|---|---|---|---|
| 28 | 8.92 | 8.975 | **8.985** | 0.041 | 9.07 | 16 | 3 | 9 (9.01–9.07) |

Times: 8.93, 8.92, 9.05, 8.95, 9.00, 8.98, 8.95, 8.97, 9.01, 8.98, 9.00, 9.06, 8.96, 9.01, 9.01, 9.04, 9.00, 8.97, 9.03, 8.95, 8.94, 9.03, 9.07, 8.97, 8.97, 8.97, 8.94, 8.93.

First contact **t = 272.75 s, s = 18.97 (hp1 apex, inside e_lat +0.22…+0.28)**. Straight along-track **+0.40…+0.72 m**, v_est 0.3–0.6 high in the brake zone, delay **0.125 s** (early laps ~0.12). Early-turn-in inner wall — same as M13, once in 29 attempts. Then recovery cascade.

## Verdict

**Best run of the campaign.** Mean under 9, 28 clean. Not yet 30 clean and not every lap < 9.0. Next: same config re-run (the hit was an along-track outlier, not a lap that had been drifting). Do not put exit-guard back on; it costs ~0.1 s and does not stop inside cuts.
