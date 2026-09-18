# M15 — M14 repeat (the run to copy)

- **date** 2026-09-18  **line** `rl_mt_b05b15w05_ell_L65_B50_v9.0.csv`
- **config** `distance_source:=slip v_max:=9.0 enc_rate_window_s:=0.10 exit_guard 0`  **fresh sim**, no target_lead, no brake_cap, no drag_ff
- **hypothesis** M14’s 28-lap mean 8.985 was the stack; a cold re-run clears 30.

## Result

**56 consecutive clean timed laps**, then one hp1 contact (recovery cascade after). First **30 timed: mean 8.963, best 8.92, worst 9.03**. 26/30 strictly under 9.0.

| window | n | contacts | best | median | mean | std | worst | < 9.0 |
|---|---|---|---|---|---|---|---|---|
| first 30 timed | 30 | 0 | 8.92 | 8.955 | **8.963** | 0.029 | 9.03 | 26 |
| until first RESET | 56 | 0 then 1 | 8.89 | 8.96 | **8.965** | 0.032 | 9.05 | 47 |

Times (laps 2–31, the 30-lap window):  
8.95 8.95 8.98 8.94 8.98 8.94 9.01 8.94 8.93 9.02 8.94 8.96 8.94 8.95 8.97 8.97 8.92 8.98 8.97 8.94 8.96 8.93 8.98 8.99 8.92 8.95 8.94 8.98 9.03 9.02

Four of the first 30 are 9.01–9.03. Mean and median are under 9. Longest streak of every-lap < 9.0 is 18.

## How to reproduce

Inside `rr_bridge`, after a **manual** sim Reset+Connect (never `/autodrive/reset_command`):

```bash
/root/Documents/roboracer/experiments/iros2026/scripts/run_mt.sh \
  mt_m15 rl_mt_b05b15w05_ell_L65_B50_v9.0.csv slip v_max:=9.0
```

`run_mt.sh` already sets `enc_rate_window_s:=0.10` and `exit_guard_from/full:=0.0`.

## Verdict

**ACCEPTED as the IROS 2026 race config on this machine.** 30+ clean laps with mean < 9.0. Residual: occasional 9.01–9.05 and a rare hp1 inside-cut after ~50 laps (along-track loc on the straight).
