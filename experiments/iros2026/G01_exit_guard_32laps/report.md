# G01 - exit guard, 32 timed laps

- **date** 2026-09-17
- **line** `raceline_tum_iqp_h7.0_a7.0b.csv`
- **overrides** `control_hz:=45 warmup_v_max:=2.0 warmup_dist_m:=21.0 enc_rate_window_s:=0.10 amcl_params_file:=params/amcl_beams360.yaml exit_guard_from:=0.05 exit_guard_full:=0.15`
- **raw log** `logs/exit_guard_clean_launch.log`
- **telemetry** `logs/exit_guard_clean.csv`

## Result

The timed window is laps 2-33; lap 1 is the warmup/out-lap.

| timed laps | contacts | best | median | mean | std | worst |
|---:|---:|---:|---:|---:|---:|---:|
| 32 | 0 | 9.580 | 9.740 | 9.729 | 0.070 | 9.860 |

The run completed 32 consecutive timed laps without a crash. The first later
session failure appeared after lap 36: the localization bootstrap reported
multiple resets and the follower repeatedly lost localization. Lap 37 was 12.58
s. Those later events are excluded from the clean 32-lap window but remain a
known stability risk.

## Comparison

T03 baseline: mean 9.589 s, std 0.046 s, worst 9.681 s over 20 clean laps.
The guard did not improve speed or lap-time variance in this run, but it
survived a longer clean window and is the first direct test of the measured
hairpin-exit understeer hypothesis.

## Verdict

**PROMISING, CONDITIONAL.** Keep the guard as an experimental option. Do not
make it the accepted default yet: the clean window is stable, but the later
localization/reset cascade prevents claiming dozens of uninterrupted laps.
Next experiment should isolate session degradation/localization reset handling
before increasing speed.
