# Loop-rate sweep on the qualification stack, 2026-09-12

Seven runs, one per cap, same line (`raceline_a7.0_rec_6.36.csv`), same image
(this branch at commit `97b8a48` plus a scipy layer; it does NOT contain
`472d782 Adaptive CMD delay`, which was pulled after the runs), dev mode,
`control_hz` 40, about 30 laps each. Everything below is reproduced by
`python3 tools/analyze_caps.py` from `logs/`.

| cap | stack log | CSV |
|---|---|---|
| 20 | `racer_20260912_002127.log` | `run_cap20.csv` |
| 40 | `racer_20260912_001751.log` | `run_cap40.csv` |
| 45 | `racer_20260911_235859.log` | `run_cap45.csv` |
| 50 | `racer_20260912_000256.log` | `run_cap50.csv` |
| 60 | `racer_20260912_000643.log` | `run_cap60.csv` |
| 70 | `racer_20260912_001019.log` | `run_cap70.csv` |
| 80 | `racer_20260912_001408.log` | `run_cap80.csv` |

(`racer_20260911_235611.log` is a first run on the default `raceline_a7.0.csv`,
not part of the sweep.)

## Bottom line

All seven runs were clean: zero contacts, localization steady at 0.18-0.25 m
mean error at every cap. The loop rate did not change how the corners were
driven. What it changed is straight-line speed: above 50 Hz the car
accelerates less and laps get slower, and the CSVs show why.

| cap | laps (launch lap excluded) | best / mean / sd (s) | body clearance min / p5 (m) | straight speed vs plan | accel on straights |
|---|---|---|---|---|---|
| 20 | 30 | 6.32 / 6.37 / 0.03 | 0.05 / 0.10 | -0.31 m/s | 3.2 m/s^2 |
| 40 | 30 | 6.31 / 6.36 / 0.03 | 0.05 / 0.10 | -0.31 | 2.6 |
| 45 | 32 | 6.32 / 6.40 / 0.04 | 0.05 / 0.10 | -0.40 | 2.3 |
| 50 | 30 | 6.35 / 6.44 / 0.04 | 0.05 / 0.10 | -0.41 | 2.3 |
| 60 | 29 | 6.49 / 6.60 / 0.08 | 0.05 / 0.10 | -0.73 | 1.7 |
| 70 | 31 | 6.41 / 6.52 / 0.08 | 0.05 / 0.10 | -0.46 | 2.2 |
| 80 | 29 | 6.43 / 6.73 / 0.24 | 0.00 / 0.10 | -0.95 | 1.6 |

## Corners and walls

**Corner cutting is identical across caps.** Three corners have curvature above
0.45 1/m: C1 (s 0-1.7, right, the start hairpin), C2 (s 13.2-16.4, right),
C3 (s 26.0-27.5, right). In C1 the true position runs 6-14 cm WIDE of the line
at every cap, with no trend against the rate. C2 and C3 sit within 4 cm of the
line. Corner exit speeds are the same at every cap: 0.1 m/s under plan out of
C2, 0.25 m/s under plan out of C3. The follower runs on its own 40 Hz timer
with real time steps, so this is expected.

**The wall is a property of the line, not the rate.** Against the map with a
0.50 x 0.28 m footprint, the body comes within one map cell (5 cm) of a wall
in every run, at s 14.5 in five of the seven. That is the C2 apex, where the
line itself passes 0.20 m from the wall, its tightest point; with a 14 cm
half-width the line leaves 6 cm for the body there by design. The car spends
about 5 % of every lap within 10 cm of a wall at every cap. The only
rate-specific event is one tick at 80 Hz at s 4.2 (C1 exit) where the
footprint read zero clearance: a brush the simulator did not count.

Caveats: the footprint size and where the IPS point sits on the car are
assumptions; the map is 5 cm cells; the logger samples at 20 Hz.

## Why the fast caps are slower

On the straights while accelerating, the slip-band throttle law is saturated
at every cap (the command sits at the band edge 74-97 % of the time), yet
delivered acceleration falls from 3.2 m/s^2 at 20 Hz to 1.6 at 80. Two things
combine:

- **The speed estimate under-reads truth by 0.12-0.20 m/s at every cap**
  (`pp_v_est - speed`; the pose-speed correction is inactive in this build,
  `pp_v_pose` is never valid). The band is anchored on that estimate, so the
  follower believes it runs 8 % slip while the true wheel slip is 2-5 %, on the
  weak part of the tire curve.
- **At 20 Hz the delay compensation masks it.** The band is placed at the
  predicted landing speed, `v_est + a * cmd_delay`. At 150 ms that adds ~0.4
  m/s of wheel speed and the true slip comes out near where the law intends.
  At 50 ms the lead is 0.08 m/s and the estimator's bias is no longer covered.
  Commanded wheel speed above the estimate: 0.88 m/s at 20 Hz, 0.55 at 80;
  throttle 0.28 -> 0.21; top speed 7.13 -> 6.65 against a planned 7.46.

That is the most consistent reading of the columns; the direct test is to log
the estimate against truth during acceleration at 40 and 80 Hz in a repeat.
The 80 Hz run also has a slow phase (laps 4-15 at 6.7-7.4 s, then back to
6.5), which looks external (host load) rather than the controller; 60 Hz is
uniformly slow, which fits the mechanism. One run per cap cannot separate
those: repeat 60 and 80 before concluding anything about them.

## The adaptive delay

The built-in `cmd_delay_auto` is in this image and works, with two weaknesses
visible in `pp_delay`: it takes ~20 s (2-3 laps) to walk from the 175 ms
default to the loop's value, and its 25 ms lag grid quantizes the result (45 Hz
reads 0.075; 50 Hz flips between 0.067 and 0.075; 70 and 80 Hz sit on the
0.050 floor while three frames are 43 and 37 ms).

`472d782 Adaptive CMD delay` is NOT in these runs. Its diff targets exactly
those two weaknesses (5 ms grid; delay seeded and floored from the measured
tick within half a second) plus a control loop that tracks the tick up to 50
Hz and a square-root lookahead law for fast loops. Whether it works is
unanswerable from these logs. It would not touch the straight-line mechanism
above, which lives in the speed estimate and the slip band, not in the delay
estimate; a more accurate short delay shrinks the landing lead further.

## Issues, in order

1. **The line, not the rate, is the wall risk:** 6 cm of body margin at the C2
   apex at every cap. Look at s 14.5 before trading C1's wide exit for speed.
2. **Speed delivery collapses above 50 Hz** through the estimator bias the
   delay lead used to hide. At 40-50 Hz it is ~0.1 m/s of straight speed; at
   60-80 Hz it is 0.4-0.6 s per lap.
3. **Delay estimation is slow to start and coarse.** Rebuild with `472d782`
   and re-run 45 and 80 to see whether it changes the picture.
4. **One run per cap, 20 Hz logger.** 60 and 80 want a repeat.

## Reproducing

    python3 tools/analyze_caps.py            # all tables above, from logs/

Needs numpy and scipy on the host (system python3 has both here).
