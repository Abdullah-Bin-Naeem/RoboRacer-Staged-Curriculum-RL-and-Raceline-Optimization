# T05c — h70b_warm30

- **date** 2026-09-17  **line** `raceline_tum_iqp_h7.0_a7.0b.csv` (profile 9.31 s)  **loop cap** 45  **laps asked** 2
- **overrides** `control_hz:=45 warmup_v_max:=3.0 warmup_dist_m:=19.5`
- **hypothesis** Out-lap robustness: warmup_v_max 3.0 over 19.5 m (was 4.0 over 19) gives AMCL more scans per metre through hairpin 1, so the +0.6 m along-track start-straight error is corrected before turn-in; control_hz 45 as T03
- **prediction** 0 contacts on the out-lap in 4 of 4 repeats; hairpin-1 entry along error |.| < 0.3 m

## Result

| timed laps | contacts | best | median | mean | std | worst | <10 s | gap to profile | loop Hz | delay |
|---|---|---|---|---|---|---|---|---|---|---|
| 2 | 0 | 9.651 | 9.744 | 9.744 | 0.093 | 9.837 | 2 | 0.434 | 24.2 | 0.124 |

First contact: none

| tracking (timed laps) | mean | p90 | max |
|---|---|---|---|
| |e_lat| m | 0.092 | 0.177 | 0.370 |
| outward m | 0.010 | 0.163 | 0.370 |
| localization m | 0.178 | 0.448 | 0.830 |
| a_lat m/s² | 3.701 | 6.543 | 7.742 |
| |v_est − v| m/s | 0.194 | 0.417 | 0.863 |

Body clearance: min 0.05 m, p05 0.106 m.  Steering at lock: 0.0 % of ticks.

| corner | s | side | κ | v plan apex | v true apex | worst outward | |e| p90 | min clearance | a_lat max | at lock % |
|---|---|---|---|---|---|---|---|---|---|---|
| C1 | 0.0-0.0 | L | 0.36 | 3.2 | 3.62 | 0.161 | 0.158 | 0.35 | 2.82 | 0.0 |
| C2 | 17.2-20.1 | L | 1.2 | 2.41 | 2.27 | 0.37 | 0.338 | 0.202 | 7.32 | 0.0 |
| C3 | 21.1-21.2 | R | 0.38 | 4.28 | 4.32 | -0.071 | 0.199 | 0.313 | 3.6 | 0.0 |
| C4 | 23.0-24.9 | L | 0.8 | 2.96 | 2.95 | 0.087 | 0.167 | 0.056 | 6.59 | 0.0 |
| C5 | 35.9-37.6 | L | 0.66 | 3.27 | 3.24 | 0.126 | 0.11 | 0.168 | 6.86 | 0.0 |
| C6 | 40.7-43.3 | L | 1.21 | 2.4 | 2.27 | 0.168 | 0.163 | 0.168 | 7.18 | 0.0 |

Gates: 50 clean laps **no**, mean < 10 s (worst < 10.2) **PASS**

## Figures

![track](figures/track_overlay.png)

![along](figures/along_track.png)

![envelope](figures/envelope.png)

![laps](figures/lap_times.png)

![speed](figures/run_speed_profile.png)

![loss](figures/run_time_loss.png)

Time-loss split (plot_speed_tracking.py): see `speed_tracking.txt`.

## Verdict

_to be written after reading the figures_
