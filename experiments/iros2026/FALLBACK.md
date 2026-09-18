# Safe fallback: IROS 2026 compete track

Validated in **E06** (2026-09-17): 19 consecutive clean timed laps at the 45 Hz loop,
10.47-10.70 s (mean 10.59, std 0.07), 0 contacts, stopped by hand, not by a failure.
Report: [E06_kb11c2_h45_45hz/report.md](E06_kb11c2_h45_45hz/report.md).

Locked at git tag **`iros-safe-fallback-e06`** on branch `adil_iros`. Check out the tag to get
the exact follower code this ran on (later merges change the encoder-rate implementation).

| | |
|---|---|
| raceline | `raceline/iros2026/raceline_kb11c2_h45_a45_b45.csv` |
| geometry | `make_raceline.py track_solid.yaml --kappa-bound 1.1 --margin-zones 12.5:15.5:0.25:R` (`raceline/iros2026/kb/raceline_tum_iqp_kb1.1_c2r25.csv`) |
| profile | `tools/tuning/profile.py --a-lat 7.0 --hairpin 0.9:4.5:1.2 --a-acc 4.5 --a-brake 4.5 --v-max 8` -> 10.25 s |
| loop | bridge `tcp_nodelay:=true loop_hz_cap:=45` |
| follower overrides | `enc_window_s:=0.10 cmd_delay_tick_seed:=1` (plus the track defaults `v_max 8.0`, `target_lead_s 0.0`) |
| checkpoints | the three in `frames.TRACKS['iros2026']` at the tag |

Run it (from the tag):

```bash
git checkout iros-safe-fallback-e06
tools/tuning/run_experiment.sh F01 fallback_check --line raceline_kb11c2_h45_a45_b45.csv --laps 25 --hz 45 \
    -- enc_window_s:=0.10 cmd_delay_tick_seed:=1
```

Why it is safe: hairpins at kappa 1.1 (20 deg of 30 deg steering, never at lock) planned at 4.5 m/s^2,
C2 exit held 0.51 m off the right-hand wall, everything else at 7.0 lateral / 4.5 longitudinal.
