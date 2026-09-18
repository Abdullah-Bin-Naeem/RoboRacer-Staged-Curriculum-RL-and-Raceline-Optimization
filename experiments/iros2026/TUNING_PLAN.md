# Controller tuning plan: IROS 2026 compete track

Branch `adil_iros` (off `multi-track`). Simulator build `2026-iros-compete`, devkit
image `autodrive_roboracer_api:2026-iros-compete` + nav2/matplotlib (`docker/dev/Dockerfile`).
Stack: AMCL + dead reckoning + pure pursuit (`racer_bringup/race.launch.py`).

## 0. The goal, as gates

| gate | definition | measured by |
|---|---|---|
| **G1 clean** | 50 consecutive timed laps, `collision_count` 0 | `laps.csv` (simulator telemetry) |
| **G2 fast** | mean timed lap < 10.00 s, worst < 10.20 s | `summary.json` `lap_mean`, `lap_worst` |
| **G3 stable** | truth \|e_lat\| p90 ≤ 0.10 m; min body clearance ≥ 0.10 m on every lap; localization p90 ≤ 0.25 m | `summary.json`, `figures/envelope.png` |
| **G4 legal** | G1–G3 reproduced once with `mode:=race` and once in a second session | two final runs |

All gates are judged **at the evaluation loop rate**: the organizers run 40–50 Hz, so the
tuning rate is **45 Hz** (`tcp_nodelay` shim, `loop_hz_cap 45`; `LOOP_RATE.md` on
`abdullah_qualification_analysis`). Every constant in the follower was tuned at the
laptop's accidental 18 Hz / 175 ms round trip, so none of them is trusted at 45 Hz until measured.

---

## 1. Vehicle dynamics that decide the tuning

Full derivation with sources: `raceline/VEHICLE_MODEL.md`. What each fact means for the controller:

| fact (measured / from the Unity source) | consequence |
|---|---|
| Throttle sets **wheel speed** u = 25.25·θ within 1 ms; motor torque is ~1000× what the tire can pass | encoders read the command, not the car; speed comes from the tire observer (`speed_source: tire`); throttle is a slip command (`throttle_mode: slip`) |
| Longitudinal tire: peak 7.06 m/s² at slip 0.15, **robust 4.55** (asymptote); lateral: peak 9.81 at 0.57°, **robust 4.90** | demands above the asymptote are held only while slip stays in a narrow window; ICRA/Porto hold up to **~7.0 lateral** with the curvature cap, and hairpins need less |
| PhysX slip denominator max(v, 4 m/s) | below 4 m/s the slip band is ±0.6 m/s of wheel speed: this is the whole hairpin regime (apex 1.9–2.2 m/s) |
| Drag linear, 0.273·v (2.2 m/s² at 8 m/s) | acceleration budget falls with speed; straight speed above 8 is expensive |
| Steering ±30° (κ_max 1.78, r 0.56 m), rate 3.2 rad/s | the hairpins (κ 1.51, r 0.66 m) use **26° of 30°**: 13 % of steering headroom, the tightest resource on this track |
| Command → wheel round trip = **3 simulator frames** (175 ms at 18 Hz, ~65–75 ms at 45 Hz) | the follower predicts the landing speed and scales lookahead with the measured delay; both laws were validated only at ≥ 175 ms |
| A wall contact respawns the car at the last checkpoint with v = 0, +10 s | one contact ends G1; recovery must re-localize (needs this track's checkpoints) |

## 2. The raceline

Geometry `raceline_tum_iqp*.csv` (TUM iterative min-curvature, κ bound 1.5, 0.20 m margin),
43.23 m, 433 points at 0.10 m. `tools/tuning/line_report.py` prints this.

Counter-clockwise loop, **four left-hand corners and nothing else**:

| | s (m) | κ max | r | steer | apex plan (tum_iqp) | room inside / outside | notes |
|---|---|---|---|---|---|---|---|
| **C1** bottom hairpin | 4.6–6.9 | 1.50 | 0.67 | 26° | 2.21 m/s, a_lat 7.32 | 0.44 / 1.51 | E00 contact: 0.38 m wide on every lap, then the wall |
| **C2** right chevron tip | 10.0–11.9 | 0.72 | 1.40 | 13° | 3.19, 7.29 | 0.32 / 1.35 | body clearance 0.05 m at the tip in E00 |
| **C3** upper chevron tip | 22.8–24.7 | 0.70 | 1.43 | 13° | 3.23, 7.29 | 0.32 / 1.38 | 0.25 m wide in E00 |
| **C4** top hairpin | 27.7–30.3 | 1.51 | 0.66 | 26° | 2.21, 7.35 | 0.33 / 1.57 | |
| main straight | 30.4 → 43.2 → 4.6 | | | | 8.0 cap from s ≈ 40 | 0.32 min | 17 m; clearance 0.05–0.10 m at s 40–43 in E00 (check the IPS-anchor caveat) |

Profiles on this geometry (the only difference between the files is `v_mps`):

| file | lap (profile) | hairpin apex | lateral in hairpins | status |
|---|---|---|---|---|
| `raceline_tum_iqp.csv` | 9.60 s | 2.21 | **7.3**, above the ~7.0 the tire holds | E00: contact at C1, lap 3 |
| `raceline_tum_iqp_a7.0z.csv` | 9.97 s | 1.90 | 5.45 | what `run_iros_08` actually followed (target min 1.89; §4) — 11 clean laps, 10.25–10.70 s |
| `raceline_tum_iqp_a6.5z/a6.0z.csv` | 9.99 / 10.03 s | 1.90 | 5.45 | untested |

Defects found in the profiles: (1) a speed step at the lap seam (7.41 → 7.54 across s = 0, a
nominal −11 m/s² that is worth 0.01 s, harmless); (2) the width columns spike at s ≈ 36.8
(LiDAR gap in the middle wall), which feeds `lookahead_sag_frac` one bad margin sample.

**Pricing the 10 s target** (`tools/tuning/profile.py --sweep`, agrees with the exported
profiles to ~0.15 s; the real car has run **0.28 s** (run_iros_08) to **0.70 s** (E00) over its profile):

| profile variant | lap | apex |
|---|---|---|
| hairpins 6.0, rest 7.0, accel 4.5 / brake 4.5 (today's z class) | 9.97 | 2.00 |
| + accel 5.5 / brake 5.0 | 9.62 | 2.00 |
| + accel 6.0 / brake 5.0 | 9.54 | 2.00 |
| + straight 9 m/s | 9.49 | 2.00 |
| hairpins 6.5 + the above | 9.38 | 2.08 |
| hairpins 6.5, brake 5.5 + the above | 9.30 | 2.08 |

So **the z lines cannot make 10 s** (9.97 on paper + ≥ 0.25 s of controller gap). G2 needs
a profile at ≈ 9.5 s **and** the controller gap held at ≤ 0.4 s. On ICRA the same
longitudinal ladder (accel 5.5 → 6.0, brake 5.0, straight 9) was worth 0.40 s and held clean,
so the path exists; the hairpins stay at ≤ 6.0–6.5 because steering headroom there is 13 %.

## 3. What E00 (the first run) showed

`experiments/iros2026/E00_first_run_tum_iqp/report.md`. Plain `tum_iqp` at the stock 18 Hz loop:
laps 15.45 (out-lap), 10.31, contact on lap 3 at C1. At the contact localization was 3–6 cm,
steering at lock, speed 2.0–2.4 m/s against a 2.2 plan: **understeer at a hairpin planned
above the tire**, not a localization or loop failure. On the straights the car ran 0.1–0.2 m
right of the line with 0.2–0.45 m of localization error and entered C1 carrying it. Recovery
then looped for three minutes: `frames.TRACKS['iros2026']['checkpoints']` is empty, so every
re-seed fell to the global search, which timed out. Reset poses measured: (1.728, −15.765,
0.293) after C1, (5.047, −11.466, 1.571) after C2.

## 4. Levers, with the evidence already on file

Sources: Porto runs 9–41 and ICRA runs 1–24 (`raceline/VEHICLE_MODEL.md` §7), the ICRA follower
campaign E0–E15 (`docs/plans/CONTROLLER_TUNING_PLAN.md` on `adil-icra-longitudinal`), the loop-rate
sweep (`HZ_ANALYSIS.md` on `qualification_1_pure_pursuit`), and the adaptive delay (`472d782`, Usman).

| lever | now (iros2026) | evidence | expected at 45 Hz |
|---|---|---|---|
| loop rate (`tcp_nodelay`, `loop_hz_cap`) | stock 18 Hz | sweep 20→45 Hz (Porto): same corners, mean 6.37 → 6.40 s; >50 Hz loses straight speed (estimator bias the long delay used to hide) | tune here; watch `pp_v_est − speed` while accelerating |
| `control_hz` | 20 fixed | at 45 Hz a 20 Hz follower drops every other frame (+25 ms staleness) | 40–45; Usman's `control_hz_auto` does this |
| delay estimate (`cmd_delay_auto`) | 25 ms grid, 0.3 blend, ~20 s to converge | quantizes 45 Hz to 75 ms and 70–80 Hz to the 50 ms floor; 2–3 laps on slow-machine settings | port `472d782`: tick-seeded 3-frame delay, 5 ms grid, tick floor |
| lookahead delay scaling | linear, floor 0.7 → Ld_max 1.54 m at ≤ 123 ms | ICRA E0: removing it = 49 contacts; linear law untested in the fast direction | `472d782` sqrt law (Ld_max ≈ 1.4–1.5 m at 70 ms); A/B against linear |
| `latency_comp_s` | 0.05 (tuned at 175 ms) | Porto run 25: −0.1 s at 175 ms; no delay scaling | ≈ 0.02–0.03 at 70 ms; one A/B |
| hairpin lateral limit (profile `--lat-zones`) | 7.3 (tum_iqp) / 5.45 (z) | ICRA hairpins κ 1.37: 6.5 → 2 hits in 46 laps, 6.0 → 0 hits in 60+ | 6.0 first; 6.5 only if C1/C4 outward p90 < 0.15 m |
| `steer_a_lat_max` (curvature cap) | 7.0 | Porto 6.5 rung spiralled without it; ICRA 8.5 → 146 hits | keep 7.0; revisit only if hairpins run at lock |
| `lookahead_min` | 0.80 | ICRA: Ld pinned at the floor in hairpins 84 % of ticks; 0.65 hit at T2 | only if C1/C4 show inside-cut or lag with Ld at floor |
| `target_lead_s` | 0.0 (registry) | ICRA: 0.0 took 0.11 s off braking zones (Abdullah run 13); on the macOS host 0.0 was 0.40 s slower + contact | re-bracket 0.0 / 0.04 / 0.08 at 45 Hz |
| `slip_kp` | 0.76 | ICRA E13: 1.2 accepted, lap std 0.12 → 0.05 (variance, not mean) | 1.0–1.2 for G2's worst-lap bound |
| `slip_circle` + `accel_ff` | off | ICRA runs 18–23: accel delivery 91 → 98 %, part of −0.55 s; on the macOS host `slip_kp 1.2 + accel_ff` = contact | only with the longitudinal profile ladder |
| profile accel / brake budget | 4.5 / 4.5 | ICRA: 5.5/5.0 −0.18 s, 6.0/5.0 −0.14 s more, both clean | the main source of the 0.45 s G2 needs |
| straight speed (`--v-zones`, follower `v_max`) | 8 | ICRA: 9 on the straight −0.05 s; global 9 hit T2 | last, straight only |
| recovery checkpoints | none | ICRA: 3 measured, recovery converged | add C1/C2 now, the rest as they appear |

## 5. Phases

Each experiment changes **one** thing against the last accepted configuration (the "control").

### Phase 0: groundwork, no lap-time claims (today)
1. Tooling: `tools/tuning/run_experiment.sh`, `report.py`, `sim_ctl.py`, `profile.py`, `line_report.py`; ported `plot_speed_tracking.py`, `compare_runs.py`, `ab_report.py`; `analyze_run.py` reset-section crash fixed. **Done**, validated on E00.
2. Add the two measured reset poses to `frames.TRACKS['iros2026']['checkpoints']`.
3. Check that a software reset (`sim_ctl.py reset`) returns the car to the spawn with both counters at 0, so runs need no GUI clicks.
4. Build `tools/libnodelay.so` in the container and measure the loop at stock / 45 / uncapped (`tools/topic_rates.py`).
5. Port `472d782` (control_hz auto, tick-seeded delay, 5 ms grid, sqrt lookahead) into `racer_control/pure_pursuit.py` **behind parameters that default to today's behaviour**, so it is A/B-able.
6. Generate the candidate profiles with `profile.py` (hairpins 6.0 and 6.5 × accel 4.5/5.5/6.0 × brake 5.0 × straight 8/9) and fix the seam step.

### Phase 1: baselines and the loop-rate decision (25 laps each)
| id | line | loop | change | purpose |
|---|---|---|---|---|
| E01 | `a7.0z` | stock 18 Hz | none | reproduce run_iros_08 with full logging: the known-clean reference |
| E02 | `a7.0z` | 45 Hz | none | what the evaluation rate does to today's constants |
| E03 | `a7.0z` | 45 Hz | + `control_hz 45` | frame staleness |
| E04 | `a7.0z` | 45 Hz | + adaptive delay port | delay converges in < 1 s at 45 Hz instead of 20 s |
| E05 | `a7.0z` | 45 Hz | lookahead sqrt vs linear | phase margin vs chord sag at a short delay |

Exit: a 45 Hz control configuration that is clean for 25 laps with G3 met. That is the
**controller baseline**.

### Phase 2: lateral stability where it will break (25 laps each, on the Phase 1 control)
Order set by what the figures show: C1/C4 outward error and at-lock %, C2 tip clearance,
straight-line localization error. Candidates: `latency_comp_s` 0.05 → 0.025; `target_lead_s`
bracket; margin zones from the envelope plots (C2 tip, main straight left wall) regenerated
into the line; `slip_kp` 1.0/1.2. Exit: G3 on a 25-lap run, lap std ≤ 0.08 s.

### Phase 3: the speed ladder (25 laps each, one rung per run)
1. hairpins 6.0 + accel 5.5 / brake 5.0 (profile 9.62)
2. accel 6.0 (9.54), then `slip_circle 0.12` + `accel_ff 1` as its own A/B
3. straight 9 via `--v-zones` + follower `v_max 9` (9.49)
4. hairpins 6.5 (9.38), only if rung 3 shows C1/C4 outward p90 < 0.15 m and ≤ 5 % at lock
5. brake 5.5 (9.30), only if braking delivery ≥ 90 %

Stop climbing at the first rung that meets G2 on a 25-lap run with margin (mean ≤ 9.90); the rest is risk.

### Phase 4: validation
- V1: 50 laps on the chosen configuration (G1–G3).
- V2: 50 laps in a fresh session (simulator restarted, laptop cool): the ICRA/Porto lesson is that a long graphics session decays the loop.
- V3: `mode:=race` run (no ground-truth readers after the seed; lap times from the simulator HUD/telemetry only).
- V4: robustness, 10 laps each at stock 18 Hz and 50 Hz caps: must stay clean (slower is acceptable).

## 6. Rules for every run

Adapted from the ICRA campaign, where each one was learned by losing a run.
1. **One variable per experiment**, passed as a launch override or as a new line file; the yaml stays untouched until a change is accepted.
2. **Hypothesis and prediction written before the run** (`--hypothesis`, `--prediction` go into `config.json`), scored after.
3. **Software reset before every run**; verify both counters read 0 (`sim_ctl.py reset` does and logs it).
4. **Out-lap discarded**; timed laps end at the first contact (the runner stops there by default, `--max-contacts 1`).
5. **Check the intermediate quantity moved** (`pp_ld`, `pp_delay`, `pp_slip`, `pp_v_target`) before attributing the outcome to the change.
6. **Accept** only if: 0 contacts; median ≥ 0.03 s better, or median held and \|e_lat\| p90 / min clearance better; no corner's worst outward error or min clearance worse by > 0.03 m; `pp_delay` median within 0.01 s of the control.
7. **Loop decay**: record `loop_hz` and `delay_median` for every run (the report does); if the loop drops > 10 % from the session's first run, restart the simulator before comparing.
8. **GPU temperature**: sampled every 5 s during the run; the RTX 2060 reached 86 °C in E00. A thermal slowdown shows up as `loop_hz` drift first.
9. Every run, including aborted ones, keeps its directory and its row in `EXPERIMENTS.md` with a verdict.

## 7. What gets logged per experiment

`experiments/iros2026/<ID>_<slug>/`:

| file | content |
|---|---|
| `config.json` | line, laps, loop cap, overrides, hypothesis, prediction, git commit and dirty flag |
| `run.csv` | 20 Hz: truth pose/speed/yaw rate, AMCL estimate and its error, and the follower's status (`pp_v_est`, `pp_v_target`, `pp_u_cmd`, `pp_throttle`, `pp_steering`, `pp_ld`, `pp_e_lat`, `pp_e_head`, `pp_kappa`, `pp_s`, `pp_slip`, `pp_delay`) |
| `laps.csv` | the simulator's own lap times and contact count per lap |
| `launch.log` | every node's output, including override confirmation and recovery events |
| `summary.json` | all numbers below, machine-readable |
| `report.md` | result tables, per-corner table, gates, figures, verdict |
| `figures/track_overlay.png` | map + raceline + true path coloured by lateral error, contacts marked |
| `figures/along_track.png` | per lap against s: lateral error with wall room, body clearance, speed vs plan, localization error |
| `figures/envelope.png` | mean/min/max lateral error and worst/median clearance over all laps, per s |
| `figures/lap_times.png` | every timed lap against the 10 s target and the profile |
| `figures/run_speed_profile.png`, `run_time_loss.png`, `run_slip_band.png` | planned vs commanded vs achieved speed, and where the time goes (follower clip vs delivery), per section |

Summary numbers: timed laps, contacts and where, best/median/mean/std/worst lap, laps < 10 s,
gap to profile, \|e_lat\| and outward mean/p90/max, min and p05 body clearance, localization
mean/p90/max, speed-estimate error, a_lat used, % at steering lock, loop Hz, delay; per corner:
apex speed plan vs true, worst outward, \|e\| p90, min clearance, a_lat max, % at lock.
`EXPERIMENTS.md` holds one row per run. `raceline/ab_report.py` and `compare_runs.py` overlay
any two runs for the accept/reject decision.

Run one:

```bash
tools/tuning/run_experiment.sh E01 baseline_a70z_stock --line raceline_tum_iqp_a7.0z.csv --laps 25 --hz stock \
    --hypothesis "run_iros_08's profile is clean at 18 Hz" --prediction "0 contacts, 10.3-10.5 s"
```

## 8. Open questions

- The slam fixes mentioned on Usman's side are not on `iros_compete_usman` (it carries no slam
  code); which branch or commit are they on?
- Does the software reset leave the simulator connected and the drive channel armed? (Phase 0, item 3.)
- Evaluation-rate check: are the 40–50 Hz figures for the compete round unchanged?
