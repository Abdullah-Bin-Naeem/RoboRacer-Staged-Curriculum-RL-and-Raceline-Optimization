# Follower tuning plan — ICRA 2026 track

**Owner:** Adil (pure pursuit controller + speed-profile matching)
**Branch:** `adil-icra-longitudinal`
**Goal:** **0 collisions** and **≤ 11.50 s** on the ICRA 2026 track.
**Status:** baseline measured, no tuning change accepted yet.

Companion: [`follower-internals.html`](follower-internals.html) — interactive explanation of
the geometry, the lookahead chain, profile sampling and the tire model. Open it first if you
have not read the controller before. This document is the *plan*; that one is the *map*.

---

## 1. The one-paragraph version

The raceline file already contains the speed we should be driving. `raceline_a7.0.csv` plans
**11.441 s**. We drive it in **11.75 s at best, 11.92 s median**, with **1 collision in 20 laps**.
So the raceline is not the bottleneck and does not need to get faster — the follower needs to
deliver the profile it is already given. The gap splits cleanly into two budgets that need
separate experiments:

| Budget | Size | Nature | Evidence |
|---|---|---|---|
| **best → plan** | +0.31 s | **Longitudinal.** Car brakes early at every corner. | Achieved speed drops before planned in every braking zone (s 16–22, 33–39, 45–50); acceleration tracks well. |
| **median → best** | +0.17 s | **Lateral.** Intermittent excursions. | Every fast lap held \|e\|max ≤ 0.10 m; every slow lap had an excursion to 0.23–0.60 m. |

0.31 + 0.17 = 0.48 s available, against the 0.42 s that 11.50 requires. Both budgets are real and
neither alone is quite enough — but the lateral one also closes out the collision, which is a
hard gate, not a trade.

---

## 2. Where everything is

| What | Path |
|---|---|
| The controller | `devkit_ws/src/racer_control/racer_control/pure_pursuit.py` |
| Its defaults | `devkit_ws/src/racer_control/config/pure_pursuit.yaml` |
| Launch overrides (`TUNABLES`) | `devkit_ws/src/racer_control/launch/follower.launch.py` |
| Per-track registry | `devkit_ws/src/racer_common/racer_common/frames.py` |
| Racelines | `raceline/icra2026/raceline_*.csv` — cols `s,x,y,psi,kappa,w_right,w_left,v_mps` |
| Raceline solver | `raceline/optimize_raceline.py` |
| Run analysis (text) | `raceline/analyze_run.py` |
| **Gap analysis + charts** | `raceline/plot_speed_tracking.py` |
| **A/B overlay charts** | `raceline/compare_runs.py` |
| Run logger | `raceline/log_run.py` (copy lives at `dev_ws/log_run.py`) |
| **Safe stop** | `raceline/stop_follower.sh` (copy at `dev_ws/stop_follower.sh`) |
| Measured vehicle model | `raceline/VEHICLE_MODEL.md` |
| Prior findings | `raceline/FINDINGS.md` |

**Host note.** Runs happen on the macOS + Rosetta box: native simulator app
(`autodrive_simulator 2/` — the *compete* build, the only one with the ICRA scene) plus the
devkit in the `autodrive_roboracer_api` container, bind-mounting `~/AutoDRIVE/dev_ws`.
The container's `frames.py` resolves racelines under `~/Documents/roboracer`, which does **not**
exist here — always pass `path_csv:=` explicitly.

---

## 3. Rules of the experiment

These are not bureaucracy; each one exists because ignoring it has already destroyed a run.

1. **Reset the sim before every run.** `lap_count` and `collision_count` are cumulative since
   the simulator started. A run launched on top of a previous one inherits its totals.
2. **Reset means GUI Reset *and* Connect.** A reset without the reconnect leaves the drive
   channel half-armed: commands are received, wheels animate, the car does not move.
3. **Verify the car is stopped, not just that nothing is publishing.** The sim latches the last
   throttle command. A killed follower leaves the car driving with `Publisher count: 0` on both
   command topics. Use `stop_follower.sh` — it kills *first*, then zeros, then checks
   `odom` `linear.x == 0.0`.
4. **One variable per run.** Pass it as a launch override so the yaml is untouched and the
   startup log records exactly what ran.
5. **20 laps, then stop.** The sim tick holds 17.7–19.4 Hz over 20 laps but sags to ~14 Hz in a
   long session, and lap times move with it. Never compare a lap from minute 2 to one from minute 9.
6. **Judge against the plan (11.441 s), not the previous run.** The plan does not drift.
7. **Zero collisions is a gate.** A faster median with a collision is a failed experiment.
8. **Discard the out-lap.** Lap 1 spans from sim reset to the first crossing and is meaningless.

### Canonical run commands

```bash
# 0. stop anything running, verify the car is actually still
docker exec autodrive_roboracer_api bash -c "bash /home/autodrive_devkit/dev_ws/stop_follower.sh"
# 1. GUI: Reset, then Connect. Confirm collision_count and lap_count both read 0.
# 2. logger
docker exec -d autodrive_roboracer_api bash -c \
  "source /opt/ros/humble/setup.bash && cd /home/autodrive_devkit/dev_ws && \
   python3 log_run.py <RUN_NAME>.csv --hz 20 > /tmp/<RUN_NAME>.log 2>&1"
# 3. follower — ONE override
docker exec -d autodrive_roboracer_api bash -c \
  "source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/dev_ws/install/setup.bash && \
   ros2 launch racer_control follower.launch.py track:=icra2026 \
     path_csv:=/home/autodrive_devkit/dev_ws/raceline/icra2026/raceline_a7.0.csv \
     <PARAM>:=<VALUE> > /tmp/<RUN_NAME>_foll.log 2>&1"
```

Analysis:

```bash
python3 raceline/plot_speed_tracking.py <RUN>.csv \
  --path raceline/icra2026/raceline_a7.0.csv \
  --slip-circle 0 --slip-accel 0.16 --slip-brake 0.08 --lap-lo 11.5 --lap-hi 12.6
python3 raceline/compare_runs.py --path raceline/icra2026/raceline_a7.0.csv \
  base.csv:"baseline" test.csv:"target_lead_s 0.0"
```

> **Tool caveat.** `plot_speed_tracking.py` needs to be told the band that was actually in force.
> With `slip_circle > 0` pass `--slip-circle <v>`; with it off pass `--slip-circle 0` *and* the
> `--slip-accel/--slip-brake` in use. Getting this wrong silently reports ~90 % band saturation
> everywhere. Likewise its "follower clip" term is only meaningful because `v_target` is shifted
> back by the command lead first — see §6.2.

---

## 4. What the state of the art actually does

Our controller is a geometric tracker, and the literature on geometric trackers is small and
settled. Worth knowing what is genuinely SOTA versus what is folklore.

**Classic pure pursuit** (Coulter, CMU 1992) is what we run: fit a circular arc from the rear
axle through a point `Ld` ahead, command `κ = 2y'/Ld²`. Ollera's stability analysis shows it
converges on straight and constant-curvature paths. It is *velocity-independent by construction* —
speed enters only through whatever you do to `Ld`.

**Adaptive Pure Pursuit (APP)** — MIT's DARPA Urban Challenge entry — makes the lookahead
proportional to speed, `Ld = v · l_t`, where `l_t` is a gain in *seconds*: how far forward in time
you project. This is exactly our `lookahead_k` (0.55 s). This is the de-facto standard and we
already have it.

**Regulated Pure Pursuit (RPP)** — the Nav2 default — adds regulation heuristics on top:
a **curvature heuristic** that scales speed down as `1/(r_min·κ)` above a curvature threshold,
and a proximity heuristic for obstacles, taking the max of the two. Its important structural
insight for us: *the angular velocity is computed from the **regulated** velocity, not the desired
one*, which removes undershoot against the target curvature. We have the analogous ideas in
`lookahead_curv_gain` (shorten `Ld` in corners) and `steer_a_lat_max` (cap commanded curvature at
`a_cap/v²`) — the latter is arguably stronger than RPP's, because it is expressed in the physical
quantity that actually saturates, lateral acceleration.

**Where we are actually behind the literature:** every one of our lookahead numbers is a single
global constant. The racing work has moved to *per-location* lookahead:

- **Adaptive Lookahead Pure-Pursuit** (Shalev-Shwartz et al., F1TENTH, arXiv 2111.08873) assigns
  an optimal lookahead **per waypoint** with a greedy search over the reference trajectory, and
  reports ~**20 % improvement** in racing metrics on a real F1/10 car versus speed-proportional
  lookahead.
- **RL-tuned pure pursuit** (PPO, IEEE 2026) chooses lookahead *and* steering gain online from
  compact state features (speed plus curvature taps), trained in F1TENTH Gym and deployed in ROS 2,
  with no per-map retuning.

**What this means for our plan.** We should not jump to RL. The honest reading is that our
lateral controller has the right *structure* (speed-proportional + curvature regulation + a
physical saturation cap) and the open gap is that the constants are global while the failure is
local — our excursions cluster at specific track locations. The cheap version of the SOTA idea
is a **per-zone lookahead**, mirroring the `lat_zones` / `margin_zones` mechanism the raceline side
already uses. That is E5 below.

**Longitudinal.** The racing literature tracks a velocity profile generated under a friction-circle
budget, and the standard structure is **feedforward from the plan plus feedback on the error** —
which is what `accel_ff` implements, and what the friction-circle band (`slip_circle`) is a
principled form of. Both exist in our tree already. The relevant caution from the same literature
is that combined-slip coupling means longitudinal demand *steals* from lateral capacity, which is
precisely why raising `v_max` cost us T2 (§9, rejected R3).

Sources:
[Regulated Pure Pursuit](https://arxiv.org/pdf/2305.20026) ·
[Adaptive Lookahead Pure-Pursuit](https://arxiv.org/pdf/2111.08873) ·
[Learning to Tune Pure Pursuit with PPO](https://ieeexplore.ieee.org/document/11419795/) ·
[Coulter, Implementation of Pure Pursuit](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf) ·
[Autonomous Vehicle Racing survey](https://arxiv.org/pdf/2202.07008)

---

## 5. Lateral tuning

### 5.1 Parameters

| Parameter | Now | Effect of increasing | Watch for |
|---|---|---|---|
| `lookahead_k` | 0.55 | Longer `Ld` at speed — smoother, more corner-cutting | Apex cutting |
| `lookahead_max` | 2.20 m | Ceiling; raises straight-line stability | Corner-cutting; too low → weave |
| `lookahead_min` | 0.80 m | Floor at low speed | Too low → chases noise |
| `lookahead_curv_gain` | 0.67 | Shortens `Ld` harder in corners | Too high → nervous in corners |
| `lookahead_sag_frac` | 0.5 | Allows more chord bulge before capping | Cutting inside the margin |
| `steer_a_lat_max` | 7.0 | Allows more commanded curvature | **Load-bearing — see below** |
| `steering_gain` | 1.0 | Scales the whole steer command | Trim only; 1.0 = trust the model |
| `latency_comp_s` | 0.05 s | Predicts pose further forward | Over-prediction → lead/oscillation |

### 5.2 Symptom → lever

| Symptom (from `analyze_run.py`) | Read | Lever |
|---|---|---|
| Weave > 1.2 sign-changes/s at v > 4 | `Ld` too short on straights | raise `lookahead_max`, or `lookahead_k` |
| Corner bias **positive** (inside) | `Ld` too long in corners | raise `lookahead_curv_gain`, lower `lookahead_k` |
| Corner bias **negative** (wide) | Tire, not controller | lower the lateral rung on the *line*; do **not** add steering |
| Excursion only at specific `s` | Local, not global | per-zone lookahead (E5), or a `margin_zones` entry on the line |
| Wide at full lock, growing | Past the lateral force peak | this is what `steer_a_lat_max` exists to prevent — do not raise it |

> **`steer_a_lat_max` is not a tuning knob.** Front lateral force peaks near 1.0 g at ~0.57° slip
> and is down to 0.5 g by 5.7°. Past the peak, more steering means *less* force: the car runs
> wider, the controller steers harder, and it spirals — measured 0.2 m → 0.9 m wide in one second
> at the 6.5 rung. Raising this cap to 8.5 produced **146 collisions and no completed laps.**

### 5.3 Procedure

1. Run the baseline, get the per-lap `|e|max` table from `plot_speed_tracking.py`.
2. Separate **global** from **local**: if slow laps differ from fast laps only at one or two `s`
   ranges, it is local — go to E5 and do not touch the global constants.
3. If global, sweep `lookahead_max` first (it has the widest effect and the clearest symptom),
   then `lookahead_curv_gain`.
4. Re-check corner bias sign after every change; a fix that flips bias from wide to cutting has
   overshot.

---

## 6. Longitudinal tuning

### 6.1 Parameters

| Parameter | Now | Effect of increasing | Watch for |
|---|---|---|---|
| `target_lead_s` | 0.08 s | Reads the profile further ahead → brakes **earlier** | The current suspect |
| `cmd_delay_s` | 0.175 s | Same, plus shifts the band centre | Auto-estimated; measured 0.085–0.140 here |
| `slip_accel` | 0.16 | More acceleration authority | > 0.15 is past the friction peak |
| `slip_brake` | 0.08 | More braking authority | Deliberately low — high-slip braking loses the tire |
| `slip_kp` | 0.76 | Harder correction inside the band | Oscillation |
| `slip_circle` | 0 (off) | Replaces the fixed band with a load-scaled one | **Replaces `slip_accel` entirely** |
| `accel_ff` | 0 (off) | Feeds plan acceleration forward | Accel side only; brake side cost 0.09 s |
| `v_max` | 8.0 | Unclips the plan | Moves braking points — see R3 |

### 6.2 How to read the gap

`plot_speed_tracking.py` splits the gap into two halves with different owners:

- **follower clip** (`v_plan → v_target`): what the follower's own limits rewrote. Only meaningful
  because `v_target` is first shifted back by the command lead — the follower samples the profile
  ahead *on purpose*, and comparing it to the plan at the same `s` scores that as a loss on every
  braking zone and a gain on every acceleration.
- **delivery** (`v_target → v_true`): the car not producing the speed it was asked for. This is
  the controller's half.

Then look at the **speed profile chart**, which is the one that actually diagnoses:

- Achieved **below** plan while accelerating → not enough authority (band, or feedforward).
- Achieved **dropping before** plan in braking zones → reading the profile too far ahead
  (`target_lead_s`).
- Achieved **above** plan entering a corner → arriving hot; braking point is wrong for the speed
  actually reached. This is what killed the `v_max 9.0` experiment.

### 6.3 Procedure

1. Confirm the shape first from the profile chart — early braking, weak acceleration, or hot entry.
   Do not sweep a parameter before you know which of the three you have.
2. Longitudinal changes that move **braking points** (`v_max`, `target_lead_s`) must be checked at
   corner *entry clearance*, not just lap time.
3. Band changes (`slip_accel`, `slip_circle`) should be checked against **saturation percentage**,
   not lap time alone: if the band is pinned < 30 % of the time it is not the limiter and widening
   it will do nothing.

---

## 7. Experiment queue

Ordered by expected value ÷ risk. Each is one variable. Fill the results matrix in §8 as you go.

| ID | Change | Hypothesis | Predicted | Risk |
|---|---|---|---|---|
| **E1** | `target_lead_s:=0.0` | 0.08 is Porto's value; at 8 m/s it reads ~2.1 m into a ~7.8 m braking zone, so the car brakes ~¼ of the zone early at every corner | −0.28 s | Low. Braking later *on the plan*, not harder. Watch T2 entry clearance. |
| **E2** | `target_lead_s:=0.04` | If E1 overshoots (hot entries), the optimum is between | between E1 and base | Low |
| **E3** | `slip_accel:=0.20` | Re-confirm the 0.16→0.20 plateau now that the gap is properly measured | ~0 | Low. Pure confirmation; skip if E1 lands. |
| **E4** | `accel_ff:=1.0` (circle still off) | Plan-acceleration feedforward without the circle band. multi-track measured accel delivery 91 %→98 %. Never tested here in isolation. | −0.05 to −0.10 s | Medium. Untested on this host *without* `slip_circle`. |
| **E5** | Per-zone lookahead | Excursions are local, not global. Mirror `lat_zones`: shorter `Ld` only at the `s` ranges where `\|e\|` spikes | −0.10 to −0.17 s, and the collision | Medium. Needs a small code change; see §7.1. |
| **E6** | `lookahead_max` sweep 2.0 / 2.2 / 2.4 | Global fallback if E5's excursions turn out not to be local | unknown | Low |
| **E7** | `enc_window_s:=0.0` vs `0.05` | A/B the encoder fix on the *current* stack, now that it is launch-exposed | confirms 3.7 s claim | Low. Diagnostic, not a candidate. |

### 7.1 Note on E5

`lat_zones` and `margin_zones` already establish the pattern of per-`s` overrides in
`frames.py`, parsed as `s0:s1:value` lists. A `lookahead_zones` entry parsed the same way and
applied as a cap on `Ld` inside those ranges is the cheapest possible version of the per-waypoint
lookahead the F1TENTH adaptive-lookahead work does with a greedy search. Do E1–E4 first: if the
longitudinal budget alone reaches 11.50 with zero collisions, E5 is optional.

---

## 8. Results matrices — fill these in

### 8.1 Run log

One row per run. **`Coll` is the per-run delta**, not the cumulative counter.

| Run | Date | Line | Change (one variable) | Laps | Best | Median | Mean | Coll | Gap vs 11.441 | Verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| `icra_base20_a70` | 2026-09-08 | a7.0 | *baseline, yaml defaults* | 20 | 11.75 | 11.92 | 11.92 | 1 | +0.48 | reference |
| `E1` | | a7.0 | `target_lead_s:=0.0` | | | | | | | |
| `E2` | | a7.0 | `target_lead_s:=0.04` | | | | | | | |
| `E3` | | a7.0 | `slip_accel:=0.20` | | | | | | | |
| `E4` | | a7.0 | `accel_ff:=1.0` | | | | | | | |
| `E5` | | a7.0 | per-zone lookahead | | | | | | | |
| `E6` | | a7.0 | `lookahead_max:=…` | | | | | | | |

### 8.2 Section-level time loss (from `plot_speed_tracking.py`)

Seconds lost to the plan per section. Corners shaded in the chart; `s` ranges are on the a7.0 line.

| Section | Baseline | E1 | E2 | E4 | E5 |
|---|---|---|---|---|---|
| straight 1–21 m | | | | | |
| corner 21–24 m (T2) | | | | | |
| straight 24–37 m | | | | | |
| corner 39–41 m | | | | | |
| straight 41–43 m | | | | | |
| corner 43–46 m | | | | | |
| straight 46–54 m | | | | | |
| **total** | **+0.48** | | | | |

### 8.3 Lateral excursion matrix

Where the car leaves the line, and by how much. Fill from the per-lap `|e|max` table.

| Run | \|e\| mean | \|e\| p90 | \|e\| max | # laps with \|e\| > 0.25 m | `s` of worst excursion | Corner bias (+in/−wide) |
|---|---|---|---|---|---|---|
| baseline | | | 0.60 | 7 of 21 | | |
| E1 | | | | | | |
| E5 | | | | | | |

### 8.4 Band saturation

Only meaningful with the right `--slip-*` flags. > 30 % means the band is a real limiter.

| Run | Band in force | Straight 1–21 | Corner 21–24 | Straight 24–37 | Corner 43–46 |
|---|---|---|---|---|---|
| baseline | ±0.16 / −0.08 | 25 % | 12 % | 18 % | 12 % |
| E3 | ±0.20 / −0.08 | | | | |
| E4 | ±0.16 + ff | | | | |

### 8.5 Decision gate

An experiment is **accepted** only if all four hold:

- [ ] collisions in run = **0**
- [ ] median improves by ≥ 0.03 s, or median holds and `|e|` p90 improves
- [ ] no section in §8.2 gets worse by > 0.03 s
- [ ] minimum corner clearance not reduced below the previous accepted run

---

## 9. Already tried — do not repeat

| ID | Change | Result | Why it failed |
|---|---|---|---|
| **R1** | multi-track follower config (`slip_circle 0.12` + `accel_ff` + `lookahead_max 2.6` + their line) | **11 collisions in 26 laps**, median 12.1 | Validated on the Linux box. Does not transfer to this host. |
| **R2** | `observer_wheels:=4` | +0.02 m/s bias (works), but understeer into the exit wall | The one-wheel optimism was doubling as a throttle limiter |
| **R3** | `v_max:=9.0` | Straight unclipped (8.07 → 8.90 m/s), **6 collisions, all at T2** | Car reaches only ~8.2 of the planned 9.0, so it crosses into T2 above the profile with less distance to shed |
| **R4** | `steer_a_lat_max:=8.5` | 146 collisions, no laps | Past the lateral force peak |
| **R5** | `accel_ff` on the **brake** side | −0.09 s slower | Tracked the plan's deceleration instead of its speed; every apex 0.1 m/s lower |
| **R6** | 1 m minimum-preview for `v_target` | ~0.55 s/lap | Brakes for each corner twice; the profile already has its braking distances |
| **R7** | multi-track's line (`…corners_h.csv`) | plans 11.447 vs a7.0's 11.441 | No gain available; strictly harder to drive here |

---

## 10. Known measurement traps

Each of these has already produced a confidently wrong number.

1. **Cumulative counters.** `collision_count` / `lap_count` never reset except on sim reset.
   Always report per-run deltas. The logger records the starting value explicitly.
2. **The latched throttle.** `Publisher count: 0` does not mean the car stopped.
3. **Band-saturation flags.** Passing the wrong `--slip-*` to `plot_speed_tracking.py` reports
   ~90 % saturation everywhere (it scores against a 0.02 floor). Real baseline figure is 12–29 %.
4. **The command lead is not a loss.** Comparing `v_target` to `v_plan` at the same `s` without
   shifting by `v·(cmd_delay+target_lead)+lead_min` scores the deliberate lead as loss on braking
   and as gain on acceleration.
5. **Line auto-detection is ambiguous.** Several a7.0 variants sit within 0.04 m of each other.
   Always pass `--path` explicitly.
6. **Session length.** Compare only runs of the same length, from a fresh reset.
7. **`/odom` is ground truth and restricted.** Every number in this document is a development
   number. Race-legal runs need `use_tf_pose:=true` and the localizer.

---

## 11. Definition of done

- 20 consecutive laps, fresh reset, **0 collisions**
- median ≤ 11.50 s on `raceline_a7.0.csv`
- the accepted configuration written into `frames.py`'s `icra2026` `follower` dict, with the
  measurement that justifies each value in the comment beside it
- `VEHICLE_MODEL.md` §7 updated with the run history
- §8 matrices in this file filled, including the rejected experiments
