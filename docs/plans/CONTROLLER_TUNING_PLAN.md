# Follower tuning plan — ICRA 2026 track

**Owner:** Adil (pure pursuit controller + speed-profile matching)
**Branch:** `adil-icra-longitudinal`
**Goal:** **0 collisions** and **≤ 11.50 s** on the ICRA 2026 track.
**Status (2026-09-09):** reference run is **`E0_base`** — 25 laps, median **11.800 s**, best
11.650, **0 collisions**. Gap to the plan: **+0.360 s**. One experiment run and rejected (E0/R8).
Plan audited against the code on 2026-09-09; audit corrections are marked **[audit 09-09]**.

Companion: [`follower-internals.html`](follower-internals.html) — interactive explanation of
the geometry, the lookahead chain, profile sampling and the tire model. Open it first if you
have not read the controller before. This document is the *plan*; that one is the *map*.

---

## 0. Executive summary — the experiment table

**This table is the plan.** Everything below it is evidence for a row in it. Priority is
expected impact ÷ risk, re-sorted after every run. Fill **Result** the day the run happens and
write the detail into §12.

**Where we stand:** plan **11.440 s**, we deliver **11.800 s** median, 0 collisions.
Gate is **≤ 11.50 s with 0 collisions**, so we need **−0.30 s** and must not spend the clean sheet.

| # | ID | Existing (what runs today) | Proposed change | Reason — the measured evidence | Expected / calculated impact | Result |
|---|---|---|---|---|---|---|
| **1** | **E4** | `accel_ff: 0.0` — no plan-acceleration feedforward. Throttle reacts only to a *speed error* that has already happened. | `accel_ff:=1.0`, `slip_circle` left at 0 | **78 % of all time lost is on accelerating stretches** (+0.526 s vs +0.106 s braking). There the car sits **−0.784 m/s below its own target** while `v_target` is *above* plan, so the target is not the limiter. Not authority either: throttle median **0.203**, at 1.0 on **0.0 %** of ticks. Not the band: realized slip p90 **0.160**, already on the friction curve's flat top. The plan asks only **2.49 m/s² mean** (max 4.12) against a ~7 tire peak and the car delivers **1.64, i.e. 66 %**. Feedforward is the one mechanism that asks for the plan's acceleration *before* the error appears. | **−0.15 to −0.25 s.** Closing delivery 66 % → 90 % recovers ~0.35 s of the +0.526 s accelerating loss; halved for the usual gap between mechanism and outcome. | |
| **2** | **E8** | `lookahead_sag_frac: 0.5` — the chord is allowed to sag inside by up to 50 % of the line's own inside margin. | `lookahead_sag_frac:=0.35` | The **only** real lateral failure is apex cutting at **s 43–46**: worst inside excursion **+0.525 m at s 46.2** in the clean reference, and when E0 doubled the sag it became **+0.700 m and 33 collisions in that exact bin**. R8 proved the mechanism (`sag = \|κ\|·Ld²/8`) is what decides that corner. This caps the same quantity directly and in the physical unit — fraction of *actual* margin — instead of guessing a length. Launch-exposed, no code change. | **−0.00 to −0.05 s**, and the excursion tail shrinks. Bought as **insurance on the collision gate**, not as lap time. | |
| **3** | **E9** | `lookahead_curv_gain: 0.67` — `Ld` divided by `1 + 0.67·\|κ\|` inside the lookahead. | `lookahead_curv_gain:=0.90` | Same target as E8, different lever, and §5.2's prescribed response to **positive (inside) corner bias**, which is what s 43–46 shows (`+0.026` mean, `+0.525` worst). Run only if E8 misses — they act on the same failure and running both at once breaks one-variable. | **−0.00 to −0.05 s**; same insurance argument as E8. | |
| **4** | **E1** | `target_lead_s: 0.08` — profile sampled `v·(cmd_delay + 0.08) + 0.10` ahead. | `target_lead_s:=0.0` | Braking zones hold **16 %** of the loss (+0.106 s). Of the 1.69 m total lead at 8 m/s only **0.64 m** is this parameter; the rest is `cmd_delay` and is deliberate. That is ~10 % of the 6.10 m main zone but **40 % of the 2.50 m zone at s 42–45**, so the gain is concentrated in the short zones. | **−0.03 to −0.05 s.** (Was −0.28 s in the original plan; that used a 2.1 m lead at a `cmd_delay` of 0.175 that does not run, against a 7.8 m zone that does not exist.) | |
| **5** | **E2** | as E1 | `target_lead_s:=0.04` | Only if E1 overshoots into hot corner entries. The midpoint. | between E1 and reference | |
| **6** | **E5** | Lookahead constants are **global**; `lat_zones` / `margin_zones` already establish per-`s` overrides on the raceline side. | Add `lookahead_zones` (`s0:s1:cap`), applied as a cap on `Ld` | Excursions are **local, not global** — 165 baseline ticks in three clusters (s 0–10, s 20–26, s 40–48) and nothing between. E0's collisions localised harder still: **33 of 49 in s 44–46**. This is the cheap version of the F1TENTH per-waypoint adaptive lookahead (§4). **Needs a code change**, so it runs after E8/E9 show whether local lateral work pays at all. | **−0.05 to −0.15 s** and the excursion tail | |
| **7** | **E10** | `latency_comp_s: 0.05` — pose propagated 50 ms forward before steering. **Fixed; not delay-scaled.** | `latency_comp_s:=0.03` | Tuned at a **175 ms** round trip (yaml: 6.60/6.70 vs 6.70/6.76 at 0, runs 24–25). This host measures **97 ms**. Unlike the lookahead, this parameter has **no automatic delay compensation**, so it is over-predicting by roughly the delay ratio. ⚠️ **This is the same reasoning shape that produced R8** — the difference is that R8 removed a compensator that was working, and here there is no compensator at all. Rank held low until E4/E8 land. | **−0.00 to −0.04 s.** Low confidence, explicitly flagged. | |
| **8** | **E3** | `slip_accel: 0.16` | `slip_accel:=0.20` | Confirmation only, and now **predicted from the log rather than guessed**: realized slip p90 is already **0.160**, on the curve's flat top (0.10–0.18), so there is nothing above it to buy. Keep it in the queue so the plateau is on record, not because it should pay. | **~0.** | |
| **9** | **E7** | `enc_window_s: 0.05` | A/B `0.0` vs `0.05` | Diagnostic, not a candidate: confirms the encoder-window fix on the current stack. | confirms the 3.7 s claim | |
| — | ~~E0~~ | ~~`lookahead_delay_ref: 0.175`~~ | ~~`:=0.0`~~ | ~~Effective `Ld` is 1.48 m, not the documented 2.20 m~~ | ~~−0.10 to −0.17 s + the collision~~ | **❌ REJECTED 09-09 — 49 collisions, median 11.799 vs 11.800. See R8 and §12.1.** |
| — | ~~E6~~ | ~~`lookahead_max: 2.20`~~ | ~~sweep 2.0/2.2/2.4~~ | ~~global fallback if E5 is not local~~ | — | **❌ DROPPED — R8 showed `lookahead_max` is not the lever on a 97 ms host.** |

### Standing rules for this table

1. **Everything gets logged here.** Every run — accepted, rejected or aborted — gets its
   **Result** cell filled the same day and a full record in **§12**, with the launch command, the
   raw lap list, the log excerpt and the chart. A run that is not written down did not happen,
   and the reason we can reject E0 cleanly is that E0's control was written down.
2. **Re-sort after every run.** E0 moved E5 *up* by failing. Priorities are posterior, not fixed.
3. **Record the prediction before the run and score it after.** E0's prediction was wrong in
   sign; that is only visible because it was written down first.
4. Nothing enters this table without a measured number in the **Reason** column.

---

## 1. The one-paragraph version

The raceline file already contains the speed we should be driving. `raceline_a7.0.csv` plans
**11.441 s**. We drive it in **11.75 s at best, 11.92 s median**, with **1 collision in 20 laps**.
So the raceline is not the bottleneck and does not need to get faster — the follower needs to
deliver the profile it is already given. The gap splits cleanly into two budgets that need
separate experiments:

| Budget | Size | Nature | Evidence |
|---|---|---|---|
| **best → plan** | +0.31 s | **Longitudinal.** Speed is held *below* plan through every braking zone. | See the onset table below. |
| **median → best** | +0.17 s | **Lateral.** Intermittent excursions. | Every fast lap held \|e\|max ≤ 0.10 m; every slow lap had an excursion to 0.23–0.77 m. |

0.31 + 0.17 = 0.48 s available, against the 0.42 s that 11.50 requires. Both budgets are real and
neither alone is quite enough — but the lateral one also closes out the collision, which is a
hard gate, not a trade.

> **[audit 09-09] The longitudinal mechanism is not "brakes early".** This row used to read
> *"Car brakes early at every corner — achieved speed drops before planned in every braking
> zone."* Measured over the 15 clean laps of `icra_base20_a70.csv`, that is false: braking
> **onset** in `s` is at or *after* the plan's, never before.
>
> | Zone | plan brakes at | `v_target` drops at | achieved drops at | mean v: plan / target / achieved |
> |---|---|---|---|---|
> | T2, s 14–24 | 16.70 | 15.50 | **16.80** | 5.67 / 5.27 / 5.43 |
> | s 31–40 | 33.40 | 32.50 | **34.30** | 5.22 / 4.83 / 4.84 |
> | s 48–54 | 50.80 | 50.10 | **51.90** | 5.24 / 5.02 / 4.87 |
>
> (onset = first point 0.3 m/s below the window's local max.) The real mechanism: sampling a
> *descending* profile `lead` metres ahead returns a lower value at **every point** of the
> descent, so `v_target` is depressed across the whole zone — and the car tracks that depressed
> target closely (4.84 vs 4.83, 4.87 vs 5.02). The loss is a **magnitude** error in `v_target`,
> not a **position** error in the braking point. That distinction decides what E1 is allowed to
> claim, and it means the fix is bounded by how much of `lead` is discretionary — see E1.

> **[audit 09-09] And the longitudinal loss is not in the braking zones at all.** Splitting the
> baseline's loss by what the *plan* is doing at each point:
>
> | Plan is… | track | time lost | share | mean v deficit |
> |---|---|---|---|---|
> | **accelerating** (dv/ds > +0.05) | 30.6 m | **+0.526 s** | **78 %** | +0.502 m/s |
> | braking (dv/ds < −0.05) | 20.0 m | +0.106 s | 16 % | +0.146 m/s |
> | cruise / flat | 3.5 m | +0.046 s | 7 % | +0.745 m/s |
>
> §1 used to assert *"acceleration tracks well"*. It does not — the deficit while accelerating is
> **3.4× the deficit while braking**. And on those accelerating stretches `v_target` is
> **+0.282 m/s ABOVE plan** (the lead reads *up* a rising profile, so it helps), while achieved is
> **−0.784 m/s below its own target**. So on 78 % of the loss the target is not the limiter and
> `target_lead_s` is not the lever — **delivery** is.
>
> What is limiting it is neither throttle authority nor the band's width:
>
> - throttle median **0.203**, p90 0.295, at 1.0 on **0.0 %** of ticks — nowhere near saturated;
> - realized slip median 0.136, p90 **0.160**, i.e. already sitting on the `slip_accel` edge and
>   on the flat top of the friction curve (0.10–0.18 per the yaml), so E3's widening to 0.20
>   pushes past the peak and should measure ~0 — the plan's E3 prediction is right, for a reason
>   it did not state;
> - **but the plan's demand is modest**: mean `a_long` **2.49 m/s²**, p90 3.54, max 4.12 — against
>   a tire peak near 7. The car delivers **1.64 m/s², 66 % of plan**. There is real headroom and
>   the band is not what withholds it.
>
> That is the exact gap `accel_ff` exists to close (§7 E4 quotes 91 % → 98 % delivery elsewhere).
> **E4 is therefore the highest-value longitudinal experiment and E1/E2 the lowest.** The queue in
> §7 is reordered accordingly.

> **[audit 09-09] The lateral budget has a cause that was not in this document.** The lookahead
> actually in force is 30 % shorter than §5.1 says, at all times, because of
> `lookahead_delay_ref`. See the boxed note in §5.1 and **E0** in §7 — this is now the first
> experiment, ahead of E1.

---

## 2. Where everything is

| What | Path |
|---|---|
| The controller | `devkit_ws/src/racer_control/racer_control/pure_pursuit.py` |
| Its defaults | `devkit_ws/src/racer_control/config/pure_pursuit.yaml` |
| Launch overrides (`TUNABLES`) | `devkit_ws/src/racer_control/launch/follower.launch.py` |
| Per-track registry | `devkit_ws/src/racer_common/racer_common/frames.py` |
| Racelines | `raceline/icra2026/raceline_*.csv` — cols `# s_m,x_m,y_m,psi_rad,kappa_radpm,w_right_m,w_left_m,v_mps` |
| Raceline solver | `raceline/optimize_raceline.py` |
| Run analysis (text) | `raceline/analyze_run.py` |
| **Gap analysis + charts** | `raceline/plot_speed_tracking.py` |
| **A/B overlay charts** | `raceline/compare_runs.py` |
| **A/B report (numbers + 5 figures + gate)** | `raceline/ab_report.py` — **[audit 09-09]** new; prints §8's tables and evaluates §8.5 |
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
5. **25 laps, then stop.** The sim tick holds over a short session but sags in a long one, and lap
   times move with it. Never compare a lap from minute 2 to one from minute 9, and never compare a
   20-lap run to a 25-lap one. **[audit 09-09]** Raised from 20 to 25 so each arm has enough clean
   laps for the median to be worth a 0.03 s decision gate; the baseline's 21 laps yielded only 15
   clean ones. The observable is `pp_delay`, not the tick — record its median and p90 per run.
6. **Judge against the plan (11.441 s), not the previous run.** The plan does not drift.
7. **Zero collisions is a gate.** A faster median with a collision is a failed experiment.
8. **Discard the out-lap.** Lap 1 spans from sim reset to the first crossing and is meaningless.
9. **Every run is written into this document, the same day.** [2026-09-09] Fill the **Result**
   cell in §0 and add the full record to §12: launch command as issued, the override-confirmation
   line, raw per-lap times, the log excerpt, the chart, and what it means. **Aborted and failed
   runs included** — E0 is the most useful entry in §12 precisely because it failed, and it is
   only usable because its control run was recorded first. No run is exempt for being
   uninteresting: "no change" is a measurement.
10. **Write the prediction down before the run, and score it after.** §0 carries the predicted
   impact; §12 carries what actually happened and whether the prediction held. E0's prediction was
   wrong in sign, which is knowable only because it was recorded in advance.

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
  --slip-circle 0 --slip-accel 0.16 --slip-brake 0.08 \
  --cmd-delay 0.12 --lap-lo 11.5 --lap-hi 12.6
python3 raceline/compare_runs.py --path raceline/icra2026/raceline_a7.0.csv \
  base.csv:"baseline" test.csv:"target_lead_s 0.0"
```

> **Tool caveat.** `plot_speed_tracking.py` needs to be told the band that was actually in force.
> With `slip_circle > 0` pass `--slip-circle <v>`; with it off pass `--slip-circle 0` *and* the
> `--slip-accel/--slip-brake` in use. Getting this wrong silently reports ~90 % band saturation
> everywhere.
>
> **[audit 09-09] `--cmd-delay` is not optional and the default is wrong.** It defaults to
> `0.15`, but the measured `pp_delay` on this host is **median 0.119 s** (p10 0.090, p90 0.138,
> max 0.175 over 5372 ticks of the baseline). Read the run's own `pp_delay` column and pass its
> median. And note the tool's lead term is **incomplete** — it omits `target_lead_s` entirely,
> so the "follower clip" number is biased. See trap 4 in §10 before quoting it.

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
| `lookahead_delay_ref` | 0.175 s | Scales `k` and `max` by `cmd_delay/ref`, clamped [0.7, 1.2] | **Currently floored — see below** |

> ### [audit 09-09] The lookahead in this table is not the lookahead that runs
>
> `pure_pursuit.py:1059-1061`:
>
> ```python
> d_scale = max(0.7, min(1.2, self.cmd_delay / self.ld_delay_ref))
> ld = clip(self.ld_k * d_scale * abs(self.speed), self.ld_min, self.ld_max * d_scale)
> ```
>
> `ld_delay_ref` is 0.175 s, but the measured `cmd_delay` on this host is **0.119 s median**
> (never above 0.175 in 5372 baseline ticks). `0.119/0.175 = 0.68`, so **`d_scale` sits pinned
> at its 0.70 floor for the entire run**:
>
> | | yaml says | actually in force |
> |---|---|---|
> | `lookahead_k` | 0.55 | **0.385** |
> | `lookahead_max` | 2.20 m | **1.54 m** |
>
> Confirmed in the log, not just inferred: `pp_ld` at v > 6 m/s has **median 1.499 m, p90 1.521**,
> flat against the 0.70 × 2.20 = 1.540 ceiling.
>
> This matters because `pure_pursuit.yaml` records, in its own words, why 2.20 was chosen:
> *"max 2.2, not 1.6: at 6.7 m/s on the straights the shorter value weaved (yaw ±9 deg in 0.2 s,
> 51 deg of phase lag at a 175 ms round trip) and clipped a wall 0.29 m from the line."* We are
> running **1.54 m**. The documented failure mode of a short lookahead — weave on the straights,
> occasional wall contact — is the failure mode we have.
>
> The scaling was written for a host whose round trip was three sim frames at 17.5 Hz. That is
> not this host any more, and the clamp floor has turned a compensator into a constant 0.70
> derate. `lookahead_delay_ref` is launch-exposed, so testing it is one override: **E0**.
>
> ### [2026-09-09, after the run] E0 was tested and REJECTED — the 0.70 floor is load-bearing
>
> **The measurement above is correct and the inference from it was wrong.** The effective
> lookahead really is 1.48 m rather than 2.20 m, and `lookahead_delay_ref:=0.0` really does
> restore 2.09 m (measured, both arms below). But restoring it is not an improvement — it is
> **49 collisions**. See **R8** in §9.
>
> The reasoning error: `lookahead_max: 2.20` was tuned *at the reference delay*, where
> `d_scale = 1.0`. It is the value the geometry wants **when the round trip is 175 ms** and the
> phase lag needs that much lookahead to stay stable. This host now runs at **97 ms**. There is
> less lag to buy margin against, so the extra length buys nothing on the straights and spends
> itself cutting apexes: the chord sagitta goes as `|κ|·Ld²/8`, so 1.48 → 2.09 m **doubles the
> cut**, 0.218 → 0.438 m at κ = 0.8, against a line that runs 0.28 m from the inside wall there.
>
> So `lookahead_delay_ref` is **doing its job**, and the floor is what keeps the geometry sane on
> a fast host. "Configured 2.20" and "effective 1.48" are not a discrepancy to fix; 2.20 is the
> value *at the reference delay* and this document should say so wherever it quotes it.
>
> **What stands from the audit:** `pp_delay` must be logged per run and the arms kept in parity
> (trap 7) — it rescales the geometry, and the two runs here differed by only 0.003 s, which is
> why the comparison is clean enough to reject on. **What falls:** E0, and E6's premise that
> `lookahead_max` is the lever. On this host the lever, if any, is the *floor* (0.70) or
> per-zone lookahead (E5) — and E5's case is now stronger, because the failure is at
> **s 43–46 specifically**, not everywhere.
>
> It also invalidates **E6** as written: sweeping `lookahead_max` 2.0 / 2.2 / 2.4 actually sweeps
> **1.40 / 1.54 / 1.68** and never reaches any tuned value. Do E0 first, or sweep with
> `lookahead_delay_ref:=0`.

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
| `derate_delay_from` / `_to` / `derate_a_lat` | 0.175 / 0.21 / 6.0 | Scales **all** of `v_target` down on a slow loop | **Armed but inert here — see below** |

> **[audit 09-09] The derate was missing from this plan and it is not harmless in principle.**
> `pure_pursuit.py:1142-1145` multiplies `v_target` by `sqrt(a_eff / profile_a_lat)` whenever
> `cmd_delay > derate_delay_from`, where `a_eff` slides from the profile's own lateral limit down
> to `derate_a_lat`. It gates on `profile_a_lat > derate_a_lat`, and a7.0 measures
> `max(v²·|κ|) = 7.0001 > 6.0`, so **the gate is armed on this line.** At full derate it would
> scale every target by `sqrt(6.0/7.0) = 0.926` — about **+0.9 s a lap**, and it would look
> exactly like "the follower is slow everywhere".
>
> It did **not** fire in the baseline: `cmd_delay` never exceeded 0.175, so the derate was active
> on **0.0 % of ticks**. Recorded here so nobody re-derives it, and so it is checked rather than
> assumed on any run where the tick sags (§3 rule 5). **Check `pp_delay` p90 on every run.**

### 6.2 How to read the gap

`plot_speed_tracking.py` splits the gap into two halves with different owners:

- **follower clip** (`v_plan → v_target`): what the follower's own limits rewrote. Only meaningful
  because `v_target` is first shifted back by the command lead — the follower samples the profile
  ahead *on purpose*, and comparing it to the plan at the same `s` scores that as a loss on every
  braking zone and a gain on every acceleration.
  > **[audit 09-09] The tool does not currently do this correctly.**
  > `plot_speed_tracking.py:175` computes `lead = v_true * cmd_delay + lead_min`. The controller
  > uses `v * (cmd_delay + target_lead) + lead_min` (`pure_pursuit.py:1138`). **`target_lead_s`
  > is missing from the tool**, so it under-shifts by `v · 0.08` = **0.64 m at 8 m/s** and commits
  > precisely the artifact trap 4 warns about. Fix the tool before judging E1/E2 on this number —
  > it is the term E1 changes, so an uncorrected tool will misreport E1's own effect.
- **delivery** (`v_target → v_true`): the car not producing the speed it was asked for. This is
  the controller's half.

Then look at the **speed profile chart**, which is the one that actually diagnoses:

- Achieved **below** plan while accelerating → not enough authority (band, or feedforward).
- Achieved **below** plan through a braking zone *without* the onset moving → `v_target` is
  depressed by the lead, not mistimed (`target_lead_s`). **[audit 09-09]** This, not the bullet
  it replaced ("achieved dropping *before* plan"), is what the baseline actually shows; compare
  braking **onset** in `s`, not just the speed trace, before blaming the lead.
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

> **[2026-09-09] §0 is the authoritative queue.** This section is kept for the per-experiment
> rationale and the E5 note; the ordering, the predictions and the results live in the §0 table,
> and E8/E9/E10 exist only there. If the two ever disagree, §0 wins.

Ordered by expected value ÷ risk. Each is one variable. Fill the results matrix in §8 as you go.

| ID | Change | Hypothesis | Predicted | Risk |
|---|---|---|---|---|
| ~~**E0**~~ | ~~`lookahead_delay_ref:=0.0`~~ **DONE 09-09 → REJECTED, see R8** | **[audit 09-09]** `d_scale` is pinned at its 0.70 floor, so the car runs `lookahead_k` 0.385 / `lookahead_max` 1.54 m — 30 % shorter than the values tuned to stop weave and wall contact. Restoring them targets the **lateral** budget and the collision gate. | −0.10 to −0.17 s and the collision | Medium. Restores a *tested* configuration, but one tested at a longer round trip. A longer `Ld` cuts corners: check corner bias sign and T2 clearance, not just \|e\|. |
| **E1** | `target_lead_s:=0.0` | **[audit 09-09, prediction revised down]** `target_lead_s` contributes `v·0.08` = **0.64 m** of the 1.69 m total lead at 8 m/s (the rest is `cmd_delay`, which is deliberate and stays). Against the real braking-zone lengths — **6.10, 5.90, 2.50, 3.70 m**, not 7.8 m — that is **~10 %** of the main zone. It bites hardest in the *short* zones: 1.00 m of a 2.50 m zone at 4.5 m/s, i.e. **40 %**. | **−0.08 to −0.12 s** (was −0.28 s; that used a 2.1 m lead at a `cmd_delay` of 0.175 that does not run, against a zone length that does not exist) | Low. Watch T2 and the s 42–45 entry. |
| **E2** | `target_lead_s:=0.04` | If E1 overshoots (hot entries), the optimum is between | between E1 and base | Low |
| **E3** | `slip_accel:=0.20` | Re-confirm the 0.16→0.20 plateau. **[audit 09-09]** Now predicted from the log rather than guessed: realized slip p90 is already 0.160, on the friction curve's flat top, so there is nothing above it to buy. | ~0 | Low. **Deprioritised — the band is not the limiter.** |
| **E4** | `accel_ff:=1.0` (circle still off) | Plan-acceleration feedforward without the circle band. multi-track measured accel delivery 91 %→98 %. Never tested here in isolation. **[audit 09-09] Now the primary longitudinal experiment**: 78 % of the loss is on accelerating stretches, the car delivers 66 % of a plan that asks only 2.49 m/s² mean, and neither throttle (median 0.203) nor the band (already at its edge) is what withholds it. | **−0.15 to −0.25 s** (raised from −0.05/−0.10) | Medium. Untested on this host *without* `slip_circle`. R5 is the warning: it cost 0.09 s on the **brake** side, so keep it accel-only. |
| **E5** | Per-zone lookahead | Excursions are local, not global. Mirror `lat_zones`: shorter `Ld` only at the `s` ranges where `\|e\|` spikes | −0.10 to −0.17 s, and the collision | Medium. Needs a small code change; see §7.1. |
| **E6** | `lookahead_max` sweep 2.0 / 2.2 / 2.4 | Global fallback if E5's excursions turn out not to be local | unknown | Low |
| **E7** | `enc_window_s:=0.0` vs `0.05` | A/B the encoder fix on the *current* stack, now that it is launch-exposed | confirms 3.7 s claim | Low. Diagnostic, not a candidate. |

**[audit 09-09] Every experiment is a paired 25-lap A/B, not a run against the old baseline.**
`cmd_delay` drifts between sessions and rescales the lookahead, the lead and the derate (trap 7),
so a control arm from another session is not a control. Run control and treatment back to back,
same session, fresh reset each, 25 laps each, and let `ab_report.py` check `pp_delay` parity
before you read anything else.

> **[2026-09-09, after E0] Order is now: E4 → E5 → E1/E2 → E3.** E0 is done and rejected (R8);
> E6 is dropped with it, since `lookahead_max` is not the lever on a 97 ms host. **E4 is next**:
> it owns 78 % of the longitudinal loss and, unlike E0, it does not touch lateral geometry at all,
> so it cannot reproduce R8's failure mode. E5 rises to second — E0's collisions landed at
> **s 43–46 specifically**, which is direct evidence that lookahead wants to be *per-zone* rather
> than globally longer or globally shorter.
>
> ~~**[audit 09-09] Revised order: E0 → E4 → E1/E2 → E5 → E3/E6.** E0 owns the collision gate and
> the lateral budget; E4 owns 78 % of the longitudinal one.~~ E1/E2 address the braking zones, which
> carry **16 %** of the longitudinal loss, and can reach only the discretionary 0.64 m of a 1.69 m
> lead within that — a realistic ceiling of **0.03–0.05 s**, which is why E1's prediction is
> revised down from −0.28 s.
>
> ~~**Ordering.** Run **E0 first.**~~ It is one launch override, no code change, it is
> the only queued item that addresses the collision gate directly, and until it is settled every
> lateral number in §8.3 describes a controller nobody intended to run. Then fix the
> `target_lead_s` term in `plot_speed_tracking.py` (§6.2), *then* E1/E2 — otherwise the tool
> misreports the very quantity E1 changes.

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

| Run | Date | Line | Change (one variable) | Laps | Best | Median | Mean | Coll | `pp_delay` med/p90 | Gap vs 11.440 | Verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `icra_base20_a70` | 2026-09-08 | a7.0 | *baseline, yaml defaults* | 21 | 11.750 | 11.898 | 11.950 | 1 | 0.119 / 0.138 | +0.458 | reference (superseded) |
| `E0_base` | 2026-09-09 | a7.0 | *baseline, 25 laps* | 25 | 11.650 | **11.800** | 11.830 | **0** | 0.097 / 0.108 | +0.360 | **new reference** |
| `E0_test` | 2026-09-09 | a7.0 | `lookahead_delay_ref:=0.0` | 13 then aborted | 11.700 | 11.799 | 11.804 | **49** | 0.100 / — | +0.359 | **REJECTED — R8** |
| `E1` | | a7.0 | `target_lead_s:=0.0` | | | | | | | | |
| `E2` | | a7.0 | `target_lead_s:=0.04` | | | | | | | | |
| `E3` | | a7.0 | `slip_accel:=0.20` | | | | | | | | |
| `E4` | | a7.0 | `accel_ff:=1.0` | | | | | | | | |
| `E5` | | a7.0 | per-zone lookahead | | | | | | | | |
| `E6` | | a7.0 | `lookahead_max:=…` | | | | | | | | |

> **[audit 09-09]** The baseline row above was re-derived from `icra_base20_a70.csv` directly:
> 21 laps after the out-lap, best **11.750**, median **11.898** (was quoted 11.92), mean **11.950**,
> collision delta **1**. Gap vs 11.4398 is **+0.458**. Recomputed plan time is **11.4398 s**
> (`raceline_a7.0.csv`, trapezoid over `ds/v̄`, loop closed), so 11.441 is right to the mm.
> **Its successor is `E0-base` at 25 laps — compare E0 against that, not against this row**
> (§3 rule 5: never compare runs of different length).

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

**[audit 09-09]** Add these two rows — they are where the loss actually is, and the split by `s`
obscures it because every "straight" row mixes accelerating and braking track:

| Split | Baseline | E0 | E4 | E1 |
|---|---|---|---|---|
| plan **accelerating** (78 % of loss) | **+0.526** | | | |
| plan **braking** (16 % of loss) | **+0.106** | | | |

Generate all of this — both tables, the decision gate and the figures — with:

```bash
python3 raceline/ab_report.py base.csv:"baseline" test.csv:"E0" \
  --path raceline/icra2026/raceline_a7.0.csv --name E0
```

### 8.3 Lateral excursion matrix

Where the car leaves the line, and by how much. Fill from the per-lap `|e|max` table.

| Run | \|e\| mean | \|e\| p90 | \|e\| max | # laps with \|e\| > 0.25 m | `s` of worst excursion | Corner bias (+in/−wide) |
|---|---|---|---|---|---|---|
| baseline | 0.045 | 0.078 | **0.765** | 7 of 21 | **3.9** | |
| E0_base | 0.038 | 0.074 | 0.540 | — | 46.2 (+0.525 inside) | +0.026 at 43–46 |
| E0_test (clean phase) | 0.066 | 0.124 | 0.651 | — | 45.8 (**+0.700** inside) | +0.004 at 43–46 |
| E1 | | | | | | |
| E5 | | | | | | |

**[audit 09-09]** `\|e\|max` was recorded as 0.60 m; the log's actual worst is **0.765 m at
s = 3.9**. Mean and p90 filled in from the same log (36 of 5408 ticks have a blank `pp_e_lat` and
are excluded). Excursion ticks (\|e\| > 0.25 m, 165 of them) by 2 m bin — this is the evidence
that the problem is **local**, and the shortlist of `s` ranges for E5:

| `s` bin | 0 | 2 | 4 | 6 | 8 | 20 | 22 | 24 | 40 | 44 | 46 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ticks | 7 | 10 | 8 | 3 | 11 | 16 | **49** | 19 | 13 | 19 | 10 |

Three clusters: **s 0–10**, **s 20–26 (T2, the worst)**, **s 40–48**. Nothing in between.

### 8.4 Band saturation

Only meaningful with the right `--slip-*` flags. > 30 % means the band is a real limiter.

| Run | Band in force | Straight 1–21 | Corner 21–24 | Straight 24–37 | Corner 43–46 |
|---|---|---|---|---|---|
| baseline | ±0.16 / −0.08 | 25 % | 12 % | 18 % | 12 % |
| E3 | ±0.20 / −0.08 | | | | |
| E4 | ±0.16 + ff | | | | |

### 8.5 Decision gate

An experiment is **accepted** only if all four hold:

- [ ] `pp_delay` median within 0.01 s of the arm it is being compared to **[audit 09-09]**
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
| **R8** | `lookahead_delay_ref:=0.0` (**E0**, 2026-09-09) | **49 collisions**, aborted at lap 18 of 25; 13 clean laps first, median **11.799 vs the control's 11.800 — no gain at all** | Restored `Ld` at speed from 1.48 to **2.09 m** exactly as intended, and that is the problem. 2.20 is the value tuned *at* the 175 ms reference delay; this host runs 97 ms, so the length buys no phase margin and doubles the chord sag (0.218 → 0.438 m at κ 0.8) into a line 0.28 m off the inside wall. Tracking got **worse** on every measure: \|e\| mean 0.038 → 0.066, p90 0.074 → 0.124, worst inside excursion +0.525 → **+0.700 m at s ≈ 46**. Then 33 of the 49 hits at **s 44–46** and 14 at **s 30**, respawn-rehit looping. Same failure family as R4: a lateral-geometry change that grows past a physical margin cascades rather than degrades. |

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
   **[audit 09-09] `plot_speed_tracking.py` does not do this — it is a live bug, not a caveat.**
   Line 175 is `lead = v_true * cmd_delay + lead_min`; `target_lead_s` is absent. It under-shifts
   by 0.64 m at 8 m/s. Add the term (or pass it) before quoting "follower clip" anywhere,
   including in §8.2.
5. **Line auto-detection is ambiguous.** Several a7.0 variants sit within 0.04 m of each other.
   Always pass `--path` explicitly.
6. **Session length.** Compare only runs of the same length, from a fresh reset.
7. **A parameter's yaml value is not necessarily its effective value.** Three parameters rescale
   others at runtime from the *measured* `cmd_delay`: `lookahead_delay_ref` (scales `lookahead_k`
   and `lookahead_max`), `derate_delay_from/to` (scales all of `v_target`), and the `lead` term
   itself. `cmd_delay_auto` is `true`, so `cmd_delay` drifts within and between sessions — which
   makes it a **hidden second variable in every "one variable per run" experiment**. Log its
   median and p90 (`pp_delay`) in §8.1 for every run, and treat two runs with different `pp_delay`
   as not directly comparable. This is how the effective lookahead came to be 1.54 m while the
   plan said 2.20 m for a month.
8. **`follower.launch.py` and `race.launch.py` do not apply the same configuration.**
   `frames.follower_args()` is read **only** by `race.launch.py:296`; `follower.launch.py` — the
   launch file in §3's canonical commands — ignores `frames.TRACKS['icra2026']['follower']`
   entirely and takes its values from `pure_pursuit.yaml`. Today both say `target_lead_s 0.08`
   and `v_max 8.0`, so nothing diverges; the moment §11 writes an accepted value into `frames.py`
   only, every later experiment run through `follower.launch.py` silently reverts it. **Write
   accepted values to both, or measure on the launch file you ship.**
9. **`/odom` is ground truth and restricted.** Every number in this document is a development
   number. Race-legal runs need `use_tf_pose:=true` and the localizer.

---

## 11. Definition of done

- 25 consecutive laps, fresh reset, **0 collisions**
- median ≤ 11.50 s on `raceline_a7.0.csv` — currently **11.800** (`E0_base`, 2026-09-09), so
  **−0.30 s** to find, against a plan of 11.440
- every run recorded in §0's Result column and §12 (§3 rules 9–10)
- the accepted configuration written into `frames.py`'s `icra2026` `follower` dict **and into
  `pure_pursuit.yaml`**, with the measurement that justifies each value in the comment beside it.
  **[audit 09-09]** Both, not either: `follower.launch.py` never reads `frames.py` — see trap 8.
  Verify by reading the `[pure_pursuit] launch overrides:` line and the node's startup banner on
  the first run after the change
- `VEHICLE_MODEL.md` §7 updated with the run history
- §8 matrices in this file filled, including the rejected experiments

---

## 12. Experiment log — full records

One subsection per run, newest last. Every run goes here: accepted, rejected, aborted,
inconclusive. Required contents are the launch command **as issued**, the override-confirmation
line from the follower log, the raw per-lap list, the headline numbers, the chart, and the
reading. See §3 rules 9 and 10.

---

### 12.1 E0 — `lookahead_delay_ref:=0.0` — ❌ REJECTED (2026-09-09)

**Prediction on record before the run:** −0.10 to −0.17 s and closes out the collision.
**Outcome: wrong in sign.** No time gain, 49 collisions. Recorded as **R8** in §9.

#### Hypothesis

`pure_pursuit.py:1059-1061` scales the lookahead by the *measured* command delay:

```python
d_scale = max(0.7, min(1.2, self.cmd_delay / self.ld_delay_ref))   # ld_delay_ref = 0.175
ld = clip(self.ld_k * d_scale * abs(self.speed), self.ld_min, self.ld_max * d_scale)
```

Measured `cmd_delay` on this host is **0.097 s**, so `0.097/0.175 = 0.55` floors `d_scale` at
**0.70** and the car runs `lookahead_k` 0.385 / `lookahead_max` 1.54 against the documented
0.55 / 2.20. The yaml records that a **1.6 m** lookahead *"weaved (yaw ±9 deg in 0.2 s) and
clipped a wall 0.29 m from the line"*, and intermittent excursions plus a rare wall contact were
exactly the observed failure. Setting `lookahead_delay_ref:=0` disables the scaling and restores
the tuned values.

#### Method

Paired A/B, same session, fresh Reset + Connect before each arm, both counters verified at 0 from
ROS as well as the GUI, 25 laps each, out-lap discarded. Line `raceline_a7.0.csv` (541 pts,
54.09 m, plans 11.4398 s).

```bash
# Arm A — control, no overrides
ros2 launch racer_control follower.launch.py track:=icra2026 \
  path_csv:=/home/autodrive_devkit/dev_ws/raceline/icra2026/raceline_a7.0.csv

# Arm B — one variable
ros2 launch racer_control follower.launch.py track:=icra2026 \
  path_csv:=/home/autodrive_devkit/dev_ws/raceline/icra2026/raceline_a7.0.csv \
  lookahead_delay_ref:=0.0
```

Override confirmed in `/tmp/E0_test_foll.log` before the run was allowed to proceed:

```
[pure_pursuit] launch overrides: {'lookahead_delay_ref': 0.0}
[pure_pursuit] speed_source=tire  throttle_mode=slip  slip band [-0.08, +0.16]
               u_launch 0.4 m/s  lookahead 0.8-2.2 m (k 0.55)  latency_comp 0.05 s
```

Arm A's log has **no** `launch overrides` line — it is a true default run.

#### Raw lap times

```
Arm A  E0_base.csv        25 laps, 0 collisions
  11.85 11.80 11.75 11.90 11.80 11.75 11.75 11.80 11.90 11.75 11.75 11.70 12.25
  11.70 11.80 11.95 11.65 12.00 11.80 11.80 12.00 11.90 11.85 11.80 11.75

Arm B  E0_test_aborted.csv 17 laps then aborted   (* = collision on that lap)
  11.80 11.70 11.70 11.80 11.80 11.75 11.85 11.85 12.00 11.90 11.70 11.75 11.85
  12.25* 11.95 11.75 11.70
```

#### Results

| | Arm A `E0_base` | Arm B `E0_test` |
|---|---|---|
| Laps | 25 | 13 clean, then collisions, aborted at 17 |
| Best / **median** / mean | 11.650 / **11.800** / 11.830 | 11.700 / **11.799** / 11.804 |
| Lap-time std | 0.122 | — |
| **Collisions** | **0** | **49** |
| Gap vs plan 11.440 | **+0.360** | +0.359 |
| `pp_delay` med / p90 | **0.097** / 0.108 | **0.100** / 0.129 |
| `pp_ld` median at v > 6 | **1.477 m** | **2.093 m** |
| `pp_ld` max | 2.183 | 2.184 |
| \|e\| mean / p90 / max | **0.038 / 0.074 / 0.540** | **0.066 / 0.124 / 0.651** |
| Worst **inside** excursion | +0.525 m at s 46.2 | **+0.700 m at s 45.8** |

`pp_delay` medians differ by **0.008 s**, inside the 0.01 s parity gate — the arms are
comparable and this is a clean rejection, not a confounded one.

**Collision distribution by 2 m bin:** `s 44: 33` · `s 30: 14` · `s 28: 2`.
First contact at **t 177.5 s, s 45.99 m, v 2.98 m/s, e_lat +0.749 m** (positive = inside),
`Ld` 1.18 m at that instant. After the first hit the car respawned into the same wall repeatedly —
the 49 are a loop, not 49 independent events.

**Section-level time loss vs plan** (from `ab_report.py`; Δ is the result):

| Section | Arm A | Arm B | Δ |
|---|---|---|---|
| straight 0–21 | +0.122 | +0.167 | **+0.046** |
| corner 21–24 (T2) | +0.007 | +0.012 | +0.004 |
| straight 24–37 | +0.074 | +0.081 | +0.007 |
| corner 37–41 | +0.001 | +0.026 | +0.024 |
| straight 41–43 | +0.034 | +0.024 | −0.010 |
| corner 43–46 | +0.008 | +0.037 | **+0.028** |
| straight 46–end | +0.056 | +0.094 | **+0.038** |
| **total** | **+0.303** | **+0.441** | **+0.138** |

**Decision gate (§8.5): 1 of 4 — REJECT.**

- ❌ collisions = 0 → **49**
- ❌ median improves ≥ 0.03 s → **−0.001 s**, noise
- ❌ no section worse by > 0.03 s → **three** are
- ✅ `pp_delay` parity within 0.01 s → 0.008

Chart: `figures/E0_ab.png` (five panels; panel 2 is the effective-lookahead trace, panel 4 the
excursion histogram). Regenerate with:

```bash
python3 raceline/ab_report.py E0_base.csv:"E0_base (control)" E0_test_aborted.csv:"E0 delay_ref 0" \
  --path raceline/icra2026/raceline_a7.0.csv --name E0 --lap-lo 11.4 --lap-hi 12.4
```

#### Reading — why the prediction was wrong

The measurement was right and the inference from it was wrong. The override did **exactly** what
it was predicted to do mechanically: `Ld` at speed went **1.477 → 2.093 m**. It bought nothing —
median 11.799 against 11.800 — and cost the collision gate outright.

`lookahead_max: 2.20` is the value tuned **at the reference delay**, where `d_scale = 1.0`. It is
what the geometry wants when the round trip is **175 ms** and the phase lag needs that much
lookahead to stay stable. This host now runs at **97 ms**. There is far less lag to buy margin
against, so the extra length buys nothing on the straights and spends itself cutting apexes. The
chord sags inside by `|κ|·Ld²/8`, which is quadratic in `Ld`:

| | `Ld` at speed | sag at κ = 0.8 |
|---|---|---|
| Arm A (floored) | 1.477 m | **0.218 m** |
| Arm B (restored) | 2.093 m | **0.438 m** |

The line runs **0.28 m** from the inside wall at s 43–46. Arm A fits under that; Arm B does not,
and the measured worst inside excursion (+0.700 m) confirms it. Same failure family as **R4**: a
lateral-geometry change that grows past a physical margin cascades rather than degrades.

**Conclusion: `lookahead_delay_ref` is doing its job and the 0.70 floor is load-bearing.**
"Configured 2.20 / effective 1.48" is not a discrepancy to fix — 2.20 is the value *at the
reference delay*, and §5.1 now says so.

#### What this run bought us anyway

1. **`E0_base` is the new reference** and is strictly better evidence than the historical
   `icra_base20_a70`: 25 laps instead of 21, median 11.800 vs 11.898, **0 collisions vs 1**, and
   its `pp_delay` is recorded so future arms can be checked against it.
2. **The `pp_delay` parity rule earned itself immediately** — 0.008 s apart is what makes this
   rejection trustworthy rather than arguable.
3. **E5's case got stronger, not weaker.** The failure localised to **s 43–46**, which is direct
   evidence that lookahead wants to be *per-zone*, not globally longer or globally shorter. E8 and
   E9 in §0 exist because of this run.
4. **E6 is dead.** Sweeping `lookahead_max` on a 97 ms host sweeps the wrong quantity.
