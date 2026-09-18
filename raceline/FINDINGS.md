# Classical stack: findings and results

What was learned tuning the AMCL + pure-pursuit stack on the Porto track, and
where it ended up. The per-run history lives in `VEHICLE_MODEL.md` section 7
(runs 9-41); the measured results table is in the top-level `README.md`. This
file is the summary: the levers that mattered, the bugs found the expensive way,
and the final answer on the lap-time ceiling.

Every number here is measured in the AutoDRIVE simulator on the development
laptop (an ASUS TUF gaming laptop), race-legal sensors only (LiDAR, IMU,
encoders) against the pre-built map, with the one-shot ground-truth warmup seed
the rules allow.

---

## 1. Summary

The stack drives the Porto track cleanly at lap times within a tenth of the
track record, and one configuration beats it.

| line | best lap | mean | wall clearance | role |
|---|---|---|---|---|
| `raceline_a7.0_rec.csv` | **6.45 s** | 6.53 s | 0.03 m | record attempt, beats the 6.46 record; only on a stable machine |
| `raceline_a7.0.csv` | 6.50 s | 6.58 s | 0.07 m | fast line, robust |
| `raceline_a6.5.csv` | 6.65 s | 6.71 s | comfortable | **submission default**, 32 consecutive clean laps |

For reference: the track record is 6.46 s, and the velocity profile's ideal
(assuming commands act instantly) is 6.36 s. The gap between the real lap and
that ideal, about 0.14 s, is the control loop's round trip and cannot be closed
without a faster simulator.

**Recommended submission:** the 6.5 line with the current `pure_pursuit.yaml`
defaults (it is already `frames.DEFAULT_RACELINE`). Use the 7.0 line for a
faster run on a machine known to be stable, and the record line only for a
single timed best-lap attempt where a retry is allowed.

---

## 2. The one fact that drives everything

The simulator spins each wheel to `25.25 x throttle` m/s within a millisecond,
whatever the car is doing (`VEHICLE_MODEL.md` section 3.1). Three consequences
shaped the whole stack:

- **The encoders report the throttle command, not the car.** Closing a speed
  loop on them closes it around the wheel and leaves the car open-loop. The
  follower instead runs the simulator's own tire curve and drag on the measured
  wheel speed to recover the car's true speed (`speed_source: tire`), a
  contracting observer with bounded error (p90 0.15-0.22 m/s against truth).
- **Throttle is a wheel-speed command.** The follower commands a wheel speed
  inside a +/-0.08 slip band around the tire's peak-force slip
  (`throttle_mode: slip`), which caps the force demand both ways and keeps the
  encoder error inside the band in every regime, launch and wall-respawn
  included.
- **The command lands late.** The wheel follows the throttle three simulator
  frames later, 175 ms at 17.5 Hz. The band is therefore placed around the
  speed the car will have when the command arrives, and the delay is measured
  online (`cmd_delay_auto`) so the stack adapts to whatever rate the machine
  runs at.

---

## 3. The loop rate is a fixed cadence, not compute

This changed how to think about the evaluation machine. On the development
laptop:

- Headless (`-batchmode -nographics`): 20.3 Hz. With graphics: 17.6 Hz.
- Pinning the simulator to a **single core** still gave 20.3 Hz.
- Starving the bridge (pinned beside a busy core) still gave 19.8 Hz.

If the rate were compute-bound, one core would have collapsed it. It did not, so
the roughly 20 Hz is the simulator's own socket cadence, and a faster CPU or GPU
will not lift it. **Expect the evaluation machine to sit near 20 Hz too**, which
means near the best lap times measured here rather than faster.

A loop *slower* than 20 Hz comes from load, not from a slow machine: this laptop
decayed from 17.3 to 12.8 Hz within one long graphics session, and that decay,
not the GPU driver mode it was first blamed on, is what caused the early wall
hits. A system restart brought it back to 17.5 Hz.

---

## 4. Bugs found the expensive way

Two localization timing bugs were each worth about 0.1 s a lap and, left in,
partly masked each other.

- **The estimate led the car.** The odometry transform is post-dated by 20 ms
  so lookups at "now" succeed, which means the transform covering each LiDAR
  scan is published before that frame's data arrives and carries the previous
  frame. AMCL then paired every scan with odometry one frame old and pushed the
  estimate forward, 35-40 ms x speed on every run. Fixed by extrapolating the
  dead-reckoned **position** to each transform stamp, mirroring the heading
  extrapolation that was already there (`dead_reckoning.py`, `extrapolate_pos`).
- **The follower read a correction half a second stale.** Asking tf2 for
  map->base at the latest time evaluates AMCL's post-dated map->odom correction
  by interpolating samples half a second old. The follower now composes the
  newest sample of each transform leg instead (`pure_pursuit.compose_latest`).

A third, subtler one: the online delay regression used a 50 ms grid and read the
round trip as 150 ms, when at 5 ms resolution it is 165-180 ms, three frames.
Every controller constant tuned against the coarse reading was 17 % off. The
references were moved to 0.175 s.

One change that did **not** pan out and was reverted: a landing-speed prediction
from the profile's rise over the delay. It looked like it would fix the corner
speed deficit, but checked against the logs it moved the wheel command by under
0.03 m/s. The corner deficit is the tire at its lateral limit, not a throttle
bug.

---

## 5. What moved the lap time, in order

Starting from a 7.90 s baseline, on the a7.0-class line:

| change | effect |
|---|---|
| profile speed target at the delay-matched point, not a 1 m preview | -0.45 s |
| stepped the lateral limit up the ladder to 7.0 with a curvature cap | to 6.65 |
| widened line + online delay + speed derate (robustness) | held clean |
| both localization timing fixes | to 6.50, best tracking |
| steering latency compensation 0.05 s + speed preview 0.08 s | cleaner corners |
| longitudinal profile limit 4.55 -> 5.0 m/s (measured deliverable ~5.5) | -0.08 s |
| safety margin 0.15 -> 0.10 m (record line only) | to 6.45, spends the buffer |

**What is tapped out:** the speed limits (5.5 longitudinal ties 5.0 for less
margin; lateral washes above the 7.0 cap), the geometry family (minimum-time
optimization gained nothing on this short track), and v_max (the straights never
reach 8 m/s).

---

## 6. Robustness, validated in the simulator

The stack is built to give up speed rather than wall margin when the loop is
slow, because the evaluation machine is unknown and this one decayed mid-session.

- The line carries extra left margin at the two sites where drifts used to land
  (the straight after the first corner and the S-exit approach).
- The follower measures its own round trip every two seconds and, when it is
  slower than the 0.175 s the profile was proven at, scales the speed targets
  down toward the 6.0 rung and lengthens the lookahead.

This was validated directly: adding 10 ms of loopback latency
(`tc qdisc add dev lo root netem delay 10ms`) forced the loop to 11.0 Hz with a
254 ms round trip, well past the 210 ms that used to hit walls. The car ran
seven clean laps, no contact, giving back about a second a lap. That is the
trade the derate is for.

---

## 7. The ceiling, and the one lever left

The real lap (6.50 on the fast line) sits 0.14 s above the profile's ideal
(6.36). That gap is the 175 ms round trip: the car runs 2-3 % under the profile
through the two tight corners because it is reasoning about a car 175 ms in the
past. Measured precisely on the second track (section 9), the gap has two named
parts, and neither is "braking early": the speed target is read further ahead
than the delay alone requires, and the speed observer reads high in tight
corners. Removing the first bought 0.1 s there; removing the second bought
nothing and cost exit margin. Nothing on the car side moves the rest.

The only thing that beats 6.46 is spending wall margin. Cutting the safety
buffer from 0.15 to 0.10 m widens the corridor, lowers curvature, and raises
corner speed to a 6.45 real lap, but the line then runs to 3 cm of the wall at
the S-exit. Margin and sub-6.46 are the same quantity there, set by the delay,
so there is no free clearance: deepening that corner's margin to a comfortable
level costs 0.03 s and gives the record straight back.

The only untried lever below 6.45 is a predictive controller that steers on the
vehicle model run forward by the measured delay, rather than the current
pursuit-plus-anticipation. It is a real rewrite with genuine weave risk, and was
judged not worth starting once the record was already beaten.

---

## 8. Final configuration

- **Follower:** `pure_pursuit.yaml` defaults. `speed_source: tire`,
  `throttle_mode: slip`, `cmd_delay_auto: true`, lookahead 0.80-2.20 m
  (k 0.55), `steer_a_lat_max: 7.0`, `latency_comp_s: 0.05`,
  `target_lead_s: 0.08`, derate references at 0.175/0.21 s.
- **Localizer:** AMCL on the pre-built map, LiDAR + IMU + encoders, with the
  warmup ground-truth seed; `dead_reckoning` extrapolating both heading and
  position to each stamp.
- **Lines:** `optimize_raceline.py --track porto` plans at longitudinal
  5.0 m/s and lateral at the tire asymptote. In `raceline/porto/`:
  `raceline_a6.5.csv` is the default and submission line, `raceline_a7.0.csv`
  the fast line, `raceline_a7.0_rec.csv` the record attempt.

All of these numbers are Porto's. The controller transfers to another track
unchanged; the spawn, the safe grip rung and the margin zones do not, and are
re-derived per track (see **Tracks** in `CLAUDE.md`).

Run it with:

```bash
source ros_env.sh
ros2 launch racer_bringup race.launch.py localizer:=amcl track:=porto log_csv:=run.csv
python3 raceline/analyze_run.py run.csv --path raceline/porto/raceline_a6.5.csv
```

---

## 9. Second track: what transferred and what did not

The ICRA 2026 competition track was mapped and climbed in ten runs on the
branch `multi-track`, with the controller untouched. It confirmed the split
predicted in section 2 exactly.

**Transferred unchanged:** the tire observer, the slip-band throttle, the
delay measurement, the derate, the curvature cap, both localization timing
fixes. The first drive on the new map ran 13 clean laps with tracking error
0.062 m and equal-time localization 0.115 m, Porto's numbers.

**Did not transfer, and had to be re-derived:** the spawn (read after a reset,
not after driving), the margin zones (three, each placed from a per-sample
reading of where the car ran wide, never from a section average, which misled
once), and the grip rung. The track's two hairpins have curvature 1.37 against
Porto's 0.92, which puts the apex at 2.2 m/s; there the same ±0.2 m/s of
delay-induced speed error is a 13 % lateral overload where on Porto it was 7 %.
The 6.5 rung hit twice in 46 laps, both understeer at the tire's limit on a
hairpin exit, traced tick by tick. The fix is a per-corner grip limit
(`--lat-zones`), capping only the hairpins at 6.0: 0.15 s a lap, and the
measured peak demand fell from 7.23 to 6.27 m/s².

**Found on the way:** the map's duct walls are hollow and the LiDAR sees
through their open ends, so the raceline needs a sealed geometry map while AMCL
keeps the sensor map; the centreline extractor's greedy skeleton walk failed on
a switchback and now orders the ring by BFS; the logger and the bootstrap still
read Porto's constants and are now track-scoped.

**The controller gap, taken apart (runs 11-14).** With the hairpins zoned
and the main straight at 8 m/s the real lap sat 0.30 s over its profile, and
0.28 of it was in the two braking zones. The car did not brake early: onset
was 0.2-0.6 m *after* the profile's. It ran 0.5-1.0 m/s under the profile all
the way down because the speed target is read `v x (delay + target_lead_s
0.08) + 0.10 m` ahead, 1.8 m at 7 m/s where the delay alone puts it at 1.2, and
in a zone falling 0.62 m/s per metre the extra lead saturates the brake side
of the slip band. `target_lead_s` was tuned on Porto's short braking zones;
zero here took the hairpin-approach loss from 0.18 to 0.07 s and is the ICRA
registry default (run 13, 12 clean laps, 12.15 best / 12.22 mean).

The second part is the speed observer: it simulates one wheel at the car's
speed, while a hairpin's four wheels see four contact speeds on a concave
friction curve, and it reads 0.15-0.19 m/s high at the apexes, zero on the
straights. A four-wheel observer (`observer_wheels: 4`) removes the bias, on
the car as in the offline replay, and it stays **off**: it gained no lap time,
the hairpin sections were already on the profile's time, and with the true
speed in hand the follower asked for more acceleration out of the left-leg
apex at full steering lock, exit tire slip up 60 %, heading error up 50 %, and
on the fourth lap the front tires ran out of grip and the car understeered
into the exit wall, exactly run 9b's signature. The old observer's optimism
had been the throttle limiter at that corner. The principled fix is a
friction-circle allocation of the acceleration slip band; it is not built,
because the remaining 0.26 s is spread across the lap and the physics floor
for this configuration is 11.7 s ideal, 11.36 with every margin spent. Sub-11
is not available from this car on this track. The floor line was driven once
and hit within eight seconds at a gentle corner with 4.6 m/s² of demand: the
margin was the limit there, not the tire. The 7.0 line, hardened from its own
samples with a margin zone at the T1 exit and T2 capped at 6.5, ran ten clean
laps at 12.00 best / 12.05 mean.

**The longitudinal lever (runs 17-23).** Only 4 % of that lap was
grip-limited and 18 % at the speed cap; 78 % was the car accelerating or
braking on a plan of `a_long` 5.0, a Porto number never revisited, while the
tire's curve is flat-topped at 6.8-7.1 m/s² over slip 0.10-0.18 and the car
ran at slip 0.03 delivering 91 % of a plan asking 2.5 m/s². Three changes:
a slip band on the flat top (0.12) scaled by the friction circle so hairpin
exits get *less* than before; a feedforward of the plan's acceleration through
the inverse tire curve (acceleration only: fed forward on braking it tracked
the plan's deceleration instead of its speed and cost every apex 0.1 m/s);
and a separate brake budget so acceleration climbs while the braking zones
into the hairpins stay as validated. Rungs 5.5 then 6.0, the straight to 9,
T2 back to 7.0 and T3 to 7.5 into its 0.45 m of outer room, two more margin
zones from the samples: 11.60 best / 11.66 mean over eleven clean laps, the
ICRA submission, 0.55 s a lap under the day's starting point. The ladder tops
out at 6.0: at 8 m/s drag eats the tire and delivery falls to 89 %.

**Still open:** a wall contact respawns the car at a checkpoint and nothing
re-localizes it. One time AMCL re-converged on its own in 25 s; another time it
never did and the car drove blind for half an hour. As the stack stands, one
contact can end a run. The legal fix, detect the respawn, stop, call AMCL's
global relocalization, creep until converged, is scoped but not built.

### 10.5 What finally worked: minimum lap time, with a margin that is measured

Seven levers were tried on the follower and the profile and all seven failed the
same way -- they found time on paper by asking the car for more, and the car,
already 0.2 m/s under its target everywhere, gave it back in wall clearance:

| change | apex clearance | best lap | outcome |
|---|---|---|---|
| baseline | 0.175 m | 9.50 | -- |
| `slip_kp` 1.5 | 0.035 m | 9.45 | contact in 10 laps |
| apex margin line | 0.146 m | 9.45 | s 23.9 to 0.025 m |
| `lookahead_min` 0.6 | 0.090 m | 9.60 | two contacts, slower |
| lateral limit 7.5 | -- | 9.50 | contact in 5 laps |
| curvature bound 1.3 | -- | -- | 0.036 s for curvature the car cannot make |
| `control_hz` 45 | -- | -- | five resets, sim tick fell to 16.5 Hz |
| shortest-path blend | -- | -- | 10.02-11.39 s predicted, strictly worse |

What worked was `raceline/opt_mintime.py`: one NLP over lateral offset AND
speed, minimising lap time, so the geometry can trade curvature against a better
exit instead of being fixed before speed is ever considered. **Run mtb15: 30
laps, zero contacts, 9.40 best / 9.45 typical against 9.45 / 9.55, and the
hairpin-1 apex went from 0.035 m of body clearance to 0.125 m.**

Three things had to be right, and two of them bit first.

**The steering rate has to be constrained against the RIGHT metric.** The NLP
measures curvature as a circle through three adjacent points, which is correct
for curvature (1.25 against the spline's 1.26) but smooths its DERIVATIVE, and
the rate depends on the derivative: 2.91 by that measure against 5.04 by spline
on the same line. The first line looked feasible and demanded 4.47 rad/s at
s 21.6 against an actuator limit of 3.2; the car washed WIDE there on ten of
twelve contacts. `--rate-iters` now solves, measures the rate the way the
profiler does, tightens the internal limit by the ratio and repeats. It
converges in three passes and costs nothing -- constraining it made the line
FASTER, because the sharp steering was never where the time was, only somewhere
the solver was free to be sloppy.

**The margin IS the design, not a safety detail.** A min-time solver spends
every centimetre of corridor it is given, because clearance it does not use is
time it does not get. Left at the uniform 0.20 m it pinned itself to the floor
on the fastest part of the lap. The measured exchange rate on this track is
about **0.5 s of lap for 0.1 m of clearance**, and the right buffer comes from
what a contact costs (10 s) rather than from what makes the prediction look
good.

**A uniform margin is wrong by a factor of four.** Measured over 63 laps, the
car holds its line to 0.088 m at p99 down the main straight and to 0.369 m
through hairpin 1. So 0.20 m was simultaneously 0.11 m too generous on the
straights and 0.17 m too thin in the hairpin -- which is the whole reason that
apex was always the tight spot, through every line this project has raced.
`--margin-from RUN.csv` feeds the real profile in. That single change is what
turned the min-time line from a crash into a win: it put clearance where the car
wanders and let the solver use the corridor where it does not.

Still open: the follower's 0.2 m/s deficit is untouched (the run predicts 9.13
and drives 9.45), `mode:=race` has never been run on this track, and this line
has 30 clean laps against the previous line's 63.


## 11. Pushing the lap time on IROS 2026 (2026-09-18)

Plain summary of one day's runs. Every run below was at the 45 Hz loop
(`tcp_nodelay:=true loop_hz_cap:=45`), on Adil's race arguments (slip dead
reckoning, 360-beam AMCL, hybrid LQR), fresh simulator each time. A run is
"clean" when the car never touched a wall.

### Best runs

| stack | line | laps | mean | best |
|---|---|---|---|---|
| **race-legal (AMCL)** | `rl_mt_lat7.25_hp70_bendz65_L70.csv` | 57 clean of 58 | **8.92 s** | 8.85 |
| **ground-truth pose (not race-legal)** | `rl_mt_tb10_lat875_b55_L70.csv` + `steer_a_lat_max:=8.0` | 22 clean | **8.45 s** | 8.40 |

The ground-truth run is the controller's ceiling on this track: same car,
same follower, no localization error. The gap between the two rows, about
half a second, is what localization costs today, and that is being fixed
separately.

### What we did, in order

Start of the day: 8.90 s on AMCL, 50 clean laps, lateral limit 7.25.

1. **Tried the obvious knobs on AMCL and they all hit the same wall.** Lateral
   7.50, braking later, more acceleration: each one contacted at the exit of
   the corner before the bend (s 38-39), where the line sits 0.35 m from the
   right wall and the car always runs 0.15 m wide. Capping the lateral limit at
   that one corner (6.5) fixed it: 57 clean laps, 8.92 s. Lap time did not
   move, but the car stopped hitting.
2. **Switched to the true pose to find out what the controller alone can do**
   (`drive_on_truth:=true`). Same line: 8.88 s. So localization was costing
   only 0.04 s at that point; the rest was the line and the follower.
3. **Raised the lateral limit in steps**, 7.25 -> 7.50 -> 7.75 -> 8.0 -> 8.25
   -> 8.50 -> 8.75, hairpins held a step lower. Every step was clean and the
   car's corner errors never grew. Together: 8.88 -> 8.49 s.
4. **Re-solved the line with the smaller wall margin the true-pose car needs**
   (its worst error is 0.09 m against 0.13 on AMCL). 0.08 s.
5. **Found the follower's own lateral cap** (`steer_a_lat_max`, default 7.0)
   was clipping the steering at every hairpin apex and starving the throttle
   there. Raised to 8.0: 0.05 s. 8.5 was worse, so 8.0 it is.
6. **Braking budget 5.0 -> 5.5**: 0.04 s. 6.0 slid at a hairpin exit and hit;
   rejected.
7. **Things that did nothing:** top speed 9.5 (the car cannot reach it before
   the braking point), and pulling the speed target back (`target_lead_s`
   -0.06); it moved the loss around and ran every corner a few centimetres
   wider.

Result on the true pose: 8.88 -> 8.45 s in eleven runs, every kept step clean.

### Where the rest is

The best line predicts 8.30 s and the car drives 8.45. The missing 0.15 s is
in the hairpin apexes: pure pursuit cuts a chord through a 1.22 curvature
corner and runs 0.13 m wide, and the throttle's slip budget collapses at the
apex. That 0.13 m is also what sets the wall margin, which is the single
biggest cost on the lap (about 0.5 s). The next lever is therefore the
steering law, not another budget. A physics floor for this car on this track,
tire peak everywhere and no margin, is about 7.1 s; a realistic ceiling with a
better controller is 7.8-7.9 on the true pose and about 8.0 race-legal.

### Carrying the gains to AMCL

The same budgets on the AMCL-margin geometry ran 8.54 s on its clean laps but
the localized car exits hairpin 1 0.25 m wide at the 8.0 hairpin budget
(0.13 on truth) and contacted there. The line to continue from is
`rl_mt_amcl_lat80_hp725_b55_L70.csv` (hairpins back at 7.25, predicts 8.55)
with `warmup_dist_m:=28` so lap 1 accelerates after the right-wall zone. That
run also exposed a recovery bug: after two contacts the re-seed picked the
wrong checkpoint (estimate 12 m and 27 m off) and the run never came back.
One contact should cost 10 s, not the session.

### 11.1 Controller experiment: curvature feed-forward + LQR (`controller_mode:=ff_lqr`)

Tried on the true pose against the 8.45 s pursuit baseline, three runs on
2026-09-18/19. The idea: steer the path's own curvature at the landing point
instead of the pursuit chord, so the hairpins are not cut 0.13 m wide.

- Run 1: LQR at its design bound (0.12 rad) bypassed the steering cap and
  tipped the front tire past its peak at the apex (yaw rate 1.5 against 2.8,
  0.57 m wide). Cap now applies to the total command.
- Run 2: bound 0.03 rad could not rejoin the line after a reset (drove parallel
  0.4 m off into the next wall) and could not catch the exit understeer.
- Run 3: bound scaled by the tire's remaining lateral room. 20 laps at
  **8.495 s** against 8.450: the apexes sit on the line either way, the corner
  EXITS run the same 0.1 m wide either way (that is the tire under combined
  load, not steering geometry), and on the straights the LQR chatters
  (|e_lat| p90 0.12 m against 0.04, steering std 0.06 against 0.01).

Verdict: shelved. The mode stays in the code as an opt-in experiment; the
defaults and the other two modes are untouched. The 0.13 m the hairpins run
wide is the tire at the exit, so it will not yield to a steering law that
still asks the same lateral demand; the remaining ways in are the profile at
the exits (less combined demand) or a controller that knows the tire.

### 11.2 True-pose record: 8.37 s (2026-09-20, early)

After the controller experiments (11.1 and the `mpc` branch, both shelved),
three follower-side steps and one profile step on the true pose, each a
single change, each 19-25 clean laps:

| step | mean | best |
|---|---|---|
| baseline (section 11) | 8.450 | 8.40 |
| `latency_comp_s` 0.05 -> 0.10 | 8.355, but 2 contacts (exits 0.05 m wider) | 8.33 |
| `latency_comp_s` 0.075, `warmup_dist_m` 28 | **8.410**, clean | 8.38 |
| + `slip_circle_ref` 9.5 (throttle slip budget no longer tied to the 8.0 steering cap) | **8.385**, clean | 8.36 |
| + line `rl_mt_tb07_z9_L70.csv` (9.0 lateral zone s 28-36, pre-bend 6.75, margin re-solved at 0.07 buffer) | **8.371**, clean | 8.35 |

What the steps say: the steering was being timed to a 50 ms delay against a
measured 120 ms, and every exit is where the time and the risk both sit, so
the propagation is a dial between lap time and exit margin, 0.075 being the
value that keeps every wall at 0.20 m or more. The car is now 0.12 s off its
own profile (8.25 predicted), all of it in the hairpin braking zones and
exits, and the levers that remain there are controller structure, not knobs.

Record configuration on the true pose (not race-legal; `drive_on_truth`),
the exact launch line, from the repo root with the simulator up and the
bridge started by this launch:

```bash
pgrep -af "nav2_amcl|racer_localization|racer_control|map_server" | grep -v pgrep || echo clean
ros2 launch racer_bringup race.launch.py track:=iros2026 tcp_nodelay:=true loop_hz_cap:=45 rviz:=false \
  drive_on_truth:=true log_rate:=45 \
  path_csv:=$PWD/raceline/iros2026/rl_mt_tb07_z9_L70.csv log_csv:=run_truth_record.csv \
  distance_source:=slip v_max:=9.0 control_hz:=45 warmup_v_max:=2.0 warmup_dist_m:=28.0 enc_rate_window_s:=0.10 \
  amcl_params_file:=$PWD/experiments/iros2026/params/amcl_beams360.yaml exit_guard_from:=0.0 exit_guard_full:=0.0 \
  controller_mode:=hybrid_lqr lqr_k_lat:=0.03 lqr_k_head:=0.05 lqr_k_yaw:=0.0 lqr_max_correction_rad:=0.02 \
  recover_settle_s:=2.0 recover_creep_s:=3.0 steer_a_lat_max:=8.0 latency_comp_s:=0.075 slip_circle_ref:=9.5
```

Read the result with
`python3 raceline/analyze_run.py run_truth_record.csv --path raceline/iros2026/rl_mt_tb07_z9_L70.csv`
(from `.venv-rl`). The same line without `drive_on_truth:=true` is the
race-legal launch, and there the propagation should start at
`latency_comp_s:=0.05`: the localized car already exits 0.05 m wider.
