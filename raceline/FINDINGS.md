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

## 12. Where AMCL's error actually comes from, and localizer v2 (2026-09-19)

Measured on the three logged docker runs (`runs_docker/m16.csv`, `base.csv`,
`tb10_on_amcl.csv`; 45 Hz, slip dead reckoning, 360-beam AMCL) and on the map
itself. Plain statements, each with the number behind it.

**Cross-track is solved.** AMCL's cross-track error is 0.016 m mean, p90
0.03-0.045. The map's geometry (raycast along the centreline, Gauss-Newton
information of a likelihood-field fit) allows 1.5-2.2 cm: AMCL is at the floor.

**Along-track is the whole problem.** Along-track p90 0.32-0.51 m, max 0.75;
+0.22..+0.34 m mean on the straight (estimate ahead), within +-0.09 everywhere
else.

CORRECTION (2026-09-19, after shadow run 1). An earlier draft of this section
said slip dead reckoning was "exact, 1.0000 of the true distance". That was a
botched measurement: it compared `dist_m` against ground truth, and `dist_m`
in log_localization IS the ground-truth distance, so it was a self-comparison.
The honest numbers are `enc/true` 1.016 and a dead-reckoned path 1.028 -- the
odometry over-reads about 2 %, which over the 16 m blind straight is +0.3 to
+0.5 m. Dead reckoning does NOT carry that straight accurately, and no
localizer can fix it there, because nothing observes along-track. What can be
fixed is how the estimate BEHAVES while carrying it.

**The straight is blind, for any algorithm.** Centreline s 26.7-35.9 has
along-track information 0.0-2.6 against 370-450 cross-track: parallel walls
and a 270 deg FOV that sees nothing fore or aft. Hairpin 1's end wall pins
the estimate only over the last ~2 m (s 25.8 -> 24.9: 22 -> 344), not at
lidar range. No scan matcher, no beam count, no particle count changes this.

**AMCL injects the along error on that blind stretch.** Its map->odom
correction walked 3.8 m (m16) and 10.9 m (base) cumulatively along-track
there, in per-sample steps p90 0.07-0.10 m and max 0.14-0.19 m -- against a
0.09-0.13 m wall margin -- on an axis the scan cannot observe. A particle
filter cannot hold still where the sensor model has no gradient: alpha3
spreads the cloud along the corridor and resampling picks a member. L1
(alpha3 0.10 -> 0.25) got noisier for exactly this reason. `fit_est` runs
1.5-2x `fit_true`: a better-fitting pose exists that the filter misses.

**AMCL spends a yaw it does not have.** dead_reckoning carries the IMU's
absolute quaternion, so map->odom yaw is structurally 0. AMCL estimated it
anyway and wandered it 88-587 deg cumulatively per run; corr(err_yaw,
m2o_yaw) = 0.85, and removing it takes the follower's heading error from
0.81 to 0.43 deg std.

**Localizer v2** (`localizer:=v2`, `racer_localization/localization_v2.py`):
solve translation only, per scan, by Gauss-Newton on the grid's signed
distance field; fuse the 2x2 information matrix in the car frame, clamped per
axis against the filter's own sigma; rate-limit the published correction per
segment. Segments come from `tools/segment_track.py`, computed from the map.

**What shadow run 1 changed in that design** (57 s beside AMCL on identical
scans, replayed offline in seconds with `tools/replay_localization_v2.py`):

1. *The per-segment along GAIN is gone.* Forcing it to 0 on the blind straight
   was the original rule and it is measurably worse: replayed at gain
   0 / 0.3 / 1.0 the run gives along p90 0.429 / 0.331 / 0.316 (AMCL 0.334).
   The information-form fuse already weights by observability -- where
   along_info is ~0 the correction is ~0 whatever the gain says -- so the
   hand-set gain was redundant with the matrix and only threw away the real
   information the duct-gap beams carry. The parameter stays, defaulted to 1.0
   everywhere. `blind_along_info` remains the floor for a truly blind scan.
   The segment table's real job is the per-mode RATE LIMIT.
2. *A fixed whole-step gate cannot work.* `max_step_m` 0.35 rejected matches
   with inliers 1.00 and residual 0.007 because their ALONG component was
   large -- the component the design was about to discard -- and killed the
   CROSS correction with them. The error grew, so the next step was bigger, so
   it was rejected harder: cross went 0.06 -> 0.43 m through hairpin 1 while
   every scan fitted perfectly. Now each axis is CLAMPED against
   `gate_sigma * sigma + gate_floor`, never dropped, so a large error slows the
   correction and can never lock it out. 242 rejections -> 2.
3. *The covariance needs a floor.* Independent-beam information drove the
   posterior sigma_cross to 0.002 m, past any real map/extrinsic/timing error,
   which made the gate above nonsense. Nothing now claims better than the cell.
4. *"Cumulative walk" was the wrong metric* -- it penalises tracking a real
   drift smoothly, which is correct behaviour. What threatens a 0.09-0.13 m
   wall margin is the per-SAMPLE jump.

Result on that run, offline, against AMCL on the same scans:

| | v2 | AMCL |
|---|---|---|
| along p90 | **0.301 m** | 0.334 |
| cross p90 | **0.027 m** | 0.042 |
| worst per-sample correction step | **21 mm** | **460 mm** |
| corner along p90 | **0.048 m** | 0.063 |

The 20x smoothness is the point: AMCL moves the pose up to 0.46 m in one
sample against a 0.09-0.13 m margin; v2 never exceeds 0.021 m.
### What localizer v2 did on the car (2026-09-19)

Every run below on iros2026, 43-45 Hz loop (the cap works; a `1/median(dt)`
reading of the scan interval says 56 Hz and is wrong -- the distribution is
bimodal, one frame and two, so the rate is scans/second).

| run | localizer | line | laps | contacts |
|---|---|---|---|---|
| `lv_race_2` | **v2** | `b05b15w05_ell_L65_B50_v9.0` | **9 clean, 8.90-9.00, mean 8.93** | 0 |
| `lv_slow_dist` | **v2** (2 machines) | same | **10 clean, ~8.9** | 0 |
| `gt_dist` | ground truth | `tb10_lat875_b55_L70` | 9, 8.40-8.50, mean 8.45 | 0 |
| `lv_L750_warm` | **v2** | `tb10_lat750_hp725_L70` | 8.65, 8.75 | 1 |

So v2 matches AMCL's lap time on the proven line with **zero contacts over 19
laps across two machines**, and it is not the limit on the faster lines.

**Against AMCL on identical scans** (shadow run, replayed offline): along p90
0.301 vs 0.334, cross p90 0.027 vs 0.042, corner along p90 0.048 vs 0.063, and
the worst per-sample correction step **21 mm against AMCL's 460 mm**. That last
number is the one that matters against a 0.09-0.13 m wall margin.

**The odometry scale state.** Dead reckoning over-reads 2.0-3.4 % (o2b vs truth,
six runs), which over the 16 m blind straight is 0.3-0.5 m of along error that
no scan can see. It IS observable at every corner that pins the along axis, so
V2Filter estimates it as a third state and spends it on the straight. Live it
took along p90 from 0.375 to 0.235-0.265; offline across seven logs it roughly
halves the along error. Two honest caveats: the estimator always runs to its
bound, so it is absorbing more than scale and the bound is set from the measured
physics (3.0 %) rather than from what minimises the metric; and it makes a short
low-speed run slightly worse (`lv_margin_2` 0.114 -> 0.174), because there is
no over-read to correct when the car never gets up to speed.

**Where the remaining time is, and it is not the localizer.** On
`tb10_lat750` the car ran 8.65/8.75 and then hit the tight left onto the main
straight. Passes of that corner: 0.081, 0.089, 0.081, 0.113 m of tracking
error -- then one pass at 0.403. Through the failing corner v2 reported
**along error +-0.012 m, cross +-0.015 m, 100 % inliers, 9-13 mm residual**:
the pose was right and the car still drifted from +0.05 to -0.52 m wide at
2.5 m/s. The same corner is where `lv_L750_scale` hit too. The fix is the one
section 11 used on this class of corner -- a lateral cap there (`--lat-zones`,
what `bendz65` does on the line that runs 57 clean laps) -- and the whole tb10
family lacks one. Ground truth clears it, so it is the follower plus the
line's margin, not the estimate.

**Race choice.** A contact costs 10 s, so over ten laps the proven line at
8.93 with no contacts beats `tb10_lat750` at 8.70 with a contact every four
laps (about 11.2 effective). Race `b05b15w05_ell_L65_B50_v9.0`.

**Not yet validated:** the 19 clean laps were driven BEFORE the scale state
existed. It is on by default now and is neutral-to-better on that line
offline (along p90 0.298 -> 0.165), but one confirmation run on the race line
with the current build is owed before trusting it in a race. `est_scale:=false`
restores the validated behaviour exactly.

### Bugs the offline harness caught before any simulator time

Each would have read as "v2 is worse than AMCL":
the information matrix is a beam COUNT and must be divided by the per-beam
noise variance before it meets P^-1 (the fuse took 2 % of a correct
correction); a cross-only measurement moved the along-track state through the
covariance coupling (0.92 m of forbidden walk on the straight; the state
update is now projected onto the gain subspace); the plain distance transform
is zero at cell CENTRES, not wall faces, which pulled the estimate 1-2 cm
toward the end wall and flattened every gradient at the solution (now a signed
field); the segment table must be built with the node's own beam cuts; and a
dozen beams through the middle wall's duct gap are real but ambiguous, so the
table's blind threshold is 20 while the live guard stays at 5.

### Bugs only the car caught

- **A fixed whole-step gate cannot work.** `max_step_m` 0.35 rejected matches
  with inliers 1.00 and residual 0.007 because their ALONG component was large
  -- the component the design was about to discard -- and killed the CROSS
  correction with them. Cross error went 0.06 -> 0.43 m through hairpin 1 while
  every scan fitted perfectly. Each axis is now CLAMPED against
  `gate_sigma * sigma + gate_floor`, never dropped. 242 rejections -> 2.
- **The per-segment gains and rate limits were both stale proxies.** Forcing
  the along gain to 0 on the blind straight is WORSE than not (along p90 0.429
  vs 0.316): the information-form fuse already zeroes what it cannot see. And a
  per-mode rate limit throttled the correction at hairpin 1 while the table
  still said "blind" and the live information said otherwise -- it removed
  0.239 m in 0.75 s, exactly the 0.30 m/s limit, saturated all the way in, and
  the car turned in carrying 0.32 m. Both are uniform now; the segment table
  survives as a LABEL for analyze_run's per-mode breakdown, which is how both
  bugs were found.
- **Publishing map->odom only once seeded deadlocks the bootstrap**, whose
  `ready_check:=tf` waits for that transform BEFORE it sends the seed. The
  first `localizer:=v2` run sat in "waiting for TF map -> odom" for the full
  30 s and the car never moved. v2 now starts on the track's registered spawn
  as a fallback prior, exactly as slam_toolbox starts on `map_start_pose`.
- **The warmup release is a speed step if it lands where the profile is fast.**
  `warmup_dist_m:=21` lifts the cap 5 m before a corner on the tb10 line, so
  the car accelerates 2.0 -> 5.6 m/s into it and runs wide; the same corner is
  passed cleanly at a HIGHER speed on every subsequent lap. `frames.py` states
  the rule -- release where the profile is already slower than the cap -- and
  36 m satisfies it on that line.

## 13. Running the fast lines on localizer v2 (2026-09-19, afternoon)

Goal set by the user: ten clean laps at about 8.5 s, race-legal, on v2. Every
run below is the docker stack (`scripts/run.sh race`, 45 Hz loop), a fresh
headless simulator per run, `warmup_dist_m:=28`, `steer_a_lat_max:=8.0`.
Ground-truth baselines are the same command with `truth`. Contacts are read
off the log with the localizer's error at the last moving sample before the
teleport, which is the number that says whether the estimate put the car there.

### The first contacts were never the localizer

| run | line | where | car, true | v2 cross | v2 along |
|---|---|---|---|---|---|
| lv_fast_19_1, 19_2, lv_fast_2, 3, lv_L750, lv_L750_scale | tb10 (any rung), warmup 21 | lap 1, s 26.2, exit of the left after the cap lifts | 0.20-0.22 m wide | <= 0.022 | <= 0.13 |
| gt_dist (ground truth) | tb10 lat875 | same corner, lap 1 | 0.211 m wide, survived by ~3 cm | | |
| gt_dist laps 2-9 | | | 0.10-0.12 | | |

The lap-1 launch out of the 21 m warmup cap goes straight into the s 21-27
sequence at full throttle; the true-pose car clears that corner by 3 cm on
lap 1 and by 10 cm afterwards. Every localized run hit it. `warmup_dist_m:=28`
moves the launch past it: `v2_L750_w28` then ran 9 clean laps at 8.70 flat,
exactly the ground-truth car's 8.70-8.75 on the same line (`b_truth_L750`).

### What the localizer does cost, measured

Same corners, per-lap worst wide error, ground truth vs v2, later laps:

| zone | truth | v2 |
|---|---|---|
| hairpin-1 exit (s 18.5-20.8) | -0.11..-0.13 | -0.13..-0.15 |
| s 36-40 | -0.11 | -0.12..-0.13 |
| launch corner (s 24-27.5) | -0.11..-0.12 | -0.12..-0.14 |

Two centimetres, everywhere, which is v2's cross error (p90 0.027-0.030 m
on every run today; the map cell is 0.025). At the hairpin-1 exit it is a
**fixed -2.2 cm bias**: replayed offline with sigma 0.08/0.12/0.16, beam
stride 2/3, range cut 6/9.5 m and the cross rate limit off, the bias does not
move by a millimetre, so it is the SLAM map's wall at that spot, not the
matcher. The follower's own lateral error under v2 disagrees with the truth
by 4-5 cm at the apex (1 cm when driving on truth): the bias plus the
estimate's noise. A time-shift fit finds no extra latency in the TF path
(both minimise at the same 25 ms).

### Where the localized car slides, and why

At the grip limit those centimetres decide. Hairpin-1 exit passes on v2, by
hairpin budget (`lat_zones` 16.5-21):

| hairpin a_lat, rest of lap | passes | slides (>= 0.25 m wide) | runs |
|---|---|---|---|
| 7.0, lat 6.5 (the race line) | 20 | 0 | lv_race_2, lv_slow_dist |
| 7.25, lat 7.5 | 17 | 0 (one at 0.18) | v2_L750_w28, v2_L750_w21 |
| 7.0, lat 8.75 | 39 | 2 | v5_L875h700a (15 clean, then one), v5_L875h700b (9 + 10 clean) |
| 7.25, lat 8.0-8.75 | 31 | 3 | L800h725, L850h725, L875h725 (10 clean at 8.50-8.55, then one) |
| 7.25, lat 8.75, exit guard on | 12 | 3 | v3_L875h725 |
| 7.25, lat 8.75, exit drive budget 5.5 / 4.5 | 14 / 13 | 2 / 3 | v4_L875s55, v4_L875s45 |
| 7.25, lat 8.75, exit geometry 3-8 cm inside | 37 | 5 | v7_x725a (12 clean, then two), v7_x725b |
| 7.5 | 16 | 2 | v2_L775_w28, v2_L800_w28 |
| 7.75 | 10 | 2 | v2_L825_w28 |
| 8.0 | 5 | 2 | v2_L850_w28 |
| 8.0 / 7.25, ground truth | 18 | 0 | gt_dist, b_truth_L750 |

A slide is the car going from -0.13 to -0.30..-0.53 m in 0.3 s under
acceleration at s 19.2-19.8, pose right to 1-3 cm every time. The inputs of a
sliding pass are not distinguishable before it starts (apex speed 2.28-2.40 vs
2.23-2.28, throttle within the normal range); it is a marginal-grip event with
about a one-in-ten incidence per pass at 7.25 and rising above.

Levers tried, in the sim:

- **Exit guard** (`exit_guard_from:=0.15 exit_guard_full:=0.25`): three
  slides in 12 laps and 0.05 s slower. It trims speed after the car is wide.
- **Drive budget lowered over the first 1.6 m of each exit**
  (`enforce_friction_ellipse.py --long-zones 18.3:19.9:5.5,41.8:43.23:5.5`,
  new option): worse, 2 and 3 slides in 14 and 13 laps at 8.60-8.65. On
  every sliding pass the car is 0.15-0.25 m/s OVER its profile at s 19.5
  whatever the profile says there; lowering it only widens that gap.
- **Hairpins at 7.0 instead of 7.25** (`rl_mt_tb10_lat875_hp700_b55_L70`):
  0.08 s slower (8.55-8.65) and still one slide per 19-20 passes.
- **The race line's exit geometry blended in** over s 18.6-22.0
  (`rl_mt_tb10x_*`, 3-8 cm inside, 5-9 cm more room to the outside wall):
  12 clean laps at 8.55-8.60, then two slides; the repeat slid five times. A
  slide that starts at -0.13 m ends at -0.35..-0.53 m and in the wall with
  0.77 m of room, so room does not save it.
- **Command-delay preset** (`cmd_delay_s:=0.125`): the estimator starts at
  the host's 0.175 and learns 0.125 over laps 1-2 in the container; preset,
  lap 1 stopped hitting the s 39 corner (three lap-1 contacts there before).
- **Wheelspin cap in v2** (`odom_accel_max`): built for v2_L850h725 lap 2,
  where the launch out of hairpin 1 read 20-30 % more odometry than the car
  covered for 0.25 s and the car hit at s 27.4 with the estimate 0.39 m ahead.
  Fixed that run offline, wrecked four others (the windowed wheel speed is
  too noisy to gate on). Ships disabled.

### Recovery, fixed

Before today a contact cost more than 10 s: `lv_fast_19_1` spent 7.2 s
recovering and hit twice more. Four causes, all in the log:

1. **Phantom odometry.** After the reset the wheel reads 0 but the tire
   observer decays from 6 m/s over a second, and the `slip` source integrated
   that as 3.3 m of travel while the car sat still; the first seed was rejected
   2.2 m off. `slip_wheel_min_m_s`: no slip term from a stopped wheel.
2. **A reset neither signature saw** (heading step 8.8 deg, encoder reading the
   command): the follower drove on with 0.6 m of error and hit again a second
   later. Signature 3, `reset_scan_jump_m`: a teleport moves every beam.
3. **Wrong or missing checkpoint.** Two added from ground truth
   ((3.323, 2.124), (0.884, 4.715)); adjacent checkpoints a metre apart are
   ambiguous to the centimetre (the trigger fires when the body enters), so a
   scan-confirmed estimate that settled off the prior is accepted
   (`recover_offprior_m`) and the matcher lets a clean fit take a step above
   `max_step_m` instead of refusing the right pose for 2.3 s.
4. **The held car drove on.** The bridge keeps the last command; the follower
   published nothing when held and the bootstrap's single zero lost the race
   once: 0.17 throttle, 0.9 -> 4.2 m/s, into the hairpin-2 wall. Both now
   publish an explicit zero, every tick while holding.

With v2 the lidar creep after confirmation is pure exposure (it drove into
the wall at hairpin 2 and twelve times in a row at s 21.6): `run.sh` passes
`recover_creep_s:=0.0 recover_settle_s:=1.0` for `LOCALIZER=v2`. Recovery is
now 3-5 s with the seed confirmed on the first attempt at 2-12 cm.

### The slide, solved on the follower side (runs 1-5, evening)

With the user driving the runs and the analysis coming back per run:

- **Run 1** (delay preset only): 11 laps at 8.55-8.60, a hairpin-1 slide, and
  a second contact 8 s later at s 39. The second one was the follower's delay
  estimator: through the reset its correlation window held a standstill and a
  launch and it read 0.114 -> 0.043 s in 4 s, so the car led by a third of the
  real delay and arrived at s 35-37 0.4-0.6 m/s over target. Fixed: the
  estimator is frozen while the follower is held and for one window after
  (`pure_pursuit._delay_hold_until`); it has read 105-125 ms ever since.
- **Runs 2-3**: same slides (hairpin 2 too), pose right to 1 cm every time.
  Sample by sample, a sliding pass differs from a clean one in ONE thing: the
  heading error keeps growing past -0.25 rad at s 18.7 (-0.36, -0.45) while
  the steering sits at 0.84-0.89 against 0.72-0.82 on clean passes. That is
  front-limit understeer, and pure pursuit answers it with more steering. A
  slide-onset guard that coasts the wheel (`exit_slide_rate_m_s`, new,
  off) fired on every slide and stopped none.
- **Run 4**: steering cap `steer_a_lat_max` 8.0 -> 7.5 (matched to the 7.25
  hairpins instead of the truth car's 8.0). 30 laps at 8.55-8.65, steering
  peaks down 0.04, slides still on 3 of 32 hairpin-1 passes and 3 of 32
  hairpin-2 passes: not the controller asking for more than the plan.
- **Run 5**: `lookahead_min` 0.80 -> 1.0 m. The lookahead sat on its floor
  through both hairpins, so the follower chased the apex with the tightest
  geometry it had. **39 timed laps at 8.50-8.55 s, zero contacts.** Over 40
  hairpin-1 passes: worst 0.173 m wide (median 0.149), steering peak 0.75
  (median 0.70), heading error never below -0.28; hairpin 2 worst 0.065 m.
  The apex moved 5 cm inward (+0.09 vs +0.04, 1.19 m of room there), the
  exit straightened, and the lap time did not move.

The 2 cm map bias at the hairpin exit is still there; the follower no longer
sits close enough to the tire's peak for it to matter.

### The recipe (promoted; registry default and `run.sh race` since 2026-09-19)

| item | value |
|---|---|
| localizer | v2 (`localizer:=v2`; `run.sh` default) |
| line | `raceline/iros2026/rl_mt_tb10_lat875_hp725_b55_L70.csv` (tb10 geometry, lat 8.75, hairpins 7.25, brake 5.5, predicts 8.415) |
| warmup | `warmup_v_max:=2.0 warmup_dist_m:=28` (released after the launch corner) |
| steering cap | `steer_a_lat_max:=7.5` |
| lookahead floor | `lookahead_min:=1.0` |
| command delay | `cmd_delay_s:=0.125` (`CMD_DELAY=`), estimator frozen through recoveries |
| recovery | `recover_settle_s:=1.0 recover_creep_s:=0.0 recover_warmup_dist_m:=14` |
| loop | 45 Hz (`tcp_nodelay:=true loop_hz_cap:=45 control_hz:=45`), slip dead reckoning, hybrid LQR as M15 |
| image | `autodrive_racer:multi-track` built from this tree (`./scripts/build.sh`) |

    ./scripts/run.sh sim            # then
    ./scripts/run.sh race NAME      # exactly my_run_5's configuration

Measured: my_run_5, 354 s, 39 timed laps 8.50-8.55 (best 8.50, profile
8.42), zero contacts, tracking p90 0.147 m max 0.231, v2 cross p90 0.026,
along p90 0.160, encoder/true 1.024. Before it the same line without the
lookahead floor: one hairpin-1 slide per 10-15 laps in six runs.
