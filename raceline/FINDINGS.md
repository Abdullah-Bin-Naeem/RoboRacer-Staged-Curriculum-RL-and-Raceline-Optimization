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
(6.36). That gap is the 175 ms round trip: the car brakes early and runs 2-3 %
under the profile through the two tight corners because it is reasoning about a
car 175 ms in the past. Nothing on the car side moves it.

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
