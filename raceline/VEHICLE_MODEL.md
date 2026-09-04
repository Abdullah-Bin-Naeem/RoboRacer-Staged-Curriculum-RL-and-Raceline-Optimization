# The simulated car, from the simulator's source

Everything numeric the raceline and velocity-profile code assumes about the
RoboRacer, with where each number came from. Derived 2026-09-03 from the public
AutoDRIVE Simulator Unity project, branch `AutoDRIVE-Simulator` at commit
`bb0c5018` (2026-08-10), cross-checked against the competition guide, the
AutoDRIVE papers, Unity's documentation and this repo's own measurements.

The published guide tabulates the parameters. The source shows how they are
*used*, and that is what changes the engineering conclusions. Section 3 is the
part to read if you only read one.

---

## 1. Sources

| what | where |
|---|---|
| Competition guide (parameter tables, API, rules) | https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2026/ |
| Simulator source (Unity project) | https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator |
| Vehicle controller | `Assets/Scripts/VehicleController.cs` |
| Car prefab: Rigidbody, WheelColliders, encoders | `Assets/Prefabs/F1TENTH/F1TENTH.prefab` |
| Sim-racing scene: per-instance overrides | `Assets/Scenes/RoboRacer - Sim Racing.unity` |
| Sensors, lap timer, socket | `Assets/Scripts/{IMU,LIDAR,WheelEncoder,LapTimer,Socket}.cs` |
| Physics settings | `ProjectSettings/{TimeManager,DynamicsManager}.asset` |
| Unity friction-curve definition | https://docs.unity3d.com/ScriptReference/WheelFrictionCurve.html |
| Unity WheelCollider manual | https://docs.unity3d.com/Manual/class-WheelCollider.html |
| AutoDRIVE Ecosystem paper (suspension, tire, actuator equations) | https://arxiv.org/abs/2212.05241 |
| AutoDRIVE Simulator paper | https://arxiv.org/abs/2103.10030 |
| F1TENTH digital twin, MARL racing | https://arxiv.org/abs/2309.10007 |
| Sim2Real with AutoDRIVE | https://arxiv.org/abs/2307.13272 |
| Validation across scales (states linear drag for small vehicles) | https://arxiv.org/abs/2402.12670 |
| This repo's measurements | `CLAUDE.md` (measured constants), `racer_control/config/pure_pursuit.yaml`, logged runs `amcl*.csv`, `drift*.csv` |

The papers restate the guide's model (rigid body + sprung masses, two-piece cubic
friction curve, slip definitions, Ackermann) and add that drag on small-scale
vehicles is *linear* in velocity. They contain no numbers beyond the guide.
All numbers below are from the prefab, the scene, and the project settings.

---

## 2. Parameters, with provenance

| parameter | value | source |
|---|---|---|
| sprung mass | 3.47 kg | prefab, Rigidbody `m_Mass` |
| unsprung mass | 4 × 0.109 kg | prefab, WheelCollider `m_Mass` |
| total | 3.906 kg | matches guide |
| centre of mass | 0.155 m ahead of rear axle, ~0.06 m up | guide; prefab `COM` |
| wheelbase, track | 0.324 m, 0.236 m | controller fields (mm) |
| wheel radius | 0.059 m | WheelCollider `m_Radius` (0.0581 effective, measured here) |
| wheel damping rate | 0.25 N·m·s | WheelCollider `m_WheelDampingRate` |
| suspension | spring 500 N/m, damper 100 N·s/m, travel 0.05 m | WheelCollider |
| linear drag | 0.273 s⁻¹ | Rigidbody `m_Drag` |
| angular drag | 0.1 s⁻¹ | Rigidbody `m_AngularDrag` |
| motor torque | 428 N·m total, common all-wheel drive | scene override of `MotorTorque` (prefab has 85.6); `driveType` 5 = CAWD |
| brake torque | 428 N·m per wheel when throttle == 0 | `VehicleController.Drive()` |
| steering | ±30°, rate 183.35°/s = 3.2 rad/s | controller `SteeringLimit`, `SteeringRate` |
| forward friction | extremum (0.15, 0.9), asymptote (0.25, 0.58), stiffness 0.8 → effective (0.15, **0.72**) / (0.25, **0.464**) | prefab; stiffness and extremum overridden in scene |
| sideways friction | extremum (0.01, **1.00**), asymptote (0.10, **0.50**), stiffness 1 | prefab |
| encoders | 16 PPR × gear 120 = 1920 ticks/rev, rear wheels | prefab `WheelEncoder` |
| physics step | 0.001 s, 6 solver iterations | `TimeManager`, `DynamicsManager` |
| LiDAR | 1081 rays, 270°, 0.06–10 m, refreshed every 1/40 s | scene `ScanRate` 40; guide |

Frame table, sensor extrinsics and topic list are in the guide and already in
`racer_common/frames.py`.

---

## 3. How the numbers are used, and what follows

### 3.1 Throttle commands a wheel speed, not a torque

```
per-wheel motor torque   T = (428 / 4) · θ                      θ ∈ [−1, 1]
wheel damping torque     −d · ω,  d = 0.25
free-spin wheel speed    ω = T / d          →   u = r·ω = 0.059 · 107 θ / 0.25 = 25.25 · θ  m/s
wheel inertia            I = ½ m r² ≈ 1.9e-4 kg·m²   →   time constant I/d ≈ 0.8 ms
tire reaction torque     F·r ≤ 6.9 N × 0.059 m = 0.41 N·m     (< 4 % of T at θ ≥ 0.1)
```

The wheel reaches its free-spin speed within one physics step and the tire
barely slows it. So **the throttle sets the wheel surface speed
u = 25.25 θ**, the car speed v follows through the tire, and the
longitudinal slip is

```
S = (u − v) / max(|v|, 4 m/s)
```

The denominator is PhysX's `minLongSlipDenominator`, default 4 m/s (Vehicle
SDK docs; Unity exposes no setting for it). Below 4 m/s slip is therefore a
wheel-speed *difference* over 4, not a ratio: peak force from rest needs
u = v + 0.6 m/s, and u = 1.15 v at 0.5 m/s is almost no force at all. Measured
2026-09-04: with a percentage band the car sat at 0.5–0.9 m/s for five
seconds; with u − v ≥ 0.5 m/s it pulled 5.3–5.9 m/s² at every speed below 4.5.
This is also why the old launch data showed 1.24× encoder overread at
1.5–2.5 m/s for 3–6 m/s² of acceleration.

Three consequences:

- **The encoders measure the throttle command, not the car.** They read u.
  Under acceleration they overread by the slip; at launch (v ≈ 0) they read
  25.25 θ while the car barely moves; under braking they read zero while the car
  slides. They agree with the car only after it has settled to the wheel speed.
- **The motor never limits acceleration; the tire does.** 428 N·m is about 1000×
  what the tire can transmit.
- **Braking is a wheel-speed command too.** Any u below 0.75 v is past the
  asymptote and gives the locked-wheel force. Exactly θ = 0 applies 428 N·m of
  brake torque and locks the wheels outright.

### 3.2 The friction curves: peak versus asymptote

Unity's curve is a two-piece cubic through (0, 0), the extremum and the
asymptote, with zero tangent at the extremum and the asymptote, and flat beyond
the asymptote. The initial slope is not documented.

| | peak | window where the peak is available | asymptote (any slip beyond) |
|---|---|---|---|
| longitudinal | 0.72 g = **7.06 m/s²** at slip 0.15 | slip 0.10 to 0.25 | 0.464 g = **4.55 m/s²** |
| lateral | 1.00 g = **9.81 m/s²** at 0.57° | roughly 0.3° to 5.7° | 0.50 g = **4.90 m/s²** |

Force per wheel is μ(S)·N, linear in normal load, so load transfer does not
change the total: 4 wheels at the same slip give μ·m·g whatever the transfer.

The **asymptote is the force the tire gives no matter how badly it slides**. A
demand below it is always met and the car cannot run away from it. A demand
between asymptote and peak is met only while slip stays inside the window,
which is half a degree wide laterally; a pure-pursuit follower with ~100 ms of
loop latency cannot hold that, and once slip leaves the window the force drops
*below* the demand. That is what "the car drifts" and "the encoders lie" are.

Measured on this car, sampled only while within 10 cm of the line: lateral
acceleration p50 3.77, p90 4.74 m/s². The p90 sits on the 4.90 asymptote.
The speed ladder behaved exactly as the asymptotes predict:

| rung (a_lat / a_long) | against asymptotes 4.90 / 4.55 | observed |
|---|---|---|
| 4.0 / 3.5 | both under | holds |
| 5.0 / 4.0 | lateral just over | edge |
| 6.0 / 5.0 | both over | washed wide at 5.5 |
| 8.0 / 6.5 | both over, a_long at 92 % of peak | drifts, encoders 3× off |

### 3.3 Drag is linear

`Rigidbody.drag = 0.273` gives a_drag = −0.273·v. At 8 m/s that is 2.2 m/s²,
comparable to the tire's robust longitudinal limit. The velocity-profile tool
(TUM `trajectory_planning_helpers`) models drag as −c·v²/m, which cannot
represent this; a least-squares quadratic is fitted over the speeds the profile
visits (see 5.1).

### 3.4 Available accelerations

| regime | available a_x | encoder behaviour |
|---|---|---|
| accelerating, any slip (robust) | 4.55 − 0.273 v | overread by (u−v)/v |
| accelerating, slip held at 0.15 (peak) | 7.06 − 0.273 v | overread 15 % |
| braking with θ = 0 (locked) | 4.55 + 0.273 v | read zero |
| braking with u held at 0.85 v (peak) | 7.06 + 0.273 v | underread 15 % |

At v = 2, 4, 6, 8 m/s: robust acceleration 4.0, 3.5, 2.9, 2.4; locked braking
5.1, 5.6, 6.2, 6.7.

### 3.5 Steering rate as a path constraint

The steering angle is rate-limited to 3.2 rad/s toward its setpoint (full lock
in 0.16 s). With δ = atan(κL), a path is only followable if

```
|dκ/ds| · v · L / (1 + (κL)²)  ≤  3.2 rad/s
```

At 8 m/s on a straight that is |dκ/ds| ≤ 1.2 m⁻². Neither min-curvature
optimizer enforces it; `optimize_raceline.py` checks it after the fact.

### 3.6 Sensing and timing facts that matter

- **Odometry twist is the centre-of-mass velocity** in body axes
  (`IMU.cs` transforms `Rigidbody.velocity`). Rear tire slip angle is
  `atan(v_y/v_x) − atan(0.155 κ)`, not the body sideslip.
- **The IMU's acceleration is the world-frame derivative rotated into the
  body**, `(v(t) − v(t−dt))/dt` then `InverseTransformDirection`, with no
  gravity term (`GravityCompensation` defaults on). So `a_x = v̇ − ω_z v_y`,
  and with the CoM's kinematic `v_y ≈ 0.155 ω_z` the x axis under-reports by
  `0.155 ω_z²`: 1.1 m/s² at 2.7 rad/s. It is also sampled once per 1 ms
  physics step, so what the bridge ships at 18 Hz is an instantaneous value,
  not a tick average, and under a throttle law that changes the wheel speed
  at 20 Hz the sampled force is off by up to 2× either way (measured). It is
  not usable for speed integration; the encoder-driven tire observer is.
- **Dead reckoning's heading lagged the scan by one tick.** `odom→base` is
  post-dated and published before tick k's messages arrive, so the transform
  covering scan k carried IMU heading k−1: 8° at the S-curve's yaw rate.
  AMCL absorbed it by rotating map→odom (m2o_yaw swings of 8–14°, 0.5–1.1 m
  of cross error at s ≈ 20 m on every run). `dead_reckoning` now extrapolates
  the heading to each stamp with the IMU's yaw rate (`extrapolate_yaw`).
- **LiDAR scans are snapshots**: all rays cast at one instant every 25 ms. No
  motion deskew needed. Returns under 0.06 m become `inf`.
- **Throttle to wheel takes three sim frames: 175 ms at 17.5 Hz.** Regressing
  the measured wheel speed on 25.25 × the published throttle over 12 laps gives
  gain 0.98 and the best fit at a 150 ms lag on a 50 ms grid (RMS 0.18 m/s,
  against 0.59 at zero lag). At 5 ms resolution the trough is 165–180 ms on
  every 17–18 Hz run and 190–220 ms on the 13–14.6 Hz runs, i.e. three frames:
  the command is applied at the next frame, acts over one, and the encoder
  reading returns one later. Any throttle law that reasons about the car's
  current speed is reasoning about a car 175 ms in the past: a 0.4 m/s lead
  commanded against the current speed arrived as 0.04 m/s of wheel slip.
- **The loop is ping-pong.** The sim emits telemetry only in reply to a devkit
  message; the devkit decodes a JPEG and a gzip, publishes, and replies. The
  tick rate (18 Hz on this machine) is machine-dependent, not a sim constant.
  The evaluation machine is an i9-14900K with an RTX 4090. Headless
  (`-batchmode -nographics`) the same machine gives 20.3 Hz, and that rate is
  NOT compute-bound on either side: the simulator pinned to a single core
  (verified on all 67 threads) still ran 20.3 Hz, its two busy threads spin;
  the bridge uses 15 % of one core and, pinned beside a spinning hog, still
  gave 19.8 Hz (runs 32, 34). The ~20 Hz is the simulator's own socket loop;
  rendering costs about 2.5 Hz on top. Expect the evaluation machine to sit
  near 20 Hz too, not far above it. A loop slower than that comes from load,
  not from a slow CPU: this laptop decayed to 12.8 Hz within a long graphics
  session.
- **Every transform must be extrapolated to its stamp.** The odometry
  transform is post-dated by 20 ms so "now" lookups land inside its window;
  the sample that covers frame k's scan is therefore published before frame k
  arrives, and AMCL reads frame k−1's heading and position for scan k unless
  the node carries both forward, with the IMU rate and the wheel speed (5.4).
  Measured cost of not doing so: 8–14° map→odom swings at the S-curve, and a
  35–40 ms × speed lead of the estimate at every tick rate. The other half of
  the same trap: a chain lookup of map→base at the latest time is served from
  the localizer's post-dated map→odom (nav2 `transform_tolerance`, 0.5 s)
  *interpolated half a second back*, so the controller got every correction
  and every heading swing 0.5 s late. The follower composes the newest sample
  of each leg instead (`compose_latest`, 5.3).
- **A wall contact respawns the car** at the last checkpoint with velocity
  zeroed and `collision_count += 1` (`LapTimer.OnCollisionEnter`). Wall margin is
  worth more than the lap time it costs.
- Lap time increments every physics step; a lap counts at the finish-line
  trigger after all checkpoints (21 on the public 2026 ICRA scene).
- A variable-friction scene and a per-location friction map exist in the sim
  (`TireFriction.cs`). They are not enabled in the sim-racing scene.

---

## 4. Validation against this repo's measurements

| quantity | model | measured or documented |
|---|---|---|
| steady speed at θ = 0.10 | 2.50 m/s | ~2.5 m/s (`pure_pursuit.yaml`) |
| cruise speed per unit throttle | 24.6 to 25.2 | ≈ 24 (`CLAUDE.md`) |
| top speed at θ = 1 | 22.6 to 22.9 m/s | 22.88 m/s (guide) |
| deceleration with locked wheels near 6 m/s | 6.2 m/s² | 6.2 to 6.7 m/s² (longest logged run) |
| peak acceleration, 0.5 s smoothed | ≤ 7.06 | 4.9 to 5.4 across four runs |
| launch encoder overread | u/v, unbounded as v → 0 | median 5× to 12× from rest, ~1.1× from 2.5 m/s |

The top speed is emergent (there is no speed cap in the controller): torque,
damping, tire curve and drag alone land within 1.5 % of the documented value.

---

## 5. Implications used in this repo

### 5.1 Velocity profile (`optimize_raceline.py`)

TUM `calc_vel_profile` interpolates `ggv` rows `[v, a_x, a_y]` by speed and
uses a_x for braking and, after the friction ellipse, for acceleration;
`ax_max_machines` rows `[v, a_x]` cap acceleration only; drag is
`−c·v²/m` and is applied *after* the ellipse. The encoding used:

```
ggv               = [[0, 4.55, a_lat], [v_max+1, 4.55, a_lat]]     tire, symmetric
ax_max_machines   = same a_x                                        the motor never limits
drag_coeff        = least-squares fit of c·v²/m to 0.273·v over [2, v_max]
                    → c = 0.166 for v_max 8; error under ±0.4 m/s² in that range
a_lat             = 4.5 default (92 % of the 4.90 asymptote); ladder 4.0 / 4.5 / 4.9
dyn_model_exp     = 2  (the sim evaluates the two axes from separate slips and
                        curves; coupling is undocumented, so the ellipse is the
                        conservative choice)
```

Why a fitted quadratic rather than folding the linear drag into the tables:
tph applies no drag at all where the ellipse binds, so a table with
`4.55 − 0.273 v` on the machine side is exact on straights but over-allows
corner-exit acceleration by 0.273 v and, if braking is left at 4.55,
under-estimates braking by twice that. The fit is the smaller error everywhere.

### 5.2 Raceline

Minimum curvature remains the right objective: corner speed is √(a_lat/κ) and
a_lat is fixed at the asymptote. The line must also respect the steering-rate
bound (3.5) and carry a body margin to the wall, because a touch is a respawn.

### 5.3 Controller (`racer_control/pure_pursuit.py`, implemented 2026-09-04)

- **Speed source.** `speed_source: tire` simulates the car on the measured
  wheel speed through the sim's own curve and drag, `v̇ = sign(S) μ(|S|) g −
  0.273 v` with `S = (u − v)/max(v, 4)`. Two copies of that system driven by
  the same u converge, so the error is bounded by the curve's slip error
  times the denominator and cannot drift; offline against ground truth on
  four runs it gives p90 0.17–0.28 m/s with `tire_rise_slope 3`. The IMU and
  pose-based estimate (`fused`) is kept for reference and failed twice: the
  sim's IMU is a 1 ms sample of a force that a 20 Hz throttle law makes jump
  every tick (+6.3 at launch against a true 4.0, −7.6 in a corner against
  −3.6), and the pose speed between localizer updates is dead reckoning, i.e.
  the encoders, so its correction first deadlocked the car at 0.6 m/s and,
  once rejection was removed, chased wheelspin upward. A heading jump the yaw
  rate cannot explain resets the estimate: that is a wall respawn.
- **Throttle as a slip command.** `throttle_mode: slip` commands the wheel
  speed u = target, clamped to `v_land ± slip · max(v_land, 4 m/s)` with slip
  0.08 both ways, where `v_land = v + a_plan·0.15 s` predicts the speed when
  the command reaches the wheel (3.6) from the *profile's* planned
  acceleration at that point of the path, used only when it points toward the
  target. Predicting with the observer's own acceleration closed a loop of
  delay 0.15 s and gain about 5 and oscillated at 1 Hz. Inside the band the
  command is `u = v_t + 0.76·(v_t − v_land)`, the legacy law's proportional
  lead (0.76 = 25.25 × throttle_kp) applied to the error at landing time;
  without it tire mode tracked the target 0.15 m/s low against legacy's 0.07
  and lapped 0.05 s slower on average (run 5). The 4 m/s is PhysX's minimum slip
  denominator (3.1); below it the band is ±0.5 m/s of wheel speed, which is
  what launches the car from rest at near-peak grip. The edges bind only at
  launch, restarts and when the target is far from the car; at 0.15 they
  braked at 7 m/s² into corners the profile planned at 4.55. The old law,
  `ff·v_t + kp·(v_t − v_enc)` with ff = 1/25.25, is kept as `legacy` for A/B.
- **Status.** `~/status` publishes v_est, encoder speed, pose speed, target,
  wheel command, throttle, steering, lookahead, lateral and heading error, path
  kappa and s, the commanded slip and the raw IMU a_x every tick.
  `log_localization` records them as `pp_*` columns; `raceline/analyze_run.py`
  reports lap times, tracking, corner bias, weave, estimator error, encoder
  ratio and the IMU integral's gain per run.
- **The command delay is measured online** (`cmd_delay_auto`): the last 6 s
  of published throttle and measured wheel speed are cross-correlated every
  2 s at lags 0 to 0.35 s and the best lag is blended into `cmd_delay_s`. The
  round trip is three simulator frames and moved from 175 ms at 17.3 Hz to
  210 ms at 14.2 Hz as a long session decayed (blamed on a GPU driver change
  at the time; a restart with the same driver setting gave 17.5 Hz again),
  taking 0.2 s a lap with it; the evaluation machine will sit somewhere else
  again. The lookahead scales with
  the measured delay over `lookahead_delay_ref` (0.175, capped at 1.2×) and the
  speed targets derate from the profile's limit to `derate_a_lat` (6.0) as the
  delay goes from 0.175 to 0.21 s, so a slower machine loses lap time, not
  margin. The references are the delays at which the 7.0 profile was proven
  clean and at which it hit.
- Latency compensation for steering exists (`latency_comp_s`) but is off: the
  lookahead was tuned with the delay in, so the two move together.

### 5.4 Dead reckoning (`racer_localization/dead_reckoning.py`)

Position is the encoder arc length integrated along the IMU heading, published
at 200 Hz as `odom → roboracer_1` post-dated by 20 ms. Both the heading and
the position are carried forward to each transform's stamp (`extrapolate_yaw`,
`extrapolate_pos`): the transform that covers scan k is published before frame
k's messages arrive, so without extrapolation it carries frame k−1 and AMCL
pairs every scan with odometry one frame old. The heading half of that was the
±8–14° map→odom swings at the S-curve (3.6). The position half was measured
on runs 16–22: the estimate led the true position by 35–40 ms × speed at every
tick rate from 13 to 18 Hz, 0.30 m at 6.5 m/s, and shifting the truth by 40 ms
removes it (along error 0.21 → 0.08 m rms, no residual offset). That lead sat
in the follower's speed-target lookup as 40 ms of extra preview, plus a
±28 ms sawtooth with the frame phase. Fixed 2026-09-04, late. From then on
`log_localization` compares the estimate and the truth at the truth's stamp,
so localization-error numbers from run 23 on are not comparable with earlier
runs, which measured the latest estimate against a truth one frame old.

The encoders still overread under slip (2–4 % at cruise; `distance_scale` is
left at 1.0). The car's speed follows the one-state model
`v̇ = 4 F(S)/m − 0.273 v` with `S = (u − v)/max(v, 4)`; the follower runs that
observer (5.3) and `dead_reckoning` could take it too, which would matter most
for slam_toolbox, whose ±0.5 m correlative search has less room than AMCL's
`alpha3`.

---

## 6. Open items and how to measure them

- **Rising-branch slope.** The 22.88 m/s top speed implies the forward curve
  reaches 88 % of peak by slip 0.10, steeper than a symmetric cubic. Measure with
  constant-throttle steady states: each run gives one point
  (S, μ) = ((25.25 θ − v)/v, 0.0278 v).
- **Lateral curve.** Steady circles at fixed steering; μ_y = v²κ/g against the
  rear slip angle from 3.6.
- **Combined slip.** Unity's manual documents separate curves per axis and says
  nothing about coupling. If the axes are independent, the ellipse is
  conservative and exponent 2 can be raised.
- **Tick rate on the evaluation machine.** Everything latency-tuned (lookahead,
  RL control period) should be re-checked there.

---

## 7. History and results

### ICRA 2026 track (branch `multi-track`)

| run | line | laps | best / mean | note |
|---|---|---|---|---|
| icra 1 | a4.0, v_max 7.0 | 8 | 14.70 / 14.75 | first drive after mapping: clean, seed 1.6 cm; no log (logger had a pre-layout map path, fixed) |
| icra 2 | a4.0, v_max 7.0 | 13 | 14.70 / 14.75 | clean; tracking 0.062 m, equal-time localization 0.115 m, bias +0.05 inside, delay 172 ms = three frames at 17.7 Hz. The car model and controller transferred unchanged |
| zone | a4.0 ladder regenerated with `--margin-zones 45.5:51.5:L:0.20` | | | 14.23 predicted | run 2 ran 0.02-0.07 m from the outer wall on every lap at the hairpin exit (s 48-50, car 0.10-0.13 m outside the line, line 0.26 m from the wall): Porto's R1 pattern. Line now 0.43 m off that wall, 0.5 m shorter, 0.09 s faster on paper |
| icra 3 | a4.0 zoned, v_max 7.0 | 4 | 14.65 / 14.69 | clean; zone validated: clearance at the hairpin exit 0.02 -> 0.20 m. Tightest spot now 0.11 m at s 36, the car cutting 0.14 m inside the middle-wall hairpin at 3.3 m/s (lookahead chord; safe direction, watch on the climb). Ladder climb starts |
| icra 4 | a5.0, v_max 7.0 | 5 | 13.45 / 13.47 | clean; bias +0.06 inside; hairpin-exit zone 0.17 m; s 36 inside cut unchanged at 0.11 m across rungs, so geometric (lookahead chord), to get an inside zone after the climb |
| note | | | | | Correction: the s 36 spot is not a cut. Per-sample, the car is on the line to 2 cm and the line runs along that lane's right wall at the design margin; the "+0.17 inside" was a 5 m section average mixing the hairpin with the straight. No zone there. A per-side clearance clamp was added to the width cast as a guard for walls ahead of the normal, but the failure that prompted it did not exist |

### Porto


- 2026-09-03: model derived from source; raceline pipeline reviewed and
  rebuilt (`optimize_raceline.py`), lines re-exported.
- 2026-09-04: controller and localization changes driven by six logged runs.
  Findings in order: AMCL heading lag at the S-curve (3.6, fixed in
  `dead_reckoning`); PhysX 4 m/s slip denominator (3.1); the IMU is a 1 ms
  sample and cannot be integrated (3.6); pose speed is the encoders in
  disguise (5.3); 175 ms throttle-to-wheel delay (3.6). Final A/B on the
  a4.9 line, same session, same localizer:

  | | legacy law, encoder speed | tire observer, slip band |
  |---|---|---|
  | laps | 7, best 7.95 s, mean 7.96 | 12, best 7.90 s, mean 7.98 |
  | speed estimate p90 error | 0.30 m/s (raw encoder) | 0.22 m/s |
  | encoders ÷ car, accelerating | 1.01 | 1.00 |
  | tracking error, truth, mean | 0.063 m | 0.059 m |
  | car below target, mean | 0.07 m/s | 0.10 m/s |

  Both are within 0.6 s of the profile's 7.29 s prediction, and both are
  limited by the same round trip. The profile assumes commands act instantly.
- 2026-09-04, later: the ladder, one change per run, same localizer (AMCL),
  same line geometry until the last row:

  | run | change | rung | laps | best / mean | note |
  |---|---|---|---|---|---|
  | 9 | profile lead instead of 1 m preview; curvature-scaled lookahead | 4.9 | 6 | 7.45 / 7.52 | the preview alone was 0.45 s |
  | 10 | | 5.5 | 9 | 7.15 / 7.22 | corner bias −0.011 |
  | 11 | | 6.0 | 6 | 6.90 / 7.00 | first legal sub-7; bias −0.015 |
  | 12 | | 6.5 | 2 then wall | 6.85 | understeer spiral at full lock, S entry |
  | 13 | curvature cap `steer_a_lat_max` 7.0 | 6.5 | 7 | 6.80 / 6.91 | 0 % at lock; per-corner zones tried, slower |
  | 14 | | 7.0 | 5 then wall | 6.70 | straight-line weave, 1.6 m lookahead at 6.7 m/s |
  | 15 | `lookahead_max` 2.2 | 7.0 | 5 then wall | 6.70 | cut the gentle S-exit L, inside wall 0.28 m |
  | 16 | sagitta-bounded lookahead | 7.0 / 6.5 | 11 / wall | 6.70 / 6.80 | 6.5 touched: exits S-entry wide into the L's inside |
  | 17 | line regenerated with 0.30 m left margin on the S-exit approach | 7.0 / 6.5 | 9 / 7 | 6.65 / 6.79, 6.85 / 6.94 | both clean; submittable |
  | 18–20 | GPU driver "performance mode", then the session decaying: sim tick 14.2 → 12.8 Hz, delay 190–210 ms | 7.0 | 7, 0, 0 then wall | 6.80 / 6.90 | left wall on the straight after R1 each time: phase margin lost |
  | 21 | after a restart (17.5 Hz): line widened at both hit sites, delay measured online, lookahead scaled by delay (cap 1.2), targets derated on a slow loop | 7.0 / 6.5 | 5 / 6 | 6.80 / 6.88, 6.85 / 6.96 | clean; tracking 0.060 m, bias −0.058; the derate was 36 % in because the 0.15 reference was a coarse reading of 175 |
  | 22 | delay references moved to the measured 175 ms (three frames); derate and lookahead scale idle on this machine | 7.0 / 6.5 | 9 / 9 | 6.65 / 6.77, 6.85 / 6.89 | clean at 17.6 Hz; delay estimate 173–175 ms; run 17's pace on the widened line |
  | 23 | dead reckoning position extrapolated to the stamp (5.4); logger compares at equal times | 7.0 | 11 | 6.80 / 6.92 | clean; the lead is gone as measured (10 ms), but R1 and the S entry ran 0.12 m wider and 0.15 s a lap slower with the same entry speed and the same cap binding; found the follower's chain lookup serving a 0.5 s-old correction (3.6), fixed for run 24 |
  | 24 | follower composes the newest sample of each TF leg (`compose_latest`) | 7.0 | 7 | 6.70 / 6.76 | clean; loss against the profile 0.15 s a lap, the lowest yet; tracking on the estimate 0.059 m; R1 still 0.06 m wider than run 22, S entry back to −0.13 |
  | 25 | `latency_comp_s` 0.05: the follower steers on the pose propagated 50 ms ahead along the mid-interval heading | 7.0 | 25 | 6.60 / 6.70 | clean at 17.7 Hz, best run; corner bias −0.066; S entry −0.06; the S-exit L now 0.20 m inside; braking zones 9 % slow because the propagated pose also moved the speed-target lookup earlier |
  | 26 | speed target placed from the unpropagated pose (steering still propagated 0.05) | 7.0 | 13 | 6.70 / 6.78 | clean; braking zones back to 5 % but R1 −0.22 and the S entry −0.13: at the profile speed the tire runs wide there and the longer path costs more than braking early. The 50 ms became `target_lead_s` (default 0.05) |
  | 27 | yaml defaults only: `latency_comp_s` 0.05, `target_lead_s` 0.05 | 7.0 | 10 | 6.65 / 6.70 | clean; reproduces run 25 bin for bin. This yaml plus the 7.0 or 6.5 line is the submission configuration |
  | 28 | `target_lead_s` 0.08 | 7.0 | 12 | 6.60 / 6.69 | clean; corner bias −0.04 (R1 −0.11), tracking 0.081 m; braking zones 11 % slow. Adopted as the default: same pace, cleaner line |
  | 29 | simulator headless (`-batchmode -nographics`), `control_hz` 40, logger 50 Hz | 7.0 | 9 | 6.58 / 6.63 | clean; sim tick 20.3 Hz, delay 145–150 ms = three frames again; corner bias −0.024; every lap within 6.58–6.66. The sim is CPU-bound: no rendering bought only 2.7 Hz |
  | 30 | headless, simulator pinned to 4 threads to force a slow loop | 7.0 | 9 | 6.58 / 6.60 | the pin did not bite (20.5 Hz, delay 150 ms), so the derate was not exercised; another clean run, best mean yet |
  | 31 | headless, simulator pinned to 2 threads | 7.0 | 8 | 6.54 / 6.61 | pin still did not bite (20.3 Hz); clean, new best lap |
  | 32 | headless, simulator pinned to ONE core, verified on all 67 threads | 7.0 | 6 | 6.60 / 6.63 | still 20.3 Hz: the sim's frame time is not CPU work, its busy threads spin; the loop is paced elsewhere (bridge or an internal cap) |
  | 33–34 | probes launched from the agent: bridge threads sampled from /proc (15 % of a core), then the bridge pinned to core 0 beside a spinning hog | 7.0 | 8 / 18 | 6.60 / 6.63, 6.56 / 6.64 | 19.6 and 19.8 Hz: the bridge is not the pace either; the loop rate is the simulator's socket cadence. The derate has still not been exercised in the simulator |
  | 35–36 | tried to force a slow loop by SIGSTOP-pausing the simulator in a duty cycle | 7.0 | — | — | broke the simulator's websocket to the bridge (frozen sim misses the ping); no reconnect, 0 rows. Pausing is not a usable way to slow the loop. The derate remains validated only offline (wire test) and by the graphics-session decay that motivated it; a root `tc netem` delay on `lo` is the one clean in-sim test left |
  | netem | `tc qdisc add dev lo root netem delay 10ms` after a sim restart: a genuinely slow loop | 7.0 | 7 | 7.32 / 7.37 | DERATE VALIDATED in-sim. Loop 11.0 Hz, delay measured 254 ms; v_target held below 6.9 (healthy run reaches 7.06), lookahead scaled to 2.52 m; clean, no hit, bias −0.033. Gives back ~1 s a lap and stays on the line, exactly the intended trade |
  | 38 | profile longitudinal limit 4.55 -> 5.0 m/s^2 (`a_long_profile`; the slip band delivers ~5.5, the car beat the 4.55 profile on every exit), same geometry | 7.0 | 7 | 6.50 / 6.58 | clean; R1 −0.11, S-entry −0.03, wall clearance L 0.07 R 0.10 m — margins intact, 0.08 s a lap faster. Closest to the 6.46 record |
  | 39 | longitudinal 5.5 | 7.0 | 10 | 6.50 / 6.52 | clean but wall clearance L 0.04 R 0.07: same best as 5.0 for 3 cm less margin, not worth it for a best-lap event. 5.0 adopted, 5.5 dropped |
  | 40 | the 6.5 default line at the new 5.0 profile | 6.5 | 32 | 6.65 / 6.71 | clean, 32 consecutive laps, corner bias −0.012, tracking 0.062 m; the safe default is 0.15–0.20 s faster than at the old profile, margins intact |
  | 41 | record attempt: `raceline_a7.0_rec.csv`, safety margin 0.15 -> 0.10 m, zones kept | 7.0 | 9 | 6.45 / 6.53 | BEATS the 6.46 record. Min wall clearance 0.03 m at the S-exit (s=20.1), no hit, no respawn. Deepening that zone to recover clearance costs 0.03 s and gives the record back: margin and sub-6.46 are in tension, set by the delay. Safe line 6.50 stays the submission |

  Above the lateral asymptote the car holds up to about 6.5–7.0 m/s² with
  the curvature cap; the limits that bit were never grip alone but the wall
  margin where a tracking error lands. Localization cross error stayed at
  0.05–0.06 m throughout, and rose to 0.12 only while sliding at 7.0 before
  the cap.
- 2026-09-04, late: robustness. The setup that ran nine clean laps at 17.3 Hz
  hit the left wall on the straight after R1 three times at 12.8 to 14.6 Hz,
  where the round trip is 190 to 210 ms. The fix trades speed for margin only
  when the machine is slow (5.3): the line carries 0.39 m of left margin there
  and 0.43 m on the S-exit approach, the follower measures its own delay, and
  the lookahead and the speed targets follow it. The delay regression at 5 ms
  resolution showed the round trip to be three sim frames, 175 ms rather than
  150 at 17.5 Hz, so every "150 ms" constant was re-referenced to 0.175; run
  21's 0.1 s a lap over run 17 was the derate firing on a healthy machine.
- 2026-09-04, later still: the time lost against the profile was integrated
  along the track for run 22: 0.19 s a lap in speed (3 % slow in corners, 5 %
  in braking zones, nothing on straights) plus the longer path from running
  8 cm wide in corners. Looking for the braking loss found the position lead
  of 5.4; fixed in `dead_reckoning`, measured as run 23.
