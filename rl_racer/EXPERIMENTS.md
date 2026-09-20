# RoboRacer SAC — experiment log

Running record of every training run, the measured constants, and the bugs
found. Kept because several of these were discovered the expensive way.

Last updated: 2026-09-15

---

## Measured constants for this simulator

Verified empirically, not assumed. Several contradict the documentation.

| quantity | value | how |
|---|---|---|
| LiDAR | **1081** beams, 270°, 0.25°/beam | live scan header |
| Sim tick rate | **~18 Hz** | measured; the bridge *advertises* 40 |
| Control period | **~55.6 ms** | measured per-run at startup |
| Encoder units | **radians** (wheel angle) | 4 trials — the Technical Guide says "ticks, 1920/rev" and is **wrong** by 300× |
| Wheel radius | **0.0581 m** | path-integration 0.05815, steady-state speed match 0.05813 (guide says 0.0590) |
| Speed vs throttle | **≈ 24 × throttle** | 3 independent points; docs give 22.88 m/s top speed |
| Track lap | **~33 m** | distance ÷ laps |
| `twist.linear` frame | **BODY**, not world | velocity sits 2.9° off body x-axis, 93.9° off world yaw |
| IMU yaw axis | `angular_velocity.z`, rad/s | corr 0.83 with truth, slope 0.974 |
| `/imu` vs `/odom` angular | **identical** | bridge feeds both from one variable |

---

## Bugs found (chronological)

1. **Sim tick 18 Hz, not 40.** The bridge reports `lidar_scan_rate=40`; the real
   Bridge round-trip is 18 Hz. `decimation=2` would have given 9 Hz control.
2. **Scan is 1081 beams, not 1080** (inclusive endpoints).
3. **`reset_command` is level-triggered** — re-emitted every tick, so it must be
   pulsed True→False or the sim resets forever.
4. **Speed sign bug.** `twist.linear` is BODY frame; the code projected it onto
   the WORLD heading. Mislabelled **61%** of steps as reverse. Killed by moving
   to encoders (whose deltas are naturally signed).
5. **Curriculum fired on competence, not demand.** v3 raised the cap +0.10 (+50%
   speed) at step 379,318 because the policy was driving well — but it was using
   only 71% of the cap and hit full throttle 1.3% of the time. Result: 28 laps →
   0, episodes 1060 → 24. Fixed by adding a `throttle_demand >= 55%` gate and
   dropping the step to +0.02.
6. **Resume with no replay buffer collapses training.** `save_replay_buffer` was
   False, so a resume began gradient updates against a near-empty buffer:
   episodes fell 850 → 24 in ~7k steps. Fixed with `--warmup` (collect with the
   loaded policy before training) and `save_replay_buffer=True`.
7. **`gamma` is a step-count horizon, not a time horizon.** `0.99` = 100 steps =
   13 s at v3's 7.7 Hz but only **5.6 s** at 18 Hz. Moving to the GPU silently
   halved the planning horizon. Fixed: gamma is now DERIVED from the measured
   control period (`1 - dt/horizon_seconds`, horizon 13 s), so it reproduces
   0.99001 at 7.7 Hz and 0.9957 at 18 Hz.
8. **`--gradient-steps` silently overrode the checkpoint** on resume (default 2
   vs v3's 4). Now defaults to None = inherit, and every change prints.
9. **Reward scales guessed wrong twice.** `w_speed=0.2` was 0.6% of total reward
   (invisible); `w_lap=50` was 2.4%, not the ~10% claimed. Lesson: measure the
   term's magnitude before choosing its weight.
10. **`max_episode_steps` has the same steps-vs-time flaw as gamma.** 1800 steps
    is 234 s at 7.7 Hz but 96 s at 18 Hz. Set to 4400 to restore v3's episode
    length on this machine. TODO: derive it from `control_period`.
11. **Crash-rate gate is cap-dependent.** `crash_rate_max=0.25` was chosen when
    episodes were 1800 steps; at 4400 the same policy reports a far higher crash
    rate (more chances to crash before the cap). Stage 3 run 1 therefore **never
    raised the cap** — best crash rate seen was 0.60 vs a 0.25 gate.
12. **Replay buffer filename mismatch — buffers never loaded.** The code looked
    for `<checkpoint>_replay_buffer.pkl`; SB3 writes
    `<prefix>_replay_buffer_<N>_steps.pkl`. So EVERY resume (stage 2, stage 3
    run 1, v2) silently started with an empty buffer and trained a critic on a
    few hundred samples. 30+ buffer files existed and none was ever read.
13. **Entropy coefficient re-inflates on resume.** SAC tunes alpha toward a
    fixed entropy target; a converged policy sits BELOW that target, so alpha
    climbs and re-randomises a good policy. Observed 0.037 -> 0.524 (14x).
    Fixed indirectly by loading the buffer (a stable critic keeps the policy
    near-optimal so alpha has no reason to climb). `--ent-coef` can freeze it,
    but freezing risks blocking the adaptation the curriculum needs.
14. **Crash-gate headroom, not just pass rate.** Every cap raise degrades the
    policy temporarily, so the gate must tolerate that. Headroom = how far MTTC
    may fall before the gate shuts: cap 1200/gate 0.25 -> 3.6x, cap 2250/0.35
    -> 2.9x, cap 4400/0.35 -> only 1.5x (stalls after 1-3 raises).
15. **The gate is a 20-episode SAMPLE, so it is softer than it looks.** At a
    true crash rate of 0.25 a window passes 62% of the time; at 0.35, 25%. The
    practical bar is "true rate <~0.35", not a strict 0.25.
16. **ROS 2 DDS was talking to the LAN.** `ROS_LOCALHOST_ONLY=0` meant discovery
    found a node on another host (192.168.1.16) and retried forever, dropping
    commands. Fix: `export ROS_LOCALHOST_ONLY=1` in **every** terminal.

---

## Competition legality

Per the [Technical Guide](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2026),
these are **restricted at race time**: `ips`, `odom`, `reset_command`,
`collision_count`, `lap_count`, `lap_time`, `tf`.

| observation slot | source | legal |
|---|---|---|
| 1–110 LiDAR beams | `/lidar` | ✅ |
| 111 wheel speed `u` | `/left_encoder` + `/right_encoder` | ✅ |
| 112 yaw rate | `/imu` | ✅ |
| 113–114 previous action | own commands | ✅ |
| 115 `v_est`, 116 slip `S`, 117 yaw residual | encoders + IMU + own command + the sim's published tire model | ✅ |

`/odom` is used **only** for the reward (progress + speed) and `reset_command` /
`collision_count` only for episode management — all training-time only, none of
which exist at inference. RL is explicitly permitted by the rules.

---

## 2026-09-15 — fresh lineage (stages 5–6), branch `rl-fresh-fov110`

Context: qualified; the competition track is in the simulator, unmapped.
`stage3_v3` drives it zero-shot but fails at sudden turns — the case where a
≥180° exit corridor sits exactly on the ±90° FOV boundary. Classical stack is
~1 s/lap faster on Porto; RL is a side quest for the final.

Decisions (each argued in the stage docstrings):
- **±110°, 110 beams** (2°/beam kept; 20° of margin past the old boundary).
- **From scratch, 117-dim.** Widening breaks every checkpoint; a transplant was
  designed but a fixed throttle scale needs no prior calibrated policy, and at
  the new loop rate a fresh run is ~4 h, not the CPU-era 38 h. The legacy
  lineage (stages 1–4, 95-dim, its runs) was removed from this branch.
- **Throttle scale 1.0, fixed, no curriculum.** The cap was never a speed limit,
  it was the action's unit; every raise re-labelled every learned action, and
  the curriculum's crash gate measured exploration noise (training 0.80 vs
  deterministic 0.00). Peak-grip throttle is speed-dependent, ~1.15·v/25.25
  (0.36 at 8 m/s, 0.9 at 20); the competition track has a straight long enough
  to use the top of the range, and the slip slot shows where wheelspin starts.
  The constant throttle-cap slot of the old observation is gone with the cap.
- **Three race-legal slip slots** from `sensors.py`: tire-observer car speed
  `v_est` (the classical stack's `speed_source: tire`), slip `S`, yaw-rate
  residual. The encoder is the throttle echo, so without `v_est` the policy had
  no real speedometer.
- **Seconds, not steps** for the episode length (gamma was already derived). Tick guard 15–32 ms so the fresh stages cannot run on a
  stock 55 ms or uncapped 13 ms loop by mistake.
- Speed pressure staged: 5 = learn to drive (`w_speed` 0.2), 6 = push
  (`w_speed` 0.6, lap bonus 200, grip 0.05), resuming WITH the buffer.

Bugs found and fixed on the way (#17–22):
17. **Encoder spike on every reset.** The simulator's `ResetManager` restores
    `TotalRevolutions` to the spawn value on `reset_command`; the env never
    cleared its encoder state, so the first step of every episode computed a
    rate from a pre-reset angle (slot 91 = −1.0 at the launch decision).
    Fixed: sensor state cleared after the reset settles + a 300 rad
    discontinuity guard (as `dead_reckoning.py`).
18. **Single-tick encoder rate carried the loop's stamp jitter** (~15% at 18
    Hz; 12.8 median / 25 ms max at the shimmed loop). 3-tick window: error
    0.87 → 0.21 m/s in the unit test.
19. **All lap times before this date were derived (`steps × dt / laps`) with a
    dt that excluded policy inference** — ~5% optimistic. `enjoy.py` now prints
    the simulator's own lap timer. The README's 7.44 s is the derived figure.
20. **`step()` counted the decimation window from the send, so the caller's
    overhead extended it in whole ticks.** Period = tick × (decimation +
    ⌊overhead / tick⌋): at 55 ms ticks with decimation 1 that was invisible,
    at 22 ms ticks 3 gradient steps (~30 ms) make it 3 ticks = 66 ms, not the
    44 ms gamma and the episode length were derived from. Now phase-locked to
    the last observation: measured 44.0 ms / 2.00 ticks per step with 30 ms of
    injected overhead (mock bridge). `diag/step_ms` shows it live.
21. **`env.close()` could not stop its spin thread** (`Executor.spin()`
    busy-loops after `shutdown()`), so Ctrl-C printed a traceback and once
    segfaulted at exit. Own spin loop with a stop flag, joined in `close()`.
    `enjoy.py --race` added for deployment: no reset pulse (would teleport the
    car mid-race), no termination on collision or stall, no step cap.
22. **Command latency depended on the caller's overhead.** The bridge forwards
    a command in its reply to the next telemetry: sent 2 ms after the
    observation (deployment) it rides tick 1 and shows in the same step's
    observation; sent 30 ms after it (training, 3 gradient steps) it rides
    tick 2 and shows in the next step's. Found because the yaw-residual
    warning fired in a resume smoke run but not in the random-action one.
    Now the send waits for the tick after the observation (decimation ≥ 2),
    so it is "next step" in both, and the residual is taken against the
    previous command, the one that produced the measured yaw rate.

23. **Stage 5 run 1 (2026-09-15, 743k steps, 13.2 h, 8606 episodes): 100% crash,
    at the same spot.** `ends/crash` 1.000 for the whole run, zero laps, zero
    timeouts, `race/dist_per_episode_m` pinned at 19.2 ± 0.3 m, mean episode
    87 steps (4.9 s) against a 5596-step cap. The longest episode ever (237
    steps) was in the random phase. Reward decomposition closes to the logged
    74.5: progress **+95.9**, speed +2.7, centre −2.0, smooth −4.7, prox −0.6,
    step −1.7, crash **−15**. Crashing was a 16% tax on a profitable sprint,
    so sprint-crash-repeat was the optimum, and `action/steer_abs_mean` fell
    0.48 → 0.21: steering bought nothing when the episode ended at the wall
    regardless, and entropy collapsed (ent_coef 0.11 → 0.03). Perception was
    not the problem: `v_est` within 3% of odom, `slip/frac_peak` 0.02.
    Three causes, three changes:
    - `w_progress` is speed pressure (progress/step = w·v·dt = 1.1/step at
      4 m/s); the docstring's "low speed pressure" was wrong about its own
      reward. Now `min(ds, v_ref·dt)`, v_ref 3 m/s in stage 5, off in stage 6.
      In the reward, never the action (see the cap collapses above).
    - No braking gradient existed: `prox` is zero outside 0.5 m = 1–2 control
      steps at 4–8 m/s, and `front_min` was computed but unused. Added a
      time-to-collision term over the forward ±10°, per beam r/(v cos θ),
      `w_ttc·max(0, 1 − ttc/1.2 s)`. At 3 m/s it wakes 3.6 m before a wall
      ahead and is silent at 1.5 m/s with 1.8 m to go (the classical raceline
      takes the hairpins at ~1.5 m/s). A wall alongside is not "ahead".
    - The crash-penalty docstring assumed the deterrent is the forfeited
      future; a fresh critic has never seen a future. 15 → 50, above the
      reward-to-go at the braking point (~20) and below where creeping wins.
    Also: the sim held ~37 Hz (26.7 ms tick), not the 45 Hz cap; and with
    `gradient_steps=3` on the GTX 1080 Ti `diag/overhead_ms` reached 50 of a
    55 ms window late in the run (ticks_per_step 2.39). Stage 5 now defaults
    to 2. `episode_ends.csv` logs the /odom pose and approach speed at every
    episode end so the crash geometry is measured, not inferred.

24. **The simulator respawns the car on a collision, in the same frame that
    increments `collision_count`.** Recorded live (30 s, 14 collisions): the
    frame with the increment already reports the car at x = 0.800 and
    y ∈ {3.66, 0.66, −2.34, −5.34} (checkpoints 3 m apart along the opening
    straight) at ~0.07 m/s, while 0.15 s earlier it was mid-track at 3–6 m/s.
    The env's reset pulse then moves it to the spawn (0.800, 3.160) and the
    sim zeroes the counter. So on a crash step the odom pose is the respawn,
    not the crash: the progress term was paying a clamped ±1 m (±5 reward) for
    the teleport, and the first `episode_ends.csv` rows logged checkpoints
    (105 of 110 at x = 0.800 exactly). Now the crash step pays no progress
    and logs the previous step's pose and speed. At the race this respawn is
    what a collision costs (plus the time penalty) — `--race` mode keeps
    driving through it by design. Same recording: pre-crash car-centre x
    spanned 0.27–1.23 about the 0.80 centreline, so the straight is ~1.2 m
    wide; the TTC sector went ±10° → ±6° so a straight road does not charge
    below ~4.4 m/s.

25. **The corner, measured — and a reverse curriculum for free.** Recording
    odom pose + heading through 5 collisions of the stage-5 v2 policy: the car
    runs the opening straight on a heading of −90° to −104° and dies every
    time at **y ≈ −15.6, x ≈ 0.6**, arriving at **5.1–6.2 m/s**. The spawn is
    (0.800, 3.160), so that is **18.8 m** down the straight — which confirms
    the estimate the classical raceline gave (straight ends 19–20 m, item 23),
    the one that could not be verified before because the CSV's origin is not
    the spawn. The checkpoint there carries **yaw −35°** against the car's
    −103° arrival: the track turns ~68° left and the policy simply does not
    turn with it.
    The simulator has **no spawn API**: the bridge subscribes to exactly
    `throttle_command`, `steering_command`, `reset_command`, the socket.io
    payload carries exactly `V1 Throttle`, `V1 Steering`, `V1 Reset`, the Unity
    build has no `StreamingAssets` or track config, and no `V1 Pose` /
    `Position` / `Spawn` / `Teleport` string exists in `GameAssembly.so`.
    `V1 Reset` is a bare boolean meaning "the start line" and takes no
    argument, so a chosen start pose cannot be set — it has to be **driven
    to**. `cfg.env.spawn_drive` does that: after the reset pulse the env
    steers dead straight to within `spawn_radius` of (`spawn_x`, `spawn_y`)
    on a decelerating approach (`v_want = sqrt(2 a d)`, a = 4 m/s² from the
    tire asymptote — braking only on arrival overshoots by v²/2a ≈ 1 m from
    3 m/s), brakes to a standstill, and only then hands over. Measured against
    the mock: **0.11 m** from the point, stopped, ~3 s. Stage 5 uses
    (0.80, −13.31), 2.3 m before the corner. The car is stationary at handover,
    so the first observation is exactly what a normal reset produces and the
    observer still starts at 0 — no privileged speed leaks in. Those ticks are
    not policy steps and not recorded transitions; at deployment (`race_mode`)
    reset does nothing, so the whole path is absent.
    A second, free mechanism also exists and is kept behind
    `cfg.env.crash_restart` (default off, off in stage 5): a counted collision
    already leaves the car at the last checkpoint (item 24), checkpoints are
    ~3 m apart and carry the track's heading, so **not pulsing reset** starts
    the next episode exactly where the policy failed, pointed along the line —
    self-targeting and free, but the start pose scatters between checkpoints
    (y = −14.34 at yaw −90° and y = −15.57 at yaw −35° both border this
    corner), which is why the fixed drive-in is preferred. It runs for
    `crash_restart_prob` (0.75) of crash-ended episodes, capped at
    `crash_restart_max` (20) in a row so a checkpoint the policy cannot leave
    cannot trap the run. The crash still TERMINATES, deliberately:
    bootstrapping across the respawn teleport would let `V(checkpoint)` offset
    the crash penalty and could make crashing look positive. Only the next
    episode's start state moves. A scrape or a truncation never resumes (the
    sim respawned nothing). `episode_ends.csv` gains `start_x`/`start_y`/
    `resumed`: without them an episode's distance cannot be read, since 2 m
    from the corner is not the same failure as 2 m from the line.

26. **Alpha runaway killed a converging run — and `--ent-coef` then made its
    checkpoints unloadable.** `stage5_v4` was healthy to ~90k steps
    (`ep_len_mean` 107 at 70k, `critic_loss` 5.5, `ent_coef` 0.050) and then
    spiralled: `ent_coef` 0.039 → **0.447** (11x), `critic_loss` 2 → **1.9e6**
    (peak at step 149,562), `ep_len_mean` 107 → 22. Mechanism: SAC tunes alpha
    toward a fixed target entropy (`-dim(A)` = −2), but the policy converged to
    sigma ≈ 0.055, whose 2-D differential entropy is ≈ −3.0, i.e. *below* the
    target — so alpha climbs to force exploration back up. At 0.45 the entropy
    bonus is ~±0.9/step, the size of the entire capped progress reward
    (5.0 × 3.0 × 0.0588 = 0.88), so the policy was paid as much to be random as
    to drive: it re-randomised, the data worsened, the critic diverged.
    `action/throttle_sat` 0.001 → 0.314 is the fingerprint. Item 13 is the same
    failure on *resume*; it happens mid-run too. Fixed by resuming the 50k
    checkpoint with `--ent-coef 0.045` (the value alpha held while learning was
    healthy) and `--gradient-steps 1`: `ends/timeout` reached 0.97,
    `ep_len_mean` 4242 of a 4342 cap, 10 laps and 439.9 m per episode,
    `overhead_ms` 29–49 → 14. The trainer now WARNS as soon as alpha exceeds 3x
    its running low, naming the value to freeze at.
    The freeze had a latent bug that only shows when the checkpoint is re-read:
    it cleared `ent_coef_optimizer` and set `ent_coef_tensor` but never
    `model.ent_coef`, so SB3 wrote no `ent_coef_optimizer.pth` while the saved
    metadata still said `"auto"`. Every load of such a file then rebuilt a
    model expecting that optimizer, failed `set_parameters`, and SB3's own
    fallback raised `KeyError: 'policy.optimizer'` — so the error that escapes
    never mentions the entropy coefficient, and catching `ValueError` on the
    message does not work. The checkpoints are complete (the frozen value is in
    `pytorch_variables`' `ent_coef_tensor`); `rl_racer/checkpoint.py` detects
    the state from the zip itself and reloads with `custom_objects`, and
    `model.ent_coef` is now set so new checkpoints need none of it. `enjoy.py`
    also loads the policy BEFORE building the env, so a bad checkpoint no
    longer leaves a connected ROS node behind and abort at exit.

27. **Stage 6 worked, then plateaued on speed; the discount horizon was
    shorter than a lap.** Resuming stage 5 with `--ent-coef 0.045
    --gradient-steps 1`: reward 1413 -> 2420, laps/episode 9.6 -> 12.4 (peak
    15), lap 22.2 -> 19.5 s (best episode 16.1 s), `v_est_mean` 2.26 -> 2.80,
    survival held (80-100% timeout, 11 crashes in 123 episodes), alpha pinned,
    `critic_loss` 1.79, `overhead_ms` 13.8 of a 51 ms window. But lap time by
    slice went 19.5 -> **17.6** -> 19.6 -> 18.5 -> 19.7 -> 19.5: it peaked at
    ~1.94M steps and was flat for the next ~400k. Four measured causes, all
    addressed in stage 7:
    - **`gamma`'s horizon (13 s, 267 steps) was shorter than a lap (19.5 s,
      401 steps).** The critic could not see one lap ahead, so `w_lap`'s
      200/lap_time arrived discounted to ~22% of face value -- and it is the
      only term that pays for being FAST rather than FAR. `horizon_seconds`
      13 -> 30 (gamma 0.99626 -> 0.99838, 617 steps, 1.5 laps).
    - **`w_ttc` 1.0 became the speed cap.** -362/episode against +217 of speed
      reward, the largest single cost. On a 44 m track the car is always within
      ~2 m of a wall through the corners, so the penalty is unavoidable there
      and the only way to pay less is to go slower. 1.0 -> 0.3.
    - **`ttc_forward` charged for SATURATED beams.** `ttc = front_min/v` with
      `front_min` capped at `range_max` = 10 m, so empty road registered as a
      1.0 s time-to-collision at 10 m/s: above `range_max/ttc_ref` = 8.3 m/s
      the term taxed a completely clear straight, which is exactly where the
      raceline's 8 m/s peak belongs. A beam at `range_max` now reads as
      infinite ttc. Latent, not yet binding (0.000/step at 2.8, 5.0 and 8.0).
    - **The risk math said "do not push".** A crash costs 50 plus the forfeited
      rest of the episode, ~178 in discounted value at a 13 s horizon; going
      8 m/s instead of 4 on the 18.8 m straight gains ~15 inside that horizon.
      The policy was being rational, not timid. The 30 s horizon raises the
      implicit forfeit to ~339, so `crash_penalty` 50 -> 25 without reopening
      the sprint-crash equilibrium.
    Also measured and not yet fixed: the policy **wheelspins instead of
    accelerating**. `throttle_mean` 0.21 commands a wheel speed of 5.3 m/s at
    an actual 2.8, i.e. slip 0.63 -- far past the tire's 0.15 peak, so it gets
    mu 0.464 instead of 0.72. Peak-grip throttle at 2.8 m/s is 0.135 and at
    10 m/s is 0.455. `slip/frac_peak` 0.067: it uses the tire properly 7% of
    the time. `w_grip` 0.05 is evidently too weak to fix that.
    Reference for this track, from the classical raceline: 44.0 m, **9.94 s per
    lap, 4.42 m/s average, 8.0 m/s peak** (it never exceeds 8, so a "10 m/s"
    target is above what the reference line itself uses). At 19.5 s the RL
    policy is 2.0x off, not 4x.
28. **Steering jitter: |d_steer| averaged 0.164/step against a mean lock of
    0.273.** That is 4.9 deg every 48.6 ms, ~101 deg/s, i.e. the policy
    reversed most of its steering every single step; `slip/yaw_res_rms` rose
    0.485 -> 0.779 from stage 5 to 6 with it. The linear `w_smooth` 0.3 charged
    that only 9% of progress. Raising the linear term suppresses jitter and
    quick hands equally, which is wrong when the same stage wants more speed,
    so stage 7 adds a QUADRATIC companion `w_smooth2 * d_steer^2` = 3.0:
    d=0.05 costs 0.023/step, d=0.164 costs 0.130, d=0.5 costs 0.900 -- hard on
    a reversal, near-free when smooth. Both terms use only `prev_steer`, which
    is observation slot 113, so the reward stays Markovian. A true jerk term
    (second difference) would need `prev_prev_steer` in the observation and
    would unload every checkpoint, so it is deliberately not done.

29. **The car was slow because the policy PUMPS THE BRAKE, not because of any
    reward weight and not because of exploration.** Stage 7 delivered its
    brief (laps/episode 11.4 -> 14.9, lap 21.0 -> 16.4 s, `yaw_res_rms` 0.779
    -> 0.607) and its steering fix worked -- replaying the DETERMINISTIC policy
    gave |d_steer| 0.094 -> 0.055 per step, **-41%** -- but average speed
    stayed at 2.9 m/s against a classical setup measured at **8 s/lap and
    9.5 m/s** on this track. (The `raceline_a7.0_xyvk.csv` profile used as the
    reference in items 23 and 27 caps at 8.0 m/s and implies 9.94 s/lap; it is
    evidently stale and understates what the track allows.)
    Two theories were tested against the live sim and both were WRONG:
    - *"It never opens the throttle; the exploration window is 6.7 sigma short
      of the 0.376 that 9.5 m/s needs."* Replaying the policy with its throttle
      multiplied by 1.5, 2.0 and 2.5 moved `v_mean` only 2.98 -> 3.18. Scaling
      a zero is still a zero. It already commands throttle >0.3 on 27% of steps
      and >0.9 on 2.9%.
    - *"`w_ttc` is the speed cap."* True in stage 6 (-402/episode) but not
      after the cut to 0.3: episode cost is 0.026/step, 4% of progress.
    What the deterministic policy actually does, measured at the actions over
    700 steps: **|d_throttle| 0.165/step, 25% of steps at FULL brake, in 83
    bursts averaging 2.1 steps (104 ms)** -- about 6x more often than a lap's
    corners require. Throttle histogram is bimodal, median 0.130 (coasting)
    against p90 0.613. Throttle 0 is brake LOCK in this sim, so each burst
    throws away ~0.47 m/s at 4.55 m/s^2. Sustained, throttle 0.40 is 9.6 m/s
    terminal, so the mechanism was never the limit.
    Root cause: **the reward had no throttle-smoothness term at all** --
    `w_smooth` penalises steering only, so chatter was free. Stage 8 adds
    `w_thr_smooth` 0.6 on `|throttle_t - throttle_{t-1}|`, **linear** where the
    steering term is quadratic: steering jitter is large reversals (a square
    targets those and leaves small corrections free), throttle chatter is
    frequent transitions (linear total variation measures exactly that, while a
    square would charge one decisive 0.6 brake application 0.54 against six 0.1
    chatters at 0.09 -- i.e. it would REWARD the chatter). `prev_throttle` is
    observation slot 114, so the term is Markovian and no checkpoint breaks.
    Stage 8 also raises `w_grip` 0.05 -> 0.15 (sustained peak-slip acceleration
    is the opposite of pump-and-slam, and `slip/frac_peak` 0.064 shows 0.05 was
    too weak to matter) and trims `w_ttc` 0.3 -> 0.15 and `w_prox` 0.5 -> 0.3,
    which spike locally near a wall (ttc reaches ~0.24/step in a corner, 39% of
    progress) on a policy now known to over-brake.
    **Lesson for the next plateau: measure the ACTIONS of the deterministic
    policy before touching a weight.** Three stages of reward tuning were spent
    on a term that was not in the reward at all, and the two leading hypotheses
    were both falsified in ten minutes by replaying the checkpoint.

30. **The car never grips: mean |S| 0.924, 75.6% of the lap past the tire's
    asymptote — and `w_grip` has ZERO gradient there, which is why tripling it
    did nothing.** Stage 8 delivered its brief (|d_throttle| 0.165 -> 0.126,
    full brake 24.7% -> 21.6%, brake bursts 83 -> 60 per 700 steps, lap 16.4 ->
    15.6 s, reward +22%) but `slip/frac_peak` did not move at all: 0.064 before
    and after `w_grip` 0.05 -> 0.15, while the term paid out 477/episode.
    The reason is in the tire curve: `mu(|S|)` is **flat at the asymptote 0.464
    for every |S| >= 0.25**, so `mu/mu_peak` is a constant 0.644 across the
    entire region the policy occupies. Tripling a weight multiplies a zero
    gradient. Measured on the deterministic policy over 1400 steps: mean |S|
    0.924, 75.6% of steps past 0.25, 6.3% in the peak band -- it spins up under
    throttle and locks under brake, spending three quarters of the lap at 64%
    of the grip it has. That is the speed ceiling, and it limits cornering and
    braking as much as acceleration.
    Stage 9 adds `w_slip` 0.25 on `max(0, |S| - tire_s_peak)`: linear, so it
    has a constant non-zero gradient everywhere above the peak, and **symmetric**,
    because peak mu is at |S| = 0.15 for braking too -- it asks for threshold
    braking and threshold acceleration, i.e. the limit of the car. At the
    measured 0.924 it costs 0.194/step (29% of progress) and at the peak
    exactly 0.
    Note `obs.slip_max` is 0.5, so slot 116 is PINNED at 1.0 for 76% of steps --
    the policy cannot see how much wheelspin it has. Raising it would un-blind
    the slot but would also invalidate every observation already in the replay
    buffer (SB3 stores observations, not just rewards), and the policy can
    derive S from slots 111 (u) and 115 (v_est) regardless, so it was left
    alone. Worth revisiting only on a fresh lineage.
    Also in stage 9, from the same measurement: the policy **will not leave the
    centreline** -- mean lateral offset 0.18 m of a 0.77 m half-width, 23% of
    the room available, in a corridor averaging 1.55 m. A racing line is
    out-in-out and `w_center` pays for the opposite, while `prox` at
    `safe_dist` 0.5 m starts charging before an apex can be reached at all
    (min_range mean 0.55 m -- it rides that threshold constantly). So
    `w_center` 0.15 -> 0.05 and `safe_dist` 0.5 -> 0.35. NOT `w_center` 0:
    item 5's lineage saw the crash rate rise 20x at zero, and although that
    lineage had no ttc term and a smaller crash penalty, zero is not a step to
    take blind.

Sim-loop facts that matter for RL (from the user's `LOOP_RATE.md`): the loop
is a socket round trip, 18 Hz was a Nagle/delayed-ACK deadlock, the nodelay
shim gives 77–85 Hz here, and `loop_hz_cap:=45` pins the organisers' 40–50 Hz
evaluation rate. Stages 5–6 train at that rate with decimation 2.
