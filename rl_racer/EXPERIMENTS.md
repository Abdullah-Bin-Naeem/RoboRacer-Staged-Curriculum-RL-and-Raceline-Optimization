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

Sim-loop facts that matter for RL (from the user's `LOOP_RATE.md`): the loop
is a socket round trip, 18 Hz was a Nagle/delayed-ACK deadlock, the nodelay
shim gives 77–85 Hz here, and `loop_hz_cap:=45` pins the organisers' 40–50 Hz
evaluation rate. Stages 5–6 train at that rate with decimation 2.
