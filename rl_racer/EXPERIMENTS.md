# RoboRacer SAC — experiment log

Running record of every training run, the measured constants, and the bugs
found. Kept because several of these were discovered the expensive way.

Last updated: 2026-09-15

---

## Compute spent

| run | episodes | steps | wall-clock | fps |
|---|---|---|---|---|
| `v3` (= stage 1) | 1,119 | 403,112 | **14.72 h** | 7.6 |
| `v_obs_1005_6` (abandoned) | 1,303 | 74,218 | 1.48 h | 13.9 |
| `stage2_sensors` | 38 | 149,126 | **2.32 h** | 17.9 |
| `stage3_speed` run 1 | 438 | 298,426 | **4.73 h** | 17.5 |
| `stage3_speed_v2` | 476 | 191,704 | **3.42 h** | 15.6 |
| `stage3_v3` | 702 | 597,930 | **11.99 h** | 13.8 |
| | | | **38.66 h total** | |

fps went 7.6 -> 17.5 when torch moved to CUDA: 4 gradient steps cost ~76 ms on
CPU (overrunning the 55 ms sim tick) vs ~40 ms on the RTX 4070 (fits inside it).
Later runs sit at 13-16 fps because a high crash rate means more resets, and
each reset costs ~0.9 s.

**stage3_v3 produced the best policy so far (7.44 s/lap). Runs 1 and v2 (8.15 h)
were lost to the replay-buffer bug (#12).**

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

## Policy performance (deterministic, `enjoy.py`)

Compare **lap time**, never lap count — episode caps are step counts, so they
mean different amounts of driving time at different control rates.

| policy | speed source | rate | lap time |
|---|---|---|---|
| v3 @370k | `/odom` | 7.7 Hz | **7.79 s** |
| v3 @370k | encoders | 7.7 Hz | 8.50 s |
| v3 @370k | encoders | 18.8 Hz | 8.00 s |
| stage2 final | encoders | 17.8 Hz | 8.23 s (1 crash in 2 eps) |
| **stage3_v3 final** | encoders | 18.5 Hz | **7.44 s — 32 laps x 3 eps, ZERO crashes** |

Cap-independent reliability (MTTC = mean steps between crashes) is the fair way
to compare runs, since every run used a different episode cap:

| run | episode cap | MTTC | crash@its own cap |
|---|---|---|---|
| stage 1 (v3, pre-collapse) | 1800 | 555 | 0.88 |
| **stage 2** | 4400 | **14,913** | **0.26** |
| stage3 run 1 | 4400 | 697 | 0.98 |
| stage3 v2 | 1200 | 494 | 0.88 |

Stage 2 was one step from triggering the ladder (0.26 vs a 0.25 gate) and is
still the best policy available. Everything after it regressed 20-30x.

**Stage 2 did not deliver the predicted improvement** (8.23 s vs the 7.79 s
target). **Stage 3 did**: 7.44 s/lap, 9.6% faster than stage 2 and better than
any earlier policy, with zero crashes across 3 full episodes.

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
| 1–90 LiDAR beams | `/lidar` | ✅ |
| 91 speed | `/left_encoder` + `/right_encoder` | ✅ |
| 92 yaw rate | `/imu` | ✅ |
| 93–94 prev actions | own commands | ✅ |
| 95 throttle cap | internal | ✅ |

`/odom` is used **only** for the reward (progress + speed) and `reset_command` /
`collision_count` only for episode management — all training-time only, none of
which exist at inference. RL is explicitly permitted by the rules.

---

## Run history

### v1, v2 — scaffolding, discarded

### v3 (= stage 1) — the good policy
370k steps, 14.7 h, CPU, 7.7 Hz. **1800 steps / 30 laps / zero crashes**,
7.79 s/lap. Destroyed at step 379,318 by the curriculum bug (#5); the 370k
checkpoint predates the raise and is the one to keep.

### v_obs_1005_6 — abandoned at 73k
Changed too much at once (±100° FOV, `w_speed` 1.0, `v_max` 25, encoder speed,
GPU/18 Hz). Plateaued at 62-step episodes and stayed flat 53k steps. Leading
suspect: gamma (#7), never isolated.

### stage2_sensors — 370k → 520k, 2.3 h
Sensor swap only. Completed, no improvement (8.23 s/lap vs 8.00 unadapted).

### stage3 — three attempts; the third SUCCEEDED

| run | steps | cap | `w_center` | buffer | alpha | outcome |
|---|---|---|---|---|---|---|
| run 1 | 520k→820k | 4400 | 0.0 | empty ❌ | spiked 0.037→**0.52** | cap never raised |
| v2 | 810k→1,000k | 1200 | 0.0 | empty ❌ | spiked 0.093→**0.26** | cap never raised |
| **v3** | **520k→1,120k** | 2250 | 0.15 | **loaded ✅** | **stable 0.032→0.085 ✅** | **cap 0.20→0.22, 7.44 s/lap** |

Runs 1 and v2 failed because of the replay-buffer bug (#12): every resume
started with an empty buffer, so the critic trained on a few hundred samples and
the policy collapsed. Fixing the filename lookup fixed both the collapse and the
alpha spike (a stable critic keeps the policy near-optimal, so SAC has no reason
to inject noise).

**v3 raised the cap at step 740,873** (0.20 → 0.22, ~4.8 → ~5.3 m/s) and its
final policy is the fastest yet: **7.44 s/lap, 32 laps per episode, zero
crashes** across 3 deterministic episodes.

#### The methodological mistake that nearly killed a working run

Throughout v3 I judged it by `ends/crash`, which sat at 0.8–1.0, and concluded
it was failing. I recommended stopping it and reverting the reward. That was
wrong:

| measure | value |
|---|---|
| training crash rate (with gSDE exploration) | **0.80** |
| deterministic crash rate (`enjoy.py`) | **0.00** |

**Training crash rate is not policy quality.** It is dominated by the
exploration noise SAC deliberately injects. The same gap was visible in stage 2
(training 0.2–0.4, deterministic a clean 4400-step / 30-lap episode) and in v3
itself (stage 1, training MTTC 555, deterministically flawless).

**Always validate with `enjoy.py` before concluding a run has failed.**
I also declared "cap never raised" at step 672k; the raise came at 740k. Judging
a curriculum before its cooldown has had room to fire is meaningless.



---

## 2026-09-15 — fresh lineage (stages 5–6), branch `rl-fresh-fov110`

Context: qualified; the competition track is in the simulator, unmapped.
`stage3_v3` drives it zero-shot but fails at sudden turns — the case where a
≥180° exit corridor sits exactly on the ±90° FOV boundary. Classical stack is
~1 s/lap faster on Porto; RL is a side quest for the final.

Decisions (each argued in the stage docstrings):
- **±110°, 110 beams** (2°/beam kept; 20° of margin past the old boundary).
- **From scratch, 118-dim.** Widening breaks every checkpoint; a transplant was
  designed but a fixed throttle scale needs no prior calibrated policy, and at
  the new loop rate a fresh run is ~6-8 h, not the CPU-era 38 h.
- **Throttle scale 0.5, fixed, no curriculum.** The cap was never a speed limit,
  it was the action's unit; every raise re-labelled every learned action, and
  the curriculum's crash gate measured exploration noise (training 0.80 vs
  deterministic 0.00). Peak-grip throttle is speed-dependent and never exceeds
  ~0.36 here (wheel speed = 25.25·θ, peak at slip 0.15); 0.5 covers the track.
- **Three race-legal slip slots** from `sensors.py`: tire-observer car speed
  `v_est` (the classical stack's `speed_source: tire`), slip `S`, yaw-rate
  residual. The encoder is the throttle echo, so without `v_est` the policy had
  no real speedometer.
- **Seconds, not steps** for episode length and curriculum cooldown (gamma was
  already derived). Tick guard 15–32 ms so the fresh stages cannot run on a
  stock 55 ms or uncapped 13 ms loop by mistake.
- Speed pressure staged: 5 = learn to drive (`w_speed` 0.2), 6 = push
  (`w_speed` 0.6, lap bonus 200, grip 0.05), resuming WITH the buffer.

Bugs found and fixed on the way (#17–19):
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

Sim-loop facts that matter for RL (from the user's `LOOP_RATE.md`): the loop
is a socket round trip, 18 Hz was a Nagle/delayed-ACK deadlock, the nodelay
shim gives 77–85 Hz here, and `loop_hz_cap:=45` pins the organisers' 40–50 Hz
evaluation rate. Stages 5–6 train at that rate with decimation 2.
