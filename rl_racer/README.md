# rl_racer — SAC on AutoDRIVE RoboRacer

Single-instance SAC against the live simulator over ROS 2, race-legal sensors
only. Commands and environment setup: `../CLAUDE.md`.

| file | purpose |
|---|---|
| `rl_racer/config.py` | **all tunables**; reward weights are what you iterate on |
| `rl_racer/sensors.py` | encoder window + discontinuity guard, tire curve, car-speed observer, yaw residual (ROS-free) |
| `rl_racer/obs.py` | LiDAR crop + min-pool + observation assembly |
| `rl_racer/env.py` | Gymnasium env over ROS 2; the LiDAR scan is the clock |
| `stages/` | 5 (learn to drive), 6 (speed), 7 (smooth), 8 (no brake-pumping), 9 (grip + racing line) |
| `train_sac.py`, `run_train.sh` | trainer and its guarded launcher |
| `enjoy.py` | deterministic run: validation gate, or `--race` for deployment |
| `probe.py` | pre-flight: plumbing, tick rate, reset, collisions |
| `tests/` | mock bridge + 66-check env test, no simulator needed |

## Observation (117, all in [-1, 1])

| slots | value | source |
|---|---|---|
| 1–110 | beams over ±110°, min-pooled 8→1 (2°/beam), `/ range_max` | LiDAR |
| 111 | wheel speed `u / 26` (the throttle echo: the sim spins the wheel to 25.25·θ within a millisecond) | encoders, 3-tick window |
| 112 | yaw rate `/ 5` | IMU |
| 113–114 | previous steer, previous throttle | own command |
| 115 | observer car speed `v_est / 26` (sim's tire curve run on `u`) | sensors.py |
| 116 | longitudinal slip `S / 0.5` (peak grip at 0.15, asymptote 0.25) | sensors.py |
| 117 | yaw-rate residual `/ 8`: measured minus kinematic-bicycle for the previous command | IMU + own command |

## Action

`[steer, throttle]` in [-1, 1]. Steer ±30°. Throttle `(a+1)/2 × 1.0`: the
full simulator range, fixed, no curriculum. Peak-grip throttle is
`~1.15 × v / 25.25`; beyond it the extra is wheelspin, which the slip slot
shows and stage 6's grip term charges for.

## Reward (per step)

```
  + w_progress·min(ds, v_ref·dt)        + w_speed·v/26
  + w_grip·μ(|S|)/μ_peak                + w_lap/lap_time
  − w_center·centre_err                 − w_smooth·|Δsteer| − w_smooth2·Δsteer²
  − w_thr_smooth·|Δthrottle|            − w_slip·max(0, |S| − 0.15)
  − w_prox·prox(safe_dist)              − w_ttc·max(0, 1 − ttc/1.2 s)
  − step_penalty                        − crash / stall on termination
```

`ttc` is time-to-collision over the forward ±6° of the scan at the current
speed, with a beam at `range_max` treated as clear rather than as a wall at
10 m — the braking gradient `prox` arrives too late to give. `w_slip` is the
only term with a gradient past the tire's asymptote, where `μ(|S|)` is flat at
0.464 and `w_grip` therefore cannot pull slip down at all. Progress, the speed
term and the speed inside `ttc` use `/odom` — training only. The speed cap
lives in the reward, never in the action: progress above `v_ref` earns nothing,
and lifting it re-labels no action.

| stage | horizon | v_ref | w_ttc | w_prox | w_lap | w_grip | steer smooth | thr smooth | crash | lr | resumes |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 5 `stage5_fresh` | 13 s | 3.0 m/s | 1.0 | 0.5 | 0 | 0 | 0.3 | — | 50 | 3e-4 | — |
| 6 `stage6_push` | 13 s | off | 1.0 | 0.5 | 200 | 0.05 | 0.3 | — | 50 | 2e-4 | stage 5 + buffer |
| 7 `stage7_smooth` | **30 s** | off | 0.3 | 0.5 | 200 | 0.05 | 0.3 + **3.0·Δ²** | — | 25 | 1.5e-4 | stage 6 + buffer |
| 8 `stage8_flow` | 30 s | off | 0.15 | 0.3 | 200 | **0.15** | 0.3 + 3.0·Δ² | **0.6·\|Δ\|** | 25 | 1.5e-4 | stage 7 + buffer |
| 9 `stage9_limit` | 30 s | off | 0.15 | 0.3 @ **0.35 m** | 200 | 0.15 | 0.3 + 3.0·Δ² | 0.6·\|Δ\| | 25 | 1.5e-4 | stage 8 + buffer |

Stage 9 also sets `w_slip` **0.25** and drops `w_center` 0.15 → **0.05**.

Steering smoothness is quadratic and throttle smoothness is linear, on purpose:
steering jitter is *large reversals*, which a square targets while leaving small
corrections nearly free; throttle chatter is *frequent* transitions, which the
total variation a linear term sums measures — a square there would charge one
decisive brake application more than six chatters.

Stage 7's horizon is the load-bearing change: at 13 s the planning horizon
(267 steps) was *shorter than a lap* (19.5 s, 401 steps), so `w_lap` reached the
critic discounted to ~22 % and lap time barely entered the value function.
Reference for this track: the classical SLAM setup, **measured at 8 s/lap and
9.5 m/s peak**. (`raceline_a7.0_xyvk.csv` implies 9.94 s and caps at 8.0 m/s —
it is stale and understates what the track allows; don't use it as the target.)

Every episode end is appended to `runs/<name>/episode_ends.csv` (reason, steps,
distance, `/odom` x/y, speed at the end and over the last second, and where the
episode *started*) and averaged into `end/*` in TensorBoard: it says where the
policy dies and how fast it got there. On a crash the pose is the step *before*
the collision frame: the sim respawns the car at the last checkpoint in the
frame it counts the hit (EXPERIMENTS.md 24), and `ttc` uses the forward ±6°
because the straight is only ~1.2 m wide.

**Corner first (stage 5), `cfg.env.crash_restart`.** A counted collision already
leaves the car at the simulator's own checkpoint, pointed along the track, so
*not* pulsing reset starts the next episode at the corner that just beat the
policy — free, and it follows the frontier by itself. 75% of crash-ended
episodes resume that way (`crash_restart_prob`), at most 20 in a row
(`crash_restart_max`) so a checkpoint the policy cannot leave cannot trap the
run; the other 25% start at the line, keeping the approach and its braking in
the buffer. The crash still terminates, so the penalty target never bootstraps
across the respawn teleport. Read episode distance against `start_x`/`start_y`
in `episode_ends.csv` — a 2 m episode from the corner is not a 2 m episode from
the line. `cfg.env.spawn_drive` is the alternative: the sim has no spawn
command, so it *drives* to a fixed `(spawn_x, spawn_y)` and stops there before
handing over (~2.5 s/episode). Default off; stage 6 uses neither.

Timing: decimation 2 on the 45 Hz-capped bridge → 22.5 Hz control; episodes
245 s and gamma's 13 s horizon are derived from the measured period.

## Watch in TensorBoard

- `diag/step_ms` ≈ startup period (44), `diag/overhead_ms` < 44, `diag/ticks_per_step` ≈ 2.
- `ends/crash` falling; `rollout/ep_len_mean` flat for >30k steps means stuck.
- `slip/frac_peak` (share of steps with |S| in 0.10–0.20: is it using the tire),
  `slip/v_est_max` (real top speed, not the encoder echo).
- A printed WARNING about the yaw residual means the steering sign convention
  is inverted for this build; about the control period, lower `--gradient-steps`.
