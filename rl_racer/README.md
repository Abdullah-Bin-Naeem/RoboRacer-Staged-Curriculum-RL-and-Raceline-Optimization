# rl_racer — SAC on AutoDRIVE RoboRacer

Single-instance SAC against the live simulator over ROS 2, race-legal sensors
only. Commands and environment setup: `../CLAUDE.md`.

| file | purpose |
|---|---|
| `rl_racer/config.py` | **all tunables**; reward weights are what you iterate on |
| `rl_racer/sensors.py` | encoder window + discontinuity guard, tire curve, car-speed observer, yaw residual (ROS-free) |
| `rl_racer/obs.py` | LiDAR crop + min-pool + observation assembly |
| `rl_racer/env.py` | Gymnasium env over ROS 2; the LiDAR scan is the clock |
| `stages/` | stage 5 (learn to drive), stage 6 (speed pressure) |
| `train_sac.py`, `run_train.sh` | trainer and its guarded launcher |
| `enjoy.py` | deterministic run: validation gate, or `--race` for deployment |
| `probe.py` | pre-flight: plumbing, tick rate, reset, collisions |
| `tests/` | mock bridge + 42-check env test, no simulator needed |

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

`5·progress(m) + w_speed·v/26 + w_grip·μ(|S|)/μ_peak + w_lap/lap_time − 0.15·centre_err
− 0.3·|Δsteer| − 0.5·prox − 0.02`; crash −15 (episode ends), stall −5.
Progress and the speed term use `/odom` — training only.

| stage | w_speed | w_lap | w_grip | timesteps | lr | resumes |
|---|---|---|---|---|---|---|
| 5 `stage5_fresh` | 0.2 | 0 | 0 | 250k | 3e-4 | — |
| 6 `stage6_push` | 0.6 | 200 | 0.05 | 300k | 2e-4 | stage 5 + its replay buffer |

Timing: decimation 2 on the 45 Hz-capped bridge → 22.5 Hz control; episodes
245 s and gamma's 13 s horizon are derived from the measured period.

## Watch in TensorBoard

- `diag/step_ms` ≈ startup period (44), `diag/overhead_ms` < 44, `diag/ticks_per_step` ≈ 2.
- `ends/crash` falling; `rollout/ep_len_mean` flat for >30k steps means stuck.
- `slip/frac_peak` (share of steps with |S| in 0.10–0.20: is it using the tire),
  `slip/v_est_max` (real top speed, not the encoder echo).
- A printed WARNING about the yaw residual means the steering sign convention
  is inverted for this build; about the control period, lower `--gradient-steps`.
