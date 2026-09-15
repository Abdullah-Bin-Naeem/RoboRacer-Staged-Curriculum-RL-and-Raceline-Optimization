# CLAUDE.md

Guidance for Claude Code in this repository (branch `rl-fresh-fov110`).

## What this is

A SAC policy for the AutoDRIVE RoboRacer simulator (Sim Racing League 2026)
that drives straight off LiDAR: no map, no localization. This branch carries
only the RL stack and the bridge it needs. The classical stack (SLAM, raceline,
pure pursuit, the run logs) lives on `multi-track` / `main`; this branch
**deletes** it, so never merge this branch into those. Cherry-pick `rl_racer/`
changes instead.

- `rl_racer/` — env, config, sensors, stages, trainer, deployment runner, test.
- `devkit_ws/src/` — `autodrive_devkit` (third-party bridge, BSD, **unmodified**),
  `racer_bringup` (`bridge.launch.py`: the bridge with the TCP shim and the TF
  remap), `racer_common` (workspace paths, `cyclonedds.xml`).
- `tools/` — `nodelay.c` (the shim), `sim_rate_probe.py`, `topic_rates.py`.

`rl_racer/EXPERIMENTS.md` holds the measured constants and every bug found the
expensive way. Read it before changing a number.

## Environment

Two Python environments; mixing them is the most common failure:

| Context | Python | Why |
|---|---|---|
| bridge, `probe.py` | **system** python3.10 | `rclpy`/`cv_bridge` are C extensions built against it |
| training, `enjoy.py`, the test | **`.venv-rl`** | torch + CUDA + stable-baselines3 live only there |

Source ROS **first**, then the venv: ROS puts `rclpy` on `PYTHONPATH`, the venv
must win on `PATH`. Conda does not work. `source ros_env.sh` does the ROS half
(`ROS_LOCALHOST_ONLY=1`, humble, the workspace, `CYCLONEDDS_URI`) and warns if
the workspace is unbuilt.

## Build

```bash
cd devkit_ws && PYTHONNOUSERSITE=1 colcon build        # never from the venv
gcc -shared -fPIC -O2 -o tools/libnodelay.so tools/nodelay.c -ldl   # the shim
```

`PYTHONNOUSERSITE=1` is required: a newer setuptools in `~/.local` breaks any
`ament_python` build. The installed launch files are copies; after editing or
pulling `racer_bringup`, rebuild it.

## Running

The simulator is not in this repo (Unity build from the AutoDRIVE releases).
Start it first, select the track in its menu, then:

```bash
# terminal 1 — bridge (system python), pinned to the evaluation loop rate
source ros_env.sh
ros2 launch racer_bringup bridge.launch.py tcp_nodelay:=true loop_hz_cap:=45

# terminal 2 — RL (ROS first, then the venv)
source ros_env.sh && source .venv-rl/bin/activate && cd rl_racer
python probe.py                     # ALWAYS first: plumbing, tick rate, reset, collisions
./run_train.sh --stage 5            # learn to drive, from scratch (~4 h)
python enjoy.py runs/stage5_fresh/final.zip --episodes 3          # validation gate
./run_train.sh --stage 6 --resume runs/stage5_fresh/final.zip     # speed pressure, loads the buffer
python enjoy.py runs/stage6_push/final.zip --race                 # deployment
python tests/test_env_mock.py       # env against a mock bridge; no sim; run after any env.py change
tensorboard --logdir runs/
```

`run_train.sh` refuses to start unless the sim, the bridge on port 4567 and
CUDA are up, then `exec`s `train_sac.py`, so every trainer flag passes through.
Stages 5–6 refuse a sim tick outside 15–32 ms (stock bridge: 55 ms; uncapped
shim: 13 ms). `--race` sends no reset pulse, never stops on a collision or
stall, has no step cap and ends on Ctrl-C.

Validation gate: zero crashes (`end=timeout`) across the episodes and the lap
time from the simulator's own timer (printed) no worse than the previous stage.

## RL design (`rl_racer/`)

- `rl_racer/config.py` — **all tunables** as dataclasses; every constant carries
  its measurement.
- `rl_racer/sensors.py` — ROS-free, unit-tested: encoder window with a
  discontinuity guard, the sim's tire curve, the car-speed observer, the
  yaw-rate residual.
- `rl_racer/obs.py` — LiDAR crop + min-pool + observation assembly.
- `rl_racer/env.py` — Gymnasium env over ROS 2. The sim is free-running, so
  `step()` blocks on the LiDAR topic: **the scan is the master clock** (the
  bridge publishes every topic together inside one socket.io handler).
- `stages/stage5_fresh.py`, `stage6_push.py` — `NAME` / `RESUME_FROM` /
  `DEFAULTS` / `apply(cfg)`; stage 6 composes stage 5 and changes the reward.
- `tests/mock_bridge.py` — lockstep stand-in for the bridge with the sim's tire
  model; `tests/test_env_mock.py` runs the env against it (42 checks).

Observation, 117 floats in [-1, 1]: 110 min-pooled beams over ±110° (raw scan
indices 100:980, 8 raw beams → 1), wheel speed `u` (the throttle echo),
IMU yaw rate, previous steer, previous throttle, observer car speed `v_est`,
longitudinal slip `S`, yaw-rate residual. Action: `[steer, throttle]` in
[-1, 1], steer ±30°, throttle `(a+1)/2 × 1.0`.

Invariants:

- **Every stage emits the same 117-dim observation.** Change `n_beams`,
  `fov_half_deg` or the slot list and every checkpoint is unloadable.
- **Throttle scale 1.0 is fixed.** No curriculum: raising a cap re-labels every
  learned action (it collapsed the earlier lineage twice). Throttle commands a
  wheel speed; beyond ~1.15·v/25.25 the extra is wheelspin, which the slip slot
  shows and stage 6's grip term charges for.
- **`gamma` and the episode length are derived** from the measured control
  period (`1 − dt/13 s`, `245 s / dt`), so they are fixed in seconds on every
  machine. The printed values differ per machine on purpose.
- **`step()` is phase- and latency-locked.** It waits `decimation` ticks past
  the *last observation*, not past the send, so gradient time is absorbed into
  the window; and it sends no earlier than the tick after the observation, so
  the command always rides the second tick's reply and the action delay is the
  same in training and at deployment. `diag/step_ms` must equal the startup-
  measured period; `diag/overhead_ms` must stay under it.
- **Checkpoint numbering is cumulative** across stages
  (`reset_num_timesteps=False`).
- **Resume with the replay buffer.** `final_replay_buffer.pkl` is saved next to
  `final.zip`; a resume that finds no buffer collects `--warmup` steps first.
  Training against an empty buffer is what collapsed the earlier lineage.

## Competition legality

`/ips`, `/odom`, `/tf`, and all lap and collision telemetry are restricted.
The policy reads LiDAR, encoders, IMU and its own commands only. `/odom` feeds
the **progress and speed reward**, `collision_count` / `lap_count` /
`last_lap_time` / `reset_command` feed **episode management**: training-time
only, none of it exists at inference. Keep it that way.

## Measured simulator constants

| quantity | measured | documented |
|---|---|---|
| Encoder units | **radians** (wheel angle) | "ticks, 1920/rev" — off ~300× |
| Wheel radius | **0.0581 m** | 0.0590 m |
| Throttle → wheel speed | **25.25 m/s per unit**, within a millisecond; the encoders echo the command, not the car | 22.88 m/s top speed |
| Tire (forward) | peak μ 0.72 at slip 0.15, asymptote 0.464 at 0.25; drag 0.273·v | — |
| Sim loop | **18.6 Hz** stock, **77 Hz** with `tcp_nodelay:=true`; a round trip, not a clock | 40 Hz |
| Scan | **1081** beams, 270°, 0.25°/beam | 1080 |
| `/imu` vs `/odom` angular | identical, one source | unstated |

The loop is a socket round trip: the sim emits telemetry only in reply to the
bridge, and Nagle plus delayed ACK on loopback hold it to ~50 ms. The shim sets
`TCP_NODELAY` and re-arms `QUICKACK` after every recv **and send**;
`loop_hz_cap:=45` paces the replies to the organizers' 40–50 Hz evaluation
rate. `tools/sim_rate_probe.py` measures the loop with no ROS.

## Gotchas

- Subscriber QoS must match the bridge exactly (RELIABLE / VOLATILE / KEEP_LAST
  / depth 1); a mismatch connects and silently delivers nothing.
- `reset_command` is level-triggered: the env pulses it True→False.
- The sim's ResetManager restores the encoder counters on reset; the env clears
  its encoder state after the settle.
- `pgrep -f` / `pkill -f` match the calling shell's own script text.
- Nothing before `learning_starts` (5k fresh, `--warmup` on a bufferless
  resume) means anything in the curves.
- `.gitignore` excludes `runs/*` and re-admits `final.zip` and `monitor.csv`;
  replay buffers (~290 MB each) are never committed.
