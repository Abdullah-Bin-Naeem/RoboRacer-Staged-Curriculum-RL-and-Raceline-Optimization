# rl_racer — SAC on AutoDRIVE RoboRacer

Single-instance SAC training against the live AutoDRIVE simulator, using a
cropped forward LiDAR field of view instead of the full 270°.

## Layout

| File | Purpose |
|---|---|
| `rl_racer/config.py` | **All tunables.** Reward weights live here — this is the file you iterate on. |
| `rl_racer/obs.py` | FOV crop + min-pool + observation assembly (shared with any future gym sim). |
| `rl_racer/env.py` | Gymnasium env over ROS 2; makes the free-running sim step-synchronous. |
| `probe.py` | Pre-flight check of the ROS plumbing. **Run this first.** |
| `train_sac.py` | SAC training loop. |
| `enjoy.py` | Run a trained checkpoint deterministically. |

## Observation (94 dims, all in [-1, 1])

- **throttle_cap** — the current curriculum cap (see below).
- **90 beams** — the ±90° forward window of the 1080-beam scan (raw indices
  180→900 = 720 beams), **min-pooled** by 8. Min-pooling, not striding: striding
  can skip the one beam that sees a thin obstacle.
- `speed / 5.0`, `yaw_rate / 3.0`, `prev_steer`, `prev_throttle`.

Previous action is in the observation so that penalising steering jerk stays
Markovian.

## Action (2 dims)

`[steer, throttle]` in [-1, 1], rescaled on publish to full `±1.0` steering and
`[0.0, throttle_cap]`. No braking or reverse for now.

### Throttle curriculum

Top speed is **not** fixed. The cap starts at 0.20 and is raised by 0.10 each
time the agent proves it can drive at the current one:

- last 20 episodes have crash rate ≤ 0.35 **and** mean length ≥ 200 steps
- at least 15k steps since the previous raise
- ceiling 1.0 (`--throttle-ceiling`, or `--no-curriculum` to fix it)

Why a curriculum rather than an uncapped throttle: uncapped random exploration
crashes within a few steps, so the replay buffer fills with nothing but crashes
and SAC has almost no useful signal to learn from. Why not a permanent cap: it
would be the thing limiting lap time, which defeats the point.

**The current cap is part of the observation** (the 95th element). This matters:
raising the cap changes what `action[1] = +1` physically means, so transitions
already in the replay buffer would otherwise contradict new ones. Feeding the
cap in lets the critic tell the two regimes apart.

Watch `action/throttle_sat` — the fraction of steps at >0.9 of the cap. If it
pins near 1.0 the agent wants more speed than allowed; if it sits interior, the
cap is not what limits it. `race/max_speed` shows the peak speed actually hit.

Note `v_max` (20.0) is only a normaliser, but it **must** stay above any
reachable speed: if it clips, the agent cannot tell 5 m/s from 12 m/s and the
speed reward saturates.

## Reward

```
+ w_progress * forward_progress_m     (5.0)   <- dominant term, "distance covered"
+ w_speed    * v/v_max                (0.2)   <- "speed"
- w_center   * lateral_asymmetry      (0.3)   <- "center of road"
- w_smooth   * |steer - prev_steer|   (0.3)   <- "steering smoothness"
- w_prox     * wall_proximity         (0.5)
- step_penalty                        (0.02)
- crash_penalty on collision          (15.0, terminal)
- stall_penalty if stopped 2 s        (5.0,  terminal)
```

Progress is the displacement projected on the heading, **not** raw speed —
rewarding raw speed teaches the car to floor it into a wall. Centring is
map-free: it uses left/right LiDAR asymmetry, so no centreline file is needed.

## Measured facts about this setup

These were verified with `probe.py` against the live simulator, and several
contradict what the code suggests:

| Thing | Value | Note |
|---|---|---|
| Scan size | **1081 beams**, not 1080 | inclusive endpoints; the crop derives indices from the live header |
| ±90° crop | raw indices `[180:900]` = 720 beams | min-pooled by 8 → 90 |
| Sim tick rate | **~18 Hz** | the bridge *advertises* `lidar_scan_rate=40`; the real Bridge round-trip is 18 Hz |
| Control rate | ~18 Hz (`decimation=1`) | `decimation=2` would give 9 Hz — too slow to race |
| Throttle 0.10 | ≈ 2.5 m/s | so `throttle_max=0.20` ≈ 5 m/s, matching `v_max` |
| Camera decode | **not** the bottleneck | skipping it measured 18.0 Hz, identical — the cap is the sim itself |

**Torch is CPU-only here, deliberately.** At 18 Hz you get a 55 ms budget per
step and SAC's updates take single-digit ms, so the GPU would sit ~95% idle.
The CUDA build plus its `nvidia-*` deps is ~3 GB versus 190 MB for CPU. To
switch later: `pip install --user --force-reinstall torch` (default PyPI index).

## Expected wall-clock

At ~15 effective steps/s (18 Hz minus reset overhead):

| Milestone | Steps | Wall-clock |
|---|---|---|
| Buffer seeded, learning starts | 5k | ~6 min |
| Stops crashing constantly | ~100k | ~2 h |
| Drives clean laps | ~400k | **~7 h** |
| Competitive pace | ~1M | ~18 h |

## Running

Three terminals.

```bash
# 1. simulator (drop -batchmode -nographics if you want to watch it)
cd simulator_practice/autodrive_simulator && ./AutoDRIVE\ Simulator.x86_64

# 2. bridge
source devkit_ws/install/setup.bash
ros2 launch autodrive_roboracer bringup_headless.launch.py

# 3. check, then train
cd rl_racer
python3 probe.py                       # verify plumbing first
python3 train_sac.py --timesteps 400000 --run-name v1
tensorboard --logdir runs/             # watch convergence
```

## Reading convergence

Total episode reward alone will not tell you if the reward is broken, so
`train_sac.py` logs every term separately:

- `reward/progress` should climb and dominate. If `reward/speed` dominates
  instead, the car is revving without covering ground.
- `ends/crash` should fall below ~0.3 before you expect lap times to improve.
- `ends/stall` staying high means the car learned to stop — raise
  `w_progress` or lower `step_penalty`.
- `rollout/ep_rew_mean` flat for >50k steps after `learning_starts` means the
  reward, not SAC, is the problem.

## Notes

- **Conda will not work here.** `rclpy` is a C extension built for the system
  Python 3.10; conda ships its own Python and libstdc++. Use system Python.
- `reset_command` is level-triggered — the bridge re-emits it every tick, so it
  must be pulsed True→False or the sim resets forever. `env.py` handles this.
- Subscriber QoS must match the bridge exactly (RELIABLE/VOLATILE/KEEP_LAST/1)
  or topics connect and deliver nothing.
