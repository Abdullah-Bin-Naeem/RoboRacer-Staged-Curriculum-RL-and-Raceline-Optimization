# Staged training

Each stage starts from the previous stage's checkpoint. **Every stage produces
the same 95-dim observation** — that invariant is what makes checkpoints
transferable. Change the beam count or FOV and every checkpoint below becomes
unloadable.

| Stage | Name | What changes | Legal? | Resume from | ep cap |
|---|---|---|---|---|---|
| 1 | `stage1_base` | — (reproduces v3) | ❌ uses `/odom` | scratch | 4400 |
| 2 | `stage2_sensors` | **2 obs slots → legal topics** | ✅ | stage 1 | 4400 |
| 3 | `stage3_speed` | curriculum on, `w_speed` 0.2→0.6, `w_center` 0.3→**0**, cap ceiling 0.5 | ✅ | stage 2 | **1200** |
| 4 | `stage4_reserved` | *not defined yet* | — | stage 3 | — |
| **5** | `stage5_fresh` | **new lineage, 118-dim**: ±110°, 110 beams, slip slots, throttle scale 0.5 fixed | ✅ | scratch | 245 s |
| **6** | `stage6_push` | stage 5 + speed pressure (`w_speed` 0.6, lap 200, grip 0.05) | ✅ | stage 5 | 245 s |

Stages 1→3 differ in **only two config fields**:

```
obs.use_encoder_speed   False -> True    slot 91: /odom  -> wheel encoders
env.legacy_speed_sign   True  -> False   (moot on the encoder path)
```

Slot 92 (yaw rate) moves `/odom` → `/imu` but the **number is identical** —
the bridge feeds both topics from the same variable.

## Fresh lineage (stages 5–6)

Nothing from stages 1–4 loads into it (118-dim vs 95). Design, in one table:

| | v3 lineage | fresh |
|---|---|---|
| FOV | ±90° (hairpin exit ON the boundary) | **±110°**, still 2°/beam |
| extra slots | — | `v_est` (tire observer), slip `S`, yaw-rate residual — all race-legal |
| throttle | cap 0.20 raised by a curriculum | **scale 0.5, fixed** — RL finds its own speed |
| encoders | 1-tick rate, spikes on every reset | 3-tick window + discontinuity guard |
| episode / cooldown | steps | **seconds**, derived from the measured tick |
| bridge | stock, ~18 Hz | **shimmed, capped at the 45 Hz eval rate**, decimation 2 |

Why no cap: throttle commands *wheel speed* (25.25 m/s per unit). Peak-grip
acceleration at 8 m/s needs 0.36; above ~0.40 the tire is past its asymptote
and more throttle is wheelspin. 0.5 covers the track. Stage 5 refuses a sim
tick outside 15–32 ms so it cannot silently train at the wrong rate.

```bash
# bridge, from the multi-track branch, pinned to the evaluation loop rate
ros2 launch racer_bringup bridge.launch.py tcp_nodelay:=true loop_hz_cap:=45

./run_train.sh --stage 5                                        # ~6-8 h
python enjoy.py runs/stage5_fresh/final.zip --stage 5 --episodes 3
./run_train.sh --stage 6 --resume runs/stage5_fresh/final.zip   # loads its buffer
```

Banner to confirm: `measured control period ~44 ms (22.5 Hz) = sim tick ~22 ms x
decimation 2`, `max_episode_steps ≈ 5500 (245 s)`, `obs_dim=118`,
`gradient_steps=3`. Expect `time/fps` ≈ 18–22 once episodes lengthen.

Watch `slip/frac_peak` (share of steps with |S| in 0.10–0.20 — is it using the
tire) and `slip/v_est_max` (real top speed, not the encoder echo).

## Prerequisites (every stage)

```bash
# terminal 1 — simulator
cd ~/Documents/roboracer/simulator_practice/autodrive_simulator
./AutoDRIVE\ Simulator.x86_64 -ip 127.0.0.1 -port 4567

# terminal 2 — bridge (NO venv: needs system python for cv_bridge)
cd ~/Documents/roboracer/devkit_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch autodrive_roboracer bringup_headless.launch.py     # wait for "Connected!"
```

## Running

`run_train.sh` sources ROS, activates the venv, and checks sim + bridge + CUDA.

```bash
cd ~/Documents/roboracer/rl_racer

# Stage 1 — base (or SKIP: use the existing runs/v3/checkpoints/sac_370000_steps.zip)
./run_train.sh --stage 1 --gradient-steps 4

# Stage 2 — sensor adaptation
./run_train.sh --stage 2 --gradient-steps 4 \
  --resume runs/v3/checkpoints/sac_370000_steps.zip

# Stage 3 — speed curriculum
./run_train.sh --stage 3 --gradient-steps 4 \
  --resume runs/stage2_sensors/checkpoints/sac_520000_steps.zip
```

Each stage sets its own run name (`runs/stage2_sensors/` etc.), timesteps,
learning rate, curriculum flag and warmup. Override any of them by passing the
flag explicitly.

**Stage 1 is optional** — `runs/v3/checkpoints/sac_370000_steps.zip` already is
a stage-1 result (1800 steps, 30 laps, zero crashes).

## Validation gate — run between stages

Never promote a checkpoint you have not measured deterministically:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/roboracer/.venv-rl/bin/activate
python enjoy.py runs/<stage>/checkpoints/<ckpt>.zip --legacy-obs --speed encoder --episodes 2
```

Pass criteria: **zero crashes** (`end=timeout`) and lap time no worse than the
previous stage. Lap time = `episode_seconds / laps`, and episode seconds =
`steps × control_period` — do not compare lap *counts* across different control
rates, only lap *times*.

Measured references:

| policy | speed source | rate | lap time |
|---|---|---|---|
| v3 370k | `/odom` | 7.7 Hz | **7.79 s** |
| v3 370k | encoders | 7.7 Hz | 8.50 s |
| v3 370k | encoders | 18.8 Hz | 7.99 s |

Checkpoint numbering is CUMULATIVE across stages (resume uses
`reset_num_timesteps=False`), so stage 2 starting at 370k produces
`sac_380000_steps.zip` ... `sac_520000_steps.zip` -- not `sac_150000`.

Stage 2's job is to close that 8.50 → 7.79 gap (~9%) caused by the sensor swap.

## Watch during training

- `time/fps` — ~12–18 here. A drop means gradients are overrunning the sim tick.
- `rollout/ep_len_mean` — flat for >30k steps means stuck, not learning slowly.
- `ends/crash` — should fall below ~0.3.
- `action/throttle_cap` (stage 3) — steps up 0.20 → 0.22 → …
- `action/throttle_demand` (stage 3) — needs ≥ 55% to earn a raise.

## Gotchas

- **Nothing before `learning_starts` means anything** (5k fresh, or `--warmup`
  on resume).
- **`gamma` is derived** from the measured control period, not hard-coded — the
  printed value will differ between machines and that is intentional.
- **`max_episode_steps` is PER STAGE** and is a step count, so it means
  different amounts of driving time on different machines:

  | stage | steps | at ~18 Hz | laps |
  |---|---|---|---|
  | 1, 2 | 4400 | ~245 s | ~30 |
  | **3** | **1200** | **~67 s** | **~8** |

  Stage 3 is shorter *on purpose*: the curriculum's crash gate is "fraction of
  episodes ending in a crash", which depends on the cap as well as the policy.
  At 4400 the gate (<=0.25) could never open -- run 1 never raised the cap once
  in 290k steps. At 1200 it opens when the car survives ~4,170 steps between
  crashes.
