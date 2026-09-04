# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Autonomous racing on the Porto track in the AutoDRIVE RoboRacer simulator
(Sim Racing League 2026). Two independent stacks share the workspace and share
nothing else:

- **`rl_racer/`** — SAC policy driving straight off LiDAR. No map, no
  localization. Trained as a staged curriculum where each stage resumes from
  the previous stage's weights.
- **`devkit_ws/` + `raceline/`** — classical stack: SLAM map → minimum-curvature
  raceline → AMCL localization → pure pursuit.

`README.md` holds measured results and lap times; `rl_racer/EXPERIMENTS.md` is
the full training log including bugs found the expensive way.

## Environment

Three separate Python environments, and mixing them is the most common failure:

| Context | Python | Why |
|---|---|---|
| devkit bridge, ROS nodes | **system** python3.10 | `rclpy`/`cv_bridge` are C extensions built against it |
| RL training / `enjoy.py` | **`.venv-rl`** | torch + CUDA + stable-baselines3 live only there |
| notebooks (`raceline/`) | `.venv-rl` | needs `trajectory_planning_helpers`, `casadi` |

Order matters when both are needed: source ROS **first**, then the venv — ROS
puts `rclpy` on `PYTHONPATH`, and the venv must win on `PATH`. Conda will not
work at all.

Every ROS terminal:

```bash
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source devkit_ws/install/setup.bash
export CYCLONEDDS_URI=file://$(ros2 pkg prefix racer_common)/share/racer_common/config/cyclonedds.xml
```

or just `source ros_env.sh`, which does all four and warns if the workspace is
unbuilt.

The `CYCLONEDDS_URI` line is not optional for the full stack: CycloneDDS defaults
`MaxAutoParticipantIndex` to 9, so with localhost-only the tenth node fails to
start — and `race.launch.py` is about ten nodes.

## Build

```bash
cd devkit_ws
PYTHONNOUSERSITE=1 colcon build                       # all packages
PYTHONNOUSERSITE=1 colcon build --packages-select racer_localization
```

`PYTHONNOUSERSITE=1` is required — a newer setuptools in `~/.local` breaks any
`ament_python` build against the system `packaging`. Do not run colcon from an
activated venv.

## Running

The simulator is not in this repo (382 MB Unity build, download from AutoDRIVE
releases; expected at `simulator_practice/autodrive_simulator/`). Start it
first, in every workflow.

```bash
# classical stack — bridge (TF remapped) + chassis + localizer + pure pursuit
ros2 launch racer_bringup race.launch.py                      # AMCL
ros2 launch racer_bringup race.launch.py localizer:=slam      # slam_toolbox
ros2 launch racer_bringup race.launch.py mode:=race           # nothing restricted
ros2 launch racer_bringup race.launch.py bridge:=false lookahead_k:=0.70

# mapping
ros2 launch racer_mapping mapping.launch.py     # save via the RViz SlamToolbox panel

# RL: bridge in a system-python terminal, policy in the venv
ros2 launch autodrive_roboracer bringup_headless.launch.py    # wait for "Connected!"
cd rl_racer && python enjoy.py runs/stage3_v3/checkpoints/sac_990000_steps.zip --episodes 1
```

`race.launch.py` (in `racer_bringup`, the only package that composes others)
starts `bridge` + `chassis` + the localizer named by `localizer:=` + `follower`,
plus `instruments` and one RViz. Each is launchable alone, and each piece can be
switched off (`bridge:=`, `chassis:=`, `localization:=`, `follower:=`).
`localizer:=` picks `amcl` (nav2 against `track_clean.pgm`), `slam`
(slam_toolbox against the `track_sm` pose graph), or `none`.
`mode:=race` forces every legal setting at once and omits `instruments`
entirely; `mode:=dev` (default) keeps the development conveniences and every
node reading a restricted topic says so at startup. Pure-pursuit tunables
(`lookahead_*`, `v_max`, `a_lat_max`, `throttle_max`, `steering_gain`) pass
through both launch files, so tuning never needs a rebuild.

## RL training

```bash
cd rl_racer
python3 probe.py                    # ALWAYS first: verifies plumbing, tick rate, reset, collisions
python verify_env.py                # confirms each import resolves to the venv, not ~/.local or ROS

./run_train.sh --stage 2 --gradient-steps 4 --resume runs/v3/checkpoints/sac_370000_steps.zip
tensorboard --logdir runs/
```

`run_train.sh` sources ROS, activates the venv, and refuses to start unless the
sim, the bridge on port 4567, and CUDA are all up. It `exec`s `train_sac.py`, so
any `train_sac.py` flag passes straight through.

Validation gate between stages — never promote an unmeasured checkpoint:

```bash
python enjoy.py runs/<stage>/checkpoints/<ckpt>.zip --speed encoder --episodes 2
```

Add `--legacy-obs` to load a v1–v3 checkpoint (it restores the old odom speed
sign; runtime override only, `config.py` is untouched).

Pass = zero crashes (`end=timeout`) and lap time no worse than the previous
stage. Lap time is `steps × control_period / laps`; never compare lap *counts*
across different control rates.

There is no test suite. The only `colcon test` targets are the third-party
devkit's flake8/pep257/copyright linters.

## Architecture

### RL side (`rl_racer/`)

- `rl_racer/config.py` — **all tunables**, as dataclasses. Reward weights are
  the file you actually iterate on. Every constant carries the measurement that
  justifies it; read the comment before changing a number.
- `rl_racer/obs.py` — LiDAR FOV crop + min-pool + observation assembly. Kept
  separate so a future gym sim can produce byte-identical observations.
- `rl_racer/env.py` — Gymnasium env over ROS 2. The sim is free-running, so
  `step()` blocks on the LiDAR topic: **the scan is the master clock**, since
  the bridge publishes every topic together inside one socket.io handler.
- `stages/stage*.py` — each stage is a `NAME`/`RESUME_FROM`/`DEFAULTS`/`apply(cfg)`
  module that mutates the config. Stage N's `apply()` calls stage N-1's, so
  stages compose rather than duplicate.

Invariants that break checkpoints if violated:

- **Every stage emits the same 95-dim observation** (90 min-pooled beams +
  speed, yaw_rate, prev_steer, prev_throttle, throttle_cap). Change `n_beams`
  or `fov_half_deg` and every existing checkpoint becomes unloadable.
- **The throttle cap is in the observation.** Raising it changes what
  `action[1] = +1` physically means; feeding the cap in is what lets the critic
  distinguish old replay-buffer transitions from new ones.
- **`gamma` is derived** from the measured control period
  (`1 - dt/horizon_seconds`), not hard-coded, so the planning horizon stays
  fixed in seconds across machines. The printed value differs per machine on
  purpose.
- **Checkpoint numbering is cumulative** across stages (`reset_num_timesteps=False`):
  stage 2 resuming at 370k produces `sac_380000_steps.zip`, not `sac_10000`.
- `max_episode_steps` is per stage and is a *step* count, so it means different
  driving time at different control rates.

### Classical side (`devkit_ws/src/`)

One rule holds the layout together: **only `racer_bringup` composes.** Every
other package launches its own subsystem and nothing else. Before that rule,
`race.launch.py` lived in `racer_control` and included the AMCL launch file by
name, so the A/B could not be run from the top-level entry point at all — and
the shared half of the two localizers was copy-pasted into both.

- `autodrive_devkit/` — third-party bridge, BSD, **unmodified**. When its
  behaviour needs changing, do it with a launch-level remap in `racer_bringup`
  (see `bridge.launch.py`), never by editing it.
- `racer_common/` — no nodes. TF frame names, the lidar extrinsic, the spawn
  pose, workspace paths (`frames.py`), and the competition restricted-topic list
  with its runtime warning (`restricted.py`). Also owns `cyclonedds.xml`.
  Everything else imports from here instead of repeating literals.
- `racer_localization/` — `dead_reckoning` (encoders + IMU → `odom→roboracer_1`,
  heading and position carried forward to each stamp, `VEHICLE_MODEL.md` §5.4),
  `localization_bootstrap`, `localization_error`, and the two interchangeable
  localizers. `chassis.launch.py` is the half both share; `amcl.launch.py` and
  `slam.launch.py` contain *only* their localizer; `instruments.launch.py` holds
  every node that reads a restricted topic, so legality is one inclusion.
- `racer_control/` — `pure_pursuit`, `calibrate_steering`. Control only; the
  name is now true. `pure_pursuit` estimates the car's speed by running the
  sim's own tire curve on the measured wheel speed (`speed_source: tire`, a
  contracting observer with bounded error, p90 0.15–0.22 m/s against ground
  truth) and commands throttle as a wheel speed inside a ±0.08 slip band placed
  one measured round trip ahead, 175 ms here (`throttle_mode: slip`,
  `cmd_delay_auto`), because in this sim the
  encoders report the throttle command, not the car, and the command reaches
  the wheel one bridge round trip late (`raceline/VEHICLE_MODEL.md` §3.1, §3.6).
  `encoder` / `legacy` keep the old law for A/B; `fused` (IMU + pose) is kept
  for reference and does not work here, see §5.3. It publishes `~/status` every tick; `log_localization` records it as
  `pp_*` columns and `raceline/analyze_run.py run.csv` turns a log into lap
  times, tracking error, corner bias, weave, estimator error and encoder ratio.
- `racer_mapping/` — slam_toolbox mapping config, `map_publisher`, and the
  committed Porto map. Scan matching is **on** (karto only adds graph vertices
  inside that branch, so a map built without it is unusable for localization);
  loop closure is on too, because the scan matcher's per-node slop integrates —
  measured +0.023 m after one lap, +0.750 m after three.
- `racer_bringup/` — `race.launch.py`, `bridge.launch.py`, the RViz configs.
  The composition root, and the only package that depends on the others.

TF tree under localization: `map →(amcl) odom →(dead_reckoning) roboracer_1
→(static) lidar`. The devkit broadcasts `world→roboracer_1` from the IPS; if
that stays on `/tf`, `roboracer_1` gets two parents and the tree breaks — hence
the remap to `/tf_ground_truth`.

### Raceline (`raceline/`)

`optimize_raceline.py` is the pipeline: map → centerline → minimum-curvature
line → velocity profile → CSVs (`s,x,y,psi,kappa,w_r,w_l[,v_mps]`) that
`pure_pursuit` consumes. Run it from `.venv-rl`. It rebuilds the centerline when
the CSV is missing and exports `raceline_a4.0/4.5/4.9.csv`: the same geometry at
three lateral limits, so grip is stepped up on the car instead of guessed.
Geometry does not depend on grip, only the velocity profile does. The notebooks
import from it for plots; `make_speed_variants.py` re-profiles any other line
under the same physics.

Every number about the car is in `raceline/VEHICLE_MODEL.md`, derived from the
simulator's Unity source, with references. The three that change decisions:
throttle commands a wheel speed (25.25 m/s per unit), so encoders read the
command, not the car; the tire's robust limits are the friction-curve
asymptotes, 4.55 m/s² longitudinal and 4.90 lateral, and demands between the
asymptote and the peak are what makes the car drift; drag is linear, 0.273·v.
Read it before changing a limit.

## Competition legality

`/ips`, `/odom`, `/tf`, and all lap and collision telemetry are **restricted**.
Legal inputs: LiDAR, camera, IMU, wheel encoders, steering/throttle feedback.

But restriction is about **when and how often**, not only which topic. The first
lap is a **warmup** and the timer starts after it, so those topics are readable
up to that point. What is never acceptable is a node that keeps reading ground
truth into the timed laps. Two access patterns, two functions in
`racer_common/restricted.py`:

| | pattern | legal | function |
|---|---|---|---|
| seed | one read, before the car moves, then the subscription is **destroyed** | yes | `restricted.seed()` + `restricted.released()` |
| stream | a continuous subscription | no | `restricted.warn()` (red) |

Keep this separation intact when adding code:

- The RL policy reads LiDAR + encoders only. `/odom` feeds the **progress
  reward** and `collision_count`/`reset_command` feed **episode management** —
  training-time only, none of it exists at inference.
- The classical stack localizes with LiDAR + IMU + encoders against the
  pre-built map. Ground truth appears in exactly two places: the one-shot pose
  seed (`bootstrap_mode:=truth`, **race-legal**, released by
  `_release_truth()` as soon as it resolves) and `localization_error`
  (continuous, development-only, omitted by `mode:=race`).
- Anything reading a restricted topic **continuously** must (a) live in
  `racer_localization/launch/instruments.launch.py`, which `mode:=race` omits
  wholesale, and (b) call `racer_common.restricted.warn()` so it announces itself
  at startup. A comment is not enough: the boundary used to be 33 comments and no
  checks, and `pure_pursuit`'s `pose_topic` DEFAULTED to ground-truth `/odom`, so
  development runs produced lap times that read as race-legal and were not.

`mode:=race` deliberately does **not** force `bootstrap_mode:=global` any more.
It used to, and that was a bug rather than caution: slam_toolbox has no global
relocalization at all, so `mode:=race localizer:=slam` fell through to the
hardcoded `frames.SPAWN_*` — and slam seeds with a ±0.5 m correlative search, so
a wrong constant could never be recovered and stayed frozen for the whole timed
run. The warmup seed is both legal and the only thing that makes that
combination work. `SPAWN_*` is now only the fallback for `bootstrap:=false` and
`bootstrap_mode:=global`; the bootstrap prints the measured spawn whenever the
constant disagrees with it.

## Measured simulator constants

Several contradict the published documentation. Trust the measurements — they
were re-derived at cost.

| quantity | measured | documented |
|---|---|---|
| Encoder units | **radians** (wheel angle) | "ticks, 1920/rev" — off by ~300× |
| Wheel radius | **0.0581 m** | 0.0590 m |
| Sim tick rate | **~18 Hz** | bridge advertises 40 Hz |
| Speed vs throttle | **≈ 24 × throttle** | 22.88 m/s top speed |
| `twist.linear` frame | **body**, not world | unstated |
| `/imu` vs `/odom` angular | identical — one source | unstated |
| Scan size | **1081** beams (inclusive endpoints) | 1080 |

`LidarFOV` derives its crop indices from the live scan header rather than
hard-coding them, precisely because of that last row.

## Gotchas

- **Subscriber QoS must match the bridge exactly** (RELIABLE / VOLATILE /
  KEEP_LAST / depth 1). A mismatch connects and then silently delivers nothing.
- **`reset_command` is level-triggered** — the bridge re-emits it every tick, so
  it must be pulsed True→False or the sim resets forever. `env.py` handles this.
- **Nothing before `learning_starts` means anything** (5k fresh, or `--warmup`
  on resume) when reading training curves.
- **`.gitignore` re-admits specific run files** after excluding `runs/*`
  wholesale; pattern order matters, since git cannot re-include a file whose
  parent directory is excluded. Replay buffers (~29 GB) must never be committed.
- Torch in the venv is a CUDA build, but `enjoy.py` and the env are happy on
  CPU — at 18 Hz the per-step budget is 55 ms and SAC updates take single-digit
  ms.
