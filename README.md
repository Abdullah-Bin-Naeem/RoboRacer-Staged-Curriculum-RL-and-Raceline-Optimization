# RoboRacer — AutoDRIVE Sim Racing League 2026

Autonomous racing on the **Porto** track in the AutoDRIVE RoboRacer simulator.

Two independent approaches, one workspace:

- **Hierarchical RL** — a SAC policy driving straight off LiDAR. No map, no
  localization, fully reactive. Trained as a staged curriculum where each stage
  resumes from the previous stage's weights.
- **Classical stack** — SLAM map → minimum-curvature raceline → particle-filter
  localization → pure pursuit.

Both run entirely on competition-legal sensors.

---

## Results

### RL policies (deterministic, `enjoy.py`)

| policy | speed source | rate | lap time | notes |
|---|---|---|---|---|
| v3 @370k (stage 1) | `/odom` | 7.7 Hz | 7.79 s | not race-legal |
| v3 @370k | encoders | 7.7 Hz | 8.50 s | 9% cost of the legal swap |
| stage 2 final | encoders | 17.8 Hz | 8.23 s | 1 crash in 2 episodes |
| **stage 3 final** | encoders | 18.5 Hz | **7.44 s** | **32 laps × 3 eps, zero crashes** |

Reliability, measured as mean steps between crashes (cap-independent, since each
run used a different episode cap):

| run | MTTC | crash rate at its own cap |
|---|---|---|
| stage 1 (pre-collapse) | 555 | 0.88 |
| **stage 2** | **14,913** | **0.26** |
| stage 3 run 1 | 697 | 0.98 |

**38.7 hours** of training across six runs. Best policy is stage 3: 9.6% faster
than stage 2, race-legal, and crash-free over three full episodes.

### Classical stack (measured, AMCL + pure pursuit, race-legal sensors)

Minimum-curvature line on the SLAM map, velocity profile from the simulator's
own tire and drag model (`raceline/VEHICLE_MODEL.md`), AMCL on LiDAR + IMU +
encoders, pure pursuit with a tire-observer speed estimate and a slip-band
throttle placed one measured round trip ahead. Laps are timed from the log by
track position, on this laptop; the loop's round trip is three simulator
frames (175 ms at 17.6 Hz).

| line (a_lat) | simulator | tick | laps | best | mean | note |
|---|---|---|---|---|---|---|
| 7.0 m/s² | headless | 19.3 Hz | 7 clean | **6.50 s** | 6.58 s | run 38; longitudinal profile 5.0 m/s², wall clearance 0.07 m |
| 7.0 m/s² | headless | 20.3 Hz | 9 clean | 6.58 s | 6.63 s | run 29; longitudinal profile 4.55 m/s² |
| 7.0 m/s² | graphics | 17.6 Hz | 12 clean | 6.60 s | 6.69 s | run 28; 69 consecutive clean laps, runs 25–29 |
| 6.5 m/s² | headless | 19.8 Hz | 32 clean | 6.65 s | 6.71 s | run 40; the safe default, 5.0 profile, corner bias −0.01 |
| profile prediction, 7.0 | | | | 6.36 s | | at the 5.0 longitudinal limit; assumes instant commands |

#### ICRA 2026 track (branch `multi-track`)

Mapped from the competition simulator and climbed the same ladder in ten
runs. Its two hairpins (curvature 1.37, against Porto's tightest 0.92) are
where this track differs: at the 6.5 rung the profile asks the tire for
essentially its full 7.0 m/s² there, and the delay's speed error tipped it over
twice in 46 laps. The submission line keeps 6.5 everywhere except the two
hairpins, capped at 6.0 by lateral zones.

| line | simulator | tick | laps | best | mean | note |
|---|---|---|---|---|---|---|
| 7.0, hairpins 6.0, T3 7.5, longitudinal 6.0/5.0, straight 9, five margin zones (`raceline_a7.0zv_hard_l6.0_corners_h.csv`), circle band + feedforward | graphics | 17.7 Hz | 11 clean | **11.60 s** | 11.66 s | run 23; **submission**, the `track:=icra2026` default; nothing under 0.18 m, hairpin demand ≤ 6.1 |
| same at longitudinal 5.5/5.0, straight 8 (`raceline_a7.0zv_hard_l5.5.csv`) | graphics | 17.8 Hz | 10 clean | 11.80 s | 11.87 s | run 20; first longitudinal rung |
| 7.0, hairpins 6.0, T2 6.5, straight 8, T1-exit margin (`raceline_a7.0zv_hard.csv`) | graphics | 17.8 Hz | 10 clean | 12.00 s | 12.05 s | run 17; the flag-free fallback |
| 7.0, hairpins 6.0, straight 8 (`raceline_a7.0zv.csv`) | graphics | 17.8 Hz | 11 clean | 12.00 s | 12.05 s | run 15; 0.11 m at the T1 exit and T2 at 7.06, hence the hardening |
| 6.5, hairpins 6.0, straight 8 (`raceline_a6.5zv.csv`), `target_lead_s 0` | graphics | 17.7 Hz | 12 clean | 12.15 s | 12.22 s | run 13; the fallback line |
| same line, `target_lead_s 0.08` | graphics | 17.4 Hz | 3 clean | 12.25 s | 12.27 s | run 11; the lead cost 0.1 s in this track's 7.8 m braking zones |
| 6.5, hairpins 6.0 (`raceline_a6.5z.csv`) | graphics | 17.7 Hz | 13 clean | 12.35 s | 12.43 s | run 10; hairpin demand ≤ 6.3 m/s², nothing under 0.17 m |
| 6.5 | graphics | 17.2 Hz | 8 clean | 12.20 s | 12.33 s | run 7; 2 hits in 46 laps at this rung over later runs, both hairpins |
| 7.0 | graphics | 17.0 Hz | 8 clean | 12.05 s | 12.09 s | run 8; the tire's limit, fast line |
| 4.0 | graphics | 17.7 Hz | 13 clean | 14.70 s | 14.75 s | run 2; first drive, stack unchanged from Porto |
| profile prediction, 6.5z | | | | 11.90 s | | assumes instant commands |
| physics floor: margin 0.10, 7.0 everywhere, 8 m/s, instant commands | | | | 11.36 s | | driven once (run 16): hit in 8 s at the right-wall lane, demand 4.6 — the margin is the bottleneck, not the tire |

The 8 m/s speed cap was tried and dropped: 7 % of the lap is above 7 m/s, it
bought 0.03 s, and the extra approach speed is what pushed the middle-wall
hairpin over. `track:=icra2026` launches this configuration with no other
arguments.

The track's best known lap is 6.46 s. What remains is the round trip: the car
brakes early to keep the tire on the line in the two tightest corners and
runs 2–3 % under the profile through them. On a slower machine the follower
measures its own delay and derates the speed targets rather than the wall
margin (`derate_*` in `pure_pursuit.yaml`). Every run and every change, in
order, is in `raceline/VEHICLE_MODEL.md` §7.

---

## Hierarchical training

Every stage emits the **same 95-dim observation**, which is what makes
checkpoints transferable between them. Change the beam count or FOV and every
earlier checkpoint becomes unloadable.

| Stage | What changes | Race-legal | Resumes from |
|---|---|---|---|
| 1 `stage1_base` | reproduce a known-good policy from scratch | ✗ uses `/odom` | — |
| 2 `stage2_sensors` | 2 observation slots → permitted topics | ✓ | stage 1 |
| 3 `stage3_speed` | throttle-cap curriculum, reward reweighted toward speed | ✓ | stage 2 |
| 4 `stage4_reserved` | robustness (rate randomisation, sensor noise) — not yet defined | — | stage 3 |
| 5 `stage5_fresh` | **new 118-dim lineage**: ±110°, race-legal slip slots, throttle scale 0.5 fixed (no curriculum), from scratch on the competition track | ✓ | — |
| 6 `stage6_push` | stage 5 + speed pressure (`w_speed` 0.6, lap bonus, grip term) | ✓ | stage 5 |

Stage 3's curriculum raises the throttle ceiling only when the agent proves it
can use the current one: crash rate ≤ 0.25, mean episode length ≥ 400, **and**
throttle demand ≥ 55% of the cap. That last gate is what stage 1 lacked — its
curriculum fired on competence alone and destroyed a working policy with a +50%
jump at step 379,318.

---

## Measured simulator constants

Verified empirically. Several contradict the published documentation.

| quantity | measured | documented |
|---|---|---|
| Encoder units | **radians** (wheel angle) | "ticks, 1920/rev" — wrong by ~300× |
| Wheel radius | **0.0581 m** | 0.0590 m |
| Sim tick rate | **~18 Hz** | bridge advertises 40 Hz |
| Speed vs throttle | **≈ 24 × throttle** | 22.88 m/s top speed |
| `twist.linear` frame | **body**, not world | unstated |
| `/imu` vs `/odom` angular | identical — one source | unstated |

---

## Layout

```
devkit_ws/src/
  autodrive_devkit/   third-party bridge (BSD, unmodified)
  racer_mapping/      slam_toolbox config + the Porto map
  racer_control/      pure pursuit, dead reckoning, AMCL localization
  wall_follow/        simple baseline
raceline/             notebooks: centerline extraction, raceline optimization
rl_racer/             SAC training, staged curriculum, evaluation
```

## Setup

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-amcl \
                 ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager

cd devkit_ws
PYTHONNOUSERSITE=1 colcon build
source install/setup.bash
```

`PYTHONNOUSERSITE=1` is required: a newer setuptools in `~/.local` breaks any
`ament_python` build against the system `packaging`.

Every terminal:

```bash
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source devkit_ws/install/setup.bash
export CYCLONEDDS_URI=file://$(ros2 pkg prefix racer_common)/share/racer_common/config/cyclonedds.xml
```

That last line matters. CycloneDDS defaults `MaxAutoParticipantIndex` to 9, so
with localhost-only the tenth node on the domain fails to start — and the full
stack is about ten nodes.

## Running

Start the simulator, then:

```bash
# classical stack: bridge + localization + raceline following
ros2 launch racer_bringup race.launch.py

# build a map
ros2 launch racer_mapping mapping.launch.py

# RL policy
cd rl_racer
python enjoy.py runs/stage3_v3/checkpoints/sac_990000_steps.zip --episodes 1
```

---

## Competition constraints

`/ips`, `/odom`, `/tf`, and all lap and collision telemetry are **restricted
during racing**. Only LiDAR, camera, IMU, wheel encoders, and steering/throttle
feedback are legal inputs.

The RL policy reads LiDAR and encoders only, so it is legal by construction.
The classical stack localizes with LiDAR + IMU + encoders against a pre-built
map. Ground truth appears in exactly two places, both development-only and both
flagged at runtime: seeding the initial pose, and `localization_error`, which
measures how far the race-legal estimate drifts from truth.

`autodrive_devkit/` is third-party and **unmodified**. Where its behaviour needed
changing — remapping its ground-truth TF off `/tf` so the frame tree stays
valid — that is done with a launch-level remap in `racer_control`.

## Not in this repo

- **The simulator** (382 MB Unity build) — download from AutoDRIVE releases.
- **RL replay buffers** (~29 GB, 132 files at ~240 MB each).
- **Intermediate checkpoints** — each run's `final.zip` and the two policies
  referenced above are kept; the other ~670 MB are not.
- **The serialized SLAM pose graph** (30 MB) — only needed to resume a mapping
  session. The `.pgm` + `.yaml` map is committed.

Full training log, including the bugs found the expensive way, is in
[`rl_racer/EXPERIMENTS.md`](rl_racer/EXPERIMENTS.md).
