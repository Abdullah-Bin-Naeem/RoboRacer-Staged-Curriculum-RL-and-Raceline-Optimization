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

### Classical stack (predicted, from the raceline notebooks)

| line | length | \|κ\| max | lap |
|---|---|---|---|
| centerline | 30.93 m | 0.92 | 7.580 s |
| min-curvature (scipy) | 27.72 m | 1.00 | **6.813 s** |
| min-curvature (TUM `tph`) | 27.54 m | 1.07 | 7.000 s |

Both optimized lines beat the centerline by ~8–10%, almost entirely by
shortening the path — the corridor is 1.40 m wide on a 0.27 m car, so there is
little room for geometry to do more. These times assume a lateral grip limit
that has not yet been measured on the vehicle, so treat the *comparison* as
sound and the absolute numbers as provisional.

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
