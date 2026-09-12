# Qualification 1 — pure pursuit

Submission branch for the AutoDRIVE RoboRacer Sim Racing League 2026,
qualification round 1, Porto track.

**Algorithm:** minimum-curvature raceline → AMCL localization → pure pursuit.
No RL, no online mapping. The map and the racing line are built offline on
`main` and baked into the image as finished artefacts.

**Measured:** 6.45 s best on `raceline_a7.0.csv`, more than 500 laps without a
contact across loop rates from 18 to 85 Hz (2026-09-12).

This branch carries only what has to be in the container. The RL stack, the
raceline optimizer, the notebooks and the run logs stay on `main`; the
development tooling, the loop-rate measurements and the other racing lines stay
on `qualification_1_pure_pursuit`, which this branch was cut from.

---

## Layout

The repository root is a copy of the devkit workspace's `src/`, the same place
our package lives inside the container (`/home/autodrive_devkit/src/`):

```
Dockerfile              FROM the official devkit image; adds one package
autodrive_devkit.sh     the entrypoint the rules name — the only automation
roboracer_stack/        ours: localization, planning, control
scripts/                build.sh, run.sh — for us; not part of the image
```

`autodrive_roboracer` is never modified — the rules forbid it, and the base
image already ships it built. Where its behaviour has to change we do it with a
launch-level remap (`roboracer_stack/launch/bridge.launch.py` moves the
ground-truth TF off `/tf`, because otherwise `roboracer_1` has two parents and
the TF tree breaks). Our code is one separate package, as the guide requires; see
[`roboracer_stack/README.md`](roboracer_stack/README.md) for what is in it.

## Build and run

Two containers, same host network. The simulator image is the organizers'; the
racer image is ours.

```bash
./scripts/build.sh                 # -> autodrive_racer:qualification-1

./scripts/run.sh sim --headless    # terminal 1: simulator
./scripts/run.sh racer             # terminal 2: our stack
```

Order does not matter — the bridge listens on 4567 and blocks until the
simulator connects. In graphics mode, hit **Connect** in the simulator; the car
starts driving on its own.

## How the organizers run it

The default entrypoint is the whole submission. No arguments, no manual step:

```bash
docker run --name autodrive_roboracer_api --rm -it \
  --network=host --ipc=host \
  <dockerhub-user>/roboracer:qualification-1
```

`/home/autodrive_devkit.sh` sets the environment, sources the workspace, and
launches the devkit bridge together with our nodes. It then hands the terminal
to `bash`, so the container is immediately usable for inspection with the car
already driving.

If the container is started with `--entrypoint /bin/bash` — the form printed in
the technical guide — the entrypoint never runs. Bring the stack up by running
it by hand:

```bash
/home/autodrive_devkit.sh
```

### Inspection shells

```bash
docker exec -it autodrive_roboracer_api bash
source /home/autodrive_devkit/install/setup.bash   # needed: see below
ros2 topic list
tail -f /home/autodrive_devkit/log/racer_*.log
```

Nothing is appended to `~/.bashrc` and nothing is automated from it, per the
guide, so a new shell starts no nodes — and, as a consequence, has no ROS on
its path until you source the workspace yourself. That one line is the whole
cost, and it is what guarantees a second stack can never come up behind you.

The DDS settings are not in that line: they are `ENV` in the image, so every
`docker exec` shell inherits them and can actually see the running nodes. A
shell on a different RMW would find none — an empty `ros2 node list` and no
error.

## Changing the configuration without rebuilding

Every knob is an environment variable read by `/home/autodrive_devkit.sh`.
None is needed for the submission; the defaults are the race configuration.

| variable | default | what it does |
|---|---|---|
| `RACER_BOOTSTRAP_MODE` | `spawn` | `spawn` seeds AMCL from the measured spawn constant + IMU heading, no restricted topic; `truth` seeds once from `/ips` in the warm-up lap (organizer-confirmed); `global` searches with no prior |
| `RACER_CONTROL_HZ` | `40` | follower loop rate |
| `RACER_MODE` | `race` | `dev` turns lap telemetry (restricted lap topics) back on |
| `RACER_AUTOSTART` | `1` | `0` gives a shell with nothing running |
| `RACER_EXTRA_ARGS` | — | any `race.launch.py` argument, e.g. `"v_max:=7.5 lookahead_k:=0.6"` |

## The racing line

`raceline_a7.0.csv`: minimum-curvature geometry, velocity profile at 7.0 m/s²
lateral, 0.15 m body-to-wall margin with extra margin on the straight after R1
and the S-exit approach. It passes 0.25 m from the wall at its tightest point
(the C2 apex). Chosen over the faster `a7.0_rec` line (0.03 m of S-exit
clearance) because qualification rewards clean laps, not tenths.

## Competition legality

Legal inputs: LiDAR, camera, IMU, wheel encoders, steering/throttle feedback.
`/ips`, `/odom`, `/tf` and all lap and collision telemetry are restricted.

The container launches with `mode:=race`: lap telemetry off, and the follower
steers on the localizer's estimate rather than on `/odom`. No node that reads a
restricted topic continuously exists on this branch at all.

The default container reads **no restricted topic at all**. AMCL is seeded by
`localization_bootstrap` in `spawn` mode from the measured spawn constant
(`common/frames.py` `SPAWN_*` = 0.800, 3.158, -1.5707, measured with the car
parked on 2026-09-10 and identical in 30 logged launches) plus the IMU's
absolute heading, and the node confirms AMCL adopted it before the follower is
released. Nothing subscribes to `/ips`, so the organizers' `rqt_graph` and bag
show a stack that never touches ground truth.

`RACER_BOOTSTRAP_MODE=truth` keeps the previous behaviour: **one** sample of
`/ips` before the car has moved, then the subscription is destroyed
(`roboracer_stack/common/restricted.py`, `seed()` / `released()`). The
organizers confirmed that restricted topics may be read during the warm-up lap,
so this is legal too; it is the fallback for a track whose spawn has not been
measured yet. `global` runs AMCL's particle search with no prior at all and
costs a convergence phase.

## Submitting

```bash
docker tag autodrive_racer:qualification-1 <user>/roboracer:qualification-1
docker login
docker push <user>/roboracer:qualification-1
```

Then submit the Docker Hub link. The guide requires the repository overview
there to carry step-by-step instructions for that exact tag — the next section
is written to be pasted in as-is.

---

## Docker Hub overview (paste this into the repository overview)

### RoboRacer Sim Racing League 2026 — Qualification 1

AMCL localization and pure pursuit along a pre-optimized minimum-curvature
raceline, on the Porto track. Built on
`autodriveecosystem/autodrive_roboracer_api:2026-iros-practice`. The map and the
racing line ship inside the image; there is nothing to mount and nothing to
configure.

**1. Pull the tag**

```bash
docker pull <user>/roboracer:qualification-1
```

**2. Start the simulator** (organizers' image, in its own terminal)

```bash
docker run --name autodrive_roboracer_sim --rm -it \
  --network=host --ipc=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all \
  autodriveecosystem/autodrive_roboracer_sim:2026-iros-practice
```

**3. Start this container**

```bash
docker run --name autodrive_roboracer_api --rm -it \
  --network=host --ipc=host \
  <user>/roboracer:qualification-1
```

No entrypoint override, no extra commands. `/home/autodrive_devkit.sh` sets the
environment, sources the workspace, and starts the AutoDRIVE bridge together
with our localization and control nodes. The bridge listens on port 4567 and
waits, so the order of steps 2 and 3 does not matter.

**4. Hit Connect** on the simulator's Menu Panel. The car starts driving
immediately.

**Inspecting a run.** Extra bash sessions start nothing — the codebase is
launched only from the entrypoint, never from `~/.bashrc`:

```bash
docker exec -it autodrive_roboracer_api bash
source /home/autodrive_devkit/install/setup.bash
ros2 topic list
tail -f /home/autodrive_devkit/log/racer_*.log
```

If `ros2 topic list` comes back empty, a `ros2` daemon from the host is
answering (the container is on the host network); run `ros2 daemon stop` once
and repeat. `ros2 bag record -a` and `rqt_graph` do not use the daemon.

**Contents.** `autodrive_roboracer` is the provided devkit package, unmodified.
`roboracer_stack` is our separate package: dead reckoning, nav2 AMCL against the
baked-in map, and the pure pursuit follower.
