# RoboRacer Sim Racing League 2026 — Qualification 1

Our submission for qualification round 1 on the Porto track: one Docker image
that drives the car by itself.

**How it drives.** Wheel encoders and IMU give dead reckoning. AMCL matches
the LiDAR against a map of the track that ships inside the image. Pure pursuit
follows a racing line that also ships inside the image. No ground truth is used.

**IROS 2026 (ported from the `multi-track` branch).** Default line
`raceline_tum_iqp_h7.0_a7.0b.csv`, warmup speed cap, friction-circle slip band
with acceleration feedforward, and re-localization after a wall reset at
measured checkpoints; validated there at a 45 Hz loop (run 18: 19 clean laps,
best 9.45 s). The Porto qualification result below is kept for history.

**Result.** Best lap 6.45 s; more than 500 laps without touching a wall
(2026-09-12).

## Run it the way the organizers do

Two containers on the same host network.

**1. Start the simulator** (the organizers' image):

```bash
docker run --name autodrive_roboracer_sim --rm -it \
  --network=host --ipc=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all \
  autodriveecosystem/autodrive_roboracer_sim:2026-iros-practice
```

**2. Start our image:**

```bash
docker run --name autodrive_roboracer_api --rm -it \
  --network=host --ipc=host \
  <dockerhub-user>/roboracer:qualification-1
```

**3. Press Connect** in the simulator. The car starts driving on its own.

The order of steps 1 and 2 does not matter: our bridge waits on port 4567 until
the simulator connects. If the container is started with `--entrypoint
/bin/bash`, nothing runs automatically; start the stack with
`/home/autodrive_devkit.sh`.

**Looking inside a run:**

```bash
docker exec -it autodrive_roboracer_api bash
source /home/autodrive_devkit/install/setup.bash
ros2 topic list
tail -f /home/autodrive_devkit/log/racer_*.log
```

New shells start nothing: the stack is launched only by the entrypoint, never
from `~/.bashrc`. If `ros2 topic list` is empty, a `ros2` daemon on the host is
answering instead of the container; run `ros2 daemon stop` once and retry.

## What is in the image

| part | what it is |
|---|---|
| `autodrive_roboracer` | the provided devkit package, unmodified |
| `roboracer_stack` | our package: dead reckoning, the AMCL bootstrap, pure pursuit, the AMCL config, the map, the racing line. See [roboracer_stack/README.md](roboracer_stack/README.md). |
| `/home/autodrive_devkit.sh` | the entrypoint: sets the environment and launches everything |

Repository layout: `Dockerfile` builds the image from the official devkit
image and adds `roboracer_stack`; `scripts/build.sh` and `scripts/run.sh` are
for us and are not part of the image.

## Competition legality

Restricted topics: `/ips`, `/odom`, `/tf`, and all lap and collision telemetry.

The image reads none of them. The initial pose comes from the measured spawn
position of the car (`roboracer_stack/roboracer_stack/common/frames.py`) plus
the IMU heading; the bootstrap node confirms AMCL adopted it before the
follower is allowed to drive. Lap telemetry is off. The launch prints a banner
saying so at startup, and `rqt_graph` shows no subscriber on any restricted
topic.

`RACER_BOOTSTRAP_MODE=truth` (not used by default) instead reads `/ips` once
before the car moves and then destroys the subscription. The organizers
confirmed restricted topics may be read during the warm-up lap, so this is
legal too; it exists for a track whose spawn has not been measured.

## Build and develop

```bash
./scripts/build.sh                 # builds autodrive_racer:qualification-1
./scripts/run.sh sim --headless    # simulator without a window (or: sim)
./scripts/run.sh racer             # our image, same as the organizers run it
```

Settings for development, all optional, read by the entrypoint from
`docker run -e ...`:

| variable | default | effect |
|---|---|---|
| `RACER_BOOTSTRAP_MODE` | `spawn` | `truth` reads `/ips` once in the warm-up lap; `global` starts AMCL with no prior |
| `RACER_CONTROL_HZ` | `20` | follower loop rate (IROS 2026 values were validated at 20) |
| `RACER_TCP_NODELAY` | `true` | socket shim on the bridge; `false` = stock bridge (~18 Hz loop) |
| `RACER_LOOP_HZ_CAP` | `45` | with the shim, cap the simulator loop at this rate; `0` = uncapped |
| `RACER_MODE` | `race` | `dev` turns lap telemetry back on (restricted, development only) |
| `RACER_AUTOSTART` | `1` | `0` starts the container with nothing running |
| `RACER_EXTRA_ARGS` | | extra `race.launch.py` arguments, e.g. `"v_max:=7.5"` |

The development tools, the loop-rate measurements and the other racing lines
live on the `qualification_1_pure_pursuit` branch, which this branch was cut
from. The racing line itself was planned on `main`.

## Submit

```bash
docker tag autodrive_racer:qualification-1 <user>/roboracer:qualification-1
docker login
docker push <user>/roboracer:qualification-1
```

Submit the Docker Hub link. The guide asks for run instructions in the
repository overview on Docker Hub; the section below is written to be pasted
there as-is.

---

## Docker Hub overview (paste as-is)

### RoboRacer Sim Racing League 2026 — Qualification 1

AMCL localization and pure pursuit along a pre-optimized racing line on the
Porto track. Built on `autodriveecosystem/autodrive_roboracer_api:2026-iros-practice`.
The map and the racing line ship inside the image; nothing to mount, nothing to
configure.

**1. Pull:** `docker pull <user>/roboracer:qualification-1`

**2. Start the simulator** (organizers' image, in its own terminal):

```bash
docker run --name autodrive_roboracer_sim --rm -it \
  --network=host --ipc=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all \
  autodriveecosystem/autodrive_roboracer_sim:2026-iros-practice
```

**3. Start this container:**

```bash
docker run --name autodrive_roboracer_api --rm -it \
  --network=host --ipc=host \
  <user>/roboracer:qualification-1
```

**4. Press Connect** on the simulator's menu panel. The car drives immediately.
Steps 2 and 3 can be done in either order.

**Inspecting a run.** Extra shells start nothing (no `~/.bashrc` automation):

```bash
docker exec -it autodrive_roboracer_api bash
source /home/autodrive_devkit/install/setup.bash
ros2 topic list        # if empty: ros2 daemon stop, then retry
tail -f /home/autodrive_devkit/log/racer_*.log
```

**Contents.** `autodrive_roboracer` is the provided devkit, unmodified.
`roboracer_stack` is our package: dead reckoning, nav2 AMCL against the built-in
map, and the pure pursuit follower. No restricted topic is read.
