# RoboRacer Sim Racing League 2026 — Final Race

Our submission for the final race on the IROS 2026 track: one Docker image that
drives the car by itself.

**How it drives.** Wheel encoders and IMU give dead reckoning. A segmented
scan-to-map localizer matches the LiDAR against a map of the track that ships
inside the image. Pure pursuit, with a small bounded LQR correction, follows a
racing line that also ships inside the image. No ground truth is used.

**Result.** 39 timed laps at 8.50–8.55 s with zero wall contacts (2026-09-19).

**What changed since qualification 1.** The localizer. AMCL was blind along the
track on the long straight, which is exactly where this circuit starts, so the
error it could not correct was carried into the first braking zone. The
replacement cuts the track into segments and gives each one its own acceptance
gates. Three follower settings went with it and each was measured on the car:
the warm-up speed cap released after the launch corner, the steering cap matched
to the line's planned hairpin budget, and a 1.0 m lookahead floor, which took
the hairpin-1 exit slide to zero.

## Run it the way the organizers do

Two containers on the same host network.

**1. Start the simulator** (the organizers' image):

```bash
docker run --name autodrive_roboracer_sim --rm -it \
  --network=host --ipc=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all \
  autodriveecosystem/autodrive_roboracer_sim:2026-iros-compete
```

**2. Start our image:**

```bash
docker run --name autodrive_roboracer_api --rm -it \
  --network=host --ipc=host \
  <dockerhub-user>/roboracer:iros-2026-final
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
```

The stack runs in the foreground, so its output is the container's output: it is
on screen during the run and in `docker logs` afterwards. Nothing is written to
disk. New shells start nothing — the stack is launched only by the entrypoint,
never from `~/.bashrc`. If `ros2 topic list` is empty, a `ros2` daemon on the
host is answering instead of the container; run `ros2 daemon stop` once and
retry.

## For us

```bash
./scripts/build.sh          # -> autodrive_racer:iros-2026-final
./scripts/run.sh sim --headless
./scripts/run.sh race
```

`race` takes no arguments. To try a change without rebuilding, pass it through
the entrypoint: `RACER_EXTRA_ARGS="v_max:=8.5" ./scripts/run.sh race`.

## What is in the image

| part | what it is |
|---|---|
| `autodrive_roboracer` | the provided devkit package, unmodified |
| `roboracer_stack` | our package: dead reckoning, the localizer and its bootstrap, pure pursuit, the configs, the map, the racing line. See [roboracer_stack/README.md](roboracer_stack/README.md). |
| `/home/autodrive_devkit.sh` | the entrypoint: sets the environment and launches everything |

Repository layout: `Dockerfile` builds the image from the official devkit image
and adds `roboracer_stack`; `scripts/build.sh` and `scripts/run.sh` are for us
and are not part of the image.

The image installs two packages on top of the devkit image, `python3-scipy` and
`ros-humble-rmw-cyclonedds-cpp`, and no nav2: the localizer reads the occupancy
grid itself and is a plain node, so it needs neither a map server nor a
lifecycle manager.

## Competition legality

Restricted topics: `/ips`, `/odom`, `/tf`, and all lap and collision telemetry.

The image reads none of them. The initial pose comes from the measured spawn
position of the car (`roboracer_stack/roboracer_stack/common/frames.py`) plus
the IMU heading; the bootstrap node confirms the localizer adopted it before the
follower is allowed to drive. Lap telemetry is off and the nodes that read
ground truth for development — the CSV logger, the error measurement, the scan
dump, the drive-on-truth diagnostic — are not in this image at all, so there is
no mode switch that could be left in the wrong position. `rqt_graph` shows no
subscriber on any restricted topic.

The devkit bridge's ground-truth transform is remapped from `/tf` to
`/tf_ground_truth` at launch level. The devkit package itself is untouched; the
remap is what keeps `roboracer_1` from having two parents and breaking the
transform tree, and the organizers confirmed it is acceptable.

`RACER_BOOTSTRAP_MODE` is not read by this entrypoint. `bootstrap_mode:=truth`
still exists in the launch file — one read of `/ips` before the car moves, then
the subscription is destroyed — for a track whose spawn has not been measured.
It is not what the submission runs.
