# Qualification 1 — pure pursuit

Submission branch for the AutoDRIVE RoboRacer Sim Racing League 2026,
qualification round 1, Porto track.

**Algorithm:** minimum-curvature raceline → AMCL localization → pure pursuit.
No RL, no online mapping. The map and the racing line are built offline on
`main` and baked into the image as finished artefacts.

**Measured:** 6.50 s best, 6.58 s mean over 7 clean laps (run 38, headless sim
at 19.3 Hz). The track's best known lap is 6.46 s.

This branch is deliberately **not** a fork of `main`. It shares no history with
it and carries only what has to be in the container: the devkit, the five
`racer_*` packages, the Porto map, three racing lines, and the Docker build.
The RL stack, the raceline optimizer, the notebooks and the run logs stay on
`main`.

---

## Build and run

Two containers, same host network. The simulator image is the organizers'; the
racer image is ours.

```bash
./docker/build.sh                  # -> autodrive_racer:qualification-1

./docker/run.sh sim --headless     # terminal 1: simulator
./docker/run.sh racer              # terminal 2: our stack
```

Order does not matter — the bridge listens on 4567 and blocks until the
simulator connects. In graphics mode, hit **Connect** in the simulator; the car
starts driving on its own.

To watch what it is doing:

```bash
docker exec -it autodrive_roboracer_api bash
tail -f /home/racer_ws/log/racer_*.log
```

That shell gets the ROS environment from `~/.bashrc` and starts nothing — one
stack per container, which is what the rules require.

## How the organizers run it

The default entrypoint is the whole submission. No arguments, no manual step:

```bash
docker run --name autodrive_roboracer_api --rm -it \
  --network=host --ipc=host \
  <dockerhub-user>/roboracer:qualification-1
```

`/home/autodrive_devkit.sh` sources ROS, the devkit workspace and ours, sets the
DDS configuration, and launches the devkit bridge together with our nodes. It
then hands the terminal to `bash`, so the container is immediately usable for
inspection with the car already driving.

If the container is started with `--entrypoint /bin/bash` — the form printed in
the technical guide — the entrypoint never runs. Start the stack by hand:

```bash
/home/start_racer.sh
```

## Changing the configuration without rebuilding

Every knob is an environment variable read by `/home/start_racer.sh`:

| variable | default | what it does |
|---|---|---|
| `RACER_PATH_CSV` | `raceline_a7.0.csv` | which line to drive (absolute path) |
| `RACER_LOCALIZER` | `amcl` | `amcl`, `slam`, or `none` |
| `RACER_BOOTSTRAP_MODE` | `truth` | `truth` seeds the pose once from `/ips`; `global` searches with no prior |
| `RACER_CONTROL_HZ` | `40` | follower loop rate |
| `RACER_MODE` | `race` | `dev` re-enables the instruments |
| `RACER_AUTOSTART` | `1` | `0` gives a shell with nothing running |
| `RACER_EXTRA_ARGS` | — | anything else, e.g. `"v_max:=7.5 lookahead_k:=0.6"` |

```bash
docker run --rm -it --network=host --ipc=host \
  -e RACER_PATH_CSV=/home/racer_ws/install/racer_control/share/racer_control/raceline/raceline_a6.5.csv \
  autodrive_racer:qualification-1
```

## The three racing lines

Same geometry, three velocity profiles. All measured on the car with AMCL and
this controller; the full run history is `raceline/VEHICLE_MODEL.md` §7 on
`main`.

| line | best / mean | clean laps | wall clearance | |
|---|---|---|---|---|
| `raceline_a7.0.csv` | **6.50 / 6.58** | 7 | 0.07 m L, 0.10 m R | **the default** (run 38) |
| `raceline_a6.5.csv` | 6.65 / 6.71 | 32 | intact | the safe rung (run 40) |
| `raceline_a7.0_rec.csv` | 6.45 / 6.53 | 9 | **0.03 m** at the S-exit | beats the record, no margin (run 41) |

`a7.0` is the submission line: it is 0.15 s a lap faster than the safe rung and
keeps a real wall margin. `a7.0_rec` is faster still and was deliberately not
adopted — margin and a sub-6.46 lap are in tension, and the tension is set by
the bridge round trip, not by grip.

If the evaluation machine runs the loop slow, the controller handles it without
being told: `pure_pursuit` measures its own command delay online and derates the
speed targets (`derate_*` in `pure_pursuit.yaml`), which was validated in-sim at
11 Hz — it gives back about a second a lap and stays on the line rather than
holding the pace and hitting a wall.

## Competition legality

Legal inputs: LiDAR, camera, IMU, wheel encoders, steering/throttle feedback.
`/ips`, `/odom`, `/tf` and all lap and collision telemetry are restricted.

The container launches with `mode:=race`, which is a single switch rather than
nine defaults that each have to be right:

- `instruments.launch.py` — every node that reads a restricted topic
  continuously — is **not included at all**;
- lap telemetry off, and the follower steers on the localizer's estimate rather
  than on `/odom`.

Ground truth is read in exactly one place: `localization_bootstrap` takes **one**
sample of `/ips` before the car has moved, seeds AMCL with it, and destroys the
subscription (`racer_common/restricted.py`, `seed()` / `released()`). The first
lap is a warmup and the timer starts after it, so that read is inside the
permitted window, and nothing reads ground truth during the timed laps. Set
`RACER_BOOTSTRAP_MODE=global` for AMCL's particle search if a stricter reading is
wanted; it costs a convergence phase.

`autodrive_devkit/` is unmodified and is **not copied into the image** — the base
image already ships it built, and the rules forbid modifying it. Where its
behaviour has to change we do it with a launch-level remap: `bridge.launch.py`
moves the devkit's ground-truth TF off `/tf`, because otherwise `roboracer_1`
has two parents and the TF tree breaks.

## Layout

```
docker/
  Dockerfile            FROM the official devkit image; adds /home/racer_ws
  autodrive_devkit.sh   the entrypoint the rules name
  start_racer.sh        the launch command, also runnable by hand
  racer_env.sh          environment only; launches nothing
  build.sh  run.sh      convenience wrappers
devkit_ws/src/
  autodrive_devkit/     third-party bridge, BSD, unmodified, NOT in the image
  racer_common/         frames, paths, the restricted-topic list
  racer_localization/   dead reckoning, AMCL, slam fallback, bootstrap
  racer_control/        pure_pursuit + the three racing lines
  racer_mapping/        the Porto map
  racer_bringup/        race.launch.py — the composition root
```

Only `racer_bringup` composes. Every other package launches its own subsystem
and nothing else.

## Submitting

```bash
docker tag autodrive_racer:qualification-1 <user>/roboracer:qualification-1
docker login
docker push <user>/roboracer:qualification-1
```

Then submit the DockerHub link. The repository overview there needs step-by-step
instructions for pulling that exact tag and running it — the "How the organizers
run it" section above is written to be pasted in.
