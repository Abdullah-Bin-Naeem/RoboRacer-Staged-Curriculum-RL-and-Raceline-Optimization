# Qualification 1 — pure pursuit

Submission branch for the AutoDRIVE RoboRacer Sim Racing League 2026,
qualification round 1, Porto track.

**Algorithm:** minimum-curvature raceline → AMCL localization → pure pursuit.
No RL, no online mapping. The map and the racing line are built offline on
`main` and baked into the image as finished artefacts.

**Measured:** 6.50 s best, 6.58 s mean over 7 clean laps (run 38, headless sim
at 19.3 Hz). The track's best known lap is 6.46 s.

This branch is deliberately **not** a fork of `main`. It shares no history with
it and carries only what has to be in the container. The RL stack, the raceline
optimizer, the notebooks and the run logs stay on `main`.

---

## Layout

The repository root is a copy of the devkit workspace's `src/` — the same two
packages, in the same places, as `/home/autodrive_devkit/src/` inside the
container:

```
Dockerfile              FROM the official devkit image; adds one package
autodrive_devkit.sh     the entrypoint the rules name — the only automation
autodrive_devkit/       the provided package, unmodified, NOT copied into the image
roboracer_stack/        ours: perception, localization, planning, control, mapping
                        + tools/, offline analysis; NOT installed into the image
scripts/                build.sh, run.sh, bench.sh, raceline_editor.py — for us;
                        not part of the image
```

`autodrive_roboracer` is never modified — the rules forbid it, and the base
image already ships it built. Where its behaviour has to change we do it with a
launch-level remap (`roboracer_stack/launch/bridge.launch.py` moves the
ground-truth TF off `/tf`, because otherwise `roboracer_1` has two parents and
the TF tree breaks). Our code is one separate package, as §3.4 requires; see
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

Every knob is an environment variable read by `/home/autodrive_devkit.sh`:

| variable | default | what it does |
|---|---|---|
| `RACER_TRACK` | `icra` | which circuit: `icra` or `porto`. Selects the map, the spawn pose, the scan-fit grid and the default line together |
| `RACER_RACELINE` | per track | which line to drive, by **bare filename** — `raceline_a4.0.csv`. Looked up in that track's raceline directory |
| `RACER_PATH_CSV` | — | the older form: a full path. Use `RACER_RACELINE` instead |
| `RACER_LOCALIZER` | `amcl` | `amcl`, `slam`, or `none` |
| `RACER_BOOTSTRAP_MODE` | `truth` | `truth` seeds the pose once from `/ips`; `global` searches with no prior |
| `RACER_CONTROL_HZ` | `40` | follower loop rate |
| `RACER_MODE` | `race` | `dev` re-enables the instruments; needed for any localization measurement, since `race` omits `instruments.launch.py` entirely |
| `RACER_AUTOSTART` | `1` | `0` gives a shell with nothing running |
| `RACER_EXTRA_ARGS` | — | anything else, e.g. `"v_max:=7.5 lookahead_k:=0.6"` |

`scripts/run.sh racer` forwards every one of these from your shell, so the
usual invocation stays one command:

```bash
RACER_TRACK=porto RACER_RACELINE=raceline_a6.5.csv ./scripts/run.sh racer
```

or against the image directly:

```bash
docker run --rm -it --network=host --ipc=host \
  -e RACER_TRACK=icra -e RACER_RACELINE=raceline_a4.0.csv \
  autodrive_racer:qualification-1
```

Both tracks and all twenty racing lines are baked into the image, so switching
between them never needs a rebuild. `RACER_LOCALIZER=slam` works on `porto`
only — icra ships no pose graph.

## Benchmarking the localizer

The simulator knows exactly where the car is, so localization quality is a
measurable number rather than an impression. Two containers as usual, plus a
third command:

```bash
./scripts/run.sh sim --headless       # terminal 1
./scripts/bench.sh up                 # terminal 2
./scripts/bench.sh record baseline --seconds 90
./scripts/bench.sh score baseline     # -> runs/baseline.html
./scripts/bench.sh sweep              # every AMCL variant, ranked
```

`runs/` is bind-mounted, so the CSV and the report land on the host and survive
the container. The report opens in a browser with no server and no network.

**What is compared.** Truth is `/ips` position with `/imu` heading. The estimate
is TF `map->odom` composed with `odom->roboracer_1` -- AMCL's correction applied
on top of dead reckoning, which is literally what `pure_pursuit` reads, so the
benchmark scores the pose the car actually races on rather than `/amcl_pose`.

**Why a separate container from `run.sh racer`.** That one is the submission,
run exactly as the organizers run it. Benchmarking needs a host mount (or the
CSV dies with `--rm`), `RACER_MODE=dev` (both ground-truth readers live in
`instruments.launch.py`, which `mode:=race` omits wholesale), and scipy (the
image does not ship it). `bench.sh` supplies all three without touching the
image: the tools are bind-mounted, never baked in. `mode:=dev` changes no part
of the control path -- the car steers on the estimate either way.

**scipy is installed with `--no-deps` on purpose.** A plain `pip install scipy`
pulls numpy 2.x over the image's 1.22.2 and every compiled ROS Humble extension
stops importing. `bench.sh` pins `scipy==1.10.1`, checks numpy did not move, and
rolls back if it did.

**The error floor is about 0.03 m.** Comparing an estimate in `map` against
truth in `world` assumes the frames coincide, which holds to 0.024 m RMS and
0.19 deg. A result at that magnitude is the frame alignment, not the localizer.

**It will not agree with what `localization_error` prints, on purpose.** That
node compares the *latest* transform against the *latest* truth with no time
alignment, so the two poses are typically one bridge frame apart -- and at 7 m/s
a 40 ms offset is 0.3 m of pure bookkeeping. The CSV looks the odometry leg up
*at the truth's own timestamp*, so both poses describe the same instant. On a
real 90 s run the two read 0.32 m and 0.13 m. The aligned one is the honest
number; the node's is a live upper bound.

The report answers four questions in order: does the error matter (cross-track
error against the wall margin actually available at that point of the lap), is
the localizer contributing anything (against the `dr_err` column, which is dead
reckoning measured on the same trajectory at the same instants), is it rotating
the pose when it should only ever translate it, and -- from the scan-vs-map fit
at the true pose versus the estimated one -- whether the remaining error is
tunable at all or is the map.

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
speed targets (`derate_*` in `config/pure_pursuit.yaml`), which was validated
in-sim at 11 Hz — it gives back about a second a lap and stays on the line
rather than holding the pace and hitting a wall.

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
subscription (`roboracer_stack/common/restricted.py`, `seed()` / `released()`).
The first lap is a warmup and the timer starts after it, so that read is inside
the permitted window, and nothing reads ground truth during the timed laps. Set
`RACER_BOOTSTRAP_MODE=global` for AMCL's particle search if a stricter reading is
wanted; it costs a convergence phase.

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

**Contents.** `autodrive_roboracer` is the provided devkit package, unmodified.
`roboracer_stack` is our separate package: dead reckoning, nav2 AMCL against the
baked-in map, and the pure pursuit follower.
