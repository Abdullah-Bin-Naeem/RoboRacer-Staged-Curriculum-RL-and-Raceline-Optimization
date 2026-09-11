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
scripts/                build.sh, run.sh, hz.sh — for us; not part of the image
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

## Running the sim and the racer on two machines

The bridge is a WebSocket server on 4567 and the simulator is its *client*, so
the two never had to share a host — and on a machine that cannot feed the bridge
fast enough, they should not. Everything the bridge publishes comes out of a
single `@sio.on('Bridge')` handler, one burst per frame, and the throttle and
steering go back to the sim only after that handler returns. So the tick rate is
whatever the racer machine can sustain: put the simulator and ten ROS nodes on
one desktop and the whole loop slows down together.

Measured here: ~17.5 Hz with both containers on one machine, 40–50 Hz with the
racer container on its own. The organizers tune and verify at 40–50 Hz.

**On the racer machine** (Docker Desktop for Windows):

```powershell
docker load -i autodrive_racer_qualification-1.tar
docker run --name autodrive_roboracer_api --rm -it -p 4567:4567 autodrive_racer:qualification-1
```

`--network=host` is deliberately **not** used here. On Docker Desktop it puts the
container in the WSL2 VM's namespace rather than on the Windows LAN interface, so
the sim cannot reach 4567 at all; `-p 4567:4567` publishes it properly.
`--ipc=host` is meaningless there too. On a Linux racer machine, keep both flags
and use `./scripts/run.sh racer` unchanged.

Allow inbound TCP 4567 through Windows Defender Firewall — Docker Desktop
prompts the first time it binds; accept **Private networks**. `ipconfig` gives
the address the simulator connects to.

**On this machine:**

```bash
./scripts/run.sh sim --headless 192.168.1.42   # the racer machine's address
./scripts/run.sh sim                           # or with a window: type it into Connect
```

Prefer headless for anything timed. Unity throttles its frame rate whenever its
window is minimised, occluded or on another workspace, and the symptom is not an
error — every rate in the system drops silently and the lap times still look
plausible. Watching a rate readout in another window is exactly the situation
that triggers it.

Use a wire. Wi-Fi jitter arrives as tick jitter, one for one.

## Watching the sensor rate

`tools/rate_monitor.py` reports the simulator tick and the control-loop rate once
a second, live, while the car drives. It needs no rebuild — it is copied into the
running container and imports nothing from `roboracer_stack`:

```bash
./scripts/hz.sh                          # Linux racer machine
./scripts/hz.sh --laps --csv runs/rate_run62.csv
```

```powershell
# Windows: the same two steps by hand
docker cp roboracer_stack\tools\rate_monitor.py autodrive_roboracer_api:/tmp/
docker exec -it autodrive_roboracer_api bash -lc "source /opt/ros/humble/setup.bash && python3 /tmp/rate_monitor.py --csv /tmp/rate.csv"
docker cp autodrive_roboracer_api:/tmp/rate.csv .    # afterwards
```

```
t=  42.0 | imu  44.8Hz   p5  38.1 gap   41ms | cmd  39.9Hz   | pp  39.9Hz   p5  38.4 delay   71ms | stalls imu=0 pp=2(+1)
```

| column | what it tells you |
|---|---|
| `imu` | the simulator tick. All sensors ship from one bridge callback, so this *is* the lidar rate, measured with a 40-byte message instead of a 1080-float scan (`--lidar` to confirm rather than assume). |
| `cmd` | our own `steering_command`. Sensors fast but `cmd` slow ⇒ **our** loop is the limiter. Both slow ⇒ the bridge, the machine, or the LAN. |
| `pp` | the pure pursuit loop, and `delay`, its own online round-trip estimate. |
| `p5` | the 5th-percentile rate — sustained jitter. |
| `gap`, `stalls` | the largest single gap, and a count of gaps over twice the median. **These** catch one 200 ms freeze; the mean does not. |

With `--laps`, the CSV also carries `lap_count`, `last_lap_s` and `best_lap_s`.

That last row is the point. A 300 ms freeze moved the mean from 45.0 to 40.6 Hz —
nothing you would notice — while `gap` went to 313 ms and `stalls` ticked to
`1(+1)`. One such freeze per lap is a wall. `ros2 topic hz` reports only the mean,
which is why this took a race admin to spot rather than showing up in testing.
`--` means a topic never published (a name or a QoS fault, not a speed one);
`DEAD` means it published and then stopped.

### Lap times

`--laps` adds a line as each lap closes, with the tick that produced it:

```
LAP   7    6.382 s   best   6.351   tick  44.9 Hz   worst gap   31 ms   stalls 0
LAP   8    7.104 s   best   6.351   tick  41.2 Hz   worst gap  287 ms   stalls 3
```

That is the whole diagnosis on one line: lap 8 lost 0.7 s, and it lost it to
three interruptions, not to the racing line. A closing table repeats every lap of
the session, and flags the best lap that had no stall in it — the one that is
actually representative of the setup rather than of a quiet moment on the
machine.

The per-lap worst gap is accumulated as the lap runs rather than read from the
rolling window: the window is 3 s and a lap is ~6.4 s, so a stall in the first
half would otherwise have aged out before the lap closed.

**The lap topics are restricted** (`common/restricted.py` — they are race
telemetry), so `--laps` is off by default and announces itself when on, exactly
like pure_pursuit's `dev_lap_telemetry`. Use it while testing; leave it off for
anything you intend to quote as a result. The simulator shows lap times in its
own window either way, so an official run loses nothing by running without it.

Without `--laps` the monitor reads no restricted topic and publishes nothing, so
it is race-legal and safe to leave running through a timed lap. If you want a
number without copying anything in at all:

```bash
docker exec -it autodrive_roboracer_api bash -lc \
  "source /opt/ros/humble/setup.bash && ros2 topic hz /autodrive/roboracer_1/imu"
```

### Once you can see the rate

- **Match the loop to the tick.** `RACER_CONTROL_HZ` is an environment variable
  (below), so if `imu` lands at 45–50 Hz, `-e RACER_CONTROL_HZ=50` costs nothing
  and keeps the follower off the critical path.
- **Watch `delay` against the tune.** `cmd_delay` is roughly three sim frames:
  ~175 ms at 17.5 Hz, ~65 ms at 45 Hz. Two things key off it and were both tuned
  at 17.5 Hz — `derate_delay_from: 0.175` gives lateral grip back once the delay
  drops (pure gain), but `lookahead_delay_ref: 0.175` scales the lookahead by
  `cmd_delay / 0.175` clamped to `[0.7, 1.2]`, so a fast machine pins it at 0.7:
  a 30 % shorter lookahead than any measured lap used. Worth a deliberate A/B
  once the rate is known, not a blind edit.

## Changing the configuration without rebuilding

Every knob is an environment variable read by `/home/autodrive_devkit.sh`:

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
  -e RACER_PATH_CSV=/home/autodrive_devkit/install/roboracer_stack/share/roboracer_stack/raceline/raceline_a6.5.csv \
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

`tools/rate_monitor.py` is built to the same rule: by default it subscribes only
to `/imu`, `/left_encoder`, `/lidar`, our own `steering_command` and
`/pure_pursuit/status`, publishes nothing, and touches no lap or collision
counter — so it can be left running through a timed lap. Its `--laps` flag does
read the restricted lap topics; it is off by default and says so when on, and a
lap time measured with it is a development number.

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
