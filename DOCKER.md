# Running the multi-track stack in Docker

No ROS on the host. The image is `autodriveecosystem/autodrive_roboracer_api`
plus nav2 AMCL, slam_toolbox, cyclonedds, scipy and this branch's
`devkit_ws/src` built on top. The devkit bridge comes from the base image, which
is identical to `devkit_ws/src/autodrive_devkit`.

```bash
./scripts/build.sh                    # -> autodrive_racer:multi-track (rebuild after code changes)

# terminal 1: the simulator (pick the scene, then Connect)
SIM_TAG=2026-iros-compete ./scripts/run.sh sim     # default tag: 2026-iros-practice

# terminal 2a: the multi-track race config (M15), log to runs_docker/m16.csv
./scripts/run.sh race m16

# terminal 2b: or exactly the host command, minus "ros2 launch racer_bringup race.launch.py"
./scripts/run.sh racer track:=iros2026 tcp_nodelay:=true loop_hz_cap:=45 log_csv:=run_iros_21.csv

# terminal 3: a second shell in the running container (workspace already sourced)
./scripts/run.sh exec
ros2 topic hz /autodrive/roboracer_1/lidar
```

Then analyse the run on the host (needs only numpy/pandas, no ROS):

```bash
python3 raceline/analyze_run.py runs_docker/run_iros_21.csv \
    --path raceline/iros2026/raceline_tum_iqp_h7.0_a7.0b.csv
```

| what | where |
|---|---|
| relative `log_csv:=` files | `runs_docker/` on the host (mounted at `/root/Documents/roboracer/runs`) |
| racelines | host `raceline/` is mounted live; a new CSV needs no rebuild (`MOUNT_RACELINE=0` uses the baked copy) |
| `path_csv:=` | use the in-container path, e.g. `path_csv:=/root/Documents/roboracer/raceline/iros2026/<line>.csv` |
| RViz | opens on the host display; add `rviz:=false` to skip it |
| a bare shell, nothing started | `./scripts/run.sh shell` |

Code under `devkit_ws/src`, maps, and `tools/` are baked in: rebuild the image
after changing them. `frames.py` resolves assets from `~/Documents/roboracer`,
which is why the tree lives at `/root/Documents/roboracer` in the container.

Only one bridge may own port 4567 (`--network=host`): stop any other racer
container first (`run.sh` removes its own previous one).

## The loop really runs at the cap in here (2026-09-18)

`tcp_nodelay:=true` preloads `tools/libnodelay.so`, whose path comes from
`frames.REPO` = `~/Documents/roboracer`. That directory does not exist on the
development laptop (the checkout has a different name), so on the host the
preload pointed at a missing file and did nothing: every logged run before this
measured a command delay of 0.107 s, about 28 Hz. The image builds the shim and
puts the tree where `frames.REPO` expects it, so in the container the cap binds
-- `loop_hz_cap:=45` measures 0.066 s, exactly 3 frames at 45 Hz.

That is the organizers' rate and the intended behaviour, but it is NOT the rate
the current lines and follower constants were tuned at. When a run in here
behaves differently from a logged host run, check `pp_delay` in the CSV first:

    python3 raceline/analyze_run.py runs_docker/<run>.csv --path raceline/iros2026/<line>.csv

`loop_hz_cap:=28` reproduces the old delay if you need the comparison.

## The multi-track race config (`run.sh race`)

`./scripts/run.sh race [NAME] [extra name:=value ...]` runs what
`experiments/iros2026/TEAM_IROS2026.md` calls the race config -- M15, 56 clean
timed laps, first-30 mean 8.963 s (best 8.92, worst 9.03):

| item | value |
|---|---|
| raceline | `raceline/iros2026/rl_mt_b05b15w05_ell_L65_B50_v9.0.csv` |
| dead reckoning | `distance_source:=slip` |
| `v_max` | 9.0 |
| `enc_rate_window_s` | 0.10 (`ENC_WIN=` to change) |
| `exit_guard_*`, `target_lead_s`, `brake_cap_margin`, `drag_ff` | 0 / off |
| loop | 45 Hz, `warmup_v_max 2.0` for 21 m, hybrid LQR |
| AMCL | `experiments/iros2026/params/amcl_beams360.yaml` |
| log | `runs_docker/NAME.csv` on the host (default NAME `race`) |

The **registry default is not this line**: `frames.py` still points `iros2026` at
`rl_mt_b0.15_a7.0b.csv` at `v_max` 8.5, which M09 measured at 9.10-9.35 s, so
`./scripts/run.sh racer track:=iros2026 ...` alone does not race the fast config.
`race` passes every knob explicitly, exactly as `run_mt.sh` does.

Two things this deliberately does **not** copy from `run_mt.sh`:

- `run_mt.sh` uses `bridge:=false` because it raced against the persistent
  bridge of the dev container (`docker/dev/bridge.sh`). Here the bridge runs in
  the same container, so the 45 Hz pacing comes from `tcp_nodelay:=true
  loop_hz_cap:=45`; `HZ_CAP=` overrides it.
- `experiments/iros2026/scripts/teardown.sh` restarts a container named
  `rr_bridge`, which is the dev container, not this one. Here the equivalent is
  Ctrl-C, or `docker rm -f autodrive_roboracer_api`.

Reset and Connect the simulator **by hand** between runs. Never publish
`/autodrive/reset_command` -- it is a restricted topic.

## Two images, on purpose

| | `Dockerfile` + `scripts/` | `docker/dev/` |
|---|---|---|
| what | the stack baked in, `colcon build` at image build | repo bind-mounted, built at run time |
| for | racing and handing over a fixed artefact | tuning: persistent bridge, experiment sweeps |
| extras | nav2, slam_toolbox, cyclonedds, scipy | also matplotlib, skimage, quadprog, `trajectory_planning_helpers` |
| entry | `scripts/run.sh` | `docker/dev/{bridge,race,experiment}.sh`, `tools/tuning/run_experiment.sh` |

`tools/tuning/report.py` and `wall_spots.py` need matplotlib/skimage, which only
the dev image has; run those on the host or in `docker/dev` after a race.
`tools/tuning/sim_ctl.py` needs only rclpy and works in both.

## Tuning AMCL (`run.sh truth`)

```bash
./scripts/run.sh truth base                              # amcl_beams360.yaml (default)
AMCL_PARAMS=amcl_beams720.yaml ./scripts/run.sh truth b720
AMCL_PARAMS=amcl_alpha3_025.yaml ./scripts/run.sh truth a3
AMCL_PARAMS=my_amcl.yaml ./scripts/run.sh truth mine      # any yaml you drop in that dir
```

`truth` steers on the simulator's ground-truth pose (`drive_on_truth:=true`) on
`rl_mt_tb10_lat875_b55_L70.csv` + `steer_a_lat_max:=8.0` -- `FINDINGS.md` section
11, 8.45 s over 22 clean laps.

**It is not race-legal.** `mode:=race` refuses it, the follower prints the
warning in red, and the line is solved for the 0.09 m worst-case error the
true-pose car has against AMCL's 0.13 m, so driving it *on AMCL* puts it into
the right wall at s 38-39. The race-legal best is 8.92 s on
`rl_mt_lat7.25_hp70_bendz65_L70.csv`; the half-second gap is what localization
costs.

### Why this is the right rig for tuning AMCL

AMCL still runs, and `race.launch.py` hands `instruments.launch.py` the
*unforced* `use_tf`, so `map->odom` is published and its error is logged against
ground truth exactly as in a normal run. Only the follower stops consuming it
(`race.launch.py:312`). The comment at line 174 puts it plainly: *"the localizer
still runs, so its error is still logged beside a car that is not using it."*

That means the car drives the **same line every run** no matter how badly AMCL
is tuned -- no feedback loop where a worse estimate changes the trajectory and
changes the error you are trying to measure. The AMCL parameters become the only
variable, and two runs are directly comparable. Tuning on a car that steers on
AMCL cannot give you that.

Trade-off to keep in mind: it measures AMCL's error along the *true-pose*
trajectory, which is slightly tighter and faster than the one an AMCL-driven car
takes. Confirm a winning parameter set with a normal `./scripts/run.sh race`
before trusting it.

### Reading the result

| what | where |
|---|---|
| localizer error beside ground truth | `runs_docker/NAME.csv` (the `log_csv`) |
| drift plots, position and heading | `tools/localization_drift.sh [SECONDS] [NAME]` while the stack runs |

`localization_drift.sh` records inside the running container (it expects the
`autodrive_roboracer_api` name `run.sh` already uses) and plots on the host from
`.venv-plot/`; it writes `logs/drift/NAME.csv` and figures beside it. It holds
each ground-truth sample and looks the estimate up **at its stamp**, because
AMCL only publishes `map->odom` after processing a scan -- a "latest" lookup
reports the car's own motion as error, about 0.2 m at 8 m/s.

The three shipped parameter sets differ in exactly two fields:

| file | `max_beams` | `alpha3` |
|---|---|---|
| `amcl_beams360.yaml` (race default) | 360 | 0.10 |
| `amcl_beams720.yaml` | 720 | 0.10 |
| `amcl_alpha3_025.yaml` | 180 | 0.25 |

`alpha3` is the odometry rotation noise from translation; `max_beams` is how
much of the scan AMCL matches per update, and costs CPU at 45 Hz.

Any `name:=value` after the run name is passed through and **wins over the
built-in value** (last duplicate wins in `ros2 launch`), so a one-off is
`./scripts/run.sh truth mine v_max:=8.5`. `AMCL_PARAMS`, `V_MAX`, `ENC_WIN`,
`HZ_CAP` and `RVIZ` do the same for the common knobs.

Changing `run.sh`, a raceline or an AMCL yaml needs **no rebuild**: `raceline/`
and `experiments/iros2026/params/` are both bind-mounted over the baked copies,
and `run.sh` runs on the host. Edit the yaml, run again. `MOUNT_PARAMS=0` and
`MOUNT_RACELINE=0` fall back to what is in the image. Only `devkit_ws/src`
changes need `./scripts/build.sh`.
