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
