# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Autonomous racing in the AutoDRIVE RoboRacer simulator (Sim Racing League
2026). Porto is the developed track; the simulator ships several others and the
classical stack is track-scoped (see **Tracks**). Two independent stacks share the workspace and share
nothing else:

- **`rl_racer/`** — SAC policy driving straight off LiDAR. No map, no
  localization. Trained as a staged curriculum where each stage resumes from
  the previous stage's weights.
- **`devkit_ws/` + `raceline/`** — classical stack: SLAM map → minimum-curvature
  raceline → AMCL localization → pure pursuit.

`README.md` holds measured results and lap times; `rl_racer/EXPERIMENTS.md` is
the full training log including bugs found the expensive way.
`raceline/FINDINGS.md` is the classical stack's summary: the tuning levers, the
timing bugs, and the lap-time ceiling (its per-run history is `VEHICLE_MODEL.md`
section 7).

## Environment

Three separate Python environments, and mixing them is the most common failure:

| Context | Python | Why |
|---|---|---|
| devkit bridge, ROS nodes | **system** python3.10 | `rclpy`/`cv_bridge` are C extensions built against it |
| RL training / `enjoy.py` | **`.venv-rl`** | torch + CUDA + stable-baselines3 live only there |
| notebooks (`raceline/`) | `.venv-rl` | needs `trajectory_planning_helpers`, `casadi` |

Order matters when both are needed: source ROS **first**, then the venv — ROS
puts `rclpy` on `PYTHONPATH`, and the venv must win on `PATH`. Conda will not
work at all.

Every ROS terminal:

```bash
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source devkit_ws/install/setup.bash
export CYCLONEDDS_URI=file://$(ros2 pkg prefix racer_common)/share/racer_common/config/cyclonedds.xml
```

or just `source ros_env.sh`, which does all four and warns if the workspace is
unbuilt.

The `CYCLONEDDS_URI` line is not optional for the full stack: CycloneDDS defaults
`MaxAutoParticipantIndex` to 9, so with localhost-only the tenth node fails to
start — and `race.launch.py` is about ten nodes.

## Build

```bash
cd devkit_ws
PYTHONNOUSERSITE=1 colcon build                       # all packages
PYTHONNOUSERSITE=1 colcon build --packages-select racer_localization
```

`PYTHONNOUSERSITE=1` is required — a newer setuptools in `~/.local` breaks any
`ament_python` build against the system `packaging`. Do not run colcon from an
activated venv.

## Running

The simulator is not in this repo (382 MB Unity build, download from AutoDRIVE
releases; expected at `simulator_practice/autodrive_simulator/`). Start it
first, in every workflow.

```bash
# classical stack — bridge (TF remapped) + chassis + localizer + pure pursuit
ros2 launch racer_bringup race.launch.py                      # AMCL
ros2 launch racer_bringup race.launch.py localizer:=slam      # slam_toolbox
ros2 launch racer_bringup race.launch.py localizer:=v2        # segmented scan-to-map (localization_v2)
ros2 launch racer_bringup race.launch.py v2_shadow:=true      # AMCL owns TF, v2 logged beside it
ros2 launch racer_bringup race.launch.py mode:=race           # nothing restricted
ros2 launch racer_bringup race.launch.py bridge:=false lookahead_k:=0.70

# mapping
ros2 launch racer_mapping mapping.launch.py     # save via the RViz SlamToolbox panel

# RL: bridge in a system-python terminal, policy in the venv
ros2 launch autodrive_roboracer bringup_headless.launch.py    # wait for "Connected!"
cd rl_racer && python enjoy.py runs/stage3_v3/checkpoints/sac_990000_steps.zip --episodes 1
```

`race.launch.py` (in `racer_bringup`, the only package that composes others)
starts `bridge` + `chassis` + the localizer named by `localizer:=` + `follower`,
plus `instruments` and one RViz. Each is launchable alone, and each piece can be
switched off (`bridge:=`, `chassis:=`, `localization:=`, `follower:=`).
`localizer:=` picks `amcl` (nav2 against the track's `track_clean.pgm`), `slam`
(slam_toolbox against its `track_sm` pose graph), `v2` (`localization_v2`,
segmented scan-to-map against the same grid; see **Localizer v2**), or `none`.
`track:=` picks which track's map, raceline and spawn to use; see **Tracks**.
`mode:=race` forces every legal setting at once and omits `instruments`
entirely; `mode:=dev` (default) keeps the development conveniences and every
node reading a restricted topic says so at startup. Pure-pursuit tunables
(`lookahead_*`, `v_max`, `a_lat_max`, `throttle_max`, `steering_gain`) pass
through both launch files, so tuning never needs a rebuild.

## RL training

```bash
cd rl_racer
python3 probe.py                    # ALWAYS first: verifies plumbing, tick rate, reset, collisions
python verify_env.py                # confirms each import resolves to the venv, not ~/.local or ROS

./run_train.sh --stage 2 --gradient-steps 4 --resume runs/v3/checkpoints/sac_370000_steps.zip
tensorboard --logdir runs/
```

`run_train.sh` sources ROS, activates the venv, and refuses to start unless the
sim, the bridge on port 4567, and CUDA are all up. It `exec`s `train_sac.py`, so
any `train_sac.py` flag passes straight through.

Validation gate between stages — never promote an unmeasured checkpoint:

```bash
python enjoy.py runs/<stage>/checkpoints/<ckpt>.zip --speed encoder --episodes 2
```

Add `--legacy-obs` to load a v1–v3 checkpoint (it restores the old odom speed
sign; runtime override only, `config.py` is untouched).

Pass = zero crashes (`end=timeout`) and lap time no worse than the previous
stage. Lap time is `steps × control_period / laps`; never compare lap *counts*
across different control rates.

There is no test suite. The only `colcon test` targets are the third-party
devkit's flake8/pep257/copyright linters.

## Architecture

### RL side (`rl_racer/`)

- `rl_racer/config.py` — **all tunables**, as dataclasses. Reward weights are
  the file you actually iterate on. Every constant carries the measurement that
  justifies it; read the comment before changing a number.
- `rl_racer/obs.py` — LiDAR FOV crop + min-pool + observation assembly. Kept
  separate so a future gym sim can produce byte-identical observations.
- `rl_racer/env.py` — Gymnasium env over ROS 2. The sim is free-running, so
  `step()` blocks on the LiDAR topic: **the scan is the master clock**, since
  the bridge publishes every topic together inside one socket.io handler.
- `stages/stage*.py` — each stage is a `NAME`/`RESUME_FROM`/`DEFAULTS`/`apply(cfg)`
  module that mutates the config. Stage N's `apply()` calls stage N-1's, so
  stages compose rather than duplicate.

Invariants that break checkpoints if violated:

- **Every stage emits the same 95-dim observation** (90 min-pooled beams +
  speed, yaw_rate, prev_steer, prev_throttle, throttle_cap). Change `n_beams`
  or `fov_half_deg` and every existing checkpoint becomes unloadable.
- **The throttle cap is in the observation.** Raising it changes what
  `action[1] = +1` physically means; feeding the cap in is what lets the critic
  distinguish old replay-buffer transitions from new ones.
- **`gamma` is derived** from the measured control period
  (`1 - dt/horizon_seconds`), not hard-coded, so the planning horizon stays
  fixed in seconds across machines. The printed value differs per machine on
  purpose.
- **Checkpoint numbering is cumulative** across stages (`reset_num_timesteps=False`):
  stage 2 resuming at 370k produces `sac_380000_steps.zip`, not `sac_10000`.
- `max_episode_steps` is per stage and is a *step* count, so it means different
  driving time at different control rates.

### Classical side (`devkit_ws/src/`)

One rule holds the layout together: **only `racer_bringup` composes.** Every
other package launches its own subsystem and nothing else. Before that rule,
`race.launch.py` lived in `racer_control` and included the AMCL launch file by
name, so the A/B could not be run from the top-level entry point at all — and
the shared half of the two localizers was copy-pasted into both.

- `autodrive_devkit/` — third-party bridge, BSD, **unmodified**. When its
  behaviour needs changing, do it with a launch-level remap in `racer_bringup`
  (see `bridge.launch.py`), never by editing it.
- `racer_common/` — no nodes. TF frame names, the lidar extrinsic, the spawn
  pose, workspace paths (`frames.py`), and the competition restricted-topic list
  with its runtime warning (`restricted.py`). Also owns `cyclonedds.xml`.
  Everything else imports from here instead of repeating literals.
- `racer_localization/` — `dead_reckoning` (encoders + IMU → `odom→roboracer_1`,
  heading and position carried forward to each stamp, `VEHICLE_MODEL.md` §5.4),
  `localization_bootstrap`, `localization_error`, and the three interchangeable
  localizers. `chassis.launch.py` is the half they share; `amcl.launch.py`,
  `slam.launch.py` and `v2.launch.py` contain *only* their localizer;
  `instruments.launch.py` holds every node that reads a restricted topic, so
  legality is one inclusion.

  **Localizer v2** (`localization_v2.py`, estimator `v2_filter.py`, matcher
  `scan_matcher.py`; only the node needs ROS). Built 2026-09-19. **Measured on
  the car: 19 clean laps over two machines, mean 8.93 s on the race line, zero
  contacts** — AMCL's lap time with better error and 20× smoother corrections
  (against AMCL on identical scans: along p90 0.301 vs 0.334, cross 0.027 vs
  0.042, worst per-sample correction step **21 mm vs 460 mm**). See
  `raceline/FINDINGS.md` §12 for the runs and every bug found getting there.

  Why it exists: AMCL's cross-track error is 1.6 cm, already the geometric
  floor, but its along-track error is p90 0.32–0.51 m, built on the long
  straight where the scan carries **zero** along-track information (a 0.30 m
  offset changes the scan cost by 1.3 mm against a 5.5 mm noise floor), and it
  jumps `map→odom` by up to 0.46 m in one sample there against a 0.09–0.13 m
  wall margin. It also wandered `map→odom` yaw by hundreds of degrees on a
  transform that is structurally zero (dead_reckoning carries the IMU's
  absolute quaternion; `log_localization`'s `m2o_yaw_deg` is the invariant).

  v2 solves **translation only** per scan by Gauss–Newton on the grid's signed
  distance field, **clamps each axis of the innovation** against the filter's
  own sigma (never drops it), fuses the 2×2 information matrix, and estimates
  an **odometry scale-error state** — unobservable on the straight, observable
  at every corner that pins the along axis — which it spends across the blind
  stretch. Dead reckoning over-reads 2.0–3.4 %, worth 0.3–0.5 m over the 16 m
  straight; the state halves the along error and is bounded at the measured
  3.0 % because it otherwise runs to whatever bound it is given.

  `tools/segment_track.py --track <t> [--bench]` computes
  `raceline/<t>/segments.csv` from the map and benches the matcher. The table
  is a **label**, not a controller: both of its levers (per-mode gain, per-mode
  rate limit) were measured to be stale proxies for what the live information
  matrix already knows, and each caused a crash before being removed. It earns
  its place through `analyze_run.py`'s per-mode breakdown, which is how those
  bugs were found. `blind_along_info` is the real floor.

  Interfaces are AMCL-shaped (`/initialpose`, the two `Empty` services — global
  search is a 2-D sweep at the IMU yaw — `/map` from the node), so the
  bootstrap and reset recovery work unchanged; recovery was confirmed live.
  It starts on the track's registered spawn as a fallback prior, as
  slam_toolbox does: publishing `map→odom` only once seeded **deadlocks** the
  bootstrap, whose `ready_check:=tf` waits for that transform before seeding.

  `v2_shadow:=true` runs it beside AMCL without TF, `log_localization` writes
  `v2_*` columns, and `scan_dump:=x.npz` records every scan so
  `tools/replay_localization_v2.py x.npz [--set k=v]` re-runs the SAME filter
  offline in seconds — tune there, not in the sim.
  (`tools/synth_v2_dataset.py` makes a sim-free dataset.)

  Traps, all found the expensive way: the matcher's information is a beam
  count, so `beam_sigma_m` must turn it into 1/m² before it meets `P⁻¹`; a
  fixed whole-step gate rejects a perfect match whose *unobservable* component
  is large and takes the good component with it; and the distance transform
  must be signed to the wall **face**, not zero at occupied cell centres.

- `racer_control/` — `pure_pursuit`, `calibrate_steering`. Control only; the
  name is now true. `pure_pursuit` estimates the car's speed by running the
  sim's own tire curve on the measured wheel speed (`speed_source: tire`, a
  contracting observer with bounded error, p90 0.15–0.22 m/s against ground
  truth) and commands throttle as a wheel speed inside a ±0.08 slip band placed
  one measured round trip ahead, 175 ms here (`throttle_mode: slip`,
  `cmd_delay_auto`), because in this sim the
  encoders report the throttle command, not the car, and the command reaches
  the wheel one bridge round trip late (`raceline/VEHICLE_MODEL.md` §3.1, §3.6).
  It also carries a **warmup speed cap** (`warmup_v_max`, `warmup_dist_m`, off
unless the track's registry row sets it): the speed target is held down over
the first N metres of driving, measured from the encoders. The first lap is a
warmup and the timer starts after it, so this costs no race time. It exists
because the encoders report the throttle command, so dead reckoning over-reads
distance by 2-5 % while the wheel slips, and a long straight gives AMCL almost
no along-track correction, its walls being parallel to the error. On IROS 2026,
whose spawn is at the top of a 16 m straight, a run started from the spawn
carried +0.43 to +1.01 m of along-track error into the hairpin-1 braking point
(runs 3, 13, 14 -- 14 then hit); the same stack started mid-track, so the filter
meets a corner first, stayed under 0.10 m and reached +0.32 m on that straight
later in the run (run 18). Release where the profile is already slower than the
cap, or the lift is a speed step. `encoder` / `legacy` keep the old law for A/B; `fused` (IMU + pose) is kept
  for reference and does not work here, see §5.3. It publishes `~/status` every tick; `log_localization` records it as
  `pp_*` columns and `raceline/analyze_run.py run.csv` turns a log into lap
  times, tracking error, corner bias, weave, estimator error and encoder ratio.
- `racer_mapping/` — slam_toolbox mapping config, `map_publisher`, and the
  committed maps, one directory per track (`maps/porto/`). Scan matching is **on** (karto only adds graph vertices
  inside that branch, so a map built without it is unusable for localization);
  loop closure is on too, because the scan matcher's per-node slop integrates —
  measured +0.023 m after one lap, +0.750 m after three.
- `racer_bringup/` — `race.launch.py`, `bridge.launch.py`, the RViz configs.
  The composition root, and the only package that depends on the others.

TF tree under localization: `map →(amcl | slam | v2) odom →(dead_reckoning) roboracer_1
→(static) lidar`. The devkit broadcasts `world→roboracer_1` from the IPS; if
that stays on `/tf`, `roboracer_1` gets two parents and the tree breaks — hence
the remap to `/tf_ground_truth`.

### Raceline (`raceline/`)

`optimize_raceline.py` is the pipeline: map → centerline → minimum-curvature
line → velocity profile → CSVs (`s,x,y,psi,kappa,w_r,w_l[,v_mps]`) that
`pure_pursuit` consumes. Run it from `.venv-rl`, with `--track` choosing which
`maps/<track>/` it reads and which `raceline/<track>/` it writes. It rebuilds the
centerline when the CSV is missing and exports a ladder (`raceline_a4.0.csv`
through `a7.0.csv`): the same geometry at several lateral limits, so grip is
stepped up on the car instead of guessed. Geometry does not depend on grip, only
the velocity profile does. The notebooks import from it for plots;
`make_speed_variants.py` re-profiles any other line under the same physics.
`line_editor/` is the interactive version (`python -m line_editor --track ...`
from `raceline/` in `.venv-rl`): a page where the line is dragged on the map and
the speed shaped with the same zone flags or by hand, saving a CSV in the
follower's layout plus an `.edit.json` with the arguments and the hand edits.
Its `README.md` has the details and the Selenium test.

## Tracks

The simulator ships several tracks (Porto, Berlin, the SRL 2024/2025 scenes),
selected from its in-sim menu, so **nothing may assume Porto**. Every track
asset is scoped by name:

| asset | path |
|---|---|
| occupancy grid, pose graph | `devkit_ws/src/racer_mapping/maps/<track>/` |
| centreline, raceline ladder | `raceline/<track>/` |
| spawn, default line, margin zones, lateral zones | `racer_common.frames.TRACKS['<track>']` |

`racer_common/frames.py` is the registry and the only place that resolves a
track to paths (`map_yaml()`, `pose_graph()`, `raceline()`, `spawn()`). Pick one
with `track:=<name>` on any launch file, or `RACER_TRACK=<name>` in the
environment; naming a `path_csv:=`/`map_yaml:=` explicitly still overrides.

**What is per track and what is not.** The car model is not: the tire curves,
the 25.25 m/s per unit throttle, the command-delay handling and the whole
controller are derived from the simulator's physics and transfer unchanged.
What does not transfer is exactly what the registry holds — the spawn, the grip
rung measured safe, and the **margin zones**, which are hand-placed at the
s-ranges where *that* track's tracking error lands. Reusing another track's
margin zones pushes the line toward a wall rather than away from one.

Adding a track:

```bash
# 1. select the track in the simulator's menu, start the bridge, then map it
ros2 launch racer_mapping mapping.launch.py     # drive a few laps, save via RViz
# 2. put the grid and pose graph in maps/<track>/ as track_clean.* / track_sm.*
# 3. read the spawn the car actually starts at -- RIGHT AFTER A RESET, before
#    driving: after the mapping laps /ips is wherever you parked, not the spawn
ros2 topic echo /autodrive/roboracer_1/ips --once
# 4. add a row to racer_common.frames.TRACKS, then build the lines
cd raceline && python optimize_raceline.py --track <track> --ladder 4.0,4.5,5.0,5.5,6.0
# 5. step the ladder up on the car, lowest rung first, and read every run
ros2 launch racer_bringup race.launch.py track:=<track> \
    path_csv:=$PWD/raceline/<track>/raceline_a4.0.csv log_csv:=run.csv
python3 raceline/analyze_run.py run.csv --path raceline/<track>/raceline_a4.0.csv
```

Only once a rung runs clean do the margin zones get placed, from where
`analyze_run.py` reports the corner bias and the wall contacts, not guessed.
The same goes for **lateral zones** (`--lat-zones s0:s1:a_lat`, exported as the
`z` lines): a per-corner grip limit for corners so tight that the rung's
lateral demand leaves no headroom for the delay's speed error. On ICRA 2026
the two hairpins (curvature 1.37) hit twice in 46 laps at the 6.5 rung with
8 % of headroom; capped at 6.0 they keep 14 %, the rest of the lap keeps 6.5.
`--v-zones s0:s1:v_max` is the same idea for the SPEED ceiling, which is the
only limit that binds on a straight (kappa is ~0 there, so no lateral limit is
active and the profile simply runs to v_max). The zone's value applies inside
the range and `--v-max` outside it; the pipeline solves the base profile at the
highest ceiling in play and then re-imposes longitudinal feasibility, so the
braking out of a fast zone is real rather than a step. Exported as the `v`
lines. Use it to let a genuine straight run fast while a corner approach stays
capped: on ICRA 2026 a global 8 m/s bought 0.03 s and made the approach to the
middle-wall hairpin fast enough to cause a hit, while 8 m/s on the main
straight alone predicts 0.085 s with the approach untouched.

**Two grids per track when the walls are hollow.** Tracks built from ducting
map as two thin faces with an unknown strip between, and rays enter the
hollow at the ducts' open ends. `track_clean.pgm` is the **sensor map**: the
world as the LiDAR sees it, faces, gaps and all, and it is what AMCL
localizes against, because that is what its rays will return at race time.
If that grid has openings the LiDAR sees through but the car cannot drive
through, the raceline needs a **geometry map**, `track_solid.pgm`, with them
sealed; `optimize_raceline.py` prefers it automatically when it exists. On the
ICRA 2026 track it took capping the two duct ends by hand (GIMP, then export
to PGM, then the cleaner over the result). Never give AMCL the solid map.

The centreline extractor orders the skeleton ring by BFS to the antipode and
back (`_order_loop`), not by a greedy neighbour walk. The greedy walk strands
itself at the thick spots a skeleton keeps after pruning and then closes the
partial path through walls; a near-oval like Porto never triggered it, a
switchback did, dropping a third of the lap.

Every number about the car is in `raceline/VEHICLE_MODEL.md`, derived from the
simulator's Unity source, with references. The three that change decisions:
throttle commands a wheel speed (25.25 m/s per unit), so encoders read the
command, not the car; the tire's robust limits are the friction-curve
asymptotes, 4.55 m/s² longitudinal and 4.90 lateral, and demands between the
asymptote and the peak are what makes the car drift; drag is linear, 0.273·v.
Read it before changing a limit.

## Competition legality

`/ips`, `/odom`, `/tf`, and all lap and collision telemetry are **restricted**.
Legal inputs: LiDAR, camera, IMU, wheel encoders, steering/throttle feedback.

But restriction is about **when and how often**, not only which topic. The first
lap is a **warmup** and the timer starts after it, so those topics are readable
up to that point. What is never acceptable is a node that keeps reading ground
truth into the timed laps. Two access patterns, two functions in
`racer_common/restricted.py`:

| | pattern | legal | function |
|---|---|---|---|
| seed | one read, before the car moves, then the subscription is **destroyed** | yes | `restricted.seed()` + `restricted.released()` |
| stream | a continuous subscription | no | `restricted.warn()` (red) |

Keep this separation intact when adding code:

- The RL policy reads LiDAR + encoders only. `/odom` feeds the **progress
  reward** and `collision_count`/`reset_command` feed **episode management** —
  training-time only, none of it exists at inference.
- The classical stack localizes with LiDAR + IMU + encoders against the
  pre-built map. Ground truth appears in exactly two places: the one-shot pose
  seed (`bootstrap_mode:=truth`, **race-legal**, released by
  `_release_truth()` as soon as it resolves) and `localization_error`
  (continuous, development-only, omitted by `mode:=race`).
- Anything reading a restricted topic **continuously** must (a) live in
  `racer_localization/launch/instruments.launch.py`, which `mode:=race` omits
  wholesale, and (b) call `racer_common.restricted.warn()` so it announces itself
  at startup. A comment is not enough: the boundary used to be 33 comments and no
  checks, and `pure_pursuit`'s `pose_topic` DEFAULTED to ground-truth `/odom`, so
  development runs produced lap times that read as race-legal and were not.

**Recovery after a wall contact** (`localization_bootstrap`, `recover:=true`,
the default). The simulator resets a hit car to the last checkpoint and zeroes
its velocity; the rules add 10 s per contact and say localization "will have
to be robust against this re-setting action". Before this, dead reckoning kept
integrating from the old pose, AMCL's particles no longer explained the scan and
the follower drove blind (runs 14 and 24). Now the bootstrap stays alive after
the first handover and watches two legal signatures, the encoder speed
collapsing to zero in one sample and an IMU heading step the yaw rate cannot
explain (10 deg; on IROS 2026 at 45 Hz the encoder did not collapse through
either reset of run 14, it kept reading the command, so the heading step is
the only signature there). On either it drops `/localization_ready` (the follower stops and
publishes nothing), re-seeds AMCL at the **checkpoint behind the last trusted
pose** with the IMU heading, confirms the estimate against the seed, creeps for
a second on lidar so the filter tightens on motion, and latches ready again;
if the prior is not adopted it falls back to the global search, and if that
times out it resumes on the current estimate rather than park for the rest of
the race. The checkpoints are measured, not guessed: `frames.TRACKS[...]
['checkpoints']` holds the reset poses seen in logged runs (three on ICRA 2026,
repeatable to the centimetre, on the centreline), ordered along the lap by the
centreline's arc length, and the bootstrap prints any reset it cannot match so
the list can grow. Seeds are built from the yaw as planar quaternions: nav2
rejects a copied IMU quaternion whose norm is off by 1e-4, which the bridge's
rounded components produce intermittently ("malformed" re-seeds, run 24). The
logger records `ready`, and `analyze_run.py` reports resets, time to
re-confirmation and the error afterwards.

Two things about that recovery are worth keeping in mind, both learned the
expensive way on IROS 2026. The centreline extractor does not orient the ring,
so on some tracks its `s` runs AGAINST the lap (Porto and ICRA run with it,
IROS against); the bootstrap reverses it against the spawn heading, and
reversing the values without the order left the lap length at 0 and killed the
node with a ZeroDivisionError on the first reset of every run -- run 20, nine
resets, none recovered, and `ready` never dropped because nothing was left
alive to publish it. And the simulator always resets BACKWARD to a checkpoint
already passed, but a car cutting inside a tight corner PROJECTS onto the
centreline short of the progress it has made, by -1.04 to +1.50 m over the six
logged contacts, so `BACK_SLOP_M` lets a checkpoint sit up to 1.20 m 'ahead'
and still count as behind (the window that fits all six is 1.05-1.35).
`racer_localization/tools/check_recovery_prior.py` runs that selection against
the real centreline and every logged reset, calling the node's own helper --
the earlier check re-implemented the arithmetic and passed while the node
crashed. Run it after touching the centreline or the checkpoint list.

`mode:=race` deliberately does **not** force `bootstrap_mode:=global` any more.
It used to, and that was a bug rather than caution: slam_toolbox has no global
relocalization at all, so `mode:=race localizer:=slam` fell through to the
hardcoded `frames.SPAWN_*` — and slam seeds with a ±0.5 m correlative search, so
a wrong constant could never be recovered and stayed frozen for the whole timed
run. The warmup seed is both legal and the only thing that makes that
combination work. `SPAWN_*` is now only the fallback for `bootstrap:=false` and
`bootstrap_mode:=global`; the bootstrap prints the measured spawn whenever the
constant disagrees with it.

## Measured simulator constants

Several contradict the published documentation. Trust the measurements — they
were re-derived at cost.

| quantity | measured | documented |
|---|---|---|
| Encoder units | **radians** (wheel angle) | "ticks, 1920/rev" — off by ~300× |
| Wheel radius | **0.0581 m** | 0.0590 m |
| Sim tick rate | **18.6 Hz** with the stock bridge, **77 Hz** with `tcp_nodelay:=true`; not a clock but a round trip, see below | bridge advertises 40 Hz |
| Speed vs throttle | **≈ 24 × throttle** | 22.88 m/s top speed |
| `twist.linear` frame | **body**, not world | unstated |
| `/imu` vs `/odom` angular | identical — one source | unstated |
| Scan size | **1081** beams (inclusive endpoints) | 1080 |

`LidarFOV` derives its crop indices from the live scan header rather than
hard-coding them, precisely because of that last row.

The tick rate is not a constant in either program. The simulator emits
telemetry only in reply to the bridge's message (`Socket.cs`, `OnBridge` ->
`EmitTelemetry`) and the bridge publishes and replies inside its handler, so
the loop is a round trip, and it runs at whichever is slower of the
simulator's frame and a TCP deadlock: the simulator's ~13 KB telemetry is one
sub-MSS segment on loopback (MTU 65536), Nagle holds it until our side
acknowledges, and Linux delays that acknowledgment up to 40 ms, so the cycle
is ~50 ms on any machine whose frame is shorter than that. Measured
2026-09-12 on this laptop (HUD at 144 fps), same session, loopback: stock
bridge 18.6 Hz on the lidar topic; `tcp_nodelay:=true` 77.3 Hz (median
12.8 ms, max 25); the ideal replier in `tools/sim_rate_probe.py` 19.3 ->
101.6 Hz; `sudo ip link set lo mtu 1500` alone 107 Hz (full-size segments
Nagle does not hold); the same simulator replied to by a second machine over
a LAN cable, 63.6 Hz. The earlier reading that the frame period was the limit
here was wrong. `tcp_nodelay:=true` preloads `tools/libnodelay.so` into the
bridge: NODELAY on its sockets and QUICKACK re-armed after every recv AND
every send syscall. The send half is what matters: Linux re-enters delayed-ACK
mode on a reply sent right after a receive, and a Python-level re-arm after
`emit` lands before the actual write and does nothing (measured, 19.4 Hz). It
stays OFF by default: every follower constant was tuned at a 175 ms command
delay, and the first run with it on (run 24, hand-edited line) measured a far
shorter delay and hit. Validated at `loop_hz_cap:=45` on IROS 2026
(2026-09-17, runs 14-16): the follower needs no retuning there, but the
encoder rate did (`enc_rate_window_s`, see pure_pursuit.py): a per-sample
rate at 22 ms scattered 2x and the tire observer ran 0.4 m/s low, 0.55 s a
lap (run 15); with the rate spanning 50 ms, run 16 did 8 clean laps at
9.95-10.00 against 10.00 at 18 Hz with tracking p90 0.12 m against 0.19.
Two follower changes tried at the fast loop were worse and are off:
`steer_excess_rad` (a slip-angle limiter; it throttles turn-in and the car
runs 0.3 m wide out of the hairpins) and `lookahead_min:=0.6`. The
organizers say the evaluation machine runs 40-50 Hz. `loop_hz_cap:=45` (with
`tcp_nodelay:=true`) paces the bridge's replies so the loop runs at that rate
here; the simulator only emits in reply, so the cap holds the whole loop. The
devkit is untouched; it is the process's environment. The `.so` must exist in the container (`gcc -shared
-fPIC -O2 -o tools/libnodelay.so tools/nodelay.c -ldl`).
`tools/sim_rate_probe.py [--bind=127.0.0.1]` measures the loop with an ideal
replier and no ROS (stop the bridge, run it, press Connect); with
`LD_PRELOAD=tools/libnodelay.so` in front it measures the shim.

## Gotchas

- **Subscriber QoS must match the bridge exactly** (RELIABLE / VOLATILE /
  KEEP_LAST / depth 1). A mismatch connects and then silently delivers nothing.
- **`reset_command` is level-triggered** — the bridge re-emits it every tick, so
  it must be pulsed True→False or the sim resets forever. `env.py` handles this.
- **Nothing before `learning_starts` means anything** (5k fresh, or `--warmup`
  on resume) when reading training curves.
- **`.gitignore` re-admits specific run files** after excluding `runs/*`
  wholesale; pattern order matters, since git cannot re-include a file whose
  parent directory is excluded. Replay buffers (~29 GB) must never be committed.
- Torch in the venv is a CUDA build, but `enjoy.py` and the env are happy on
  CPU — at 18 Hz the per-step budget is 55 ms and SAC updates take single-digit
  ms.
