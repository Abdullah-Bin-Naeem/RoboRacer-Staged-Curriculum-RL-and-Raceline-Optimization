# Localizer: how it works, what was fixed, how to measure it

Reviewed 2026-09-15; bugs fixed 2026-09-16 on branch `iros_compete_usman`.
Status of every item is at the bottom. Line numbers are not used: the files
changed.

## How it works

Three pieces build one chain of frames:

```
map ──► odom ──► roboracer_1 ──► lidar
 AMCL           dead reckoning    static offset (0.2733 m ahead)
 (correction)   (drifts)
```

1. **Dead reckoning** (`roboracer_stack/roboracer_stack/localization/dead_reckoning.py`)
   publishes `odom -> roboracer_1`. Travel comes from the cumulative
   wheel-encoder angle (or, with `distance_source: tire`, from the tire
   observer's car speed). Heading comes from the IMU's absolute yaw, so heading
   does not drift but position does. Travel is banked once per simulator frame
   along the mean heading of the frame interval. The transform is republished
   at 200 Hz, stamped 20 ms ahead and extrapolated to that stamp.
2. **AMCL** (nav2, `roboracer_stack/config/amcl.yaml`) publishes `map -> odom`.
   It keeps 500 to 8000 particles, moves them by the dead-reckoning delta plus
   noise (`alpha1..4`), scores each against the map with the lidar, and
   resamples.
3. **Bootstrap** (`roboracer_stack/roboracer_stack/localization/bootstrap.py`)
   waits for AMCL to be active, seeds `/initialpose` from the measured spawn
   (`common/frames.py` `SPAWN_*`) plus the IMU heading, confirms AMCL adopted it,
   scores the lidar scan against the map at that pose, then latches
   `/localization_ready`. After that it stays up as a **respawn watch**.

Pure pursuit waits for `/localization_ready`, then on every control tick
combines the newest sample of each TF leg (`compose_latest` in
`control/pure_pursuit.py`). AMCL post-dates `map -> odom` by 0.5 s, so a normal
chain lookup would return a stale correction.

## Timing

Every node is its own process, and they talk over DDS (cyclonedds, localhost),
so they run in parallel.

| stage | what triggers it | rate | notes |
|---|---|---|---|
| Simulator -> bridge | one WebSocket message per sim frame | ~18-20 Hz measured here (19.8 headless); drops to ~13 Hz under load | the bridge handler decodes the frame, publishes all topics in one burst (throttle/steering feedback, left encoder, right encoder, IPS, IMU, odom, TF, lidar, camera, lap data), then replies with the latest throttle/steering command |
| message stamps | `get_clock().now()` in the bridge at publish | per frame | receipt time, so WebSocket and Unity jitter is in every stamp |
| dead reckoning: integrate | a complete frame (every live encoder reported, plus its IMU or 20 ms) | once per sim frame | checked on the 200 Hz timer |
| dead reckoning: publish `odom -> roboracer_1` | 200 Hz timer | 200 Hz | stamped now + 20 ms, extrapolated with yaw rate and speed |
| AMCL: update | each lidar scan, if moved >= 0.10 m or 0.10 rad | = sim frame rate while driving (0.44 m per frame at 8 m/s) | looks up `odom -> roboracer_1` at the scan stamp |
| AMCL: publish `map -> odom` | each scan (fresh after an update, otherwise re-sent) | = sim frame rate | stamped scan + 0.5 s |
| pure pursuit: control | own timer, retuned every second to the measured sim tick | clamped 20-50 Hz (entrypoint seeds 40); 20 Hz on an 18-20 Hz sim | TF arrives on the same single-threaded executor; each tick reads the newest sample of both legs |
| command -> car | bridge sends the latest command in its reply to the NEXT frame | once per frame | measured throttle-to-wheel delay ~175 ms (3 frames at 17 Hz); pure pursuit steers on its pose propagated 50 ms ahead |

## How much lap time does the localizer cost?

Race the same image twice in dev mode. The only difference is the pose source;
speed comes from the wheel encoders in both.

```bash
./scripts/build.sh                    # rebuild so the image has the current code, map and raceline
./scripts/run.sh sim --headless       # terminal 1

# terminal 2, run A: the real stack (AMCL)
docker run --name autodrive_roboracer_api --rm -it --network=host --ipc=host \
  -e RACER_MODE=dev \
  autodrive_racer:qualification-1

# run A2: same, dead reckoning on the tire observer
#   -e RACER_EXTRA_ARGS="dr_distance_source:=tire"

# terminal 2, run B: ground-truth pose, localizer bypassed
docker run --name autodrive_roboracer_api --rm -it --network=host --ipc=host \
  -e RACER_MODE=dev \
  -e RACER_EXTRA_ARGS="localizer:=none use_tf_pose:=false pose_topic:=/autodrive/roboracer_1/odom" \
  autodrive_racer:qualification-1
```

Both print `lap N: X.XX s` in dev mode:

```bash
docker exec autodrive_roboracer_api bash -c 'grep -h "lap [0-9]*:" /home/autodrive_devkit/log/racer_*.log'
```

Run B needs nothing removed before submission, because it is launch arguments
only. `race.launch.py` refuses `localizer:=none` in race mode, which the
entrypoint uses by default. Caveat: `/odom` arrives once per sim frame and is not
extrapolated like the TF path, so B has slightly more latency than a perfect
localizer would.

Result 2026-09-16 (user): run B's lap time matched the localizer's, so the
localizer was not costing measurable time before these fixes.

## Wheel slip and the IMU

**The encoders do not measure the car.** The sim spins each wheel to its
commanded speed within a millisecond, so the encoders report the throttle: 5-12x
high from rest, low while braking (`raceline/VEHICLE_MODEL.md` on `main`).
Pure pursuit's `throttle_mode: slip` bounds that. It commands the wheel no more
than +11 % / -8 % from the estimated car speed, so while accelerating or braking
hard the encoders over- or under-read by up to that much. Braking from 8 to
3 m/s travels about 5.5 m, and the wheels under-read that by up to ~0.4 m.

**AMCL is weakest exactly there.** On a long straight both walls are parallel,
so the lidar cannot tell along-track position until features at the end of
the straight come within `laser_max_range` (9.5 m).

**Why not double-integrate the IMU?** It was tried, as pure pursuit's
`speed_source: fused`, and failed twice. The sim's IMU acceleration is a 1 ms
sample of a tire force that jumps every control tick: measured +6.3 m/s² at
launch against a true 4.0, and -7.6 in a corner against -3.6. Integrating that
once gives a drifting speed; integrating twice, the position error grows with
t². The IMU is still used where it is exact: absolute heading, yaw rate
(extrapolation), and the respawn signature.

**What does work: the tire observer** (`common/tire_model.py`). It runs the
sim's own longitudinal tire model on the measured wheel speed and recovers the
car's speed. The error is bounded and cannot drift; offline against ground truth
it measured p90 0.17-0.28 m/s. `dr_distance_source:=tire` makes dead reckoning
integrate that speed instead of the wheel angle. It is OFF by default until
it is measured in the sim (run A2 above).

Offline result (`tools/localizer_tests/test_dead_reckoning.py`, a simulated
car driven by the slip band, 18 Hz, 15 % stamp jitter), odom error at the scan
stamps AMCL uses, one lap, no AMCL correction:

| distance source | mean | p95 | end of lap |
|---|---|---|---|
| encoder, old code | 0.34 m | 0.73 m | 0.24 m |
| encoder, new code | 0.38 m | 0.68 m | 0.26 m |
| tire, new code | 0.07 m | 0.15 m | 0.12 m |
| tire, with the simulated car's tire deliberately different (rise slope 2.5 vs 3.0) | 0.09 m | 0.16 m | 0.13 m |

That test's car follows the same tire model the observer uses, so it is an
upper bound on the benefit. The real test is run A2.

## Bugs: fixed

### 1. Travel moved along the newest heading, not the interval's mean (FIXED)
Each encoder delta covers frame k-1 to k but was applied along frame k's yaw, so
every chord of a corner rotated the same way. Now it uses the mean of the
(unwrapped) IMU yaw at both frames. The IMU for a frame arrives just after its
encoders, so a complete frame waits up to `imu_wait_s` (20 ms) for it.

### 2. Spawn-mode confirmation compared AMCL with its own seed (FIXED)
Added: the scan is scored against the map at the confirmed pose. The score is
the fraction of beams (every 4th, < 6 m) ending within 10 cm of an occupied
cell. The node refuses to hand over below `scan_match_min` (0.5). It is skipped
(with a warning) only if no map or scan has arrived.

Calibrated by ray-casting `track_clean` at the spawn and 16 raceline poses
(`tools/localizer_tests/test_bootstrap.py`):

| pose error | score min / median / max | below 0.5 |
|---|---|---|
| none | 1.00 / 1.00 / 1.00 | 0 % |
| lateral 0.10 m | 0.75 / 1.00 / 1.00 | 0 % |
| lateral 0.20 m | 0.00 / 0.11 / 0.84 | 82 % |
| lateral 0.30 m | 0.00 / 0.08 / 0.72 | 91 % |
| along-track 0.50 m | 0.05 / 0.91 / 1.00 | 18 % |
| yaw 5 deg | 0.71 / 0.83 / 0.97 | 0 % |
| yaw 10 deg | 0.47 / 0.67 / 0.80 | 3 % |

At the spawn: a 0.3 m sideways error scores 0.2 and is refused. A 0.5 m
along-track error (the spawn is on a straight) or a 5 deg yaw error scores 1.0 /
0.82 and is NOT caught. In the node test, a map shifted 0.30 m was refused and an
aligned map was confirmed.

### 3. A detected reset was never handled (FIXED, differently than first described)
A wall contact respawns the car at the last checkpoint with its velocity zeroed,
and the encoder counters do NOT reset, so the old encoder-jump detector never
saw it. The signature is an IMU heading jump the yaw rate cannot explain (> 0.35
rad), the same one pure pursuit uses. Bootstrap now watches for it after
hand-over. It re-seeds AMCL on the raceline behind the last estimate (up to 6 m)
at the point whose heading best matches the new IMU heading. The covariance is
stretched along the matching stretch (std = length / sqrt(12), at least 0.5 m;
0.3 m across).

Offline: in 174 simulated respawns the seed was within 2 along-track std of the
checkpoint 95 % of the time. Limits: only about a third of the lap changes
heading enough over 2 m to be detectable, and a respawn that keeps the heading
(mid-straight) is invisible. IMU gaps > 0.3 s are ignored so a stall cannot
look like a respawn.

### 4. A tick between left and right encoder published half a step (FIXED)
Travel is now banked only when every live encoder has reported the frame.

### 5. A dead encoder silently halved distance (FIXED)
An encoder silent for `encoder_stale_s` (0.25 s) leaves the mean, with a warning.
When it returns, its first delta is dropped, because the other side already
banked those frames. Offline, the right encoder dying at 8 s: old code 2.93 m off
at lap end, new code 0.03 m.

### 6. Extrapolation speed was one jittery frame (FIXED)
Speed is now averaged over 3 frames on a de-jittered frame clock
(`_frame_clock`). The clock advances by the median frame period and follows raw
stamps with gain `frame_clock_gain` (0.1); anything more than half a period off
snaps. Set the gain to 1.0 to use raw stamps. Without the clock, the tire
observer ended a lap 0.99 m out on jittered stamps instead of 0.03 m.

Overall, no slip, offline (odom error at scan stamps over one lap, no AMCL):

| sim rate | old mean / p95 / end | new mean / p95 / end |
|---|---|---|
| 18 Hz | 0.167 / 0.286 / 0.170 m | 0.046 / 0.096 / 0.024 m |
| 40 Hz | 0.069 / 0.124 / 0.046 m | 0.028 / 0.053 / 0.023 m |

## Still open (tuning; test against ground truth)

7. **Random particle injection is on** (`recovery_alpha_slow: 0.001`,
   `recovery_alpha_fast: 0.10`). A brief scan-match dip can scatter particles.
   nav2's default of 0.0 may be safer now that respawns are re-seeded explicitly.
8. **AMCL may rotate a heading the IMU already has right.** Try lower
   `alpha1`/`alpha2`.
9. **Sensor model.** `resample_interval: 1` plus about 5 nudges while parked
   deplete particles. `z_rand: 0.30` is high for a clean sim lidar.
   `z_short`/`z_max` have no effect in `likelihood_field`.
10. **No correction below 10 cm of travel** (`update_min_d`). Only matters when
    parked or creeping.
11. **Map gray is free, not unknown.** `track_clean.yaml` has `free_thresh: 0.25`,
    and gray 205 gives occupancy 0.196 < 0.25. So nav2 loads the whole grid
    as free or occupied, with 0 unknown cells, and `global` mode samples
    particles outside the track too. Tracking is unaffected. `free_thresh: 0.19`
    would restore unknown.

## Still unverified

- The new map's alignment with the sim's world frame. The scan check now catches
  lateral misalignment at the spawn; along-track and small rotations still need
  a scan overlay in the sim.
- `wheel_radius: 0.0581` was measured on the practice track.
- The real sim's receipt-stamp jitter: the frame clock assumes most stamp noise
  is receipt jitter rather than uneven sim frames.
- `raceline_a7.0.csv` has a curvature spike of |kappa| = 11.46 (outside the
  localizer; pure pursuit reads it).

## New parameters

| node | parameter | default | launch argument |
|---|---|---|---|
| dead_reckoning | `distance_source` | `encoder` | `dr_distance_source` |
| dead_reckoning | `encoder_stale_s` | 0.25 | |
| dead_reckoning | `imu_wait_s` | 0.02 | |
| dead_reckoning | `speed_window_frames` | 3 | |
| dead_reckoning | `frame_clock_gain` | 0.1 | |
| dead_reckoning | `tire_rise_slope`, `v_slip_den` | 3.0, 4.0 | |
| localization_bootstrap | `scan_match_min` (0 disables) | 0.5 | |
| localization_bootstrap | `scan_match_tol_m`, `scan_match_max_range` | 0.10, 6.0 | |
| localization_bootstrap | `respawn_watch` | true | |
| localization_bootstrap | `respawn_yaw_jump`, `respawn_back_m`, `respawn_max_dyaw_deg`, `respawn_std_cross` | 0.35, 6.0, 15, 0.30 | |
| localization_bootstrap | `path_csv` | the default raceline | `path_csv` (shared with the follower) |

Offline tests: `tools/localizer_tests/` (not in the image). Run them in the
devkit image with `PYTHONPATH=roboracer_stack`; see each file's header.
