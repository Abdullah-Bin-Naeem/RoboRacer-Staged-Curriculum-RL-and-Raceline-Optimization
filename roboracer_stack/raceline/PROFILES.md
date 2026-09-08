# Velocity profiles: how the lines in this directory were made

The **geometry** of every line here (columns 0-6: `s, x, y, psi, kappa, w_right,
w_left`) comes from the minimum-curvature solve in `raceline/optimize_raceline.py`
on the `main` branch. Nothing in this document changes it.

The **velocity profile** (column 7, `v_mps`) is what the follower tracks when
`use_path_speed: true`, and it is re-planned offline by
`../tools/reprofile_raceline.py`. That tool copies columns 0-6 through as the
strings they arrived as -- so the optimizer's geometry survives byte-for-byte --
and rewrites only `v_mps`.

This file records what was measured, what was changed, and how to check it.

---

## 1. What the shipped profile actually assumed

`raceline_a7.0.csv` carries no record of the limits it was planned with, so they
were recovered by differentiating it (`a_x = v dv/ds`, `a_lat = v^2 |kappa|`):

| quantity | value | how it was read |
|---|---|---|
| lateral limit | **7.00 m/s^2** | `max(v^2 |kappa|)`, hit at the tightest corner |
| braking limit | ~7.0 m/s^2 | fits `a_brake * sqrt(1 - (a_lat/7)^2)` |
| acceleration limit | **3.95 m/s^2** | the plateau below ~5 m/s |
| "power" limit | ~20.5 W/kg | `a_x * v` is flat at 20.4-20.5 from 5.5 to 7.0 m/s |
| lap, 400 pts | 27.82 m | |
| speed range | 2.81 - 7.30 m/s, mean 4.68 | |
| plan time | 6.357 s | |

The shape is a forward-backward pass with a **friction circle** shared between
lateral and longitudinal demand:

```
v_curve = sqrt(a_lat / |kappa|)                    cornering limit
circle  = sqrt(1 - (v^2 |kappa| / a_lat)^2)        grip left over for a_x
accel   = min(a_accel * circle, power / v)         forward pass
brake   = a_brake * circle                         backward pass
```

Those four numbers are the tool's **defaults**, so running it with no flags
re-plans `raceline_a7.0.csv` onto itself (rms 0.03 m/s, worst 0.09 m/s at the lap
seam, where the tool closes the loop slightly more conservatively than the
original did). That is the tool's self-check: **a run with one flag changed
isolates that one flag.**

### Where the headroom was

Cornering and braking were already at the tyre's limit -- `TIRE_MU_PEAK * G =
0.72 * 9.81 = 7.06 m/s^2` in `control/pure_pursuit.py`, and the profile sits at
7.00, i.e. 99 % of it. **There is nothing to win in `a_lat` or `a_brake`.**

Acceleration was the outlier: planned at 3.95 where the same tyre curve offers
6.35 m/s^2 at slip 0.08 and 7.06 at its peak. That is the one number worth
raising, and it is what produced the fast line below.

---

## 2. The line the car is fastest on

**`raceline_a7.0_6.4laptime.csv`** -- ~6.4 s driven, the best measured so far.

```sh
cd roboracer_stack/raceline
python3 ../tools/reprofile_raceline.py \
    raceline_a7.0.csv raceline_a7.0_6.4laptime.csv \
    --a-accel 6.0
```

**One flag.** `--a-accel 6.0` (from the shipped 3.95); everything else left at
the defaults in section 1 -- `--a-lat 7.0 --a-brake 7.0 --power 20.5 --drag 0.0
--v-max 8.0 --v-min 1.0`.

Verified byte-for-byte: re-running that command reproduces the file exactly.

| | `raceline_a7.0.csv` | `raceline_a7.0_6.4laptime.csv` |
|---|---|---|
| peak speed | 7.30 | **7.49** m/s |
| mean speed | 4.68 | **4.82** m/s |
| slowest corner | 2.81 | 2.81 m/s (unchanged) |
| realized acceleration | 3.95 | **4.56** m/s^2 |
| braking | -6.66 | -6.59 m/s^2 |
| lateral | 7.00 | **7.00** (unchanged) |
| plan time | 6.357 | **6.207** s |
| largest speed change | | 0.414 m/s |

Two properties of this change are worth keeping in mind:

* **Corner speeds are untouched.** `a_lat` stays at exactly 7.00, so
  `profile_a_lat` (which `Raceline` reads back as `max(v^2 |kappa|)`) is
  unchanged, and the follower's delay derate behaves exactly as before. The car
  is faster only between the corners.
* **The realized acceleration is 4.56, not 6.0.** The friction circle and the
  20.5 W/kg power term cap it. Asking for 6.0 buys the low-speed corner exits,
  where the circle is what binds; the straight is still power-limited.

### Why the plan says 6.207 and the car drives ~6.4

That ~0.19 s is not slack in the plan. It is the follower and the vehicle: the
plan models neither drag (`DRAG_LIN = 0.273 /s`, about 2.0 m/s^2 at 7.3 m/s) nor
the tyre's transient build-up of slip, the car tracks a slightly longer path
than the line, and **the delay derate may be scaling every target down** -- see
`../config/pure_pursuit.yaml`, `derate_delay_from`. Treat the plan time as a
comparison between profiles, never as a lap-time prediction.

---

## 3. A correction to the model

The ~20.5 W/kg term above is **not the simulator's physics.** The sim's
longitudinal model, transcribed into `control/pure_pursuit.py:89-94` from its
source, is

```
a = mu(slip) * g - 0.273 * v
```

-- tyre force, minus linear Rigidbody drag. There is no power term anywhere in
it. Fitted against the shipped profile's high-speed taper, a drag-shaped term
explains it better than a power one (rms 0.047 vs 0.084 m/s), with a coefficient
near 0.50 /s -- about double the sim's real 0.273. At 7 m/s the shipped plan asks
for 2.93 m/s^2 where the car can pull `6.35 - 0.273*7 = 4.44`.

So `--power 20.5` is kept as the default only because it is what
`raceline_a7.0.csv` was planned with and what the 6.4 s line inherited.
**`--power 0 --drag 0.273` is the physically correct model** and is what further
work should use. It is not automatically faster: drag charges the lap for speed
the power model gave away for free, so it only pays once `--a-accel` is raised
past about 6.

---

## 4. The files

| file | derived from | flags | plan | driven |
|---|---|---|---|---|
| `raceline_a6.5.csv` | optimizer | `a_lat` 6.5 | 6.537 | |
| `raceline_a7.0.csv` | optimizer | `a_lat` 7.0 | 6.357 | 6.60-6.69 [1] |
| `raceline_a7.0_rec.csv` | optimizer | recentred variant | 6.291 | |
| `raceline_a7.0_6.4laptime.csv` | `a7.0` | `--a-accel 6.0` | 6.207 | **~6.4** |
| `raceline_a7.0_acc5.0.csv` | `a7.0` | `--a-accel 5.0` | 6.255 | |
| `raceline_a7.0_fast.csv` | `a7.0` | `--a-accel 6.0 --a-brake 6.35 --power 0 --drag 0.273` | 6.238 | |
| `raceline_a7.0_max.csv` | `a7.0` | `--a-accel 6.8 --a-brake 7.0 --power 0 --drag 0.273` | 6.150 | needs `slip_accel` 0.11 |

[1] from the run notes in `../config/pure_pursuit.yaml` (runs 24-28), not
measured here.

Note `raceline_a7.0_rec.csv` plans 6.291 against `raceline_a7.0.csv`'s 6.357 --
the recentred geometry is worth 0.066 s before any profile change, and it has
never been re-planned with a raised `--a-accel`. That is the cheapest untried
experiment in this directory.

`setup.py` installs `raceline/*.csv` by glob, so a new file here ships on the
next build with no edit. Select one at run time:

```sh
ros2 launch roboracer_stack race.launch.py \
    path_csv:=$(ros2 pkg prefix roboracer_stack)/share/roboracer_stack/raceline/<file>.csv
```

`raceline_a7.0_max.csv` plans 6.150 but asks the tyre for 6.8 m/s^2, above what
`slip_accel: 0.08` delivers (6.35). Without raising `slip_accel` to ~0.11 the
follower's slip band clips it and the extra plan is never driven.

---

## 5. Checking a file before you drive it

**Write each profile to its own path, once.** Running the tool twice into one
output produced a 47 KB, 404-line file with interleaved rows -- `np.loadtxt` in
`Raceline` raises on that at ROS start-up. The tool now refuses a ragged file by
line number, so re-reading a profile is a sufficient check:

```sh
python3 ../tools/reprofile_raceline.py <file>.csv      # reports; writes nothing
```

and to prove the geometry was preserved:

```sh
diff <(cut -d, -f1-7 raceline_a7.0.csv) <(cut -d, -f1-7 <file>.csv)   # must be empty
```

---

## 6. Measuring a run

Comparing two profiles on lap time alone is unsound: the follower's measured
`cmd_delay` scales every speed target (see `derate_delay_from` in
`../config/pure_pursuit.yaml`), so a busy machine and a slow raceline look
identical. Record both together, from outside the submission container:

```sh
docker exec -i autodrive_roboracer_api bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/autodrive_devkit/install/setup.bash
    exec python3 - --seconds 180
' < ../tools/record_run.py > ../../runs/run42.csv
```

`../tools/record_run.py` reads the official race telemetry the organizers time
with (`lap_count`, `last_lap_time`, `collision_count` -- restricted, so
development only) plus the follower's own `/pure_pursuit/status`, and reports
clean laps separately from laps that touched a wall. A profile is only better
if its CLEAN laps are better at the same `cmd_delay`.
