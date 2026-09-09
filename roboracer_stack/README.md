# roboracer_stack

The team's racing package. Separate from `autodrive_roboracer`, which is used
unmodified, as §3.4 of the technical guide requires.

```
roboracer_stack/
├── roboracer_stack/
│   ├── common/        frames, paths, the competition restricted-topic list
│   ├── perception/    scan-vs-map alignment check (diagnostic)
│   ├── localization/  dead reckoning, AMCL/slam bootstrap, error instruments
│   ├── planning/      the racing line: read the CSV, derive what the follower needs
│   ├── control/       pure pursuit, steering calibration
│   └── mapping/       map publisher
├── launch/            race.launch.py composes; the rest are one subsystem each
├── config/            amcl.yaml slam.yaml pure_pursuit.yaml cyclonedds.xml
├── rviz/              one config per launch file
├── maps/              the tracks, baked in: porto at the top level, icra/
├── raceline/          the lines, baked in, one subdirectory per added track
├── tools/             offline utilities; not installed, not part of a run
└── test/              the flake8 gate; `.flake8` records where it differs
                       from ament's default, and why
```

## Running it

```bash
ros2 launch roboracer_stack race.launch.py                 # AMCL, dev mode
ros2 launch roboracer_stack race.launch.py mode:=race      # nothing restricted
ros2 launch roboracer_stack race.launch.py localizer:=slam # slam_toolbox instead
```

`race.launch.py` is the only file that composes others. It brings up the devkit
bridge (with the ground-truth TF remapped off `/tf`, or `roboracer_1` gets two
parents and the TF tree breaks), the chassis TF chain, a localizer, and the
follower. Every other launch file starts its own subsystem and can be run alone
against an already-running remainder.

In the container the entrypoint runs this for you; see the repository README.

## Where the data comes from

`maps/` and `raceline/` are installed into the package share, and
`common/frames.py` resolves them through the ament index — no path in this
package assumes where the repo was cloned. Override with `$RACER_MAPS_DIR` and
`$RACER_RACELINE_DIR` if you need to.

Which track those paths point INTO is `common/frames.py`'s `TRACKS` table: one
row per circuit, holding the spawn pose, the AMCL grid, the pose graph and the
default line. `$RACER_TRACK` picks a row (default `icra`, `porto` for the
qualification track) and is read at import time, so it is an environment
variable rather than a launch argument. `path_csv:=` takes a bare filename and
resolves it inside the selected track's directory:

```bash
RACER_TRACK=porto ros2 launch roboracer_stack race.launch.py \
    path_csv:=raceline_a6.5.csv
```

Adding a track means dropping a grid in `maps/<name>/`, lines in
`raceline/<name>/`, and adding a row — `setup.py` installs both by discovery.

The lines themselves are planned offline by `raceline/optimize_raceline.py` on
the `main` branch; this branch ships the finished CSVs.
[`tools/MAPPING.md`](tools/MAPPING.md) records how the map was made.

## Restricted topics

`/ips`, `/odom`, `/tf` and all lap and collision telemetry are restricted during
timed laps. `common/restricted.py` holds the list and the two access patterns —
`seed()`, one read inside the warmup window with the subscription destroyed
afterwards, and `warn()`, a continuous subscription that `mode:=race` refuses
outright. Read it before adding any subscription.

## Tools

`tools/` is offline and **not installed** -- `setup.py` excludes it, so nothing
here can reach a submission image by accident. `scripts/bench.sh` bind-mounts
the directory instead.

| | |
|---|---|
| `analyze_localization.py` | scores a `log_localization` CSV: position and heading error, the along/cross split, per-lap and per-corner breakdowns, the `map->odom` rotation invariant, and a verdict. Writes `<run>.html` and `<run>.json`; `--compare` ranks several runs |
| `svg_report.py` | the HTML/SVG writer it renders through. Standard library only -- the image has no matplotlib, and this is not worth a dependency in a submission image |
| `bench/variants.tsv` | the A/B list: one line per AMCL variant |
| `bench/make_variant.py` | materialises a variant from the **shipped** `config/amcl.yaml` plus one change, so a variant is a genuine one-line diff and `config/amcl.yaml` is never edited |
| `clean_map.py` | despeckle a fresh map |
| `diagnose_localization.py` | walk the localization chain link by link; the first FLAT link is the bug |

`clean_map.py` and `diagnose_localization.py` want `scipy`, which the submission
image does not ship -- `scripts/bench.sh up` installs it into the bench
container (pinned, `--no-deps`; an unpinned install replaces numpy and breaks
every compiled ROS module). `perception/scan_alignment.py` also uses scipy but
imports it lazily, so the rest of the package stays importable without it.
`analyze_localization.py` and `svg_report.py` need only numpy, which ROS ships.

## Measuring the localizer

`launch/instruments.launch.py` holds every node that reads a restricted topic,
and `mode:=race` omits that file wholesale -- so nothing measures anything in a
race-mode run, by design. To measure, run `mode:=dev`:

```bash
ros2 launch roboracer_stack race.launch.py mode:=dev \
    log_csv:=/path/run.csv log_map:=<share>/maps/track_clean
```

`log_map` matters more than it looks: scoring the scan fit against a map the
localizer never saw makes the `fit_ratio` column -- the one that separates a
filter problem from a map problem -- meaningless. It now defaults to
`common.frames.DEFAULT_FIT_MAP`, the selected track's AMCL grid, so it is
only worth passing to score against a DIFFERENT grid -- `maps/track_sm` when
debugging `localizer:=slam` on porto. It used to default to `track_sm`
unconditionally, which is a Porto file.

Then `python3 tools/analyze_localization.py run.csv`. See the repository README
for the wrapper that does all of this in one command.
