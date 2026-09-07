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
├── maps/              the Porto track, baked in
├── raceline/          three velocity profiles on one geometry, baked in
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

`tools/` is offline and not installed: `clean_map.py` (despeckle a fresh map)
and `diagnose_localization.py`. Both want `scipy`, which the submission image
does not ship. `perception/scan_alignment.py` also uses it, but imports it
lazily, so the rest of the package stays importable without it.
