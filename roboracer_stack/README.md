# roboracer_stack

The team's racing package. Separate from `autodrive_roboracer`, which is used
unmodified, as the technical guide requires.

```
roboracer_stack/
├── roboracer_stack/
│   ├── common/        frames, paths, the spawn constant, the restricted-topic list
│   ├── localization/  dead reckoning, the AMCL bootstrap
│   ├── planning/      the racing line: read the CSV, derive what the follower needs
│   └── control/       pure pursuit
├── launch/            race.launch.py composes; the rest are one subsystem each
├── config/            amcl.yaml pure_pursuit.yaml cyclonedds.xml
├── maps/              the Porto track, baked in
├── raceline/          the racing line, baked in
└── test/              the flake8 gate; `.flake8` records where it differs
                       from ament's default, and why
```

## Running it

```bash
ros2 launch roboracer_stack race.launch.py mode:=race      # what the entrypoint runs
ros2 launch roboracer_stack race.launch.py                 # dev mode: lap telemetry on
```

`race.launch.py` is the only file that composes others. It brings up the devkit
bridge (with the ground-truth TF remapped off `/tf`, or `roboracer_1` gets two
parents and the TF tree breaks), the chassis TF chain, AMCL with its bootstrap,
and the follower. Every other launch file starts its own subsystem and can be
run alone against an already-running remainder.

In the container the entrypoint runs this for you; see the repository README.

## Where the data comes from

`maps/` and `raceline/` are installed into the package share, and
`common/frames.py` resolves them through the ament index; no path in this
package assumes where the repo was cloned. Override with `$RACER_MAPS_DIR` and
`$RACER_RACELINE_DIR` if you need to.

The line was planned offline by `raceline/optimize_raceline.py` on the `main`
branch; this branch ships the finished CSV.

## Restricted topics

`/ips`, `/odom`, `/tf` and all lap and collision telemetry are restricted during
timed laps. `common/restricted.py` holds the list and the two access patterns:
`seed()`, one read inside the warm-up window with the subscription destroyed
afterwards, and `warn()`, a continuous subscription that `mode:=race` refuses
outright. The default container uses neither: AMCL is seeded from the measured
spawn constant. Read it before adding any subscription.
