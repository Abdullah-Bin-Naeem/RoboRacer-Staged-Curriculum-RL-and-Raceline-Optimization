# roboracer_stack

Our racing package. It is separate from the provided `autodrive_roboracer`
package, which is used unmodified.

```
roboracer_stack/
├── roboracer_stack/
│   ├── common/        frame names, data paths, the spawn position, the restricted-topic list
│   ├── localization/  dead reckoning; the bootstrap that seeds and confirms AMCL
│   ├── planning/      reads the racing line CSV
│   └── control/       pure pursuit
├── launch/            race.launch.py starts everything; the others start one part each
├── config/            amcl.yaml, pure_pursuit.yaml, cyclonedds.xml
├── maps/              the Porto track
└── raceline/          the racing line (raceline_a7.0.csv)
```

## Running it

```bash
ros2 launch roboracer_stack race.launch.py mode:=race   # what the container runs
ros2 launch roboracer_stack race.launch.py              # development: lap telemetry on
```

`race.launch.py` starts, in order: the devkit bridge (with its ground-truth
transform moved off `/tf`, so our localization owns the transform tree), dead
reckoning, AMCL with the bootstrap, and pure pursuit. In the container the
entrypoint runs this for you.

## Data

`maps/` and `raceline/` are installed with the package and found through the
ROS package index, so nothing depends on where the repository was cloned.
`RACER_MAPS_DIR` and `RACER_RACELINE_DIR` override them if needed.

## Restricted topics

`/ips`, `/odom`, `/tf` and all lap and collision telemetry are restricted
during timed laps. `common/restricted.py` lists them and defines the two ways to
touch one: `seed()` (a single read in the warm-up lap, subscription destroyed
afterwards) and `warn()` (a continuous subscription, refused in race mode). The
container uses neither: AMCL is seeded from the measured spawn position.
