# roboracer_stack

Our racing package. It is separate from the provided `autodrive_roboracer`
package, which is used unmodified.

```
roboracer_stack/
├── roboracer_stack/
│   ├── common/        frame names, data paths, the spawn, the checkpoints,
│   │                  the tire model, the restricted-topic list
│   ├── localization/  dead reckoning; the scan-to-map localizer and its
│   │                  matcher, filter and status codes; the bootstrap that
│   │                  seeds it, confirms the seed, and recovers after a
│   │                  wall contact
│   └── control/       pure pursuit with a bounded LQR correction
├── launch/            race.launch.py starts everything; the others start one part each
├── config/            localization_v2.yaml, pure_pursuit.yaml, cyclonedds.xml
├── maps/              the IROS 2026 track
├── raceline/          the racing line, the centreline and the segmentation
└── tools/             nodelay.c, compiled into the image by the Dockerfile
```

## Running it

```bash
ros2 launch roboracer_stack race.launch.py     # exactly what the container runs
```

No arguments. Everything that makes this the promoted configuration is a launch
default, so there is no second copy of the configuration to drift.

`race.launch.py` starts, in order: the devkit bridge (with its ground-truth
transform moved off `/tf`, so our localization owns the transform tree), dead
reckoning, the localizer with its bootstrap, and pure pursuit. In the container
the entrypoint runs this for you.

## How it localizes

`localization_v2` matches each lidar scan against a likelihood field built from
the occupancy grid. The track is cut into segments and each segment carries its
own acceptance gates, because a scan on a long straight pins the car across the
corridor but barely along it, while a scan in a hairpin pins both. That
along-track blindness on the straights is what the previous AMCL stack could not
fix.

It needs no nav2: no map server, because the node reads the PGM itself, and no
lifecycle manager, because it is a plain node.

`localization/bootstrap.py` seeds it from the measured spawn, confirms the
localizer adopted that pose before the follower is allowed to drive, and then
keeps watching for the signature of a wall contact. When it sees one it holds
the car, re-seeds from the checkpoint behind the last trusted pose, and releases
the car once that seed is confirmed.

## Data

`maps/`, `raceline/` and `tools/` are installed with the package and found
through the ROS package index, so nothing depends on where the repository was
cloned. `RACER_MAPS_DIR`, `RACER_RACELINE_DIR` and `RACER_TOOLS_DIR` override
them if needed.

## Restricted topics

`/ips`, `/odom`, `/tf` and all lap and collision telemetry are restricted during
timed laps. `common/restricted.py` lists them and defines the two ways to touch
one: `seed()` (a single read in the warm-up lap, subscription destroyed
afterwards) and `warn()` (a continuous subscription). The container uses
neither: the localizer is seeded from the measured spawn constant and the IMU's
heading, so no restricted topic is subscribed at any point.
