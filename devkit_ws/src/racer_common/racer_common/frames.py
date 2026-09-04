"""Frame names, sensor extrinsics and workspace paths, in one place.

Every launch file in the workspace imports from here rather than repeating
literals. Before this module the lidar extrinsic and the spawn pose were
duplicated across three launch files each, and drifted.

Launch files are ordinary Python, so `from racer_common.frames import BASE`
works in any package that depends on racer_common.
"""

import os

# ---- TF frames -------------------------------------------------------------
MAP = 'map'
ODOM = 'odom'
BASE = 'roboracer_1'        # the devkit's vehicle frame, at the rear axle centre
LIDAR = 'lidar'
# The devkit's ground-truth parent. Restricted at race time, and remapped off
# /tf by bridge.launch.py -- see racer_bringup.
WORLD = 'world'

# ---- Sensor extrinsics -----------------------------------------------------
# From autodrive_bridge.py's TF broadcast. We republish these ourselves once the
# devkit's /tf is remapped away.
LIDAR_XYZ = ('0.2733', '0.0', '0.096')

# ---- Paths -----------------------------------------------------------------
REPO = os.path.join(os.path.expanduser('~'), 'Documents/roboracer')
MAPS_DIR = os.path.join(REPO, 'devkit_ws/src/racer_mapping/maps')
RACELINE_DIR = os.path.join(REPO, 'raceline')

# ---- Tracks ----------------------------------------------------------------
# Every track asset is scoped by track name: the occupancy grid and pose graph
# live in maps/<track>/, the centreline and the raceline ladder in
# raceline/<track>/. The simulator ships several tracks (Porto, Berlin, and the
# SRL 2024/2025 scenes), so nothing here may assume Porto.
#
# WHAT IS PER TRACK AND WHAT IS NOT: the car model is not -- the tire curves,
# the 25.25 m/s per unit throttle, the command delay handling and the whole
# controller come from the simulator's physics and transfer unchanged
# (raceline/FINDINGS.md section 2). What lives in this registry is exactly the
# part that does NOT transfer: where the car spawns, which grip rung was
# measured safe, and the margin zones, which are hand-placed at the spots where
# THIS track's tracking error lands. Reusing another track's margin zones would
# push the line toward a wall rather than away from one.
#
# Adding a track: map it (racer_mapping), drop the grid in maps/<name>/, run
# optimize_raceline.py --track <name>, then add a row here.
TRACKS = {
    'porto': {
        # Fallback initial pose. NOT (0, 0, 0): that cell is a WALL in
        # track_clean.pgm and sits 0.71 m off the racing line, so a filter
        # seeded there starts confidently inside a barrier. This is the nearest
        # centreline point. Replace with the measured spawn if the car starts
        # elsewhere:  ros2 topic echo /autodrive/roboracer_1/ips --once
        'spawn': ('0.71', '0.02', '-1.599'),
        # Built by raceline/optimize_raceline.py: min-curvature geometry with
        # extra left margin on the straight after R1 and on the S-exit approach,
        # 0.15 m body-to-wall margin elsewhere, velocity profile at a_lat 6.5.
        # That is over the tire's 4.90 asymptote and is held by the follower's
        # curvature cap (steer_a_lat_max): measured clean, 32 consecutive laps,
        # 6.65 s best. raceline_a7.0.csv is the fast line (6.50 s) and
        # raceline_a7.0_rec.csv the record attempt (6.45 s, 0.03 m clearance);
        # same geometry, ladder in VEHICLE_MODEL.md section 7.
        'raceline': 'raceline_a6.5.csv',
        # The argument that built this track's ladder. Per track by
        # construction: these are s-ranges on THIS centreline.
        'margin_zones': '2.5:6:L:0.10,17:21.5:L:0.15',
    },
    'icra2026': {
        # The pose the simulator resets the car to, read by the bootstrap on
        # the first run (2026-09-05). A first reading of (1.16, 1.82) was
        # where the car had been PARKED after the mapping laps, not the spawn:
        # read /ips right after a reset, not after driving. Coincides with
        # Porto's spawn; the scenes share a spawn point.
        'spawn': ('0.800', '3.158', '-1.5707'),
        # Lowest rung until the ladder has been climbed on the car; raise this
        # to the highest rung that runs clean, exactly as Porto's was.
        'raceline': 'raceline_a4.0.csv',
        # Placed from measurement, exactly as Porto's were: on the first logged
        # run (a4.0, 13 laps) the car exited the right-hand hairpin round the
        # left leg's tip 0.10-0.13 m to the left of the line on EVERY lap, and
        # the line ran 0.26 m from the outer wall there, leaving 0.02 m. Extra
        # left margin over that exit puts the line at 0.43 m and the car at
        # 0.32 m; the re-solved line is shorter and predicts 0.09 s faster.
        'margin_zones': '45.5:51.5:L:0.20',
    },
}

# Which track the stack uses when nothing says otherwise. Override with the
# RACER_TRACK environment variable, or track:= on any launch file.
TRACK = os.environ.get('RACER_TRACK', 'porto')


def _track(track=None):
    name = track or TRACK
    if name not in TRACKS:
        raise KeyError(
            f'unknown track {name!r}; known: {sorted(TRACKS)}. Add a row to '
            'racer_common.frames.TRACKS after mapping it.')
    return name


def map_yaml(track=None):
    """Occupancy grid for AMCL."""
    return os.path.join(MAPS_DIR, _track(track), 'track_clean.yaml')


def pose_graph(track=None):
    """BASE PATH, NO EXTENSION -- slam_toolbox appends .posegraph and .data."""
    return os.path.join(MAPS_DIR, _track(track), 'track_sm')


def raceline_dir(track=None):
    return os.path.join(RACELINE_DIR, _track(track))


def raceline(track=None, name=None):
    """A line for this track: the registry's default, or `name` within it."""
    t = _track(track)
    return os.path.join(RACELINE_DIR, t, name or TRACKS[t]['raceline'])


def spawn(track=None):
    """(x, y, yaw) as strings, for launch arguments."""
    return TRACKS[_track(track)]['spawn']


# Back-compatible module constants, resolved for the current TRACK. Launch
# files that accept track:= call the functions above instead.
DEFAULT_MAP_YAML = map_yaml()
DEFAULT_POSE_GRAPH = pose_graph()
DEFAULT_RACELINE = raceline()
SPAWN_X, SPAWN_Y, SPAWN_YAW = spawn()

# ---- Vehicle ---------------------------------------------------------------
NS = '/autodrive/roboracer_1'
