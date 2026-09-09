"""Frame names, sensor extrinsics and workspace paths, in one place.

Every launch file and every node imports from here rather than repeating
literals. Before this module the lidar extrinsic and the spawn pose were
duplicated across three launch files each, and drifted.

Launch files are ordinary Python, so
`from roboracer_stack.common.frames import BASE` works in them too.
"""

import os

# ---- TF frames -------------------------------------------------------------
MAP = 'map'
ODOM = 'odom'
BASE = 'roboracer_1'        # the devkit's vehicle frame, at the rear axle centre
LIDAR = 'lidar'
# The devkit's ground-truth parent. Restricted at race time, and remapped off
# /tf by launch/bridge.launch.py.
WORLD = 'world'

# ---- Sensor extrinsics -----------------------------------------------------
# From autodrive_bridge.py's TF broadcast. We republish these ourselves once the
# devkit's /tf is remapped away.
LIDAR_XYZ = ('0.2733', '0.0', '0.096')

# ---- Paths -----------------------------------------------------------------
PACKAGE = 'roboracer_stack'

# Track data is resolved at RUNTIME, in this order, and never from a path that
# assumes where the repo was cloned:
#
#   1. $RACER_MAPS_DIR / $RACER_RACELINE_DIR   explicit override
#   2. the installed package share             how the container finds it
#   3. the source tree beside this file        colcon build --symlink-install
#
# This used to be REPO = ~/Documents/roboracer, hard-coded. That directory does
# not exist in the submission container and had already drifted from the
# checkout on the development machine, so every default here silently pointed at
# nothing and the follower launched with an empty path. Resolution order, not a
# constant.


def _data_dir(env_var, subdir):
    """First of: the env override, the installed share, the source tree."""
    override = os.environ.get(env_var)
    if override:
        return override

    try:
        from ament_index_python.packages import get_package_share_directory
        share = os.path.join(get_package_share_directory(PACKAGE), subdir)
        if os.path.isdir(share):
            return share
    except Exception:
        # Not built / not on the ament index yet. Fall through rather than
        # raise: importing this module must never be what breaks a launch.
        pass

    # roboracer_stack/roboracer_stack/common/frames.py -> the package directory,
    # which is where maps/ and raceline/ live in the source tree. THREE levels:
    #   common/ -> roboracer_stack/ (the python package) -> roboracer_stack/
    # A fourth climbed out to the repo root and returned <repo>/maps, which does
    # not exist. It never showed inside the container -- the ament share is
    # found first and this branch is dead there -- so the failure was confined
    # to running a tool or a launch file straight from the source tree.
    pkg = os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(pkg, subdir)


MAPS_DIR = _data_dir('RACER_MAPS_DIR', 'maps')
RACELINE_DIR = _data_dir('RACER_RACELINE_DIR', 'raceline')

# ---- Tracks ----------------------------------------------------------------
# Everything that changes when the car is put on a different circuit, in one
# table. Paths are relative to MAPS_DIR / RACELINE_DIR above; Porto's assets sit
# at the top level of each (where they have always been) and every track added
# since is a subdirectory, so the relative path is per track rather than a
# naming rule.
#
# WHAT IS PER TRACK AND WHAT IS NOT: the car model is not. The tire curves, the
# throttle scale, the command-delay handling and the whole controller come from
# the simulator's physics and transfer unchanged. What belongs here is the part
# that does NOT transfer -- where the car spawns, which grip rung was measured
# safe, and which grid the localizer matches against.
#
# Adding a track: drop the grid in maps/<name>/, the lines in raceline/<name>/,
# and add a row. setup.py installs both subdirectories by discovery, so it does
# not need editing.
TRACKS = {
    'icra': {
        # The pose the simulator resets the car to, measured on the multi-track
        # branch (2026-09-05). Its note is worth repeating: an earlier reading
        # of (1.16, 1.82) was where the car had been PARKED after the mapping
        # laps, not the spawn. Read /ips right after a reset, not after driving:
        #     ros2 topic echo /autodrive/roboracer_1/ips --once
        'spawn': ('0.800', '3.158', '-1.5707'),
        'map_yaml': 'icra/track_clean.yaml',
        # NOT SHIPPED. icra/track.posegraph and .data were deliberately left out
        # (54 MB for a localizer this branch does not qualify on), so
        # localizer:=slam will fail on this track until they are re-imported
        # from the multi-track branch. AMCL is unaffected.
        'pose_graph': 'icra/track',
        # The top rung of an eight-file ladder whose geometry is IDENTICAL
        # across every rung -- columns 1-7 are byte-for-byte the same in a4.0
        # through a7.0, and only v_mps differs. So this is the fast velocity
        # profile (peak a_lat 7.00 m/s^2, v 2.31-8.00) on the same line the
        # lower rungs drive, not a re-solved fast line: no extra wall margin was
        # bought to pay for the speed. Minimum inside clearance is 0.134 m at
        # every rung. Drop to raceline_a4.0.csv (peak 4.00) if it runs wide.
        #
        # raceline_tum.csv is a duplicate of raceline_a4.5.csv, not a separate
        # solver's line -- same md5. centerline_widths.csv (4 columns) and
        # centerline_full.csv (7) are optimizer INPUTS; the first crashes
        # planning.raceline, the second loads with no velocity profile.
        'raceline': 'icra/raceline_a7.0_edit_10277.csv', # 10296 works good with 10.8 time
    },
    'porto': {
        # NOT (0, 0, 0): that cell is a WALL in track_clean.pgm and sits 0.71 m
        # off the racing line, so a filter seeded there starts confidently
        # inside a barrier. This is the nearest centreline point.
        'spawn': ('0.71', '0.02', '-1.599'),
        'map_yaml': 'track_clean.yaml',
        'pose_graph': 'track_sm',
        # THE QUALIFICATION LINE. Measured on the car, AMCL + pure pursuit,
        # run 38: 6.50 s best / 6.58 s mean over 7 clean laps, wall clearance
        # 0.07 m left and 0.10 m right. Two other rungs ship beside it, same
        # geometry:
        #
        #   raceline_a6.5.csv      run 40, 6.65 / 6.71 over 32 consecutive clean
        #                          laps. The safe rung -- most margin. Use it if
        #                          the evaluation machine runs the loop slow.
        #   raceline_a7.0_rec.csv  run 41, 6.45 / 6.53 -- beats the 6.46 track
        #                          record, but the S-exit clearance falls to
        #                          0.03 m. Deliberately NOT the default; see
        #                          VEHICLE_MODEL.md section 7.
        'raceline': 'raceline_a7.0.csv',
    },
}

# Which track the stack uses when nothing says otherwise. One environment
# variable switches the whole stack back:  RACER_TRACK=porto
TRACK = os.environ.get('RACER_TRACK', 'icra')


def track_spec(track=None):
    """The row for `track`, or for the current TRACK. Raises with the list."""
    name = track or TRACK
    if name not in TRACKS:
        raise KeyError(f'unknown track {name!r}; known: {sorted(TRACKS)}. Add a '
                       'row to roboracer_stack.common.frames.TRACKS.')
    return TRACKS[name]


def raceline_path(name, track=None):
    """Resolve a racing line named on the command line.

    Anything holding a path separator is returned untouched, so an absolute
    path still means exactly what it says. A BARE FILENAME is resolved inside
    the current track's raceline directory -- with eleven lines shipped for
    icra alone, choosing one should not mean typing an install path that moves
    between the source tree and the container share.

        path_csv:=raceline_a4.0.csv      ->  <raceline dir>/icra/raceline_a4.0.csv
        path_csv:=/tmp/experiment.csv    ->  unchanged
    """
    if not name or os.sep in name:
        return name
    # The track's own subdirectory, which is '' for a track whose lines sit at
    # the top level of raceline/ (Porto).
    subdir = os.path.dirname(track_spec(track)['raceline'])
    d = os.path.join(RACELINE_DIR, subdir)
    path = os.path.join(d, name)
    if not os.path.isfile(path):
        # Fail HERE, at launch, naming what is available. A bare name that
        # resolves to nothing otherwise reaches pure_pursuit and surfaces as a
        # numpy read error on a path the operator never typed -- and the usual
        # cause is asking a track for a line that belongs to the other one.
        avail = sorted(f for f in os.listdir(d) if f.endswith('.csv')) \
            if os.path.isdir(d) else []
        raise FileNotFoundError(
            f'no raceline {name!r} for track {track or TRACK!r} in {d}. '
            f'Available: {avail}')
    return path


_spec = track_spec()

DEFAULT_MAP_YAML = os.path.join(MAPS_DIR, _spec['map_yaml'])
# BASE PATH, NO EXTENSION -- slam_toolbox appends .posegraph and .data itself.
DEFAULT_POSE_GRAPH = os.path.join(MAPS_DIR, _spec['pose_graph'])
# Override without rebuilding:  race.launch.py path_csv:=<abs path>
DEFAULT_RACELINE = os.path.join(RACELINE_DIR, _spec['raceline'])

# The grid the scan-fit diagnostics score against: a BASE PATH with no
# extension, since they append .pgm and .yaml themselves. It must be the grid
# the LOCALIZER is using or fit_ratio answers the wrong question, so this
# follows the track and points at AMCL's map. It used to be a hardcoded
# track_sm in two separate files -- Porto's pose-graph grid, which does not even
# exist under maps/icra/. Pass the pose-graph grid explicitly when debugging
# localizer:=slam on a track that ships one.
DEFAULT_FIT_MAP = os.path.splitext(DEFAULT_MAP_YAML)[0]

# ---- Spawn -----------------------------------------------------------------
# Fallback initial pose, used when the bootstrap seed never lands. Per track by
# construction -- see the table.
SPAWN_X, SPAWN_Y, SPAWN_YAW = _spec['spawn']

# ---- Vehicle ---------------------------------------------------------------
NS = '/autodrive/roboracer_1'
