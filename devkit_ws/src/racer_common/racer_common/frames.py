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

# ---- Spawn -----------------------------------------------------------------
# Fallback initial pose. NOT (0, 0, 0): that cell is a WALL in track_clean.pgm
# and sits 0.71 m off the racing line, so a filter seeded there starts
# confidently inside a barrier. This is the nearest centreline point.
# Replace with the measured spawn if the car starts elsewhere:
#     ros2 topic echo /autodrive/roboracer_1/ips --once
SPAWN_X = '0.71'
SPAWN_Y = '0.02'
SPAWN_YAW = '-1.599'

# ---- Paths -----------------------------------------------------------------
# Track data is resolved at RUNTIME, in this order, and never from a path that
# assumes where the repo was cloned:
#
#   1. $RACER_MAPS_DIR / $RACER_RACELINE_DIR   explicit override
#   2. the installed package share             how the container finds it
#   3. the source tree beside this file        colcon build --symlink-install
#
# This used to be REPO = ~/Documents/roboracer, hard-coded. That directory does
# not exist in the submission container (the workspace is /home/racer_ws) and
# had already drifted from the checkout on the development machine, so every
# default here silently pointed at nothing and the follower launched with an
# empty path. Resolution order, not a constant.


def _data_dir(env_var, package, subdir):
    """First of: the env override, the installed share, the source tree."""
    override = os.environ.get(env_var)
    if override:
        return override

    try:
        from ament_index_python.packages import get_package_share_directory
        share = os.path.join(get_package_share_directory(package), subdir)
        if os.path.isdir(share):
            return share
    except Exception:
        # Not built / not on the ament index yet. Fall through rather than
        # raise: importing this module must never be what breaks a launch.
        pass

    # racer_common/racer_common/frames.py -> <workspace>/src
    src = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(src, package, subdir)


MAPS_DIR = _data_dir('RACER_MAPS_DIR', 'racer_mapping', 'maps')
RACELINE_DIR = _data_dir('RACER_RACELINE_DIR', 'racer_control', 'raceline')

DEFAULT_MAP_YAML = os.path.join(MAPS_DIR, 'track_clean.yaml')
# BASE PATH, NO EXTENSION -- slam_toolbox appends .posegraph and .data itself.
DEFAULT_POSE_GRAPH = os.path.join(MAPS_DIR, 'track_sm')
# Built by raceline/optimize_raceline.py on the main branch: min-curvature
# geometry with extra left margin on the straight after R1 and on the S-exit
# approach (--margin-zones 2.5:6:L:0.10,17:21.5:L:0.15), 0.15 m body-to-wall
# margin elsewhere; velocity profile at a_lat 7.0, a_long 5.0 m/s^2.
#
# THE QUALIFICATION DEFAULT. Measured on the car, AMCL + pure pursuit, run 38:
# 6.50 s best / 6.58 s mean over 7 clean laps, wall clearance 0.07 m left and
# 0.10 m right. Two other rungs ship beside it, same geometry:
#
#   raceline_a6.5.csv      run 40, 6.65 / 6.71 over 32 consecutive clean laps.
#                          The safe rung -- most margin. Use it if the
#                          evaluation machine runs the loop slow.
#   raceline_a7.0_rec.csv  run 41, 6.45 / 6.53 -- beats the 6.46 track record,
#                          but the S-exit clearance falls to 0.03 m. Deliberately
#                          NOT the default; see VEHICLE_MODEL.md section 7.
#
# Override without rebuilding:  race.launch.py path_csv:=<abs path>
DEFAULT_RACELINE = os.path.join(RACELINE_DIR, 'raceline_a7.0.csv')

# ---- Vehicle ---------------------------------------------------------------
NS = '/autodrive/roboracer_1'
