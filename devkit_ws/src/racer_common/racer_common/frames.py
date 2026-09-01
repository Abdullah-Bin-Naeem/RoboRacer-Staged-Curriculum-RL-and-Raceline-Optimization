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
REPO = os.path.join(os.path.expanduser('~'), 'Documents/roboracer')
MAPS_DIR = os.path.join(REPO, 'devkit_ws/src/racer_mapping/maps')
RACELINE_DIR = os.path.join(REPO, 'raceline')

DEFAULT_MAP_YAML = os.path.join(MAPS_DIR, 'track_clean.yaml')
# BASE PATH, NO EXTENSION -- slam_toolbox appends .posegraph and .data itself.
DEFAULT_POSE_GRAPH = os.path.join(MAPS_DIR, 'track_sm')
DEFAULT_RACELINE = os.path.join(RACELINE_DIR, 'raceline_scipy_a8.csv')

# ---- Vehicle ---------------------------------------------------------------
NS = '/autodrive/roboracer_1'
