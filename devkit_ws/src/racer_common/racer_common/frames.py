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
# Built by raceline/optimize_raceline.py: min-curvature geometry with extra left
# margin on the straight after R1 and on the S-exit approach (--margin-zones
# 2.5:6:L:0.10,17:21.5:L:0.15), 0.15 m body-to-wall margin elsewhere, velocity
# profile at a_lat 6.5 m/s^2. That is over the tire's 4.90 asymptote and is held
# by the follower's curvature cap (steer_a_lat_max): measured clean on the car,
# 13 laps at 17.5 Hz, 6.85 s best. raceline_a6.0.csv is the safe rung,
# raceline_a7.0.csv the record attempt (6.65 s best); same geometry, ladder in
# VEHICLE_MODEL.md section 7.
DEFAULT_RACELINE = os.path.join(RACELINE_DIR, 'raceline_a6.5.csv')

# ---- Vehicle ---------------------------------------------------------------
NS = '/autodrive/roboracer_1'
