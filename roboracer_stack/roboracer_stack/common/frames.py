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

# ---- Spawn -----------------------------------------------------------------
# The MEASURED spawn on the 2026-iros-compete track: the rear-axle pose the
# simulator reports while the car is still parked. Measured live on 2026-09-15
# with the stack not driving: x 0.8176 -> 0.8207 over a few minutes (slow creep),
# y 3.1583, orientation quaternion z -0.707 w 0.707 (yaw -90 deg). Practically
# the same point as the 2026-iros-practice Porto spawn (0.800, 3.158, -1.5707,
# 2026-09-10), in the new map's frame.
#
# This is what bootstrap_mode:=spawn seeds the localizer with (no /ips read at
# all), and the fallback for every other mode. It used to be (0.71, 0.02), a
# centreline point 3.14 m further down the same straight; the bootstrap warned
# about that in 17 logs and the truth seed hid it.
#
# Re-measure with the simulator connected and the car parked:
#     ros2 topic echo /autodrive/roboracer_1/ips --once
#     ros2 topic echo /autodrive/roboracer_1/imu --once --field orientation
SPAWN_X = '0.818'
SPAWN_Y = '3.158'
SPAWN_YAW = '-1.5707'

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
    # which is where maps/ and raceline/ live in the source tree.
    pkg = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    return os.path.join(pkg, subdir)


MAPS_DIR = _data_dir('RACER_MAPS_DIR', 'maps')
RACELINE_DIR = _data_dir('RACER_RACELINE_DIR', 'raceline')

DEFAULT_MAP_YAML = os.path.join(MAPS_DIR, 'track_clean.yaml')
# Built by raceline/optimize_raceline.py on the main branch: min-curvature
# geometry with extra left margin on the straight after R1 and on the S-exit
# approach (--margin-zones 2.5:6:L:0.10,17:21.5:L:0.15), 0.15 m body-to-wall
# margin elsewhere; velocity profile at a_lat 7.0, a_long 5.0 m/s^2.
#
# THE QUALIFICATION LINE, the only one shipped. Measured on the car, AMCL +
# pure pursuit: 6.45 s best over 500+ clean laps at 40-85 Hz on 2026-09-12; the
# line passes 0.25 m from the wall at its tightest point (the C2 apex, s 14.7).
# The other rungs (a6.5, the safe one; a7.0_rec, faster but 0.03 m of S-exit
# clearance) stay on the qualification_1_pure_pursuit branch.
DEFAULT_RACELINE = os.path.join(RACELINE_DIR, 'raceline_a7.0_edit_10.csv')

# ---- Vehicle ---------------------------------------------------------------
NS = '/autodrive/roboracer_1'
