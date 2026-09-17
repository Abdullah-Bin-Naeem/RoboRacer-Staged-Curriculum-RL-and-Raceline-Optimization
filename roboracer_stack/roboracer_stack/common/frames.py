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
#
# IROS 2026 (ported from the multi-track branch, 2026-09-17): the default is now
# raceline_tum_iqp_h7.0_a7.0b.csv -- tum_iqp geometry (kappa_bound 1.2, margin
# zones 19.3:23:0.30:R,24:29:0.15:R,36.5:39:0.12:R,42.3:3.7:0.30:R), profile
# a_lat 7.0, a_long 5.0, a_brake 5.5, v_max 8.5. Run 17: 20 clean laps at
# 9.55-9.70 s on a 45 Hz loop; run 18 with slip_circle 0.12 + accel_ff 1.0:
# 19 clean laps, best 9.45. Body-corner clearance min 0.071 m at s 39.6-40.2.
# raceline_a7.0_edit_10.csv (the previous default) is still shipped.
DEFAULT_RACELINE = os.path.join(RACELINE_DIR, 'raceline_tum_iqp_h7.0_a7.0b.csv')

# The track's centreline, used by localization_bootstrap to ORDER the
# checkpoints along the lap after a wall reset. Its s runs AGAINST the lap on
# this track; the bootstrap reverses it against the spawn heading.
CENTRELINE_CSV = os.path.join(RACELINE_DIR, 'centerline_full.csv')

# ---- Reset checkpoints -------------------------------------------------------
# Where the simulator put the car back after a wall contact, read from logged
# runs 1-14 on IROS 2026 (multi-track branch, 2026-09-16/17), ordered along the
# lap and annotated with centerline_full.csv's s (which runs against the lap).
# Every pose within 0.07 m of the centreline except the two marked. The encoder
# does not collapse on a reset here (it reads the throttle command); only the
# IMU heading step catches one. localization_bootstrap re-seeds AMCL at the one
# behind the last trusted pose, and prints a CHECKPOINT CANDIDATE for any reset
# it cannot match, so this list can grow.
CHECKPOINTS = [('0.800', '3.653', '-1.577'),      # s 44.9  start straight
               ('0.800', '0.647', '-1.578'),      # s 41.9
               ('0.800', '-2.348', '-1.575'),     # s 38.9
               ('0.800', '-5.346', '-1.570'),     # s 35.8
               ('0.800', '-8.348', '-1.572'),     # s 32.7
               ('0.800', '-11.340', '-1.571'),    # s 29.8
               ('0.718', '-15.575', '-0.600'),    # s 25.6  hairpin 1 entry (once, run 14, 0.13 m off)
               ('1.713', '-15.770', '0.268'),     # s 24.6  hairpin 1 exit (6 runs)
               ('3.018', '-13.624', '1.225'),     # s 22.2  after hairpin 1 (runs 8, 12)
               ('5.047', '-11.480', '1.585'),     # s 19.3  after the chicane (run 15, 5 resets, 0.28 m off)
               ('3.728', '-9.942', '2.356'),      # s 17.5  (once, run 3)
               ('2.500', '-6.660', '1.571'),      # s 13.8
               ('2.500', '-4.653', '1.574'),      # s 11.7
               ('3.525', '-1.396', '1.047'),      # s 8.2   (run 13)
               ('5.070', '0.121', '1.573')]       # s 6.1   (once, run 13, 0.29 m off)

# ---- Vehicle ---------------------------------------------------------------
NS = '/autodrive/roboracer_1'
