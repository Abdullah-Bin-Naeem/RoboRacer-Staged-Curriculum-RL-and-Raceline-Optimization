"""Frame names, sensor extrinsics, track constants and workspace paths.

Every launch file and every node imports from here rather than repeating
literals. Before this module the lidar extrinsic and the spawn pose were
duplicated across three launch files each, and drifted.

Launch files are ordinary Python, so
`from roboracer_stack.common.frames import BASE` works in them too.

ONE TRACK. The development branch carries a registry of tracks (Porto, ICRA
2026, IROS 2026) because the raceline tooling has to address all of them. This
branch is the submission and races exactly one, so the registry is gone and the
IROS 2026 row is spelled out below. The `track` arguments the nodes still pass
are accepted and checked, never used to pick between rows.
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

# ---- Vehicle ---------------------------------------------------------------
NS = '/autodrive/roboracer_1'

# ---- The track -------------------------------------------------------------
TRACK = 'iros2026'

# The MEASURED spawn on the IROS 2026 scene: the rear-axle pose the simulator
# reports while the car is still parked, on the long straight heading -y.
# Measured on this scene and stable to about 2 cm across resets.
#
# This is what bootstrap_mode:=spawn seeds the localizer with -- no ground
# truth topic is read at any point -- and the fallback for every other mode.
#
# Re-measure with the simulator connected and the car parked:
#     ros2 topic echo /autodrive/roboracer_1/ips --once
#     ros2 topic echo /autodrive/roboracer_1/imu --once --field orientation
SPAWN_X = '0.818'
SPAWN_Y = '3.158'
SPAWN_YAW = '-1.5707'

# Where the simulator puts the car after a wall contact: the last checkpoint
# behind it. Read off logged resets, ordered along the lap and annotated with
# the centreline's arc length (which on this track runs AGAINST the lap; the
# bootstrap reverses it). They are the recovery prior in
# localization/bootstrap.py; a reset the bootstrap cannot match to one of these
# is printed so the list can grow.
CHECKPOINTS = [
    ('0.800', '3.653', '-1.577'),      # s 44.9  start straight
    ('0.800', '0.647', '-1.578'),      # s 41.9
    ('0.800', '-2.348', '-1.575'),     # s 38.9
    ('0.800', '-5.346', '-1.570'),     # s 35.8
    ('0.800', '-8.348', '-1.572'),     # s 32.7
    ('0.800', '-11.340', '-1.571'),    # s 29.8
    ('0.718', '-15.575', '-0.600'),    # s 25.6  hairpin 1 entry
    ('1.713', '-15.770', '0.268'),     # s 24.6  hairpin 1 exit
    ('3.018', '-13.624', '1.225'),     # s 22.2  after hairpin 1
    ('5.047', '-11.480', '1.585'),     # s 19.3  after the chicane
    ('3.728', '-9.942', '2.356'),      # s 17.5
    ('2.500', '-6.660', '1.571'),      # s 13.8
    ('2.500', '-4.653', '1.574'),      # s 11.7
    ('3.525', '-1.396', '1.047'),      # s 8.2
    ('5.070', '0.121', '1.573'),       # s 6.1
    ('3.323', '2.124', '2.397'),       # s 3.5   before hairpin 2
    ('0.884', '4.715', '-2.511'),      # s 46.0  hairpin 2 exit, onto the straight
]

# THE RACE LINE, the only one shipped. The tb10 geometry at lateral 8.75,
# hairpins 7.25, brake 5.5. Measured on the car with this localizer: 39 timed
# laps at 8.50-8.55 s, zero contacts. Three follower settings are what made it
# hold, and each was measured rather than guessed -- the warmup cap released
# after the launch corner (28 m), the steering cap matched to the planned
# hairpin budget (7.5), and a 1.0 m lookahead floor, which took the hairpin-1
# exit slide to zero. They are the launch defaults in launch/race.launch.py.
RACELINE = 'rl_mt_tb07_z9_L70.csv'

# Follower arguments this track was validated with. race.launch.py applies each
# unless the same name was given explicitly on the command line.
FOLLOWER = {
    # Speed and the tracking pair validated on this car.
    'v_max': '9.0',
    'target_lead_s': '0.0',
    'slip_circle': '0.12',
    'accel_ff': '1.0',
    # The warmup cap, released after the launch corner. This track's spawn sits
    # at the top of the long straight, where the localizer has the least
    # along-track correction, so a run started from the spawn carries the
    # encoders' distance over-read all the way into the hairpin-1 braking
    # point. 28 m covers the straight AND the launch corner: releasing at 21
    # lifted the cap 5 m before the corner and lap 1 hit the wall there on
    # every localized run. The first lap is a warmup and the timer starts
    # after it, so this is free.
    'warmup_v_max': '2.0',
    'warmup_dist_m': '28.0',
    # The steering cap matched to the line's planned hairpin budget, and the
    # lookahead floor that took the hairpin-1 exit slide (one pass in 10-15 on
    # every faster line, steering peaking 0.82-0.89) to zero in 40 passes with
    # the steering peaking 0.75. The slide was understeer, not the localizer.
    'steer_a_lat_max': '7.5',
    'lookahead_min': '1.0',
    # What the follower measures for itself in this container over the first
    # two laps. Preset so laps 1-2 do not run 2-4 cm wide through the corners.
    'cmd_delay_s': '0.125',
    # Post-recovery speed cap, long enough to clear s 36-39.
    'recover_warmup_dist_m': '14.0',
    # Dead reckoning rate window, and the exit guard left off: the slide it
    # was written for is fixed by the lookahead floor above.
    'enc_rate_window_s': '0.10',
    'exit_guard_from': '0.0',
    'exit_guard_full': '0.0',
    # Hybrid LQR: pure pursuit with a small bounded lateral/heading correction.
    'controller_mode': 'hybrid_lqr',
    'lqr_k_lat': '0.03',
    'lqr_k_head': '0.05',
    'lqr_k_yaw': '0.0',
    'lqr_max_correction_rad': '0.02',
    # Control loop rate, matched to the bridge's 45 Hz cap so the loop is never
    # the limiter. pure_pursuit measures its own round trip and derates the
    # speed targets, so a slower simulator is handled rather than assumed away.
    'control_hz': '45',
}

# ---- Paths -----------------------------------------------------------------
PACKAGE = 'roboracer_stack'

# Track data is resolved at RUNTIME, in this order, and never from a path that
# assumes where the repo was cloned:
#
#   1. $RACER_MAPS_DIR / $RACER_RACELINE_DIR   explicit override
#   2. the installed package share             how the container finds it
#   3. the source tree beside this file        colcon build --symlink-install
#
# The development branch used REPO = ~/Documents/roboracer, hard-coded. That
# directory does not exist in the submission container, so every default here
# would silently point at nothing. Resolution order, not a constant.


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

    # <pkg>/roboracer_stack/common/frames.py -> <pkg>, which is where maps/,
    # raceline/ and tools/ live in the source tree. Three dirnames, not four:
    # the fourth lands on the repo root, where none of them exist.
    pkg = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    return os.path.join(pkg, subdir)


MAPS_DIR = _data_dir('RACER_MAPS_DIR', 'maps')
RACELINE_DIR = _data_dir('RACER_RACELINE_DIR', 'raceline')
TOOLS_DIR = _data_dir('RACER_TOOLS_DIR', 'tools')

# The LD_PRELOAD shim bridge.launch.py preloads into the devkit bridge. Built
# from tools/nodelay.c by the Dockerfile, into the installed share.
NODELAY_SHIM = os.environ.get(
    'RACER_NODELAY_SHIM', os.path.join(TOOLS_DIR, 'libnodelay.so'))


def _track(track=None):
    """Accept the `track` argument the nodes pass, and refuse a wrong one.

    One track ships on this branch; a launch that asks for another would
    otherwise silently get IROS 2026's map and line."""
    name = track or TRACK
    if name != TRACK:
        raise KeyError(
            f'unknown track {name!r}; this branch ships {TRACK!r} only. The '
            'multi-track registry lives on the development branch.')
    return name


def map_yaml(track=None):
    """Occupancy grid. Read directly by localization_v2 (no map server)."""
    _track(track)
    return os.path.join(MAPS_DIR, 'track_clean.yaml')


def raceline_dir(track=None):
    _track(track)
    return RACELINE_DIR


def raceline(track=None, name=None):
    """The shipped line, or `name` beside it."""
    _track(track)
    return os.path.join(RACELINE_DIR, name or RACELINE)


def segments_csv(track=None):
    """Track segmentation localization_v2 keys its per-segment gates off."""
    _track(track)
    return os.path.join(RACELINE_DIR, 'segments.csv')


def follower_args(track=None):
    """Follower arguments this track was validated with, name -> string.
    race.launch.py applies each unless the same name was given explicitly."""
    _track(track)
    return dict(FOLLOWER)


def v_max(track=None):
    """The follower's validated speed cap for this track, as a string."""
    return follower_args(track).get('v_max', '8.0')


def spawn(track=None):
    """(x, y, yaw) as strings, for launch arguments."""
    _track(track)
    return (SPAWN_X, SPAWN_Y, SPAWN_YAW)


def checkpoints(track=None):
    """Known reset poses [(x, y, yaw) floats]."""
    _track(track)
    return [tuple(float(v) for v in c) for c in CHECKPOINTS]


# Back-compatible module constants, resolved for the shipped track.
DEFAULT_MAP_YAML = map_yaml()
DEFAULT_RACELINE = raceline()
