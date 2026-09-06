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
        'lat_zones': '',
        # Follower arguments this track was validated with; race.launch.py applies
        # them unless the same name is given on the command line. v_max 8.0:
        # Porto's straights top out at 7.3 in the profile and the car reached
        # 7.24. target_lead_s 0.08 is Porto's tuned value (short braking zones).
        'follower': {'v_max': '8.0', 'target_lead_s': '0.08'},
    },
    'icra2026': {
        # The pose the simulator resets the car to, read by the bootstrap on
        # the first run (2026-09-05). A first reading of (1.16, 1.82) was
        # where the car had been PARKED after the mapping laps, not the spawn:
        # read /ips right after a reset, not after driving. Coincides with
        # Porto's spawn; the scenes share a spawn point.
        'spawn': ('0.800', '3.158', '-1.5707'),
        # Climbed 4.0 -> 7.0 on the car (VEHICLE_MODEL section 7, ICRA runs
        # 2-9). 7.0 is the tire's limit, 6.5 ran 2 hits in 46 laps, both at
        # the two hairpins whose curvature (1.37, Porto's tightest is 0.92)
        # leaves only 8 % of lateral headroom at that rung. The submission
        # line is 6.5 with the hairpins capped at 6.0 by lat_zones below:
        # about 14 % headroom there, the rest of the lap untouched.
        # The hardened fast line: 7.0 rung, hairpins capped at 6.0 and T2 at
        # 6.5 (lat_zones), the main straight at 8 m/s and the rest held at 7 by
        # the LINE (--v-zones), plus a left margin zone at the T1 exit. Run 17:
        # 10 clean laps, 12.00 best / 12.05 mean, tightest 0.15 m at the
        # left-leg entry tip (s 43.8, unchanged on every line since run 8).
        # raceline_a6.5zv.csv is the previous submission (run 13, 12.15/12.22)
        # and the fallback if the fast line ever shows contact.
        # Run 20: the hardened fast line re-profiled with the LONGITUDINAL
        # budget climbed to accel 5.5 / brake 5.0 (--a-long 5.5 --a-brake 5.0)
        # and the follower's circle band + acceleration feedforward: 10 clean
        # laps, 11.80 best / 11.87 mean, tightest 0.13 m at the left-leg entry
        # tip. raceline_a7.0zv_hard.csv (run 17, 12.00/12.05) is the fallback
        # if the feedforward is ever suspect: it needs neither flag.
        # Run 23 (the submission): the 6.0 / 5.0 longitudinal rung with the main
        # straight at 9 m/s, T2 back to 7.0 and T3 zoned to 7.5 (its outer wall
        # is 0.45 m off the line, so understeer there runs into room), plus the
        # left-leg exit and T2 inner-wall margin zones: 11 clean laps, 11.60
        # best / 11.66 mean, nothing under 0.18 m, hairpin demand <= 6.1.
        # Regenerate with: --a-long 6.0 --a-brake 5.0 --v-max 7.0 --v-zones
        # 7.5:18.5:9.0 and the zones below. Fallbacks, all validated clean:
        # raceline_a7.0zv_hard_l5.5.csv (run 20, 11.80/11.87) and
        # raceline_a7.0zv_hard.csv (run 17, 12.00/12.05, needs no flags).
        'raceline': 'raceline_a7.0zv_hard_l6.0_corners_h.csv',
        'a_long': '6.0', 'a_brake': '5.0', 'v_zones': '7.5:18.5:9.0',
        # Per-corner lateral limits (--lat-zones s0:s1:a_lat), the 'z' lines.
        # Both hairpins: apex demand at 6.5 was measured up to 7.23 m/s^2
        # against a tire that gives 7.0.
        # T2 (s 17.5-24) at 6.5: on the plain 7.0 rung its measured demand
        # reached 7.06, the tire's limit, run 15.
        # T3 (s 24-30) at 7.5, above the rung: its outer wall is 0.45 m off the
        # line and the car measured 0.42-0.53 there, so understeer runs into
        # room; the combined limit settles it at 6.74 planned. T2's 6.5 cap
        # came off once its inner-wall margin zone gave it 0.26 m (it measured
        # 6.84 max at 0.27 m, run 23).
        'lat_zones': '37.5:43.5:6.0,43:47:6.0,24:30:7.5',
        # Placed from measurement, exactly as Porto's were: on the first logged
        # run (a4.0, 13 laps) the car exited the right-hand hairpin round the
        # left leg's tip 0.10-0.13 m to the left of the line on EVERY lap, and
        # the line ran 0.26 m from the outer wall there, leaving 0.02 m. Extra
        # left margin over that exit puts the line at 0.43 m and the car at
        # 0.32 m; the re-solved line is shorter and predicts 0.09 s faster.
        # Second zone, right side, s 34-38.5: the line runs along that lane's
        # right wall at the design margin, and at the 6.0 rung the car began
        # running 0.06-0.08 m wide toward it on the gentle left-hander, 0.07 m
        # of clearance (0.11 at 4.0 and 5.0, where it sat on the line). Per
        # sample, wall on the right, not a section average. The left has 1.2 m.
        # Third zone and a deeper first one, from the 7.0 rung: the tire's
        # limit shows at the left-leg hairpin, 0.19 m inside toward the tip on
        # entry (s 43.8, 0.11 m) and 0.32 m outside on exit (0.13 m with the
        # 0.20 zone). 6.5 needs neither; they are what makes 7.0 a fast line
        # rather than a gamble. NOTE: the entry zone at 43:45 does not bite at
        # 0.10 (the line is already 0.33 m off the tip, above the 0.285 m the
        # solver requires); ~0.22 would be needed to move the line there.
        # Fourth zone, left, s 8.5-11.5, the T1 exit onto the straight: at
        # 7 m/s the car drifts 0.19-0.23 m left of the line and had 0.11 m on
        # the plain 7.0 line (run 15). 0.15 did not bite (the line sat 0.47 m
        # from that wall, above what 0.15 asks); 0.35 moved it 0.127 m and the
        # car has 0.27 m there now (run 17).
        # Left-leg exit deepened 0.30 -> 0.45 after the 6.0 rung touched there on
        # run 21's warm-up lap (understeer wide, lateral +0.53): the line moved
        # 0.155 m and the car reads 0.42 (run 23). Entry tip R 0.10 -> 0.25 (the
        # amount that bites; the tip went 0.13 -> 0.18). T2 R 0.15 at s 21.5-23.5:
        # the line sat 0.13 m from T2's INNER wall, now 0.26, car 0.27.
        'margin_zones': '45.5:51.5:L:0.45,34:38.5:R:0.15,43:45:R:0.25,8.5:11.5:L:0.35,21.5:23.5:R:0.15',
        # v_max 8.0 is safe HERE ONLY because the zv line itself holds every
        # section but the main straight at 7: a global 8 (run 9) bought 0.03 s
        # and pushed the middle-wall hairpin's demand to 7.23, both hits were
        # at 8. lookahead_max 2.6 goes with 8 m/s. target_lead_s 0.0: the
        # 0.08 tuned on Porto put the speed target 1.8 m ahead in this track's
        # 7.8 m braking zones and cost 0.28 s a lap (runs 11 vs 13).
        # slip_circle 0.12: the band on the tire curve's flat top on straights,
        # scaled down by the friction circle in corners (0.06 at the hairpin
        # exits, less than the fixed 0.08). accel_ff: plan-acceleration
        # feedforward, acceleration side only (run 18 showed the brake side
        # costs apex speed). Together: plan delivery 91 -> 95-99 %.
        'follower': {'v_max': '8.0', 'lookahead_max': '2.6', 'target_lead_s': '0.0',
                     'slip_circle': '0.12', 'accel_ff': '1.0'},
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


def follower_args(track=None):
    """Follower arguments this track was validated with, name -> string.
    race.launch.py applies each unless the same name was given explicitly."""
    return dict(TRACKS[_track(track)].get('follower', {}))


def v_max(track=None):
    """The follower's validated speed cap for this track, as a string."""
    return follower_args(track).get('v_max', '8.0')


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
