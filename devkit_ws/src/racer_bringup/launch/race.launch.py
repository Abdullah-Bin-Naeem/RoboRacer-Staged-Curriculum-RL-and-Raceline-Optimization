"""Everything except the simulator, in one command.

    ros2 launch racer_bringup race.launch.py                    # AMCL
    ros2 launch racer_bringup race.launch.py localizer:=slam    # slam_toolbox
    ros2 launch racer_bringup race.launch.py mode:=race         # nothing restricted

This is the ONLY package that composes others. Everything it starts is a
subsystem that owns itself and nothing else:

    racer_bringup/bridge.launch.py            devkit bridge, ground-truth TF remapped
    racer_localization/chassis.launch.py      odom -> roboracer_1 -> lidar
    racer_localization/{amcl,slam}.launch.py  map -> odom          <- interchangeable
    racer_localization/instruments.launch.py  everything restricted
    racer_control/follower.launch.py          pure pursuit

Each localizer opens its own RViz with the config that matches it; `rviz:=` is
passed through, so composing never produces two windows.

THE localizer ARGUMENT
----------------------
Before this file existed in this form, `race.launch.py` included the AMCL launch
file by name, so the A/B the whole design is built around could not be run from
the top-level entry point at all -- you had to hand-compose three terminals. Doing
that by hand is how several runs ended up driving on ground truth without anyone
noticing. `localizer:=slam` is the entire fix.

    amcl    nav2 AMCL against track_clean.pgm; publishes /amcl_pose
    slam    slam_toolbox against the track_sm pose graph; publishes /pose.
            Seeded by the same localization_bootstrap as AMCL. It has NO global
            relocalization, so the one-shot truth seed is not a convenience
            here -- it is the only thing that gets it onto the right pose.
    none    no map -> odom at all. Only useful with mode:=dev, where the
            follower can still drive on ground truth.

THE mode ARGUMENT
-----------------
`mode:=race` is a single switch that makes the run legal, rather than nine
defaults that each have to be right:

    - instruments.launch.py is not included at all
    - dev_lap_telemetry off      (lap topics are restricted)
    - bootstrap_seconds:=0       (no ground-truth driving phase)
    - use_tf_pose:=true          (steer on the estimate, not on /odom)

`mode:=dev` (the default) leaves the development conveniences on and every node
reading a restricted topic says so at startup.

WHAT mode:=race DOES *NOT* FORCE, AND WHY
-----------------------------------------
bootstrap_mode. It used to be pinned to 'global', on the belief that any read of
/ips was illegal. It is not: the first lap is a warmup and the timer starts
after it, so seeding the initial pose from one /ips sample before the car moves
is inside the permitted window -- and localization_bootstrap destroys that
subscription as soon as the seed resolves, so nothing reads ground truth during
the timed laps. See racer_common/restricted.py, THE WARMUP WINDOW.

Pinning it was actively harmful. slam_toolbox has no global search at all, so
`mode:=race localizer:=slam` fell through to the hardcoded map_start_pose, and
slam seeds with a +-0.5 m correlative search -- an error larger than that could
never be recovered and stayed frozen for the whole run. The default is now
bootstrap_mode:=truth in both modes; pass bootstrap_mode:=global explicitly to
start with no prior at all (AMCL only).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from racer_common import frames
from racer_common.frames import TRACK

# Passed straight through to pure_pursuit when given.
# curvature_preview_m matters more than it looks: with no v_mps column in the
# CSV, speed is derived from the worst curvature within this distance. Braking
# 8.0 -> 1.8 m/s takes ~6 m, so a 1 m preview means the car meets the corner at
# full speed. Irrelevant when the path carries its own velocity profile.
#
# KEEP IN STEP with racer_control/launch/follower.launch.py: a name here that it
# does not declare is dropped silently on the way through.
TUNABLES = ('lookahead_min', 'lookahead_max', 'lookahead_k', 'lookahead_curv_gain', 'lookahead_sag_frac',
            'lookahead_delay_ref', 'derate_delay_from', 'derate_delay_to', 'derate_a_lat',
            'steer_a_lat_max', 'steer_excess_rad',
            'exit_guard_from', 'exit_guard_full',
            'lqr_k_lat', 'lqr_k_head', 'lqr_k_yaw', 'lqr_max_correction_rad',
            'v_max', 'a_lat_max', 'throttle_max', 'steering_gain',
            'warmup_v_max', 'warmup_dist_m',
            'curvature_preview_m',
            # Slip throttle and fused speed estimate (pure_pursuit.py docstring).
            # Tunable from the command line for the same reason as the rest:
            # limits are measured on the car, and a rebuild per attempt is tedious.
            'slip_accel', 'slip_brake', 'u_launch', 'u_per_throttle', 'v_slip_den', 'tire_rise_slope',
            'observer_wheels', 'slip_circle', 'accel_ff', 'drag_ff', 'brake_cap_margin',
            'cmd_delay_s', 'slip_kp', 'target_lead_s', 'enc_rate_window_s',
            # control loop rate; 20 matches the 17.5 Hz sim tick seen here, raise it
            # with the tick (headless sim, faster machine) so the loop is not the limit
            'control_hz',
            'pose_speed_window', 'pose_speed_gain', 'pose_corr_max', 'imu_lever_arm', 'latency_comp_s',
            'speed_source', 'throttle_mode', 'steer_excess_ref', 'controller_mode',
            # legacy launch ramp (throttle_mode:=legacy only)
            'a_long_launch', 'a_long_launch_v')

# Everything that differs between the two localizers, in one table. Adding a
# third localizer means adding a row here and one launch file -- not editing
# conditionals scattered through this function.
LOCALIZERS = {
    'amcl': dict(
        launch='amcl.launch.py',
        # AMCL only publishes /amcl_pose when the filter resamples, so it is
        # low-rate and steps discontinuously. Kept as the follower's fallback.
        pose_topic='/amcl_pose',
        # AMCL's bootstrap latches /localization_ready; the follower waits on it.
        has_ready=True,
        # Particles can be scattered across the map with no prior, which is the
        # race-legal way to start.
        has_global=True,
    ),
    'slam': dict(
        launch='slam.launch.py',
        # slam_toolbox publishes the same message type on /pose, VOLATILE.
        pose_topic='/pose',
        # localization_bootstrap now serves this localizer too, so the topic
        # does get latched. It used to be AMCL-only, which left slam living on
        # the hardcoded map_start_pose -- and slam_toolbox seeds with a +-0.5 m
        # correlative search, so a wrong constant was frozen for the whole run.
        has_ready=True,
        # But it has NO global search to fall back on, so mode:=race starts it
        # on racer_common.frames.SPAWN_* and nothing else.
        has_global=False,
    ),
}


def _launch(context, *args, **kwargs):
    cfg = lambda n: LaunchConfiguration(n).perform(context)

    # Track assets resolve from track:= unless the path was given explicitly.
    # Empty defaults rather than module constants, because the constants are
    # computed at import time for frames.TRACK and would ignore track:=.
    track = cfg('track')
    path_csv = cfg('path_csv') or frames.raceline(track)
    map_yaml = cfg('map_yaml') or frames.map_yaml(track)
    map_graph = cfg('map_graph') or frames.pose_graph(track)
    spawn = frames.spawn(track)
    initial = [cfg(n) or spawn[i] for i, n in
               enumerate(('initial_x', 'initial_y', 'initial_yaw'))]

    localizer = cfg('localizer').lower()
    if localizer not in LOCALIZERS and localizer != 'none':
        raise RuntimeError(
            f"localizer:={localizer} is not one of "
            f"{sorted(LOCALIZERS) + ['none']}")
    spec = LOCALIZERS.get(localizer)

    race_mode = cfg('mode').lower() == 'race'
    # In race mode the legal value wins over whatever was passed, so that a
    # stale flag on the command line cannot quietly make the run illegal.
    measure_error = 'false' if race_mode else cfg('measure_error')
    dev_lap = 'false' if race_mode else cfg('dev_lap_telemetry')
    bootstrap_seconds = '0.0' if race_mode else cfg('bootstrap_seconds')
    use_tf_pose = 'true' if race_mode else cfg('use_tf_pose')
    # DIAGNOSTIC ONLY. Steer on the simulator's own pose instead of the
    # localizer's, so a run separates "the controller cannot follow this line"
    # from "the estimate is not good enough to follow it". Nothing else changes:
    # the localizer still runs, so its error is still logged beside a car that
    # is not using it. Refused in race mode, and the follower prints
    # restricted.warn() on top of the line printed here.
    drive_on_truth = (not race_mode) and cfg('drive_on_truth').lower() == 'true'
    # NOT forced. bootstrap_mode:=truth is race-legal: the first lap is a warmup
    # and the timer starts after it, so the one-shot /ips read that seeds the
    # localizer happens inside the permitted window, and localization_bootstrap
    # destroys the subscription the moment the seed resolves. See
    # racer_common/restricted.py, THE WARMUP WINDOW.
    #
    # Forcing 'global' here was the bug. slam_toolbox has NO global search, so
    # `mode:=race localizer:=slam` silently fell back to the hardcoded
    # map_start_pose -- and slam seeds with a +-0.5 m correlative search, so a
    # wrong constant was frozen for the entire timed run. The seed is both legal
    # and the only thing that makes that combination work.
    bootstrap_mode = cfg('bootstrap_mode')

    loc_share = get_package_share_directory('racer_localization')
    ctl_share = get_package_share_directory('racer_control')
    own_share = get_package_share_directory('racer_bringup')
    src = lambda share, name: PythonLaunchDescriptionSource(
        os.path.join(share, 'launch', name))

    banner = (f'mode={cfg("mode")}  localizer={localizer}  '
              f'instruments={"off" if measure_error == "false" else "ON"}  '
              f'bootstrap={bootstrap_mode}')
    if race_mode:
        banner += ('  -- RACE MODE: nothing reads ground truth continuously'
                   + ('; the initial pose is seeded once from /ips inside the '
                      'warmup window, then released'
                      if bootstrap_mode == 'truth' and
                      cfg('bootstrap').lower() == 'true'
                      else '; no ground truth at all'))

    actions = [LogInfo(msg=f'[race] {banner}')]

    # 1. bridge
    actions.append(IncludeLaunchDescription(
        src(own_share, 'bridge.launch.py'),
        condition=IfCondition(LaunchConfiguration('bridge')),
        launch_arguments={'tcp_nodelay': cfg('tcp_nodelay'), 'loop_hz_cap': cfg('loop_hz_cap')}.items(),
    ))

    # 2. chassis -- odom -> base -> lidar. Needed by every localizer, and by the
    #    follower's odometry even when localizer:=none.
    actions.append(IncludeLaunchDescription(
        src(loc_share, 'chassis.launch.py'),
        launch_arguments={'distance_source': cfg('distance_source')}.items(),
        condition=IfCondition(LaunchConfiguration('chassis')),
    ))

    # 3. the chosen localizer -- map -> odom
    if spec is not None:
        # The seed handshake is now common to both localizers; only the map
        # format differs.
        loc_args = {'track': track,
                    'initial_x': initial[0],
                    'initial_y': initial[1],
                    'initial_yaw': initial[2],
                    'bootstrap': cfg('bootstrap'),
                    'bootstrap_mode': bootstrap_mode,
                    'require_convergence': cfg('require_convergence'),
                    'recover': cfg('recover'),
                    'recover_use_checkpoints': cfg('recover_use_checkpoints'),
                    'recover_settle_s': cfg('recover_settle_s'),
                    'recover_creep_s': cfg('recover_creep_s')}
        loc_args['map_yaml' if localizer == 'amcl' else 'map_graph'] = (
            map_yaml if localizer == 'amcl' else map_graph)
        # A/B a localizer parameter set without touching the package's yaml.
        # Always pass a real path: launch configurations are inherited by the
        # include, so an empty value declared here would override amcl.launch.py's
        # own default and start map_server/amcl with no parameters (they hang).
        if localizer == 'amcl':
            loc_args['amcl_params_file'] = (cfg('amcl_params_file')
                                            or os.path.join(loc_share, 'config', 'amcl.yaml'))

        # Warn only when nothing will seed this localizer: no global search AND
        # no one-shot truth seed leaves it on the hardcoded constant, which
        # slam_toolbox's +-0.5 m search cannot correct.
        seeded = (cfg('bootstrap').lower() == 'true' and bootstrap_mode == 'truth')
        if not spec['has_global'] and not seeded:
            actions.append(LogInfo(msg=(
                f'[race] WARNING: {localizer} has no global relocalization and '
                'no truth seed, so it starts on racer_common.frames.SPAWN_* '
                f'({initial[0]}, {initial[1]}, {initial[2]}) '
                'and cannot recover an error larger than ~0.5 m. '
                'bootstrap_mode:=truth is race-legal and fixes this.')))
        # The localizer owns its own RViz (so that launching it standalone is
        # not blind); we just pass our value through rather than opening a
        # second window.
        loc_args['rviz'] = cfg('rviz')
        actions.append(IncludeLaunchDescription(
            src(loc_share, spec['launch']),
            launch_arguments=loc_args.items(),
            condition=IfCondition(LaunchConfiguration('localization')),
        ))

    # /localization_ready is latched by the bootstrap node, so it appears only
    # when a localizer that HAS one is running AND the bootstrap is enabled.
    # Anything that waits on it must agree, or it waits forever: with
    # bootstrap:=false, localization_error used to hold every sample silently.
    ready_latched = ('true' if (spec and spec['has_ready']
                                and cfg('bootstrap').lower() == 'true')
                     else 'false')

    # 4. instruments -- every restricted reader, together, omitted in race mode
    #    log_csv lives here because the CSV logger reads ground truth
    #    continuously, which is the pattern mode:=race must exclude wholesale.
    if cfg('log_csv') and measure_error == 'false':
        actions.append(LogInfo(msg=(
            f'[race] NOTE: log_csv:={cfg("log_csv")} is ignored -- the CSV '
            'logger reads ground truth continuously, so it lives in '
            'instruments.launch.py, which this mode omits. Use mode:=dev.')))

    actions.append(IncludeLaunchDescription(
        src(loc_share, 'instruments.launch.py'),
        launch_arguments={
            'use_tf': use_tf_pose,
            'require_ready': ready_latched,
            'wall_margin': cfg('wall_margin'),
            'log_csv': cfg('log_csv'),
            'log_rate': cfg('log_rate'),
            'track': track,
        }.items(),
        condition=IfCondition(measure_error),
    ))

    # 5. RViz -- only when no localizer is running, since each localizer opens
    #    its own with the config that matches it.
    if spec is None:
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', os.path.join(own_share, 'config', 'follow.rviz')],
        ))

    # 6. follower, on a delay so the localizer is publishing before it asks
    follower_args = {
        'path_csv': path_csv,
        'pose_topic': (f'{frames.NS}/odom' if drive_on_truth
                       else (spec['pose_topic'] if spec else cfg('pose_topic'))),
        'use_tf_pose': 'false' if drive_on_truth else use_tf_pose,
        'dev_lap_telemetry': dev_lap,
        'wait_for_ready': ready_latched,
        'bootstrap_seconds': bootstrap_seconds,
    }
    follower_args.update({n: cfg(n) for n in TUNABLES if cfg(n) != ''})
    # The track's validated follower arguments (frames.TRACKS[...]['follower']),
    # each unless the same name was given explicitly on the command line.
    for name, value in frames.follower_args(track).items():
        if name in TUNABLES and cfg(name) == '':
            follower_args[name] = value
    actions.append(TimerAction(
        period=float(cfg('follower_delay')),
        actions=[IncludeLaunchDescription(
            src(ctl_share, 'follower.launch.py'),
            launch_arguments=follower_args.items(),
            condition=IfCondition(LaunchConfiguration('follower')),
        )],
    ))

    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'localizer', default_value='amcl',
            description="which map->odom source: 'amcl', 'slam', or 'none'"),
        DeclareLaunchArgument(
            'mode', default_value='dev',
            description="'race' refuses every restricted topic and forces the "
                        "legal settings; 'dev' keeps the conveniences"),
        DeclareLaunchArgument(
            'track', default_value=TRACK,
            description='which track\'s map, raceline and spawn to use '
                        '(racer_common.frames.TRACKS)'),
        DeclareLaunchArgument(
            'path_csv', default_value='',
            description="raceline CSV; empty = the track's default line"),
        DeclareLaunchArgument(
            'map_yaml', default_value='',
            description="occupancy grid, AMCL only; empty = the track's"),
        DeclareLaunchArgument(
            'map_graph', default_value='',
            description="serialized pose graph, slam only; empty = the track's"),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('amcl_params_file', default_value='',
                              description='AMCL yaml to use instead of racer_localization/config/amcl.yaml (empty = the package default)'),

        # Turn pieces off when running them yourself.
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('tcp_nodelay', default_value='false',
                              description='TCP_NODELAY on the bridge websocket via LD_PRELOAD; see bridge.launch.py'),
        DeclareLaunchArgument('loop_hz_cap', default_value='0',
                              description='with tcp_nodelay, cap the simulator loop at this many Hz (0 = uncapped); see bridge.launch.py'),
        DeclareLaunchArgument('recover', default_value='true',
                              description='re-localize after a wall reset; see localization_bootstrap'),
        DeclareLaunchArgument('recover_use_checkpoints', default_value='true',
                              description='false forces the no-data recovery tier; see localization_bootstrap'),
        DeclareLaunchArgument('recover_settle_s', default_value='1.0',
                              description='pause after a reset seed before creep'),
        DeclareLaunchArgument('recover_creep_s', default_value='1.0',
                              description='gentle lidar creep duration after recovery'),
        DeclareLaunchArgument(
            'distance_source', default_value='encoder',
            description="dead reckoning distance source, 'encoder', 'tire' or 'slip'; see dead_reckoning.py"),
        DeclareLaunchArgument(
            'drive_on_truth', default_value='false',
            description='DIAGNOSTIC: steer on the simulator ground-truth pose instead of the '
                        'localizer. NOT race-legal, refused by mode:=race, and the follower says so '
                        'in red. Use it to separate controller error from localization error.'),
        DeclareLaunchArgument('chassis', default_value='true'),
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('follower', default_value='true'),

        # --- development-only; all forced off by mode:=race ---
        DeclareLaunchArgument(
            'measure_error', default_value='true',
            description='compare the estimate against ground truth (RESTRICTED)'),
        DeclareLaunchArgument('dev_lap_telemetry', default_value='true'),
        DeclareLaunchArgument('bootstrap', default_value='true'),
        DeclareLaunchArgument('bootstrap_mode', default_value='truth'),
        DeclareLaunchArgument(
            'bootstrap_seconds', default_value='0.0',
            description='DIAGNOSTIC: drive on ground truth this long before '
                        'switching to the estimate. 0 = steer on the estimate '
                        'from the first command (the normal case).'),

        DeclareLaunchArgument(
            'use_tf_pose', default_value='true',
            description='follower reads TF map->roboracer_1 rather than the '
                        "localizer's pose topic. Measured better for AMCL "
                        '(0.177 m vs 0.192 m mean) and REQUIRED for slam.'),
        DeclareLaunchArgument(
            'pose_topic', default_value='/amcl_pose',
            description='only consulted when localizer:=none; otherwise the '
                        'localizer table picks it'),
        DeclareLaunchArgument(
            'require_convergence', default_value='true',
            description='park the follower rather than drive on a pose that was '
                        'never confirmed'),
        # Body-to-wall clearance at the tightest point of the line being driven,
        # with the 0.27 m car width subtracted:
        #   centerline_full.csv 0.367     raceline_a4.5.csv  0.15 (optimize_raceline.py --safety)
        DeclareLaunchArgument(
            'log_csv', default_value='',
            description='write ground truth vs estimate to this CSV, one row '
                        'per sample; empty disables it. dev mode only'),
        DeclareLaunchArgument('log_rate', default_value='20.0',
                              description='CSV samples per second'),
        DeclareLaunchArgument('wall_margin', default_value='0.15'),
        DeclareLaunchArgument(
            'follower_delay', default_value='2.0',
            description='fallback delay; with an AMCL bootstrap the follower '
                        'waits for /localization_ready instead'),
        DeclareLaunchArgument('initial_x', default_value=''),
        DeclareLaunchArgument('initial_y', default_value=''),
        DeclareLaunchArgument('initial_yaw', default_value=''),
    ]
    args += [DeclareLaunchArgument(n, default_value='',
                                   description='override the pure_pursuit value')
             for n in TUNABLES]

    return LaunchDescription(args + [OpaqueFunction(function=_launch)])
