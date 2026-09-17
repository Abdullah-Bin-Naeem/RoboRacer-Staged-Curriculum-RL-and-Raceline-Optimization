"""Everything except the simulator, in one command.

    ros2 launch roboracer_stack race.launch.py mode:=race    # the submission
    ros2 launch roboracer_stack race.launch.py               # dev mode

This is the ONLY file that composes others. Everything it starts is a
subsystem that owns itself and nothing else:

    launch/bridge.launch.py     devkit bridge, ground-truth TF remapped off /tf
    launch/chassis.launch.py    odom -> roboracer_1 -> lidar (dead reckoning)
    launch/amcl.launch.py       map -> odom (nav2 AMCL + localization_bootstrap)
    launch/follower.launch.py   pure pursuit

THE mode ARGUMENT
-----------------
`mode:=race` (what the entrypoint runs) makes the run legal with one switch:

    - dev_lap_telemetry off      (lap topics are restricted)
    - bootstrap_seconds:=0       (no ground-truth driving phase)
    - use_tf_pose:=true          (steer on the estimate, not on /odom)

`mode:=dev` keeps those development conveniences; any node that reads a
restricted topic says so at startup (common/restricted.py).

bootstrap_mode is not forced by mode. The default, `spawn`, seeds AMCL from the
measured spawn constant (common/frames.SPAWN_*) plus the IMU heading and reads
no restricted topic at all. `truth` reads /ips once inside the warm-up lap
(organizer-confirmed) and destroys the subscription; `global` starts AMCL with
no prior and pays a convergence phase.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from roboracer_stack.common.frames import (
    DEFAULT_MAP_YAML, DEFAULT_RACELINE, SPAWN_X, SPAWN_Y, SPAWN_YAW)

# Passed straight through to pure_pursuit when given.
# curvature_preview_m matters more than it looks: with no v_mps column in the
# CSV, speed is derived from the worst curvature within this distance. Braking
# 8.0 -> 1.8 m/s takes ~6 m, so a 1 m preview means the car meets the corner at
# full speed. Irrelevant when the path carries its own velocity profile.
#
# KEEP IN STEP with launch/follower.launch.py: a name here that it
# does not declare is dropped silently on the way through.
TUNABLES = ('lookahead_min', 'lookahead_max', 'lookahead_k', 'lookahead_curv_gain', 'lookahead_sag_frac',
            'lookahead_delay_ref', 'derate_delay_from', 'derate_delay_to', 'derate_a_lat',
            'steer_a_lat_max',
            'v_max', 'a_lat_max', 'throttle_max', 'steering_gain',
            'curvature_preview_m',
            # Slip throttle and fused speed estimate (pure_pursuit.py docstring).
            # Tunable from the command line for the same reason as the rest:
            # limits are measured on the car, and a rebuild per attempt is tedious.
            'slip_accel', 'slip_brake', 'u_launch', 'u_per_throttle', 'v_slip_den', 'tire_rise_slope',
            'cmd_delay_s', 'slip_kp', 'target_lead_s',
            # control loop rate; 20 matches the 17.5 Hz sim tick seen here, raise it
            # with the tick (headless sim, faster machine) so the loop is not the limit
            'control_hz',
            'pose_speed_window', 'pose_speed_gain', 'pose_corr_max', 'imu_lever_arm', 'latency_comp_s',
            'speed_source', 'throttle_mode',
            # legacy launch ramp (throttle_mode:=legacy only)
            'a_long_launch', 'a_long_launch_v')

# Everything that is specific to the localizer, in one table. Adding another
# localizer means adding a row here and one launch file -- not editing
# conditionals scattered through this function.
LOCALIZERS = {
    'amcl': dict(
        launch='amcl.launch.py',
        # AMCL only publishes /amcl_pose when the filter resamples, so it is
        # low-rate and steps discontinuously. Kept as the follower's fallback.
        pose_topic='/amcl_pose',
        # AMCL's bootstrap latches /localization_ready; the follower waits on it.
        has_ready=True,
    ),
}


def _launch(context, *args, **kwargs):
    cfg = lambda n: LaunchConfiguration(n).perform(context)

    localizer = cfg('localizer').lower()
    if localizer not in LOCALIZERS and localizer != 'none':
        raise RuntimeError(
            f"localizer:={localizer} is not one of "
            f"{sorted(LOCALIZERS) + ['none']}")
    spec = LOCALIZERS.get(localizer)

    race_mode = cfg('mode').lower() == 'race'
    # With no localizer there is no map->base, so the follower's TF lookup fails
    # and it falls back to pose_topic -- the devkit's ground-truth /odom by
    # default. That is the ground-truth A/B run (LOCALIZER.md), never a race.
    if race_mode and spec is None:
        raise RuntimeError(
            'mode:=race needs a localizer; localizer:=none steers on pose_topic, '
            'which is ground truth. Use mode:=dev for the ground-truth A/B run.')
    # In race mode the legal value wins over whatever was passed, so that a
    # stale flag on the command line cannot quietly make the run illegal.
    dev_lap = 'false' if race_mode else cfg('dev_lap_telemetry')
    bootstrap_seconds = '0.0' if race_mode else cfg('bootstrap_seconds')
    use_tf_pose = 'true' if race_mode else cfg('use_tf_pose')
    # NOT forced. bootstrap_mode:=truth is race-legal: the first lap is a warmup
    # and the timer starts after it, so the one-shot /ips read that seeds the
    # localizer happens inside the permitted window, and localization_bootstrap
    # destroys the subscription the moment the seed resolves. See
    # common/restricted.py, THE WARMUP WINDOW.
    #
    # Forcing 'global' would only cost a convergence phase; the seed is legal,
    # so it is not forced.
    bootstrap_mode = cfg('bootstrap_mode').lower()
    if bootstrap_mode not in ('spawn', 'truth', 'global'):
        raise RuntimeError(
            f"bootstrap_mode:={bootstrap_mode} is not one of ['spawn', 'truth', 'global']")

    loc_share = get_package_share_directory('roboracer_stack')
    ctl_share = get_package_share_directory('roboracer_stack')
    own_share = get_package_share_directory('roboracer_stack')
    src = lambda share, name: PythonLaunchDescriptionSource(
        os.path.join(share, 'launch', name))

    banner = (f'mode={cfg("mode")}  localizer={localizer}  '
              f'bootstrap={bootstrap_mode}')
    if race_mode:
        seeding = cfg('bootstrap').lower() == 'true'
        if seeding and bootstrap_mode == 'truth':
            how = ('; the initial pose is seeded once from /ips inside the warmup '
                   'window (organizer-confirmed), then released')
        elif seeding and bootstrap_mode == 'spawn':
            how = ('; the initial pose is seeded from the measured spawn constant '
                   '+ IMU heading -- no ground truth at all')
        else:
            how = '; no ground truth at all'
        banner += '  -- RACE MODE: nothing reads ground truth continuously' + how

    actions = [LogInfo(msg=f'[race] {banner}')]

    # 1. bridge
    actions.append(IncludeLaunchDescription(
        src(own_share, 'bridge.launch.py'),
        condition=IfCondition(LaunchConfiguration('bridge')),
    ))

    # 2. chassis -- odom -> base -> lidar. Needed by every localizer, and by the
    #    follower's odometry even when localizer:=none.
    actions.append(IncludeLaunchDescription(
        src(loc_share, 'chassis.launch.py'),
        launch_arguments={'dr_distance_source': cfg('dr_distance_source')}.items(),
        condition=IfCondition(LaunchConfiguration('chassis')),
    ))

    # 3. the chosen localizer -- map -> odom
    if spec is not None:
        # The seed handshake is now common to both localizers; only the map
        # format differs.
        loc_args = {'initial_x': cfg('initial_x'),
                    'initial_y': cfg('initial_y'),
                    'initial_yaw': cfg('initial_yaw'),
                    'bootstrap': cfg('bootstrap'),
                    'bootstrap_mode': bootstrap_mode,
                    'require_convergence': cfg('require_convergence')}
        loc_args['map_yaml'] = cfg('map_yaml')
        loc_args['path_csv'] = cfg('path_csv')

        actions.append(IncludeLaunchDescription(
            src(loc_share, spec['launch']),
            launch_arguments=loc_args.items(),
            condition=IfCondition(LaunchConfiguration('localization')),
        ))

    # /localization_ready is latched by the bootstrap node, so it appears only
    # when a localizer that HAS one is running AND the bootstrap is enabled.
    # Anything that waits on it must agree, or it waits forever.
    ready_latched = ('true' if (spec and spec['has_ready']
                                and cfg('bootstrap').lower() == 'true')
                     else 'false')

    # 4. follower, on a delay so the localizer is publishing before it asks
    follower_args = {
        'path_csv': cfg('path_csv'),
        'pose_topic': spec['pose_topic'] if spec else cfg('pose_topic'),
        'use_tf_pose': use_tf_pose,
        'dev_lap_telemetry': dev_lap,
        'wait_for_ready': ready_latched,
        'bootstrap_seconds': bootstrap_seconds,
    }
    follower_args.update({n: cfg(n) for n in TUNABLES if cfg(n) != ''})
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
            description="which map->odom source: 'amcl' or 'none'"),
        DeclareLaunchArgument(
            'mode', default_value='dev',
            description="'race' refuses every restricted topic and forces the "
                        "legal settings; 'dev' keeps the conveniences"),
        DeclareLaunchArgument('path_csv', default_value=DEFAULT_RACELINE),
        DeclareLaunchArgument(
            'dr_distance_source', default_value='encoder',
            description="dead reckoning distance: 'encoder' (wheel angle) or 'tire' "
                        '(tire-observer car speed; removes wheelspin/braking slip)'),
        DeclareLaunchArgument('map_yaml', default_value=DEFAULT_MAP_YAML,
                              description='occupancy grid, AMCL only'),

        # Turn pieces off when running them yourself.
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('chassis', default_value='true'),
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('follower', default_value='true'),

        # --- development-only; all forced off by mode:=race ---
        DeclareLaunchArgument('dev_lap_telemetry', default_value='true'),
        DeclareLaunchArgument('bootstrap', default_value='true'),
        DeclareLaunchArgument(
            'bootstrap_mode', default_value='spawn',
            description="'spawn' seeds from the measured spawn constant + IMU "
                        "heading and reads no restricted topic (RACE DEFAULT); "
                        "'truth' seeds once from /ips in the warm-up lap "
                        "(organizer-confirmed); 'global' searches with no prior"),
        DeclareLaunchArgument(
            'bootstrap_seconds', default_value='0.0',
            description='DIAGNOSTIC: drive on ground truth this long before '
                        'switching to the estimate. 0 = steer on the estimate '
                        'from the first command (the normal case).'),

        DeclareLaunchArgument(
            'use_tf_pose', default_value='true',
            description='follower reads TF map->roboracer_1 rather than the '
                        "localizer's pose topic. Measured better for AMCL "
                        '(0.177 m vs 0.192 m mean).'),
        DeclareLaunchArgument(
            'pose_topic', default_value='/amcl_pose',
            description='only consulted when localizer:=none; otherwise the '
                        'localizer table picks it'),
        DeclareLaunchArgument(
            'require_convergence', default_value='true',
            description='park the follower rather than drive on a pose that was '
                        'never confirmed'),
        DeclareLaunchArgument(
            'follower_delay', default_value='2.0',
            description='fallback delay; with an AMCL bootstrap the follower '
                        'waits for /localization_ready instead'),
        DeclareLaunchArgument('initial_x', default_value=SPAWN_X),
        DeclareLaunchArgument('initial_y', default_value=SPAWN_Y),
        DeclareLaunchArgument('initial_yaw', default_value=SPAWN_YAW),
    ]
    args += [DeclareLaunchArgument(n, default_value='',
                                   description='override the pure_pursuit value')
             for n in TUNABLES]

    return LaunchDescription(args + [OpaqueFunction(function=_launch)])
