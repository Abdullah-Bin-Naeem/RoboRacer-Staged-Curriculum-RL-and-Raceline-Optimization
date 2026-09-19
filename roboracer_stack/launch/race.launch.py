"""Everything except the simulator, in one command.

    ros2 launch roboracer_stack race.launch.py

This is the only launch file that composes others. Everything it starts is a
subsystem that owns itself and nothing else:

    launch/bridge.launch.py    devkit bridge, ground-truth TF remapped off /tf
    launch/chassis.launch.py   odom -> roboracer_1 -> lidar
    launch/v2.launch.py        map -> odom, the localizer
    launch/follower.launch.py  pure pursuit along the shipped line

THE DEFAULTS ARE THE SUBMISSION
-------------------------------
Run with no arguments and the car races the promoted configuration: the tb10
line, the localizer seeded from the measured spawn, and the follower arguments
in roboracer_stack.common.frames.FOLLOWER. 39 timed laps at 8.50-8.55 s with
zero contacts. Every argument below exists so one of those can be moved without
a rebuild; none of them has to be passed.

RACE MODE IS THE ONLY MODE
--------------------------
The development branch has a `mode` argument, because there it also has the
things mode:=race exists to switch off: a CSV logger that reads ground truth
continuously, lap and collision telemetry, a scan dump, and a diagnostic that
steers on the simulator's own pose. None of that is on this branch at all, so
there is no switch and nothing to leave in the wrong position.

What that means concretely, and what a steward can check in `rqt_graph`:

  - no node subscribes to /ips, /odom, /tf or any lap or collision topic
  - the devkit's ground-truth TF is remapped to /tf_ground_truth by
    bridge.launch.py, so nothing publishes /tf but our own dead reckoning and
    the localizer (that remap is also what keeps roboracer_1 from having two
    parents and breaking the TF tree)
  - the follower steers on the localizer's map->roboracer_1 correction
  - the initial pose is the measured spawn constant plus the IMU heading; no
    restricted topic is read to obtain it

See roboracer_stack/common/restricted.py, which every node checks itself
against at startup.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from roboracer_stack.common import frames
from roboracer_stack.common.frames import TRACK

# Passed straight through to pure_pursuit when given, and otherwise left to
# frames.FOLLOWER and then to config/pure_pursuit.yaml.
#
# curvature_preview_m matters more than it looks: with no v_mps column in the
# CSV, speed is derived from the worst curvature within this distance. Braking
# 8.0 -> 1.8 m/s takes ~6 m, so a 1 m preview means the car meets the corner at
# full speed. Irrelevant when the path carries its own velocity profile, which
# the shipped line does.
#
# KEEP IN STEP with launch/follower.launch.py: a name here that it does not
# declare is dropped silently on the way through.
TUNABLES = ('lookahead_min', 'lookahead_max', 'lookahead_k', 'lookahead_curv_gain', 'lookahead_sag_frac',
            'lookahead_delay_ref', 'derate_delay_from', 'derate_delay_to', 'derate_a_lat',
            'steer_a_lat_max', 'steer_excess_rad',
            'exit_guard_from', 'exit_guard_full', 'exit_slide_rate_m_s', 'exit_slide_hold_s',
            'lqr_k_lat', 'lqr_k_head', 'lqr_k_yaw', 'lqr_max_correction_rad',
            'v_max', 'a_lat_max', 'throttle_max', 'steering_gain',
            'warmup_v_max', 'warmup_dist_m',
            'recover_warmup_v_max', 'recover_warmup_dist_m',
            'curvature_preview_m',
            # Slip throttle and fused speed estimate (pure_pursuit.py docstring).
            # Tunable from the command line for the same reason as the rest:
            # limits are measured on the car, and a rebuild per attempt is tedious.
            'slip_accel', 'slip_brake', 'u_launch', 'u_per_throttle', 'v_slip_den', 'tire_rise_slope',
            'observer_wheels', 'slip_circle', 'accel_ff', 'drag_ff', 'brake_cap_margin',
            'cmd_delay_s', 'slip_kp', 'target_lead_s', 'enc_rate_window_s',
            'control_hz',
            'pose_speed_window', 'pose_speed_gain', 'pose_corr_max', 'imu_lever_arm', 'latency_comp_s',
            'speed_source', 'throttle_mode', 'steer_excess_ref', 'controller_mode',
            # legacy launch ramp (throttle_mode:=legacy only)
            'a_long_launch', 'a_long_launch_v')

# Published by localization_v2 every scan, with the posterior covariance. The
# follower reads TF rather than this topic (use_tf_pose), which is continuous
# rather than update-triggered; the topic is what the bootstrap verifies
# against.
POSE_TOPIC = '/localization_v2/pose'


def _launch(context, *args, **kwargs):
    cfg = lambda n: LaunchConfiguration(n).perform(context)   # noqa: E731

    # Track assets resolve from the package share unless a path was given.
    # Empty defaults rather than module constants, so an override is honoured.
    track = cfg('track')
    path_csv = cfg('path_csv') or frames.raceline(track)
    map_yaml = cfg('map_yaml') or frames.map_yaml(track)
    spawn = frames.spawn(track)
    initial = [cfg(n) or spawn[i] for i, n in
               enumerate(('initial_x', 'initial_y', 'initial_yaw'))]

    share = get_package_share_directory('roboracer_stack')
    src = lambda name: PythonLaunchDescriptionSource(   # noqa: E731
        os.path.join(share, 'launch', name))

    bootstrap_mode = cfg('bootstrap_mode')
    seed = ('the measured spawn constant' if bootstrap_mode == 'spawn'
            else 'a map-wide search, no prior')
    actions = [LogInfo(msg=(
        f'[race] track={track}  localizer=v2  line={os.path.basename(path_csv)}  '
        f'initial pose from {seed}'))]
    actions.append(LogInfo(msg=(
        '[race] RACE MODE: no node reads /ips, /odom, /tf or any lap or '
        'collision topic, and nothing is logged to disk')))

    # 1. bridge -- the devkit's, unmodified, with its ground-truth TF remapped.
    actions.append(IncludeLaunchDescription(
        src('bridge.launch.py'),
        condition=IfCondition(LaunchConfiguration('bridge')),
        launch_arguments={'tcp_nodelay': cfg('tcp_nodelay'),
                          'loop_hz_cap': cfg('loop_hz_cap')}.items(),
    ))

    # 2. chassis -- odom -> base -> lidar. The localizer builds on it and the
    #    follower's odometry comes through it.
    actions.append(IncludeLaunchDescription(
        src('chassis.launch.py'),
        launch_arguments={'distance_source': cfg('distance_source')}.items(),
        condition=IfCondition(LaunchConfiguration('chassis')),
    ))

    # 3. the localizer -- map -> odom
    actions.append(IncludeLaunchDescription(
        src('v2.launch.py'),
        launch_arguments={
            'track': track,
            'map_yaml': map_yaml,
            'segments_csv': cfg('segments_csv'),
            'v2_params_file': (cfg('v2_params_file') or
                               os.path.join(share, 'config', 'localization_v2.yaml')),
            'initial_x': initial[0],
            'initial_y': initial[1],
            'initial_yaw': initial[2],
            'bootstrap': cfg('bootstrap'),
            'bootstrap_mode': bootstrap_mode,
            'require_convergence': cfg('require_convergence'),
            'recover': cfg('recover'),
            'recover_use_checkpoints': cfg('recover_use_checkpoints'),
            'recover_settle_s': cfg('recover_settle_s'),
            'recover_creep_s': cfg('recover_creep_s'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('localization')),
    ))

    # /localization_ready is latched by the bootstrap, so it appears only when
    # the bootstrap is running. Anything that waits on it must agree, or it
    # waits forever.
    ready_latched = 'true' if cfg('bootstrap').lower() == 'true' else 'false'

    # 4. follower, on a delay so the localizer is publishing before it asks.
    #    With the bootstrap running it waits for /localization_ready instead.
    follower_args = {
        'path_csv': path_csv,
        'pose_topic': POSE_TOPIC,
        'use_tf_pose': 'true',
        'dev_lap_telemetry': 'false',
        'wait_for_ready': ready_latched,
        'bootstrap_seconds': '0.0',
    }
    follower_args.update({n: cfg(n) for n in TUNABLES if cfg(n) != ''})
    # The validated follower arguments (frames.FOLLOWER), each unless the same
    # name was given explicitly on the command line.
    for name, value in frames.follower_args(track).items():
        if name in TUNABLES and cfg(name) == '':
            follower_args[name] = value
    actions.append(TimerAction(
        period=float(cfg('follower_delay')),
        actions=[IncludeLaunchDescription(
            src('follower.launch.py'),
            launch_arguments=follower_args.items(),
            condition=IfCondition(LaunchConfiguration('follower')),
        )],
    ))

    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'track', default_value=TRACK,
            description='the only track this branch ships; here so the nodes '
                        'that take it are passed something explicit'),
        DeclareLaunchArgument(
            'path_csv', default_value='',
            description="raceline CSV; empty = the shipped line"),
        DeclareLaunchArgument(
            'map_yaml', default_value='',
            description='occupancy grid; empty = the shipped one'),
        DeclareLaunchArgument(
            'segments_csv', default_value='',
            description='track segmentation; empty = the shipped one'),
        DeclareLaunchArgument(
            'v2_params_file', default_value='',
            description='localizer yaml; empty = config/localization_v2.yaml'),

        # Turn pieces off when running them yourself.
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('chassis', default_value='true'),
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('follower', default_value='true'),

        # The bridge. Both default to the race setting: without TCP_NODELAY the
        # simulator loop runs at 10-20 Hz on any machine, and the cap is what
        # the stack is tuned at (the organizers quote 40-50 Hz).
        DeclareLaunchArgument(
            'tcp_nodelay', default_value='true',
            description='TCP_NODELAY and a QUICKACK re-arm on the bridge '
                        'websocket via LD_PRELOAD; see bridge.launch.py'),
        DeclareLaunchArgument(
            'loop_hz_cap', default_value='45',
            description='pace the bridge replies so the simulator loop runs at '
                        'most this many Hz (0 = uncapped)'),

        # Dead reckoning. 'slip' is the measured one: the encoders report the
        # throttle command rather than the wheel's travel, so distance has to
        # come from the tire observer with the wheelspin taken out.
        DeclareLaunchArgument(
            'distance_source', default_value='slip',
            description="dead reckoning distance source: 'slip', 'tire' or "
                        "'encoder'; see localization/dead_reckoning.py"),

        # The localizer's seed and its recovery after a wall contact.
        DeclareLaunchArgument('bootstrap', default_value='true'),
        DeclareLaunchArgument(
            'bootstrap_mode', default_value='spawn',
            description="'spawn' seeds from the measured spawn constant and the "
                        'IMU heading, reading no restricted topic; '
                        "'global' runs the map-wide search with no prior"),
        DeclareLaunchArgument(
            'require_convergence', default_value='true',
            description='park the follower rather than drive on a pose that was '
                        'never confirmed'),
        DeclareLaunchArgument(
            'recover', default_value='true',
            description='re-localize after a wall reset; see localization/bootstrap.py'),
        DeclareLaunchArgument(
            'recover_use_checkpoints', default_value='true',
            description='false forces the no-data recovery tier'),
        DeclareLaunchArgument('recover_settle_s', default_value='1.0',
                              description='pause after a reset seed before creep'),
        DeclareLaunchArgument('recover_creep_s', default_value='0.0',
                              description='gentle lidar creep after recovery; this '
                                          'localizer needs none and the creep used to '
                                          'drive the car into the wall at hairpin 2'),

        DeclareLaunchArgument(
            'follower_delay', default_value='2.0',
            description='fallback delay; with the bootstrap running the follower '
                        'waits for /localization_ready instead'),
        DeclareLaunchArgument('initial_x', default_value=''),
        DeclareLaunchArgument('initial_y', default_value=''),
        DeclareLaunchArgument('initial_yaw', default_value=''),
    ]
    args += [DeclareLaunchArgument(n, default_value='',
                                   description='override the validated value')
             for n in TUNABLES]

    return LaunchDescription(args + [OpaqueFunction(function=_launch)])
