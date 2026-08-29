"""Everything except the simulator, in one command.

    ros2 launch racer_control race.launch.py

Starts, in order:

    1. the devkit bridge, with its ground-truth TF remapped to /tf_ground_truth
    2. localization  -- dead reckoning + AMCL against the SLAM map
    3. pure pursuit  -- following the raceline off the ESTIMATED pose

The follower starts on a delay, because AMCL needs a few seconds and a little
vehicle motion before its particle cloud converges. Handing it a pose that has
not settled makes the car chase a moving estimate.

Start the simulator first, then this. Anything already running can be excluded:

    ros2 launch racer_control race.launch.py bridge:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HOME = os.path.expanduser('~')
DEFAULT_PATH = os.path.join(HOME, 'Documents/roboracer/raceline/raceline_scipy_a8.csv')
DEFAULT_MAP = os.path.join(
    HOME, 'Documents/roboracer/devkit_ws/src/racer_mapping/maps/track_clean.yaml')

# Passed straight through to pure_pursuit when given.
# curvature_preview_m matters more than it looks: with no v_mps column in the
# CSV, speed is derived from the worst curvature within this distance. Braking
# 8.0 -> 1.8 m/s takes ~6 m, so a 1 m preview means the car meets the corner at
# full speed. Irrelevant when the path carries its own velocity profile.
TUNABLES = ('lookahead_min', 'lookahead_max', 'lookahead_k',
            'v_max', 'a_lat_max', 'throttle_max', 'steering_gain',
            'curvature_preview_m')


def _launch(context, *args, **kwargs):
    cfg = lambda n: LaunchConfiguration(n).perform(context)
    share = get_package_share_directory('racer_control')
    inc = lambda name: PythonLaunchDescriptionSource(
        os.path.join(share, 'launch', name))

    follower_args = {
        'path_csv': cfg('path_csv'),
        'pose_topic': '/amcl_pose',
        # AMCL only publishes /amcl_pose when the filter resamples. Its real
        # output is the map->odom correction on /tf, which composed with
        # dead_reckoning's 50 Hz odom->base gives a pose that is both smooth
        # and drift-free. The topic stays configured as a fallback.
        'use_tf_pose': cfg('use_tf_pose'),
        'dev_lap_telemetry': cfg('dev_lap_telemetry'),
        'rviz': 'false',            # localization.launch.py owns the RViz window
        'map_publisher': 'false',   # nav2 map_server owns /map when AMCL runs
        'wait_for_ready': cfg('bootstrap'),   # hold until the pose has converged
        'bootstrap_seconds': cfg('bootstrap_seconds'),
    }
    follower_args.update({n: cfg(n) for n in TUNABLES if cfg(n) != ''})

    return [
        IncludeLaunchDescription(
            inc('bridge_remapped.launch.py'),
            condition=IfCondition(LaunchConfiguration('bridge')),
        ),

        IncludeLaunchDescription(
            inc('localization.launch.py'),
            launch_arguments={
                'map_yaml': cfg('map_yaml'),
                'rviz': cfg('rviz'),
                'measure_error': cfg('measure_error'),
                'initial_x': cfg('initial_x'),
                'initial_y': cfg('initial_y'),
                'initial_yaw': cfg('initial_yaw'),
                'bootstrap': cfg('bootstrap'),
                'bootstrap_mode': cfg('bootstrap_mode'),
                'require_convergence': cfg('require_convergence'),
                'wall_margin': cfg('wall_margin'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('localization')),
        ),

        TimerAction(
            period=float(cfg('follower_delay')),
            actions=[IncludeLaunchDescription(
                inc('pure_pursuit.launch.py'),
                launch_arguments=follower_args.items(),
                condition=IfCondition(LaunchConfiguration('follower')),
            )],
        ),
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('path_csv', default_value=DEFAULT_PATH),
        DeclareLaunchArgument('map_yaml', default_value=DEFAULT_MAP),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('dev_lap_telemetry', default_value='true'),
        DeclareLaunchArgument(
            'measure_error', default_value='true',
            description='compare AMCL against ground truth (RESTRICTED at race time)'),
        DeclareLaunchArgument(
            'follower_delay', default_value='2.0',
            description='fallback delay; with bootstrap:=true the follower waits '
                        'for /localization_ready instead'),
        # Turn pieces off when running them yourself.
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('follower', default_value='true'),
        DeclareLaunchArgument('bootstrap', default_value='true'),
        DeclareLaunchArgument('bootstrap_mode', default_value='truth'),
        # DEFAULT TRUE, on measurement. /amcl_pose is published only when the
        # filter resamples, so it is low-rate and steps discontinuously; TF
        # map->base composes AMCL's correction with dead_reckoning's 200 Hz
        # odom->base and interpolates between updates.
        #
        # This depends on map->odom being clean, which in turn depends on
        # dead_reckoning post-dating its TF: with the old 50 Hz / no-tolerance
        # publisher, 94% of scans landed ahead of the newest transform, AMCL
        # built map->odom from mistimed lookups, and TF measured 22.4 m mean
        # error against 0.28 m for the topic. After the fix, on the same track:
        #     TF map->base   mean 0.177 m    /amcl_pose  mean 0.192 m
        # If map->odom ever starts drifting again (diagnose_localization.py
        # shows its path in the thousands), set this false and fix the skew.
        DeclareLaunchArgument(
            'use_tf_pose', default_value='true',
            description='follower reads TF map->roboracer_1 rather than the '
                        '/amcl_pose topic; false falls back to the topic'),
        # 0.089 = optimized raceline clearance; pass 0.367 for the centerline.
        DeclareLaunchArgument('wall_margin', default_value='0.089'),
        DeclareLaunchArgument(
            'require_convergence', default_value='true',
            description='park the follower rather than drive on a pose that was '
                        'never confirmed; false restores the old behaviour'),
        # DEFAULT 0 -- no ground-truth driving phase at all.
        #
        # This existed to give AMCL time to converge before the follower trusted
        # it. That reasoning no longer holds: localization_bootstrap SEEDS the
        # filter with the true pose and refuses to latch /localization_ready
        # until it has confirmed AMCL adopted it (measured 1.4 cm / 0.1 deg on a
        # good seed). The filter is already converged when the follower starts,
        # so there is nothing to wait out -- the handover gap was just a sample
        # from the steady-state error distribution, not a settling measurement.
        #
        # Non-zero is still useful as a DIAGNOSTIC: driving on truth isolates
        # the controller from localization, which is how you tell "the follower
        # cannot hold the line" from "the follower is chasing a bad pose".
        # Ground truth is RESTRICTED at race time; leave this at 0 for any run
        # that is meant to be legal.
        DeclareLaunchArgument(
            'bootstrap_seconds', default_value='0.0',
            description='DIAGNOSTIC: drive on ground truth this long before '
                        'switching to the estimate. 0 = steer on the estimate '
                        'from the first command (the normal case).'),
        # Fallback only -- see localization.launch.py. (0, 0, 0) is a WALL cell.
        DeclareLaunchArgument('initial_x', default_value='0.71'),
        DeclareLaunchArgument('initial_y', default_value='0.02'),
        DeclareLaunchArgument('initial_yaw', default_value='-1.599'),
    ]
    args += [DeclareLaunchArgument(n, default_value='',
                                   description='override the pure_pursuit value')
             for n in TUNABLES]

    return LaunchDescription(args + [OpaqueFunction(function=_launch)])
