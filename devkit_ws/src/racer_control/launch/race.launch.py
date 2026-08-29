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
TUNABLES = ('lookahead_min', 'lookahead_max', 'lookahead_k',
            'v_max', 'a_lat_max', 'throttle_max', 'steering_gain')


def _launch(context, *args, **kwargs):
    cfg = lambda n: LaunchConfiguration(n).perform(context)
    share = get_package_share_directory('racer_control')
    inc = lambda name: PythonLaunchDescriptionSource(
        os.path.join(share, 'launch', name))

    follower_args = {
        'path_csv': cfg('path_csv'),
        'pose_topic': '/amcl_pose',
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
        DeclareLaunchArgument(
            'bootstrap_seconds', default_value='20.0',
            description='drive on ground truth this long while AMCL converges, '
                        'then switch to /amcl_pose. 0 disables.'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
    ]
    args += [DeclareLaunchArgument(n, default_value='',
                                   description='override the pure_pursuit value')
             for n in TUNABLES]

    return LaunchDescription(args + [OpaqueFunction(function=_launch)])
