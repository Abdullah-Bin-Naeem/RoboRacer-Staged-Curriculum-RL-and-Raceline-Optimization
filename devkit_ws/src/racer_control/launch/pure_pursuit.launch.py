"""Pure pursuit follower, with the map and RViz for watching it work.

Assumes the simulator and devkit bridge are already up:
    ros2 launch autodrive_roboracer bringup_headless.launch.py

Everything is visualised in the devkit's `world` frame. The mapping run had scan
matching disabled, so slam_toolbox's map->odom correction was identity and the
saved grid lines up with `world` without any static transform.

Tuning knobs are exposed as launch arguments so they can be overridden without
a rebuild, e.g.

    ros2 launch racer_control pure_pursuit.launch.py lookahead_k:=0.70
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HOME = os.path.expanduser('~')

# Overridable at launch time; empty string keeps whatever the params file says.
TUNABLES = ('lookahead_min', 'lookahead_max', 'lookahead_k',
            'v_max', 'a_lat_max', 'throttle_max', 'steering_gain')


def _nodes(context, *args, **kwargs):
    cfg = lambda n: LaunchConfiguration(n).perform(context)
    pkg_share = get_package_share_directory('racer_control')

    overrides = {n: float(cfg(n)) for n in TUNABLES if cfg(n) != ''}
    if overrides:
        print(f'[pure_pursuit] launch overrides: {overrides}')

    return [
        Node(
            package='racer_control',
            executable='pure_pursuit',
            name='pure_pursuit',
            output='screen',
            emulate_tty=True,
            parameters=[
                cfg('pp_params_file'),
                {'path_csv': cfg('path_csv'),
                 'pose_topic': cfg('pose_topic'),
                 'wait_for_ready': cfg('wait_for_ready').lower() == 'true',
                 'bootstrap_seconds': float(cfg('bootstrap_seconds')),
                 'dev_lap_telemetry': cfg('dev_lap_telemetry').lower() == 'true'},
                overrides,
            ],
        ),
        Node(
            package='racer_control',
            executable='map_publisher',
            name='map_publisher',
            condition=IfCondition(LaunchConfiguration('map_publisher')),
            output='screen',
            parameters=[{'map_yaml': cfg('map_yaml'), 'frame_id': 'world'}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', os.path.join(pkg_share, 'config', 'follow.rviz')],
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('racer_control')

    args = [
        DeclareLaunchArgument(
            'pp_params_file',
            default_value=os.path.join(pkg_share, 'config', 'pure_pursuit.yaml')),
        DeclareLaunchArgument(
            'path_csv',
            default_value=os.path.join(HOME, 'Documents/roboracer/raceline/centerline_full.csv'),
            description='path to follow (s,x,y,psi,kappa,w_r,w_l)'),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=os.path.join(
                HOME, 'Documents/roboracer/devkit_ws/src/racer_mapping/maps/track_clean.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        # Off when AMCL is running: nav2's map_server already publishes /map in
        # the `map` frame, and this one publishes `world`. Two publishers on one
        # topic means AMCL may latch the wrong frame.
        DeclareLaunchArgument('map_publisher', default_value='true'),
        DeclareLaunchArgument('wait_for_ready', default_value='false'),
        DeclareLaunchArgument('bootstrap_seconds', default_value='0.0'),
        DeclareLaunchArgument(
            'pose_topic', default_value='/autodrive/roboracer_1/odom',
            description='/amcl_pose for the race-legal estimate; the devkit odom '
                        'is ground truth and RESTRICTED at race time'),
        DeclareLaunchArgument('dev_lap_telemetry', default_value='false'),
    ]
    args += [DeclareLaunchArgument(n, default_value='',
                                   description='override the params file value')
             for n in TUNABLES]

    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])
