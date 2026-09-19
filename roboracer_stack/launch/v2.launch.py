"""The localizer: segmented scan-to-map matching against the occupancy grid.

    ros2 launch roboracer_stack v2.launch.py

Publishes map -> odom. That is ALL this file does; it owns nothing else:

  odom -> roboracer_1 -> lidar    launch/chassis.launch.py
  bridge and follower             launch/race.launch.py

Normally reached from race.launch.py; launchable alone for debugging, provided
chassis.launch.py is already up.

WHAT IT IS
----------
localization/localization_v2.py matches each lidar scan against a likelihood
field built from the shipped occupancy grid, on a per-segment basis: the track
is cut into segments (raceline/segments.csv) and each one carries its own
acceptance gates, because a scan on a long straight constrains the car across
the corridor but barely along it, and a scan in a hairpin constrains both.
Along-track blindness on the straights is what the AMCL stack could not fix.

It needs no nav2. There is no map server and no lifecycle manager: the node
reads the PGM itself, and it is a plain node, so nothing has to transition it
to `active` before it will work.

localization/bootstrap.py serves it with the slam-style handshake:
ready_check:=tf (wait for map->odom to appear, then seed), the estimate on
/localization_v2/pose, and verification through TF. It also owns recovery after
a wall contact, for which the node offers both services the bootstrap knows,
/request_nomotion_update and /reinitialize_global_localization.

Race-legal: lidar against a pre-built map, odometry from dead_reckoning's TF,
heading from the IMU. The default seed is the measured spawn constant, so no
restricted topic is read at any point.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from roboracer_stack.common import frames
from roboracer_stack.common.frames import BASE, MAP, ODOM, TRACK


def _nodes(context, *args, **kwargs):
    cfg = lambda n: LaunchConfiguration(n).perform(context)   # noqa: E731
    track = cfg('track')
    map_yaml = cfg('map_yaml') or frames.map_yaml(track)
    segments_csv = cfg('segments_csv') or frames.segments_csv(track)
    # The seed, and the fallback prior the node starts on before it arrives.
    # One source: an override on the command line moves both.
    spawn = frames.spawn(track)
    ix, iy, iyaw = (cfg(n) or spawn[i] for i, n in
                    enumerate(('initial_x', 'initial_y', 'initial_yaw')))

    return [
        Node(
            package='roboracer_stack', executable='localization_v2',
            name='localization_v2', output='screen', emulate_tty=True,
            parameters=[cfg('v2_params_file'), {
                'track': track,
                'map_yaml': map_yaml,
                'segments_csv': segments_csv,
                'shadow': False,
                # The fallback prior; the bootstrap's seed refines it.
                'initial_x': ix,
                'initial_y': iy,
            }],
        ),

        # Seeds the pose and latches /localization_ready, which the follower
        # waits on instead of a fixed timer.
        Node(
            package='roboracer_stack', executable='localization_bootstrap',
            name='localization_bootstrap', output='screen', emulate_tty=True,
            parameters=[{
                'mode': cfg('bootstrap_mode'),
                'track': track,
                # What bootstrap_mode:=spawn seeds from.
                'spawn_x': float(ix),
                'spawn_y': float(iy),
                'spawn_yaw': float(iyaw),
                'ready_check': 'tf',
                'estimate_topic': '/localization_v2/pose',
                'verify_via_tf': True,
                'map_frame': MAP,
                'odom_frame': ODOM,
                'base_frame': BASE,
                'nudge_service': '/request_nomotion_update',
                'global_service': '/reinitialize_global_localization',
                'require_convergence': cfg('require_convergence').lower() == 'true',
                'recover': cfg('recover').lower() == 'true',
                'recover_use_checkpoints': cfg('recover_use_checkpoints').lower() == 'true',
                'recover_settle_s': float(cfg('recover_settle_s')),
                'recover_creep_s': float(cfg('recover_creep_s')),
            }],
            condition=IfCondition(LaunchConfiguration('bootstrap')),
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('roboracer_stack')
    return LaunchDescription([
        DeclareLaunchArgument('v2_params_file',
                              default_value=os.path.join(pkg_share, 'config', 'localization_v2.yaml')),
        DeclareLaunchArgument('track', default_value=TRACK),
        DeclareLaunchArgument('map_yaml', default_value='', description="empty = the shipped grid"),
        DeclareLaunchArgument('segments_csv', default_value='',
                              description='empty = the shipped raceline/segments.csv'),
        DeclareLaunchArgument('bootstrap', default_value='true'),
        DeclareLaunchArgument(
            'bootstrap_mode', default_value='spawn',
            description="'spawn' seeds from the measured spawn constant and the IMU "
                        'heading, reading no restricted topic; '
                        "'global' runs the 2-D search with no prior at all"),
        DeclareLaunchArgument('require_convergence', default_value='true'),
        DeclareLaunchArgument('recover', default_value='true'),
        DeclareLaunchArgument('recover_use_checkpoints', default_value='true'),
        # This localizer confirms a recovery seed on its first scan and does not
        # tighten on motion, so AMCL's 2 s settle and 3 s lidar creep only cost
        # time -- and the creep drove the car into the wall at hairpin 2.
        DeclareLaunchArgument('recover_settle_s', default_value='1.0'),
        DeclareLaunchArgument('recover_creep_s', default_value='0.0'),
        # The fallback prior the node starts on so that map->odom exists before
        # the bootstrap's seed. Empty = the measured spawn.
        DeclareLaunchArgument('initial_x', default_value=''),
        DeclareLaunchArgument('initial_y', default_value=''),
        DeclareLaunchArgument('initial_yaw', default_value=''),
        OpaqueFunction(function=_nodes),
    ])
