"""Localizer C: segmented scan-to-map localization against the occupancy grid.

    ros2 launch racer_localization v2.launch.py

Publishes map -> odom. That is ALL this file does -- it is one of three
interchangeable localizers and owns nothing else:

  odom -> roboracer_1 -> lidar    racer_localization/chassis.launch.py
  restricted instruments          racer_localization/instruments.launch.py
  RViz, bridge, follower          racer_bringup/race.launch.py

Normally reached as `race.launch.py localizer:=v2`; launchable alone for
debugging, provided chassis.launch.py is already up.

HOW IT DIFFERS FROM THE AMCL STACK
----------------------------------
no nav2_map_server      the node reads the PGM itself (scan_matcher.LikelihoodField)
no lifecycle_manager    a plain node
same bootstrap          localization_bootstrap serves it with the slam-style
                        parameters: ready_check:=tf (the bootstrap waits for
                        map->odom before seeding, so the node starts on a
                        fallback prior like slam's map_start_pose -- see
                        initial_x in the node, and the deadlock it fixes),
                        estimate_topic:=/localization_v2/pose, verify_via_tf.
                        Unlike slam it DOES offer both services the bootstrap
                        knows: /request_nomotion_update and
                        /reinitialize_global_localization (a 2-D search at the
                        IMU's heading), so bootstrap_mode:=global is race-legal.
shadow:=true            everything but the TF broadcast, for running beside
                        AMCL on the same live data (race.launch.py v2_shadow:=).

Race-legal: lidar against a pre-built map; odometry from dead_reckoning's TF.
The one development-only input is bootstrap_mode:=truth (one /ips read,
released), exactly as for AMCL.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from racer_common import frames
from racer_common.frames import BASE, MAP, ODOM, TRACK


def _nodes(context, *args, **kwargs):
    cfg = lambda n: LaunchConfiguration(n).perform(context)   # noqa: E731
    track = cfg('track')
    map_yaml = cfg('map_yaml') or frames.map_yaml(track)
    pkg_share = get_package_share_directory('racer_localization')
    shadow = cfg('shadow').lower() == 'true'

    return [
        Node(
            package='racer_localization', executable='localization_v2',
            name='localization_v2', output='screen', emulate_tty=True,
            parameters=[cfg('v2_params_file'), {
                'track': track,
                'map_yaml': map_yaml,
                'segments_csv': cfg('segments_csv'),
                'shadow': shadow,
                # The fallback prior; the bootstrap's seed refines it.
                'initial_x': cfg('initial_x'),
                'initial_y': cfg('initial_y'),
            }],
        ),

        # Seeds the pose and latches /localization_ready, as for AMCL and slam.
        # In shadow mode AMCL's own bootstrap does that job and this one must
        # not also publish /initialpose, so it is skipped.
        Node(
            package='racer_localization', executable='localization_bootstrap',
            name='localization_bootstrap', output='screen', emulate_tty=True,
            parameters=[{
                'mode': cfg('bootstrap_mode'),
                'track': track,
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
            condition=IfCondition(str(cfg('bootstrap').lower() == 'true' and not shadow).lower()),
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', os.path.join(pkg_share, 'config', 'v2.rviz')],
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('racer_localization')
    return LaunchDescription([
        DeclareLaunchArgument('v2_params_file',
                              default_value=os.path.join(pkg_share, 'config', 'localization_v2.yaml')),
        DeclareLaunchArgument('track', default_value=TRACK),
        DeclareLaunchArgument('map_yaml', default_value='', description="empty = the track's grid"),
        DeclareLaunchArgument('segments_csv', default_value='',
                              description="empty = raceline/<track>/segments.csv (tools/segment_track.py)"),
        DeclareLaunchArgument('shadow', default_value='false',
                              description='publish pose and status only; AMCL keeps map->odom'),
        DeclareLaunchArgument('bootstrap', default_value='true'),
        DeclareLaunchArgument('bootstrap_mode', default_value='truth',
                              description="'truth' seeds once from /ips (warmup-legal); "
                                          "'global' runs the 2-D search, no ground truth at all"),
        DeclareLaunchArgument('require_convergence', default_value='true'),
        DeclareLaunchArgument('recover', default_value='true'),
        DeclareLaunchArgument('recover_use_checkpoints', default_value='true'),
        DeclareLaunchArgument('recover_settle_s', default_value='1.0'),
        DeclareLaunchArgument('recover_creep_s', default_value='1.0'),
        # The fallback prior the node starts on so that map->odom exists before
        # the bootstrap's seed (see the node's initial_x comment). Empty = the
        # track's registered spawn.
        DeclareLaunchArgument('initial_x', default_value=''),
        DeclareLaunchArgument('initial_y', default_value=''),
        DeclareLaunchArgument('initial_yaw', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_nodes),
    ])
