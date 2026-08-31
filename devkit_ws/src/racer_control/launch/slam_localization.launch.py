"""Race-legal localization with slam_toolbox instead of AMCL.

    ros2 launch racer_control slam_localization.launch.py

The A/B partner to localization.launch.py. Same frames, same sensors, same
error instrument -- only the localizer changes, so a lap time or an error
number can be attributed to the algorithm rather than to the plumbing.

IMPORTANT -- the bridge must be started with its ground-truth TF remapped away:

    ros2 launch racer_control bridge_remapped.launch.py

Resulting tree, identical to the AMCL stack:

    map -> odom -> roboracer_1 -> lidar
     |       |          |
     |       |          +-- static_transform_publisher (sensor extrinsics)
     |       +-- racer_control/dead_reckoning  (encoders + IMU)
     +-- slam_toolbox, mode: localization      (lidar + pose graph)

WHAT IS DIFFERENT FROM THE AMCL STACK
-------------------------------------
no nav2_map_server      slam_toolbox publishes /map itself, from the pose graph
no lifecycle_manager    localization_slam_toolbox_node is a plain node
no localization_bootstrap
                        it polls /amcl/get_state and confirms via /amcl_pose,
                        neither of which exists here. The initial pose comes
                        from map_start_pose in the yaml instead. slam_toolbox
                        DOES accept /initialpose, so RViz's "2D Pose Estimate"
                        tool reseeds it by hand at any time.
no .pgm                 the map is the serialized pose graph, not an occupancy
                        grid -- map_yaml is not a parameter here

Because nothing latches /localization_ready without the bootstrap node,
localization_error runs with require_ready:=false and use_tf:=true.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HOME = os.path.expanduser('~')
# BASE PATH, NO EXTENSION -- slam_toolbox appends .posegraph and .data.
#
# track_sm, not track: `track` was mapped with use_scan_matching:=false, which
# in karto means AddVertex/AddEdges never ran, so track.posegraph holds a graph
# with zero vertices and localization mode segfaults on the first scan. track_sm
# is the re-map with scan matching on. track.* is kept because track_clean.pgm
# (AMCL's map) and the raceline CSVs were derived from it.
DEFAULT_GRAPH = os.path.join(
    HOME, 'Documents/roboracer/devkit_ws/src/racer_mapping/maps/track_sm')

# From autodrive_bridge.py's TF broadcast -- constant sensor extrinsics we have
# to republish ourselves once the devkit's /tf is remapped away.
LIDAR_XYZ = ('0.2733', '0.0', '0.096')


def _nodes(context, *args, **kwargs):
    cfg = lambda n: LaunchConfiguration(n).perform(context)
    pkg_share = get_package_share_directory('racer_control')

    slam_overrides = {
        'map_file_name': cfg('map_graph'),
        'map_start_pose': [float(cfg('initial_x')),
                           float(cfg('initial_y')),
                           float(cfg('initial_yaw'))],
    }

    return [
        # odom -> roboracer_1, from race-legal sensors only. Identical to the
        # AMCL stack -- this is the shared half of the A/B.
        Node(
            package='racer_control', executable='dead_reckoning',
            name='dead_reckoning', output='screen', emulate_tty=True,
            parameters=[{'odom_frame': 'odom', 'base_frame': 'roboracer_1'}],
        ),

        # roboracer_1 -> lidar, normally supplied by the devkit's /tf
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='lidar_tf', output='log',
            arguments=['--x', LIDAR_XYZ[0], '--y', LIDAR_XYZ[1], '--z', LIDAR_XYZ[2],
                       '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                       '--frame-id', 'roboracer_1', '--child-frame-id', 'lidar'],
        ),

        # map -> odom, by scan-matching the live scan against the stored graph.
        # If this executable is missing on your slam_toolbox build, the fallback
        # is `async_slam_toolbox_node` with the same params -- mode: localization
        # in the yaml is what actually selects the behaviour.
        Node(
            package='slam_toolbox', executable='localization_slam_toolbox_node',
            name='slam_toolbox', output='screen', emulate_tty=True,
            parameters=[cfg('slam_params_file'), slam_overrides],
        ),

        # Development instrument: estimate vs ground truth.
        # use_tf because slam_toolbox publishes no /amcl_pose; require_ready
        # because nothing latches /localization_ready without the bootstrap.
        Node(
            package='racer_control', executable='localization_error',
            name='localization_error', output='screen', emulate_tty=True,
            parameters=[{
                'use_tf': True,
                'map_frame': 'map',
                'base_frame': 'roboracer_1',
                'require_ready': False,
            }],
            condition=IfCondition(LaunchConfiguration('measure_error')),
        ),

        # map -> world, identity. Purely so RViz can DRAW the ground-truth pose
        # next to the estimate: /odom and /ips are published in the devkit's
        # `world` frame, which left /tf when the bridge was remapped.
        #
        # Identity is correct because the SLAM run had scan matching off against
        # ground-truth odometry, making slam_toolbox's correction the identity --
        # map coordinates ARE world coordinates.
        #
        # Tied to measure_error because it exists only to visualise a RESTRICTED
        # topic. It feeds nothing in the control path.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_world_tf', output='log',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                       '--frame-id', 'map', '--child-frame-id', 'world'],
            condition=IfCondition(LaunchConfiguration('measure_error')),
        ),

        # slam_localization.rviz, not localization.rviz: the AMCL config draws
        # the estimate from /amcl_pose, which only AMCL publishes. slam_toolbox
        # publishes the SAME message type on /pose (see slam_toolbox_common.cpp
        # pose_pub_) carrying the scan matcher's own covariance -- so this config
        # is the identical display pointed at the native topic, plus the
        # ParticleCloud dropped since there are none.
        #
        # /pose is VOLATILE, not latched: pose_pub_ is a plain depth-10
        # publisher. Subscribing transient_local is QoS-incompatible and RViz
        # silently draws nothing.
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', os.path.join(pkg_share, 'config',
                                          'slam_localization.rviz')],
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('racer_control')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_share, 'config', 'slam_localization.yaml')),
        DeclareLaunchArgument(
            'map_graph', default_value=DEFAULT_GRAPH,
            description='serialized pose graph, BASE PATH with no extension'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'measure_error', default_value='true',
            description='compare against ground truth (RESTRICTED at race time)'),
        # Starting pose for the graph. Same spawn the AMCL config falls back to.
        # Replace with the measured spawn if the car starts elsewhere:
        #     ros2 topic echo /autodrive/roboracer_1/ips --once
        DeclareLaunchArgument('initial_x', default_value='0.71'),
        DeclareLaunchArgument('initial_y', default_value='0.02'),
        DeclareLaunchArgument('initial_yaw', default_value='-1.599'),
        OpaqueFunction(function=_nodes),
    ])
