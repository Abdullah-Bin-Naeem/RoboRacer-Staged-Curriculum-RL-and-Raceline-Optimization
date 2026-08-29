"""Race-legal localization: lidar + IMU + encoders against the SLAM map.

    ros2 launch racer_control localization.launch.py

IMPORTANT -- the bridge must be started with its ground-truth TF remapped away:

    ros2 launch autodrive_roboracer bringup_headless.launch.py \\
        --ros-args -r /tf:=/tf_ground_truth

The devkit broadcasts world->roboracer_1 from the IPS. If that stays on /tf,
roboracer_1 gets two parents (world and odom) and the TF tree breaks. Remapping
it costs nothing -- /tf is restricted at race time anyway -- and /tf_ground_truth
remains available for measuring how well this stack actually works.

Resulting tree:

    map -> odom -> roboracer_1 -> lidar
     |       |          |
     |       |          +-- static_transform_publisher (sensor extrinsics)
     |       +-- racer_control/dead_reckoning  (encoders + IMU)
     +-- nav2 amcl                             (lidar + map)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HOME = os.path.expanduser('~')
DEFAULT_MAP = os.path.join(
    HOME, 'Documents/roboracer/devkit_ws/src/racer_mapping/maps/track_clean.yaml')

# From autodrive_bridge.py's TF broadcast -- constant sensor extrinsics we have
# to republish ourselves once the devkit's /tf is remapped away.
LIDAR_XYZ = ('0.2733', '0.0', '0.096')


def _nodes(context, *args, **kwargs):
    cfg = lambda n: LaunchConfiguration(n).perform(context)
    pkg_share = get_package_share_directory('racer_control')

    amcl_overrides = {
        'set_initial_pose': True,
        'initial_pose.x': float(cfg('initial_x')),
        'initial_pose.y': float(cfg('initial_y')),
        'initial_pose.yaw': float(cfg('initial_yaw')),
    }

    return [
        # odom -> roboracer_1, from race-legal sensors only
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

        Node(
            package='nav2_map_server', executable='map_server',
            name='map_server', output='screen',
            parameters=[cfg('amcl_params_file'), {'yaml_filename': cfg('map_yaml')}],
        ),

        Node(
            package='nav2_amcl', executable='amcl',
            name='amcl', output='screen', emulate_tty=True,
            parameters=[cfg('amcl_params_file'), amcl_overrides],
        ),

        # map_server and amcl are lifecycle nodes: without this they start up
        # UNCONFIGURED and silently do nothing.
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{'use_sim_time': False,
                         'autostart': True,
                         'node_names': ['map_server', 'amcl']}],
        ),

        # Creeps forward on lidar alone until the pose converges, then latches
        # /localization_ready. Replaces guessing an initial pose.
        Node(
            package='racer_control', executable='localization_bootstrap',
            name='localization_bootstrap', output='screen', emulate_tty=True,
            parameters=[{'mode': LaunchConfiguration('bootstrap_mode').perform(context)}],
            condition=IfCondition(LaunchConfiguration('bootstrap')),
        ),

        # Development instrument: AMCL pose vs ground truth.
        Node(
            package='racer_control', executable='localization_error',
            name='localization_error', output='screen', emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('measure_error')),
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', os.path.join(pkg_share, 'config', 'localization.rviz')],
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('racer_control')

    return LaunchDescription([
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=os.path.join(pkg_share, 'config', 'amcl.yaml')),
        DeclareLaunchArgument('map_yaml', default_value=DEFAULT_MAP),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'measure_error', default_value='true',
            description='compare against ground truth (RESTRICTED at race time)'),
        DeclareLaunchArgument(
            'bootstrap_mode', default_value='truth',
            description="'truth' seeds the pose from /ips once (development); "
                        "'global' searches with no initial pose (race-legal)"),
        DeclareLaunchArgument(
            'bootstrap', default_value='true',
            description='drive forward until AMCL converges instead of being '
                        'given an initial pose'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        OpaqueFunction(function=_nodes),
    ])
