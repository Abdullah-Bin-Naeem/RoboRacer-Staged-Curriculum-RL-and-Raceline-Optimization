"""Localizer A: nav2 AMCL against the saved occupancy grid.

    ros2 launch roboracer_stack amcl.launch.py

Publishes map -> odom. That is ALL this file does -- it is one of two
interchangeable localizers and owns nothing else:

  odom -> roboracer_1 -> lidar    launch/chassis.launch.py
  restricted instruments          launch/instruments.launch.py
  RViz, bridge, follower          launch/race.launch.py

Normally reached as `race.launch.py localizer:=amcl`; launchable alone for
debugging, provided chassis.launch.py is already up.

Race-legal: lidar against a pre-built map. The one development-only input is
bootstrap_mode:=truth, which seeds the initial pose from /ips; bootstrap_mode:=global
searches with no prior and is legal.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from roboracer_stack.common.frames import DEFAULT_MAP_YAML, SPAWN_X, SPAWN_Y, SPAWN_YAW


def _nodes(context, *args, **kwargs):
    cfg = lambda n: LaunchConfiguration(n).perform(context)
    pkg_share = get_package_share_directory('roboracer_stack')

    amcl_overrides = {
        'set_initial_pose': True,
        'initial_pose.x': float(cfg('initial_x')),
        'initial_pose.y': float(cfg('initial_y')),
        'initial_pose.yaw': float(cfg('initial_yaw')),
    }

    return [
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
            package='roboracer_stack', executable='localization_bootstrap',
            name='localization_bootstrap', output='screen', emulate_tty=True,
            parameters=[{
                'mode': cfg('bootstrap_mode'),
                # AMCL is a lifecycle node and DISCARDS anything sent to
                # /initialpose before it is active, so the seed is a handshake:
                # wait for active, send, confirm, retry. See the node docstring.
                'ready_check': 'lifecycle',
                'localizer_node': 'amcl',
                'estimate_topic': '/amcl_pose',
                'require_convergence': cfg('require_convergence').lower() == 'true',
            }],
            condition=IfCondition(LaunchConfiguration('bootstrap')),
        ),

        # RViz belongs with the localizer, not only with the composition root.
        # It used to live in race.launch.py, which made `ros2 launch
        # roboracer_stack amcl.launch.py` -- the obvious command for debugging
        # one localizer -- come up blind. Defaults ON so the standalone command just works;
        # race.launch.py passes its own rviz:= through, so composing does not
        # produce two windows.
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'amcl.rviz')],
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('roboracer_stack')

    return LaunchDescription([
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=os.path.join(pkg_share, 'config', 'amcl.yaml')),
        DeclareLaunchArgument('map_yaml', default_value=DEFAULT_MAP_YAML),
        DeclareLaunchArgument(
            'bootstrap', default_value='true',
            description='drive forward until AMCL converges instead of being '
                        'given an initial pose'),
        DeclareLaunchArgument(
            'bootstrap_mode', default_value='truth',
            description="'truth' seeds the pose from /ips once (DEVELOPMENT); "
                        "'global' searches with no initial pose (race-legal)"),
        DeclareLaunchArgument(
            'require_convergence', default_value='true',
            description='refuse to latch /localization_ready unless the pose was '
                        'actually confirmed; false hands over regardless'),
        # Fallback pose, used only when the bootstrap seed is unavailable.
        DeclareLaunchArgument('initial_x', default_value=SPAWN_X),
        DeclareLaunchArgument('initial_y', default_value=SPAWN_Y),
        DeclareLaunchArgument('initial_yaw', default_value=SPAWN_YAW),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='open RViz with this localizer''s config; '
                        'race.launch.py passes its own value through'),
        OpaqueFunction(function=_nodes),
    ])
