"""Every node that reads a RESTRICTED topic, in one file.

    ros2 launch racer_localization instruments.launch.py

Nothing here feeds the control path. Keeping them together means "is this run
race-legal" has a single mechanical answer -- did this file get included -- and
race.launch.py can withhold it wholesale with mode:=race.

  localization_error   estimate vs ground truth
  map_world_tf         map -> world identity, so RViz can draw the true pose
                       beside the estimate. /odom and /ips are published in the
                       devkit's `world` frame, which leaves /tf when the bridge
                       is remapped, so without this they cannot be rendered at
                       all. Identity is correct because the SLAM map was built
                       against ground-truth odometry: map coordinates ARE world
                       coordinates. Measured 0.027 m mean wall agreement.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from racer_common.frames import BASE, MAP, WORLD


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_tf', default_value='true',
            description='read the estimate from TF map->base rather than a pose '
                        'topic. Required for slam_toolbox, which publishes no '
                        '/amcl_pose.'),
        DeclareLaunchArgument(
            'require_ready', default_value='false',
            description='wait for /localization_ready before sampling. Only the '
                        'AMCL stack latches it; slam_toolbox has no bootstrap.'),
        DeclareLaunchArgument('wall_margin', default_value='0.089',
                              description='clearance at the tightest point of the '
                                          'line being driven; 0.367 for the centreline'),

        Node(
            package='racer_localization', executable='localization_error',
            name='localization_error', output='screen', emulate_tty=True,
            parameters=[{
                'use_tf': LaunchConfiguration('use_tf'),
                'map_frame': MAP,
                'base_frame': BASE,
                'require_ready': LaunchConfiguration('require_ready'),
                'wall_margin_m': LaunchConfiguration('wall_margin'),
            }],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_world_tf', output='log',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                       '--frame-id', MAP, '--child-frame-id', WORLD],
        ),
    ])
