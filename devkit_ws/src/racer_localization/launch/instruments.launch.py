"""Every node that reads a RESTRICTED topic, in one file.

    ros2 launch racer_localization instruments.launch.py

Nothing here feeds the control path. Keeping them together means "is this run
race-legal" has a single mechanical answer -- did this file get included -- and
race.launch.py can withhold it wholesale with mode:=race.

  localization_error   estimate vs ground truth
  log_localization     the same comparison, every sample, to a CSV -- off
                       unless log_csv:= names a file. localization_error prints
                       a magnitude every 5 s, which says something is wrong but
                       not WHICH WAY, nor what the car was doing when it went
                       wrong. The CSV carries the signed along/cross/yaw
                       decomposition plus yaw_rate so drift can be plotted and
                       correlated instead of eyeballed in RViz.
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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from racer_common.frames import BASE, MAP, TRACK, WORLD


def generate_launch_description():
    # Empty log_csv means "do not log", so the node is conditioned on the
    # string being non-empty rather than on a separate boolean flag -- one
    # argument, and naming the file is what turns it on.
    logging = IfCondition(PythonExpression(
        ["'", LaunchConfiguration('log_csv'), "' != ''"]))

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
        DeclareLaunchArgument(
            'log_csv', default_value='',
            description='write every sample to this CSV; empty disables it. '
                        'Relative paths land in the directory you launched from'),
        DeclareLaunchArgument('track', default_value=TRACK,
                              description='track whose grid the logger scores scans against'),
        DeclareLaunchArgument('log_rate', default_value='20.0',
                              description='CSV samples per second'),
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
        # Same comparison as localization_error, but every sample and to disk.
        # Writes to the directory the launch was started from.
        Node(
            package='racer_localization', executable='log_localization',
            name='log_localization', output='screen', emulate_tty=True,
            condition=logging,
            parameters=[{
                'out': LaunchConfiguration('log_csv'),
                # cast: log_rate:=50 arrives as an int and the node declares a double
                'rate': ParameterValue(LaunchConfiguration('log_rate'), value_type=float),
                'track': LaunchConfiguration('track'),
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
