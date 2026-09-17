"""The half of localization that is not a localizer.

    ros2 launch racer_localization chassis.launch.py

    odom -> roboracer_1 -> lidar

Encoder + IMU dead reckoning, and the lidar extrinsic the devkit normally
broadcasts. Both AMCL and slam_toolbox build on top of this and neither owns it,
so it lives here rather than being copy-pasted into both -- which is exactly
what it was before this package existed.

Race-legal: encoders, IMU and a constant transform. No ground truth.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from racer_common.frames import BASE, LIDAR, LIDAR_XYZ, ODOM


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'distance_source', default_value='encoder',
            description="dead reckoning distance: 'encoder' (the wheel angle, which in "
                        "this sim is the throttle echo) or 'tire' (the tire observer's "
                        "car speed, which leaves the wheelspin out). See dead_reckoning.py."),
        Node(
            package='racer_localization', executable='dead_reckoning',
            name='dead_reckoning', output='screen', emulate_tty=True,
            parameters=[{'odom_frame': ODOM, 'base_frame': BASE,
                         'distance_source': LaunchConfiguration('distance_source')}],
        ),
        # Normally supplied by the devkit's /tf, which bridge.launch.py remaps
        # away so that roboracer_1 does not end up with two parents.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='lidar_tf', output='log',
            arguments=['--x', LIDAR_XYZ[0], '--y', LIDAR_XYZ[1], '--z', LIDAR_XYZ[2],
                       '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                       '--frame-id', BASE, '--child-frame-id', LIDAR],
        ),
    ])
