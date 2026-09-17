"""The half of localization that is not a localizer.

    ros2 launch roboracer_stack chassis.launch.py

    odom -> roboracer_1 -> lidar

Encoder + IMU dead reckoning, and the lidar extrinsic the devkit normally
broadcasts. AMCL builds on top of this and does not own it, so it lives here
rather than in the localizer's launch file.

Race-legal: encoders, IMU and a constant transform. No ground truth.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from roboracer_stack.common.frames import BASE, LIDAR, LIDAR_XYZ, ODOM


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'dr_distance_source', default_value='encoder',
            description="'encoder' integrates the wheel angle; 'tire' integrates the "
                        'tire observer\'s car speed, which removes wheelspin and '
                        'braking under-read (common/tire_model.py)'),
        Node(
            package='roboracer_stack', executable='dead_reckoning',
            name='dead_reckoning', output='screen', emulate_tty=True,
            parameters=[{'odom_frame': ODOM, 'base_frame': BASE,
                         'distance_source': LaunchConfiguration('dr_distance_source')}],
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
