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
from launch_ros.parameter_descriptions import ParameterValue
from roboracer_stack.common.frames import BASE, LIDAR, LIDAR_XYZ, ODOM


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'dr_distance_source', default_value='encoder',
            description="'encoder' integrates the wheel angle; 'tire' integrates the "
                        'tire observer\'s car speed, which removes wheelspin and '
                        'braking under-read (common/tire_model.py)'),
        # How far the odom -> base transform is stamped into the future. It only
        # has to cover one 200 Hz publish gap so AMCL's lookup at the scan stamp
        # is bracketed; everything beyond that is yaw and position lead handed
        # to every reader that asks for the newest transform.
        DeclareLaunchArgument(
            'dr_transform_tolerance', default_value='0.005',   # KEEP IN STEP with dead_reckoning.py's own default
            description='seconds the odom -> base transform is post-dated'),
        # Encoders over-read under slip (they report the throttle command), so
        # <1.0 trims the systematic part. MEASURED per source: the integrated
        # distance against ground truth over a lap.
        DeclareLaunchArgument(
            'dr_distance_scale', default_value='1.0',   # KEEP IN STEP with dead_reckoning.py
            description='multiplies the integrated distance; <1 trims encoder over-read'),
        Node(
            package='roboracer_stack', executable='dead_reckoning',
            name='dead_reckoning', output='screen', emulate_tty=True,
            parameters=[{'odom_frame': ODOM, 'base_frame': BASE,
                         'distance_source': LaunchConfiguration('dr_distance_source'),
                         'transform_tolerance': ParameterValue(
                             LaunchConfiguration('dr_transform_tolerance'),
                             value_type=float),
                         'distance_scale': ParameterValue(
                             LaunchConfiguration('dr_distance_scale'),
                             value_type=float)}],
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
