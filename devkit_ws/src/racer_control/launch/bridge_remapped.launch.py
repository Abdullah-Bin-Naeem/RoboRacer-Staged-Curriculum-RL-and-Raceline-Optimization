"""Devkit bridge with its ground-truth TF remapped off /tf.

Use this instead of `autodrive_roboracer bringup_headless.launch.py` whenever
localization is running.

The bridge broadcasts world->roboracer_1 from the IPS. If that stays on /tf then
roboracer_1 has two parents -- world from the devkit and odom from dead
reckoning -- and the TF tree breaks. Remapping costs nothing: /tf is restricted
at race time anyway, and /tf_ground_truth stays available for measuring how good
the race-legal estimate actually is.

autodrive_devkit itself is not modified; this is a topic remap only.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tf_topic', default_value='/tf_ground_truth',
            description="where the devkit's ground-truth TF goes instead of /tf"),
        Node(
            package='autodrive_roboracer',
            executable='autodrive_bridge',
            name='autodrive_bridge',
            output='screen',
            emulate_tty=True,
            remappings=[('/tf', LaunchConfiguration('tf_topic'))],
        ),
    ])
