"""Run slam_toolbox against the AutoDRIVE devkit's existing frames.

Assumes the simulator and the devkit bridge are already up:
    ros2 launch autodrive_roboracer bringup_headless.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('racer_mapping')
    default_params = os.path.join(pkg_share, 'config', 'slam_mapping.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'mapping.rviz')

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='slam_toolbox parameter file',
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz preloaded with the map/scan/TF displays',
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file')],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', rviz_config],
        ),
    ])
