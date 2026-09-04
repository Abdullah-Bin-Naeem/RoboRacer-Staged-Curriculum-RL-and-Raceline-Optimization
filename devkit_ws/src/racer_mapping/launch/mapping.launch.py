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
from launch_ros.parameter_descriptions import ParameterValue


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

        # Scan matching decides which artifacts the session yields, not the
        # quality of the grid: sim odometry is ground truth, so the occupancy
        # grid is rasterised at true poses either way and AMCL is unaffected.
        # karto builds the pose GRAPH only inside its scan-matching branch, so
        # scan_matching:=false gives a grid-only map (exactly how Porto's
        # track_clean.pgm was made) and a graph the slam localizer would
        # segfault on. Loop closure follows it: with no graph there is nothing
        # to close. Leave true when you want track_sm.* as well.
        DeclareLaunchArgument(
            'scan_matching',
            default_value='true',
            description='false = AMCL-only mapping: grid only, no pose graph',
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_scan_matching': ParameterValue(
                    LaunchConfiguration('scan_matching'), value_type=bool),
                 'do_loop_closing': ParameterValue(
                    LaunchConfiguration('scan_matching'), value_type=bool)},
            ],
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
