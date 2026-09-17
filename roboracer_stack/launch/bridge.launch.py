"""Devkit bridge with its ground-truth TF remapped off /tf.

    ros2 launch roboracer_stack bridge.launch.py

Use this instead of `autodrive_roboracer bringup_headless.launch.py` whenever
localization is running.

The bridge broadcasts world->roboracer_1 from the IPS. If that stays on /tf then
roboracer_1 has two parents -- world from the devkit and odom from dead
reckoning -- and the TF tree breaks. Remapping costs nothing: /tf is restricted
at race time anyway, and /tf_ground_truth stays available for measuring how good
the race-legal estimate actually is.

autodrive_devkit itself is not modified; this is a topic remap only.

tcp_nodelay:=true preloads libnodelay.so (share/roboracer_stack/tools, built by
the Dockerfile from tools/nodelay.c) into the bridge process, which sets
TCP_NODELAY on every TCP socket it creates or accepts and re-arms TCP_QUICKACK
after every recv AND every send syscall on them. The simulator only emits
telemetry in reply to the bridge's message, over a websocket on loopback;
Nagle's algorithm holds each small write until the previous one is acknowledged
and the receiver's delayed ACK holds that acknowledgment for up to 40 ms, so the
loop runs at 10-20 Hz on any machine. Measured 2026-09-12 on the lidar topic,
same session: 18.6 Hz off, 77.3 Hz on (tools/nodelay.c has the table).
Still a launch-level change: the devkit's code is untouched, only the
environment the process starts with. (Ported from multi-track.)

loop_hz_cap:=45 (needs tcp_nodelay:=true) paces the bridge's replies so the
simulator loop runs at most that fast. The simulator only emits in reply, so
the whole loop follows. The organizers quote 40-50 Hz for the evaluation
machine; the cap is how the stack is tuned at that rate. 0 = uncapped.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

NODELAY_SHIM = os.path.join(get_package_share_directory('roboracer_stack'), 'tools', 'libnodelay.so')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tf_topic', default_value='/tf_ground_truth',
            description="where the devkit's ground-truth TF goes instead of /tf"),
        DeclareLaunchArgument(
            'tcp_nodelay', default_value='false',
            description='preload libnodelay.so into the bridge: TCP_NODELAY and a QUICKACK '
                        're-arm on its websocket, 18.6 -> 77 Hz; see the module docstring'),
        DeclareLaunchArgument(
            'loop_hz_cap', default_value='0',
            description='with tcp_nodelay: pace the bridge replies so the simulator loop runs at most '
                        'this many Hz (organizers: evaluation is 40-50); 0 = uncapped'),
        LogInfo(
            condition=IfCondition(LaunchConfiguration('tcp_nodelay')),
            msg=['[bridge] loop cap NODELAY_CAP_HZ=', LaunchConfiguration('loop_hz_cap'), ' (0 = uncapped)']),
        LogInfo(
            condition=IfCondition(LaunchConfiguration('tcp_nodelay')),
            msg=(f'[bridge] TCP_NODELAY via LD_PRELOAD={NODELAY_SHIM}'
                 if os.path.exists(NODELAY_SHIM) else
                 f'[bridge] WARNING: tcp_nodelay requested but {NODELAY_SHIM} is missing; '
                 'the Dockerfile builds it from roboracer_stack/tools/nodelay.c')),
        Node(
            package='autodrive_roboracer',
            executable='autodrive_bridge',
            name='autodrive_bridge',
            output='screen',
            emulate_tty=True,
            remappings=[('/tf', LaunchConfiguration('tf_topic'))],
            additional_env={
                'LD_PRELOAD': PythonExpression([
                    "'", NODELAY_SHIM, "' if '", LaunchConfiguration('tcp_nodelay'), "'.lower() == 'true' else ''"]),
                'NODELAY_CAP_HZ': LaunchConfiguration('loop_hz_cap'),
            },
        ),
    ])
