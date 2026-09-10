"""Devkit bridge with its ground-truth TF remapped off /tf.

    ros2 launch racer_bringup bridge.launch.py

Use this instead of `autodrive_roboracer bringup_headless.launch.py` whenever
localization is running.

The bridge broadcasts world->roboracer_1 from the IPS. If that stays on /tf then
roboracer_1 has two parents -- world from the devkit and odom from dead
reckoning -- and the TF tree breaks. Remapping costs nothing: /tf is restricted
at race time anyway, and /tf_ground_truth stays available for measuring how good
the race-legal estimate actually is.

autodrive_devkit itself is not modified; this is a topic remap only.

tcp_nodelay:=true preloads tools/libnodelay.so into the bridge process, which
sets TCP_NODELAY on every TCP socket it creates or accepts. The simulator only
emits telemetry in reply to the bridge's message, over a websocket on loopback;
Nagle's algorithm holds each small write until the previous one is acknowledged
and the receiver's delayed ACK holds that acknowledgment for up to 40 ms, so the
loop runs at 10-20 Hz on any machine -- every rate measured here, and what
another team traced to this cause. Still a launch-level change: the devkit's
code is untouched, only the environment the process starts with.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from racer_common import frames

NODELAY_SHIM = os.path.join(frames.REPO, 'tools', 'libnodelay.so')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tf_topic', default_value='/tf_ground_truth',
            description="where the devkit's ground-truth TF goes instead of /tf"),
        DeclareLaunchArgument(
            'tcp_nodelay', default_value='false',
            description='preload tools/libnodelay.so into the bridge: TCP_NODELAY on its '
                        'websocket, see the module docstring'),
        LogInfo(
            condition=IfCondition(LaunchConfiguration('tcp_nodelay')),
            msg=(f'[bridge] TCP_NODELAY via LD_PRELOAD={NODELAY_SHIM}'
                 if os.path.exists(NODELAY_SHIM) else
                 f'[bridge] WARNING: tcp_nodelay requested but {NODELAY_SHIM} is missing; build it with '
                 'gcc -shared -fPIC -O2 -o tools/libnodelay.so tools/nodelay.c -ldl')),
        Node(
            package='autodrive_roboracer',
            executable='autodrive_bridge',
            name='autodrive_bridge',
            output='screen',
            emulate_tty=True,
            remappings=[('/tf', LaunchConfiguration('tf_topic'))],
            additional_env={'LD_PRELOAD': PythonExpression([
                "'", NODELAY_SHIM, "' if '", LaunchConfiguration('tcp_nodelay'), "'.lower() == 'true' else ''"])},
        ),
    ])
