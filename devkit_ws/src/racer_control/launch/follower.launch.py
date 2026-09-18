"""Pure pursuit follower. The controller and nothing else.

    ros2 launch racer_control follower.launch.py

RViz and /map used to be started here. They are not this package's to own --
racer_bringup composes the window, racer_mapping serves the grid -- and having
the follower start them is how two /map publishers on different frames ended up
racing each other. Normally reached as race.launch.py; launchable alone against
an already-running localizer.

Tuning knobs are exposed as launch arguments so they can be overridden without
a rebuild, e.g.

    ros2 launch racer_control follower.launch.py lookahead_k:=0.70

POSE SOURCE -- the one setting that decides whether a run means anything.
`pose_topic` defaults to the devkit's odometry, which is simulator GROUND TRUTH
and RESTRICTED at race time. That default is a development convenience for
driving the line with localization out of the picture; the node now says so
loudly at startup (racer_common/restricted.py). For a real run pass
use_tf_pose:=true, which reads the localizer's map->base correction instead.
race.launch.py sets that for you.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from racer_common import frames
from racer_common.frames import NS, TRACK

# Overridable at launch time; empty string keeps whatever the params file says.
# curvature_preview_m matters more than it looks: with no v_mps column in the
# CSV, speed is derived from the worst curvature within this distance. Braking
# 8.0 -> 1.8 m/s takes ~6 m, so a 1 m preview means the car meets the corner at
# full speed. Irrelevant when the path carries its own velocity profile.
#
# KEEP IN STEP with racer_bringup/launch/race.launch.py, which passes the same
# names through. A name listed there but not here is dropped silently.
TUNABLES = ('lookahead_min', 'lookahead_max', 'lookahead_k', 'lookahead_curv_gain', 'lookahead_sag_frac',
            'lookahead_delay_ref', 'derate_delay_from', 'derate_delay_to', 'derate_a_lat',
            'steer_a_lat_max', 'steer_excess_rad',
            'exit_guard_from', 'exit_guard_full',
            'lqr_k_lat', 'lqr_k_head', 'lqr_k_yaw', 'lqr_max_correction_rad',
            'v_max', 'a_lat_max', 'throttle_max', 'steering_gain',
            'warmup_v_max', 'warmup_dist_m',
            'curvature_preview_m',
            # slip throttle and fused speed (see pure_pursuit.py docstring)
            'slip_accel', 'slip_brake', 'u_launch', 'u_per_throttle', 'v_slip_den', 'tire_rise_slope',
            'cmd_delay_s', 'slip_kp', 'target_lead_s', 'enc_rate_window_s', 'observer_wheels', 'slip_circle', 'accel_ff', 'drag_ff',
            # control loop rate; 20 matches the 17.5 Hz sim tick seen here, raise it
            # with the tick (headless sim, faster machine) so the loop is not the limit
            'control_hz',
            'pose_speed_window', 'pose_speed_gain', 'pose_corr_max', 'imu_lever_arm', 'latency_comp_s',
            # legacy launch ramp
            'a_long_launch', 'a_long_launch_v')
# String-valued switches, passed through without the float() cast.
STR_TUNABLES = ('speed_source', 'throttle_mode', 'steer_excess_ref', 'controller_mode')


def _nodes(context, *args, **kwargs):
    cfg = lambda n: LaunchConfiguration(n).perform(context)
    # The track's default line unless a CSV was named (race.launch.py names it).
    path_csv = cfg('path_csv') or frames.raceline(cfg('track'))
    overrides = {n: float(cfg(n)) for n in TUNABLES if cfg(n) != ''}
    overrides.update({n: cfg(n) for n in STR_TUNABLES if cfg(n) != ''})
    if overrides:
        print(f'[pure_pursuit] launch overrides: {overrides}')

    return [
        Node(
            package='racer_control',
            executable='pure_pursuit',
            name='pure_pursuit',
            output='screen',
            emulate_tty=True,
            parameters=[
                cfg('pp_params_file'),
                {'path_csv': path_csv,
                 'pose_topic': cfg('pose_topic'),
                 'use_tf_pose': cfg('use_tf_pose').lower() == 'true',
                 'wait_for_ready': cfg('wait_for_ready').lower() == 'true',
                 'bootstrap_seconds': float(cfg('bootstrap_seconds')),
                 'dev_lap_telemetry': cfg('dev_lap_telemetry').lower() == 'true'},
                overrides,
            ],
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('racer_control')

    args = [
        DeclareLaunchArgument(
            'pp_params_file',
            default_value=os.path.join(pkg_share, 'config', 'pure_pursuit.yaml')),
        DeclareLaunchArgument('track', default_value=TRACK,
                              description="track whose default line to follow"),
        DeclareLaunchArgument(
            'path_csv',
            default_value='',
            description='path to follow (s,x,y,psi,kappa,w_r,w_l)'),
        DeclareLaunchArgument('wait_for_ready', default_value='false'),
        DeclareLaunchArgument('bootstrap_seconds', default_value='0.0'),
        DeclareLaunchArgument(
            'pose_topic', default_value=f'{NS}/odom',
            description='/amcl_pose for the race-legal estimate; the devkit odom '
                        'is ground truth and RESTRICTED at race time'),
        DeclareLaunchArgument('dev_lap_telemetry', default_value='false'),
        DeclareLaunchArgument(
            'use_tf_pose', default_value='false',
            description='read the pose from TF map->roboracer_1 (continuous) '
                        'instead of the /amcl_pose topic (resample-triggered). '
                        'Set true whenever AMCL is the pose source.'),
    ]
    args += [DeclareLaunchArgument(n, default_value='',
                                   description='override the params file value')
             for n in TUNABLES + STR_TUNABLES]

    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])
