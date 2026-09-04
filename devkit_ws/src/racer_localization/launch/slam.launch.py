"""Localizer B: slam_toolbox in localization mode, against a serialized pose graph.

    ros2 launch racer_localization slam.launch.py

The A/B partner to amcl.launch.py. Same frames, same sensors, same error
instrument -- and now that is enforced by construction rather than by
copy-paste: both files contain ONLY their localizer, and the shared half lives
in chassis.launch.py.

Publishes map -> odom, exactly the link AMCL publishes.

HOW IT DIFFERS FROM THE AMCL STACK
----------------------------------
no nav2_map_server      slam_toolbox publishes /map itself, from the pose graph
no lifecycle_manager    localization_slam_toolbox_node is a plain node
same bootstrap          localization_bootstrap now serves both localizers. It
                        used to be AMCL-only -- it polled /amcl/get_state and
                        confirmed on /amcl_pose, neither of which exists here --
                        so everything AMCL-shaped in it became a parameter
                        rather than a second copy of the node:

                          ready_check:=tf       map->odom appearing IS the
                                                handshake; slam_toolbox
                                                publishes it only after its
                                                first scan, which is also when
                                                localizePoseCallback stops
                                                early-returning
                          estimate_topic:=/pose slam_toolbox's own output
                          verify_via_tf:=true   confirm on map->roboracer_1,
                                                the transform the follower
                                                actually drives on
                          nudge/global:=''      it offers neither service
no .pgm                 the map is the serialized pose graph, not a grid

WHY THE SEED MATTERS MORE HERE THAN FOR AMCL
--------------------------------------------
slam_toolbox seeds with a correlative scan match over
correlation_search_space_dimension (1.0 m, i.e. +-0.5 m) centred on the guess.
An error larger than that is not in the search space at any resolution, so it is
never found -- it is baked into map->odom and frozen for the entire run, which
reads as a constant offset along the track. AMCL scatters particles and can
recover; this cannot.

mode:=race forbids /ips, and slam_toolbox has NO global search to fall back on
(DeserializePoseGraph.srv offers only START_AT_FIRST_NODE / START_AT_GIVEN_POSE
/ LOCALIZE_AT_POSE -- all of them take a pose). So a race run starts on
racer_common.frames.SPAWN_* and nothing else. Run once with
bootstrap_mode:=truth and paste the SPAWN_* block it prints into frames.py.

THE MAP MUST HAVE BEEN BUILT WITH SCAN MATCHING ON. karto only adds vertices
and edges to the graph inside `if (use_scan_matching)`, so a map built with it
false deserializes into a graph with ZERO nodes and this node segfaults on the
first scan (AddEdge: At least one vertex is invalid). `track` is such a map;
`track_sm` is the re-map that works.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from racer_common import frames
from racer_common.frames import BASE, MAP, NS, ODOM


def _nodes(context, *args, **kwargs):
    cfg = lambda n: LaunchConfiguration(n).perform(context)
    # The track's pose graph unless one was named (race.launch.py names it).
    track = cfg('track')
    map_graph = cfg('map_graph') or frames.pose_graph(track)
    spawn = frames.spawn(track)
    initial = [cfg(n) or spawn[i] for i, n in
               enumerate(('initial_x', 'initial_y', 'initial_yaw'))]
    pkg_share = get_package_share_directory('racer_localization')

    overrides = {
        'map_file_name': map_graph,
        'map_start_pose': [float(initial[0]),
                           float(initial[1]),
                           float(initial[2])],
        'odom_frame': ODOM,
        'map_frame': MAP,
        'base_frame': BASE,
        'scan_topic': f'{NS}/lidar',
    }

    bootstrap_mode = cfg('bootstrap_mode')

    return [
        # map -> odom, by scan-matching the live scan against the stored graph.
        # If this executable is missing on your slam_toolbox build, the fallback
        # is `async_slam_toolbox_node` with the same params -- mode: localization
        # in the yaml is what actually selects the behaviour.
        Node(
            package='slam_toolbox', executable='localization_slam_toolbox_node',
            name='slam_toolbox', output='screen', emulate_tty=True,
            parameters=[cfg('slam_params_file'), overrides],
        ),

        # Seeds the initial pose and latches /localization_ready, exactly as
        # it does for AMCL -- see the module docstring for the four parameters
        # that make one node serve both.
        Node(
            package='racer_localization', executable='localization_bootstrap',
            name='localization_bootstrap', output='screen', emulate_tty=True,
            parameters=[{
                'mode': bootstrap_mode,
                # Not a lifecycle node, so there is no state to poll; the
                # appearance of map->odom is the equivalent signal.
                'ready_check': 'tf',
                'estimate_topic': '/pose',
                # Confirm on the composed map->base, which is what
                # pure_pursuit consumes (use_tf_pose defaults true and is
                # REQUIRED for slam), rather than on the localizer in isolation.
                'verify_via_tf': True,
                'map_frame': MAP,
                'odom_frame': ODOM,
                'base_frame': BASE,
                # slam_toolbox offers neither service. Empty disables both;
                # with no global search, bootstrap_mode:=global falls back to
                # map_start_pose and says so loudly.
                'nudge_service': '',
                'global_service': '',
                'require_convergence': cfg('require_convergence').lower() == 'true',
            }],
            condition=IfCondition(LaunchConfiguration('bootstrap')),
        ),

        # RViz belongs with the localizer, not only with the composition root.
        # It was in racer_bringup, which made `ros2 launch racer_localization
        # slam.launch.py` -- the obvious command for debugging one localizer --
        # come up blind. Defaults ON so the standalone command just works;
        # race.launch.py passes its own rviz:= through, so composing does not
        # produce two windows.
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', os.path.join(pkg_share, 'config', 'slam.rviz')],
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('racer_localization')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_share, 'config', 'slam.yaml')),
        DeclareLaunchArgument(
            'track', default_value=frames.TRACK,
            description='track whose pose graph to use'),
        DeclareLaunchArgument(
            'map_graph', default_value='',
            description='serialized pose graph, BASE PATH with no extension -- '
                        'slam_toolbox appends .posegraph and .data itself'),
        DeclareLaunchArgument(
            'bootstrap', default_value='true',
            description='seed the initial pose and latch /localization_ready '
                        'instead of living on map_start_pose alone'),
        DeclareLaunchArgument(
            'bootstrap_mode', default_value='truth',
            description="'truth' seeds the pose from /ips once (DEVELOPMENT); "
                        "'global' is UNSUPPORTED by slam_toolbox and falls back "
                        'to map_start_pose'),
        DeclareLaunchArgument(
            'require_convergence', default_value='true',
            description='refuse to latch /localization_ready unless the pose was '
                        'actually confirmed; false hands over regardless'),
        DeclareLaunchArgument('initial_x', default_value=''),
        DeclareLaunchArgument('initial_y', default_value=''),
        DeclareLaunchArgument('initial_yaw', default_value=''),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='open RViz with this localizer''s config; '
                        'race.launch.py passes its own value through'),
        OpaqueFunction(function=_nodes),
    ])
