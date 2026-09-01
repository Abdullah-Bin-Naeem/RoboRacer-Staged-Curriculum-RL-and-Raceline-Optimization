# Source this in EVERY ROS terminal for this project:
#
#     source ~/Documents/roboracer/ros_env.sh
#
# For the RL terminals, source this FIRST and the venv SECOND -- ROS puts rclpy
# on PYTHONPATH, and the venv has to win on PATH:
#
#     source ~/Documents/roboracer/ros_env.sh
#     source ~/Documents/roboracer/.venv-rl/bin/activate
#
# Every value here is load-bearing:
#
#   ROS_LOCALHOST_ONLY  pins DDS to loopback. It must be set in EVERY terminal
#                       or nothing discovers anything -- a node without it
#                       listens on a different interface and every topic reads
#                       as dead while the rest of the stack runs perfectly.
#   CYCLONEDDS_URI      raises MaxAutoParticipantIndex above its default of 9.
#                       race.launch.py is about ten nodes, so without this the
#                       tenth fails with "Failed to find a free participant
#                       index for domain 0".

_ROBORACER="$HOME/Documents/roboracer"

export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash

if [ -f "$_ROBORACER/devkit_ws/install/setup.bash" ]; then
    source "$_ROBORACER/devkit_ws/install/setup.bash"
else
    echo "ros_env: devkit_ws/install not built yet -- run:"
    echo "  cd $_ROBORACER/devkit_ws && PYTHONNOUSERSITE=1 colcon build"
fi

_dds="$(ros2 pkg prefix racer_common 2>/dev/null)/share/racer_common/config/cyclonedds.xml"
if [ -f "$_dds" ]; then
    export CYCLONEDDS_URI="file://$_dds"
else
    echo "ros_env: WARNING cyclonedds.xml not found; the 10th node will fail to start"
fi

echo "ros_env: ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY  RMW=${RMW_IMPLEMENTATION:-default}"
echo "ros_env: CYCLONEDDS_URI=${CYCLONEDDS_URI:-UNSET}"
unset _ROBORACER _dds
