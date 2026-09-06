# Environment for every ROS session in this container. Installed at
# /home/racer_env.sh and sourced by three things:
#
#   /home/autodrive_devkit.sh   the entrypoint
#   /home/start_racer.sh        the launcher
#   ~/.bashrc                   so `docker exec ... bash` can run ros2 commands
#
# It sets variables and sources workspaces. It LAUNCHES NOTHING -- that
# separation is what lets the organizers open extra shells for inspection and
# recording without a second copy of the stack coming up behind them.
#
# Every value here is load-bearing:
#
#   ROS_LOCALHOST_ONLY   the sim is reached over a TCP socket on 4567, not over
#                        DDS, so nothing needs to leave the container. Pinning
#                        DDS to loopback also keeps our ~10 nodes from
#                        discovering anything on the host's network.
#   RMW_IMPLEMENTATION   cyclonedds. Every measured lap time on this branch was
#                        recorded on cyclone; fastrtps is untested here.
#   CYCLONEDDS_URI       raises MaxAutoParticipantIndex above its default of 9.
#                        race.launch.py is about ten nodes, so without this a
#                        different node dies each run with "Failed to find a
#                        free participant index for domain 0". Not optional.
#   XDG_RUNTIME_DIR      ROS and Qt both warn loudly without it in a container.

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ROS's setup.bash reads AMENT_TRACE_SETUP_FILES and friends without a default,
# so `set -u` in the caller kills the container on the first source with
#     /opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
# and nothing else is ever reached. Relax -u and -e across the sourcing only,
# then restore whatever the caller had, so their own typos still get caught.
_racer_opts="$-"
set +ue
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
source /home/racer_ws/install/setup.bash
case "$_racer_opts" in *u*) set -u;; esac
case "$_racer_opts" in *e*) set -e;; esac
unset _racer_opts

_dds=/home/racer_ws/install/racer_common/share/racer_common/config/cyclonedds.xml
if [ -f "$_dds" ]; then
    export CYCLONEDDS_URI="file://$_dds"
else
    echo "racer_env: WARNING cyclonedds.xml missing; the 10th node will fail to start" >&2
fi
unset _dds

export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"
