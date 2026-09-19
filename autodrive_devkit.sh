#!/usr/bin/env bash
# THE SUBMISSION ENTRYPOINT. Installed at /home/autodrive_devkit.sh, replacing
# the stub the base image ships, and named that because the technical guide
# names it:
#
#   "Please make sure that you include all the necessary commands (for sourcing
#    workspaces, setting environment variables, launching nodes, etc.) within
#    the entrypoint script (autodrive_devkit.sh file) provided within the
#    autodrive_roboracer_api container."
#
# So everything is here, in one file: the environment, the workspace, and the
# launch. There is no second script to keep in step, and nothing is automated
# from ~/.bashrc -- the guide forbids that, and it is also what lets the
# organizers open extra shells for recording and inspection without a second
# copy of the stack coming up behind them.
#
# What comes up:
#
#   the devkit bridge          launch/bridge.launch.py, included by race.launch.py
#                              with the ground-truth TF remapped off /tf (or
#                              roboracer_1 gets two parents and the TF tree
#                              breaks). Exactly one bridge, one owner.
#   our localization + control  dead reckoning, the segmented scan-to-map
#                              localizer against the baked-in IROS 2026 map,
#                              and pure pursuit along the baked-in raceline.
#
# The bridge listens on 4567 and blocks until the simulator connects, so
# starting before the operator hits Connect is correct: everything is up and
# waiting, and the car moves the moment the socket opens.
#
# NOTHING IS WRITTEN TO DISK. The stack runs in the FOREGROUND and its output is
# the container's output, so `docker run` shows it live and `docker logs` has it
# afterwards. No CSV, no scan dump, no run directory: the nodes that produced
# those are development-only and are not in this image at all.
#
# IF YOU STARTED THE CONTAINER WITH --entrypoint /bin/bash
# --------------------------------------------------------
# Then this file never ran. Bring the stack up by running it by hand:
#
#     /home/autodrive_devkit.sh
#
# TO GET A ROS ENVIRONMENT IN AN INSPECTION SHELL
# -----------------------------------------------
#     docker exec -it autodrive_roboracer_api bash
#     source /home/autodrive_devkit/install/setup.bash
set -uo pipefail

# ---- Environment -----------------------------------------------------------
#   ROS_LOCALHOST_ONLY   the simulator is reached over a TCP socket on 4567, not
#                        over DDS, so nothing needs to leave the container.
#                        Pinning DDS to loopback also keeps our ~10 nodes from
#                        discovering anything on the host's network.
#   RMW_IMPLEMENTATION   cyclonedds. Every measured lap time on this branch was
#                        recorded on cyclone; fastrtps is untested here.
#   XDG_RUNTIME_DIR      ROS and Qt both warn loudly without it in a container.
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"

# ROS's setup.bash reads AMENT_TRACE_SETUP_FILES and friends without a default,
# so `set -u` kills the container on the first source with
#     /opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
# and nothing else is ever reached. Relax -u and -e across the sourcing only,
# then restore whatever was set, so real typos below still get caught.
_opts="$-"
set +ue
source /opt/ros/humble/setup.bash
# One workspace: roboracer_stack is built into the devkit's own workspace,
# beside autodrive_roboracer and without touching it.
source /home/autodrive_devkit/install/setup.bash
case "$_opts" in *u*) set -u;; esac
case "$_opts" in *e*) set -e;; esac
unset _opts

# Raises MaxAutoParticipantIndex above its default of 9. race.launch.py is about
# ten nodes, so without this a different node dies each run with "Failed to find
# a free participant index for domain 0". Not optional.
_dds=/home/autodrive_devkit/install/roboracer_stack/share/roboracer_stack/config/cyclonedds.xml
if [ -f "$_dds" ]; then
    export CYCLONEDDS_URI="file://$_dds"
else
    echo "[entrypoint] WARNING cyclonedds.xml missing; the 10th node will fail to start" >&2
fi
unset _dds

# ---- What is being raced ---------------------------------------------------
# NOTHING, by design. Every value that makes this the promoted configuration is
# a launch default -- the line, the localizer, the seed mode, the bridge rate
# cap and the follower arguments all live in race.launch.py and
# roboracer_stack/common/frames.py, so `ros2 launch roboracer_stack
# race.launch.py` by hand races exactly what the organizers' `docker run` does.
# There is no second copy of the configuration here to drift from it.
#
# RACER_EXTRA_ARGS is the one hook, for us, during development:
#     docker run -e RACER_EXTRA_ARGS="v_max:=8.5 rviz:=false" ...
ARGS=()
# Word-split on purpose: RACER_EXTRA_ARGS="v_max:=7.5 lookahead_k:=0.6"
# shellcheck disable=SC2206
[ -n "${RACER_EXTRA_ARGS:-}" ] && ARGS+=(${RACER_EXTRA_ARGS})

# ---- Launch ----------------------------------------------------------------
# In the FOREGROUND, as the container's main process: its output is the
# container's output and nothing is written to disk. Inspection shells come from
# `docker exec`, which the image environment already sets up.
if [ "${RACER_AUTOSTART:-1}" = "1" ]; then
    echo "[entrypoint] ros2 launch roboracer_stack race.launch.py ${ARGS[*]}"
    echo "[entrypoint] waiting for the simulator to connect on port 4567"
    exec ros2 launch roboracer_stack race.launch.py "${ARGS[@]}"
fi

# RACER_AUTOSTART=0: hand over to CMD (bash) with the workspace sourced and
# nothing running.
echo "[entrypoint] RACER_AUTOSTART=0 -- not starting the stack."
echo "[entrypoint] start it by hand with: ros2 launch roboracer_stack race.launch.py"
exec "$@"
