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
#   our localization + control  dead reckoning, AMCL against the baked-in map
#                              for the selected track, and pure pursuit along
#                              that track's raceline. Both tracks and every line
#                              are in the image; RACER_TRACK picks one.
#
# The bridge listens on 4567 and blocks until the simulator connects, so
# starting before the operator hits Connect is correct: everything is up and
# waiting, and the car moves the moment the socket opens.
#
# IF YOU STARTED THE CONTAINER WITH --entrypoint /bin/bash
# --------------------------------------------------------
# Then this file never ran. Bring the stack up by running it by hand:
#
#     /home/autodrive_devkit.sh
#
# TO GET A ROS ENVIRONMENT IN AN INSPECTION SHELL
# -----------------------------------------------
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
# Every value is overridable from `docker run -e NAME=...`, so a teammate can
# change the line or fall back to slam without rebuilding the image.
#
#   RACER_TRACK=icra  which circuit. Exported rather than passed as a launch
#                     argument because roboracer_stack.common.frames reads it at
#                     IMPORT time -- it selects the map, the spawn pose, the fit
#                     grid and the default line together, and launch arguments
#                     are parsed long after that module is imported. Set `porto`
#                     for the qualification track.
#                     NOTE: icra ships no pose graph, so RACER_LOCALIZER=slam
#                     works on porto only.
#   RACER_RACELINE    a line WITHIN that track, by bare filename --
#                     `raceline_a4.0.csv`, not a share path. Unset uses the
#                     track's default from the frames table.
#   localizer:=amcl   nav2 AMCL against the track's track_clean.pgm. `slam` is
#                     built into the image too and works (6.7 s), but AMCL is
#                     what this branch is qualified on.
#   mode:=race        instruments off, no lap telemetry, steer on the estimate.
#                     race.launch.py refuses every restricted reader in this
#                     mode; see roboracer_stack/common/restricted.py.
#   bootstrap_mode:=truth
#                     ONE read of /ips before the car moves, then the
#                     subscription is destroyed (restricted.seed / released).
#                     Inside the warmup window, and the timed laps never see
#                     ground truth. Set `global` for AMCL's particle search if
#                     the organizers read the rule more strictly -- it costs a
#                     convergence phase and is occasionally slower to settle.
#   control_hz:=40    the follower loop. Runs 29-41 were all measured at 40
#                     against a headless sim so that the loop is never the
#                     limiter. Harmless when the sim is slower -- pure_pursuit
#                     measures its own round trip and derates the speed targets,
#                     it does not key off this.
#   rviz:=false       there is no display in the evaluation container.
#
# path_csv is left unset unless asked for: common/frames.py picks the track's
# default and prints it, so the choice lives in one documented place instead of
# two.
export RACER_TRACK="${RACER_TRACK:-icra}"
echo "[entrypoint] track ${RACER_TRACK}"
ARGS=(
  "localizer:=${RACER_LOCALIZER:-amcl}"
  "mode:=${RACER_MODE:-race}"
  "bootstrap:=true"
  "bootstrap_mode:=${RACER_BOOTSTRAP_MODE:-truth}"
  "control_hz:=${RACER_CONTROL_HZ:-40}"
  "rviz:=${RACER_RVIZ:-false}"
)
# RACER_RACELINE is a bare filename inside the track's raceline directory;
# RACER_PATH_CSV is the older full-path form. Both land on the SAME launch
# argument, which resolves either (follower.launch.py -> frames.raceline_path),
# so a name is enough now that a track ships eleven lines. Passing path_csv:=
# twice would leave which one wins up to ros2 launch, so they are exclusive and
# the newer name is preferred.
_line="${RACER_RACELINE:-${RACER_PATH_CSV:-}}"
if [ -n "${RACER_RACELINE:-}" ] && [ -n "${RACER_PATH_CSV:-}" ]; then
    echo "[entrypoint] WARNING both RACER_RACELINE and RACER_PATH_CSV are set;" \
         "using RACER_RACELINE=${RACER_RACELINE}" >&2
fi
[ -n "$_line" ] && ARGS+=("path_csv:=${_line}")
unset _line
# Anything else, word-split on purpose: RACER_EXTRA_ARGS="v_max:=7.5 lookahead_k:=0.6"
# shellcheck disable=SC2206
[ -n "${RACER_EXTRA_ARGS:-}" ] && ARGS+=(${RACER_EXTRA_ARGS})

# ---- Launch ----------------------------------------------------------------
# In the background, so the container's CMD (bash) runs in the foreground and
# `docker run -it` lands on a usable prompt with the car already driving.
LOG_DIR=/home/autodrive_devkit/log
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/racer_$(date +%Y%m%d_%H%M%S).log"

if [ "${RACER_AUTOSTART:-1}" = "1" ]; then
    echo "[entrypoint] ros2 launch roboracer_stack race.launch.py ${ARGS[*]}"
    ros2 launch roboracer_stack race.launch.py "${ARGS[@]}" >"$LOG" 2>&1 &
    RACER_PID=$!
    echo "[entrypoint] race stack pid $RACER_PID -- follow it with: tail -f $LOG"

    # A launch that dies on a bad argument -- a raceline name that is not in the
    # image, a params file that does not exist -- is dead within a couple of
    # seconds, and its error goes to $LOG where nobody is looking. Without this
    # check the next line cheerfully announced that we were waiting for the
    # simulator while nothing at all was running, and the symptom presented as
    # "the bridge never connects" -- which sends you to the network, the port
    # and the sim, none of which are the problem. The follower starts on a 2 s
    # timer, so a bad path_csv only surfaces after the rest is already up: wait
    # past that before deciding the stack is healthy.
    sleep 4
    if ! kill -0 "$RACER_PID" 2>/dev/null; then
        echo "[entrypoint] ERROR the race stack died during startup. Last errors:" >&2
        grep -iE 'error|exception|no raceline' "$LOG" | tail -5 >&2
        echo "[entrypoint] full log: $LOG" >&2
        echo "[entrypoint] the simulator is NOT being driven." >&2
        RACER_PID=""
    else
        echo "[entrypoint] waiting for the simulator to connect on port 4567"
    fi
else
    RACER_PID=""
    echo "[entrypoint] RACER_AUTOSTART=0 -- not starting the stack."
    echo "[entrypoint] start it by hand with: ros2 launch roboracer_stack race.launch.py"
fi

# Hand over to CMD (bash). Without a TTY -- `docker run -d`, or a CI job -- an
# interactive bash would read EOF and exit immediately, taking the container and
# our backgrounded stack with it, so in that case wait on the stack instead.
if [ $# -gt 0 ] && { [ -t 0 ] || [ -z "$RACER_PID" ]; }; then
    exec "$@"
elif [ -n "$RACER_PID" ]; then
    wait "$RACER_PID"
else
    exec "$@"
fi
