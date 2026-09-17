#!/usr/bin/env bash
# Entrypoint of the multi-track image.
#
#   docker run ... IMAGE                          bash, workspace sourced
#   docker run ... IMAGE track:=iros2026 ...      ros2 launch racer_bringup race.launch.py track:=iros2026 ...
#   docker run ... IMAGE ros2 launch ...          any command, workspace sourced
#
# Arguments of the form name:=value go straight to race.launch.py, so the
# command line is the same as on a host with ROS installed.
#
# Relative paths (log_csv:=run.csv) land in /root/Documents/roboracer/runs,
# which scripts/run.sh bind-mounts to runs_docker/ on the host.
set -o pipefail

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"

REPO="${RACER_REPO:-/root/Documents/roboracer}"
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
source "$REPO/devkit_ws/install/setup.bash"
export CYCLONEDDS_URI="file://$REPO/devkit_ws/install/racer_common/share/racer_common/config/cyclonedds.xml"

mkdir -p "$REPO/runs"
cd "$REPO/runs"

case "${1:-}" in
  *:=*)
    echo "[entrypoint] ros2 launch racer_bringup race.launch.py $*"
    exec ros2 launch racer_bringup race.launch.py "$@"
    ;;
  "")
    exec bash
    ;;
  *)
    exec "$@"
    ;;
esac
