#!/usr/bin/env bash
# Run the simulator and the multi-track stack in containers.
#
#     ./scripts/run.sh sim [--headless [BRIDGE_IP]]   the organizers' simulator
#     ./scripts/run.sh racer [name:=value ...]        race.launch.py with those arguments
#     ./scripts/run.sh shell                          bash in a racer container, nothing started
#     ./scripts/run.sh exec                           second bash in the running racer container
#
# Example:
#     ./scripts/run.sh racer track:=iros2026 tcp_nodelay:=true loop_hz_cap:=45 log_csv:=run_iros_21.csv
#
# Output: relative log_csv:= paths are written to runs_docker/ in this repo.
# Racelines: the host's raceline/ is mounted over the baked copy, so a new CSV
# is raced without a rebuild (MOUNT_RACELINE=0 uses the copy in the image).
# RViz opens on the host display when DISPLAY is set (xhost local:root is run
# for you); pass rviz:=false to skip it.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-autodrive_racer}"
TAG="${TAG:-multi-track}"
SIM_TAG="${SIM_TAG:-2026-iros-practice}"
NAME=autodrive_roboracer_api
IN=/root/Documents/roboracer

racer_run() {
  mkdir -p "$REPO/runs_docker"
  xhost local:root >/dev/null 2>&1 || true
  docker rm -f "$NAME" >/dev/null 2>&1 || true
  local mounts=(-v "$REPO/runs_docker:$IN/runs")
  if [ "${MOUNT_RACELINE:-1}" = "1" ]; then
    mounts+=(-v "$REPO/raceline:$IN/raceline")
  fi
  exec docker run --name "$NAME" --rm -it \
    --network=host --ipc=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY \
    "${mounts[@]}" \
    "${IMAGE}:${TAG}" "$@"
}

case "${1:-}" in
  sim)
    SIM_NAME=autodrive_roboracer_sim
    SIM_IMAGE="autodriveecosystem/autodrive_roboracer_sim:${SIM_TAG}"
    xhost local:root >/dev/null 2>&1 || true
    docker rm -f "$SIM_NAME" >/dev/null 2>&1 || true
    if [ "${2:-}" = "--headless" ]; then
      BRIDGE_IP="${3:-${BRIDGE_IP:-127.0.0.1}}"
      echo "[run.sh] headless sim -> bridge at ${BRIDGE_IP}:4567" >&2
      exec docker run --name "$SIM_NAME" --rm -it --entrypoint /bin/bash \
        --network=host --ipc=host \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all \
        "$SIM_IMAGE" \
        -lc "./AutoDRIVE\\ Simulator.x86_64 -batchmode -nographics -ip ${BRIDGE_IP} -port 4567"
    fi
    exec docker run --name "$SIM_NAME" --rm -it --entrypoint /bin/bash \
      --network=host --ipc=host \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all \
      "$SIM_IMAGE"
    ;;
  racer)
    shift
    [ $# -gt 0 ] || { echo "usage: $0 racer name:=value ..." >&2; exit 1; }
    racer_run "$@"
    ;;
  shell)
    racer_run bash
    ;;
  exec)
    exec docker exec -it "$NAME" bash
    ;;
  *)
    echo "usage: $0 {sim [--headless [BRIDGE_IP]] | racer name:=value ... | shell | exec}" >&2
    exit 1
    ;;
esac
