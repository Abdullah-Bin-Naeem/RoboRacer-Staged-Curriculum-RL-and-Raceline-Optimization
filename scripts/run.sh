#!/usr/bin/env bash
# Run the two containers.   ./scripts/run.sh sim [--headless [IP]] | race
#
#     ./scripts/run.sh sim --headless   simulator, no window, auto-connects
#     ./scripts/run.sh sim              simulator with a window; hit Connect
#     ./scripts/run.sh race             our stack -- starts driving on its own
#     ./scripts/run.sh shell            a shell in the image, nothing started
#     ./scripts/run.sh exec             a second shell in the running container
#
# Order does not matter. The bridge listens on 4567 and blocks until the
# simulator connects, so the racer container can be up first and waiting.
#
# `race` takes no arguments, on purpose: the promoted configuration is the
# launch default, not something this script passes in. To try a change without
# rebuilding, pass it through the entrypoint:
#
#     RACER_EXTRA_ARGS="v_max:=8.5" ./scripts/run.sh race
#
# TWO MACHINES
# ------------
# The bridge is a WebSocket server and the simulator is its client, so the two
# do not have to share a host -- and on a machine that cannot feed the bridge
# fast enough, they should not. Give headless the bridge machine's address:
#
#     ./scripts/run.sh sim --headless 192.168.1.42
#
# With a window, type the same address into the Connect panel instead.
#
# Prefer --headless for timed runs. Unity throttles its frame rate whenever its
# window is minimised, occluded or on another workspace, and the symptom is not
# an error: every rate in the system silently drops and the lap times still look
# plausible. Headless has no window to unfocus and takes the devkit IP on the
# command line, so there is no Connect step either.
#
# This script is a convenience for us. The organizers run the image with their
# own `docker run` (README, "How the organizers run it"), which this mirrors.
set -euo pipefail

IMAGE="${IMAGE:-autodrive_racer}"
TAG="${TAG:-iros-2026-final}"
SIM_TAG="${SIM_TAG:-2026-iros-compete}"
NAME=autodrive_roboracer_api

racer_run() {
  docker rm -f "$NAME" >/dev/null 2>&1 || true
  exec docker run --name "$NAME" --rm ${DOCKER_RUN_FLAGS:--it} \
    --network=host --ipc=host \
    "$@" "${IMAGE}:${TAG}"
}

case "${1:-}" in
  sim)
    SIM_NAME=autodrive_roboracer_sim
    SIM_IMAGE="autodriveecosystem/autodrive_roboracer_sim:${SIM_TAG}"
    xhost local:root >/dev/null 2>&1 || true
    docker rm -f "$SIM_NAME" >/dev/null 2>&1 || true
    if [ "${2:-}" = "--headless" ]; then
      # Where the BRIDGE is, not where the sim is: loopback when both containers
      # are on this machine, the other machine's LAN address when they are not.
      BRIDGE_IP="${3:-${BRIDGE_IP:-127.0.0.1}}"
      echo "[run.sh] headless sim -> bridge at ${BRIDGE_IP}:4567" >&2
      exec docker run --name "$SIM_NAME" --rm ${DOCKER_RUN_FLAGS:--it} --entrypoint /bin/bash \
        --network=host --ipc=host \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all \
        "$SIM_IMAGE" \
        -lc "./AutoDRIVE\\ Simulator.x86_64 -batchmode -nographics -ip ${BRIDGE_IP} -port 4567"
    fi
    exec docker run --name "$SIM_NAME" --rm ${DOCKER_RUN_FLAGS:--it} --entrypoint /bin/bash \
      --network=host --ipc=host \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all \
      "$SIM_IMAGE"
    ;;

  race|racer)
    # No --entrypoint override: the point of the submission is that the default
    # entrypoint brings everything up by itself. RACER_EXTRA_ARGS is passed
    # through when it is set, and is empty for an evaluation run.
    if [ -n "${RACER_EXTRA_ARGS:-}" ]; then
      racer_run -e "RACER_EXTRA_ARGS=${RACER_EXTRA_ARGS}"
    fi
    racer_run
    ;;

  shell)
    # The image with nothing running, for looking around before a race.
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    exec docker run --name "$NAME" --rm -it --network=host --ipc=host \
      -e RACER_AUTOSTART=0 "${IMAGE}:${TAG}"
    ;;

  exec) exec docker exec -it "$NAME" bash ;;

  *) echo "usage: $0 {sim [--headless [BRIDGE_IP]]|race|shell|exec}" >&2; exit 1 ;;
esac
