#!/usr/bin/env bash
# Run the two containers.   ./scripts/run.sh sim [--headless [IP]] | racer
#
#     ./scripts/run.sh sim --headless   simulator, no window, auto-connects
#     ./scripts/run.sh sim              simulator with a window; hit Connect
#     ./scripts/run.sh racer            our stack -- starts driving on its own
#
# Order does not matter. The bridge listens on 4567 and blocks until the
# simulator connects, so the racer container can be up first and waiting.
#
# TWO MACHINES
# ------------
# The bridge is a WebSocket server and the simulator is its client, so the two
# do not have to share a host -- and on a machine that cannot feed the bridge
# fast enough, they should not. Give headless the bridge machine's address:
#
#     ./scripts/run.sh sim --headless 192.168.1.42
#
# With a window, type the same address into the Connect panel instead. See the
# README, "Running the sim and the racer on two machines", for the other half.
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
TAG="${TAG:-qualification-1}"
SIM_TAG="${SIM_TAG:-2026-iros-practice}"

case "${1:-}" in
  sim)
    NAME=autodrive_roboracer_sim
    SIM_IMAGE="autodriveecosystem/autodrive_roboracer_sim:${SIM_TAG}"
    xhost local:root >/dev/null 2>&1 || true
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    if [ "${2:-}" = "--headless" ]; then
      # Where the BRIDGE is, not where the sim is: loopback when both containers
      # are on this machine, the other machine's LAN address when they are not.
      BRIDGE_IP="${3:-${BRIDGE_IP:-127.0.0.1}}"
      echo "[run.sh] headless sim -> bridge at ${BRIDGE_IP}:4567" >&2
      exec docker run --name "$NAME" --rm -it --entrypoint /bin/bash \
        --network=host --ipc=host \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all \
        "$SIM_IMAGE" \
        -lc "./AutoDRIVE\\ Simulator.x86_64 -batchmode -nographics -ip ${BRIDGE_IP} -port 4567"
    fi
    exec docker run --name "$NAME" --rm -it --entrypoint /bin/bash \
      --network=host --ipc=host \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all \
      "$SIM_IMAGE"
    ;;

  racer)
    NAME=autodrive_roboracer_api
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    # No --entrypoint override: the point of the submission is that the default
    # entrypoint brings everything up by itself.
    exec docker run --name "$NAME" --rm -it \
      --network=host --ipc=host \
      "${IMAGE}:${TAG}"
    ;;

  *) echo "usage: $0 {sim [--headless [BRIDGE_IP]]|racer}" >&2; exit 1 ;;
esac
