#!/usr/bin/env bash
# Run the two containers.   ./scripts/run.sh sim [--headless] | racer
#
#     ./scripts/run.sh sim --headless   simulator, no window, auto-connects
#     ./scripts/run.sh sim              simulator with a window; hit Connect
#     ./scripts/run.sh racer            our stack -- starts driving on its own
#
# Order does not matter. The bridge listens on 4567 and blocks until the
# simulator connects, so the racer container can be up first and waiting.
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
      exec docker run --name "$NAME" --rm -it --entrypoint /bin/bash \
        --network=host --ipc=host \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all \
        "$SIM_IMAGE" \
        -lc './AutoDRIVE\ Simulator.x86_64 -batchmode -nographics -ip 127.0.0.1 -port 4567'
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

  *) echo "usage: $0 {sim [--headless]|racer}" >&2; exit 1 ;;
esac
