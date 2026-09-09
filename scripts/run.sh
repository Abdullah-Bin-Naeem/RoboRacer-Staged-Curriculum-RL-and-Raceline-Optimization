#!/usr/bin/env bash
# Run the two containers.   ./scripts/run.sh sim [--headless] | racer
#
#     ./scripts/run.sh sim --headless   simulator, no window, auto-connects
#     ./scripts/run.sh sim              simulator with a window; hit Connect
#     ./scripts/run.sh racer            our stack -- starts driving on its own

# RACER_TRACK=porto ./scripts/raceline_editor.py raceline_a6.5.csv
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
SIM_TAG="${SIM_TAG:-2026-icra-compete}"

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
    # Forward the RACER_* knobs the entrypoint reads. `docker run` does NOT
    # inherit the caller's environment -- without this, exporting RACER_TRACK in
    # your shell changed nothing and the container silently raced whatever was
    # compiled in, which is the whole reason picking a track used to mean
    # editing frames.py and rebuilding.
    #
    # `-e NAME` with no `=value` passes the variable through ONLY if it is set,
    # so an unset knob leaves the entrypoint's own default alone rather than
    # overriding it with an empty string.
    #
    #   RACER_TRACK      icra | porto            which map, spawn and line ladder
    #   RACER_RACELINE   raceline_a4.0.csv       a line within that track
    #   RACER_LOCALIZER  amcl | slam | none
    #   RACER_MODE       race | dev
    #   RACER_EXTRA_ARGS "v_max:=7.5 lookahead_k:=0.6"
    #
    # Everything is per-invocation, so nothing here needs a rebuild:
    #   RACER_TRACK=porto ./scripts/run.sh racer
    env=()
    shown=()
    for v in RACER_TRACK RACER_RACELINE RACER_PATH_CSV RACER_LOCALIZER \
             RACER_MODE RACER_BOOTSTRAP_MODE RACER_CONTROL_HZ RACER_RVIZ \
             RACER_AUTOSTART RACER_EXTRA_ARGS; do
      [ -n "${!v:-}" ] && { env+=(-e "$v"); shown+=("$v=${!v}"); }
    done
    # Echo what is actually being handed over. Racing the wrong track because a
    # variable was misspelled in the calling shell is silent otherwise -- the
    # container just uses its own default.
    [ ${#shown[@]} -gt 0 ] && echo "[run.sh] forwarding ${shown[*]}" >&2

    # RACER_LIVE=1 -- iterate on racing lines WITHOUT rebuilding the image.
    #
    # The lines are baked in, so a CSV written after the last `./scripts/build.sh`
    # simply is not in the container, and the launch dies naming the files that
    # ARE there. That is correct for a submission image and tedious while
    # reprofile_raceline.py is producing a line every few minutes. This mounts
    # the host's raceline/ and maps/ read-only and points common/frames.py at
    # them through the overrides it already supports, so a freshly written CSV
    # is drivable immediately.
    #
    # DEVELOPMENT ONLY. The submission must be self-contained: never hand the
    # organizers an image that needs a mount, and rebuild before measuring
    # anything you intend to report.
    if [ "${RACER_LIVE:-0}" = "1" ]; then
      here=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
      env+=(-v "$here/roboracer_stack/raceline:/live/raceline:ro"
            -v "$here/roboracer_stack/maps:/live/maps:ro"
            -e RACER_RACELINE_DIR=/live/raceline
            -e RACER_MAPS_DIR=/live/maps)
      echo "[run.sh] RACER_LIVE=1 -- driving $here/roboracer_stack/{raceline,maps}" \
           "from the host, NOT the image. Rebuild before measuring." >&2
    fi
    # No --entrypoint override: the point of the submission is that the default
    # entrypoint brings everything up by itself.
    # NOT "${env[@]:-}": on an empty array that expands to one EMPTY argument,
    # which docker rejects as a blank image name.
    exec docker run --name "$NAME" --rm -it \
      --network=host --ipc=host \
      ${env[@]+"${env[@]}"} \
      "${IMAGE}:${TAG}"
    ;;

  *) echo "usage: $0 {sim [--headless]|racer}" >&2; exit 1 ;;
esac
