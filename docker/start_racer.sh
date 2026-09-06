#!/usr/bin/env bash
# The race stack, in one command. Installed at /home/start_racer.sh.
#
#     /home/start_racer.sh                     the qualification configuration
#     RACER_PATH_CSV=... /home/start_racer.sh  a different line
#
# This is what the entrypoint runs in the background, and it is also the command
# to run by hand when the container was started with `--entrypoint /bin/bash`
# (the form the organizers' guide prints). Keeping it a separate file rather
# than inlining it in the entrypoint means those two paths cannot drift.
#
# It execs `ros2 launch`, so it owns PID of the launch and signals reach it.
set -euo pipefail

source /home/racer_env.sh

# --- What is being raced ----------------------------------------------------
# Every value is overridable from `docker run -e NAME=...`, so a teammate can
# change the line or fall back to slam without rebuilding the image.
#
#   localizer:=amcl   nav2 AMCL against maps/track_clean.pgm. `slam` is built
#                     into the image too and works (6.7 s), but AMCL is what
#                     this branch is qualified on.
#   mode:=race        instruments off, no lap telemetry, steer on the estimate.
#                     race.launch.py refuses every restricted reader in this
#                     mode; see racer_common/restricted.py.
#   bootstrap_mode:=truth
#                     ONE read of /ips before the car moves, then the
#                     subscription is destroyed (restricted.seed / released).
#                     Inside the warmup window, and the timed laps never see
#                     ground truth. Set `global` for AMCL's particle search if
#                     the organizers read the rule more strictly -- it costs a
#                     convergence phase and is occasionally slower to settle.
#   control_hz:=40    the follower loop. 20 matches a 17.5 Hz sim tick; runs
#                     29-41 were all measured at 40 against a headless sim so
#                     that the loop is never the limiter. Harmless when the sim
#                     is slower -- pure_pursuit measures its own round trip and
#                     derates the speed targets, it does not key off this.
#   rviz:=false       there is no display in the evaluation container.
#
# path_csv is left unset on purpose: racer_common.frames picks the default
# (raceline_a7.0.csv, run 38, 6.50 s) and prints it, so the choice lives in one
# documented place instead of two.
LOCALIZER="${RACER_LOCALIZER:-amcl}"
MODE="${RACER_MODE:-race}"
BOOTSTRAP_MODE="${RACER_BOOTSTRAP_MODE:-truth}"
CONTROL_HZ="${RACER_CONTROL_HZ:-40}"
RVIZ="${RACER_RVIZ:-false}"

ARGS=(
  "localizer:=${LOCALIZER}"
  "mode:=${MODE}"
  "bootstrap:=true"
  "bootstrap_mode:=${BOOTSTRAP_MODE}"
  "control_hz:=${CONTROL_HZ}"
  "rviz:=${RVIZ}"
)
[ -n "${RACER_PATH_CSV:-}" ] && ARGS+=("path_csv:=${RACER_PATH_CSV}")
# Anything else, word-split on purpose: RACER_EXTRA_ARGS="v_max:=7.5 lookahead_k:=0.6"
# shellcheck disable=SC2206
[ -n "${RACER_EXTRA_ARGS:-}" ] && ARGS+=(${RACER_EXTRA_ARGS})

echo "[start_racer] ros2 launch racer_bringup race.launch.py ${ARGS[*]}"
exec ros2 launch racer_bringup race.launch.py "${ARGS[@]}"
