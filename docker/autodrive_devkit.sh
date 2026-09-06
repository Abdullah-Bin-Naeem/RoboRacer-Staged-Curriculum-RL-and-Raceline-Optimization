#!/usr/bin/env bash
# THE SUBMISSION ENTRYPOINT. Installed at /home/autodrive_devkit.sh, replacing
# the stub the base image ships, and named that because the rules name it.
#
# The requirement it exists to meet, from the 2026 technical guide:
#
#   "all the necessary nodes should start up ... Once we hit the Connection
#    Button ... the simulated vehicle should start running."
#
# So this script brings up the devkit bridge AND our stack, unattended. The
# bridge is not started separately: race.launch.py includes it (with the
# ground-truth TF remapped off /tf, or roboracer_1 gets two parents and the TF
# tree breaks), so there is exactly one bridge and one place that owns it.
#
# The bridge listens on 4567 and blocks until the simulator connects, so
# starting before the operator hits Connect is correct -- everything is up and
# waiting, and the car moves the moment the socket opens.
#
# WHY THE STACK RUNS IN THE BACKGROUND
# ------------------------------------
# The guide also requires that extra bash sessions stay available for the
# organizers "without re-executing the codebase". Two consequences, both
# deliberate:
#
#   - the stack is backgrounded here and the container's CMD (bash) runs in the
#     foreground, so `docker run -it` lands on a usable prompt with the car
#     already driving;
#   - `docker exec` does not run an entrypoint at all, so those shells get the
#     environment from ~/.bashrc (which sources /home/racer_env.sh and launches
#     nothing) and never start a second stack.
#
# Nothing is automated from ~/.bashrc, per the rules.
#
# IF YOU STARTED THE CONTAINER WITH --entrypoint /bin/bash
# --------------------------------------------------------
# Then this file never ran. Start the stack by hand:  /home/start_racer.sh
set -uo pipefail

source /home/racer_env.sh

LOG_DIR=/home/racer_ws/log
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/racer_$(date +%Y%m%d_%H%M%S).log"

if [ "${RACER_AUTOSTART:-1}" = "1" ]; then
    echo "[entrypoint] starting the race stack; log: $LOG"
    /home/start_racer.sh >"$LOG" 2>&1 &
    RACER_PID=$!
    echo "[entrypoint] race stack pid $RACER_PID -- follow it with: tail -f $LOG"
    echo "[entrypoint] waiting for the simulator to connect on port 4567"
else
    RACER_PID=""
    echo "[entrypoint] RACER_AUTOSTART=0 -- not starting the stack."
    echo "[entrypoint] start it by hand with: /home/start_racer.sh"
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
