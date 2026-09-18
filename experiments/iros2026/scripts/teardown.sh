#!/bin/bash
# Stop the race stack and PROVE it is stopped.
#
# pkill/pgrep patterns have silently failed here and left two complete stacks --
# two pure_pursuit nodes commanding the car at once -- while the follow-up grep
# printed nothing and looked like success. Restarting the container is the only
# teardown that cannot half-work, so that is what this does. It leaves the
# bridge and autodrive_bridge up; the sim link then needs a MANUAL reset +
# connect (never publish to /autodrive/reset_command).
set -u
echo "restarting rr_bridge ..."
docker restart rr_bridge >/dev/null
sleep 12

left=$(docker exec rr_bridge ps -eo cmd |
       grep -cE 'pure_pursuit|nav2_amcl|map_server|racer_localization|race\.launch' || true)
echo
docker exec rr_bridge ps -eo pid,cmd | grep -v 'ps -eo' | grep -v grep
echo
if [ "$left" -eq 0 ]; then
    echo "CLEAN: nothing is commanding the car. Ask Adil to reset + connect."
    exit 0
fi
echo "STILL ALIVE: $left node(s) above -- do NOT start a run."
exit 1
