#!/usr/bin/env bash
# Live sensor-tick and control-loop rate of a RUNNING racer container.
#
#     ./scripts/hz.sh                 report once a second until Ctrl-C
#     ./scripts/hz.sh --lidar         also subscribe to the 1080-beam scan
#     ./scripts/hz.sh --csv runs/rate_run62.csv
#
# Everything after the container name is passed straight to rate_monitor.py, so
# --period, --window and --warn work too.
#
# This copies the tool INTO the running container rather than requiring it in
# the image. That is the point: the qualified image needs no rebuild, and the
# same tool works against the exported tar on a machine that has never seen this
# repository. The DDS settings are image ENV (Dockerfile), so a `docker exec`
# shell already discovers the running nodes -- only the ROS base needs sourcing,
# not our overlay, because rate_monitor.py imports nothing from roboracer_stack.
#
# ON THE WINDOWS BOX there is no bash to run this from. Two lines, same effect:
#
#     docker cp rate_monitor.py autodrive_roboracer_api:/tmp/
#     docker exec -it autodrive_roboracer_api bash -lc "source /opt/ros/humble/setup.bash && python3 /tmp/rate_monitor.py --csv /tmp/rate.csv"
#
# and afterwards, to get the log back out:
#
#     docker cp autodrive_roboracer_api:/tmp/rate.csv .
set -euo pipefail

NAME="${NAME:-autodrive_roboracer_api}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOOL="$REPO/roboracer_stack/tools/rate_monitor.py"

if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "$0: container '$NAME' is not running -- start it with ./scripts/run.sh racer" >&2
  echo "$0: (override with NAME=<container> $0 ...)" >&2
  exit 1
fi

docker cp "$TOOL" "$NAME:/tmp/rate_monitor.py"
# -it so Ctrl-C reaches the tool and it prints its closing summary.
exec docker exec -it "$NAME" bash -lc \
  "source /opt/ros/humble/setup.bash && exec python3 /tmp/rate_monitor.py $*"
