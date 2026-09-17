#!/usr/bin/env bash
# Record localization drift from the running racer container, then plot it.
#
#     tools/localization_drift.sh [SECONDS] [NAME]
#
#     SECONDS  how long to record; 0 (default) records until Ctrl+C
#     NAME     output name (default drift_<date>_<time>)
#
# Start the stack first (./scripts/run.sh racer) and the simulator. Writes
# logs/drift/NAME.csv and the figures in logs/drift/NAME_drift/.
#
# The recorder runs inside the container, which has ROS; the plot runs on the
# host and needs numpy and matplotlib. It uses .venv-plot/ when that exists
#     python3 -m venv .venv-plot && .venv-plot/bin/pip install numpy matplotlib
# and PYTHON=path/to/bin/python picks another interpreter.
# See tools/localization_drift.py for what is measured.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${CONTAINER:-autodrive_roboracer_api}"
PYTHON="${PYTHON:-python3}"
[ -x "$REPO/.venv-plot/bin/python" ] && [ "$PYTHON" = python3 ] && PYTHON="$REPO/.venv-plot/bin/python"
SECONDS_ARG="${1:-0}"
NAME="${2:-drift_$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="$REPO/logs/drift"
mkdir -p "$OUT_DIR"

docker cp "$REPO/tools/localization_drift.py" "$CONTAINER:/tmp/localization_drift.py"
# The recorder is a DDS participant like any node: the image's CYCLONEDDS_URI
# (inherited by docker exec) lifts the 9-participant ceiling it would hit.
TTY=(-i); [ -t 0 ] && TTY=(-it)
# Both image layouts: the stack built into the devkit workspace (competition
# branches) or in its own overlay under /root/Documents/roboracer (multi-track).
docker exec "${TTY[@]}" "$CONTAINER" bash -c "
  source /home/autodrive_devkit/install/setup.bash
  overlay=/root/Documents/roboracer/devkit_ws/install/setup.bash
  [ -f \$overlay ] && source \$overlay
  python3 /tmp/localization_drift.py record /tmp/$NAME.csv --seconds $SECONDS_ARG" || true
docker cp "$CONTAINER:/tmp/$NAME.csv" "$OUT_DIR/$NAME.csv"

if "$PYTHON" -c 'import numpy, matplotlib' 2>/dev/null; then
  "$PYTHON" "$REPO/tools/localization_drift.py" plot "$OUT_DIR/$NAME.csv"
else
  echo "recorded $OUT_DIR/$NAME.csv; $PYTHON has no numpy/matplotlib, so plot it with:"
  echo "  <python with numpy+matplotlib> tools/localization_drift.py plot $OUT_DIR/$NAME.csv"
fi
