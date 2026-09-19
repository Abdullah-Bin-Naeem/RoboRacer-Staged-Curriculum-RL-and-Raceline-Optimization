#!/usr/bin/env bash
# One development run with full recording, then park the car at the spawn.
#
#     tools/diag_run.sh NAME SECONDS ["extra launch args"]
#
# Needs the simulator up (headless auto-connects). Writes logs/diag/NAME.csv
# (tools/run_recorder.py) and logs/diag/NAME.log (the stack log).
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

RUN="$1"; DUR="$2"; EXTRA="${3:-}"
IMAGE="${IMAGE:-autodrive_racer:diag}"
NAME=autodrive_roboracer_api
OUT=logs/diag; mkdir -p "$OUT"
say() { echo "[diag $(date +%H:%M:%S)] $*"; }
attached() { ss -tn 2>/dev/null | grep ':4567 ' | grep -q ESTAB; }

park() {
  docker rm -f "$NAME" >/dev/null 2>&1
  sleep 2
  docker run -dt --rm --name "$NAME" --network=host --ipc=host -e RACER_AUTOSTART=0 "$IMAGE" >/dev/null
  docker exec -d "$NAME" bash -c 'source /home/autodrive_devkit/install/setup.bash; ros2 launch roboracer_stack bridge.launch.py >/dev/null 2>&1'
  for _ in $(seq 1 60); do attached && break; sleep 1; done
  sleep 3
  docker exec "$NAME" bash -c '
    source /home/autodrive_devkit/install/setup.bash
    ros2 topic pub -1 /autodrive/roboracer_1/throttle_command std_msgs/msg/Float32 "{data: 0.0}" >/dev/null 2>&1
    ros2 topic pub -1 /autodrive/roboracer_1/steering_command std_msgs/msg/Float32 "{data: 0.0}" >/dev/null 2>&1
    sleep 1
    ros2 topic pub -1 /autodrive/reset_command std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1
    sleep 0.5
    ros2 topic pub -1 /autodrive/reset_command std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1
    sleep 2'
  docker rm -f "$NAME" >/dev/null 2>&1
  sleep 2
}

[ "${SKIP_PARK:-0}" = "1" ] || { say "parking"; park; }

say "run $RUN: ${DUR}s, extra: $EXTRA"
docker run -dt --rm --name "$NAME" --network=host --ipc=host \
  -v "$PWD/$OUT:/home/autodrive_devkit/log" \
  -e RACER_MODE=dev -e RACER_EXTRA_ARGS="$EXTRA" "$IMAGE" >/dev/null || { say "docker run failed"; exit 1; }
for _ in $(seq 1 90); do attached && break; sleep 1; done
attached && say "simulator attached" || say "WARNING: simulator not attached"
LOG=""
for _ in $(seq 1 20); do LOG=$(docker exec "$NAME" bash -c 'ls -t /home/autodrive_devkit/log/racer_*.log 2>/dev/null | head -1'); [ -n "$LOG" ] && break; sleep 1; done
docker cp tools/run_recorder.py "$NAME:/tmp/run_recorder.py"
docker exec "$NAME" bash -c "source /home/autodrive_devkit/install/setup.bash; timeout $((DUR + 30)) python3 /tmp/run_recorder.py /tmp/$RUN.csv --seconds $DUR" \
  > "$OUT/$RUN.recorder.txt" 2>&1
docker exec "$NAME" bash -c 'source /home/autodrive_devkit/install/setup.bash
  echo "--- nodes ---"; ros2 node list 2>&1
  echo "--- pure_pursuit status rate ---"; timeout 5 ros2 topic hz /pure_pursuit/status 2>&1 | head -3
  echo "--- commands ---"; timeout 5 ros2 topic hz /autodrive/roboracer_1/throttle_command 2>&1 | head -3
  echo "--- ps ---"; ps -eo pid,stat,etime,cmd | grep -E "pure_pursuit|dead_reck|amcl" | grep -v grep' >> "$OUT/$RUN.recorder.txt" 2>&1
docker cp "$NAME:/tmp/$RUN.csv" "$OUT/$RUN.csv"
[ -n "$LOG" ] && docker cp "$NAME:$LOG" "$OUT/$RUN.log"
say "recorded $OUT/$RUN.csv ($(wc -l < "$OUT/$RUN.csv") rows)"
[ "${SKIP_PARK:-0}" = "1" ] || { say "parking"; park; }
say "done"
