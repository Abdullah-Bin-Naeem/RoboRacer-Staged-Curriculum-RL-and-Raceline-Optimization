#!/usr/bin/env bash
# Run the racer container at several loop caps, one after another, one CSV per cap.
#
#     ./tools/cap_sweep.sh                       # CAPS="20 40 45 50 60 70 80", 180 s each
#     CAPS="20 45" DUR=120 ./tools/cap_sweep.sh  # your own list and driving time
#
# Needs the simulator container up and connected (./scripts/run.sh sim, press
# Connect once; it reconnects to every new bridge by itself). Everything lands
# in logs/: racer_<stamp>.log (the stack log, lap times), run_cap<hz>.csv (the
# per-tick run log from log_csv:=) and cap_sweep.txt (which log is which cap).
#
# Between runs the car is PARKED: the follower is killed, zero throttle and
# steering are published, and /autodrive/reset_command is pulsed true->false
# (level-triggered: the bridge re-emits the last value every tick, so it must be
# pulsed or the sim resets forever). The next container then seeds AMCL from
# the spawn constant with the car actually on it. Without this the car keeps
# its last command when the bridge dies and the next run starts localized
# wrong. The same parking runs first if a racer container is already up.
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

CAPS="${CAPS:-20 40 45 50 60 70 80}"
DUR="${DUR:-180}"                       # seconds of driving per cap, after the follower takes over
IMAGE="${IMAGE:-autodrive_racer}:${TAG:-qualification-1}"
LINE="${LINE:-/home/autodrive_devkit/install/roboracer_stack/share/roboracer_stack/raceline/raceline_a7.0_rec_6.36.csv}"
NAME=autodrive_roboracer_api
mkdir -p logs

say() { echo "[sweep $(date +%H:%M:%S)] $*"; }
sim_attached() { ss -tn 2>/dev/null | grep ':4567 ' | grep -q ESTAB; }
# The Unity client reconnects by itself across the few seconds of a container
# swap, but after a long gap with no bridge it stops retrying and only the
# Connect button brings it back. So: wait, and say so, rather than time out.
wait_attached() {
  local n=0
  until sim_attached; do
    n=$((n+1)); [ $((n % 15)) -eq 0 ] && say "waiting for the simulator: press Connect in its window (127.0.0.1:4567)"
    sleep 1
  done
}

park() {
  # Needs a bridge to talk through: the running stack's, or a temporary one.
  if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
    say "no racer container: starting a bridge-only one to park the car"
    docker run -dt --rm --name "$NAME" --network=host --ipc=host -e RACER_AUTOSTART=0 "$IMAGE" >/dev/null
    docker exec -d "$NAME" bash -c 'source /home/autodrive_devkit/install/setup.bash; ros2 launch roboracer_stack bridge.launch.py >/dev/null 2>&1'
    wait_attached
  fi
  say "parking the car at the spawn"
  docker exec "$NAME" bash -c '
    source /home/autodrive_devkit/install/setup.bash
    pkill -f roboracer_stack/pure_pursuit 2>/dev/null; sleep 1
    ros2 topic pub -1 /autodrive/roboracer_1/throttle_command std_msgs/msg/Float32 "{data: 0.0}" >/dev/null 2>&1
    ros2 topic pub -1 /autodrive/roboracer_1/steering_command std_msgs/msg/Float32 "{data: 0.0}" >/dev/null 2>&1
    sleep 2
    ros2 topic pub -1 /autodrive/reset_command std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1
    sleep 0.5
    ros2 topic pub -1 /autodrive/reset_command std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1
    sleep 2'
  docker rm -f "$NAME" >/dev/null 2>&1
  sleep 2
}

docker ps --format '{{.Names}}' | grep -q autodrive_roboracer_sim || { say "simulator container is not running: ./scripts/run.sh sim in another terminal, press Connect"; exit 1; }
park

for HZ in $CAPS; do
  CSV="run_cap${HZ}.csv"
  say "=== cap ${HZ} Hz: ${DUR} s of driving -> logs/${CSV} ==="
  before=$(ls -t logs/racer_*.log 2>/dev/null | head -1)
  docker run -d --rm --name "$NAME" --network=host --ipc=host \
    -v "$PWD/tools/libnodelay.so:/home/libnodelay.so:ro" \
    -v "$PWD/logs:/home/autodrive_devkit/log" \
    -e LD_PRELOAD=/home/libnodelay.so -e NODELAY_CAP_HZ="$HZ" \
    -e RACER_MODE=dev -e RACER_PATH_CSV="$LINE" \
    -e RACER_EXTRA_ARGS="log_csv:=/home/autodrive_devkit/log/${CSV}" \
    "$IMAGE" >/dev/null || { say "docker run failed"; exit 1; }
  wait_attached
  # the stack log for THIS run: the newest one that is not the previous newest
  LOG=""
  for _ in $(seq 1 30); do LOG=$(ls -t logs/racer_*.log 2>/dev/null | head -1); [ -n "$LOG" ] && [ "$LOG" != "$before" ] && break; sleep 1; done
  for _ in $(seq 1 90); do grep -q "taking over" "$LOG" 2>/dev/null && break; sleep 1; done
  grep -q "taking over" "$LOG" 2>/dev/null && say "follower took over; driving" || say "WARNING: follower never took over (see $LOG)"
  sleep "$DUR"
  laps=$(grep -o "lap [0-9]*: [0-9.]* s" "$LOG" 2>/dev/null | wc -l); best=$(grep -o "lap [0-9]*: [0-9.]* s" "$LOG" | awk 'NR>1{print $3}' | sort -n | head -1)
  say "cap ${HZ}: ${laps:-0} laps, best ${best:-?} s"
  echo "${HZ} ${LOG} logs/${CSV} laps=${laps:-0} best=${best:-?}" >> logs/cap_sweep.txt
  park
done
say "done. index: logs/cap_sweep.txt"
