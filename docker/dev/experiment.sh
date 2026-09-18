#!/usr/bin/env bash
# One experiment against the persistent bridge. Environment from run_experiment.sh:
#   EXP_DIR LINE LAPS MAX_CONTACTS TRACK OVERRIDES HOST_UID HOST_GID
set -o pipefail
REBUILD=1 source /root/Documents/roboracer/docker/dev/common.sh
mkdir -p "$EXP_DIR"
python3 tools/tuning/sim_ctl.py reset || { echo "EXPERIMENT: reset did not zero the counters"; }
ros2 launch racer_bringup race.launch.py bridge:=false rviz:=${RVIZ:-false} track:=$TRACK \
    path_csv:=$PWD/raceline/$TRACK/$LINE log_csv:=$EXP_DIR/run.csv $OVERRIDES \
    > "$EXP_DIR/launch.log" 2>&1 &
LAUNCH=$!
cleanup() {
    kill -INT $LAUNCH 2>/dev/null
    for _ in $(seq 20); do kill -0 $LAUNCH 2>/dev/null || break; sleep 0.5; done
    kill -KILL $LAUNCH 2>/dev/null
    pkill -INT -f "racer_(control|localization)" 2>/dev/null; sleep 1
    python3 tools/tuning/sim_ctl.py stop
    fix_owner "$EXP_DIR"
}
trap cleanup EXIT
python3 tools/tuning/sim_ctl.py watch "$LAPS" "$MAX_CONTACTS" "$EXP_DIR/laps.csv" --stall 60 --timeout $((LAPS * 20 + 300))
CODE=$?
case $CODE in
    0) echo "EXPERIMENT: done, $LAPS timed laps" ;;
    3) echo "EXPERIMENT: stopped at $MAX_CONTACTS contact(s)" ;;
    4) echo "EXPERIMENT: stalled" ;;
    *) echo "EXPERIMENT: ended with code $CODE" ;;
esac
echo $CODE > "$EXP_DIR/exit_code"
